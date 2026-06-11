"""
Problem instance for VRP-RPD on a grid topology.

Node 0 is the depot. Nodes 1..n are customers (machines).
The grid is row-major: for an 8x8 grid, position (row, col) maps to
node index row*8 + col, with (0,0) being the depot by default.

Distances are Manhattan distance on the grid, scaled by cell_traversal_time,
unless node_xy_positions is set — in that case a full pairwise matrix is
built from the measured physical coordinates via Dijkstra shortest paths.
"""
from __future__ import annotations

import csv
import heapq
import json
import math
import random
import zipfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional


@dataclass
class Instance:
    """A VRP-RPD problem instance on a grid.

    Node numbering matches the physical grid image: node 0 = depot (bottom-left),
    nodes 1-7 = bottom row left-to-right, node 8 = one row up on the left, etc.
    node 63 = top-right corner.

    Internal (row, col) uses row 0 = top of grid for canvas rendering.
    Physical (x, y) has origin at bottom-left, y increasing upward.
    node_to_rc() and rc_to_node() convert between node ids and internal coords.

    When node_xy_positions is provided (64 [x, y] pairs indexed by node id,
    node 0 first), distance() and edge_time() use real measured coordinates
    instead of the uniform Manhattan formula.
    """
    grid_rows: int
    grid_cols: int
    depot_row: int
    depot_col: int
    num_agvs: int
    capacity: int
    # One processing time per customer (indexed 1..n). Index 0 is the depot (unused).
    processing_times: list[float]
    # Seconds to traverse one grid cell (straight).
    cell_traversal_time: float
    # Seconds added per turn at an intersection (optional fidelity).
    turn_penalty: float = 0.0
    # Seconds per AGV in the staging queue (staggered launch time).
    # AGV k leaves the depot at t = k * load_time.
    load_time: float = 0.0
    # Demand: list of customer node indices that must be served.
    # If None, defaults to all non-depot nodes up to num_agvs * capacity.
    demand: list[int] = field(default_factory=list)
    seed: Optional[int] = None
    # Physical (x, y) positions for each node, relative to the depot.
    # 64-element flat list where entry i = [x, y] for node i (code array order).
    # When set, distance() and edge_time() use measured coordinates.
    node_xy_positions: Optional[list[list[float]]] = field(default=None)

    def __post_init__(self):
        self._precomputed_dist: Optional[list[list[float]]] = None
        self._edge_times: Optional[dict] = None
        if self.node_xy_positions is not None:
            self._build_dist_from_positions()

    @property
    def num_nodes(self) -> int:
        return self.grid_rows * self.grid_cols

    @property
    def num_customers(self) -> int:
        return len(self.demand)

    @property
    def depot_node(self) -> int:
        return self.rc_to_node(self.depot_row, self.depot_col)

    def node_to_rc(self, node: int) -> tuple[int, int]:
        """Internal (row, col) where row 0 = top. Node 0 = bottom-left = depot."""
        physical_row = node // self.grid_cols
        return self.grid_rows - 1 - physical_row, node % self.grid_cols

    def rc_to_node(self, row: int, col: int) -> int:
        """Node id from internal (row, col). Row 0 = top, row grid_rows-1 = bottom."""
        return (self.grid_rows - 1 - row) * self.grid_cols + col

    def node_to_xy(self, node: int) -> tuple[int, int]:
        """Physical (x, y) with origin at bottom-left, y increasing upward."""
        r, c = self.node_to_rc(node)
        return c, (self.grid_rows - 1) - r

    def xy_to_node(self, x: int, y: int) -> int:
        """Inverse of node_to_xy."""
        return self.rc_to_node((self.grid_rows - 1) - y, x)

    def distance(self, i: int, j: int) -> float:
        """Travel time in seconds between two nodes."""
        if self._precomputed_dist is not None:
            return self._precomputed_dist[i][j]
        ri, ci = self.node_to_rc(i)
        rj, cj = self.node_to_rc(j)
        cells = abs(ri - rj) + abs(ci - cj)
        turns = 1 if (ri != rj and ci != cj) else 0
        return cells * self.cell_traversal_time + turns * self.turn_penalty

    def edge_time(self, from_rc: tuple[int, int], to_rc: tuple[int, int]) -> float:
        """Per-segment traversal time (seconds) for adjacent cells.
        Falls back to cell_traversal_time when no position data is loaded."""
        if self._edge_times is not None:
            w = self._edge_times.get((from_rc, to_rc))
            if w is not None:
                return w
        return self.cell_traversal_time

    def distance_matrix(self) -> list[list[float]]:
        """Full n x n matrix of travel times."""
        n = self.num_nodes
        return [[self.distance(i, j) for j in range(n)] for i in range(n)]

    # -------- Physical position loading --------

    def set_node_positions(self, positions: list[list[float]]) -> None:
        """Set physical [x, y] positions for all nodes and rebuild distance tables.

        positions: flat list of 64 [x, y] pairs indexed by node id
        (node 0 = bottom-left / depot, node 63 = top-right of the 8x8 grid).
        """
        self.node_xy_positions = positions
        self._build_dist_from_positions()

    def _build_dist_from_positions(self) -> None:
        """Build precomputed pairwise distance matrix and per-edge times from
        physical coordinates using Dijkstra on the grid graph.

        Edge lengths are Euclidean distances between adjacent cell centers.
        Times are scaled so that the average edge maps to cell_traversal_time.
        """
        positions = self.node_xy_positions
        rows, cols = self.grid_rows, self.grid_cols
        n = rows * cols

        # Build edge physical lengths (symmetric, keyed by (from_rc, to_rc))
        def _rc_to_ni(r: int, c: int) -> int:
            return (rows - 1 - r) * cols + c

        edge_len: dict[tuple, float] = {}
        for r in range(rows):
            for c in range(cols):
                ni = _rc_to_ni(r, c)
                xi, yi = positions[ni][0], positions[ni][1]
                if c + 1 < cols:
                    nj = _rc_to_ni(r, c + 1)
                    xj, yj = positions[nj][0], positions[nj][1]
                    length = math.sqrt((xj - xi) ** 2 + (yj - yi) ** 2)
                    edge_len[((r, c), (r, c + 1))] = length
                    edge_len[((r, c + 1), (r, c))] = length
                if r + 1 < rows:
                    nj = _rc_to_ni(r + 1, c)
                    xj, yj = positions[nj][0], positions[nj][1]
                    length = math.sqrt((xj - xi) ** 2 + (yj - yi) ** 2)
                    edge_len[((r, c), (r + 1, c))] = length
                    edge_len[((r + 1, c), (r, c))] = length

        # Scale to seconds: avg edge → cell_traversal_time
        if edge_len:
            avg_len = sum(edge_len.values()) / len(edge_len)
        else:
            avg_len = 1.0
        scale = self.cell_traversal_time / avg_len if avg_len > 0 else 1.0
        self._edge_times = {k: v * scale for k, v in edge_len.items()}

        # All-pairs shortest paths via Dijkstra from each source
        dist = [[math.inf] * n for _ in range(n)]
        for src in range(n):
            dist[src][src] = 0.0
            src_r = rows - 1 - (src // cols)
            src_c = src % cols
            pq: list[tuple[float, tuple[int, int]]] = [(0.0, (src_r, src_c))]
            visited: set[tuple[int, int]] = set()
            while pq:
                d, rc = heapq.heappop(pq)
                if rc in visited:
                    continue
                visited.add(rc)
                node_idx = _rc_to_ni(rc[0], rc[1])
                if d > dist[src][node_idx]:
                    continue
                r, c = rc
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        nnode = _rc_to_ni(nr, nc)
                        w = self._edge_times.get(((r, c), (nr, nc)), math.inf)
                        nd = d + w
                        if nd < dist[src][nnode]:
                            dist[src][nnode] = nd
                            heapq.heappush(pq, (nd, (nr, nc)))
        self._precomputed_dist = dist

    # -------- Factory methods --------

    @classmethod
    def random_instance(
        cls,
        grid_rows: int = 8,
        grid_cols: int = 8,
        num_agvs: int = 3,
        capacity: int = 4,
        cell_traversal_time: float = 2.0,
        turn_penalty: float = 0.5,
        load_time: float = 0.0,
        processing_time_range: tuple[float, float] = (5.0, 30.0),
        processing_time_multiplier: float = 1.0,
        depot_row: int = 7,  # physical (0,0) = bottom-left in an 8x8 grid
        depot_col: int = 0,
        seed: Optional[int] = None,
    ) -> "Instance":
        """
        Build a random instance. Demand = num_agvs * capacity customer nodes,
        chosen uniformly from the non-depot nodes.
        """
        rng = random.Random(seed)
        n = grid_rows * grid_cols
        depot = (grid_rows - 1 - depot_row) * grid_cols + depot_col

        total_demand = num_agvs * capacity
        non_depot = [i for i in range(n) if i != depot]
        if total_demand > len(non_depot):
            raise ValueError(
                f"Demand {total_demand} exceeds available non-depot nodes {len(non_depot)}"
            )
        demand = sorted(rng.sample(non_depot, total_demand))

        # Processing times: only populate for nodes in demand; others are 0.
        pmin, pmax = processing_time_range
        proc = [0.0] * n
        for c in demand:
            base = rng.uniform(pmin, pmax)
            proc[c] = base * processing_time_multiplier

        return cls(
            grid_rows=grid_rows,
            grid_cols=grid_cols,
            depot_row=depot_row,
            depot_col=depot_col,
            num_agvs=num_agvs,
            capacity=capacity,
            processing_times=proc,
            cell_traversal_time=cell_traversal_time,
            turn_penalty=turn_penalty,
            load_time=load_time,
            demand=demand,
            seed=seed,
        )

    # -------- Serialization --------

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Instance":
        return cls(**d)

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path: str | Path) -> "Instance":
        return cls.from_dict(json.loads(Path(path).read_text()))

    # -------- Debug --------

    def summary(self) -> str:
        pos_info = " [real coords loaded]" if self.node_xy_positions is not None else ""
        return (
            f"Instance: {self.grid_rows}x{self.grid_cols} grid, "
            f"depot=node {self.depot_node} at ({self.depot_row},{self.depot_col}), "
            f"{self.num_agvs} AGVs x capacity {self.capacity} = "
            f"{self.num_agvs * self.capacity} total capacity, "
            f"{self.num_customers} customers, "
            f"cell_time={self.cell_traversal_time}s, turn_penalty={self.turn_penalty}s"
            f"{pos_info}"
        )


# ────────────────────────────────────────────────────────────────────────────
# Distance matrix I/O  (xlsx + csv)
# ────────────────────────────────────────────────────────────────────────────

def _parse_coord_cell(value: str) -> Optional[tuple[float, float]]:
    """Parse a cell containing 'y_val, x_val' or a bare float.
    Returns (x, y) tuple or None if unparseable.
    The Excel stores coordinates as 'y, x' per the physical layout convention."""
    s = value.strip().strip('"').strip("'")
    if "," in s:
        parts = s.split(",", 1)
        try:
            y_val = float(parts[0].strip())
            x_val = float(parts[1].strip())
            return (x_val, y_val)
        except ValueError:
            return None
    else:
        try:
            # Single number: only the y-value was stored (data entry shortcut).
            # Caller must fill in x from the column pattern.
            return (None, float(s))  # type: ignore[return-value]
        except ValueError:
            return None


def load_positions_xlsx(
    path: str | Path,
    grid_rows: int = 8,
    grid_cols: int = 8,
    depot_row_first: bool = True,
) -> list[list[float]]:
    """Parse physical (x, y) positions from the project's Excel file.

    The Excel format (as measured and stored):
      - Each data cell contains the string "y_coord, x_coord" in physical units.
      - User's Row 0 is the depot row (y = 0); rows increase in the +Y direction.
      - Columns increase in the +X direction.

    depot_row_first=True means the first data row in the file is the depot row
    (user convention). The function flips the row order to match the code's
    internal array convention where row 0 is the TOP of the grid.

    Returns a flat list of 64 [x, y] pairs in code array order
    (node 0 = array top-left).
    """
    path = Path(path)
    with zipfile.ZipFile(path) as z:
        sheet_xml = z.read("xl/worksheets/sheet1.xml").decode("utf-8")
        try:
            strings_xml = z.read("xl/sharedStrings.xml").decode("utf-8")
            ns = {"s": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
            root_s = ET.fromstring(strings_xml)
            shared = []
            for si in root_s.findall("s:si", ns):
                t_el = si.find(".//s:t", ns)
                shared.append(t_el.text if t_el is not None else "")
        except KeyError:
            shared = []

    ns = {"s": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
    root = ET.fromstring(sheet_xml)

    # Read all cells into a sparse dict: (excel_row, excel_col) -> str value
    raw: dict[tuple[int, int], str] = {}
    for row_el in root.findall(".//s:row", ns):
        r_idx = int(row_el.get("r", 0))
        for cell_el in row_el.findall("s:c", ns):
            ref = cell_el.get("r", "")
            col_letters = "".join(ch for ch in ref if ch.isalpha())
            col_idx = 0
            for ch in col_letters:
                col_idx = col_idx * 26 + (ord(ch.upper()) - ord("A") + 1)
            t = cell_el.get("t", "n")
            v_el = cell_el.find("s:v", ns)
            if v_el is None or v_el.text is None:
                continue
            if t == "s":
                raw[(r_idx, col_idx)] = shared[int(v_el.text)]
            else:
                raw[(r_idx, col_idx)] = v_el.text

    # Find the data block: use only cells with BOTH x and y (comma-separated)
    # to determine the bounding box. Header cells like "0","1",...,"7" are
    # single numbers and must not shift the boundary.
    coord_cells: dict[tuple[int, int], str] = {}  # cells with "y, x" format
    all_parseable: dict[tuple[int, int], str] = {}  # all numeric/coord cells
    for (r, c), v in raw.items():
        parsed = _parse_coord_cell(v)
        if parsed is None:
            continue
        all_parseable[(r, c)] = v
        if parsed[0] is not None:  # has both x and y
            coord_cells[(r, c)] = v

    if not coord_cells:
        raise ValueError(f"No 'y, x' coordinate data found in {path}")

    min_r = min(r for r, _ in coord_cells)
    max_r = max(r for r, _ in coord_cells)
    min_c = min(c for _, c in coord_cells)
    max_c = max(c for _, c in coord_cells)
    # Merge in any single-value cells that fall inside the bounding box
    data_cells: dict[tuple[int, int], str] = {
        (r, c): v for (r, c), v in all_parseable.items()
        if min_r <= r <= max_r and min_c <= c <= max_c
    }

    actual_rows = max_r - min_r + 1
    actual_cols = max_c - min_c + 1
    if actual_rows != grid_rows or actual_cols != grid_cols:
        raise ValueError(
            f"Expected {grid_rows}x{grid_cols} data block, "
            f"found {actual_rows}x{actual_cols} in {path}"
        )

    # First pass: collect established x-value per column and y-value per row
    x_by_col: dict[int, float] = {}
    y_by_row: dict[int, float] = {}
    for (r, c), v in data_cells.items():
        parsed = _parse_coord_cell(v)
        if parsed is not None and parsed[0] is not None:
            x_by_col.setdefault(c, parsed[0])
        if parsed is not None and parsed[1] is not None:
            y_by_row.setdefault(r, parsed[1])

    # Second pass: build grid (user row/col indices relative to data block)
    grid: list[list[list[float]]] = [
        [[0.0, 0.0] for _ in range(grid_cols)] for _ in range(grid_rows)
    ]
    warnings_issued: list[str] = []
    for user_r in range(grid_rows):
        for user_c in range(grid_cols):
            excel_r = min_r + user_r
            excel_c = min_c + user_c
            v = data_cells.get((excel_r, excel_c), "")
            parsed = _parse_coord_cell(v)
            x_val = parsed[0] if parsed is not None else None
            y_val = parsed[1] if parsed is not None else None
            # Fill missing x from column pattern
            if x_val is None:
                x_val = x_by_col.get(excel_c)
                if x_val is None:
                    warnings_issued.append(
                        f"Cannot determine x for user row {user_r} col {user_c}; using 0"
                    )
                    x_val = 0.0
            # Fill missing y from row pattern
            if y_val is None:
                y_val = y_by_row.get(excel_r)
                if y_val is None:
                    warnings_issued.append(
                        f"Cannot determine y for user row {user_r} col {user_c}; using 0"
                    )
                    y_val = 0.0
            grid[user_r][user_c] = [float(x_val), float(y_val)]

    for w in warnings_issued:
        print(f"[dist] WARNING: {w}")

    return _grid_to_flat(grid, grid_rows, grid_cols, depot_row_first)


def load_positions_csv(
    path: str | Path,
    grid_rows: int = 8,
    grid_cols: int = 8,
    depot_row_first: bool = True,
) -> list[list[float]]:
    """Parse physical (x, y) positions from a CSV file.

    Expected format: 8x8 CSV (headers optional — non-numeric rows/cells are
    skipped automatically). Each data cell should contain "y_val, x_val"
    matching the Excel convention, or a single float (treated as y; x inferred
    from column pattern).

    depot_row_first=True (default): first data row = depot row (y = 0),
    which the function flips to code array order (depot = last row).

    Returns a flat list of 64 [x, y] pairs in code array order.
    """
    path = Path(path)
    raw_rows: list[list[str]] = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            raw_rows.append(row)

    # Extract data rows: rows with >= grid_cols parseable cells
    data_rows: list[list[str]] = []
    for row in raw_rows:
        data_cells = [c for c in row if _parse_coord_cell(c) is not None]
        if len(data_cells) >= grid_cols:
            data_rows.append(data_cells[:grid_cols])

    if len(data_rows) != grid_rows:
        raise ValueError(
            f"Expected {grid_rows} data rows in CSV, found {len(data_rows)} in {path}"
        )

    # Build grid using same inference logic as xlsx loader
    x_by_col: dict[int, float] = {}
    y_by_row: dict[int, float] = {}
    for r_idx, row in enumerate(data_rows):
        for c_idx, cell in enumerate(row):
            parsed = _parse_coord_cell(cell)
            if parsed is not None and parsed[0] is not None:
                x_by_col.setdefault(c_idx, parsed[0])
            if parsed is not None and parsed[1] is not None:
                y_by_row.setdefault(r_idx, parsed[1])

    grid: list[list[list[float]]] = [
        [[0.0, 0.0] for _ in range(grid_cols)] for _ in range(grid_rows)
    ]
    for r_idx, row in enumerate(data_rows):
        for c_idx, cell in enumerate(row):
            parsed = _parse_coord_cell(cell)
            x_val = parsed[0] if parsed is not None else None
            y_val = parsed[1] if parsed is not None else None
            if x_val is None:
                x_val = x_by_col.get(c_idx, 0.0)
            if y_val is None:
                y_val = y_by_row.get(r_idx, 0.0)
            grid[r_idx][c_idx] = [float(x_val), float(y_val)]

    return _grid_to_flat(grid, grid_rows, grid_cols, depot_row_first)


def _grid_to_flat(
    grid: list[list[list[float]]],
    grid_rows: int,
    grid_cols: int,
    depot_row_first: bool,
) -> list[list[float]]:
    """Convert user-order 2D grid to flat list in code array order.

    depot_row_first=True: user grid row 0 is the depot (physical bottom).
    Node 0 = bottom-left, so no row reversal is needed — the flat list
    starts with the depot row, matching the new node numbering.
    """
    ordered = grid if depot_row_first else list(reversed(grid))
    positions: list[list[float]] = []
    for row in ordered:
        for xy in row:
            positions.append(xy)
    return positions
