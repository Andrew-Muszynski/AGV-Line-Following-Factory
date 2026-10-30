"""Where every node on the testbed is: the ONE copy.

    python3 testbed_coordinates.py            # print the table and its fingerprint

Read by camera_grid_navigate.py (which drives the robots there) and
apriltag_localize.py (which draws the node overlay and feeds the tools that
score robots against these positions). Until 2026-10-22 each carried its own
literal copy of the tables, kept equal by hand; they had not drifted, but
nothing stopped them from doing so. The values live in
testbed_coordinates.json beside this file.

Failing loudly is deliberate. A missing or malformed file raises at import
instead of falling back to built-in numbers, because a silent fallback on one
machine is exactly how the localiser and the navigator would end up
disagreeing about where a bay is. Both programs print COORDINATES_ID at
startup so a mismatch between machines shows in the logs.

Node labels, 8x8 grid (rows*cols = 64, bays = 49):
    "1".."64"      lattice nodes (node 1 = bottom-left, depot side)
    "65".."113"    workstation nodes, MEASURED (calibrate_workstations.py)
    "114".."162"   bay entry nodes, on the lattice row
    "D1".."D7"     depot slots, MEASURED
    "DE1".."DE7"   depot entries, MEASURED
    "0"            the depot-lane corner west of DE1
World frame: inches, x east and y north from the table corner nearest tag 20.
"""
from __future__ import annotations

import hashlib
import json
import os
import re

FILENAME = "testbed_coordinates.json"
_HERE = os.path.dirname(os.path.abspath(__file__))


def _find() -> str:
    """Beside this module (the flattened Linux checkout and fleet/ in the
    repo), or an explicit AGV_TESTBED_COORDINATES path."""
    env = os.environ.get("AGV_TESTBED_COORDINATES")
    candidates = [env] if env else []
    candidates.append(os.path.join(_HERE, FILENAME))
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    raise FileNotFoundError(
        f"{FILENAME} not found beside {__file__}. It holds every node "
        "position the navigator and the localiser use; deploy it with the "
        "fleet code (tools/deploy_linux.sh copies it).")


PATH = _find()
with open(PATH, "rb") as _fh:
    _RAW = _fh.read()
_BLOB = json.loads(_RAW.decode("utf-8"))
COORDINATES_ID = (f"{_BLOB.get('updated', '?')}/"
                  f"{hashlib.sha256(_RAW).hexdigest()[:10]}")

GRID_ROWS = int(_BLOB["grid"]["rows"])
GRID_COLS = int(_BLOB["grid"]["cols"])
GRID_NODE1_WORLD_IN: tuple[float, float] = tuple(
    float(v) for v in _BLOB["grid"]["node1_world_in"])
GRID_PITCH_IN = float(_BLOB["grid"]["pitch_in"])

NODES: dict[str, tuple[float, float]] = {
    str(k): (float(v[0]), float(v[1])) for k, v in _BLOB["nodes"].items()}

_LATTICE = GRID_ROWS * GRID_COLS
_BAYS = (GRID_ROWS - 1) * (GRID_COLS - 1)


def _num(prefix: str) -> dict[str, tuple[float, float]]:
    pat = re.compile(rf"^{prefix}(\d+)$")
    keys = sorted((k for k in NODES if pat.match(k)),
                  key=lambda k: int(pat.match(k).group(1)))
    return {k: NODES[k] for k in keys}


# Numbered nodes on this grid size, keyed by int.
NODE_WORLD_IN: dict[int, tuple[float, float]] = {
    int(k): v for k, v in NODES.items() if k.isdigit() and int(k) >= 1}
WORKSTATION_WORLD_IN: dict[int, tuple[float, float]] = {
    n: v for n, v in sorted(NODE_WORLD_IN.items())
    if _LATTICE < n <= _LATTICE + _BAYS}
DEPOT_SLOT_WORLD_IN: dict[str, tuple[float, float]] = _num("D")
DEPOT_ENTRY_WORLD_IN: dict[str, tuple[float, float]] = _num("DE")
NODE0_WORLD_IN: tuple[float, float] = NODES["0"]


def _check() -> None:
    want = {str(n) for n in range(1, _LATTICE + 2 * _BAYS + 1)} | {"0"}
    missing = sorted(want - set(NODES), key=lambda s: int(s))
    if missing:
        raise ValueError(f"{PATH}: nodes missing: {', '.join(missing[:12])}"
                         + (" ..." if len(missing) > 12 else ""))
    for k, (x, y) in NODES.items():
        if not (-5.0 < x < 110.0 and -5.0 < y < 110.0):
            raise ValueError(f"{PATH}: node {k} at ({x}, {y}) is off the table")


_check()


def _order(k: str):
    if k == "0":
        return (0, 0)
    if k.startswith("DE"):
        return (2, int(k[2:]))
    if k.startswith("D"):
        return (1, int(k[1:]))
    return (3, int(k))


def save(updates: dict[str, tuple[float, float]], updated: str,
         path: str = PATH) -> str:
    """Write `updates` (label -> (x_in, y_in)) into the file, one node per
    line, every other field kept. Returns the path. Used by
    calibrate_workstations.py --apply; the running programs pick the new
    values up at their next start."""
    with open(path, encoding="utf-8") as fh:
        blob = json.load(fh)
    nodes = {str(k): v for k, v in blob["nodes"].items()}
    for k, (x, y) in updates.items():
        nodes[str(k)] = [round(float(x), 2), round(float(y), 2)]
    blob["updated"] = updated
    head = {k: v for k, v in blob.items() if k != "nodes"}
    text = json.dumps(head, indent=2)[:-2] + ',\n  "nodes": {\n'
    keys = sorted(nodes, key=_order)
    text += ",\n".join(f'    "{k}": [{nodes[k][0]:.2f}, {nodes[k][1]:.2f}]'
                       for k in keys)
    text += "\n  }\n}\n"
    json.loads(text)                       # never write a file we cannot read
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    return path


if __name__ == "__main__":
    print(f"{PATH}\nid {COORDINATES_ID}\n")
    for k, (x, y) in NODES.items():
        print(f"  {k:>4}  {x:6.2f}  {y:6.2f}")
