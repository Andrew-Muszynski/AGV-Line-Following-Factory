const svg = document.getElementById("grid-svg");
const state = {
  rows: 8,
  cols: 8,
  agents: 4,
  capacity: 2,
  rpdMode: false,   // VRP-RPD: pick processed parts back up after drop-off
  processSec: 15,   // simulated workstation processing time (drop -> ready)
  selected: new Set(),
  routes: [],
  edgeUse: new Map(),
  totalDistance: 0,
  conflicts: [],
  schedule: null,
  hiddenAgents: new Set(),
  editAgent: 0,
  editMode: false,
  routeEdits: new Map(),
  mode: "sim",
  realSocket: null,
  realPositions: new Map(),
  visionPositions: new Map(),
  // Per-robot rostopic inspector (Run & Monitor button row): agent index ->
  // Map<topicSuffix, {value: string, receivedAt: number}>. Populated for
  // ANY currently-connected robot, independent of active dispatch state.
  robotTopics: new Map(),
  robotTopicsExpanded: new Set(),  // agent indices with the inspector panel open
  realStatus: "Simulated playback",
  timestep: 0,
  timer: null,
  missionStartMs: 0,  // wall-clock anchor for Real-mode workstation badges
  planSource: "",     // "" = built-in solver; else "algorithm (file)" import
  view: "setup",      // workflow view: "setup" | "plan" | "run" (CSS-only)
  decisionLog: [],    // Run & Monitor event feed from /fleet_events
  decisionLogSeen: new Set(),  // event seqs already logged (dedupe re-delivery)
  timelineToggles: { dropoff: true, pickup: true, depotLeave: true, depotReturn: true },
  // Latest /fleet_status lifecycle state from the supervisor (loaded/armed/
  // running/finished/aborted/error), set by renderSupervisorStatus() --
  // Page 4's KPI badge reads this so it can show ARMED/IDLE/RUNNING live,
  // not just the metrics DB's coarser created/running/completed/failed/
  // aborted run status. null until a /fleet_status message has arrived
  // this session.
  supervisorFleetStatus: null,
  // "color": FORWARD_UNTIL_<color>/turn command vocabulary, vision is a
  // monitor/veto only (the original, long-proven path). "vision": pure
  // AprilTag position/heading control (drive_leg/turn_to_heading), color
  // sensor never used. Sent with each /fleet_mission so it can be flipped
  // per test run without restarting fleetSupervisor.py.
  // Default changed 2026-08-20 to "vision" (camera only) -- that is the
  // mode actually being tested/used; color mode remains fully available
  // via the toggle.
  driveMode: "vision",
  // Turn-then-drive fusion (Camera only mode) -- skip the brake/settle
  // between a turn and the following drive when heading is already within
  // tolerance. Supervisor forces this off for missions with 2+ robots
  // regardless of this flag (see run_advised_vision()'s docstring).
  fuseTurns: false,
};

const palette = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0f766e", "#be185d", "#475569", "#ca8a04", "#0891b2", "#7c3aed", "#db2777"];
const NS = "http://www.w3.org/2000/svg";
const DEPOT_ENTRY_CLEARANCE_SEC = {
  // Alvik1-4: real hardware-measured depot-corridor clearance times (feeds
  // computeLaunchOffsets()'s collision-avoidance stagger -- see
  // depotClearanceSec() below; the per-hop times are DEPOT_MOVE_SEC).
  Alvik1: 14.7,
  Alvik2: 16.0,
  Alvik3: 18.1,
  Alvik4: 19.0,
  // Alvik5/6: NOT hardware-measured -- found 2026-08-06 that these were
  // missing entirely, silently falling back to Alvik4's value via
  // depotClearanceSec()'s `|| DEPOT_ENTRY_CLEARANCE_SEC.Alvik4`, which is
  // wrong (Alvik5/6 are farther from the depot corridor's clearance point
  // than Alvik4, so they need MORE time, not the same). Extrapolated here
  // from the Alvik1-4 trend (average step ~1.43s/robot: (16.0-14.7 +
  // 18.1-16.0 + 19.0-18.1)/3) as a placeholder ONLY -- re-measure on real
  // hardware the same way Alvik1-4 were (see the comment above
  // DEPOT_SLOT_WORLD_IN in camera_grid_navigate.py for that method) before
  // trusting these for anything safety-relevant with 5+ robots.
  Alvik5: 20.4,
  Alvik6: 21.9,
};
// Physical red-marker spacing on the table (get_pose() reports cm from
// reset_pose(0,0,0), taken at node0 right after the depot-exit maneuver).
// Every hop is 10in except node0->node1, which is 13.5in.
const NODE_SPACING_CM = 10 * 2.54;
const NODE0_TO_NODE1_SPACING_CM = 13.5 * 2.54;
// reset_pose(0,0,0) is called once at setup, while the robot faces SOUTH
// (before the depot-exit maneuver). Odometry x/y stay fixed to that initial
// world frame, so heading north into the grid afterward means -x (not +x).
// local -x = north (-r), local +y = east (+c).
// TODO: verify sign/axis against the real robot once pose tracking is
// tested on the table — flip here if the dot drifts backward.
function alvikPoseToGridPoint(xCm, yCm) {
  const g = layout();
  const northCm = -xCm;
  const rowsFromNode0 = northCm <= NODE0_TO_NODE1_SPACING_CM
    ? northCm / NODE0_TO_NODE1_SPACING_CM
    : 1 + (northCm - NODE0_TO_NODE1_SPACING_CM) / NODE_SPACING_CM;
  const colsFromNode0 = yCm / NODE_SPACING_CM;
  const r = (state.rows - 1) - rowsFromNode0;
  const c = colsFromNode0;
  return { x: g.left + c * g.cell, y: g.top + r * g.cell };
}
const MOVEMENT_SEC = {
  RED_RED: 3.56,
  YELLOW_RED: 1.70,
  YELLOW_YELLOW: 1.69,
  RED_YELLOW: 2.00,
  BLUE_RED: 4.19,
};
const TURN_SEC = {
  RIGHT: 2.65,
  LEFT: 2.65,
  ROTATE_180: 5.00,
};
const DEPOT_LAUNCH_ORDER = [3, 0, 2, 1];
const SAFETY_WINDOW_SEC = 5.0;
const EDGE_SAFETY_WINDOW_SEC = 7.0;
const EDGE_RELEASE_BUFFER_SEC = 0.1;
const AUTO_DECONFLICT_MAX_ITERATIONS = 260;
const SOLVE_POLISH_BUDGET_MS = 1800;
const REROUTE_BUDGET_MS = 1200;
const PATH_MODES = ["vh", "hv", "via-bottom", "via-top", "via-left", "via-right"];
// 2026-09-29: 127.0.0.1. The whole ROS stack (micro-ROS agent, rosbridge,
// fleetSupervisor) now runs in WSL2 on the Windows laptop, so rosbridge is
// co-located with whatever is showing this page.
//
// THIS IS THE LIVE COPY. app.py serves templates/index.html, which loads
// this file -- editing agv_grid_workstation_solver.html (the standalone
// file:// version) does NOT affect the page served on :8000. Both carry
// their own ROSBRIDGE_URL; keep them in step.
//
// History, because this value has flipped three times:
//   127.0.0.1      first WSL2 era
//   192.168.0.212  reverted 2026-08-03 -- WSL2-hosted rosbridge appeared
//                  to show a ~25s discovery-matching delay after every
//                  robot power-cycle, so the migration was put on hold
//   127.0.0.1      now. Re-tested 2026-09-29: it did NOT reproduce as a
//                  standing cost -- it hit once just after the stack came
//                  up and cleared on retry. A full route then ran with
//                  every turn inside 2deg.
//
// Use 127.0.0.1 and NOT the machine's own LAN IP: Windows cannot reliably
// reach its own WSL2-mirrored IP (a hairpin many Wi-Fi drivers drop), and
// loopback sidesteps it. Override with a query param if it ever moves:
//   ?rosbridge=192.168.0.212
const ROSBRIDGE_URL = (() => {
  let v = new URLSearchParams(window.location.search).get("rosbridge");
  if (!v) return "ws://192.168.0.212:9090";
  if (!v.startsWith("ws://") && !v.startsWith("wss://")) v = `ws://${v}`;
  if (!/:\d+$/.test(v)) v = `${v}:9090`;
  return v;
})();
// Live mode camera view: apriltag_localize.py --stream serves its annotated
// preview as MJPEG (default http://localhost:8081/stream — the camera runs on
// the same machine as this page). Override: ?stream=host[:port]
const LIVE_STREAM_URL = (() => {
  let v = new URLSearchParams(window.location.search).get("stream");
  if (!v) return "http://localhost:8081/stream";
  if (!v.startsWith("http://") && !v.startsWith("https://")) v = `http://${v}`;
  if (!v.endsWith("/stream")) v = `${v.replace(/\/$/, "")}/stream`;
  return v;
})();

// Real and Live modes both consume live robot data over rosbridge; Live
// additionally shows the camera's annotated MJPEG view above the grid.
// The Plan & Simulate view is ALWAYS pure simulation: the grid plays back the
// solved route step-by-step (Play/Step/slider), never vision, and never the
// camera feed — the live camera belongs to Run & Monitor. So Real/Live only
// take effect in the Run view; from Plan every consumer sees simulated data.
function liveDataMode() {
  if (state.view === "plan") return false;
  return state.mode === "real" || state.mode === "live";
}

// The "vrp_rpd:" route modes are solved by plan_server.py on this machine
// (the solvers are Python/numba/torch and can't run in the browser).
// Override: ?plansrv=host[:port]
const PLAN_SERVER_URL = (() => {
  let v = new URLSearchParams(window.location.search).get("plansrv");
  if (!v) return "http://localhost:8082";
  if (!v.startsWith("http://") && !v.startsWith("https://")) v = `http://${v}`;
  return v.replace(/\/$/, "");
})();

// Hard physical ceiling: the depot only has 6 real slots (D1..D6,
// hardware-measured -- see DEPOT_SLOT_WORLD_IN in camera_grid_navigate.py).
// A 7th agent would have nowhere to launch from or return to, so this is
// also the real bound for "Number of agents" (its own input element's
// max="12" is stale/wrong, not adjusted here -- fixing that is a UI-only
// follow-up, not blocking this fix).
const MAX_ROBOTS = 6;

// kind "odom": <ROBOT_NAME>_pose from AGV_Factory_color_pose.ino, std_msgs/String JSON
//   {"x":cm,"y":cm,"yaw":deg,"battery":pct,"ms":millis} (odometry frame).
// kind "vision": <ROBOT_NAME>_vision_pose from apriltag_localize.py
//   (VRP repo, camera laptop), std_msgs/String JSON {"x_in","y_in","yaw_deg",
//   "grid_x","grid_y","tag_id","ms"} — grid cells relative to node 1.
// When fresh vision is available it takes priority over odometry.
// GENERATED from MAX_ROBOTS (2026-08-06, was previously hardcoded to only
// Alvik1-4 -- found the hard way: Alvik5/6 silently got NO real battery
// data at all, and their "position" on the grid was actually the
// SIMULATED/predicted position from the solved plan silently substituting
// for missing real data via realPointForAgent()'s `|| pointAtSeconds(...)`
// fallback -- looked identical to real tracking in Real/Live mode but
// wasn't. Generating this from MAX_ROBOTS instead of hand-listing means a
// 7th robot (if the depot is ever expanded) can't silently repeat this).
const ALVIK_REAL_TOPICS = [
  ...Array.from({ length: MAX_ROBOTS }, (_, agent) => ({
    agent, topic: `${agvName(agent)}_pose`, type: "std_msgs/String", kind: "odom",
  })),
  ...Array.from({ length: MAX_ROBOTS }, (_, agent) => ({
    agent, topic: `${agvName(agent)}_vision_pose`, type: "std_msgs/String", kind: "vision",
  })),
];
const ALVIK_NAME_TO_AGENT = new Map(
  Array.from({ length: MAX_ROBOTS }, (_, agent) => [agvName(agent), agent])
);
// Vision poses older than this fall back to odometry.
const VISION_FRESH_MS = 2000;
// Where vision's grid origin (node 1, the first lattice node — NOT node 0 on
// its odd 13.5in link) sits in the on-screen grid, and how vision axes map:
// +grid_y (toward tag 22, "north" into the grid) decreases the row index,
// +grid_x (toward tag 21) increases the column. If robots render one row off
// or mirrored on the first live test, this pair of constants is the fix.
const VISION_NODE1_ROW_FROM_BOTTOM = 0;  // node 1 = bottom lattice row
const VISION_NODE1_COL = 0;              // node 1 = leftmost column

function visionGridToPoint(gridX, gridY) {
  const g = layout();
  const r = (state.rows - 1) - VISION_NODE1_ROW_FROM_BOTTOM - gridY;
  const c = VISION_NODE1_COL + gridX;
  return { x: g.left + c * g.cell, y: g.top + r * g.cell };
}

function el(tag, attrs = {}, text = "") {
  const node = document.createElementNS(NS, tag);
  for (const [key, value] of Object.entries(attrs)) node.setAttribute(key, value);
  if (text) node.textContent = text;
  return node;
}

function clampInt(value, lo, hi) {
  const n = parseInt(value, 10);
  if (Number.isNaN(n)) return lo;
  return Math.max(lo, Math.min(hi, n));
}

function bayId(r, c) {
  return `${r},${c}`;
}

function bayNumber(r, c) {
  // Bottom-up origin, matching the node numbering (robots enter from the
  // depot at the bottom): B1 = bottom-left bay, B7 = bottom-right, counting
  // upward row by row. Workstation/entry node numbers derive from this, so
  // the bottom-left workstation is 64+1 = 65 on an 8x8 grid.
  return (state.rows - 2 - r) * (state.cols - 1) + c + 1;
}

function bayFromNumber(n) {
  const perRow = state.cols - 1;
  return {
    r: state.rows - 2 - Math.floor((n - 1) / perRow),
    c: (n - 1) % perRow,
  };
}

function parseBayId(id) {
  const [r, c] = id.split(",").map(Number);
  return { r, c };
}

function nodeKey(r, c) {
  return `${r},${c}`;
}

function edgeKey(a, b) {
  return [a, b].sort().join("|");
}

// A bay ENTRY sticker lies mid-edge on the grid lattice — entry 122 sits on the
// corridor between grid nodes 18 and 19. So servicing that bay and traversing
// that grid edge share one physical lane. entryEdgeNodes maps an entry node
// label to the two grid nodes it lies between (null for grid/workstation/depot
// labels); corridorKey collapses the full grid edge and the two half-edges the
// entry splits it into to a single key, so the conflict detector treats them as
// one resource. Mirrors the off-robot dispatch geometry.
function entryEdgeNodes(label) {
  const n = Number(label);
  if (!Number.isInteger(n)) return null;
  const nodes = state.rows * state.cols;
  const bays = totalBayCount();
  if (!(n > nodes + bays && n <= nodes + 2 * bays)) return null;  // entries only
  const bay = bayFromNumber(n - nodes - bays);
  return [nodeNumber(bay.r, bay.c), nodeNumber(bay.r, bay.c + 1)];
}

function corridorKey(a, b) {
  for (const [endpoint, other] of [[a, b], [b, a]]) {
    const flank = entryEdgeNodes(endpoint);
    if (flank && (String(other) === String(flank[0]) ||
                  String(other) === String(flank[1]))) {
      return [flank[0], flank[1]].sort().join("<>");
    }
  }
  return [a, b].sort().join("<>");
}

function layout() {
  const pad = 80;
  // Cell size stays roughly constant (doesn't shrink as the grid grows) --
  // the canvas grows with rows/cols/agents instead, so a bigger grid or
  // fleet renders bigger on the page rather than cramming into a fixed box.
  // Depot geometry (DEPOT_* constants) is defined in grid-cell units, so it
  // scales correctly for free here; none of those constants change.
  const TARGET_CELL = 90;
  const gridColsSpan = Math.max(1, state.cols - 1);
  const gridRowsSpan = Math.max(1, state.rows - 1);
  const depotSlotsSpan = depotLaneEastGrid(state.agents);
  const wSpanCells = Math.max(gridColsSpan, depotSlotsSpan);
  const w = Math.round(wSpanCells * TARGET_CELL + pad * 2);
  const h = Math.round(gridRowsSpan * TARGET_CELL + 230);
  const gridW = w - pad * 2;
  const gridH = h - 230;
  const cell = Math.min(gridW / gridColsSpan, gridH / gridRowsSpan);
  const left = (w - cell * gridColsSpan) / 2;
  const top = 58;
  const bottom = top + cell * gridRowsSpan;
  // The depot (slots, DE entries, node 0, lane) is drawn in the vision-grid
  // frame via visionGridToPoint (see DEPOT_* constants + depotSlot/specialPt),
  // so it shares the live robot dots' coordinate system and lines up. No
  // separate depot Y-offsets are needed here anymore.
  return { w, h, cell, left, top, bottom };
}

function pt(r, c) {
  const g = layout();
  return { x: g.left + c * g.cell, y: g.top + r * g.cell };
}

function specialPt(id) {
  if (id === "node0") return visionGridToPoint(DEPOT_NODE0_GRID.x, DEPOT_NODE0_GRID.y);
  if (id.startsWith("depot-entry-")) {
    const i = parseInt(id.replace("depot-entry-", ""), 10);
    return depotEntry(i);
  }
  if (id.startsWith("depot-slot-")) {
    const i = parseInt(id.replace("depot-slot-", ""), 10);
    return depotSlot(i);
  }
  return visionGridToPoint(DEPOT_NODE0_GRID.x, DEPOT_NODE0_GRID.y);
}

function pointFor(pathPoint) {
  if (pathPoint.kind === "grid") return pt(pathPoint.r, pathPoint.c);
  if (pathPoint.kind === "entry") return bayEntryPoint(pathPoint.bay);
  if (pathPoint.kind === "workstation") return bayWorkPoint(pathPoint.bay);
  return specialPt(pathPoint.id);
}

function pathKey(pathPoint) {
  if (pathPoint.kind === "grid") return `g:${pathPoint.r},${pathPoint.c}`;
  if (pathPoint.kind === "entry") return `e:${bayId(pathPoint.bay.r, pathPoint.bay.c)}`;
  if (pathPoint.kind === "workstation") return `w:${bayId(pathPoint.bay.r, pathPoint.bay.c)}`;
  return `s:${pathPoint.id}`;
}

function pathLabel(pathPoint) {
  if (pathPoint.kind === "grid") return nodeNumber(pathPoint.r, pathPoint.c);
  if (pathPoint.kind === "entry") return entryNodeNumber(pathPoint.bay);
  if (pathPoint.kind === "workstation") return workstationNodeNumber(pathPoint.bay);
  if (pathPoint.id === "node0") return 0;
  if (pathPoint.id.startsWith("depot-entry-")) {
    return `DE${parseInt(pathPoint.id.replace("depot-entry-", ""), 10) + 1}`;
  }
  if (pathPoint.id.startsWith("depot-slot-")) {
    return `D${parseInt(pathPoint.id.replace("depot-slot-", ""), 10) + 1}`;
  }
  return pathPoint.id;
}

// Depot geometry in VISION-GRID coordinates (gridX east, gridY north of
// node 1), so the drawn depot and the live robot dots share one coordinate
// system via visionGridToPoint() — they line up in the digital twin. Node 0 is
// the lane junction 1.35 cells south of node 1 (matches label_to_cell(0) on the
// supervisor side). Slots sit ~0.55 cells south of node 1, pitched east; the
// DE entries are on the node-0 lane just south of each slot.
// Real geometry (overhead photo): node 1 drops STRAIGHT DOWN to node 0, which
// makes a 90° turn into a horizontal lane running east. Robots park in a
// straight row above the lane, each with a vertical stub down to the lane. So:
// the DE entries sit ON the lane (same y as node 0), directly below each slot,
// giving clean right angles — no diagonals.
const DEPOT_LANE_GRID_Y = -1.35;   // the horizontal lane (node 0's row)
// SURVEYED, not modelled. Grid-cell coordinates derived from the measured
// DEPOT_SLOT_WORLD_IN table in camera_grid_navigate.py, with node 1 at
// (13.5, 16.75) in and a 10 in pitch. Re-surveyed 2026-09-30.
//
// REPLACES a uniform "x = 0.5 + 0.6 * i" model whose error grew with the
// slot index -- 0.110 cells at D1 to 0.290 at D6 -- because the real slots
// are not evenly pitched. Two consequences, both seen on the page: the
// drawn boxes sat progressively left of the robots, and D6 vanished
// entirely. The box is hidden when a robot is further from its slot than
// 35% of a cell (so a robot out on a route does not also render parked),
// and D6's combined x+y model error was 0.329 cells -- just inside that
// threshold, so a correctly parked Alvik6 was read as "away".
const DEPOT_SLOT_GRID = [
  { x: 0.610, y: -0.617 },   // D1  (19.60, 10.58) in
  { x: 1.240, y: -0.605 },   // D2  (25.90, 10.70) in
  { x: 1.890, y: -0.605 },   // D3  (32.40, 10.70) in
  { x: 2.522, y: -0.585 },   // D4  (38.72, 10.90) in
  { x: 3.160, y: -0.595 },   // D5  (45.10, 10.80) in
  { x: 3.790, y: -0.595 },   // D6  (51.40, 10.80) in
  { x: 4.384, y: -0.552 },   // D7  (57.34, 11.23) in
];
const DEPOT_SLOT_GRID_Y = -0.75;   // nominal only; used past the table
const DEPOT_ENTRY_GRID_Y = DEPOT_LANE_GRID_Y;  // DE points are ON the lane
const DEPOT_NODE0_GRID = { x: 0, y: DEPOT_LANE_GRID_Y };

// The depot slot's grid position. Depot slots are fixed hardware -- always
// the nominal, measured layout, never inferred from a live vision reading.
// (A previous version of this function preferred a "learned" position from
// vision, updated live or on first sighting; a noisy/misdetected frame near
// the depot region -- from a robot passing through during unrelated testing
// -- silently corrupted the whole depot layout more than once, with no
// visible cause, since the depots never actually move. Removed entirely.)
function depotSlotGrid(i) {
  const s = DEPOT_SLOT_GRID[i];
  if (s) return s;
  // Past the surveyed table: extrapolate on the mean measured pitch rather
  // than the old nominal, so an 8th slot lands near where it would really be.
  const last = DEPOT_SLOT_GRID[DEPOT_SLOT_GRID.length - 1];
  const pitch = (last.x - DEPOT_SLOT_GRID[0].x) / (DEPOT_SLOT_GRID.length - 1);
  return { x: last.x + pitch * (i - DEPOT_SLOT_GRID.length + 1), y: last.y };
}

// East end of the drawn lane: half a cell past the last occupied slot.
function depotLaneEastGrid(agents) {
  return depotSlotGrid(Math.max(0, agents - 1)).x + 0.5;
}

function depotSlot(i) {
  // The single source for a depot slot's screen position: the learned real
  // parking grid coord (or nominal until vision has measured it), through
  // visionGridToPoint. Everything — box, DE point, route line, congestion
  // overlay — uses this, so they can't drift apart. Depot TIMING comes from
  // measured clearance constants, not this pixel position, so it stays
  // deterministic.
  const grid = depotSlotGrid(i);
  return visionGridToPoint(grid.x, grid.y);
}


function depotEntry(i) {
  const grid = depotSlotGrid(i);
  return visionGridToPoint(grid.x, DEPOT_ENTRY_GRID_Y);
}

function depotEntryPathToNode0(agent) {
  const path = [specialPathPoint(`depot-slot-${agent}`), specialPathPoint(`depot-entry-${agent}`)];
  for (let i = agent - 1; i >= 0; i--) {
    path.push(specialPathPoint(`depot-entry-${i}`));
  }
  path.push(specialPathPoint("node0"));
  return path;
}

function depotEntryPathFromNode0(agent) {
  const path = [specialPathPoint("node0")];
  for (let i = 0; i <= agent; i++) {
    path.push(specialPathPoint(`depot-entry-${i}`));
  }
  path.push(specialPathPoint(`depot-slot-${agent}`));
  return path;
}

function routeStart(i) {
  return { r: state.rows - 1, c: 0 };
}

function isAgentVisible(agent) {
  return !state.hiddenAgents.has(agent);
}

function visibleRoutes(routes = state.routes) {
  return routes.filter((route) => isAgentVisible(route.agent));
}

function clampAgentState() {
  if (state.editAgent >= state.agents) state.editAgent = Math.max(0, state.agents - 1);
  for (const agent of [...state.hiddenAgents]) {
    if (agent >= state.agents) state.hiddenAgents.delete(agent);
  }
  for (const agent of [...state.routeEdits.keys()]) {
    if (agent >= state.agents) state.routeEdits.delete(agent);
  }
}

function cloneWaypoints(waypoints) {
  return (waypoints || []).map((node) => ({ r: node.r, c: node.c }));
}

function cloneWaits(waitBeforeSec) {
  const waits = {};
  for (const [key, value] of Object.entries(waitBeforeSec || {})) {
    const seconds = Number(value) || 0;
    if (seconds > 0) waits[key] = seconds;
  }
  return waits;
}

function totalRouteDwell(route) {
  return Object.values(route.waitBeforeSec || {}).reduce((sum, value) => sum + (Number(value) || 0), 0);
}

function totalPlanDwell(routes) {
  return routes.reduce((sum, route) => sum + totalRouteDwell(route) + (route.extraWaitSec || 0), 0);
}

function manualWaypointsFor(agent) {
  return cloneWaypoints(state.routeEdits.get(agent) || []);
}

function setManualWaypoints(agent, waypoints) {
  if (waypoints && waypoints.length) {
    state.routeEdits.set(agent, cloneWaypoints(waypoints));
  } else {
    state.routeEdits.delete(agent);
  }
}

function bayEntryNode(bay) {
  return { r: bay.r, c: bay.c };
}

function gridPathPoint(node) {
  return { kind: "grid", r: node.r, c: node.c };
}

function specialPathPoint(id) {
  return { kind: "special", id };
}

function entryPathPoint(bay, mode) {
  const point = { kind: "entry", bay: { r: bay.r, c: bay.c } };
  if (mode) point.mode = mode;
  return point;
}

function workstationPathPoint(bay, mode) {
  const point = { kind: "workstation", bay: { r: bay.r, c: bay.c } };
  if (mode) point.mode = mode;
  return point;
}

function nodeNumber(r, c) {
  return (state.rows - 1 - r) * state.cols + c + 1;
}

function totalBayCount() {
  return (state.rows - 1) * (state.cols - 1);
}

function workstationNodeNumber(bay) {
  return state.rows * state.cols + bayNumber(bay.r, bay.c);
}

function entryNodeNumber(bay) {
  return state.rows * state.cols + totalBayCount() + bayNumber(bay.r, bay.c);
}

function bayEntryPoint(bay) {
  const a = pt(bay.r, bay.c);
  const b = pt(bay.r, bay.c + 1);
  // The entry sticker lies mid-edge on the grid lattice, directly on the
  // line between its two flanking intersections (see entryEdgeNodes /
  // corridorKey) -- not offset below it.
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

function bayWorkPoint(bay) {
  const g = layout();
  const a = pt(bay.r, bay.c);
  return { x: a.x + g.cell / 2, y: a.y + g.cell * 0.54 };
}

function appendNodeIfNew(path, node) {
  const last = path[path.length - 1];
  if (!last || last.r !== node.r || last.c !== node.c) path.push({ ...node });
}

function appendAxisPath(path, end, axisOrder) {
  let last = path[path.length - 1];
  let r = last.r;
  let c = last.c;
  const moveRows = () => {
    while (r !== end.r) {
      r += end.r > r ? 1 : -1;
      appendNodeIfNew(path, { r, c });
    }
  };
  const moveCols = () => {
    while (c !== end.c) {
      c += end.c > c ? 1 : -1;
      appendNodeIfNew(path, { r, c });
    }
  };
  for (const axis of axisOrder) {
    if (axis === "r") moveRows();
    else moveCols();
  }
}

function pathBetween(start, end, mode = "vh") {
  const path = [{ ...start }];
  const via = [];
  if (mode === "via-bottom") {
    via.push({ r: state.rows - 1, c: start.c }, { r: state.rows - 1, c: end.c });
  } else if (mode === "via-top") {
    via.push({ r: 0, c: start.c }, { r: 0, c: end.c });
  } else if (mode === "via-left") {
    via.push({ r: start.r, c: 0 }, { r: end.r, c: 0 });
  } else if (mode === "via-right") {
    via.push({ r: start.r, c: state.cols - 1 }, { r: end.r, c: state.cols - 1 });
  }
  for (const waypoint of via) appendAxisPath(path, waypoint, mode === "hv" ? ["c", "r"] : ["r", "c"]);
  appendAxisPath(path, end, mode === "hv" ? ["c", "r"] : ["r", "c"]);
  return path;
}

function routeDistance(points) {
  let distance = 0;
  for (let i = 1; i < points.length; i++) {
    distance += Math.abs(points[i].r - points[i - 1].r) + Math.abs(points[i].c - points[i - 1].c);
  }
  return distance;
}

function seededRandom(seed) {
  let x = Math.sin(seed || 1) * 10000;
  return () => {
    x = Math.sin(x) * 10000;
    return x - Math.floor(x);
  };
}

function agvName(agent) {
  return `Alvik${agent + 1}`;
}

function depotClearanceSec(agent) {
  return DEPOT_ENTRY_CLEARANCE_SEC[agvName(agent)] || DEPOT_ENTRY_CLEARANCE_SEC.Alvik4;
}

// Measured on hardware 2026-09-15 (runs_linux/fleet_log_20260915.txt, via
// tools/launch_cascade.py): each leg's "driving from"/"brake fired" pair.
// These are the SAME for every robot -- they are one lane hop, not a
// property of which robot is doing it.
//
// REPLACES depotClearanceSec(agent)/(agent+2), which had no physical basis:
// it amortised a whole-corridor clearance across a hop count and so produced
// 7.35, 5.33, 4.53, 3.80, 3.40 and 3.13 s/hop -- every successive robot
// FASTER than the one ahead of it. In the cascade that made followers close
// on their leader by up to 2s per hop, which is what Plan & Simulate showed
// on 2026-09-30. It also inflated the plan everywhere else: the planner
// thought D# -> node 0 took 14.7-21.9s against a measured 6.3-15.7s.
//
// The staggered launch OFFSETS are unaffected -- computeLaunchOffsets() uses
// depotClearanceSec() directly and is left exactly as it was.
const DEPOT_MOVE_SEC = {
  SLOT_TO_ENTRY: 2.22,   // D# -> DE#, south out of the parking slot
  ENTRY_TO_ENTRY: 2.12,  // DE(n+1) -> DE(n), 1.74 driving + 0.38 settling
  ENTRY_TO_NODE0: 1.82,  // DE1 -> node 0
};

function launchOrderForAgents(count) {
  const order = DEPOT_LAUNCH_ORDER.filter((agent) => agent < count);
  for (let agent = 0; agent < count; agent++) {
    if (!order.includes(agent)) order.push(agent);
  }
  return order;
}

function computeLaunchOffsets(count) {
  const order = launchOrderForAgents(count);
  const offsets = Array.from({ length: count }, () => 0);
  if (!order.length) return offsets;
  offsets[order[0]] = 0;
  for (let i = 1; i < order.length; i++) {
    const prev = order[i - 1];
    const next = order[i];
    const wait = prev === 0 && next === 2
      ? depotClearanceSec(prev)
      : depotClearanceSec(prev) + MOVEMENT_SEC.BLUE_RED;
    offsets[next] = offsets[prev] + wait;
  }
  return offsets;
}

function headingBetween(fromPoint, toPoint) {
  const a = pointFor(fromPoint);
  const b = pointFor(toPoint);
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  if (Math.abs(dx) >= Math.abs(dy)) return dx >= 0 ? "E" : "W";
  return dy >= 0 ? "S" : "N";
}

function headingsOpposed(a, b) {
  if (!a || !b) return false;
  const order = ["N", "E", "S", "W"];
  const ia = order.indexOf(a);
  const ib = order.indexOf(b);
  return ia >= 0 && ib >= 0 && (ib - ia + 4) % 4 === 2;
}

function turnPenaltySec(prevHeading, nextHeading) {
  if (!prevHeading || !nextHeading || prevHeading === nextHeading) return 0;
  const order = ["N", "E", "S", "W"];
  const a = order.indexOf(prevHeading);
  const b = order.indexOf(nextHeading);
  if (a < 0 || b < 0) return 0;
  const delta = (b - a + 4) % 4;
  if (delta === 2) return TURN_SEC.ROTATE_180;
  if (delta === 1) return TURN_SEC.RIGHT;
  return TURN_SEC.LEFT;
}

function movementProfile(fromPoint, toPoint, agent) {
  const fromKind = fromPoint.kind;
  const toKind = toPoint.kind;
  const fromLabel = pathLabel(fromPoint);
  const toLabel = pathLabel(toPoint);

  const fromDepotSlot = String(fromLabel).startsWith("D") && !String(fromLabel).startsWith("DE");
  const toDepotSlot = String(toLabel).startsWith("D") && !String(toLabel).startsWith("DE");
  const fromDepotEntry = String(fromLabel).startsWith("DE");
  const toDepotEntry = String(toLabel).startsWith("DE");

  if (fromDepotSlot && toDepotEntry) {
    return { seconds: DEPOT_MOVE_SEC.SLOT_TO_ENTRY, type: "DEPOT_SLOT_TO_ENTRY" };
  }
  if (fromDepotEntry && toDepotEntry) {
    return { seconds: DEPOT_MOVE_SEC.ENTRY_TO_ENTRY, type: "DEPOT_ENTRY_TO_ENTRY" };
  }
  if (fromDepotEntry && toLabel === 0) {
    return { seconds: DEPOT_MOVE_SEC.ENTRY_TO_NODE0, type: "DEPOT_ENTRY_TO_NODE_0" };
  }
  if (fromLabel === 0 && toLabel === 1) {
    return { seconds: MOVEMENT_SEC.BLUE_RED, type: "BLUE_TO_RED" };
  }
  if (fromLabel === 1 && toLabel === 0) {
    return { seconds: MOVEMENT_SEC.BLUE_RED, type: "RED_TO_BLUE_RETURN" };
  }
  if (fromLabel === 0 && toDepotEntry) {
    return { seconds: DEPOT_MOVE_SEC.ENTRY_TO_NODE0, type: "NODE_0_TO_DEPOT_ENTRY" };  // same distance, reversed
  }
  if (fromDepotEntry && toDepotSlot) {
    return { seconds: DEPOT_MOVE_SEC.SLOT_TO_ENTRY, type: "DEPOT_ENTRY_TO_SLOT" };  // same distance, reversed
  }
  if (fromKind === "grid" && toKind === "grid") {
    return { seconds: MOVEMENT_SEC.RED_RED, type: "RED_TO_RED" };
  }
  if (fromKind === "grid" && toKind === "entry") {
    return { seconds: MOVEMENT_SEC.RED_YELLOW, type: "RED_TO_YELLOW" };
  }
  if (fromKind === "entry" && toKind === "grid") {
    return { seconds: MOVEMENT_SEC.YELLOW_RED, type: "YELLOW_TO_RED" };
  }
  if ((fromKind === "entry" && toKind === "workstation") ||
      (fromKind === "workstation" && toKind === "entry")) {
    return { seconds: MOVEMENT_SEC.YELLOW_YELLOW, type: "YELLOW_TO_YELLOW" };
  }
  return { seconds: MOVEMENT_SEC.RED_RED, type: "MOVE" };
}

function applyRouteTiming(route) {
  const segments = [];
  const events = [];
  let t = route.launchOffsetSec || 0;
  let prevHeading = null;
  if (route.path.length) {
    events.push({
      node: pathLabel(route.path[0]),
      pathIndex: 0,
      arrivalSec: 0,
      departSec: t,
      note: t > 0 ? "waiting for launch clearance" : "launch",
    });
  }

  for (let i = 1; i < route.path.length; i++) {
    const from = route.path[i - 1];
    const to = route.path[i];
    const heading = headingBetween(from, to);
    const movement = movementProfile(from, to, route.agent);
    const measuredCompleteMove = movement.type.startsWith("DEPOT") ||
      movement.type === "BLUE_TO_RED" ||
      movement.type === "RED_TO_BLUE_RETURN";
    const turnSec = measuredCompleteMove ? 0 : turnPenaltySec(prevHeading, heading);
    const waitSec = Math.max(0, Number((route.waitBeforeSec || {})[i]) || 0);
    if (waitSec > 0) {
      const lastEvent = events[events.length - 1];
      if (lastEvent) {
        lastEvent.departSec = t + waitSec;
        lastEvent.dwellSec = (lastEvent.dwellSec || 0) + waitSec;
        lastEvent.note = lastEvent.note ? `${lastEvent.note}; dwell` : "dwell";
      }
      t += waitSec;
    }
    const startSec = t;
    const endSec = startSec + turnSec + movement.seconds;
    segments.push({
      from,
      to,
      segmentIndex: i,
      fromNode: pathLabel(from),
      toNode: pathLabel(to),
      startSec,
      endSec,
      moveSec: movement.seconds,
      turnSec,
      waitBeforeSec: waitSec,
      type: movement.type,
      heading,
    });
    events.push({
      node: pathLabel(to),
      pathIndex: i,
      arrivalSec: endSec,
      departSec: endSec,
      via: movement.type,
      turnSec,
    });
    t = endSec;
    prevHeading = heading;
  }

  route.segments = segments;
  route.events = events;
  route.durationSec = t;
  route.distance = t;
  return route;
}

// ---------------------------------------------------------------- launch
// Depot hop times are measured and uniform -- see DEPOT_MOVE_SEC. The
// cascade below does not depend on that: it gates against whatever segment
// times the route actually has, so it stays correct if the model changes.

function launchModeSetting() {
  const el = document.getElementById("launch-mode");
  return el && el.value === "cascade" ? "cascade" : "staggered";
}

// STAGGERED: hold each robot in its SLOT for a fixed offset, then run the
// whole route. What the solver has always done -- but on
// nearest_2026-09-09 it spent 307s of queueing, ~42% of the critical path,
// with robots sitting still in their slots.
function applyStaggeredSchedule(routes) {
  const offsets = computeLaunchOffsets(routes.length);
  for (const route of routes) {
    route.launchOffsetSec = offsets[route.agent] + (route.extraWaitSec || 0);
    applyRouteTiming(route);
  }
  return routes;
}

// CASCADE, as specified by the operator 2026-09-30:
//
//   1. all six move D# -> DE# and turn west at the same time -- distinct
//      nodes, no contention, and the turn is paid in parallel;
//   2. Alvik1 sets off; the other five stay put;
//   3. when Alvik1 reaches node 0, Alvik2 is released to DE1;
//      when Alvik2 reaches DE1, Alvik3 is released to DE2; and so on.
//
// The release condition is ARRIVAL, not departure: the robot ahead must have
// finished its hop, not merely started moving away. Robots therefore enter
// the grid strictly in order 1,2,3...
//
// WHY EVERY HOP IS GATED, not just the first. Delaying each robot once at
// its entry node and letting it run is only correct if every robot covers a
// lane hop in the same time. That is true of the MEASURED numbers, but it
// was not true of the model this file used to carry (7.35 down to 3.13
// s/hop, each robot faster than the one ahead), and a follower then closed
// on its leader by up to 2s per hop. Gating every hop is correct either way.
//
// Gating on the route's OWN segment times rather than on the measured
// constants keeps the plan self-consistent whatever that model says. If the
// timing model is later corrected, this needs no change.
function applyCascadeSchedule(routes) {
  const ordered = routes.slice().sort((a, b) => a.agent - b.agent);
  const isLane = (l) => /^DE\d+$/i.test(String(l)) || String(l) === "0";
  let ahead = null;            // node label -> when the robot ahead ARRIVED

  for (const route of ordered) {
    // Nothing is held back in the slot. A manual per-robot delay is still
    // honoured: that is the operator asking for it.
    route.launchOffsetSec = route.extraWaitSec || 0;
    const baseWaits = Object.assign({}, route.waitBeforeSec || {});

    // Pass 1: natural timing, so the gate is computed against the same
    // seconds the plan will actually use (turn penalties included).
    route.waitBeforeSec = baseWaits;
    applyRouteTiming(route);

    const waits = Object.assign({}, baseWaits);
    const mine = new Map();
    let delay = 0;
    for (const seg of route.segments || []) {
      const target = String(seg.toNode);
      const from = String(seg.fromNode);
      // D# -> DE# IS NEVER GATED. All six make that move, and the turn that
      // follows it, at the same time -- they are distinct nodes with no
      // contention, and holding a robot in its slot is pure added wait.
      // Gating started here by mistake (isLane(target) was true for DE#),
      // which left the fleet queued in the depot instead of pre-staged in
      // the lane. Caught in Plan & Simulate 2026-09-30.
      if (!isLane(from)) continue;
      let gate = 0;
      if (ahead) {
        // The node PAST this one: the robot ahead must have reached it.
        const beyond = beyondInLane(route, seg.segmentIndex);
        if (beyond !== null && ahead.has(beyond)) gate = ahead.get(beyond);
      }
      const start = seg.startSec + delay;
      const extra = Math.max(0, gate - start);
      if (extra > 0) {
        waits[seg.segmentIndex] =
          (Number(waits[seg.segmentIndex]) || 0) + extra;
        delay += extra;
      }
      mine.set(target, seg.endSec + delay);
      // Record the first node past the lane too -- the follower entering
      // node 0 has to wait for the leader to LEAVE it, which is the same
      // instant as the leader arriving at node 1.
      if (!isLane(target)) break;
    }

    // Pass 2: re-time with the gates applied.
    route.waitBeforeSec = waits;
    applyRouteTiming(route);
    ahead = mine;
  }
  return routes;
}

// The label one step further along this route than segment `idx`.
function beyondInLane(route, idx) {
  for (const seg of route.segments || []) {
    if (seg.segmentIndex === idx + 1) return String(seg.toNode);
  }
  return null;
}

function applyLaunchSchedule(routes) {
  return launchModeSetting() === "cascade"
    ? applyCascadeSchedule(routes)
    : applyStaggeredSchedule(routes);
}

function buildAgentRoute(
  agent,
  baysForAgent,
  extraWaitSec = 0,
  manualWaypoints = manualWaypointsFor(agent),
  waitBeforeSec = {},
  pathMode = "vh"
) {
  // Defensive service-merge: guarantee no adjacent same-bay drop+pick reaches
  // the path builder unmerged (which would force a mid-grid ROTATE_180). Every
  // path into buildAgentRoute — built-in solve, external plan, import, re-solve
  // during collision resolution — passes through here, so this is the single
  // choke point that makes the "never double back at a workstation" rule hold.
  baysForAgent = mergeServiceVisits(baysForAgent);

  let cur = routeStart(agent);
  const fullPath = [
    ...depotEntryPathToNode0(agent),
    gridPathPoint(routeStart(agent)),
  ];

  for (const waypoint of manualWaypoints) {
    const segment = pathBetween(cur, waypoint, pathMode);
    fullPath.push(...segment.slice(1).map(gridPathPoint));
    cur = { r: waypoint.r, c: waypoint.c };
  }

  for (let b = 0; b < baysForAgent.length; b++) {
    const bay = baysForAgent[b];
    // The entry sticker sits mid-edge between the bay's west (bay.c) and
    // east (bay.c+1) row nodes, so the bay is servable from either side.
    // Routing to the far side crosses the bay's own entry edge and forces a
    // mid-grid about-face (Alvik3 drove past its workstation, ROTATE_180'd
    // at the next red node and came back — hardware run 2026-07-14). Score
    // both sides: no heading flip first, then shortest approach.
    const sides = [
      { node: { r: bay.r, c: bay.c } },
      { node: { r: bay.r, c: bay.c + 1 } },
    ];
    for (const side of sides) {
      side.segment = pathBetween(cur, side.node, pathMode);
      const tail = [...fullPath, ...side.segment.slice(1).map(gridPathPoint)];
      const arrival = headingBetween(tail[tail.length - 2], tail[tail.length - 1]);
      const entering = headingBetween(gridPathPoint(side.node), entryPathPoint(bay));
      side.flip = headingsOpposed(arrival, entering) ? 1 : 0;
      side.dist = routeDistance(side.segment);
    }
    sides.sort((x, y) => x.flip - y.flip || x.dist - y.dist);
    fullPath.push(...sides[0].segment.slice(1).map(gridPathPoint));
    // Leave toward the next objective (next bay's entry edge, else home);
    // exiting out the far side of the edge would just re-cover ground.
    const nextRef = b + 1 < baysForAgent.length
      ? { r: baysForAgent[b + 1].r, c: baysForAgent[b + 1].c + 0.5 }
      : routeStart(agent);
    const west = { r: bay.r, c: bay.c };
    const east = { r: bay.r, c: bay.c + 1 };
    const towardNext = (node) =>
      Math.abs(node.r - nextRef.r) + Math.abs(node.c - nextRef.c);
    const exit = towardNext(east) < towardNext(west) ? east : west;
    const mode = bay.mode || "drop";
    fullPath.push(
      entryPathPoint(bay, mode),
      workstationPathPoint(bay, mode),
      entryPathPoint(bay, mode),
      gridPathPoint(exit)
    );
    cur = exit;
  }

  const home = routeStart(agent);
  const returnSegment = pathBetween(cur, home, pathMode);
  fullPath.push(...returnSegment.slice(1).map(gridPathPoint));
  fullPath.push(...depotEntryPathFromNode0(agent));

  return {
    agent,
    bays: cloneBays(baysForAgent),
    path: fullPath,
    segments: [],
    events: [],
    distance: 0,
    durationSec: 0,
    launchOffsetSec: 0,
    extraWaitSec,
    manualWaypoints: cloneWaypoints(manualWaypoints),
    waitBeforeSec: cloneWaits(waitBeforeSec),
    pathMode,
  };
}

function distributeWork() {
  const bays = [...state.selected].map(parseBayId);
  const mode = document.getElementById("route-mode").value;
  const seed = clampInt(document.getElementById("seed").value, 1, 999999);
  const rng = seededRandom(seed);

  if (mode === "sweep") {
    bays.sort((a, b) => a.r - b.r || a.c - b.c);
  } else {
    bays.sort(() => rng() - 0.5);
  }

  const buckets = Array.from({ length: state.agents }, () => []);
  bays.forEach((bay, i) => buckets[i % state.agents].push(bay));

  if (mode === "nearest") {
    for (let i = 0; i < buckets.length; i++) {
      const ordered = [];
      const remaining = [...buckets[i]];
      let cur = routeStart(i);
      while (remaining.length) {
        let best = 0;
        let bestDist = Infinity;
        for (let j = 0; j < remaining.length; j++) {
          const entry = bayEntryNode(remaining[j]);
          const d = Math.abs(cur.r - entry.r) + Math.abs(cur.c - entry.c);
          if (d < bestDist) {
            best = j;
            bestDist = d;
          }
        }
        const next = remaining.splice(best, 1)[0];
        ordered.push(next);
        cur = bayEntryNode(next);
      }
      buckets[i] = ordered;
    }
  }

  let templates = buckets.map((baysForAgent, agent) => buildAgentRoute(agent, baysForAgent, 0));
  if (state.rpdMode) templates = assignPickups(templates);
  // includeOrders=true (2026-08-20): the built-in solve previously only let
  // polishCollisionPlan() try alternate PATH_MODES for a route, never
  // alternate bay VISIT ORDERS -- reordering is what candidateOrders()
  // actually needs to escape a mid-grid ROTATE_180 that's forced by which
  // side of a bay the robot happens to approach from (see buildAgentRoute()'s
  // own sides.sort() fallback: it already prefers a flip-free approach side
  // per bay, but if NEITHER side avoids a flip for the CURRENT bay order, it
  // has no choice but to accept one -- a different visit order can change
  // which side that ends up being). This mattered little for multi-robot
  // missions (conflictFocusAgents() already ran polishCollisionPlan for
  // agents in real scheduling conflicts, which passes includeOrders=true via
  // rerouteAgent()'s own manual-reroute path), but a SINGLE-agent mission has
  // no scheduling conflict to trigger that at all -- confirmed 2026-08-20 via
  // a real 1-agent/12-bay mission producing mid-grid flips that nothing ever
  // searched an alternative for. midGridFlips is already the top-priority
  // "fatal" tier in evaluateRoutes()/isBetterPlanEval(), so enabling this
  // search doesn't change WHAT wins when a flip-free candidate exists --
  // it only widens what candidates get tried in the first place.
  const polished = polishCollisionPlan(templates, SOLVE_POLISH_BUDGET_MS, null, true);
  state.routes = polished.routes;
  state.conflicts = polished.conflicts;
  state.schedule = polished.schedule;
  state.planSource = "";  // built-in solver owns the plan again
  invalidateDispatchEditors();
}

// Generated command text belongs to the plan it was generated from. After
// any re-solve/reroute it is stale — Send mission would pair old commands
// with the new schedule and the supervisor's alignment gate would reject the
// mission (observed 2026-07-16). Drop the editors so the next Send/Start
// regenerates fresh text from the current plan.
function invalidateDispatchEditors() {
  const editors = document.getElementById("dispatch-editors");
  if (editors && editors.innerHTML) {
    editors.innerHTML = "";
    dispatchSetStatus("plan changed — command text cleared; Generate again to review");
  }
}

// ---- VRP-RPD: pickup assignment -------------------------------------------
// Every dropped part becomes pickable state.processSec after its drop-off and
// must be returned to the depot. Pickups are NOT tied to the robot that
// delivered: each is auctioned to whichever robot (with pickup capacity left)
// finishes its tour earliest after appending it — a greedy earliest-completion
// heuristic that lowers fleet makespan. Tours stay drops-then-picks, which is
// always capacity-feasible (load 3 -> 0 -> 3).
function assignPickups(templateRoutes) {
  const offsets = computeLaunchOffsets(templateRoutes.length);
  const visits = templateRoutes.map((route) => cloneBays(route.bays));
  const meta = templateRoutes.map((route) => ({
    manualWaypoints: route.manualWaypoints || [],
    pathMode: route.pathMode || "vh",
  }));

  const buildTour = (agent) => {
    const route = buildAgentRoute(
      agent, visits[agent], 0, meta[agent].manualWaypoints, {}, meta[agent].pathMode);
    route.launchOffsetSec = offsets[agent] || 0;
    return applyRouteTiming(route);
  };

  // Ready times come from the delivery legs, which pickup appends never move.
  const readyAt = new Map();
  for (let agent = 0; agent < visits.length; agent++) {
    for (const seg of buildTour(agent).segments) {
      if (seg.to && seg.to.kind === "workstation" && seg.to.mode !== "pick") {
        readyAt.set(bayId(seg.to.bay.r, seg.to.bay.c), seg.endSec + state.processSec);
      }
    }
  }

  // Tour completion including waits for parts that aren't ready on arrival
  // (waits cascade: an early wait delays every later pickup in the tour).
  const tourCompletion = (agent) => {
    const route = buildTour(agent);
    let carry = 0;
    for (const seg of route.segments) {
      if (seg.to && seg.to.kind === "workstation" && seg.to.mode === "pick") {
        const ready = readyAt.get(bayId(seg.to.bay.r, seg.to.bay.c));
        if (ready != null) carry += Math.max(0, ready - (seg.endSec + carry));
      }
    }
    return route.durationSec + carry;
  };

  const remaining = [...readyAt.keys()];
  const pickCount = visits.map(() => 0);
  while (remaining.length) {
    let best = null;
    for (let agent = 0; agent < visits.length; agent++) {
      if (pickCount[agent] >= state.capacity) continue;
      for (const id of remaining) {
        const bay = parseBayId(id);
        // A pickup appended right after its own drop is NOT skipped: it merges
        // into one in-bay "service" stay below (drop, wait out processing,
        // pick, single exit) — the cheapest option and, crucially, the only
        // one with no mid-grid about-face. Merging happens after the auction so
        // tourCompletion here still scores the raw drop-then-pick geometry
        // (an upper bound; the merge only ever makes the real tour shorter).
        visits[agent].push({ r: bay.r, c: bay.c, mode: "pick" });
        const completion = tourCompletion(agent);
        visits[agent].pop();
        if (!best || completion < best.completion) {
          best = { agent, id, completion };
        }
      }
    }
    if (!best) {
      console.warn(`RPD: ${remaining.length} pickup(s) unassigned — ` +
                   `fleet pickup capacity ${visits.length} x ${state.capacity} exceeded`);
      break;
    }
    const bay = parseBayId(best.id);
    visits[best.agent].push({ r: bay.r, c: bay.c, mode: "pick" });
    pickCount[best.agent]++;
    remaining.splice(remaining.indexOf(best.id), 1);
  }

  // Collapse an adjacent same-bay drop -> pick into one "service" visit so the
  // robot stays inside the dead-end bay through processing instead of exiting,
  // about-facing at the entry node and re-entering (the mid-grid ROTATE_180
  // seen in Alvik1's built-in nearest solve, 2026-07-20). Identical rule to
  // importPlanFromJson; downstream (buildAgentRoute, pickupReadyViolations,
  // route report, capacity check) already understands "service".
  const merged = visits.map((visitList) =>
    visitList.reduce((out, v) => {
      const prev = out[out.length - 1];
      if (prev && v.mode === "pick" && !prev.mode &&
          prev.r === v.r && prev.c === v.c) {
        prev.mode = "service";
      } else {
        out.push({ ...v });
      }
      return out;
    }, []));

  return merged.map((visitList, agent) => buildAgentRoute(
    agent, visitList, 0, meta[agent].manualWaypoints, {}, meta[agent].pathMode));
}

function computeEdgeUse() {
  state.edgeUse = new Map();
  state.totalDistance = 0;
  for (const route of state.routes) {
    state.totalDistance += route.distance;
    for (let i = 1; i < route.path.length; i++) {
      const a = pathKey(route.path[i - 1]);
      const b = pathKey(route.path[i]);
      const key = edgeKey(a, b);
      state.edgeUse.set(key, (state.edgeUse.get(key) || 0) + 1);
    }
  }
}

function intervalsOverlap(a0, a1, b0, b1) {
  return Math.max(a0, b0) < Math.min(a1, b1);
}

function intervalGap(a0, a1, b0, b1) {
  if (intervalsOverlap(a0, a1, b0, b1)) return 0;
  return Math.max(a0, b0) - Math.min(a1, b1);
}

function fmtSec(value) {
  return `${Number(value).toFixed(1)}s`;
}

function detectConflicts(routes = state.routes) {
  const conflicts = [];

  const nodeEvents = [];
  for (const route of routes) {
    for (const ev of route.events || []) {
      nodeEvents.push({ agent: route.agent, node: ev.node, time: ev.arrivalSec, pathIndex: ev.pathIndex || 0 });
    }
  }
  for (let i = 0; i < nodeEvents.length; i++) {
    for (let j = i + 1; j < nodeEvents.length; j++) {
      const a = nodeEvents[i];
      const b = nodeEvents[j];
      if (a.agent === b.agent || String(a.node) !== String(b.node)) continue;
      const gap = Math.abs(a.time - b.time);
      if (gap <= SAFETY_WINDOW_SEC) {
        conflicts.push({
          type: "same node",
          severity: "fatal",
          timeSec: Math.min(a.time, b.time),
          agents: [a.agent, b.agent],
          node: a.node,
          gapSec: gap,
          events: [
            { agent: a.agent, timeSec: a.time, pathIndex: a.pathIndex },
            { agent: b.agent, timeSec: b.time, pathIndex: b.pathIndex },
          ],
          detail: `Node ${a.node}, ${fmtSec(a.time)} vs ${fmtSec(b.time)} within ${fmtSec(SAFETY_WINDOW_SEC)} collision window`,
        });
      }
    }
  }

  const edgeEvents = routes.flatMap((route) =>
    (route.segments || [])
      .filter((seg) => String(seg.fromNode) !== String(seg.toNode))
      .map((seg) => ({
        agent: route.agent,
        from: seg.fromNode,
        to: seg.toNode,
        key: corridorKey(seg.fromNode, seg.toNode),
        startSec: seg.startSec,
        endSec: seg.endSec,
        segment: seg,
      }))
  );

  for (let i = 0; i < edgeEvents.length; i++) {
    for (let j = i + 1; j < edgeEvents.length; j++) {
      const a = edgeEvents[i];
      const b = edgeEvents[j];
      if (a.agent === b.agent || a.key !== b.key) continue;
      const sameDirection = String(a.from) === String(b.from) && String(a.to) === String(b.to);
      const overlap = intervalsOverlap(a.startSec, a.endSec, b.startSec, b.endSec);
      const gap = intervalGap(a.startSec, a.endSec, b.startSec, b.endSec);
      if (overlap) {
        const overlapStartSec = Math.max(a.startSec, b.startSec);
        const overlapEndSec = Math.min(a.endSec, b.endSec);
        conflicts.push({
          type: sameDirection ? "same edge" : "opposite edge",
          severity: "fatal",
          timeSec: overlapStartSec,
          agents: [a.agent, b.agent],
          edgeKey: a.key,
          overlapStartSec,
          overlapEndSec,
          intervals: [
            { agent: a.agent, from: a.from, to: a.to, startSec: a.startSec, endSec: a.endSec, segment: a.segment },
            { agent: b.agent, from: b.from, to: b.to, startSec: b.startSec, endSec: b.endSec, segment: b.segment },
          ],
          detail: `${a.key.replace("<>", " <-> ")} overlap ${fmtSec(overlapStartSec)}-${fmtSec(overlapEndSec)}`,
        });
      } else if (gap <= EDGE_SAFETY_WINDOW_SEC) {
        conflicts.push({
          type: sameDirection ? "same edge contention" : "opposite edge contention",
          severity: "fatal",
          timeSec: Math.min(a.endSec, b.endSec),
          agents: [a.agent, b.agent],
          edgeKey: a.key,
          gapSec: gap,
          intervals: [
            { agent: a.agent, from: a.from, to: a.to, startSec: a.startSec, endSec: a.endSec, segment: a.segment },
            { agent: b.agent, from: b.from, to: b.to, startSec: b.startSec, endSec: b.endSec, segment: b.segment },
          ],
          detail: `${a.key.replace("<>", " <-> ")} gap ${fmtSec(gap)} inside ${fmtSec(EDGE_SAFETY_WINDOW_SEC)} edge collision window`,
        });
      }
    }
  }
  return conflicts.sort((a, b) => (a.timeSec || 0) - (b.timeSec || 0));
}

function isEdgeConflict(conflict) {
  return Boolean(conflict.edgeKey);
}

function fatalEdgeConflicts(conflicts) {
  return conflicts.filter((conflict) => conflict.severity === "fatal" && isEdgeConflict(conflict));
}

function fatalSchedulingConflicts(conflicts) {
  return conflicts.filter((conflict) =>
    conflict.severity === "fatal" &&
    (isEdgeConflict(conflict) || conflict.type === "same node")
  );
}

function scheduleStatus(routes, conflicts, iterations = 0, capped = false) {
  const fatalEdges = fatalEdgeConflicts(conflicts);
  const fatalNodes = conflicts.filter((conflict) => conflict.severity === "fatal" && !isEdgeConflict(conflict));
  return {
    collisionFree: fatalEdges.length === 0 && fatalNodes.length === 0,
    edgeExclusive: fatalEdges.length === 0,
    iterations,
    capped,
    fatalEdgeRemaining: fatalEdges.length,
    fatalNodeRemaining: fatalNodes.length,
    nearMisses: conflicts.filter((conflict) => conflict.severity !== "fatal").length,
    extraWaitByAgent: routes.map((route) => ({
      agent: route.agent,
      seconds: route.extraWaitSec || 0,
      dwellSeconds: totalRouteDwell(route),
    })),
  };
}

function refreshScheduleStatus() {
  if (!state.schedule) return;
  const status = scheduleStatus(
    state.routes,
    state.conflicts,
    state.schedule.iterations || 0,
    Boolean(state.schedule.capped)
  );
  state.schedule = status;
}

function buildRoutesWithSchedule(templateRoutes, extraWaits, waitsByAgent) {
  return applyLaunchSchedule(templateRoutes.map((route) => {
    const built = buildAgentRoute(
      route.agent,
      cloneBays(route.bays),
      extraWaits[route.agent] || 0,
      cloneWaypoints(route.manualWaypoints || manualWaypointsFor(route.agent)),
      cloneWaits(waitsByAgent[route.agent] || route.waitBeforeSec || {}),
      route.pathMode || "vh"
    );
    if (route.pinnedOrder) built.pinnedOrder = true;
    return built;
  }));
}

function launchRank(agent, routeCount) {
  const order = launchOrderForAgents(routeCount);
  const rank = order.indexOf(agent);
  return rank >= 0 ? rank : order.length + agent;
}

function delayPlanForEdgeConflict(conflict, routes) {
  const intervals = conflict.intervals || [];
  if (intervals.length < 2) {
    return { agent: typeof conflict.agents[1] === "number" ? conflict.agents[1] : conflict.agents[0], seconds: 1 };
  }

  const [a, b] = intervals;
  let delayed = b;
  let blocker = a;
  if (b.startSec < a.startSec) {
    delayed = a;
    blocker = b;
  } else if (Math.abs(a.startSec - b.startSec) < 0.001) {
    const routeA = routes.find((route) => route.agent === a.agent);
    const routeB = routes.find((route) => route.agent === b.agent);
    const launchA = routeA ? routeA.launchOffsetSec || 0 : 0;
    const launchB = routeB ? routeB.launchOffsetSec || 0 : 0;
    const rankA = launchRank(a.agent, routes.length);
    const rankB = launchRank(b.agent, routes.length);
    if (launchA > launchB || (Math.abs(launchA - launchB) < 0.001 && rankA > rankB)) {
      delayed = a;
      blocker = b;
    }
  }

  let seconds = blocker.endSec + EDGE_SAFETY_WINDOW_SEC + EDGE_RELEASE_BUFFER_SEC - delayed.startSec;
  if (!Number.isFinite(seconds) || seconds <= 0) seconds = 1;
  return {
    agent: delayed.agent,
    waitIndex: delayed.segment && delayed.segment.segmentIndex,
    seconds: Math.max(EDGE_RELEASE_BUFFER_SEC, Math.ceil(seconds * 10) / 10),
  };
}

function delayPlanForNodeConflict(conflict, routes) {
  const events = conflict.events || [];
  if (events.length < 2) {
    return { agent: typeof conflict.agents[1] === "number" ? conflict.agents[1] : conflict.agents[0], seconds: 1 };
  }
  const [a, b] = events;
  let delayed = b;
  let blocker = a;
  if (b.timeSec < a.timeSec) {
    delayed = a;
    blocker = b;
  } else if (Math.abs(a.timeSec - b.timeSec) < 0.001) {
    const rankA = launchRank(a.agent, routes.length);
    const rankB = launchRank(b.agent, routes.length);
    if (rankA > rankB) {
      delayed = a;
      blocker = b;
    }
  }
  let seconds = blocker.timeSec + SAFETY_WINDOW_SEC + EDGE_RELEASE_BUFFER_SEC - delayed.timeSec;
  if (!Number.isFinite(seconds) || seconds <= 0) seconds = 1;
  return {
    agent: delayed.agent,
    waitIndex: delayed.pathIndex || 0,
    seconds: Math.max(EDGE_RELEASE_BUFFER_SEC, Math.ceil(seconds * 10) / 10),
  };
}

function delayPlanForConflict(conflict, routes) {
  return isEdgeConflict(conflict)
    ? delayPlanForEdgeConflict(conflict, routes)
    : delayPlanForNodeConflict(conflict, routes);
}

// VRP-RPD hard constraint: a robot may not leave a pickup workstation before
// the part is ready (drop-off arrival + state.processSec). Violations are
// fixed like conflicts — add a dwell on the workstation->exit segment, i.e.
// the robot waits INSIDE the bay (a dead end, so it blocks nobody) — and the
// schedule is rebuilt until the constraint holds.
function pickupReadyViolations(routes) {
  if (!state.rpdMode) return [];
  const readyAt = new Map();
  for (const route of routes) {
    for (const seg of route.segments || []) {
      if (seg.to && seg.to.kind === "workstation" && seg.to.mode !== "pick") {
        readyAt.set(bayId(seg.to.bay.r, seg.to.bay.c), seg.endSec + state.processSec);
      }
    }
  }
  const violations = [];
  for (const route of routes) {
    const segs = route.segments || [];
    for (let i = 0; i < segs.length; i++) {
      const seg = segs[i];
      // "service" = merged drop+pick in one stay: the drop happens on
      // arrival (readyAt above) and the departure must wait for readiness,
      // exactly like a pick.
      if (!(seg.to && seg.to.kind === "workstation" &&
            (seg.to.mode === "pick" || seg.to.mode === "service"))) continue;
      const ready = readyAt.get(bayId(seg.to.bay.r, seg.to.bay.c));
      if (ready == null) continue;
      const departSec = i + 1 < segs.length ? segs[i + 1].startSec : seg.endSec;
      if (departSec < ready - 0.001) {
        violations.push({
          agent: route.agent,
          waitIndex: i + 1 < segs.length ? segs[i + 1].segmentIndex : seg.segmentIndex,
          seconds: Math.ceil((ready - departSec) * 10) / 10,
        });
      }
    }
  }
  return violations;
}

function edgeExclusiveSchedule(templateRoutes) {
  const extraWaits = [];
  const waitsByAgent = [];
  for (const route of templateRoutes) {
    extraWaits[route.agent] = route.extraWaitSec || 0;
    waitsByAgent[route.agent] = cloneWaits(route.waitBeforeSec || {});
  }

  let routes = buildRoutesWithSchedule(templateRoutes, extraWaits, waitsByAgent);
  let conflicts = detectConflicts(routes);
  let iterations = 0;
  let fatalConflicts = fatalSchedulingConflicts(conflicts);
  let readyViolations = pickupReadyViolations(routes);

  while ((fatalConflicts.length || readyViolations.length) &&
         iterations < AUTO_DECONFLICT_MAX_ITERATIONS) {
    if (readyViolations.length) {
      // Part-ready waits first: they are hard constraints, and the dwells
      // they insert shift timings that conflict fixes must then respect.
      const v = readyViolations[0];
      waitsByAgent[v.agent] = waitsByAgent[v.agent] || {};
      waitsByAgent[v.agent][v.waitIndex] = (waitsByAgent[v.agent][v.waitIndex] || 0) + v.seconds;
    } else {
      const plan = delayPlanForConflict(fatalConflicts[0], routes);
      const waitIndex = Number(plan.waitIndex) || 0;
      if (waitIndex > 0) {
        waitsByAgent[plan.agent] = waitsByAgent[plan.agent] || {};
        waitsByAgent[plan.agent][waitIndex] = (waitsByAgent[plan.agent][waitIndex] || 0) + plan.seconds;
      } else {
        extraWaits[plan.agent] = (extraWaits[plan.agent] || 0) + plan.seconds;
      }
    }
    routes = buildRoutesWithSchedule(routes, extraWaits, waitsByAgent);
    conflicts = detectConflicts(routes);
    fatalConflicts = fatalSchedulingConflicts(conflicts);
    readyViolations = pickupReadyViolations(routes);
    iterations++;
  }

  return {
    routes,
    conflicts,
    schedule: scheduleStatus(routes, conflicts, iterations,
                             fatalConflicts.length > 0 || readyViolations.length > 0),
  };
}

function conflictScore(conflicts) {
  return conflicts.reduce((score, conflict) => {
    if (conflict.severity === "fatal" && isEdgeConflict(conflict)) return score + 100000;
    if (conflict.severity === "fatal") return score + 90000;
    if (isEdgeConflict(conflict)) return score + 850;
    return score + 150;
  }, 0);
}

// A mid-grid ROTATE_180 (the robot about-facing somewhere on the open
// lattice, NOT at a workstation/depot) means the route drove past its own
// path and doubled back -- the exact defect buildAgentRoute's bay-side
// scoring (see its own comment, "Alvik3 drove past its workstation,
// ROTATE_180'd... and came back — hardware run 2026-07-14") exists to
// prevent. ROOT-CAUSED 2026-08-06: buildAgentRoute's OWN output was
// confirmed clean via direct inspection (dumped real fullPath arrays) --
// the defect is introduced by polishCollisionPlan (below) trying
// alternate PATH_MODES ("hv"/"via-left"/"via-right"/etc, see PATH_MODES
// above) to resolve a scheduling conflict. Those alternate modes can
// produce a route that resolves the conflict but backtracks through a
// node it already visited -- and until now, NOTHING checked for that:
// evaluateRoutes()'s `fatal` count (which isBetterPlanEval() prioritizes
// above all other scoring) only ever counted scheduling conflicts, so a
// flip-introducing candidate could still "win" over a flip-free one as
// long as it looked better on collision/timing grounds alone. The ONLY
// place a flip was ever detected was generateCommandLines() -- a UI-
// render-time-only warning that never fed back into route SELECTION at
// all. Fixed by counting flips here too, in the same `fatal` metric
// polishCollisionPlan already treats as a hard must-avoid, so a flip-
// introducing candidate can never be chosen over a flip-free one with
// equal/better scheduling quality.
function countMidGridFlips(route) {
  // Must mirror generateCommandLines()'s own flip-exemption logic exactly
  // (arrivingAtWork AND leavingWork -- see that function's 2026-08-24
  // comment on leavingWork) or a route could score as flip-free here while
  // still producing a real mid-grid-flip warning at dispatch time, or the
  // reverse.
  const segs = route.segments || [];
  let flips = 0;
  let prevHeading = "N";
  for (const seg of segs) {
    if (seg.type === "RED_TO_BLUE_RETURN") break;
    const arrivingAtWork = seg.to && seg.to.kind === "workstation";
    const leavingWork = seg.from && seg.from.kind === "workstation";
    const turn = turnCommandBetween(prevHeading, seg.heading);
    if (turn === "ROTATE_180" && !arrivingAtWork && !leavingWork) flips++;
    prevHeading = arrivingAtWork ? "S" : seg.heading;
  }
  return flips;
}

function evaluateRoutes(routes) {
  const conflicts = detectConflicts(routes);
  const midGridFlips = routes.reduce((sum, route) => sum + countMidGridFlips(route), 0);
  return {
    conflicts,
    score: conflictScore(conflicts),
    makespan: makespan(routes),
    dwell: totalPlanDwell(routes),
    midGridFlips,
    fatal: fatalSchedulingConflicts(conflicts).length + midGridFlips,
  };
}

function nowMs() {
  return window.performance && performance.now ? performance.now() : Date.now();
}

function cleanRouteTemplate(route, overrides = {}) {
  const has = (key) => Object.prototype.hasOwnProperty.call(overrides, key);
  const built = buildAgentRoute(
    route.agent,
    has("bays") ? overrides.bays : route.bays,
    has("extraWaitSec") ? overrides.extraWaitSec : 0,
    has("manualWaypoints") ? overrides.manualWaypoints : route.manualWaypoints || manualWaypointsFor(route.agent),
    has("waitBeforeSec") ? overrides.waitBeforeSec : {},
    has("pathMode") ? overrides.pathMode : route.pathMode || "vh"
  );
  if (route.pinnedOrder) built.pinnedOrder = true;
  return built;
}

function cleanRouteTemplates(routes) {
  return routes.map((route) => cleanRouteTemplate(route));
}

function routeScheduleResult(templateRoutes) {
  const scheduled = edgeExclusiveSchedule(cleanRouteTemplates(templateRoutes));
  return {
    routes: scheduled.routes,
    conflicts: scheduled.conflicts,
    schedule: scheduled.schedule,
    eval: evaluateRoutes(scheduled.routes),
  };
}

function isBetterPlanEval(candidate, best) {
  if (candidate.eval.fatal !== best.eval.fatal) return candidate.eval.fatal < best.eval.fatal;
  if (Math.abs(candidate.eval.makespan - best.eval.makespan) > 0.05) {
    return candidate.eval.makespan < best.eval.makespan;
  }
  if (Math.abs(candidate.eval.dwell - best.eval.dwell) > 0.05) {
    return candidate.eval.dwell < best.eval.dwell;
  }
  return candidate.eval.score < best.eval.score;
}

function conflictFocusAgents(conflicts, routes, limit = 2) {
  const counts = new Map();
  for (const conflict of fatalSchedulingConflicts(conflicts)) {
    for (const agent of conflict.agents || []) counts.set(agent, (counts.get(agent) || 0) + 1);
  }
  let agents = [...counts.entries()]
    .sort((a, b) => b[1] - a[1])
    .map(([agent]) => agent);

  if (!agents.length) {
    agents = [...routes]
      .sort((a, b) => (totalRouteDwell(b) + (b.extraWaitSec || 0)) - (totalRouteDwell(a) + (a.extraWaitSec || 0)))
      .map((route) => route.agent);
  }

  return agents.slice(0, limit);
}

function routeReplacementSet(routes, replacement) {
  return routes.map((route) => route.agent === replacement.agent ? replacement : cleanRouteTemplate(route));
}

function polishCollisionPlan(templateRoutes, budgetMs = SOLVE_POLISH_BUDGET_MS, forcedAgent = null, includeOrders = false) {
  const start = nowMs();
  let best = routeScheduleResult(templateRoutes);
  const focusAgents = forcedAgent === null
    ? conflictFocusAgents(best.conflicts, best.routes, 2)
    : [forcedAgent];

  for (const agent of focusAgents) {
    const current = best.routes.find((route) => route.agent === agent) ||
      templateRoutes.find((route) => route.agent === agent);
    if (!current) continue;

    const modeCandidates = PATH_MODES.filter((mode) => mode !== (current.pathMode || "vh"));
    const orderCandidates = includeOrders ? candidateOrders(current).slice(0, 4) : [cloneBays(current.bays)];

    for (const order of orderCandidates) {
      for (const pathMode of modeCandidates) {
        if (nowMs() - start > budgetMs) {
          return best;
        }
        const replacement = cleanRouteTemplate(current, {
          bays: order,
          pathMode,
          extraWaitSec: 0,
          waitBeforeSec: {},
        });
        const candidate = routeScheduleResult(routeReplacementSet(best.routes, replacement));
        if (isBetterPlanEval(candidate, best)) best = candidate;
      }
    }
  }

  return best;
}

// ---- workstation lifecycle badges ------------------------------------------
// Per-bay part state, read off the planned schedule: awaiting delivery ->
// processing (state.processSec after the drop) -> awaiting pickup -> complete.
// Without RPD there is no processing/pickup phase: delivered = complete.
// The clock is the sim playback time; in Real mode it is wall time since
// START was pressed on this page (schedule-projected, not robot-confirmed).
function bayLifecycleTimes() {
  const times = new Map();
  for (const route of state.routes) {
    const segs = route.segments || [];
    for (let i = 0; i < segs.length; i++) {
      const seg = segs[i];
      if (!(seg.to && seg.to.kind === "workstation")) continue;
      const id = bayId(seg.to.bay.r, seg.to.bay.c);
      const info = times.get(id) || {};
      if (seg.to.mode !== "pick") {
        info.dropSec = seg.endSec;
      }
      if (seg.to.mode === "pick" || seg.to.mode === "service") {
        // picked once the robot leaves the workstation (waits included)
        info.pickupSec = i + 1 < segs.length ? segs[i + 1].startSec : seg.endSec;
      }
      times.set(id, info);
    }
  }
  return times;
}

// Timeline event categories shown as flags on the big timeline strip.
// Each toggle key maps to a checkbox id (tl-toggle-<key>) and a swatch color.
const TIMELINE_EVENT_TYPES = {
  dropoff:      { label: "Drop-off",     shape: "flag-down", color: "#f97316" },
  pickup:       { label: "Pickup",       shape: "flag-up",   color: "#22c55e" },
  depotLeave:   { label: "Leaves depot", shape: "triangle",  color: "#2563eb" },
  depotReturn:  { label: "Returns to depot", shape: "square", color: "#0f172a" },
};

// Flat, sorted list of {sec, agent, type, label} events across all current
// routes -- drop-off / pickup (from route.segments' workstation legs, same
// source bayLifecycleTimes() uses) plus depot-leave / depot-return (from the
// route's special node0 / depot-slot path points). Built fresh from
// state.routes each time it's called; cheap enough to call on every render.
function buildTimelineEvents() {
  const events = [];
  for (const route of state.routes) {
    const segs = route.segments || [];
    const agentLabel = `A${route.agent + 1}`;
    for (let i = 0; i < segs.length; i++) {
      const seg = segs[i];
      const toPoint = seg.to;
      if (toPoint && toPoint.kind === "workstation") {
        const wsNum = workstationNodeNumber(toPoint.bay);
        if (toPoint.mode !== "pick") {
          events.push({
            sec: seg.endSec, agent: route.agent, type: "dropoff",
            label: `${agentLabel} drop-off at WS${wsNum}`,
          });
        }
        if (toPoint.mode === "pick" || toPoint.mode === "service") {
          const pickSec = i + 1 < segs.length ? segs[i + 1].startSec : seg.endSec;
          events.push({
            sec: pickSec, agent: route.agent, type: "pickup",
            label: `${agentLabel} pickup at WS${wsNum}`,
          });
        }
      }
      const fromPoint = seg.from;
      if (toPoint && toPoint.kind === "special" && toPoint.id === "node0" &&
          fromPoint && fromPoint.kind === "special" &&
          String(fromPoint.id || "").startsWith("depot-entry-")) {
        // node0 reached FROM a depot entry -> just left the depot
        events.push({
          sec: seg.endSec, agent: route.agent, type: "depotLeave",
          label: `${agentLabel} leaves depot`,
        });
      }
      if (toPoint && toPoint.kind === "special" && String(toPoint.id).startsWith("depot-slot-")) {
        events.push({
          sec: seg.endSec, agent: route.agent, type: "depotReturn",
          label: `${agentLabel} returns to depot`,
        });
      }
    }
  }
  events.sort((a, b) => a.sec - b.sec);
  return events;
}

let timelineTogglesBuilt = false;

function renderTimelineToggles() {
  const box = document.getElementById("timeline-toggles");
  if (!box) return;
  if (!timelineTogglesBuilt) {
    box.innerHTML = Object.entries(TIMELINE_EVENT_TYPES).map(([key, def]) => `
      <label>
        <input type="checkbox" id="tl-toggle-${key}" ${state.timelineToggles[key] ? "checked" : ""}>
        <i class="tl-swatch" style="background:${def.color};"></i>${escapeHtml(def.label)}
      </label>
    `).join("");
    for (const key of Object.keys(TIMELINE_EVENT_TYPES)) {
      document.getElementById(`tl-toggle-${key}`).addEventListener("change", (event) => {
        state.timelineToggles[key] = event.target.checked;
        renderEventTimeline();
      });
    }
    timelineTogglesBuilt = true;
  }
}

// Small marker glyphs per event category, drawn centered at (x, y) with the
// given color. Kept intentionally simple (single closed path each) so they
// stay legible at the ~10px size used on a dense timeline.
function timelineMarkerPath(shape, x, y, s) {
  if (shape === "flag-down") {
    // pin pointing down (drop-off)
    return `M ${x} ${y + s} L ${x - s} ${y - s} L ${x + s} ${y - s} Z`;
  }
  if (shape === "flag-up") {
    // pin pointing up (pickup)
    return `M ${x} ${y - s} L ${x - s} ${y + s} L ${x + s} ${y + s} Z`;
  }
  if (shape === "triangle") {
    return `M ${x} ${y - s} L ${x + s} ${y + s} L ${x - s} ${y + s} Z`;
  }
  // square (depot return)
  return `M ${x - s} ${y - s} L ${x + s} ${y - s} L ${x + s} ${y + s} L ${x - s} ${y + s} Z`;
}

function renderEventTimeline() {
  const svg = document.getElementById("event-timeline");
  if (!svg) return;
  svg.innerHTML = "";
  const maxT = makespan();
  const VB_W = 1000, VB_H = 150;
  const marginL = 12, marginR = 12;
  const trackY = 110;
  const laneTop = 14, laneBottom = 98;

  svg.appendChild(el("line", {
    x1: marginL, y1: trackY, x2: VB_W - marginR, y2: trackY,
    stroke: "#cbd5e1", "stroke-width": 12, "stroke-linecap": "round",
  }));

  if (!state.routes.length || maxT <= 0) {
    svg.appendChild(el("text", {
      x: VB_W / 2, y: trackY - 6, "text-anchor": "middle", fill: "#94a3b8", "font-size": 11,
    }, "Solve to see drop-off, pickup, and depot events here."));
    return;
  }

  const xFor = (sec) => marginL + (Math.max(0, Math.min(maxT, sec)) / maxT) * (VB_W - marginL - marginR);

  // tick labels at 0 / 25% / 50% / 75% / 100% of makespan
  for (let f = 0; f <= 1.0001; f += 0.25) {
    const sec = maxT * f;
    const x = xFor(sec);
    svg.appendChild(el("line", { x1: x, y1: trackY - 4, x2: x, y2: trackY + 4, stroke: "#94a3b8", "stroke-width": 1 }));
    svg.appendChild(el("text", { x, y: trackY + 16, "text-anchor": "middle", fill: "#94a3b8", "font-size": 9 }, fmtSec(sec)));
  }

  const events = buildTimelineEvents().filter((e) => state.timelineToggles[e.type]);

  // Stack same-ish-time markers into lanes above the track so nearby flags
  // don't fully overlap: bucket by rounded pixel x, alternate up/down offset.
  const bucketed = new Map();
  for (const ev of events) {
    const bx = Math.round(xFor(ev.sec) / 6);
    const arr = bucketed.get(bx) || [];
    arr.push(ev);
    bucketed.set(bx, arr);
  }

  const tooltip = document.getElementById("timeline-tooltip");
  const svgRect = { current: null };

  for (const [, arr] of bucketed) {
    arr.forEach((ev, idx) => {
      const def = TIMELINE_EVENT_TYPES[ev.type];
      const x = xFor(ev.sec);
      const y = laneTop + (idx % 4) * ((laneBottom - laneTop) / 3.2);
      const color = def.color;
      const path = el("path", {
        d: timelineMarkerPath(def.shape, x, y, 7),
        fill: color,
        stroke: "#ffffff",
        "stroke-width": 1,
        style: "cursor:pointer",
      });
      path.addEventListener("pointerdown", (event) => {
        // Snap straight to this event's exact time rather than falling
        // through to the track's drag-seek (which would use the pointer's
        // raw x position instead of the marker's true timestamp).
        event.stopPropagation();
        setTimestep(ev.sec);
      });
      path.addEventListener("mouseenter", () => {
        if (!tooltip) return;
        tooltip.textContent = `${ev.label} — ${fmtSec(ev.sec)}`;
        tooltip.style.display = "block";
        const wrap = svg.parentElement.getBoundingClientRect();
        const ratio = wrap.width / VB_W;
        tooltip.style.left = `${x * ratio}px`;
        tooltip.style.top = `${(y) * (wrap.height / VB_H)}px`;
      });
      path.addEventListener("mouseleave", () => {
        if (tooltip) tooltip.style.display = "none";
      });
      svg.appendChild(path);
    });
  }

  // current-time cursor, drawn last so it's on top
  const cursorX = xFor(state.timestep);
  svg.appendChild(el("line", {
    x1: cursorX, y1: 4, x2: cursorX, y2: trackY + 20,
    stroke: "#0f172a", "stroke-width": 2,
  }));
  svg.appendChild(el("circle", { cx: cursorX, cy: trackY, r: 5, fill: "#0f172a", stroke: "#fff", "stroke-width": 1.5 }));

  // Click OR drag anywhere on the track to seek. pointerdown starts a drag
  // (setPointerCapture so it keeps tracking even if the cursor leaves the
  // strip mid-drag), pointermove while captured updates the time live,
  // pointerup ends it. A plain click (no movement) still works the same way
  // since pointerdown alone already seeks once.
  const trackHit = el("rect", {
    x: marginL, y: 0, width: VB_W - marginL - marginR, height: VB_H,
    fill: "transparent", style: "cursor:pointer",
  });
  const seekFromClientX = (clientX) => {
    const rect = svg.getBoundingClientRect();
    const relX = ((clientX - rect.left) / rect.width) * VB_W;
    const frac = Math.max(0, Math.min(1, (relX - marginL) / (VB_W - marginL - marginR)));
    setTimestep(frac * maxT);
  };
  trackHit.addEventListener("pointerdown", (event) => {
    trackHit.setPointerCapture(event.pointerId);
    seekFromClientX(event.clientX);
  });
  trackHit.addEventListener("pointermove", (event) => {
    if (event.buttons !== 1) return;
    seekFromClientX(event.clientX);
  });
  svg.insertBefore(trackHit, svg.firstChild);
}

// Per-status colors for the bay BOX itself (fills the whole cell, not just
// the small badge): awaiting delivery = orange, processing = blue,
// awaiting pickup = light green, complete = dark green. Badge fill/stroke/
// text stay higher-contrast for the small label text.
const BAY_STATUS_STYLE = {
  awaiting_delivery: { label: "AWAIT", fill: "#fed7aa", stroke: "#ea580c", text: "#9a3412", boxFill: "#ffedd5", boxStroke: "#f97316" },
  processing:        { label: "PROC",  fill: "#bfdbfe", stroke: "#2563eb", text: "#1e3a8a", boxFill: "#dbeafe", boxStroke: "#3b82f6" },
  awaiting_pickup:   { label: "READY", fill: "#bbf7d0", stroke: "#22c55e", text: "#166534", boxFill: "#dcfce7", boxStroke: "#4ade80" },
  complete:          { label: "DONE",  fill: "#86efac", stroke: "#15803d", text: "#052e16", boxFill: "#bbf7d0", boxStroke: "#15803d" },
};

function bayStatusAt(info, tSec) {
  if (!info || info.dropSec == null || tSec < info.dropSec) return "awaiting_delivery";
  if (!state.rpdMode) return "complete";
  if (tSec < info.dropSec + state.processSec) return "processing";
  if (info.pickupSec == null || tSec < info.pickupSec) return "awaiting_pickup";
  return "complete";
}

function statusClockSec() {
  if (liveDataMode() && state.missionStartMs) {
    return (Date.now() - state.missionStartMs) / 1000;
  }
  return state.timestep;
}

function drawGrid() {
  svg.innerHTML = "";
  const g = layout();
  svg.setAttribute("viewBox", `0 0 ${g.w} ${g.h}`);

  svg.appendChild(el("rect", { x: 0, y: 0, width: g.w, height: g.h, fill: "#f8fafc" }));

  // Depot drawn in the SAME vision-grid frame as the live robot dots, so they
  // line up. node0 = lane junction; the lane runs from node 1 (grid bottom-left)
  // south to node 0 then east under the parking slots — the actual robot path.
  const node0Pt = visionGridToPoint(DEPOT_NODE0_GRID.x, DEPOT_NODE0_GRID.y);
  const node1Pt = visionGridToPoint(0, 0);
  const laneEastPt = visionGridToPoint(
    depotLaneEastGrid(state.agents), DEPOT_NODE0_GRID.y);
  svg.appendChild(el("text", { x: node0Pt.x + 18, y: node0Pt.y + 26, fill: "#334155", "font-size": 12, "font-weight": 700 }, "Depot lane south of grid"));
  // horizontal lane at node-0 level
  svg.appendChild(el("line", {
    x1: node0Pt.x, y1: node0Pt.y, x2: laneEastPt.x, y2: laneEastPt.y,
    stroke: "#93a4b8", "stroke-width": 8, "stroke-linecap": "round"
  }));
  // node 1 -> node 0 connector (the leg robots drive on exit/return)
  svg.appendChild(el("line", {
    x1: node1Pt.x, y1: node1Pt.y, x2: node0Pt.x, y2: node0Pt.y,
    stroke: "#6b7280", "stroke-width": 8, "stroke-linecap": "round"
  }));
  svg.appendChild(el("circle", { cx: node0Pt.x, cy: node0Pt.y, r: 12, fill: "#111827", stroke: "#ffffff", "stroke-width": 2 }));
  svg.appendChild(el("text", { x: node0Pt.x, y: node0Pt.y + 4, "text-anchor": "middle", fill: "#fff", "font-size": 11, "font-weight": 700 }, "0"));

  const laneY = visionGridToPoint(0, DEPOT_LANE_GRID_Y).y;
  // Agents currently away from the depot (out on a route, real or simulated)
  // must not also get a static "parked here" box drawn at the same time —
  // that produced a robot rendered twice at once (live marker on the grid
  // AND its depot box, out of sync with each other, e.g. Alvik3 mid-route
  // 2026-07-23). The two draws share no state, so compare actual pixel
  // position against the depot slot's position directly rather than trying
  // to track a separate "has launched" flag that could itself drift.
  //
  // RAISED 2026-07-30 from a fixed 15px to a fraction of the grid cell
  // size: confirmed on real hardware that a robot sitting correctly
  // parked in its depot slot (Alvik1, visually identical placement to
  // Alvik2/Alvik3, which both rendered fine) still exceeded a flat 15px
  // threshold and had its depot box incorrectly hidden -- normal
  // vision-tracking noise/parking variance is a real physical distance,
  // not a fixed pixel count, so a flat pixel threshold is either too
  // tight or too loose depending on how large the grid happens to be
  // rendered (window size, --rows/--cols) at the time. 35% of one cell
  // scales with the actual rendered size and gives real margin (roughly
  // 3-4in of real-world tolerance at typical grid sizes) while still
  // catching a robot that's genuinely mid-route, which moves by whole
  // cells, not inches.
  const DEPOT_MARKER_MATCH_PX = g.cell * 0.35;
  for (let i = 0; i < state.agents; i++) {
    const route = state.routes.find((r) => r.agent === i);
    if (route) {
      const isReal = liveDataMode();
      const p = isReal
        ? realPointForAgent(i) || pointAtSeconds(route, Math.min(state.timestep, makespan()))
        : pointAtSeconds(route, Math.min(state.timestep, makespan()));
      const slotNow = depotSlot(i);
      const awayFromDepot = p && Math.hypot(p.x - slotNow.x, p.y - slotNow.y) > DEPOT_MARKER_MATCH_PX;
      if (awayFromDepot) continue;
    }
    // ONE coordinate system: the box, its DE point (straight below on the
    // lane), and the route line all use depotSlot(i). The DE point shares the
    // box x, so the slot->lane leg is vertical. No live-vs-sim snapping — the
    // depot geometry is the real geometry in both modes (see DEPOT_* constants,
    // which are set from the robots' measured parking coordinates).
    const slot = depotSlot(i);
    svg.appendChild(el("circle", { cx: slot.x, cy: laneY, r: 6, fill: "#dbeafe", stroke: "#2563eb", "stroke-width": 2 }));
    svg.appendChild(el("text", { x: slot.x, y: laneY + 16, "text-anchor": "middle", fill: "#1d4ed8", "font-size": 10, "font-weight": 700 }, `DE${i + 1}`));
    svg.appendChild(el("rect", { x: slot.x - 20, y: slot.y - 16, width: 40, height: 32, rx: 5, fill: "#dbeafe", stroke: "#2563eb", "stroke-width": 2 }));
    svg.appendChild(el("text", { x: slot.x, y: slot.y + 4, "text-anchor": "middle", fill: "#1d4ed8", "font-size": 12, "font-weight": 700 }, `D${i + 1}`));
  }

  // Status-driven bay box coloring needs each bay's lifecycle status, so
  // compute it here (before the box loop) rather than later in drawGrid.
  // A launch replay is not a mission. The bays have no parts moving and
  // no plan is executing, so their AWAIT/PROC/DONE lifecycle must not be
  // driven off the replay clock as though one were.
  const bayTimesForBoxes = (state.routes.length && !replayActive())
    ? bayLifecycleTimes() : null;
  const statusClockForBoxes = statusClockSec();

  for (let r = 0; r < state.rows - 1; r++) {
    for (let c = 0; c < state.cols - 1; c++) {
      const p = pt(r, c);
      const id = bayId(r, c);
      const selected = state.selected.has(id);
      let fill = selected ? "#dcfce7" : "#ffffff";
      let stroke = selected ? "#22c55e" : "#d9e2ee";
      let textFill = selected ? "#166534" : "#94a3b8";
      if (selected && bayTimesForBoxes) {
        const style = BAY_STATUS_STYLE[bayStatusAt(bayTimesForBoxes.get(id), statusClockForBoxes)];
        fill = style.boxFill;
        stroke = style.boxStroke;
        textFill = style.text;
      }
      const rect = el("rect", {
        x: p.x + 6, y: p.y + 6, width: g.cell - 12, height: g.cell - 12,
        fill, stroke,
        "stroke-width": selected ? 2 : 1,
        rx: 4,
        "data-bay": id,
        style: "cursor:pointer"
      });
      rect.addEventListener("click", () => {
        if (!state.editMode) toggleBay(id);
      });
      svg.appendChild(rect);
      svg.appendChild(el("text", { x: p.x + g.cell / 2, y: p.y + g.cell / 2 + 4, "text-anchor": "middle", fill: textFill, "font-size": 11 }, `B${bayNumber(r, c)}`));
    }
  }

  for (let r = 0; r < state.rows; r++) {
    const a = pt(r, 0);
    const b = pt(r, state.cols - 1);
    svg.appendChild(el("line", { x1: a.x, y1: a.y, x2: b.x, y2: b.y, stroke: "#8a98aa", "stroke-width": 5, "stroke-linecap": "round" }));
  }
  for (let c = 0; c < state.cols; c++) {
    const a = pt(0, c);
    const b = pt(state.rows - 1, c);
    svg.appendChild(el("line", { x1: a.x, y1: a.y, x2: b.x, y2: b.y, stroke: "#8a98aa", "stroke-width": 5, "stroke-linecap": "round" }));
  }

  drawHeat();
  // Planned route polylines belong to the imported plan, not to the
  // recorded launch; drawing both at once implies the robots are
  // following the plan, which during a replay they are not.
  if (!replayActive()) drawRoutes();

  const bayTimes = bayTimesForBoxes;
  const statusClock = statusClockForBoxes;
  for (const id of state.selected) {
    const bay = parseBayId(id);
    const entry = bayEntryPoint(bay);
    const work = bayWorkPoint(bay);
    svg.appendChild(el("line", { x1: entry.x, y1: entry.y, x2: work.x, y2: work.y, stroke: "#16a34a", "stroke-width": 3, "stroke-linecap": "round" }));
    svg.appendChild(el("circle", { cx: entry.x, cy: entry.y, r: 6, fill: "#facc15", stroke: "#a16207", "stroke-width": 1.5 }));
    svg.appendChild(el("text", { x: entry.x, y: entry.y - 9, "text-anchor": "middle", fill: "#854d0e", "font-size": 9, "font-weight": 700 }, entryNodeNumber(bay)));
    svg.appendChild(el("circle", { cx: work.x, cy: work.y, r: 8, fill: "#facc15", stroke: "#a16207", "stroke-width": 1.5 }));
    svg.appendChild(el("text", { x: work.x, y: work.y + 3, "text-anchor": "middle", fill: "#854d0e", "font-size": 9, "font-weight": 700 }, workstationNodeNumber(bay)));
    if (bayTimes) {
      // tiny part-status badge along the bay cell's bottom edge
      const style = BAY_STATUS_STYLE[bayStatusAt(bayTimes.get(id), statusClock)];
      const p = pt(bay.r, bay.c);
      const cx = p.x + g.cell / 2;
      const by = p.y + g.cell - 20;
      svg.appendChild(el("rect", {
        x: cx - 22, y: by, width: 44, height: 11, rx: 3,
        fill: style.fill, stroke: style.stroke, "stroke-width": 0.75,
        "pointer-events": "none",
      }));
      svg.appendChild(el("text", {
        x: cx, y: by + 8.5, "text-anchor": "middle",
        fill: style.text, "font-size": 8, "font-weight": 700,
        "pointer-events": "none",
      }, style.label));
    }
  }

  for (let r = 0; r < state.rows; r++) {
    for (let c = 0; c < state.cols; c++) {
      const p = pt(r, c);
      const node = nodeNumber(r, c);
      const circle = el("circle", {
        cx: p.x, cy: p.y, r: state.editMode ? 9 : 6,
        fill: state.editMode ? "#1d4ed8" : "#dc2626",
        stroke: "#ffffff",
        "stroke-width": 1.5,
        style: state.editMode ? "cursor:crosshair" : "",
      });
      circle.addEventListener("click", (event) => {
        event.stopPropagation();
        handleGridNodeClick(r, c);
      });
      svg.appendChild(circle);
      svg.appendChild(el("text", { x: p.x - 11, y: p.y - 10, "text-anchor": "end", fill: "#64748b", "font-size": 10 }, node));
    }
  }
  const node0Redraw = visionGridToPoint(DEPOT_NODE0_GRID.x, DEPOT_NODE0_GRID.y);
  svg.appendChild(el("circle", { cx: node0Redraw.x, cy: node0Redraw.y, r: 12, fill: "#111827", stroke: "#ffffff", "stroke-width": 2 }));
  svg.appendChild(el("text", { x: node0Redraw.x, y: node0Redraw.y + 4, "text-anchor": "middle", fill: "#fff", "font-size": 11, "font-weight": 700 }, "0"));
  drawActiveEdgeConflicts();
  drawSimulationAgents();

  for (let c = 0; c < state.cols; c++) {
    const p = pt(state.rows - 1, c);
    svg.appendChild(el("text", { x: p.x, y: p.y + 26, "text-anchor": "middle", fill: "#64748b", "font-size": 11 }, c + 1));
  }
  for (let r = 0; r < state.rows; r++) {
    const p = pt(r, 0);
    // Row 1 is the row nearest the depot (nodes 1-8), matching the node
    // numbering; the top row is row `rows` (nodes 57-64 on an 8x8).
    svg.appendChild(el("text", { x: p.x - 24, y: p.y + 4, "text-anchor": "middle", fill: "#64748b", "font-size": 11 }, state.rows - r));
  }
}

function drawHeat() {
  if (!state.edgeUse.size) return;
  const maxUse = Math.max(...state.edgeUse.values());
  for (const [key, use] of state.edgeUse.entries()) {
    if (use < 2) continue;
    const [a, b] = key.split("|").map((part) => {
      if (part.startsWith("g:")) {
        const [r, c] = part.replace("g:", "").split(",").map(Number);
        return pt(r, c);
      }
      if (part.startsWith("e:")) {
        return bayEntryPoint(parseBayId(part.replace("e:", "")));
      }
      if (part.startsWith("w:")) {
        return bayWorkPoint(parseBayId(part.replace("w:", "")));
      }
      return specialPt(part.replace("s:", ""));
    });
    const width = 5 + (use / maxUse) * 12;
    svg.appendChild(el("line", { x1: a.x, y1: a.y, x2: b.x, y2: b.y, stroke: "#f97316", "stroke-width": width, opacity: 0.25, "stroke-linecap": "round" }));
  }
}

function drawRoutes() {
  for (const route of visibleRoutes()) {
    const color = palette[route.agent % palette.length];
    const points = route.path.map((p) => {
      const q = pointFor(p);
      return `${q.x},${q.y}`;
    }).join(" ");
    if (route.path.length > 1) {
      svg.appendChild(el("polyline", { points, fill: "none", stroke: color, "stroke-width": 3, opacity: 0.72, "stroke-linejoin": "round", "stroke-linecap": "round" }));
    }
    const start = routeStart(route.agent);
    const sp = pt(start.r, start.c);
    svg.appendChild(el("circle", { cx: sp.x, cy: sp.y, r: 12, fill: color, stroke: "#ffffff", "stroke-width": 2 }));
    svg.appendChild(el("text", { x: sp.x, y: sp.y + 4, "text-anchor": "middle", fill: "#fff", "font-size": 11, "font-weight": 700 }, `A${route.agent + 1}`));
    for (let i = 0; i < (route.manualWaypoints || []).length; i++) {
      const wp = pt(route.manualWaypoints[i].r, route.manualWaypoints[i].c);
      svg.appendChild(el("rect", {
        x: wp.x - 7, y: wp.y - 7, width: 14, height: 14,
        transform: `rotate(45 ${wp.x} ${wp.y})`,
        fill: "#f8fafc",
        stroke: color,
        "stroke-width": 3,
      }));
      svg.appendChild(el("text", {
        x: wp.x,
        y: wp.y + 4,
        "text-anchor": "middle",
        fill: color,
        "font-size": 9,
        "font-weight": 800,
      }, i + 1));
    }
  }
}

function makespan(routes = state.routes) {
  // A launch replay owns the clock while it is showing, so Play/Step/the
  // scrub slider all run over the recorded window instead of the plan's.
  if (routes === state.routes && replayActive()) return replayDuration();
  if (!routes.length) return 0;
  return Math.max(...routes.map((route) => route.durationSec || 0));
}

function pointAt(route, tSec) {
  if (!route.path.length) return specialPathPoint(`depot-slot-${route.agent}`);
  const seg = (route.segments || []).find((s) => s.startSec <= tSec && tSec <= s.endSec);
  if (seg) return seg.from;
  const passed = (route.segments || []).filter((s) => s.endSec <= tSec);
  if (passed.length) return passed[passed.length - 1].to;
  return route.path[0];
}

function pointAtSeconds(route, tSec) {
  if (!route.path.length) return pointFor(specialPathPoint(`depot-slot-${route.agent}`));
  const seg = (route.segments || []).find((s) => s.startSec <= tSec && tSec <= s.endSec);
  if (!seg) return pointFor(pointAt(route, tSec));
  const denom = Math.max(0.001, seg.endSec - seg.startSec);
  const alpha = Math.max(0, Math.min(1, (tSec - seg.startSec) / denom));
  const a = pointFor(seg.from);
  const b = pointFor(seg.to);
  return {
    x: a.x + (b.x - a.x) * alpha,
    y: a.y + (b.y - a.y) * alpha,
  };
}

function locationLabelAt(route, tSec) {
  const seg = (route.segments || []).find((s) => s.startSec <= tSec && tSec <= s.endSec);
  if (seg) return `${seg.fromNode}->${seg.toNode}`;
  return String(pathLabel(pointAt(route, tSec)));
}

function activeEdgeSegment(route, tSec) {
  return (route.segments || []).find((seg) =>
    seg.startSec <= tSec &&
    tSec < seg.endSec &&
    String(seg.fromNode) !== String(seg.toNode)
  );
}

function activeEdgeConflictsAt(tSec, routes = state.routes) {
  const active = [];
  for (const route of routes) {
    const seg = activeEdgeSegment(route, tSec);
    if (!seg) continue;
    active.push({
      agent: route.agent,
      key: [seg.fromNode, seg.toNode].sort().join("<>"),
      from: seg.fromNode,
      to: seg.toNode,
      segment: seg,
    });
  }

  const conflicts = [];
  for (let i = 0; i < active.length; i++) {
    for (let j = i + 1; j < active.length; j++) {
      const a = active[i];
      const b = active[j];
      if (a.key !== b.key) continue;
      conflicts.push({
        edgeKey: a.key,
        agents: [a.agent, b.agent],
        segments: [a.segment, b.segment],
        sameDirection: String(a.from) === String(b.from) && String(a.to) === String(b.to),
      });
    }
  }
  return conflicts;
}

function edgeLabelFromConflict(conflict) {
  return conflict.edgeKey.replace("<>", " <-> ");
}

function timeWindowEdgeConflictsAt(tSec) {
  return state.conflicts
    .filter((conflict) =>
      conflict.severity === "fatal" &&
      isEdgeConflict(conflict) &&
      conflict.agents.every((agent) => isAgentVisible(agent)) &&
      tSec >= (conflict.timeSec || 0) &&
      tSec <= (conflict.timeSec || 0) + EDGE_SAFETY_WINDOW_SEC
    )
    .map((conflict) => ({
      edgeKey: conflict.edgeKey,
      agents: conflict.agents,
      segments: (conflict.intervals || []).map((item) => item.segment).filter(Boolean),
      sameDirection: conflict.type !== "opposite edge" && conflict.type !== "opposite edge contention",
    }))
    .filter((conflict) => conflict.segments.length);
}

function currentEdgeContentionsAt(tSec) {
  const active = activeEdgeConflictsAt(tSec, visibleRoutes());
  const windowed = timeWindowEdgeConflictsAt(tSec);
  const byKey = new Map();
  for (const conflict of [...active, ...windowed]) {
    const key = `${conflict.edgeKey}|${conflict.agents.join(",")}`;
    if (!byKey.has(key)) byKey.set(key, conflict);
  }
  return [...byKey.values()];
}

function drawActiveEdgeConflicts() {
  if (!state.routes.length) return;
  const t = Math.min(state.timestep, makespan());
  const conflicts = currentEdgeContentionsAt(t);
  const drawn = new Set();
  for (const conflict of conflicts) {
    if (drawn.has(conflict.edgeKey)) continue;
    drawn.add(conflict.edgeKey);
    const seg = conflict.segments[0];
    const a = pointFor(seg.from);
    const b = pointFor(seg.to);
    const midX = (a.x + b.x) / 2;
    const midY = (a.y + b.y) / 2;
    svg.appendChild(el("line", {
      x1: a.x, y1: a.y, x2: b.x, y2: b.y,
      stroke: "#ef4444",
      "stroke-width": 13,
      opacity: 0.82,
      "stroke-linecap": "round",
    }));
    svg.appendChild(el("text", {
      x: midX,
      y: midY - 10,
      "text-anchor": "middle",
      fill: "#991b1b",
      "font-size": 11,
      "font-weight": 800,
      stroke: "#ffffff",
      "stroke-width": 3,
      "paint-order": "stroke",
    }, `A${conflict.agents.map((agent) => agent + 1).join("/A")}`));
  }
}

function drawSimulationAgents() {
  if (replayActive()) { drawReplayAgents(); return; }
  if (!state.routes.length) return;
  const t = Math.min(state.timestep, makespan());
  for (const route of visibleRoutes()) {
    const isReal = liveDataMode();
    const p = isReal
      ? realPointForAgent(route.agent) || pointAtSeconds(route, t)
      : pointAtSeconds(route, t);
    const color = palette[route.agent % palette.length];
    svg.appendChild(el("circle", { cx: p.x, cy: p.y, r: 13, fill: color, stroke: "#ffffff", "stroke-width": 2.5 }));
    svg.appendChild(el("text", { x: p.x, y: p.y + 4, "text-anchor": "middle", fill: "#fff", "font-size": 11, "font-weight": 700 }, `A${route.agent + 1}`));
  }
}

function setTimestep(t) {
  const maxT = makespan();
  state.timestep = Math.max(0, Math.min(maxT, Number(t) || 0));
  render();
}

function stopPlayback() {
  if (state.timer) {
    clearInterval(state.timer);
    state.timer = null;
  }
  document.getElementById("play-route").textContent = "Play";
}

const PLAY_TICK_BASE_MS = 420;

function currentPlaySpeed() {
  const sel = document.getElementById("play-speed");
  const v = sel ? Number(sel.value) : 1;
  return v > 0 ? v : 1;
}

function startPlaybackTimer() {
  if (state.timer) clearInterval(state.timer);
  state.timer = setInterval(() => {
    if (state.timestep >= makespan()) {
      stopPlayback();
      return;
    }
    state.timestep = Math.min(makespan(), state.timestep + 1);
    render();
  }, PLAY_TICK_BASE_MS / currentPlaySpeed());
}

function togglePlayback() {
  if (!state.routes.length && !replayActive()) return;
  if (state.timer) {
    stopPlayback();
    return;
  }
  document.getElementById("play-route").textContent = "Pause";
  startPlaybackTimer();
}

function stepRoute() {
  if (!state.routes.length && !replayActive()) return;
  if (state.timestep >= makespan()) state.timestep = 0;
  else state.timestep = Math.min(makespan(), state.timestep + 1);
  render();
}

function rescheduleCurrentRoutes(resetExtraWait = true) {
  if (!state.routes.length) {
    distributeWork();
  } else {
    const scheduled = edgeExclusiveSchedule(state.routes.map((route) =>
      buildAgentRoute(
        route.agent,
        route.bays,
        resetExtraWait ? 0 : route.extraWaitSec || 0,
        manualWaypointsFor(route.agent),
        resetExtraWait ? {} : route.waitBeforeSec || {},
        route.pathMode || "vh"
      )
    ));
    state.routes = scheduled.routes;
    state.conflicts = scheduled.conflicts;
    state.schedule = scheduled.schedule;
    invalidateDispatchEditors();
  }
  computeEdgeUse();
  state.conflicts = detectConflicts();
  refreshScheduleStatus();
  state.timestep = Math.min(state.timestep, makespan());
}

function handleGridNodeClick(r, c) {
  if (!state.editMode) return;
  stopPlayback();
  const agent = state.editAgent;
  const waypoints = manualWaypointsFor(agent);
  const last = waypoints[waypoints.length - 1];
  if (last && last.r === r && last.c === c) return;
  waypoints.push({ r, c });
  setManualWaypoints(agent, waypoints);
  rescheduleCurrentRoutes(true);
  render();
}

function toggleBay(id) {
  stopPlayback();
  if (state.selected.has(id)) state.selected.delete(id);
  else state.selected.add(id);
  state.routes = [];
  state.edgeUse = new Map();
  state.totalDistance = 0;
  state.conflicts = [];
  state.schedule = null;
  render();
}

// The lab's standard 12-workstation layout, applied on page load so it
// doesn't have to be re-clicked every session. Bay numbers assume the 8x8
// grid (7 bays per row); out-of-range bays are skipped on smaller grids.
// The 12 physical workstations (W01..W12), as bay numbers under the
// bottom-up numbering (B1 = bottom-left). Same physical bays as the old
// top-down list [14,16,17,22,24,27,31,37,38,42,43,47] — renumbered
// 2026-07-17 when the bay origin moved to the bottom-left.
const DEFAULT_BAY_NUMBERS = [1, 5, 9, 10, 14, 17, 22, 24, 27, 30, 31, 42];

function applyDefaultBays() {
  state.selected.clear();
  for (const n of DEFAULT_BAY_NUMBERS) {
    const bay = bayFromNumber(n);
    if (bay.r >= 0 && bay.r < state.rows - 1 && bay.c < state.cols - 1) {
      state.selected.add(bayId(bay.r, bay.c));
    }
  }
}

function selectAll() {
  stopPlayback();
  state.selected.clear();
  for (let r = 0; r < state.rows - 1; r++) {
    for (let c = 0; c < state.cols - 1; c++) state.selected.add(bayId(r, c));
  }
  state.routes = [];
  state.edgeUse = new Map();
  state.conflicts = [];
  state.schedule = null;
  render();
}

function clearBays() {
  stopPlayback();
  state.selected.clear();
  state.routes = [];
  state.edgeUse = new Map();
  state.totalDistance = 0;
  state.conflicts = [];
  state.schedule = null;
  render();
}

function externalAlgorithm() {
  const v = document.getElementById("route-mode").value;
  return v.startsWith("ext:") ? v.slice(4) : null;
}

function buildSolveConfig() {
  return {
    grid: { rows: state.rows, cols: state.cols },
    agents: state.agents,
    capacity: state.capacity,
    rpd: { enabled: state.rpdMode, process_sec: state.processSec },
    workstations: [...state.selected].map(parseBayId).map((bay) => ({
      bay: bayNumber(bay.r, bay.c),
      north_entry_nodes: [nodeNumber(bay.r, bay.c), nodeNumber(bay.r, bay.c + 1)],
    })),
  };
}

async function solveExternal(algorithm) {
  if (!state.selected.size) {
    alert("Select workstation bays first.");
    return;
  }
  state.realStatus = `Solving with ${algorithm} on the plan server...`;
  const status = document.getElementById("real-status");
  if (status) status.textContent = state.realStatus;
  setSolveButtonBusy(true, `Solving (${algorithm})...`);
  try {
    const resp = await fetch(`${PLAN_SERVER_URL}/solve`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        algorithm,
        config: buildSolveConfig(),
        seed: clampInt(document.getElementById("seed").value, 1, 999999),
      }),
    });
    const data = await resp.json();
    if (!resp.ok) throw new Error(data.error || `HTTP ${resp.status}`);
    state.realStatus = "Simulated playback";
    importPlanFromJson(data, "plan server");
  } catch (err) {
    state.realStatus = "Simulated playback";
    alert(`External solve failed: ${err.message}\n\n` +
          `Is the plan server running on this machine?\n` +
          `  python plan_server.py`);
    render();
  } finally {
    setSolveButtonBusy(false);
  }
}

// Toggle the Solve button between its idle label and a "Solving..." state
// while an external (plan-server) solve is in flight — the built-in solver
// runs synchronously so it never needs this, but the plan-server round trip
// can take seconds and gave no visible sign of progress on the Setup tab.
function setSolveButtonBusy(busy, label) {
  const btn = document.getElementById("solve");
  if (!btn) return;
  if (busy) {
    if (btn.dataset.idleLabel === undefined) btn.dataset.idleLabel = btn.textContent;
    btn.textContent = label || "Solving...";
    btn.disabled = true;
    btn.classList.add("busy");
  } else {
    btn.textContent = btn.dataset.idleLabel || "Solve";
    btn.disabled = false;
    btn.classList.remove("busy");
  }
}

function solve(openModal = true) {
  syncControls();
  stopPlayback();
  const ext = externalAlgorithm();
  if (ext) {
    solveExternal(ext);  // async — applies the plan and opens the report
    return;
  }
  distributeWork();
  computeEdgeUse();
  state.conflicts = detectConflicts();
  refreshScheduleStatus();
  state.timestep = 0;
  render();
  // successful solve: collapse setup and put the plan front and center
  if (state.routes.length) setView("plan");
  if (openModal) showReport();
}

function syncControls() {
  state.rows = clampInt(document.getElementById("rows").value, 2, 16);
  state.cols = clampInt(document.getElementById("cols").value, 2, 16);
  state.agents = clampInt(document.getElementById("agents").value, 1, MAX_ROBOTS);
  state.capacity = clampInt(document.getElementById("capacity").value, 1, 20);
  state.rpdMode = document.getElementById("rpd-mode").checked;
  state.processSec = clampInt(document.getElementById("process-sec").value, 0, 600);
  clampAgentState();
  document.getElementById("rows").value = state.rows;
  document.getElementById("cols").value = state.cols;
  document.getElementById("agents").value = state.agents;
  document.getElementById("capacity").value = state.capacity;
  document.getElementById("process-sec").value = state.processSec;
  for (const id of [...state.selected]) {
    const bay = parseBayId(id);
    if (bay.r >= state.rows - 1 || bay.c >= state.cols - 1) state.selected.delete(id);
  }
  for (const [agent, waypoints] of [...state.routeEdits.entries()]) {
    setManualWaypoints(agent, waypoints.filter((node) =>
      node.r >= 0 && node.r < state.rows && node.c >= 0 && node.c < state.cols
    ));
  }
}

function buildGrid() {
  stopPlayback();
  syncControls();
  state.routes = [];
  state.edgeUse = new Map();
  state.totalDistance = 0;
  state.conflicts = [];
  state.schedule = null;
  render();
}

function recomputeIfSolved() {
  const hadRoutes = state.routes.length > 0;
  syncControls();
  if (externalAlgorithm()) {
    // External algorithms only run on an explicit Solve press — a settings
    // change must not silently replace their plan with a built-in one.
    if (hadRoutes) {
      state.realStatus = "settings changed — press Solve to re-run the external algorithm";
    }
    render();
    return;
  }
  if (hadRoutes && state.selected.size) {
    distributeWork();
    computeEdgeUse();
    state.conflicts = detectConflicts();
    refreshScheduleStatus();
  } else {
    state.routes = [];
    state.edgeUse = new Map();
    state.totalDistance = 0;
    state.conflicts = [];
    state.schedule = null;
  }
  state.timestep = Math.min(state.timestep, makespan());
  render();
}

function routeText(route) {
  if (!route.bays.length) return "No assigned bays";
  const hasPicks = route.bays.some((bay) => bay.mode === "pick" || bay.mode === "service");
  return route.bays.map((bay) => {
    const tag = bay.mode === "pick" ? " pick"
      : bay.mode === "service" ? " drop+pick"
      : (hasPicks ? " drop" : "");
    // B# is the bay id; (#) is the workstation node number shown in the
    // bay's green marker on the grid.
    return `B${bayNumber(bay.r, bay.c)} (${workstationNodeNumber(bay)})${tag}`;
  }).join(" -> ");
}

function edgeKeyToLabel(key) {
  return key
    .replaceAll("g:", "")
    .replaceAll("s:", "")
    .replaceAll("e:", "E")
    .replaceAll("w:", "W")
    .replace("|", " to ");
}

function renderLists() {
  const bayList = document.getElementById("bay-list");
  bayList.innerHTML = "";
  if (!state.selected.size) {
    const p = document.createElement("p");
    p.className = "muted";
    p.textContent = "No workstation bays selected.";
    bayList.appendChild(p);
  } else {
    [...state.selected].map(parseBayId).sort((a, b) => bayNumber(a.r, a.c) - bayNumber(b.r, b.c)).forEach((bay) => {
      const card = document.createElement("div");
      card.className = "row-card";
      card.innerHTML = `<b>B${bayNumber(bay.r, bay.c)}</b><span>Bay row ${bay.r + 1}, col ${bay.c + 1}<br><span class="muted">North entry between nodes ${nodeNumber(bay.r, bay.c)} and ${nodeNumber(bay.r, bay.c + 1)}</span></span>`;
      bayList.appendChild(card);
    });
  }

  const routeList = document.getElementById("route-list");
  routeList.innerHTML = "";
  if (!state.routes.length) {
    const p = document.createElement("p");
    p.className = "muted";
    p.textContent = "Solve to draw complete paths.";
    routeList.appendChild(p);
  } else {
    state.routes.forEach((route) => {
      const card = document.createElement("div");
      card.className = "row-card";
      card.innerHTML = `<b style="color:${palette[route.agent % palette.length]}">A${route.agent + 1}</b><span>${routeText(route)}<br><span class="muted">${fmtSec(route.durationSec || 0)}, launch ${fmtSec(route.launchOffsetSec || 0)}</span></span>`;
      routeList.appendChild(card);
    });
  }
}

function renderMetrics() {
  const totalBays = (state.rows - 1) * (state.cols - 1);
  document.getElementById("bay-count").textContent = state.selected.size;
  document.getElementById("bay-total").textContent = totalBays;
  document.getElementById("metric-bays").textContent = state.selected.size;
  document.getElementById("metric-agents").textContent = state.agents;
  document.getElementById("metric-distance").textContent = state.routes.length ? fmtSec(makespan()) : "0.0s";
  document.getElementById("download-json").disabled = state.routes.length === 0;
  document.getElementById("open-report").disabled = state.routes.length === 0;

  let hotspot = "None";
  let maxUse = 0;
  for (const [key, use] of state.edgeUse.entries()) {
    if (use > maxUse) {
      maxUse = use;
      hotspot = `${edgeKeyToLabel(key)} (${use}x)`;
    }
  }
  const fatalCount = state.conflicts.filter((c) => c.severity === "fatal").length;
  const nearCount = state.conflicts.length - fatalCount;
  if (state.schedule && state.schedule.collisionFree) {
    document.getElementById("metric-hotspot").textContent = nearCount
      ? `5s safe, ${nearCount} warn`
      : "5s safe";
  } else {
    document.getElementById("metric-hotspot").textContent = state.conflicts.length
      ? `${fatalCount} fatal, ${nearCount} near`
      : hotspot;
  }

  const slider = document.getElementById("timestep-slider");
  const maxT = makespan();
  state.timestep = Math.min(state.timestep, maxT);
  slider.max = Math.ceil(maxT);
  slider.step = 1;
  slider.value = state.timestep;
  const activeConflicts = currentEdgeContentionsAt(state.timestep);
  const activeText = activeConflicts.length
    ? ` | 5s EDGE CONTENTION A${activeConflicts[0].agents.map((agent) => agent + 1).join("/A")} ${edgeLabelFromConflict(activeConflicts[0])}`
    : "";
  document.getElementById("sim-time").textContent = `t=${fmtSec(state.timestep)} / ${fmtSec(maxT)}${activeText}`;

  renderTimelineToggles();
  renderEventTimeline();

  const warning = document.getElementById("capacity-warning");
  const capacity = state.agents * state.capacity;
  if (state.selected.size > capacity) {
    warning.style.display = "";
    warning.textContent = `${state.selected.size} workstations exceed one-trip capacity ${state.agents} x ${state.capacity} = ${capacity}. Routes will still visualize, but agents need reload cycles.`;
  } else {
    warning.style.display = "none";
  }
}

function renderAgentControls() {
  const visibility = document.getElementById("agent-visibility-controls");
  const edit = document.getElementById("agent-edit-controls");
  visibility.innerHTML = "";
  edit.innerHTML = "";

  for (let agent = 0; agent < state.agents; agent++) {
    const visibleButton = document.createElement("button");
    visibleButton.className = `secondary agent-toggle${isAgentVisible(agent) ? " active" : " hidden"}`;
    visibleButton.textContent = `A${agent + 1}`;
    visibleButton.title = isAgentVisible(agent) ? `Hide AGV ${agent + 1}` : `Show AGV ${agent + 1}`;
    visibleButton.addEventListener("click", () => {
      if (state.hiddenAgents.has(agent)) state.hiddenAgents.delete(agent);
      else state.hiddenAgents.add(agent);
      render();
    });
    visibility.appendChild(visibleButton);

    const editButton = document.createElement("button");
    editButton.className = `secondary agent-toggle${state.editAgent === agent ? " active" : ""}`;
    editButton.textContent = `A${agent + 1}`;
    editButton.title = `Edit AGV ${agent + 1}`;
    editButton.addEventListener("click", () => {
      state.editAgent = agent;
      state.editMode = true;
      render();
    });
    edit.appendChild(editButton);
  }

  document.getElementById("toggle-edit-route").className = state.editMode ? "secondary active" : "secondary";
  document.getElementById("toggle-edit-route").textContent = state.editMode ? "Editing on" : "Edit path";
  document.getElementById("clear-route-edit").disabled = !manualWaypointsFor(state.editAgent).length;
  document.getElementById("mode-sim").className = state.mode === "sim" ? "secondary active" : "secondary";
  document.getElementById("mode-real").className = state.mode === "real" ? "secondary active" : "secondary";
  document.getElementById("mode-live").className = state.mode === "live" ? "secondary active" : "secondary";
  document.getElementById("real-status").textContent = state.realStatus;
  document.getElementById("drive-mode-color").className =
    state.driveMode === "color" ? "secondary active" : "secondary";
  document.getElementById("drive-mode-vision").className =
    state.driveMode === "vision" ? "secondary active" : "secondary";
  document.getElementById("drive-mode-status").textContent =
    state.driveMode === "vision" ? "camera-only (no color sensor)" : "color sensor + camera safety";
  document.getElementById("edit-hint").textContent = state.editMode
    ? `Editing AGV ${state.editAgent + 1}: click intersections to add waypoints. Clear edits removes this AGV's manual path.`
    : "Select an AGV, turn on Edit path, then click intersections to add route waypoints.";
}

function render() {
  syncControls();
  drawGrid();
  renderRouteLegend();
  renderSimCommands();
  renderLists();
  renderMetrics();
  renderAgentControls();
  renderSetupSummary();
  renderConflictPane();
  renderRobotTopicPanel();
}


// ===========================================================================
// Recorded depot-launch replay, drawn on THIS grid.
//
// Waypoints in static/data/launch_replay.json are [seconds, node-label], not
// coordinates, so positions come from the solver's own specialPt()/pt() -- the
// robots land exactly on the depot slots and lattice nodes the page already
// draws, and there is no second coordinate system to drift.
//
// Built by tools/make_launch_replay.py; read that file's header for where
// each arm's numbers come from and which parts are measured.
// ===========================================================================

const REPLAY = { data: null, on: { offsets: false, cascade: false } };
const REPLAY_STYLE = {
  offsets: { ring: "#b3471f", label: "fixed offsets" },
  cascade: { ring: "#1f6f7a", label: "rolling cascade" },
};

function replayActive() {
  return !!REPLAY.data && (REPLAY.on.offsets || REPLAY.on.cascade);
}

function replayArms() {
  if (!REPLAY.data) return [];
  return REPLAY.data.arms.filter((a) => REPLAY.on[a.id]);
}

function replayDuration() {
  const arms = replayArms();
  if (!arms.length) return 0;
  return Math.ceil(Math.max(...arms.map((a) => a.end_s)));
}

function replayAgentIndex(name) {
  const m = /(\d+)/.exec(name);
  return m ? parseInt(m[1], 10) - 1 : 0;
}

// Node label -> screen point, using the page's own geometry.
function replayNodePoint(label) {
  const s = String(label);
  if (s === "0") return specialPt("node0");
  let m = /^D(\d+)$/.exec(s);
  if (m) return specialPt(`depot-slot-${parseInt(m[1], 10) - 1}`);
  m = /^DE(\d+)$/.exec(s);
  if (m) return specialPt(`depot-entry-${parseInt(m[1], 10) - 1}`);
  const n = parseInt(s, 10);
  if (!Number.isFinite(n)) return specialPt("node0");
  const idx = n - 1;
  return pt(state.rows - 1 - Math.floor(idx / state.cols), idx % state.cols);
}

function replayPosAt(wps, t) {
  if (!wps || !wps.length) return null;
  if (t <= wps[0][0]) return replayNodePoint(wps[0][1]);
  const last = wps[wps.length - 1];
  if (t >= last[0]) return replayNodePoint(last[1]);
  let lo = 0;
  let hi = wps.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (wps[mid][0] <= t) lo = mid; else hi = mid;
  }
  const a = replayNodePoint(wps[lo][1]);
  const b = replayNodePoint(wps[hi][1]);
  const span = wps[hi][0] - wps[lo][0];
  const f = span > 1e-6 ? (t - wps[lo][0]) / span : 0;
  return { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f };
}

function drawReplayAgents() {
  const arms = replayArms();
  const t = Math.min(state.timestep, replayDuration());
  arms.forEach((arm, ai) => {
    const style = REPLAY_STYLE[arm.id];
    let onGrid = 0;
    for (const [name, wps] of Object.entries(arm.robots)) {
      const p = replayPosAt(wps, t);
      if (!p) continue;
      const agent = replayAgentIndex(name);
      const color = palette[agent % palette.length];
      const entered = arm.entry_s[name] !== undefined && t >= arm.entry_s[name];
      if (entered) onGrid += 1;
      // Two arms can occupy the same node at the same instant; nudge the
      // second one so neither is hidden underneath the other.
      const dx = arms.length > 1 ? (ai === 0 ? -9 : 9) : 0;
      svg.appendChild(el("circle", {
        cx: p.x + dx, cy: p.y, r: 12,
        fill: color, "fill-opacity": entered ? 1 : 0.45,
        stroke: style.ring, "stroke-width": 3,
      }));
      svg.appendChild(el("text", {
        x: p.x + dx, y: p.y + 4, "text-anchor": "middle",
        fill: "#fff", "font-size": 11, "font-weight": 700,
      }, `A${agent + 1}`));
    }
    const g = layout();
    const y = g.top - 34 + ai * 17;
    svg.appendChild(el("text", {
      x: g.left, y, fill: style.ring, "font-size": 12, "font-weight": 700,
    }, `${style.label}: ${onGrid}/6 on the grid`
       + `  \u00b7  last onto node 0 at ${arm.last_entry_s.toFixed(1)}s`));
  });
}

function replaySyncButtons() {
  for (const b of document.querySelectorAll(".replay-arm")) {
    b.classList.toggle("active", !!REPLAY.on[b.dataset.arm]);
  }
  const note = document.getElementById("replay-note");
  if (note) {
    const arms = replayArms();
    note.textContent = arms.length
      ? arms.map((a) => `${REPLAY_STYLE[a.id].label}: ${a.source}`).join("   |   ")
      : "Same route both times \u2014 only the release schedule differs.";
  }
}

async function replayEnsureData() {
  if (REPLAY.data) return true;
  try {
    const res = await fetch("/static/data/launch_replay.json");
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    REPLAY.data = await res.json();
    return true;
  } catch (err) {
    const note = document.getElementById("replay-note");
    if (note) {
      note.textContent = "Could not load the replay data — run "
        + `tools/make_launch_replay.py. (${err.message})`;
    }
    return false;
  }
}

function replayInitUI() {
  const box = document.getElementById("replay-open");
  const row = document.getElementById("replay-arms");
  if (!box || !row) return;
  box.addEventListener("click", async () => {
    const shown = row.style.display !== "none";
    row.style.display = shown ? "none" : "inline-flex";
    box.setAttribute("aria-expanded", shown ? "false" : "true");
    if (!shown) { await replayEnsureData(); replaySyncButtons(); }
  });
  for (const b of document.querySelectorAll(".replay-arm")) {
    b.addEventListener("click", async () => {
      if (!(await replayEnsureData())) return;
      const id = b.dataset.arm;
      REPLAY.on[id] = !REPLAY.on[id];
      stopPlayback();
      state.timestep = 0;
      replaySyncButtons();
      render();
    });
  }
  replaySyncButtons();
}

document.addEventListener("DOMContentLoaded", replayInitUI);
