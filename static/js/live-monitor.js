
// ---- workflow views --------------------------------------------------------
// Pure CSS show/hide keyed on body[data-view]; no DOM is created or removed,
// so switching views never resets configuration, routes, playback, or the
// rosbridge/camera connections.
function setView(view) {
  state.view = view;
  document.body.dataset.view = view;
  document.getElementById("view-setup").className = view === "setup" ? "secondary active" : "secondary";
  document.getElementById("view-plan").className = view === "plan" ? "secondary active" : "secondary";
  document.getElementById("view-run").className = view === "run" ? "secondary active" : "secondary";
  document.getElementById("view-metrics").className = view === "metrics" ? "secondary active" : "secondary";
  // The camera feed belongs to Run & Monitor only. Leaving the Run view (e.g.
  // back to Plan & Simulate) drops the MJPEG connection and hides the <img>;
  // returning to Run while Live is selected re-attaches it. Plan & Simulate is
  // always pure route playback (see liveDataMode), so this keeps the live
  // video off that tab even if Live was left active on the Run tab.
  const panel = document.querySelector(".grid-panel");
  const feed = document.getElementById("live-feed");
  if (panel && feed) {
    if (view === "run" && state.mode === "live") {
      panel.classList.add("live-on");
      if (!feed.src) feed.src = LIVE_STREAM_URL;
    } else {
      panel.classList.remove("live-on");
      if (feed.src) feed.removeAttribute("src");
    }
  }
  render();
  // Page 4 refreshes its data when shown; guarded so a missing/broken
  // metrics.js can never affect the other three views.
  if (view === "metrics" && typeof metricsViewShown === "function") {
    try { metricsViewShown(); } catch (err) { console.warn("metrics view error:", err); }
  }
}

// Compact configuration recap shown in the top bar outside the Setup view.
function renderSetupSummary() {
  const text = document.getElementById("setup-summary-text");
  if (!text) return;
  const modeSel = document.getElementById("route-mode");
  const alg = modeSel && modeSel.selectedOptions.length
    ? modeSel.selectedOptions[0].textContent : "";
  text.textContent =
    `${state.rows}×${state.cols} grid · ${state.selected.size} bays · ` +
    `${state.agents} agents ×${state.capacity}` +
    (state.rpdMode ? ` · RPD ${state.processSec}s` : "") +
    ` · ${state.planSource || alg}`;
}

// Read-only conflict summary for the Events/Conflicts side tab (the Route
// report keeps the full cards with reroute buttons).
// Live supervisor warnings (vision proximity + errors) from the run, pulled
// from the decision-log feed. Shown ABOVE the plan-time conflict predictions so
// the same pane answers both "what did the planner foresee" and "what is
// actually happening on the floor right now".
function liveWarningCards() {
  // FILTER BY SEVERITY, NOT JUST KIND (2026-08-31). This used to be
  //   ev.kind === "vision" || ev.kind === "error"
  // which silently excluded the single most important live event there is.
  // The supervisor logs a deadlock as
  //   self._event("wait", f"DEADLOCK hold: {reason}", severity="error")
  // so its KIND is "wait" and only its SEVERITY is "error". During the
  // 6-robot deadlock on 2026-08-31 every one of the ~100 log entries was a
  // "wait", so this pane sat empty saying no contentions were detected while
  // all six robots stood still -- the one moment it was being looked at.
  const live = state.decisionLog.filter(
    (ev) => ev.kind === "vision" || ev.kind === "error"
            || ev.severity === "error" || ev.severity === "warn");
  if (!live.length) return "";

  // Collapse consecutive repeats. A held deadlock re-logs every few seconds
  // (25+ identical lines in that run), so without this the pane would just
  // be the same sentence twelve times instead of twelve distinct problems.
  const groups = [];
  for (const ev of live) {
    const key = `${ev.robot || ""}|${ev.text}`;
    const last = groups.length ? groups[groups.length - 1] : null;
    if (last && last.key === key) {
      last.count += 1;
      last.lastMs = ev.ms;
    } else {
      groups.push({ key, ev, count: 1, firstMs: ev.ms, lastMs: ev.ms });
    }
  }

  const recent = groups.slice(-12).reverse();
  const rows = recent.map((g) => {
    const ev = g.ev;
    const isError = ev.severity === "error" || ev.kind === "error";
    const deadlock = /deadlock/i.test(ev.text);
    const cls = "conflict-card " + (isError ? "live-error" : "live-vision");
    const tag = deadlock ? "DEADLOCK"
      : (isError ? "LIVE ERROR" : (ev.kind === "vision" ? "LIVE VISION" : "LIVE"));
    const who = ev.robot ? `${escapeHtml(ev.robot)} ` : "";
    // For a held condition the useful facts are how long it has persisted
    // and whether it is still going, not that it happened once.
    const heldSec = (g.lastMs - g.firstMs) / 1000;
    const repeat = g.count > 1
      ? ` <span class="muted">(x${g.count}${heldSec >= 1 ? `, held ${fmtSec(heldSec)}` : ""})</span>`
      : "";
    return `<div class="${cls}"><strong>${tag}</strong>` +
      `<span>${decisionTimeStr(g.lastMs)} — ${who}${escapeHtml(ev.text)}${repeat}</span>` +
      `<span></span></div>`;
  }).join("");
  return `<p class="muted">Live from the supervisor (${groups.length} distinct, ` +
    `${live.length} total this run — full history in the Decision Log below):` +
    `</p>${rows}`;
}

function renderConflictPane() {
  const box = document.getElementById("conflict-pane");
  if (!box) return;
  const liveCards = liveWarningCards();
  if (!state.routes.length) {
    box.innerHTML = liveCards ||
      '<p class="muted">Solve to evaluate conflicts.</p>';
    return;
  }
  const fatal = state.conflicts.filter((c) => c.severity === "fatal").length;
  const status = state.schedule && state.schedule.collisionFree
    ? "Schedule is collision-free inside the fatal safety windows."
    : `${fatal} fatal risk(s) remain — open the Route report to reroute.`;
  const cards = state.conflicts.slice(0, 40).map((conflict) => `
    <div class="conflict-card">
      <strong>${escapeHtml(conflict.severity)}</strong>
      <span>t=${escapeHtml(fmtSec(conflict.timeSec || 0))} | ${escapeHtml(conflict.type)} | A${conflict.agents.map((a) => a + 1).join(", A")} | ${escapeHtml(conflict.detail)}</span>
      <span></span>
    </div>`).join("");
  box.innerHTML = liveCards +
    `<p class="muted">Planner prediction: ${escapeHtml(status)}</p>` +
    (cards || '<p class="muted">No node or edge contentions inside the safety windows.</p>') +
    (state.conflicts.length > 40
      ? `<p class="muted">Showing first 40 of ${state.conflicts.length}.</p>` : "");
}

// Always-visible visit order per robot, under the grid: drop-offs marked v,
// pickups marked ^ (plain bay numbers when RPD is off).
function renderRouteLegend() {
  const box = document.getElementById("route-legend");
  if (!box) return;
  if (!state.routes.length) {
    box.innerHTML = "";
    return;
  }
  const items = state.routes.map((route) => {
    const hasPicks = route.bays.some((bay) => bay.mode === "pick" || bay.mode === "service");
    const seq = route.bays.length
      ? route.bays.map((bay) => {
          const arrow = bay.mode === "pick" ? "↑"
            : bay.mode === "service" ? "↓↑"
            : (hasPicks ? "↓" : "");
          return `B${bayNumber(bay.r, bay.c)}${arrow}`;
        }).join(" → ")
      : "no bays";
    return `<span style="color:${palette[route.agent % palette.length]}">` +
      `<b>${agvName(route.agent)}</b>&nbsp;${seq}</span>`;
  });
  if (state.planSource) {
    items.unshift(`<span class="status-key">plan: ${escapeHtml(state.planSource)}</span>`);
  }
  if (state.rpdMode) {
    items.push(`<span class="status-key">↓ drop-off &nbsp;↑ pickup &nbsp;|&nbsp; ` +
      `badges: AWAIT = waiting for delivery, PROC = processing, ` +
      `READY = waiting for pickup, DONE = complete</span>`);
  }
  box.innerHTML = items.join("");
}

// Per-robot generated commands under the grid in the Plan & Simulate view, so
// the plan's progress is visible textually alongside the graphical playback.
// Each robot's currently-executing command (the one whose route segment is
// active at state.timestep) is highlighted; already-finished commands dim.
// Only visible robots (Visible toggles) are shown, matching the grid. Hidden
// while a live mission is running the Run view (that tab has its own dispatch
// editors); this is the simulated-playback companion.
function renderSimCommands() {
  const box = document.getElementById("sim-commands");
  if (!box) return;
  if (!state.routes.length) {
    box.innerHTML = '<p class="muted">Solve to generate per-robot commands.</p>';
    return;
  }
  const t = Math.min(state.timestep, makespan());
  const cols = state.routes
    .filter((route) => isAgentVisible(route.agent))
    .map((route) => {
      const color = palette[route.agent % palette.length];
      const cmds = generateCommandLines(route);
      const done = route.durationSec > 0 && t >= route.durationSec;
      // Segment active at t (start<=t<end). Commands whose segment index is
      // below it are finished; the ones equal to it are executing now.
      const activeSeg = (route.segments || []).find((s) => s.startSec <= t && t < s.endSec);
      const activeIdx = activeSeg ? activeSeg.segmentIndex : null;
      const lineState = (cmd) => {
        if (cmd.warn) return "cmd-warn";
        if (done) return "done";
        if (activeIdx == null) return "";                        // t=0, nothing running yet
        if (cmd.segIndex == null) return "";                     // depot template lines
        if (cmd.segIndex < activeIdx) return "done";
        if (cmd.segIndex === activeIdx) return "active";
        return "";
      };
      const lis = cmds.map((cmd) =>
        `<li class="${lineState(cmd)}">${escapeHtml(cmd.text)}</li>`).join("");
      const total = cmds.filter((c) => !c.warn).length;
      const doneCount = done ? total
        : cmds.filter((c) => !c.warn && lineState(c) === "done").length;
      const progress = done ? "done" : `${doneCount}/${total}`;
      return `<div class="cmd-col">
        <div class="cmd-head" style="background:${color}">
          <span>${agvName(route.agent)}</span><span class="cmd-progress">${progress}</span>
        </div>
        <ol>${lis}</ol>
      </div>`;
    });
  box.innerHTML = cols.join("") ||
    '<p class="muted">All robots hidden — use the Visible toggles to show commands.</p>';
}

function pointFromNodeLabel(label) {
  const text = String(label).trim();
  if (text === "0") return specialPt("node0");
  if (text.startsWith("DE")) return specialPt(`depot-entry-${parseInt(text.replace("DE", ""), 10) - 1}`);
  if (text.startsWith("D")) return specialPt(`depot-slot-${parseInt(text.replace("D", ""), 10) - 1}`);
  const n = Number(text);
  if (Number.isFinite(n) && n >= 1 && n <= state.rows * state.cols) {
    const idx = n - 1;
    const r = state.rows - 1 - Math.floor(idx / state.cols);
    const c = idx % state.cols;
    return pt(r, c);
  }
  return null;
}

function pointFromPosition(position) {
  if (!position) return null;
  if (Number.isFinite(position.svg_x) && Number.isFinite(position.svg_y)) {
    return { x: position.svg_x, y: position.svg_y };
  }
  if (Number.isFinite(position.grid_r) && Number.isFinite(position.grid_c)) {
    return pt(position.grid_r, position.grid_c);
  }
  if (Number.isFinite(position.x) && Number.isFinite(position.y)) {
    const c = Math.max(0, Math.min(state.cols - 1, position.x));
    const r = Math.max(0, Math.min(state.rows - 1, state.rows - 1 - position.y));
    return pt(r, c);
  }
  return null;
}

// Parses the Alvik_pose std_msgs/String payload: JSON text in msg.data,
// {"x":cm,"y":cm,"yaw":deg,"battery":pct,"ms":millis}. Returns
// {point, yawDeg, batteryPct} or null if msg.data isn't that shape.
function alvikPoseStringPayload(msg) {
  if (typeof msg.data !== "string") return null;
  let pose;
  try {
    pose = JSON.parse(msg.data);
  } catch (_) {
    return null;
  }
  if (!Number.isFinite(pose.x) || !Number.isFinite(pose.y)) return null;
  return {
    point: alvikPoseToGridPoint(pose.x, pose.y),
    yawDeg: Number.isFinite(pose.yaw) ? pose.yaw : null,
    batteryPct: Number.isFinite(pose.battery) ? pose.battery : null,
  };
}

function realPayloadToPoint(msg) {
  let node;
  if (typeof msg.node_id !== "undefined") node = msg.node_id;
  else if (typeof msg.grid_node !== "undefined") node = msg.grid_node;
  else if (typeof msg.node !== "undefined") node = msg.node;
  else if (typeof msg.current_node !== "undefined") node = msg.current_node;
  const nodePoint = typeof node !== "undefined" ? pointFromNodeLabel(node) : null;
  if (nodePoint) return nodePoint;
  if (msg.pose && msg.pose.pose && msg.pose.pose.position) return pointFromPosition(msg.pose.pose.position);
  if (msg.pose && msg.pose.position) return pointFromPosition(msg.pose.position);
  if (msg.position) return pointFromPosition(msg.position);
  return pointFromPosition(msg);
}

// Parses the <ROBOT_NAME>_vision_pose payload from apriltag_localize.py.
function visionPoseStringPayload(msg) {
  if (typeof msg.data !== "string") return null;
  let pose;
  try {
    pose = JSON.parse(msg.data);
  } catch (_) {
    return null;
  }
  if (!Number.isFinite(pose.grid_x) || !Number.isFinite(pose.grid_y)) return null;
  return {
    point: visionGridToPoint(pose.grid_x, pose.grid_y),
    yawDeg: Number.isFinite(pose.yaw_deg) ? pose.yaw_deg : null,
    gridX: pose.grid_x,
    gridY: pose.grid_y,
  };
}

function realPointForAgent(agent) {
  const vision = state.visionPositions.get(agent);
  if (vision && Date.now() - vision.receivedAt <= VISION_FRESH_MS) return vision.point;
  const entry = state.realPositions.get(agent);
  return entry ? entry.point : null;
}

function realBatteryForAgent(agent) {
  const entry = state.realPositions.get(agent);
  return entry && Number.isFinite(entry.batteryPct) ? entry.batteryPct : null;
}

function disconnectRealMode() {
  if (state.realSocket) {
    state.realSocket.onopen = null;
    state.realSocket.onmessage = null;
    state.realSocket.onerror = null;
    state.realSocket.onclose = null;
    state.realSocket.close();
    state.realSocket = null;
  }
}

function subscribeRealTopics(socket) {
  for (const config of ALVIK_REAL_TOPICS.filter((item) => item.agent < state.agents)) {
    socket.send(JSON.stringify({
      op: "subscribe",
      topic: config.topic,
      type: config.type,
      throttle_rate: 100,
    }));
  }
  // Subscribe _status for ALL MAX_ROBOTS robots unconditionally (not just
  // ones in an active dispatch mission) so the rostopic inspector panel can
  // show status for any currently-connected robot, not only ones dispatched
  // from this browser tab.
  for (let agent = 0; agent < MAX_ROBOTS; agent++) {
    socket.send(JSON.stringify({
      op: "subscribe",
      topic: `${agvName(agent)}_status`,
      type: "std_msgs/String",
      throttle_rate: 100,
    }));
  }
  socket.send(JSON.stringify({
    op: "subscribe",
    topic: "fleet_status",
    type: "std_msgs/String",
    throttle_rate: 200,
  }));
  // Decision Log feed: discrete run events (vision warnings, errors, recovery,
  // waits, advisor/mission milestones). No throttle — every event matters.
  socket.send(JSON.stringify({
    op: "subscribe",
    topic: "fleet_events",
    type: "std_msgs/String",
  }));
}

function handleRealMessage(event) {
  let payload;
  try {
    payload = JSON.parse(event.data);
  } catch (_) {
    return;
  }
  if (payload.op !== "publish") return;
  if (payload.topic === "fleet_status") {
    renderSupervisorStatus(String((payload.msg || {}).data || ""));
    return;
  }
  if (payload.topic === "fleet_events") {
    ingestDecisionEvent(String((payload.msg || {}).data || ""));
    return;
  }
  if (payload.topic.endsWith("_status")) {
    const name = payload.topic.slice(0, -"_status".length);
    const text = String((payload.msg || {}).data || "").trim();
    recordRobotTopic(name, payload.topic, text);
    if (dispatch.robots[name]) {
      dispatchOnStatus(name, text);
    }
    return;
  }
  const config = ALVIK_REAL_TOPICS.find((item) => item.topic === payload.topic);
  if (!config) return;
  const msg = payload.msg || {};
  if (config.kind === "vision") {
    const visionPose = visionPoseStringPayload(msg);
    if (!visionPose) return;
    state.visionPositions.set(config.agent, {
      point: visionPose.point,
      yawDeg: visionPose.yawDeg,
      gridX: visionPose.gridX,
      gridY: visionPose.gridY,
      receivedAt: Date.now(),
      topic: config.topic,
    });
    state.realStatus = `Real mode: ${agvName(config.agent)} update from ${config.topic} (vision)`;
    recordRobotTopic(agvName(config.agent), config.topic, String(msg.data || ""));
    render();
    return;
  }
  const alvikPose = alvikPoseStringPayload(msg);
  const point = alvikPose ? alvikPose.point : realPayloadToPoint(msg);
  recordRobotTopic(agvName(config.agent), config.topic, String(msg.data || ""));
  if (!point) return;
  state.realPositions.set(config.agent, {
    point,
    yawDeg: alvikPose ? alvikPose.yawDeg : null,
    batteryPct: alvikPose ? alvikPose.batteryPct : null,
    receivedAt: Date.now(),
    topic: config.topic,
  });
  state.realStatus = `Real mode: ${agvName(config.agent)} update from ${config.topic}`;
  render();
}

// Records the latest raw value for one robot/topic pair, keyed by robot name
// (e.g. "Alvik3") so it works for topics not present in ALVIK_REAL_TOPICS
// (e.g. _status). Backs the Run & Monitor per-robot rostopic inspector panel.
function recordRobotTopic(robotName, topic, value) {
  const agent = ALVIK_NAME_TO_AGENT.get(robotName);
  if (agent === undefined) return;
  if (!state.robotTopics.has(agent)) state.robotTopics.set(agent, new Map());
  state.robotTopics.get(agent).set(topic, { value, receivedAt: Date.now() });
  scheduleRobotTopicRender();
}

// THROTTLED re-render (2026-08-31). This used to call renderRobotTopicPanel()
// synchronously on EVERY message. In Real mode that is _pose + _vision_pose +
// _status for every robot -- vision alone runs ~60Hz per robot, so with four
// robots the panel's innerHTML was being torn down and rebuilt several
// hundred times a second.
//
// That is what made the chips unclickable. A DOM click only fires if
// mousedown and mouseup land on the same element; the button under the
// cursor was being destroyed and recreated between the two, so the browser
// had nothing to fire on and the click silently vanished. Nothing was wrong
// with showRobotTopicModal() -- it was simply never reached.
//
// 200ms is well under the 1s age display's resolution, so nothing looks less
// live, and the periodic 1s interval below still refreshes ages when no
// messages arrive at all.
const ROBOT_TOPIC_RENDER_MS = 200;
let robotTopicRenderTimer = null;

function scheduleRobotTopicRender() {
  if (robotTopicRenderTimer) return;
  robotTopicRenderTimer = setTimeout(() => {
    robotTopicRenderTimer = null;
    renderRobotTopicPanel();
    renderRobotTopicModal();
  }, ROBOT_TOPIC_RENDER_MS);
}

// A robot counts as "currently connected" if we've heard from it on any
// topic within this window (rather than just "ever, since page load").
const ROBOT_TOPIC_STALE_MS = 5000;

function toggleRobotTopicPanel(agent) {
  if (state.robotTopicsExpanded.has(agent)) {
    state.robotTopicsExpanded.delete(agent);
  } else {
    state.robotTopicsExpanded.add(agent);
  }
  renderRobotTopicPanel();
}

// Delegated once on the CONTAINER rather than re-bound to each button on
// every render. Besides being cheaper, delegation is what makes a click
// survive a re-render mid-gesture: the container is the common ancestor of
// the old and new button, so the browser still fires click on it.
let robotTopicDelegated = false;
// Signature of the robot set the buttons were last BUILT for. Only a change
// here rebuilds the markup -- see renderRobotTopicPanel().
let robotTopicButtonSig = "";

function ensureRobotTopicDelegation(buttonsBox) {
  if (robotTopicDelegated) return;
  robotTopicDelegated = true;
  buttonsBox.addEventListener("click", (event) => {
    const btn = event.target.closest(".robot-topic-btn");
    if (!btn || !buttonsBox.contains(btn)) return;
    const agent = Number(btn.dataset.agent);
    if (!Number.isFinite(agent)) return;
    // Both surfaces: the inline card in the "Robot Topics" side tab (whose
    // own copy says "Click a robot above ...") and the pop-up. Before this,
    // toggleRobotTopicPanel() was dead code that nothing ever called, so
    // that side tab could never show anything no matter what was clicked.
    toggleRobotTopicPanel(agent);
    if (state.robotTopicsExpanded.has(agent)) showRobotTopicModal(agent);
    else if (agent === robotTopicModalAgent) closeRobotTopicModal();
  });
}

function renderRobotTopicPanel() {
  const buttonsBox = document.getElementById("robot-topic-buttons");
  const detailsBox = document.getElementById("robot-topic-details");
  if (!buttonsBox || !detailsBox) return;
  const now = Date.now();
  const connected = [...state.robotTopics.keys()].sort((a, b) => a - b);
  if (!connected.length) {
    if (robotTopicButtonSig !== "@none") {
      buttonsBox.innerHTML = `<span class="muted" style="font-size:12px;">No robots connected yet — Real mode subscribes on connect.</span>`;
      robotTopicButtonSig = "@none";
    }
    detailsBox.innerHTML = `<p class="muted">Click a robot above to see its live rostopic values.</p>`;
    return;
  }
  ensureRobotTopicDelegation(buttonsBox);

  // BUILD ONCE, THEN UPDATE IN PLACE (2026-08-31, second attempt).
  //
  // The first fix throttled this render to 200ms, which only made the bug
  // rarer. Rebuilding buttonsBox.innerHTML destroys and recreates every
  // button, and a mouse click needs mousedown and mouseup to land on the
  // SAME element -- a press takes ~100ms, so a rebuild every 200ms still
  // landed inside a click routinely and the browser had nothing to fire on.
  // Reducing the rate cannot fix that; only not replacing the elements can.
  //
  // So the markup is rebuilt only when the SET of connected robots changes,
  // and every per-frame value (stale dot, battery text, active highlight) is
  // written onto the existing nodes. The buttons now persist for the life of
  // the connection and a click can always complete.
  const sig = connected.join(",");
  if (sig !== robotTopicButtonSig) {
    buttonsBox.innerHTML = connected.map((agent) => {
      const color = palette[agent % palette.length];
      return `<button type="button" class="robot-topic-btn" data-agent="${agent}">` +
        `<span class="dot" style="background:${color};"></span>` +
        `<span class="rt-name">${escapeHtml(agvName(agent))}</span>` +
        `<span class="batt"></span>` +
        `</button>`;
    }).join("");
    robotTopicButtonSig = sig;
  }
  for (const btn of buttonsBox.querySelectorAll(".robot-topic-btn")) {
    const agent = Number(btn.dataset.agent);
    const topics = state.robotTopics.get(agent);
    if (!topics) continue;
    const newestAge = Math.min(...[...topics.values()].map((t) => now - t.receivedAt));
    const dot = btn.querySelector(".dot");
    if (dot) dot.classList.toggle("stale", newestAge > ROBOT_TOPIC_STALE_MS);
    btn.classList.toggle("active", state.robotTopicsExpanded.has(agent));
    // Battery rides on <name>_pose, the robot's OWN odometry publish over
    // micro-ROS. <name>_vision_pose comes from apriltag_localize.py, which
    // only needs to SEE the tag -- so a robot whose micro-ROS link is down
    // still gets a chip (the camera sees it) but has no battery. Say which.
    const battery = realBatteryForAgent(agent);
    const hasOdom = [...topics.keys()].some(
      (t) => t.endsWith("_pose") && !t.endsWith("_vision_pose"));
    const battEl = btn.querySelector(".batt");
    if (battEl) {
      const text = battery !== null ? `${battery}%` : (hasOdom ? "" : "vision only");
      if (battEl.textContent !== text) battEl.textContent = text;
      battEl.classList.toggle("rt-vision-only", battery === null && !hasOdom);
    }
  }
  const expanded = connected.filter((agent) => state.robotTopicsExpanded.has(agent));
  detailsBox.innerHTML = expanded.length
    ? expanded.map((agent) => {
        const topics = [...state.robotTopics.get(agent).entries()].sort((a, b) => a[0].localeCompare(b[0]));
        const rows = topics.map(([topic, entry]) => {
          const ageSec = ((now - entry.receivedAt) / 1000).toFixed(1);
          return `<tr><td class="rt-topic">${escapeHtml(topic)}</td>` +
            `<td class="rt-value">${escapeHtml(entry.value)}</td>` +
            `<td class="rt-age">${ageSec}s ago</td></tr>`;
        }).join("");
        return `<div class="robot-topic-card">` +
          `<h3>${escapeHtml(agvName(agent))}</h3>` +
          `<table>${rows}</table></div>`;
      }).join("")
    : `<p class="muted">Click a robot above to see its live rostopic values.</p>`;
}

// Refreshes the "Xs ago" ages / stale-dot state even when no new message has
// arrived — cheap no-op while state.robotTopics is empty (before Real connect).
setInterval(() => { renderRobotTopicPanel(); renderRobotTopicModal(); }, 1000);

// ---- Robot Topics pop-up (secondary view of the same live rostopic data) --
// A click on any connected robot's chip (in #robot-topic-buttons) opens this
// modal instead of only expanding the inline side-panel card -- same data
// source (state.robotTopics), so nothing new is subscribed here. Stays open
// and live-updating across incoming messages until closed (X button, click
// outside the card, or Escape); reused for whichever robot was last clicked.
let robotTopicModalAgent = null;

function showRobotTopicModal(agent) {
  robotTopicModalAgent = agent;
  renderRobotTopicModal();
  const modal = document.getElementById("robot-topic-modal");
  modal.classList.add("open");
  modal.setAttribute("aria-hidden", "false");
}

function closeRobotTopicModal() {
  robotTopicModalAgent = null;
  const modal = document.getElementById("robot-topic-modal");
  modal.classList.remove("open");
  modal.setAttribute("aria-hidden", "true");
}

function renderRobotTopicModal() {
  const modal = document.getElementById("robot-topic-modal");
  if (!modal || !modal.classList.contains("open")) return;
  const agent = robotTopicModalAgent;
  const title = document.getElementById("robot-topic-modal-title");
  const body = document.getElementById("robot-topic-modal-body");
  if (agent === null || !state.robotTopics.has(agent)) {
    if (title) title.textContent = "Robot Topics";
    if (body) body.innerHTML = `<p class="muted">This robot is no longer reporting data.</p>`;
    return;
  }
  const now = Date.now();
  const color = palette[agent % palette.length];
  const topics = [...state.robotTopics.get(agent).entries()].sort((a, b) => a[0].localeCompare(b[0]));
  const newestAge = Math.min(...topics.map(([, entry]) => now - entry.receivedAt));
  const stale = newestAge > ROBOT_TOPIC_STALE_MS;
  const battery = realBatteryForAgent(agent);
  if (title) {
    title.innerHTML = `<span class="dot${stale ? " stale" : ""}" style="background:${color};display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:7px;"></span>` +
      `${escapeHtml(agvName(agent))}${battery !== null ? ` <span class="muted" style="font-size:13px;font-weight:400;">${battery}%</span>` : ""}`;
  }
  if (body) {
    const rows = topics.map(([topic, entry]) => {
      const ageSec = ((now - entry.receivedAt) / 1000).toFixed(1);
      return `<tr><td class="rt-topic">${escapeHtml(topic)}</td>` +
        `<td class="rt-value">${escapeHtml(entry.value)}</td>` +
        `<td class="rt-age">${ageSec}s ago</td></tr>`;
    }).join("");
    body.innerHTML = `<div class="robot-topic-card" style="border:none; padding:0; box-shadow:none;">` +
      `<table>${rows}</table></div>` +
      (stale ? `<p class="warning" style="margin-top:10px;">No update in over ${(ROBOT_TOPIC_STALE_MS / 1000).toFixed(0)}s — this robot may be offline.</p>` : "");
  }
}

function connectRealMode() {
  if (!("WebSocket" in window)) {
    state.realStatus = "Real mode needs browser WebSocket support";
    return;
  }
  if (state.realSocket && state.realSocket.readyState <= 1) return;
  state.realStatus = `Connecting to ${ROSBRIDGE_URL}`;
  const socket = new WebSocket(ROSBRIDGE_URL);
  state.realSocket = socket;
  socket.onopen = () => {
    state.realStatus = `Connected to ${ROSBRIDGE_URL}; subscribing to Alvik topics`;
    subscribeRealTopics(socket);
    render();
  };
  socket.onmessage = handleRealMessage;
  socket.onerror = () => {
    state.realStatus = `Real mode error: start rosbridge at ${ROSBRIDGE_URL}`;
    render();
  };
  socket.onclose = () => {
    if (liveDataMode()) state.realStatus = `Disconnected from ${ROSBRIDGE_URL}`;
    render();
  };
}

function setMode(mode) {
  state.mode = mode;
  if (liveDataMode()) {
    stopPlayback();
    connectRealMode();
  } else {
    disconnectRealMode();
    state.realStatus = "Simulated playback";
  }
  // Live mode: show the camera's annotated MJPEG view above the grid. The
  // <img> src is set only while Live is active so the stream connection is
  // dropped when not being watched.
  const panel = document.querySelector(".grid-panel");
  const feed = document.getElementById("live-feed");
  if (mode === "live") {
    panel.classList.add("live-on");
    if (!feed.src) {
      feed.onerror = () => {
        state.realStatus =
          `camera stream not reachable at ${LIVE_STREAM_URL} — run ` +
          "apriltag_localize.py --rosbridge --stream on this machine";
        feed.removeAttribute("src");
        panel.classList.remove("live-on");
        render();
      };
      feed.src = LIVE_STREAM_URL;
    }
  } else {
    panel.classList.remove("live-on");
    if (feed.src) feed.removeAttribute("src");
  }
  render();
}

function setDriveMode(mode) {
  state.driveMode = mode;
  render();
}

function toggleFuseTurns() {
  state.fuseTurns = !state.fuseTurns;
  render();
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function routeSequence(route) {
  return route.path.map(pathLabel).join("-");
}

function cloneVisit(bay) {
  // A visit is a bay plus an optional mode ("pick" = collect the processed
  // part; absent/"drop" = deliver). Everything that copies bay lists must
  // preserve the mode or RPD pickup legs silently turn into drop-offs.
  return bay.mode ? { r: bay.r, c: bay.c, mode: bay.mode } : { r: bay.r, c: bay.c };
}

// Collapse an adjacent same-bay drop -> pick into ONE "service" visit (stay in
// the dead-end bay through processing). Unmerged, the robot leaves the bay,
// about-faces at the next node, and re-enters — a banned mid-grid ROTATE_180
// (verified: unmerged RPD tours produce ~2100 mid-grid flips per 6000, merged
// produces ZERO). Applied defensively at the entry of buildAgentRoute so NO
// caller (external plan, import, re-solve, manual) can bypass it. Idempotent:
// an already-"service" visit is left as-is.
function mergeServiceVisits(visits) {
  return (visits || []).reduce((out, v) => {
    const prev = out[out.length - 1];
    if (prev && v.mode === "pick" && !prev.mode &&
        prev.r === v.r && prev.c === v.c) {
      prev.mode = "service";
    } else {
      out.push(cloneVisit(v));
    }
    return out;
  }, []);
}

function cloneBays(bays) {
  return bays.map(cloneVisit);
}

function rotateBays(bays, offset) {
  if (!bays.length) return [];
  const o = offset % bays.length;
  return [...bays.slice(o), ...bays.slice(0, o)].map(cloneVisit);
}

function blockOrderVariants(block) {
  const variants = [block];
  variants.push([...block].reverse());
  variants.push([...block].sort((a, b) => bayNumber(a.r, a.c) - bayNumber(b.r, b.c)));
  variants.push([...block].sort((a, b) => bayNumber(b.r, b.c) - bayNumber(a.r, a.c)));
  for (let i = 1; i < Math.min(block.length, 8); i++) {
    variants.push(rotateBays(block, i));
  }
  return variants;
}

function candidateOrders(route) {
  const base = cloneBays(route.bays);
  // Imported plans (external solver) own their visit order — possibly an
  // interleaved pickup-delivery sequence our block rules don't describe.
  // Never reshuffle them; only path modes / dwells may vary.
  if (route.pinnedOrder) return [base];
  // RPD tours are drop-block then pick-block (capacity: robots leave the
  // depot full). Permute within each block only — a pickup reordered ahead
  // of a drop-off would overload the robot.
  const drops = base.filter((bay) => bay.mode !== "pick");
  const picks = base.filter((bay) => bay.mode === "pick");
  const orders = [];
  if (picks.length) {
    for (const order of blockOrderVariants(drops)) orders.push([...order, ...picks]);
    for (const order of blockOrderVariants(picks)) orders.push([...drops, ...order]);
  } else {
    orders.push(...blockOrderVariants(base));
  }
  const seen = new Set();
  return orders.filter((order) => {
    const key = order.map((bay) => `${bayId(bay.r, bay.c)}:${bay.mode || "drop"}`).join("|");
    if (seen.has(key)) return false;
    seen.add(key);
    return true;
  });
}

async function rerouteAgent(agent) {
  if (!state.routes[agent]) return;
  stopPlayback();
  const currentEval = evaluateRoutes(state.routes);
  const status = document.getElementById("real-status");
  const modal = document.getElementById("route-modal");
  const modalOpen = modal.classList.contains("open");
  const report = document.getElementById("route-report");
  const previousStatus = state.realStatus;
  state.realStatus = `Optimizing AGV ${agent + 1} reroute...`;
  if (status) status.textContent = state.realStatus;
  if (modalOpen && report) {
    report.querySelectorAll("button").forEach((button) => { button.disabled = true; });
    report.insertAdjacentHTML("afterbegin", `<p class="muted"><b>Optimizing AGV ${agent + 1}...</b> Testing dwell waits and alternate edges within the bounded search.</p>`);
  }
  await new Promise((resolve) => setTimeout(resolve, 20));

  const best = polishCollisionPlan(state.routes, REROUTE_BUDGET_MS, agent, true);

  if (!isBetterPlanEval(best, { eval: currentEval })) {
    state.realStatus = previousStatus || "Simulated playback";
    render();
    if (modalOpen) renderReport();
    alert(`No lower-makespan or safer reroute found for AGV ${agent + 1} within the bounded search.`);
    return;
  }

  state.routes = best.routes;
  state.schedule = best.schedule || scheduleStatus(state.routes, detectConflicts(state.routes), 0, false);
  invalidateDispatchEditors();
  computeEdgeUse();
  state.conflicts = detectConflicts();
  refreshScheduleStatus();
  state.timestep = 0;
  state.realStatus = previousStatus || "Simulated playback";
  render();
  showReport();
}

function reportSummaryHtml() {
  return state.routes.map((route) => {
    const bays = routeText(route);
    return `
      <div class="stat">
        <b style="color:${palette[route.agent % palette.length]}">AGV ${route.agent + 1}</b>
        <span>${escapeHtml(bays)}</span>
        <div class="muted" style="margin-top:6px;">launch ${fmtSec(route.launchOffsetSec || 0)} | done ${fmtSec(route.durationSec || 0)}${route.extraWaitSec ? ` | launch wait ${fmtSec(route.extraWaitSec)}` : ""}${totalRouteDwell(route) ? ` | dwell ${fmtSec(totalRouteDwell(route))}` : ""}${route.pathMode && route.pathMode !== "vh" ? ` | path ${escapeHtml(route.pathMode)}` : ""}</div>
        <button class="secondary reroute-btn" data-agent="${route.agent}" style="margin-top:8px;">Reroute AGV ${route.agent + 1}</button>
        <button class="secondary edit-route-btn" data-agent="${route.agent}" style="margin-top:8px;">Edit on map</button>
      </div>`;
  }).join("");
}

function launchScheduleHtml() {
  const order = launchOrderForAgents(state.routes.length);
  const rows = order.map((agent) => {
    const route = state.routes[agent];
    const node0Event = (route.events || []).find((ev) => ev.node === 0);
    const node1Event = (route.events || []).find((ev) => ev.node === 1);
    return `
      <tr>
        <td>${agvName(agent)}</td>
        <td>${fmtSec(route.launchOffsetSec || 0)}</td>
        <td>${fmtSec(depotClearanceSec(agent))}</td>
        <td>${fmtSec(node0Event ? node0Event.arrivalSec : 0)}</td>
        <td>${fmtSec(node1Event ? node1Event.arrivalSec : 0)}</td>
      </tr>`;
  }).join("");
  return `
    <div class="table-wrap" style="max-height:220px;">
      <table>
        <thead><tr><th>AGV</th><th>launch</th><th>D# to 0</th><th>arrive 0</th><th>arrive 1</th></tr></thead>
        <tbody>${rows}</tbody>
      </table>
    </div>`;
}

function scheduleSafetyHtml() {
  if (!state.schedule) return "";
  const extraWaitRows = state.routes
    .filter((route) => (route.extraWaitSec || 0) > 0 || totalRouteDwell(route) > 0 || (route.pathMode && route.pathMode !== "vh"))
    .map((route) => `<tr><td>AGV ${route.agent + 1}</td><td>${fmtSec(route.extraWaitSec || 0)}</td><td>${fmtSec(totalRouteDwell(route))}</td><td>${escapeHtml(route.pathMode || "vh")}</td><td>${fmtSec(route.launchOffsetSec || 0)}</td></tr>`)
    .join("");
  const status = state.schedule.collisionFree
    ? "Collision-window schedule achieved: no two AGVs contend for the same edge or node inside the fatal time window."
    : `UNSAFE: ${state.schedule.fatalEdgeRemaining} fatal edge overlap(s) and ${state.schedule.fatalNodeRemaining} fatal node contention(s) remain after ${state.schedule.iterations} delay pass(es).`;
  return `
    <h3>Edge-Exclusive Safety</h3>
    <p class="muted">${escapeHtml(status)} ${state.schedule.nearMisses ? `${state.schedule.nearMisses} warning(s) remain outside the fatal window.` : `No ${fmtSec(EDGE_SAFETY_WINDOW_SEC)} edge contention warnings remain.`}</p>
    ${extraWaitRows ? `
      <div class="table-wrap" style="max-height:180px;">
        <table>
          <thead><tr><th>AGV</th><th>launch wait</th><th>dwell wait</th><th>path mode</th><th>final launch</th></tr></thead>
          <tbody>${extraWaitRows}</tbody>
        </table>
      </div>` : `<p class="muted">No automatic dwell or launch delay was needed beyond the measured depot launch sequence.</p>`}
  `;
}

function routeSequencesHtml() {
  return state.routes.map((route) => `
    <h3>AGV ${route.agent + 1}</h3>
    <div class="route-seq">${escapeHtml(routeSequence(route))}</div>
  `).join("");
}

function timelineHtml() {
  const maxT = Math.ceil(makespan());
  const headers = state.routes.map((route) => `<th>AGV ${route.agent + 1}</th>`).join("");
  let rows = "";
  for (let t = 0; t <= maxT; t++) {
    const cells = state.routes.map((route) => `<td>${escapeHtml(locationLabelAt(route, t))}</td>`).join("");
    rows += `<tr><td>${fmtSec(t)}</td>${cells}</tr>`;
  }
  return `
    <div class="table-wrap">
      <table>
        <thead><tr><th>time</th>${headers}</tr></thead>
        <tbody>${rows}</tbody>
      </table>
    </div>`;
}

function rerouteControlsHtml() {
  return `
    <div class="actions">
      ${state.routes.map((route) => `
        <button class="secondary reroute-btn" data-agent="${route.agent}">Reroute A${route.agent + 1}</button>
        <button class="secondary edit-route-btn" data-agent="${route.agent}">Edit A${route.agent + 1} on map</button>
      `).join("")}
    </div>`;
}

function conflictsHtml() {
  if (!state.conflicts.length) {
    return `<p class="muted">No fatal node contentions were detected inside the ${fmtSec(SAFETY_WINDOW_SEC)} collision window, and no edge contentions were detected inside the ${fmtSec(EDGE_SAFETY_WINDOW_SEC)} edge collision window.</p>${rerouteControlsHtml()}`;
  }
  return state.conflicts.slice(0, 120).map((conflict) => {
    const buttons = conflict.agents.map((agent) =>
      `<button class="secondary reroute-btn" data-agent="${agent}">Reroute A${agent + 1}</button>`
    ).join(" ");
    return `
      <div class="conflict-card">
        <strong>${escapeHtml(conflict.severity)}</strong>
        <span>t=${escapeHtml(fmtSec(conflict.timeSec || 0))} | ${escapeHtml(conflict.type)} | A${conflict.agents.map((a) => a + 1).join(", A")} | ${escapeHtml(conflict.detail)}</span>
        <span>${buttons}</span>
      </div>`;
  }).join("") + rerouteControlsHtml() + (state.conflicts.length > 120 ? `<p class="muted">Showing first 120 of ${state.conflicts.length} risks.</p>` : "");
}

function renderReport() {
  const fatalCount = state.conflicts.filter((c) => c.severity === "fatal").length;
  const nearCount = state.conflicts.length - fatalCount;
  const report = document.getElementById("route-report");
  report.innerHTML = `
    <h3>Assignment At A Glance</h3>
    <div class="report-grid">${reportSummaryHtml()}</div>
    <h3>Depot Launch Schedule</h3>
    <p class="muted">Launch order uses ${launchOrderForAgents(state.routes.length).map((agent) => agvName(agent)).join(" -> ")}. The D# to 0 column is your measured depot-to-red-sticker clearance.</p>
    ${launchScheduleHtml()}
    ${scheduleSafetyHtml()}
    <h3>Exact Node Routes</h3>
    <p class="muted">Grid nodes use 1..${state.rows * state.cols}. Workstation nodes use ${state.rows * state.cols + 1}..${state.rows * state.cols + totalBayCount()}. Entry nodes use ${state.rows * state.cols + totalBayCount() + 1}..${state.rows * state.cols + totalBayCount() * 2}. Depot clearances are measured seconds from D# to node 0.</p>
    ${routeSequencesHtml()}
    <h3>Time Comparison</h3>
    <p class="muted">Overall makespan: ${fmtSec(makespan())}. Collision risk: ${fatalCount} fatal, ${nearCount} near-miss.</p>
    ${timelineHtml()}
    <h3>Collision Risks</h3>
    ${conflictsHtml()}
  `;
  report.querySelectorAll(".reroute-btn").forEach((btn) => {
    btn.addEventListener("click", () => rerouteAgent(Number(btn.dataset.agent)));
  });
  report.querySelectorAll(".edit-route-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      state.editAgent = Number(btn.dataset.agent);
      state.editMode = true;
      closeReport();
      render();
    });
  });
}

function showReport() {
  if (!state.routes.length) return;
  renderReport();
  const modal = document.getElementById("route-modal");
  modal.classList.add("open");
  modal.setAttribute("aria-hidden", "false");
}

function closeReport() {
  const modal = document.getElementById("route-modal");
  modal.classList.remove("open");
  modal.setAttribute("aria-hidden", "true");
}

function buildExportPayload() {
  return {
    grid: { rows: state.rows, cols: state.cols },
    agents: state.agents,
    capacity: state.capacity,
    rpd: { enabled: state.rpdMode, process_sec: state.processSec },
    collision_window_sec: SAFETY_WINDOW_SEC,
    edge_collision_window_sec: EDGE_SAFETY_WINDOW_SEC,
    real_mode: {
      rosbridge_url: ROSBRIDGE_URL,
      topics: ALVIK_REAL_TOPICS.filter((item) => item.agent < state.agents).map((item) => ({
        agent: item.agent + 1,
        topic: item.topic,
        type: item.type,
      })),
    },
    depot: {
      junction_node: 0,
      grid_connection_node: 1,
      layout: "south of grid, single line, all slots enter through node 0 then node 1",
    },
    depot_slots: Array.from({ length: state.agents }, (_, i) => ({
      agent: i + 1,
      agv: agvName(i),
      depot_slot: `D${i + 1}`,
      depot_entry: `DE${i + 1}`,
      start_to_node0_sec: depotClearanceSec(i),
      launch_offset_sec: (state.routes[i] && state.routes[i].launchOffsetSec) || 0,
      path_to_grid: [...depotEntryPathToNode0(i).map(pathLabel), 1],
    })),
    workstations: [...state.selected].map(parseBayId).map((bay) => ({
      bay: bayNumber(bay.r, bay.c),
      // row 1 = bay row nearest the depot (bottom-up, matching the node
      // numbering and romeshprasad's workstations.json convention)
      row: state.rows - 1 - bay.r,
      col: bay.c + 1,
      north_entry_nodes: [nodeNumber(bay.r, bay.c), nodeNumber(bay.r, bay.c + 1)],
    })),
    routes: state.routes.map((route) => ({
      agent: route.agent + 1,
      agv: agvName(route.agent),
      bays: route.bays.map((bay) => bayNumber(bay.r, bay.c)),
      visits: route.bays.map((bay) => ({
        bay: bayNumber(bay.r, bay.c),
        mode: bay.mode || "drop",
      })),
      manual_waypoints: (route.manualWaypoints || []).map((node) => ({
        row: node.r + 1,
        col: node.c + 1,
        node: nodeNumber(node.r, node.c),
      })),
      launch_offset_sec: route.launchOffsetSec || 0,
      extra_wait_sec: route.extraWaitSec || 0,
      dwell_wait_sec: totalRouteDwell(route),
      path_mode: route.pathMode || "vh",
      wait_before_segment_sec: cloneWaits(route.waitBeforeSec || {}),
      duration_sec: route.durationSec || 0,
      path_nodes: route.path.map(pathLabel),
      node_events: (route.events || []).map((ev) => ({
        node: ev.node,
        path_index: ev.pathIndex || 0,
        arrival_sec: ev.arrivalSec,
        depart_sec: ev.departSec,
        dwell_sec: ev.dwellSec || 0,
        note: ev.note || ev.via || "",
        turn_sec: ev.turnSec || 0,
      })),
      segments: (route.segments || []).map((seg) => ({
        segment_index: seg.segmentIndex,
        from: seg.fromNode,
        to: seg.toNode,
        start_sec: seg.startSec,
        end_sec: seg.endSec,
        wait_before_sec: seg.waitBeforeSec || 0,
        move_sec: seg.moveSec,
        turn_sec: seg.turnSec,
        type: seg.type,
      })),
    })),
    makespan_sec: makespan(),
    edge_exclusive_schedule: state.schedule ? {
      collision_free: state.schedule.collisionFree,
      edge_exclusive: state.schedule.edgeExclusive,
      auto_delay_iterations: state.schedule.iterations,
      capped: state.schedule.capped,
      fatal_edge_remaining: state.schedule.fatalEdgeRemaining,
      fatal_node_remaining: state.schedule.fatalNodeRemaining,
      near_misses: state.schedule.nearMisses,
      extra_wait_by_agent: state.schedule.extraWaitByAgent.map((item) => ({
        agent: item.agent + 1,
        extra_wait_sec: item.seconds,
        dwell_wait_sec: item.dwellSeconds || 0,
      })),
    } : null,
    conflicts: state.conflicts.map((conflict) => ({
      type: conflict.type,
      severity: conflict.severity,
      time_sec: conflict.timeSec || 0,
      agents: conflict.agents.map((agent) => agent + 1),
      edge: conflict.edgeKey || null,
      overlap_start_sec: typeof conflict.overlapStartSec === "number" ? conflict.overlapStartSec : null,
      overlap_end_sec: typeof conflict.overlapEndSec === "number" ? conflict.overlapEndSec : null,
      gap_sec: typeof conflict.gapSec === "number" ? conflict.gapSec : null,
      detail: conflict.detail,
    })),
  };
}

function downloadJson() {
  const payload = buildExportPayload();
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = exportFileName();
  a.click();
  URL.revokeObjectURL(url);
}

// {route mode}_{processing time}s_{timestamp}.json, e.g.
// "ext-brkga_60s_20260722-091500.json" — mode + RPD process time in the name
// so repeated downloads (different modes/settings) don't overwrite each other
// or need opening to tell apart.
function exportFileName() {
  const modeSel = document.getElementById("route-mode");
  const mode = (modeSel && modeSel.value ? modeSel.value : "plan").replace(/[^a-z0-9]+/gi, "-");
  const secs = state.rpdMode ? `_${state.processSec}s` : "";
  const ts = new Date().toISOString().replace(/[:.]/g, "-").replace("T", "_").slice(0, 19);
  return `${mode}${secs}_${ts}.json`;
}

// ===================== Live dispatch (no route files) ======================
// Executes the solved plan directly from this page over rosbridge: publishes
// AGV_Factory_color_pose.ino commands to <Name>_cmd, watches <Name>_status for done markers,
// and gates every command behind the same deterministic conflict math the
// off-robot dispatcher uses, anchored to live
// completion times and vision positions. Warn/hold only ever inserts DWELLs;
// commands are never reordered. advisor advising stays in the Python path.

const DISPATCH_DONE_MARKERS = ["IDLE", "STOPPED", "POSE_RESET", "ERROR"];
const DWELL_MIN_MS = 200;
const DWELL_MAX_MS = 30000;
const DISPATCH_HOLD_MS = 2000;          // fallback hold (AGV_Factory_color_pose.ino default dwell)
const DISPATCH_TICK_MS = 250;
const VISION_FRESH_DISPATCH_MS = 1500;  // vision this old is ignored by the gate
const VISION_TARGET_BLOCK_CELLS = 0.6;  // another robot this close to my next node = hold
const VISION_OFFROUTE_CELLS = 0.75;     // my vision vs expected node farther than this = hold

const DISPATCH_MOVE_COLORS = {
  FORWARD_UNTIL_RED: ["RED"], FORWARD_UNTIL_YELLOW: ["YELLOW"],
  FORWARD_UNTIL_BLUE: ["BLUE"], FORWARD_UNTIL_COLOR: ["RED", "YELLOW", "BLUE"],
  BACKWARD_UNTIL_COLOR: ["RED", "YELLOW", "BLUE"],
  BACKWARD_UNTIL_YELLOW: ["YELLOW"], BACKWARD_UNTIL_BLUE: ["BLUE"],
};
const DISPATCH_TURN_SEC = {
  RIGHT_UNTIL_COLOR: TURN_SEC.RIGHT, LEFT_UNTIL_COLOR: TURN_SEC.LEFT,
  ROTATE_180: TURN_SEC.ROTATE_180,
};
const DISPATCH_NOOPS = ["STOP", "RESET_POSE", "GET_STATUS"];

function dispatchMarkerColor(label) {
  // Physical marker layout (table re-stickered; node 0 changed to BLUE by
  // the user 2026-07-14 — it belongs to the depot, not the grid): grid
  // nodes 1..N = RED; node 0, depot-lane junctions (DE#) and parking
  // stickers (D#) = BLUE; entries/workstations = YELLOW.
  if (typeof label === "string") return "BLUE";
  const n = Number(label);
  if (n === 0) return "BLUE";
  if (n >= 1 && n <= state.rows * state.cols) return "RED";
  return "YELLOW";
}

// label -> vision grid cell (cells relative to node 1), for vision checks.
// Grid nodes are exact; entries/workstations are approximate cell centers;
// node 0 uses its measured 13.5in offset below node 1.
function dispatchLabelToCell(label) {
  if (typeof label === "string") return null;  // depot lane: skip checks
  const n = Number(label);
  if (n === 0) return { gx: 0, gy: -1.35 };
  const nodes = state.rows * state.cols;
  if (n >= 1 && n <= nodes) {
    const idx = n - 1;
    return { gx: idx % state.cols, gy: Math.floor(idx / state.cols) };
  }
  const bayNum = n <= nodes + totalBayCount() ? n - nodes : n - nodes - totalBayCount();
  const bIdx = bayNum - 1;
  // bay rows count bottom-up (row 0 nearest the depot); a bay's entry sits
  // on its north node row, the workstation half a cell south of it
  const rowUp = Math.floor(bIdx / (state.cols - 1));
  const c = bIdx % (state.cols - 1);
  const gy = (rowUp + 1) - (n <= nodes + totalBayCount() ? 0.5 : 0.15);
  return { gx: c + 0.5, gy };
}

// ---- default command text generated from a solved route -------------------
function turnCommandBetween(prevHeading, nextHeading) {
  const order = ["N", "E", "S", "W"];
  const a = order.indexOf(prevHeading);
  const b = order.indexOf(nextHeading);
  if (a < 0 || b < 0 || a === b) return null;
  const delta = (b - a + 4) % 4;
  if (delta === 1) return "RIGHT_UNTIL_COLOR";
  if (delta === 3) return "LEFT_UNTIL_COLOR";
  return "ROTATE_180";
}

function dwellCommandLines(seconds, out) {
  let ms = Math.round(seconds * 1000);
  while (ms >= DWELL_MIN_MS) {
    const chunk = Math.min(DWELL_MAX_MS, ms);
    out.push(`DWELL ${chunk}`);
    ms -= chunk;
  }
}

// Command generation validated against a proven hand-written Alvik1 route
// file (2026-07-12). Depot legs use fixed templates because the physical
// depot geometry is mirrored on screen:
//   exit  (robot parked on its blue sticker facing south):
//     FWD_BLUE (down stub) . RIGHT . FWD_BLUE x agent (junctions west) .
//     FWD_RED (node 0 is red) . RIGHT . FWD_RED (node 1)
//   return: FWD_BLUE (rides the node-0 corner to the first junction) .
//     FWD_BLUE x agent (east) . LEFT . FWD_BLUE (up stub) . ROTATE_180
// Grid/bay legs walk the timed segments generically; leaving a workstation
// emits ROTATE_180 (heading flip) then DWELL for the service pause.
// Command generation, returning one entry per emitted line tagged with the
// route segment it belongs to (segIndex = seg.segmentIndex, or null for the
// depot exit/return templates that don't correspond to a single timed
// segment). The Plan & Simulate command panel uses segIndex to highlight the
// line the robot is executing at the current playback time; generateCommandText
// joins .text for dispatch/mission-send, so on-wire output is unchanged.
//
// _midGridFlipWarned (2026-08-20): renderSimCommands() calls this function
// fresh on every render() -- correctly, since the returned .text/segIndex
// per-line highlighting depends on the live playback timestep, not just the
// route. But render() itself fires on EVERY incoming rosbridge pose message
// in Real/Live mode (handleRealMessage() -> render()), which at 6 robots x
// ~59Hz (post 2026-08-18 throughput work) is 300+ calls/sec -- each one
// re-running the mid-grid-flip console.warn below for a route that hasn't
// actually changed, flooding the console (confirmed: ~5000 warnings in 10s)
// and burying real errors (in this case, a live camera-stream connection
// failure someone was trying to diagnose). The in-panel warning banner
// (out.unshift() below) stays UNCONDITIONAL -- that's real, useful UI
// feedback and needs to reflect the current route every render. Only the
// console.warn is deduped, keyed by robot+flip-count, so it fires once per
// distinct warning state instead of once per pose tick.
const _midGridFlipWarned = new Map();
function generateCommandLines(route) {
  const out = [];
  const push = (text, segIndex = null, warn = false) => out.push({ text, segIndex, warn });
  const segs = route.segments;
  let i = 0;

  if (segs.length && String(segs[0].type).startsWith("DEPOT")) {
    // Tag the whole depot-exit template with the first depot segment's index so
    // the panel lights these lines while the robot is still leaving the depot.
    const de = segs[0].segmentIndex;
    push("FORWARD_UNTIL_BLUE", de); push("RIGHT_UNTIL_COLOR", de);
    for (let k = 0; k < route.agent; k++) push("FORWARD_UNTIL_BLUE", de);
    push("FORWARD_UNTIL_BLUE", de); push("RIGHT_UNTIL_COLOR", de); push("FORWARD_UNTIL_RED", de);
    while (i < segs.length &&
           (String(segs[i].type).startsWith("DEPOT") || segs[i].type === "BLUE_TO_RED")) i++;
  }

  let prevHeading = "N";  // northbound after node0 -> node1
  let midGridFlips = 0;   // ROTATE_180 belongs at workstations/depot ONLY
  for (; i < segs.length; i++) {
    const seg = segs[i];
    const si = seg.segmentIndex;
    if (seg.type === "RED_TO_BLUE_RETURN") {
      const turn = turnCommandBetween(prevHeading, "S");
      if (turn) push(turn, si);
      push("FORWARD_UNTIL_BLUE", si); push("LEFT_UNTIL_COLOR", si);
      for (let k = 0; k < route.agent + 1; k++) push("FORWARD_UNTIL_BLUE", si);
      push("LEFT_UNTIL_COLOR", si); push("FORWARD_UNTIL_BLUE", si); push("ROTATE_180", si);
      break;
    }
    // arrivingAtWork (2026-08-19, real hardware failure: 3/6 robots drove
    // off-tape because they moved forward into the next leg still facing
    // the workstation's ENTRY heading): every workstation's only
    // entry/exit is its Northern segment, so a robot always arrives
    // facing north (0deg) and the exit trajectory is always due south
    // (180deg) -- this is a fixed physical fact of the layout, not
    // something to derive from turnCommandBetween(prevHeading, seg.heading)
    // (that comparison depends on where the route goes AFTER the
    // workstation and returned null/no-turn whenever that happened to
    // match the entry heading, silently skipping the flip). The rotation
    // must happen on ARRIVAL, before the drop-off/pick-up DWELL begins --
    // NOT deferred to the departure leg -- so the robot is never sitting
    // mid-service facing the wrong way, and departs already square with
    // its exit heading (no second turn needed).
    const arrivingAtWork = seg.to && seg.to.kind === "workstation";
    // ROOT-CAUSED 2026-08-24: a bay is a dead end -- its only entry/exit is
    // the Northern segment -- so LEAVING a workstation always requires
    // retracing that same segment, which is a real, physically-necessary
    // 180 (arrive facing S per the fixed exit heading below, then must
    // face N again to walk back out to the grid). That turn was being
    // counted as a "mid-grid flip" (the open-lattice doubling-back defect
    // this counter exists to catch) even though it has nothing to do with
    // that defect -- confirmed on a real single-robot, 12-workstation
    // mission: every one of the 12 warned "flips" was exactly this
    // departure turn, not an actual route defect. leavingWork exempts it
    // the same way arrivingAtWork already exempts the arrival turn.
    const leavingWork = seg.from && seg.from.kind === "workstation";
    if (seg.waitBeforeSec > 0) {
      const dw = []; dwellCommandLines(seg.waitBeforeSec, dw); dw.forEach((l) => push(l, si));
    }
    const turn = turnCommandBetween(prevHeading, seg.heading);
    if (turn) {
      if (turn === "ROTATE_180" && !arrivingAtWork && !leavingWork) midGridFlips++;
      push(turn, si);
    }
    push(`FORWARD_UNTIL_${dispatchMarkerColor(seg.toNode)}`, si);
    if (arrivingAtWork) {
      push("ROTATE_180", si);  // square up to the fixed exit heading BEFORE service
      push("DWELL", si);       // service pause (AGV_Factory_color_pose.ino default 2000 ms)
      prevHeading = "S";       // now facing the exit heading; departure leg needs no turn
    } else {
      prevHeading = seg.heading;
    }
  }
  if (midGridFlips > 0) {
    if (_midGridFlipWarned.get(route.agent) !== midGridFlips) {
      _midGridFlipWarned.set(route.agent, midGridFlips);
      console.warn(`${agvName(route.agent)}: ${midGridFlips} mid-grid ROTATE_180 — route doubles back`);
    }
    out.unshift({
      text: `# WARNING: ${midGridFlips} mid-grid ROTATE_180 — route doubles back; fix the route before starting`,
      segIndex: null, warn: true,
    });
  } else if (_midGridFlipWarned.has(route.agent)) {
    _midGridFlipWarned.delete(route.agent);
  }
  return out;
}

function generateCommandText(route) {
  return generateCommandLines(route).map((l) => l.text).join("\n");
}

// ---- alignment: command text -> plan items (port of align_commands_to_route)
function alignCommandsToRoute(route, commandLines) {
  const segs = [...route.segments].sort((x, y) => x.segmentIndex - y.segmentIndex);
  const plan = [];
  let si = 0;
  let pendingTurnIdxs = [];
  const segDuration = (seg) => (seg.moveSec || 0) + (seg.turnSec || 0);

  for (const raw of commandLines) {
    const cmd = raw.trim().toUpperCase();
    if (!cmd || cmd.startsWith("#")) continue;
    const base = cmd.split(/\s+/)[0];

    if (base in DISPATCH_TURN_SEC) {
      plan.push({ command: cmd, kind: "turn", durationSec: DISPATCH_TURN_SEC[base] });
      pendingTurnIdxs.push(plan.length - 1);
      continue;
    }
    if (base === "DWELL") {
      const ms = parseInt(cmd.split(/\s+/)[1], 10);
      const clamped = Number.isFinite(ms) && ms > 0
        ? Math.max(DWELL_MIN_MS, Math.min(DWELL_MAX_MS, ms)) : DISPATCH_HOLD_MS;
      plan.push({ command: cmd, kind: "dwell", durationSec: clamped / 1000 });
      continue;
    }
    if (DISPATCH_NOOPS.includes(cmd)) {
      plan.push({ command: cmd, kind: "noop", durationSec: 0 });
      continue;
    }
    if (base in DISPATCH_MOVE_COLORS) {
      const colors = DISPATCH_MOVE_COLORS[base];
      let j = si;
      let implicit = [];
      let matched = null;
      while (j < segs.length) {
        const seg = segs[j];
        // Color match first: with node 0 blue, both the exit hop into node 0
        // and the returning node1->node0 hop match FORWARD_UNTIL_BLUE 1:1.
        if (colors.includes(dispatchMarkerColor(seg.toNode))) {
          matched = seg; j += 1; break;
        }
        const isDepot = String(seg.type || "").startsWith("DEPOT");
        if (isDepot) { implicit.push(seg); j += 1; continue; }
        break;
      }
      if (!matched && implicit.length) {
        matched = implicit[0];
        implicit = [];
        j = si + 1;
      }
      if (!matched) {
        const nxt = si < segs.length ? segs[si] : null;
        throw new Error(
          `${route ? agvName(route.agent) : "?"}: command #${plan.length + 1} '${cmd}' has no ` +
          `matching schedule segment (next: ${nxt ? `${nxt.fromNode}->${nxt.toNode} (${nxt.type})` : "none left"}). ` +
          `Expected destination marker in [${colors.join(", ")}].`);
      }
      si = j;
      let moveSec = matched.moveSec || 0;
      const turnSec = matched.turnSec || 0;
      if (pendingTurnIdxs.length === 1 && turnSec > 0) {
        plan[pendingTurnIdxs[0]].durationSec = turnSec;
      } else if (!pendingTurnIdxs.length && turnSec > 0) {
        moveSec += turnSec;
      }
      pendingTurnIdxs = [];
      plan.push({
        command: cmd, kind: "move", durationSec: moveSec,
        fromNode: String(matched.fromNode), toNode: String(matched.toNode),
        toLabel: matched.toNode, segmentIndex: matched.segmentIndex,
        implicitSegs: implicit.map((s) => [String(s.fromNode), String(s.toNode), segDuration(s)]),
      });
      continue;
    }
    throw new Error(`unknown command '${cmd}'`);
  }
  return plan;
}

// ---- fleet simulation + conflict detection -------------------------------
function dEdgeKey(a, b) {
  return [String(a), String(b)].sort().join("<>");
}

function simulatePlanItems(plan, startIndex, anchorSec, leadDwellMs = 0) {
  const events = [];
  const intervals = [];
  let t = anchorSec + leadDwellMs / 1000;
  let pendingTurnStart = null;
  for (let idx = startIndex; idx < plan.length; idx++) {
    const item = plan[idx];
    if (item.kind === "turn") {
      if (pendingTurnStart === null) pendingTurnStart = t;
      t += item.durationSec;
    } else if (item.kind === "dwell" || item.kind === "noop") {
      t += item.durationSec;
    } else if (item.kind === "move") {
      for (const [f, to, dur] of item.implicitSegs || []) {
        const s0 = t;
        t += dur;
        intervals.push({ from: f, to, key: dEdgeKey(f, to), startSec: s0, endSec: t, planIndex: idx });
        events.push({ node: to, timeSec: t, planIndex: idx });
      }
      const s0 = pendingTurnStart !== null ? pendingTurnStart : t;
      t += item.durationSec;
      intervals.push({
        from: item.fromNode, to: item.toNode, key: dEdgeKey(item.fromNode, item.toNode),
        startSec: s0, endSec: t, planIndex: idx,
      });
      events.push({ node: item.toNode, timeSec: t, planIndex: idx });
      pendingTurnStart = null;
    }
  }
  return { events, intervals, endSec: t };
}

function detectDispatchConflicts(fleet) {
  const names = Object.keys(fleet);
  const out = [];
  for (let i = 0; i < names.length; i++) {
    for (let j = i + 1; j < names.length; j++) {
      const [a, b] = [names[i], names[j]];
      for (const ea of fleet[a].events) {
        for (const eb of fleet[b].events) {
          if (ea.node !== eb.node) continue;
          const gap = Math.abs(ea.timeSec - eb.timeSec);
          if (gap <= SAFETY_WINDOW_SEC) {
            out.push({
              isEdge: false, key: ea.node, robots: [a, b], gapSec: gap,
              timeSec: Math.min(ea.timeSec, eb.timeSec),
              detail: `node ${ea.node}: ${a}@${ea.timeSec.toFixed(1)}s vs ${b}@${eb.timeSec.toFixed(1)}s`,
              info: { [a]: { time: ea.timeSec, planIndex: ea.planIndex },
                      [b]: { time: eb.timeSec, planIndex: eb.planIndex } },
            });
          }
        }
      }
      for (const ia of fleet[a].intervals) {
        for (const ib of fleet[b].intervals) {
          if (ia.key !== ib.key || ia.from === ia.to) continue;
          const overlap = Math.max(ia.startSec, ib.startSec) < Math.min(ia.endSec, ib.endSec);
          const gap = overlap ? 0 :
            Math.max(ia.startSec, ib.startSec) - Math.min(ia.endSec, ib.endSec);
          if (overlap || gap <= EDGE_SAFETY_WINDOW_SEC) {
            out.push({
              isEdge: true, key: ia.key, robots: [a, b], gapSec: gap,
              timeSec: overlap ? Math.max(ia.startSec, ib.startSec) : Math.min(ia.endSec, ib.endSec),
              detail: `edge ${ia.key}: ${a} ${ia.startSec.toFixed(1)}-${ia.endSec.toFixed(1)}s vs ` +
                      `${b} ${ib.startSec.toFixed(1)}-${ib.endSec.toFixed(1)}s`,
              info: { [a]: { start: ia.startSec, end: ia.endSec, planIndex: ia.planIndex },
                      [b]: { start: ib.startSec, end: ib.endSec, planIndex: ib.planIndex } },
            });
          }
        }
      }
    }
  }
  out.sort((x, y) => x.timeSec - y.timeSec);
  return out;
}

// ---- dispatch engine -------------------------------------------------------
const dispatch = { active: false, paused: false, timer: null, t0: 0, robots: {} };

function dispatchNowSec() {
  return (performance.now() - dispatch.t0) / 1000;
}

function dispatchLog(text) {
  const pre = document.getElementById("dispatch-log");
  pre.style.display = "block";
  const stamp = dispatch.active ? `t=${dispatchNowSec().toFixed(1)}s ` : "";
  pre.textContent = `${stamp}${text}\n` + pre.textContent.split("\n").slice(0, 250).join("\n");
}

function dispatchSetStatus(text) {
  document.getElementById("dispatch-status").textContent = text;
}

function dispatchPublish(topic, data) {
  if (!state.realSocket || state.realSocket.readyState !== 1) return false;
  state.realSocket.send(JSON.stringify({ op: "publish", topic, msg: { data } }));
  return true;
}

function dispatchVisionCell(agent) {
  const entry = state.visionPositions.get(agent);
  if (!entry || Date.now() - entry.receivedAt > VISION_FRESH_DISPATCH_MS) return null;
  if (!Number.isFinite(entry.gridX) || !Number.isFinite(entry.gridY)) return null;
  return { gx: entry.gridX, gy: entry.gridY };
}

function dispatchAnchor(rob, nowSec) {
  if (!rob.launched && rob.pending === null) {
    return [0, Math.max(nowSec, rob.launchOffsetSec)];
  }
  if (rob.pending !== null) {
    const anchor = rob.lastDoneSec !== null ? rob.lastDoneSec : rob.launchOffsetSec;
    if (rob.pending.kind === "plan") return [rob.pending.index, anchor];
    return [rob.nextIndex, anchor + rob.pending.ms / 1000];
  }
  const anchor = rob.lastDoneSec !== null ? rob.lastDoneSec : nowSec;
  return [rob.nextIndex, Math.max(anchor, nowSec)];
}

function dispatchSimulateFleet(nowSec, decidingName = null, leadDwellMs = 0) {
  const fleet = {};
  for (const [name, rob] of Object.entries(dispatch.robots)) {
    if (rob.phase === "error") continue;
    const [start, anchor] = dispatchAnchor(rob, nowSec);
    fleet[name] = simulatePlanItems(
      rob.plan, start, anchor, name === decidingName ? leadDwellMs : 0);
  }
  return fleet;
}

function dispatchNextMotionEnd(rob) {
  for (let i = rob.nextIndex; i < rob.plan.length; i++) {
    if (rob.plan[i].kind === "move") return i;
  }
  return null;
}

// Remaining work (seconds of plan left), independent of how long the robot
// has been holding — the deadlock-priority metric: the robot with MORE work
// left proceeds through a mutual conflict, the other keeps dwelling.
function dispatchRemainingWork(rob) {
  const start = rob.pending && rob.pending.kind === "plan"
    ? rob.pending.index : rob.nextIndex;
  let total = 0;
  for (let i = start; i < rob.plan.length; i++) {
    const item = rob.plan[i];
    total += item.durationSec || 0;
    for (const seg of item.implicitSegs || []) total += seg[2] || 0;
  }
  return total;
}

// True while the robot is physically translating (an in-flight move item);
// dwelling, turning in place, or idling counts as stationary.
function dispatchIsMoving(rob) {
  if (rob.phase !== "busy" || !rob.pending || rob.pending.kind !== "plan") return false;
  const item = rob.plan[rob.pending.index];
  return !!item && item.kind === "move";
}

function dispatchNeededDwellSec(name, conflict) {
  const other = conflict.robots[0] === name ? conflict.robots[1] : conflict.robots[0];
  const mine = conflict.info[name];
  const theirs = conflict.info[other];
  if (conflict.isEdge) {
    return theirs.end + EDGE_SAFETY_WINDOW_SEC + EDGE_RELEASE_BUFFER_SEC - mine.start;
  }
  return theirs.time + SAFETY_WINDOW_SEC + EDGE_RELEASE_BUFFER_SEC - mine.time;
}

function dispatchMyConflicts(name, nowSec, leadDwellMs) {
  const rob = dispatch.robots[name];
  const windowEnd = dispatchNextMotionEnd(rob);
  if (windowEnd === null) return [];
  const fleet = dispatchSimulateFleet(nowSec, name, leadDwellMs);
  return detectDispatchConflicts(fleet).filter((c) =>
    c.robots.includes(name) && c.info[name].planIndex <= windowEnd);
}

function dispatchSend(rob, command, pending, note) {
  if (!dispatchPublish(`${rob.name}_cmd`, command)) {
    dispatchSetStatus("rosbridge socket closed — reconnect Real mode");
    return;
  }
  rob.pending = pending;
  rob.skipNextIdle = true;
  rob.sentAtSec = dispatchNowSec();
  rob.phase = "busy";
  dispatchLog(`[${rob.name}] -> ${command}${note ? `  (${note})` : ""}`);
}

function dispatchDecide(rob, nowSec) {
  if (rob.nextIndex >= rob.plan.length) {
    rob.phase = "done";
    dispatchLog(`[${rob.name}] plan complete`);
    return;
  }
  const nextItem = rob.plan[rob.nextIndex];

  // Vision gate 1: am I where the plan thinks I am? (grid labels only)
  const myCell = dispatchLabelToCell(rob.currentNode);
  const myVision = dispatchVisionCell(rob.agent);
  if (myCell && myVision) {
    const off = Math.hypot(myVision.gx - myCell.gx, myVision.gy - myCell.gy);
    if (off > VISION_OFFROUTE_CELLS) {
      dispatchSend(rob, `DWELL ${DISPATCH_HOLD_MS}`, { kind: "dwell", ms: DISPATCH_HOLD_MS },
        `HOLD: vision says grid=(${myVision.gx.toFixed(2)},${myVision.gy.toFixed(2)}) but plan expects node ${rob.currentNode} — off-route ${off.toFixed(2)} cells`);
      rob.holdCount += 1;
      return;
    }
  }

  // Vision gate 2: is another robot sitting on my next move's target?
  if (nextItem.kind === "move") {
    const target = dispatchLabelToCell(nextItem.toLabel);
    if (target) {
      for (const [otherName, other] of Object.entries(dispatch.robots)) {
        if (otherName === rob.name) continue;
        const v = dispatchVisionCell(other.agent);
        if (!v) continue;
        const d = Math.hypot(v.gx - target.gx, v.gy - target.gy);
        if (d < VISION_TARGET_BLOCK_CELLS) {
          dispatchSend(rob, `DWELL ${DISPATCH_HOLD_MS}`, { kind: "dwell", ms: DISPATCH_HOLD_MS },
            `HOLD: ${otherName} is ${d.toFixed(2)} cells from my next node ${nextItem.toNode} (vision)`);
          rob.holdCount += 1;
          return;
        }
      }
    }
  }

  // Deterministic predicted-conflict gate.
  // Deadlock breaker (added after the live 2-robot mutual-hold 2026-07-13):
  // if the conflicting robot is currently stationary and has LESS remaining
  // work than us, WE proceed and IT keeps yielding at its own gate — the
  // longer-route robot moves first. The vision gates above still veto if the
  // other robot physically sits on our next node.
  const allMine = dispatchMyConflicts(rob.name, nowSec, 0);
  const myWork = dispatchRemainingWork(rob);
  const mine = allMine.filter((c) => {
    const otherName = c.robots.find((r) => r !== rob.name);
    const other = dispatch.robots[otherName];
    if (!other || other.phase === "error" || other.phase === "done") return false;
    if (dispatchIsMoving(other)) return true;  // it's moving: conflict stands
    const otherWork = dispatchRemainingWork(other);
    const iHavePriority = myWork > otherWork + 1e-9 ||
      (Math.abs(myWork - otherWork) <= 1e-9 && rob.agent < other.agent);
    return !iHavePriority;
  });
  if (!mine.length) {
    rob.lastHoldSig = null;
    rob.holdCount = 0;
    dispatchSend(rob, nextItem.command,
      { kind: "plan", index: rob.nextIndex },
      allMine.length
        ? `step ${rob.nextIndex + 1}/${rob.plan.length} — priority over yielding robot(s)`
        : `step ${rob.nextIndex + 1}/${rob.plan.length}`);
    return;
  }
  const sig = mine.map((c) =>
    `${c.isEdge ? "e" : "n"}:${c.key}:${c.robots.find((r) => r !== rob.name)}`).sort().join("|");
  if (rob.lastHoldSig === sig) {
    dispatchSend(rob, `DWELL ${DISPATCH_HOLD_MS}`, { kind: "dwell", ms: DISPATCH_HOLD_MS },
      "HOLD repeat (same predicted conflict)");
    rob.holdCount += 1;
    return;
  }
  const needed = Math.max(...mine.map((c) => dispatchNeededDwellSec(rob.name, c)));
  const baseMs = Math.max(DWELL_MIN_MS,
    Math.min(DWELL_MAX_MS, Math.ceil(Math.max(needed, DWELL_MIN_MS / 1000) * 10) * 100));
  const candidates = [...new Set([
    baseMs, Math.min(DWELL_MAX_MS, baseMs + 1000),
    Math.min(DWELL_MAX_MS, baseMs + 2000), DISPATCH_HOLD_MS,
  ])].sort((a, b) => a - b);
  for (const ms of candidates) {
    if (!dispatchMyConflicts(rob.name, nowSec, ms).length) {
      dispatchSend(rob, `DWELL ${ms}`, { kind: "dwell", ms },
        `HOLD ${ms}ms clears: ${mine[0].detail}`);
      rob.holdCount += 1;
      return;
    }
  }
  rob.lastHoldSig = sig;
  dispatchSend(rob, `DWELL ${DISPATCH_HOLD_MS}`, { kind: "dwell", ms: DISPATCH_HOLD_MS },
    `HOLD fallback (no clean dwell): ${mine[0].detail}`);
  rob.holdCount += 1;
}

function dispatchOnStatus(name, text) {
  const rob = dispatch.robots[name];
  if (!rob) return;
  rob.lastStatus = text;
  if (text.startsWith("ERROR")) {
    rob.phase = "error";
    rob.pending = null;
    dispatchLog(`[${name}] ERROR from robot: ${text}`);
    dispatchRenderRobots();
    return;
  }
  if (rob.skipNextIdle && text === "IDLE") {
    rob.skipNextIdle = false;
    return;
  }
  rob.skipNextIdle = false;
  const isDone = DISPATCH_DONE_MARKERS.some((m) => text === m || text.startsWith(`${m} `));
  if (!isDone || rob.phase !== "busy" || rob.pending === null) return;
  if (rob.pending.kind === "plan") {
    const item = rob.plan[rob.pending.index];
    if (item.kind === "move") rob.currentNode = item.toLabel;
    rob.nextIndex = rob.pending.index + 1;
  }
  rob.pending = null;
  rob.lastDoneSec = dispatchNowSec();
  rob.launched = true;
  rob.phase = rob.nextIndex >= rob.plan.length ? "done" : "idle";
  if (rob.phase === "done") dispatchLog(`[${name}] plan complete`);
  dispatchRenderRobots();
}

function dispatchTick() {
  if (!dispatch.active || dispatch.paused) return;
  const nowSec = dispatchNowSec();
  const robs = Object.values(dispatch.robots);

  for (const rob of robs) {
    if (rob.phase === "waiting" && nowSec >= rob.launchOffsetSec) rob.phase = "idle";
    if (rob.phase === "busy" && nowSec - rob.sentAtSec > 90) {
      rob.phase = "error";
      dispatchLog(`[${rob.name}] timed out (90s) waiting for done; excluded`);
    }
  }

  // Issue order: longest remaining work first, so when two idle robots
  // conflict, the shorter-makespan robot reaches its gate second and is the
  // one that dwells.
  const idle = robs.filter((r) => r.phase === "idle")
    .sort((a, b) => dispatchRemainingWork(b) - dispatchRemainingWork(a));
  for (const rob of idle) dispatchDecide(rob, dispatchNowSec());

  if (robs.length && robs.every((r) => r.phase === "done" || r.phase === "error")) {
    dispatch.active = false;
    clearInterval(dispatch.timer);
    dispatchSetStatus("run finished");
    dispatchLog("dispatch run finished");
    dispatchButtons();
  }
  dispatchRenderRobots();
}

function dispatchRenderRobots() {
  const div = document.getElementById("dispatch-robot-status");
  if (!Object.keys(dispatch.robots).length) { div.textContent = ""; return; }
  div.textContent = Object.values(dispatch.robots).map((rob) => {
    const v = dispatchVisionCell(rob.agent);
    return `${rob.name}  ${rob.phase.padEnd(7)} step ${Math.min(rob.nextIndex + 1, rob.plan.length)}/${rob.plan.length}` +
      `  node=${rob.currentNode}  holds=${rob.holdCount}` +
      (v ? `  vision=(${v.gx.toFixed(2)},${v.gy.toFixed(2)})` : "  vision=stale") +
      (rob.lastStatus ? `  [${rob.lastStatus}]` : "");
  }).join("\n");
}

function dispatchButtons() {
  document.getElementById("dispatch-start").disabled =
    dispatch.active || !document.querySelectorAll("#dispatch-editors textarea").length;
  document.getElementById("dispatch-hold").disabled = !dispatch.active;
  document.getElementById("dispatch-abort").disabled = !dispatch.active;
  document.getElementById("dispatch-hold").textContent = dispatch.paused ? "Resume" : "Hold";
}

// ---- mission handoff to the Linux fleet supervisor (--listen mode) --------
function supPublish(topic, data) {
  if (!state.realSocket || state.realSocket.readyState !== 1) {
    document.getElementById("sup-status").textContent = "connecting to rosbridge — retry once connected";
    connectRealMode();
    return false;
  }
  if (!state.realSocket._supAdvertised) {
    for (const t of ["fleet_mission", "fleet_control"]) {
      state.realSocket.send(JSON.stringify(
        { op: "advertise", topic: t, type: "std_msgs/String" }));
    }
    state.realSocket._supAdvertised = true;
  }
  state.realSocket.send(JSON.stringify({ op: "publish", topic, msg: { data } }));
  return true;
}

function supervisorMissionCommands() {
  const commands = {};
  const editors = document.querySelectorAll("#dispatch-editors textarea");
  const clean = (text) => text.split("\n")
    .map((l) => l.trim()).filter((l) => l && !l.startsWith("#"));
  if (editors.length) {
    for (const ta of editors) {
      commands[agvName(parseInt(ta.dataset.agent, 10))] = clean(ta.value);
    }
  } else {
    for (const route of state.routes) {
      commands[agvName(route.agent)] = clean(generateCommandText(route));
    }
  }
  return commands;
}

// ---- external solver plan import ------------------------------------------
// Interchange format (produced by export_plan_json.py running romeshprasad's
// vrp_rpd solvers — NN / max-regret / greedy-defer / BRKGA — on this
// testbed's instance):
//   {"algorithm": "...", "process_sec": 15, "capacity": 3,
//    "visits": {"Alvik1": [{"bay": 14, "mode": "drop"|"pick"}, ...], ...}}
// The import replaces only the ASSIGNMENT step; timing, ready-time waits,
// dwell deconfliction, command generation and dispatch are unchanged — so
// every algorithm runs through the identical execution stack.
function importPlanFromJson(data, sourceName) {
  // Accept either {visits: {Alvik1: [...]}} directly, or this app's own
  // export shape ({routes: [{agv, visits}, ...]}) — downloadJson() writes
  // the latter, so re-importing a file you just downloaded must work.
  let visitsByName = data.visits;
  if ((!visitsByName || typeof visitsByName !== "object") && Array.isArray(data.routes)) {
    visitsByName = {};
    for (const route of data.routes) {
      if (route && route.agv && Array.isArray(route.visits)) {
        visitsByName[route.agv] = route.visits;
      }
    }
  }
  if (!visitsByName || typeof visitsByName !== "object" || !Object.keys(visitsByName).length) {
    throw new Error("no 'visits' object ({Alvik1: [{bay, mode}, ...], ...}) or 'routes' array with agv+visits");
  }
  const names = Object.keys(visitsByName);
  if (!names.length || names.length > 12) {
    throw new Error(`bad agent count ${names.length}`);
  }
  syncControls();
  const agentCount = names.length;
  const visitLists = [];
  for (let i = 0; i < agentCount; i++) {
    const key = names.find((n) => n.toUpperCase() === agvName(i).toUpperCase());
    if (!key) {
      throw new Error(`missing visits for ${agvName(i)} (got: ${names.join(", ")})`);
    }
    visitLists.push(visitsByName[key]);
  }

  const maxBay = (state.rows - 1) * (state.cols - 1);
  const selected = new Set();
  let anyPick = false;
  const parsed = visitLists.map((list, agent) => (list || []).map((v, k) => {
    const b = Number(v.bay);
    if (!Number.isInteger(b) || b < 1 || b > maxBay) {
      throw new Error(`${agvName(agent)} visit ${k + 1}: bay '${v.bay}' outside 1..${maxBay}`);
    }
    const modeRaw = String(v.mode || "drop").toLowerCase();
    const mode = (modeRaw === "p" || modeRaw === "pick" || modeRaw === "pickup")
      ? "pick" : "drop";
    if (mode === "pick") anyPick = true;
    const { r, c } = bayFromNumber(b);
    selected.add(bayId(r, c));
    return mode === "pick" ? { r, c, mode } : { r, c };
  }).reduce((out, v) => {
    // Merge an adjacent drop+pick at the SAME bay into one "service" visit:
    // the robot stays inside the dead-end bay through processing (one entry,
    // one extended dwell, one exit) instead of leaving, about-facing at the
    // node and re-entering. Solvers like Max Regret emit this pattern a lot.
    const prev = out[out.length - 1];
    if (prev && v.mode === "pick" && !prev.mode &&
        prev.r === v.r && prev.c === v.c) {
      prev.mode = "service";
    } else {
      out.push(v);
    }
    return out;
  }, []));

  // Capacity feasibility. In the VRP-RPD model resources are REUSABLE: a
  // robot may serve more drops than its capacity by redepositing parts it
  // picked up mid-tour (e.g. D D D P D P D with capacity 3). A tour is
  // feasible iff some initial load L in [0, cap] keeps the running load
  // (D = -1, P = +1) inside [0, cap] for every prefix.
  const cap = Number.isFinite(Number(data.capacity))
    ? Math.round(Number(data.capacity)) : state.capacity;
  parsed.forEach((list, agent) => {
    let running = 0;
    let minPrefix = 0;
    let maxPrefix = 0;
    for (const v of list) {
      const deltas = v.mode === "service" ? [-1, 1]
        : [v.mode === "pick" ? 1 : -1];
      for (const d of deltas) {
        running += d;
        minPrefix = Math.min(minPrefix, running);
        maxPrefix = Math.max(maxPrefix, running);
      }
    }
    const loLoad = Math.max(0, -minPrefix);
    const hiLoad = Math.min(cap, cap - maxPrefix);
    if (loLoad > hiLoad) {
      throw new Error(
        `${agvName(agent)}: tour needs a load swing of ` +
        `${maxPrefix - minPrefix} parts > capacity ${cap}`);
    }
  });

  // Adopt the mission parameters — written to the DOM because syncControls()
  // reads the controls back on every render.
  document.getElementById("agents").value = agentCount;
  document.getElementById("capacity").value = cap;
  document.getElementById("rpd-mode").checked = anyPick;
  if (Number.isFinite(Number(data.process_sec))) {
    document.getElementById("process-sec").value = Math.round(Number(data.process_sec));
  }
  syncControls();
  state.selected = selected;
  stopPlayback();

  const templates = parsed.map((list, agent) => {
    const route = buildAgentRoute(agent, list, 0, [], {}, "vh");
    route.pinnedOrder = true;  // the external solver owns the visit order
    return route;
  });
  const polished = polishCollisionPlan(templates);
  state.routes = polished.routes;
  state.conflicts = polished.conflicts;
  state.schedule = polished.schedule;
  state.planSource = `${data.algorithm || "external"} (${sourceName})`;
  computeEdgeUse();
  refreshScheduleStatus();
  state.timestep = 0;
  invalidateDispatchEditors();
  render();
  setView("plan");  // successful solve/import: collapse setup
  showReport();
}

async function supervisorSendMission() {
  if (!state.routes.length) {
    document.getElementById("sup-status").textContent = "solve a plan first";
    return;
  }
  const mission = {
    schedule: buildExportPayload(),
    commands: supervisorMissionCommands(),
    drive_mode: state.driveMode,  // "color" | "vision" — see setDriveMode()
    // Turn+drive fusion request -- the supervisor forces this off for
    // missions with 2+ robots regardless of what's sent here (see
    // run_advised_vision()'s docstring in fleetSupervisor.py).
    fuse_turns: state.fuseTurns,
    ms: Date.now(),
  };
  // Page 4 metrics: register this mission as a run BEFORE dispatch so the
  // supervisor can stream structured events against its run_id. Guarded +
  // time-limited: if the metrics backend is down or slow the mission is
  // sent exactly as before, just without a run_id.
  if (typeof metricsCreateRunForMission === "function") {
    try {
      const reg = await metricsCreateRunForMission(mission);
      if (reg && reg.run_id) {
        mission.metrics_run_id = reg.run_id;
        mission.metrics_url = reg.metrics_url;
      }
    } catch (err) {
      console.warn("metrics run registration skipped:", err);
    }
  }
  if (supPublish("fleet_mission", JSON.stringify(mission))) {
    document.getElementById("sup-status").textContent =
      `mission sent (${state.driveMode === "vision" ? "camera-only" : "color+camera"}` +
      `${state.fuseTurns ? ", fuse_turns" : ""}) — wait for 'loaded', then Start`;
  }
}

function renderSupervisorStatus(dataStr) {
  let s;
  try { s = JSON.parse(dataStr); } catch (_) { return; }
  document.getElementById("sup-status").textContent =
    `supervisor: ${s.state}${s.note ? " — " + s.note : ""}`;
  // Live supervisor lifecycle (loaded/armed/running/finished/aborted/error),
  // straight off /fleet_status -- Page 4's KPI badge prefers this over the
  // metrics DB's coarser run status so "IDLE"/"ARMED" are visible the
  // instant the supervisor reports them, not only after a RUN_COMPLETED/
  // RUN_FAILED event has round-tripped through the metrics backend.
  state.supervisorFleetStatus = { state: s.state, note: s.note || "",
    receivedAt: Date.now() };
  if (typeof metricsOnSupervisorStatus === "function") {
    try { metricsOnSupervisorStatus(); } catch (_) {}
  }
  const lines = Object.entries(s.robots || {}).map(([name, r]) => {
    const mark = r.online === true ? "● " : (r.online === false ? "○ OFFLINE " : "");
    const step = (typeof r.step === "number" && typeof r.total === "number")
      ? `step ${r.step}/${r.total}` : "";
    return `${mark}${name}  ${String(r.phase || "").padEnd(7)} ${step}` +
      `  node=${r.node || "?"}${r.errored ? "  ERRORED" : ""}` +
      (r.last_status ? `  [${r.last_status}]` : "");
  });
  document.getElementById("sup-robot-status").textContent = lines.join("\n");
}

// ---- Decision Log (Run & Monitor) -----------------------------------------
// Accumulates /fleet_events from the supervisor: vision warnings, robot
// errors, recovery, waits, advisor decisions, mission milestones. Kept per-run in
// state.decisionLog; Clear resets it, Save downloads .json + .txt.
const DECISION_KIND_STYLE = {
  vision:   { icon: "⚠", label: "VISION",   cls: "dl-warn" },
  error:    { icon: "✖", label: "ERROR",    cls: "dl-error" },
  recovery: { icon: "⟳", label: "RECOVERY", cls: "dl-recovery" },
  wait:     { icon: "⏸", label: "WAIT",     cls: "dl-wait" },
  advisor:      { icon: "🧠", label: "advisor",      cls: "dl-advisor" },
  mission:  { icon: "▣", label: "MISSION", cls: "dl-mission" },
};

function ingestDecisionEvent(dataStr) {
  let e;
  try { e = JSON.parse(dataStr); } catch (_) { return; }
  if (!e || typeof e.text !== "string") return;
  // The supervisor stamps a per-run monotonic seq; dedupe rosbridge re-delivery.
  if (e.seq != null) {
    if (state.decisionLogSeen.has(e.seq)) return;
    state.decisionLogSeen.add(e.seq);
  }
  state.decisionLog.push({
    ms: Number(e.ms) || Date.now(),
    kind: String(e.kind || "mission"),
    severity: String(e.severity || "info"),
    robot: e.robot || null,
    text: String(e.text),
  });
  // Cap in-memory history so a very long run can't grow unbounded.
  if (state.decisionLog.length > 2000) state.decisionLog.shift();
  renderDecisionLog();
  renderConflictPane();  // live vision/error events also surface in that pane
}

function decisionTimeStr(ms) {
  const d = new Date(ms);
  const p = (n) => String(n).padStart(2, "0");
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

function renderDecisionLog() {
  const box = document.getElementById("decision-log");
  if (!box) return;
  const count = document.getElementById("decision-log-count");
  if (count) count.textContent = `${state.decisionLog.length} event(s)`;
  if (!state.decisionLog.length) {
    const connected = state.realSocket && state.realSocket.readyState === 1;
    const hint = connected
      ? "Events from the fleet supervisor (vision warnings, errors, recovery, " +
        "advisor decisions, mission milestones) appear here as the run proceeds."
      : "Switch to Real or Live mode to connect to rosbridge — then supervisor " +
        "events (vision warnings, errors, recovery, advisor, mission) stream in here.";
    box.innerHTML = `<p class="muted" style="margin:6px;">No events yet. ${hint}</p>`;
    return;
  }
  // Newest last, matching a scrolling console; auto-scroll to the bottom.
  const rows = state.decisionLog.map((ev) => {
    const style = DECISION_KIND_STYLE[ev.kind] || DECISION_KIND_STYLE.mission;
    const who = ev.robot ? `<b>${escapeHtml(ev.robot)}</b> ` : "";
    return `<div class="dl-row ${style.cls}">` +
      `<span class="dl-time">${decisionTimeStr(ev.ms)}</span>` +
      `<span class="dl-kind">${escapeHtml(style.label)}</span>` +
      `<span class="dl-text">${who}${escapeHtml(ev.text)}</span></div>`;
  }).join("");
  box.innerHTML = rows;
  box.scrollTop = box.scrollHeight;
}

function clearDecisionLog() {
  if (state.decisionLog.length &&
      !confirm(`Clear the Decision Log (${state.decisionLog.length} events)? ` +
               "Save it first if you want to keep this run's history.")) {
    return;
  }
  state.decisionLog = [];
  state.decisionLogSeen = new Set();
  renderDecisionLog();
}

function saveDecisionLog() {
  if (!state.decisionLog.length) {
    alert("Decision Log is empty — nothing to save.");
    return;
  }
  const first = state.decisionLog[0].ms;
  const d = new Date(first);
  const p = (n) => String(n).padStart(2, "0");
  const stamp = `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}` +
    `_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
  const base = `decision_log_${stamp}`;

  // .json — structured, for later analysis/plotting.
  const json = JSON.stringify({
    saved_at: new Date().toISOString(),
    plan_source: state.planSource || "built-in",
    agents: state.agents,
    events: state.decisionLog.map((ev) => ({
      time: new Date(ev.ms).toISOString(),
      ms: ev.ms, kind: ev.kind, severity: ev.severity,
      robot: ev.robot, text: ev.text,
    })),
  }, null, 2);
  downloadTextFile(`${base}.json`, json, "application/json");

  // .txt — human-readable, one line per event.
  const txt = state.decisionLog.map((ev) => {
    const style = DECISION_KIND_STYLE[ev.kind] || DECISION_KIND_STYLE.mission;
    const who = ev.robot ? `${ev.robot} ` : "";
    return `[${decisionTimeStr(ev.ms)}] ${style.label.padEnd(8)} ${who}${ev.text}`;
  }).join("\n");
  downloadTextFile(`${base}.txt`,
    `AGV run decision log — ${state.decisionLog.length} events\n` +
    `saved ${new Date().toISOString()}  plan: ${state.planSource || "built-in"}\n` +
    "".padEnd(60, "-") + "\n" + txt + "\n", "text/plain");
}

function downloadTextFile(filename, content, mime) {
  const blob = new Blob([content], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function dispatchGenerate() {
  const editors = document.getElementById("dispatch-editors");
  editors.innerHTML = "";
  if (!state.routes.length) {
    dispatchSetStatus("solve a plan first");
    return;
  }
  for (const route of state.routes) {
    const label = document.createElement("div");
    label.className = "muted";
    label.style.marginTop = "6px";
    label.textContent = `${agvName(route.agent)} commands (editable — verify the depot exit against a proven route):`;
    const ta = document.createElement("textarea");
    ta.dataset.agent = String(route.agent);
    ta.style.cssText = "width:100%; min-height:90px; font-family:monospace; font-size:11px;";
    ta.value = generateCommandText(route);
    editors.appendChild(label);
    editors.appendChild(ta);
  }
  dispatchSetStatus("commands generated — review, then Start");
  dispatchButtons();
}

function dispatchStart() {
  if (dispatch.active) return;
  if (!state.realSocket || state.realSocket.readyState !== 1) {
    connectRealMode();
    dispatchSetStatus("connecting to rosbridge — press Start again once connected");
    return;
  }
  const robots = {};
  try {
    for (const ta of document.querySelectorAll("#dispatch-editors textarea")) {
      const agent = parseInt(ta.dataset.agent, 10);
      const route = state.routes.find((r) => r.agent === agent);
      const name = agvName(agent);
      const plan = alignCommandsToRoute(route, ta.value.split("\n"));
      robots[name] = {
        name, agent, plan, nextIndex: 0, phase: "waiting",
        currentNode: `D${agent + 1}`, pending: null, sentAtSec: 0,
        lastDoneSec: null, launched: false, skipNextIdle: false,
        launchOffsetSec: route.launchOffsetSec || 0,
        lastHoldSig: null, holdCount: 0, lastStatus: "",
      };
    }
  } catch (err) {
    dispatchSetStatus(`alignment failed: ${err.message}`);
    dispatchLog(`ALIGNMENT ERROR: ${err.message}`);
    return;
  }
  for (const name of Object.keys(robots)) {
    state.realSocket.send(JSON.stringify(
      { op: "advertise", topic: `${name}_cmd`, type: "std_msgs/String" }));
    state.realSocket.send(JSON.stringify(
      { op: "subscribe", topic: `${name}_status`, type: "std_msgs/String" }));
  }
  dispatch.robots = robots;
  dispatch.t0 = performance.now();
  state.missionStartMs = Date.now();  // anchors Real-mode status badges
  dispatch.active = true;
  dispatch.paused = false;
  dispatch.timer = setInterval(dispatchTick, DISPATCH_TICK_MS);
  dispatchSetStatus(`dispatching ${Object.keys(robots).join(", ")}`);
  dispatchLog(`dispatch started: ${Object.values(robots).map((r) =>
    `${r.name} ${r.plan.length} cmds (launch +${r.launchOffsetSec.toFixed(1)}s)`).join("; ")}`);
  dispatchButtons();
}

function dispatchHold() {
  if (!dispatch.active) return;
  dispatch.paused = !dispatch.paused;
  dispatchLog(dispatch.paused
    ? "HOLD: no further commands will be issued (in-flight commands finish)"
    : "resumed");
  dispatchSetStatus(dispatch.paused ? "holding (in-flight commands finish)" : "dispatching");
  dispatchButtons();
}

function dispatchAbort() {
  for (const name of Object.keys(dispatch.robots)) {
    dispatchPublish(`${name}_cmd`, "STOP");
  }
  dispatch.active = false;
  dispatch.paused = false;
  if (dispatch.timer) clearInterval(dispatch.timer);
  dispatchSetStatus("aborted — STOP sent to all robots");
  dispatchLog("ABORT: STOP published to every robot");
  dispatchButtons();
}

