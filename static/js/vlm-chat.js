// ---- Ask: natural-language questions about the LIVE testbed ---------------
// Backed by /api/vlm/chat (vlm_chat.py -> local Ollama). Each turn sends the
// CURRENT measured state from this page plus a camera frame the server pulls
// fresh, so an answer always describes now rather than whenever the tab was
// opened.
//
// The state comes from here rather than the server because this page already
// holds it: state.visionPositions / state.realPositions are fed by the same
// rosbridge feed the operator is watching. That keeps the Flask process free
// of any ROS dependency and guarantees the model reasons about exactly the
// numbers on screen.
//
// Entirely additive. Everything below no-ops if #tab-ask is absent, and a
// failure is rendered as a message in the pane -- Pages 1-3 and the metrics
// page never depend on it.

const VLM_MAX_HISTORY = 12;          // turns kept for context (server trims too)
const vlmHistory = [];
let vlmBusy = false;

// Snapshot of what this page knows, in the shape vlm_chat.state_text() reads.
// Vision is preferred over odometry where both exist, matching how the map
// itself decides what to draw.
function vlmCollectState() {
  const robots = {};
  const add = (agent, pose, source) => {
    if (!pose || !pose.point) return;
    const name = agvName(agent);
    if (robots[name] && robots[name].source === "vision") return;
    robots[name] = {
      x_in: Number(pose.point.x ?? pose.point.c ?? null),
      y_in: Number(pose.point.y ?? pose.point.r ?? null),
      yaw_deg: pose.yawDeg ?? null,
      battery_pct: (typeof realBatteryForAgent === "function")
        ? realBatteryForAgent(agent) : null,
      source,
    };
  };
  if (state.visionPositions) {
    for (const [agent, pose] of state.visionPositions) add(agent, pose, "vision");
  }
  if (state.realPositions) {
    for (const [agent, pose] of state.realPositions) add(agent, pose, "odometry");
  }

  const run = {};
  if (state.supervisorFleetStatus) {
    run.supervisor_state = state.supervisorFleetStatus.state ?? null;
    if (state.supervisorFleetStatus.note) {
      run.note = String(state.supervisorFleetStatus.note);
    }
  }
  if (state.routes && state.routes.length) run.planned_agents = state.routes.length;
  if (state.selected) run.selected_bays = state.selected.size;
  if (typeof state.driveMode === "string") run.drive_mode = state.driveMode;

  // Recent supervisor events give the model the "what just happened" that a
  // single frame cannot show -- a deadlock hold, an abort, a robot excluded.
  const recent = (state.decisionLog || [])
    .slice(-8)
    .map((e) => `${e.kind}${e.robot ? " " + e.robot : ""}: ${e.text}`);

  return { robots, run, recent_events: recent };
}

function vlmAppend(role, text, meta) {
  const log = document.getElementById("vlm-log");
  if (!log) return;
  const placeholder = log.querySelector("p.muted");
  if (placeholder && !log.dataset.started) {
    log.innerHTML = "";
    log.dataset.started = "1";
  }
  const div = document.createElement("div");
  div.className = `vlm-msg vlm-${role}`;
  const who = document.createElement("strong");
  who.textContent = role === "user" ? "You"
    : (role === "error" ? "Error" : "Assistant");
  const body = document.createElement("span");
  body.textContent = text;               // textContent: model output is data
  div.appendChild(who);
  div.appendChild(body);
  if (meta) {
    const m = document.createElement("span");
    m.className = "vlm-meta";
    m.textContent = meta;
    div.appendChild(m);
  }
  log.appendChild(div);
  log.scrollTop = log.scrollHeight;
  return div;
}

async function vlmRefreshStatus() {
  const box = document.getElementById("vlm-status");
  if (!box) return;
  try {
    const resp = await fetch("/api/vlm/status");
    const s = await resp.json();
    if (!s.ollama_ok) {
      box.className = "vlm-status bad";
      box.textContent = s.error || "Ollama unreachable";
      return;
    }
    if (!s.model_present) {
      box.className = "vlm-status bad";
      box.textContent = `model ${s.model} not pulled — run: ollama pull ${s.model}`;
      return;
    }
    box.className = s.camera_ok ? "vlm-status ok" : "vlm-status warn";
    box.textContent = s.camera_ok
      ? `${s.model} · camera connected`
      : `${s.model} · no camera frame (answers from measured state only)`;
  } catch (err) {
    box.className = "vlm-status bad";
    box.textContent = "assistant backend unavailable";
  }
}

async function vlmSend() {
  if (vlmBusy) return;
  const input = document.getElementById("vlm-question");
  if (!input) return;
  const question = input.value.trim();
  if (!question) return;

  vlmBusy = true;
  input.value = "";
  vlmAppend("user", question);
  const pending = vlmAppend("assistant", "thinking…");

  try {
    const resp = await fetch("/api/vlm/chat", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        question,
        state: vlmCollectState(),
        history: vlmHistory.slice(-VLM_MAX_HISTORY),
      }),
    });
    const data = await resp.json();
    if (pending) pending.remove();
    if (!data.ok) {
      vlmAppend("error", data.error || "no answer");
    } else {
      // saw_frame is surfaced so an answer given without an image is never
      // mistaken for one that looked at the table.
      const meta = `${data.model} · ${data.seconds}s`
        + (data.saw_frame ? "" : " · no camera frame");
      vlmAppend("assistant", data.answer || "(empty answer)", meta);
      vlmHistory.push({ role: "user", content: question });
      vlmHistory.push({ role: "assistant", content: data.answer || "" });
      while (vlmHistory.length > VLM_MAX_HISTORY) vlmHistory.shift();
    }
  } catch (err) {
    if (pending) pending.remove();
    vlmAppend("error", String(err));
  } finally {
    vlmBusy = false;
    input.focus();
  }
}

function vlmClear() {
  const log = document.getElementById("vlm-log");
  vlmHistory.length = 0;
  if (log) {
    log.innerHTML = "";
    delete log.dataset.started;
    vlmAppend("assistant", "Conversation cleared.");
  }
}

function vlmInit() {
  if (!document.getElementById("tab-ask")) return;
  const send = document.getElementById("vlm-send");
  const clear = document.getElementById("vlm-clear");
  const input = document.getElementById("vlm-question");
  if (send) send.addEventListener("click", vlmSend);
  if (clear) clear.addEventListener("click", vlmClear);
  if (input) {
    // Enter sends, Shift+Enter newlines -- a question is usually one line.
    input.addEventListener("keydown", (ev) => {
      if (ev.key === "Enter" && !ev.shiftKey) {
        ev.preventDefault();
        vlmSend();
      }
    });
  }
  vlmRefreshStatus();
  // Slow poll: this only reports reachability, and the model is on another
  // machine that may be busy answering.
  setInterval(vlmRefreshStatus, 30000);
}
