// ===================== Page 4 · Performance Metrics ========================
// Frontend for the Flask metrics backend (app.py + SQLite). Loaded after
// solver.js/live-monitor.js and before app.js; every entry point is called
// through a typeof-guard from those files, so a failure here can never break
// Setup / Plan & Simulate / Run & Monitor.
//
// Data flow:
//   Send mission  -> metricsCreateRunForMission(): POST /api/runs with the
//                    exported plan, attach run_id + a LAN-reachable backend
//                    URL to the mission so the supervisor can stream
//                    structured events against it (fleet/metrics_events.py).
//   Page 4 shown  -> health probe, run list, KPI cards (PROVISIONAL until
//                    finalized), 9 detail tabs, history + comparison.
// All metric VALUES are computed in Python (metrics_engine.py); this file
// only formats and charts what the API returns.

const METRICS_TIMEOUT_MS = 2500;

// The supervisor (Linux laptop) needs a LAN-reachable URL for this backend;
// "localhost" only works from this machine. Override: ?metrics=host[:port]
const METRICS_SUPERVISOR_URL = (() => {
  let v = new URLSearchParams(window.location.search).get("metrics");
  if (!v) {
    const host = ["localhost", "127.0.0.1"].includes(window.location.hostname)
      ? "192.168.0.162"  // this Windows laptop's LAN address
      : window.location.hostname;
    return `http://${host}:${window.location.port || 8000}`;
  }
  if (!v.startsWith("http://") && !v.startsWith("https://")) v = `http://${v}`;
  return v.replace(/\/$/, "");
})();

let _mAvailable = null;        // null = not probed, true/false afterwards
let _mSessionRunId = null;     // run created by this session's Send mission
let _mSelectedRunId = null;    // run currently shown in the KPI cards
let _mRuns = [];               // run list for the selector
let _mMetrics = null;          // last /metrics response for the selected run
let _mDefinitions = null;      // /api/metric-definitions cache
let _mActiveTab = "summary";
let _mPollTimer = null;
let _mSort = { key: "created_at_utc", dir: -1 };
let _mHistory = null;

async function metricsFetch(path, opts = {}) {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), opts.timeoutMs || METRICS_TIMEOUT_MS);
  try {
    const resp = await fetch(path, {
      method: opts.method || "GET",
      headers: opts.body ? { "Content-Type": "application/json" } : undefined,
      body: opts.body ? JSON.stringify(opts.body) : undefined,
      signal: ctrl.signal,
    });
    const data = await resp.json().catch(() => ({}));
    if (!resp.ok) throw new Error(data.error || `HTTP ${resp.status}`);
    return data;
  } finally {
    clearTimeout(timer);
  }
}

// ---- formatting -----------------------------------------------------------

function mFmt(v, digits = 2) {
  if (v === null || v === undefined || Number.isNaN(v)) return "—";
  if (typeof v !== "number") return String(v);
  if (Number.isInteger(v)) return String(v);
  const abs = Math.abs(v);
  if (abs >= 1000) return v.toFixed(0);
  if (abs >= 10) return v.toFixed(1);
  return v.toFixed(digits);
}

function mFmtSec(v) {
  if (v === null || v === undefined) return "—";
  if (v >= 90) {
    const m = Math.floor(v / 60);
    return `${m}m ${(v - m * 60).toFixed(0)}s`;
  }
  return `${v.toFixed(1)}s`;
}

// Canonical storage is meters; the testbed is measured in inches, so show
// both (spec section 4).
function mFmtMeters(v) {
  if (v === null || v === undefined) return "—";
  return `${v.toFixed(3)} m (${(v / 0.0254).toFixed(1)} in)`;
}

function mShortId(runId) {
  return String(runId || "").slice(0, 8);
}

// Timestamps are STORED as UTC (canonical, per the metrics spec -- keeps
// runs comparable regardless of where/when they were recorded) but DISPLAYED
// in the lab's local Eastern time, which is what the user actually reads a
// clock in. America/New_York (not a fixed -5) so EDT/EST is handled
// automatically -- an August run is EDT, a January run is EST, and the
// suffix below reflects whichever applied on that date.
const METRICS_TZ = "America/New_York";

function mFmtEastern(isoUtc, opts = {}) {
  if (!isoUtc) return "—";
  const d = new Date(isoUtc);
  if (Number.isNaN(d.getTime())) return String(isoUtc);
  const parts = {
    timeZone: METRICS_TZ,
    year: "numeric", month: "2-digit", day: "2-digit",
    // hourCycle h23, not just hour12:false -- some engines render midnight
    // as "24:00" with the latter alone.
    hour: "2-digit", minute: "2-digit", hourCycle: "h23",
  };
  if (opts.seconds) parts.second = "2-digit";
  // en-CA gives YYYY-MM-DD ordering, which sorts and scans cleanly.
  let text = new Intl.DateTimeFormat("en-CA", parts).format(d).replace(",", "");
  if (opts.tzName) {
    const tz = new Intl.DateTimeFormat("en-US", {
      timeZone: METRICS_TZ, timeZoneName: "short",
    }).formatToParts(d).find((p) => p.type === "timeZoneName");
    if (tz) text += ` ${tz.value}`;
  }
  return text;
}

// Supervisor /fleet_status lifecycle -> the display vocabulary the user
// wants (ARMED once a mission is loaded and staged, IDLE once nothing is
// loaded/running -- including right after a finished/aborted run, since at
// that point the supervisor really is idle again and waiting for the next
// mission). See fleetSupervisor.py's publish_fleet_status() call sites for
// the exhaustive set of states this maps.
const SUPERVISOR_STATUS_LABEL = {
  loaded: "ARMED", armed: "ARMED", running: "RUNNING",
  finished: "IDLE", aborted: "IDLE", blocked: "ARMED", error: "ERROR",
};
// How stale a /fleet_status message can be before it's not trusted as "the
// supervisor's current phase" -- publish_fleet_status() is sent at least
// every ~1s while a mission is loaded/running, so anything much older than
// that means the rosbridge connection (or the supervisor itself) is gone,
// not that the phase is still accurate.
const SUPERVISOR_STATUS_STALE_MS = 5000;

// True only when the run currently selected on Page 4 is the SAME run this
// browser session registered via Send mission -- live supervisor phase is
// only a true statement about that run. Any other (historical) run keeps
// its own DB-recorded terminal status regardless of what the supervisor is
// doing right now.
function metricsIsLiveSelectedRun() {
  return _mSessionRunId !== null && _mSelectedRunId === _mSessionRunId;
}

function metricsLiveSupervisorPhase() {
  const live = state.supervisorFleetStatus;
  if (!live || Date.now() - live.receivedAt > SUPERVISOR_STATUS_STALE_MS) return null;
  return SUPERVISOR_STATUS_LABEL[live.state] || null;
}

function mStatusBadge(status, provisional) {
  const livePhase = metricsIsLiveSelectedRun() ? metricsLiveSupervisorPhase() : null;
  const displayText = livePhase || String(status || "?").toUpperCase();
  const cls = livePhase
    ? { RUNNING: "running", ARMED: "running", IDLE: "final",
        ERROR: "failed" }[livePhase] || "running"
    : { running: "running", created: "running", completed: "final",
        failed: "failed", aborted: "aborted" }[status] || "running";
  let html = `<span class="metrics-badge ${cls}">${escapeHtml(displayText)}</span>`;
  if (provisional && (status === "running" || status === "created")) {
    html += ` <span class="metrics-badge provisional">PROVISIONAL</span>`;
  } else if (provisional) {
    html += ` <span class="metrics-badge provisional">PROVISIONAL — not finalized</span>`;
  }
  return html;
}

// ---- lifecycle ------------------------------------------------------------

function metricsInit() {
  document.getElementById("metrics-retry").addEventListener("click", () => {
    _mAvailable = null;
    metricsViewShown();
  });
  document.getElementById("metrics-refresh").addEventListener("click", () => metricsViewShown());
  document.getElementById("metrics-mark-finished").addEventListener("click", metricsMarkStuckRunFinished);
  document.getElementById("metrics-run-select").addEventListener("change", (ev) => {
    _mSelectedRunId = ev.target.value || null;
    metricsLoadSelectedRun();
  });
  document.getElementById("metrics-apply-filters").addEventListener("click", () => metricsLoadHistory());
  document.getElementById("metrics-clear-filters").addEventListener("click", () => {
    for (const input of document.querySelectorAll("#metrics-filters input")) input.value = "";
    for (const sel of document.querySelectorAll("#metrics-filters select")) sel.selectedIndex = 0;
    metricsLoadHistory();
  });
  metricsBuildFilters();
  metricsBuildTabs();
  // Quiet background probe so the banner state is correct on first open.
  metricsProbe();
}

// Called by renderSupervisorStatus() (live-monitor.js) every time a fresh
// /fleet_status message arrives, so ARMED/RUNNING/IDLE updates the instant
// the supervisor reports them rather than waiting on the next poll tick.
function metricsOnSupervisorStatus() {
  // Run-completion fallback FIRST, before any view/selection guard -- the
  // run finishes while the user is on Run & Monitor, not Page 4.
  metricsMaybeCloseFinishedRun();
  if (state.view !== "metrics" || !_mMetrics || !metricsIsLiveSelectedRun()) return;
  const badge = document.getElementById("metrics-run-badge");
  if (badge) badge.innerHTML = mStatusBadge(_mMetrics.run_status, _mMetrics.provisional);
  metricsRenderKpis();
}

// Which supervisor lifecycle state we last acted on, so the repeated
// /fleet_status publishes can't re-PATCH the same transition.
let _mClosedRunId = null;

// ADDED 2026-08-25: without this, a run NEVER reached "completed" -- the
// browser only ever PATCHed 'running' (Start) and 'aborted' (Abort), and
// the only thing that could set 'completed' was the supervisor's
// RUN_COMPLETED metrics event, which requires fleet/metrics_events.py +
// the instrumented fleetSupervisor.py to be deployed on the Linux laptop.
// Confirmed from real data: every recorded run sat at running/aborted,
// zero completed. This closes the loop using /fleet_status (already
// streaming over the rosbridge connection the page has open anyway):
// the supervisor publishes state="finished" with
// note="complete=[...] errored=[...]" at the end of run_advised_vision(),
// so an empty errored list means a clean completion. This is a FALLBACK,
// not a replacement -- the event stream still gives the real per-robot/
// per-job metrics; this only guarantees the run's own lifecycle status is
// correct even when the emitter isn't deployed.
function metricsMaybeCloseFinishedRun() {
  const live = state.supervisorFleetStatus;
  if (!live || !_mSessionRunId) return;
  if (live.state !== "finished") return;
  if (_mClosedRunId === _mSessionRunId) return;  // already handled
  _mClosedRunId = _mSessionRunId;
  // note looks like: complete=['Alvik1'] errored=[]
  const note = String(live.note || "");
  const erroredMatch = note.match(/errored=\[(.*?)\]/);
  const hadErrors = erroredMatch ? erroredMatch[1].trim().length > 0 : false;
  const status = hadErrors ? "failed" : "completed";
  const runId = _mSessionRunId;
  (async () => {
    try {
      await metricsFetch(`/api/runs/${runId}`, {
        method: "PATCH", body: { status } });
      // Finalize so the values stop reading PROVISIONAL. Harmless 409 if
      // the supervisor's own event stream already finalized it.
      await metricsFetch(`/api/runs/${runId}/finalize`, {
        method: "POST", body: {} }).catch(() => {});
      if (state.view === "metrics") {
        await metricsLoadRunList();
        await metricsLoadSelectedRun();
        await metricsLoadHistory();
      }
    } catch (err) {
      console.warn("metrics: could not close out finished run:", err);
    }
  })();
}

async function metricsProbe() {
  try {
    await metricsFetch("/api/health");
    _mAvailable = true;
  } catch (_) {
    _mAvailable = false;
  }
  const unavailable = document.getElementById("metrics-unavailable");
  const body = document.getElementById("metrics-body");
  if (unavailable && body) {
    unavailable.style.display = _mAvailable ? "none" : "";
    body.style.display = _mAvailable ? "" : "none";
  }
  return _mAvailable;
}

async function metricsViewShown() {
  if (!(await metricsProbe())) return;
  await metricsLoadRunList();
  await metricsLoadSelectedRun();
  await metricsLoadHistory();
}

// ---- run registration from the dispatch flow ------------------------------

function metricsConditionTag(schedule, driveMode) {
  const ws = (schedule.workstations || []).length;
  const rpd = schedule.rpd && schedule.rpd.enabled
    ? `-rpd${schedule.rpd.process_sec}s` : "";
  return `${schedule.agents}r-${ws}ws-${driveMode}${rpd}`;
}

async function metricsCreateRunForMission(mission) {
  const schedule = mission.schedule || {};
  const jobCount = (schedule.routes || []).reduce(
    (sum, r) => sum + ((r.visits || []).length), 0);
  const payload = {
    experimental_condition: metricsConditionTag(schedule, mission.drive_mode),
    algorithm: state.planSource ||
      (document.getElementById("route-mode") || {}).value || "built-in",
    drive_mode: mission.drive_mode,
    robot_count: schedule.agents,
    job_count: jobCount,
    workstation_count: (schedule.workstations || []).length,
    process_sec: schedule.rpd ? schedule.rpd.process_sec : null,
    capacity: schedule.capacity,
    random_seed: clampInt((document.getElementById("seed") || {}).value, 1, 999999),
    planned_makespan_sec: schedule.makespan_sec,
    plan_json: schedule,
  };
  const data = await metricsFetch("/api/runs", { method: "POST", body: payload });
  _mSessionRunId = data.run_id;
  _mSelectedRunId = data.run_id;
  return { run_id: data.run_id, metrics_url: METRICS_SUPERVISOR_URL };
}

function metricsNotifyRunStarted() {
  if (!_mSessionRunId) return;
  metricsFetch(`/api/runs/${_mSessionRunId}`, {
    method: "PATCH", body: { status: "running" },
  }).catch(() => {});
}

function metricsNotifyRunAborted() {
  if (!_mSessionRunId) return;
  metricsFetch(`/api/runs/${_mSessionRunId}`, {
    method: "PATCH", body: { status: "aborted" },
  }).catch(() => {});
}

// ---- run selector + KPI cards ---------------------------------------------

async function metricsLoadRunList() {
  try {
    const data = await metricsFetch("/api/runs?limit=50");
    _mRuns = data.runs || [];
  } catch (_) {
    _mRuns = [];
  }
  const sel = document.getElementById("metrics-run-select");
  if (!sel) return;
  if (!_mSelectedRunId && _mRuns.length) {
    const running = _mRuns.find((r) => r.status === "running");
    _mSelectedRunId = _mSessionRunId || (running ? running.run_id : _mRuns[0].run_id);
  }
  sel.innerHTML = _mRuns.map((r) =>
    `<option value="${escapeHtml(r.run_id)}"${r.run_id === _mSelectedRunId ? " selected" : ""}>` +
    `${escapeHtml(mShortId(r.run_id))} · ${escapeHtml(mFmtEastern(r.created_at_utc))}` +
    ` · ${escapeHtml(r.status)} · ${escapeHtml(r.experimental_condition || "?")}</option>`
  ).join("") || `<option value="">no runs recorded yet</option>`;
}

function metricsValue(key, scopeType = "run", scopeId = "") {
  if (!_mMetrics) return null;
  return (_mMetrics.values || []).find((v) =>
    v.metric_key === key && v.scope_type === scopeType &&
    (v.scope_id || "") === scopeId) || null;
}

function metricsValuesFor(key, scopeType = null) {
  if (!_mMetrics) return [];
  return (_mMetrics.values || []).filter((v) =>
    v.metric_key === key && (scopeType === null || v.scope_type === scopeType));
}

async function metricsLoadSelectedRun() {
  const hint = document.getElementById("metrics-current-hint");
  const cards = document.getElementById("metrics-kpi-cards");
  const badge = document.getElementById("metrics-run-badge");
  if (_mPollTimer) { clearInterval(_mPollTimer); _mPollTimer = null; }
  if (!_mSelectedRunId) {
    if (hint) hint.textContent = "No runs recorded yet — Send a mission from Run & Monitor to create one, or POST /api/runs directly.";
    if (cards) cards.innerHTML = "";
    if (badge) badge.innerHTML = "";
    return;
  }
  try {
    _mMetrics = await metricsFetch(`/api/runs/${_mSelectedRunId}/metrics`, { timeoutMs: 8000 });
  } catch (err) {
    if (hint) hint.textContent = `Could not load metrics for this run: ${err.message}`;
    return;
  }
  const run = _mRuns.find((r) => r.run_id === _mSelectedRunId) || {};
  if (badge) badge.innerHTML = mStatusBadge(_mMetrics.run_status, _mMetrics.provisional);
  if (hint) {
    hint.textContent =
      `${run.experimental_condition || ""} · ${run.algorithm || ""} · ` +
      `created ${mFmtEastern(run.created_at_utc, { seconds: true, tzName: true })}` +
      (_mMetrics.provisional
        ? " · values are PROVISIONAL until the backend receives a run-completion, failure, or abort event and the run is finalized"
        : ` · finalized (revision ${_mMetrics.revision})`);
  }
  metricsRenderKpis();
  metricsRenderActiveTab();
  metricsRenderStuckRunNotice();
  // Live runs: refresh provisional values every few seconds while Page 4 is
  // actually visible.
  if (_mMetrics.run_status === "running") {
    _mPollTimer = setInterval(() => {
      if (state.view !== "metrics") { clearInterval(_mPollTimer); _mPollTimer = null; return; }
      metricsLoadSelectedRun();
    }, 4000);
  }
}

// A run stuck at created/running with no path to ever self-correct: it is
// NOT the run this browser session is actively watching (metricsIsLiveSelectedRun),
// so no future supervisor event about THIS run is coming -- either the
// terminal event was lost (old supervisor build, network blip) or this run
// predates the metrics integration entirely. Offers a manual PATCH to close
// it out so it stops polluting run comparisons/history as an eternally
// "running" outlier.
function metricsRenderStuckRunNotice() {
  const notice = document.getElementById("metrics-stuck-run-notice");
  if (!notice || !_mMetrics) return;
  const stuck = ["created", "running"].includes(_mMetrics.run_status) &&
    !metricsIsLiveSelectedRun();
  notice.style.display = stuck ? "" : "none";
}

async function metricsMarkStuckRunFinished() {
  if (!_mSelectedRunId) return;
  const outcome = window.prompt(
    "Mark this run as: completed, failed, or aborted?", "completed");
  if (!outcome) return;
  const status = outcome.trim().toLowerCase();
  if (!["completed", "failed", "aborted"].includes(status)) {
    alert(`'${outcome}' isn't one of completed/failed/aborted -- nothing changed.`);
    return;
  }
  try {
    await metricsFetch(`/api/runs/${_mSelectedRunId}`, {
      method: "PATCH", body: { status } });
    await metricsFetch(`/api/runs/${_mSelectedRunId}/finalize`, {
      method: "POST", body: {} });
  } catch (err) {
    alert(`Could not update this run: ${err.message}`);
    return;
  }
  await metricsLoadRunList();
  await metricsLoadSelectedRun();
  await metricsLoadHistory();
}

function metricsRenderKpis() {
  const cards = document.getElementById("metrics-kpi-cards");
  if (!cards || !_mMetrics) return;
  const v = (key) => metricsValue(key);
  const val = (key) => { const m = v(key); return m ? m.value : null; };
  const makespan = val("actual_makespan_sec");
  const observed = val("observed_duration_sec");
  const jc = v("job_completion_rate");
  const bothBatt = val("battery_drain_per_job_pct");
  const energyJob = val("energy_per_job_wh");
  const livePhase = metricsIsLiveSelectedRun() ? metricsLiveSupervisorPhase() : null;
  const statusValue = livePhase || String(_mMetrics.run_status || "?");
  const items = [
    { label: "run status", value: statusValue,
      cls: livePhase === "IDLE" || _mMetrics.run_status === "completed" ? "kpi-good"
        : livePhase === "ERROR" || ["failed", "aborted"].includes(_mMetrics.run_status) ? "kpi-bad" : "",
      sub: livePhase ? "live from supervisor" : "" },
    { label: "actual makespan", value: makespan !== null ? mFmtSec(makespan)
        : observed !== null ? `${mFmtSec(observed)} observed` : "—",
      sub: makespan === null && observed !== null ? "incomplete run — not a makespan" : "" },
    { label: "completed jobs",
      value: jc && jc.denominator ? `${mFmt(jc.numerator)}/${mFmt(jc.denominator)}` : "—" },
    { label: "throughput", value: val("throughput_jobs_per_min") !== null
        ? `${mFmt(val("throughput_jobs_per_min"))} jobs/min` : "—" },
    { label: "fleet productive utilization",
      value: val("fleet_productive_utilization_pct") !== null
        ? `${mFmt(val("fleet_productive_utilization_pct"), 1)}%` : "—" },
    { label: "traffic delay", value: val("fleet_traffic_delay_robot_sec") !== null
        ? `${mFmt(val("fleet_traffic_delay_robot_sec"), 1)} robot-s` : "—" },
    { label: "intervention-free", value: { 1: "YES", 0: "NO" }[val("intervention_free_success")] ?? "—",
      cls: val("intervention_free_success") === 1 ? "kpi-good"
        : val("intervention_free_success") === 0 ? "kpi-bad" : "" },
    { label: "min robot separation", value: val("min_fleet_separation_m") !== null
        ? mFmtMeters(val("min_fleet_separation_m")) : "—" },
    { label: "collisions", value: mFmt(val("collision_count")),
      cls: (val("collision_count") || 0) > 0 ? "kpi-bad" : "kpi-good" },
    { label: "deadlocks", value: mFmt(val("deadlock_count")),
      cls: (val("deadlock_count") || 0) > 0 ? "kpi-bad" : "" },
    { label: "battery / energy per job",
      value: energyJob !== null ? `${mFmt(energyJob)} Wh/job`
        : bothBatt !== null ? `${mFmt(bothBatt)} %-pts/job` : "—" },
    { label: "planned vs actual makespan",
      value: val("makespan_error_sec") !== null
        ? `${val("makespan_error_sec") >= 0 ? "+" : ""}${mFmt(val("makespan_error_sec"), 1)}s`
        : "—",
      sub: val("makespan_error_pct") !== null
        ? `${mFmt(val("makespan_error_pct"), 1)}% vs plan` : "" },
  ];
  const provisional = _mMetrics.provisional &&
    ["running", "created"].includes(_mMetrics.run_status);
  cards.innerHTML = items.map((item) => `
    <div class="stat ${item.cls || ""}">
      <b>${escapeHtml(String(item.value))}</b>
      <span>${escapeHtml(item.label)}${provisional ? " · PROVISIONAL" : ""}</span>
      ${item.sub ? `<span class="kpi-sub">${escapeHtml(item.sub)}</span>` : ""}
    </div>`).join("");
}

// ---- filters --------------------------------------------------------------

function metricsBuildFilters() {
  const box = document.getElementById("metrics-filters");
  if (!box) return;
  const robotOptions = Array.from({ length: 6 }, (_, i) =>
    `<option>Alvik${i + 1}</option>`).join("");
  box.innerHTML = `
    <label>Run ID <input id="mf-run_id" placeholder="uuid or blank"></label>
    <label>Date from <input id="mf-date_from" type="date"></label>
    <label>Date to <input id="mf-date_to" type="date"></label>
    <label>Condition <input id="mf-experimental_condition" placeholder="e.g. 6r-12ws-vision"></label>
    <label>Algorithm <input id="mf-algorithm" placeholder="e.g. built-in, brkga"></label>
    <label>Drive mode <select id="mf-drive_mode"><option value="">any</option>
      <option value="vision">vision (camera only)</option>
      <option value="color">color sensor</option></select></label>
    <label>Fleet size <input id="mf-robot_count" type="number" min="1" max="6"></label>
    <label>Robot <select id="mf-robot"><option value="">any</option>${robotOptions}</select></label>
    <label>Run status <select id="mf-status"><option value="">any</option>
      <option>created</option><option>running</option><option>completed</option>
      <option>failed</option><option>aborted</option></select></label>
    <label>Outcome <select id="mf-outcome"><option value="">all runs</option>
      <option value="successful">successful only</option>
      <option value="failed">failed only</option>
      <option value="aborted">aborted only</option></select></label>
    <label>Workstations <input id="mf-workstation_count" type="number" min="0"></label>
    <label>Processing time (s) <input id="mf-process_sec" type="number" min="0"></label>
    <label>Random seed <input id="mf-random_seed" type="number"></label>
    <label>Chart metric <select id="mf-chart-metric">
      <option value="actual_makespan_sec">Actual makespan</option>
      <option value="throughput_jobs_per_min">Throughput</option>
      <option value="fleet_productive_utilization_pct">Fleet utilization</option>
      <option value="fleet_traffic_delay_robot_sec">Traffic delay</option>
      <option value="yaw_mae_deg">Yaw MAE</option>
      <option value="makespan_error_sec">Makespan error</option>
      <option value="battery_drain_per_job_pct">Battery per job</option>
      <option value="total_distance_m">Total distance</option></select></label>
    <label>Compare by <select id="mf-group_by"><option value="">(no grouping)</option>
      <option value="experimental_condition">condition</option>
      <option value="algorithm">algorithm</option>
      <option value="drive_mode">drive mode</option>
      <option value="robot_count">fleet size</option>
      <option value="random_seed">seed</option></select></label>`;
}

function metricsFilterQuery() {
  const params = new URLSearchParams();
  const fields = ["run_id", "date_from", "date_to", "experimental_condition",
                  "algorithm", "drive_mode", "robot_count", "robot", "status",
                  "outcome", "workstation_count", "process_sec", "random_seed"];
  for (const f of fields) {
    const el = document.getElementById(`mf-${f}`);
    if (el && el.value) params.set(f, el.value);
  }
  const groupBy = document.getElementById("mf-group_by");
  if (groupBy && groupBy.value) params.set("group_by", groupBy.value);
  return params;
}

// ---- history + comparison -------------------------------------------------

async function metricsLoadHistory() {
  const box = document.getElementById("metrics-history");
  if (!box || _mAvailable === false) return;
  const params = metricsFilterQuery();
  const csvLink = document.getElementById("metrics-export-csv");
  if (csvLink) csvLink.href = `/api/export/runs.csv?${params.toString()}`;
  let data;
  try {
    data = await metricsFetch(`/api/history?${params.toString()}`, { timeoutMs: 8000 });
  } catch (err) {
    box.innerHTML = `<p class="metrics-note">History unavailable: ${escapeHtml(err.message)}</p>`;
    return;
  }
  _mHistory = data;
  metricsRenderHistory();
}

function metricsRenderHistory() {
  const box = document.getElementById("metrics-history");
  if (!box || !_mHistory) return;
  const runs = [..._mHistory.runs];
  const chartKey = (document.getElementById("mf-chart-metric") || {}).value ||
    "actual_makespan_sec";
  const sortVal = (r) => {
    if (_mSort.key.startsWith("m:")) {
      const mv = r.metrics[_mSort.key.slice(2)];
      return mv && mv.value !== null ? mv.value : -Infinity;
    }
    return r[_mSort.key] ?? "";
  };
  runs.sort((a, b) => {
    const va = sortVal(a), vb = sortVal(b);
    return (va < vb ? -1 : va > vb ? 1 : 0) * _mSort.dir;
  });

  const cols = [
    ["created_at_utc", "created (ET)"], ["run_id", "run"],
    ["status", "status"], ["experimental_condition", "condition"],
    ["algorithm", "algorithm"], ["drive_mode", "drive"],
    ["robot_count", "robots"],
    ["m:actual_makespan_sec", "makespan"],
    ["m:throughput_jobs_per_min", "jobs/min"],
    [`m:${chartKey}`, chartKey.replace(/_/g, " ")],
  ];
  const seen = new Set();
  const uniqueCols = cols.filter(([k]) => !seen.has(k) && seen.add(k));
  const head = uniqueCols.map(([key, label]) => {
    const cls = _mSort.key === key ? (_mSort.dir > 0 ? "sorted-asc" : "sorted-desc") : "";
    return `<th class="${cls}" data-sort="${escapeHtml(key)}">${escapeHtml(label)}</th>`;
  }).join("");
  const body = runs.map((r) => {
    const cells = uniqueCols.map(([key]) => {
      if (key === "created_at_utc") return `<td>${escapeHtml(mFmtEastern(r.created_at_utc))}</td>`;
      if (key === "run_id") return `<td title="${escapeHtml(r.run_id)}">${escapeHtml(mShortId(r.run_id))} <a href="/api/export/run/${escapeHtml(r.run_id)}.json" download title="download full run JSON">⇩</a></td>`;
      if (key.startsWith("m:")) {
        const mv = r.metrics[key.slice(2)];
        const text = mv && mv.value !== null ? mFmt(mv.value) : "—";
        const cls = mv && mv.status === "missing" ? "metrics-value-missing" : "";
        return `<td class="num ${cls}">${escapeHtml(text)}</td>`;
      }
      return `<td>${escapeHtml(String(r[key] ?? "—"))}</td>`;
    }).join("");
    return `<tr class="run-row" data-run="${escapeHtml(r.run_id)}">${cells}</tr>`;
  }).join("");

  const aggHtml = metricsAggregatesHtml(chartKey);
  box.innerHTML = `
    <h3 style="margin:14px 0 6px; font-size:14px;">Run history (${runs.length})</h3>
    <div class="metrics-table-wrap"><table>
      <thead><tr>${head}</tr></thead><tbody>${body ||
        '<tr><td colspan="10" class="muted">No runs match these filters.</td></tr>'}</tbody>
    </table></div>
    <p class="metrics-note">Click a column to sort; click a row to load that run into the cards above. ⇩ downloads the full run JSON.</p>
    <div class="metrics-chart-grid">
      <div class="metrics-chart-box"><h4>${escapeHtml(chartKey.replace(/_/g, " "))} over time</h4>
        <canvas id="metrics-chart-time"></canvas></div>
      <div class="metrics-chart-box"><h4>Comparison${_mHistory.group_by ? ` by ${escapeHtml(_mHistory.group_by)}` : " (pick 'Compare by' and Apply)"}</h4>
        <canvas id="metrics-chart-compare"></canvas></div>
    </div>
    ${aggHtml}`;

  for (const th of box.querySelectorAll("th[data-sort]")) {
    th.addEventListener("click", () => {
      const key = th.dataset.sort;
      _mSort = { key, dir: _mSort.key === key ? -_mSort.dir : 1 };
      metricsRenderHistory();
    });
  }
  for (const tr of box.querySelectorAll("tr.run-row")) {
    tr.addEventListener("click", (ev) => {
      if (ev.target.closest("a")) return;
      _mSelectedRunId = tr.dataset.run;
      const sel = document.getElementById("metrics-run-select");
      if (sel) sel.value = _mSelectedRunId;
      metricsLoadSelectedRun();
      window.scrollTo({ top: 0, behavior: "smooth" });
    });
  }
  metricsDrawTimeChart(runs, chartKey);
  metricsDrawCompareChart(chartKey);
}

function metricsAggregatesHtml(chartKey) {
  const aggs = _mHistory.aggregates || {};
  const groups = Object.keys(aggs);
  if (!groups.length) return "";
  const rows = [];
  for (const g of groups) {
    const forKey = aggs[g][chartKey];
    const success = aggs[g].run_success_rate;
    if (!forKey) continue;
    rows.push(`<tr>
      <td>${escapeHtml(g)}</td>
      <td class="num">${forKey.n}</td>
      <td class="num">${mFmt(forKey.mean)}</td>
      <td class="num">${mFmt(forKey.std)}</td>
      <td class="num">${forKey.ci95_low !== null ? `${mFmt(forKey.ci95_low)} … ${mFmt(forKey.ci95_high)}` : "—"}</td>
      <td class="num">${mFmt(forKey.min)} / ${mFmt(forKey.max)}</td>
      <td class="num">${forKey.missing_n}</td>
      <td class="num">${success ? `${success.numerator}/${success.denominator}` : "—"}</td>
      <td class="num">${success ? `${success.failed_n} / ${success.aborted_n}` : "—"}</td>
      <td><a href="#" class="agg-runs-link" data-runs="${escapeHtml(forKey.contributing_run_ids.join(","))}">${forKey.contributing_run_ids.length} run(s)</a></td>
    </tr>`);
  }
  return `
    <h3 style="margin:14px 0 6px; font-size:14px;">Aggregate: ${escapeHtml(chartKey.replace(/_/g, " "))}</h3>
    <div class="metrics-table-wrap"><table>
      <thead><tr><th>group</th><th>n</th><th>mean</th><th>std</th>
        <th>95% CI</th><th>min / max</th><th>missing</th>
        <th>success</th><th>failed / aborted</th><th>contributing</th></tr></thead>
      <tbody>${rows.join("")}</tbody></table></div>
    <p class="metrics-note">valid n, missing counts, and the exact contributing runs are shown for every
      aggregate — failed runs are never silently dropped. Success-rate CI is a Wilson interval.</p>`;
}

// ---- charts (plain canvas; no external libraries) -------------------------

function mChartSetup(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, rect.width * dpr);
  canvas.height = Math.max(1, rect.height * dpr);
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  return { ctx, w: rect.width, h: rect.height };
}

function mChartAxes(ctx, w, h, pad, yMin, yMax) {
  ctx.strokeStyle = "#e2e8f0";
  ctx.lineWidth = 1;
  ctx.fillStyle = "#94a3b8";
  ctx.font = "10px 'Segoe UI', sans-serif";
  for (let i = 0; i <= 4; i++) {
    const y = pad.t + (h - pad.t - pad.b) * (i / 4);
    ctx.beginPath();
    ctx.moveTo(pad.l, y);
    ctx.lineTo(w - pad.r, y);
    ctx.stroke();
    const value = yMax - (yMax - yMin) * (i / 4);
    ctx.fillText(mFmt(value), 2, y + 3);
  }
}

function metricsDrawTimeChart(runs, chartKey) {
  const canvas = document.getElementById("metrics-chart-time");
  if (!canvas) return;
  const { ctx, w, h } = mChartSetup(canvas);
  const pts = runs
    .map((r) => ({ run: r, v: r.metrics[chartKey] && r.metrics[chartKey].value }))
    .filter((p) => p.v !== null && p.v !== undefined)
    .sort((a, b) => String(a.run.created_at_utc).localeCompare(String(b.run.created_at_utc)));
  if (!pts.length) {
    ctx.fillStyle = "#94a3b8";
    ctx.font = "11px 'Segoe UI', sans-serif";
    ctx.fillText("No stored values yet — finalize runs to populate history.", 10, h / 2);
    return;
  }
  const pad = { l: 38, r: 8, t: 8, b: 16 };
  const vs = pts.map((p) => p.v);
  let yMin = Math.min(...vs), yMax = Math.max(...vs);
  if (yMin === yMax) { yMin -= 1; yMax += 1; }
  const span = yMax - yMin;
  yMin -= span * 0.08; yMax += span * 0.08;
  mChartAxes(ctx, w, h, pad, yMin, yMax);
  const x = (i) => pts.length === 1
    ? (pad.l + w - pad.r) / 2
    : pad.l + (w - pad.l - pad.r) * (i / (pts.length - 1));
  const y = (v) => pad.t + (h - pad.t - pad.b) * (1 - (v - yMin) / (yMax - yMin));
  ctx.strokeStyle = "#2563eb";
  ctx.lineWidth = 2;
  ctx.beginPath();
  pts.forEach((p, i) => (i ? ctx.lineTo(x(i), y(p.v)) : ctx.moveTo(x(i), y(p.v))));
  ctx.stroke();
  pts.forEach((p, i) => {
    ctx.fillStyle = { completed: "#16a34a", failed: "#dc2626",
                      aborted: "#9333ea" }[p.run.status] || "#64748b";
    ctx.beginPath();
    ctx.arc(x(i), y(p.v), 3.5, 0, Math.PI * 2);
    ctx.fill();
  });
}

function metricsDrawCompareChart(chartKey) {
  const canvas = document.getElementById("metrics-chart-compare");
  if (!canvas) return;
  const { ctx, w, h } = mChartSetup(canvas);
  const aggs = _mHistory.aggregates || {};
  const groups = Object.keys(aggs)
    .map((g) => ({ g, s: aggs[g][chartKey] }))
    .filter((e) => e.s && e.s.mean !== null);
  if (!groups.length || (groups.length === 1 && groups[0].g === "all")) {
    ctx.fillStyle = "#94a3b8";
    ctx.font = "11px 'Segoe UI', sans-serif";
    ctx.fillText("Pick a 'Compare by' grouping and Apply filters.", 10, h / 2);
    return;
  }
  const pad = { l: 38, r: 8, t: 8, b: 30 };
  const highs = groups.map((e) => e.s.ci95_high ?? e.s.mean);
  const lows = groups.map((e) => e.s.ci95_low ?? e.s.mean);
  let yMax = Math.max(...highs), yMin = Math.min(0, ...lows);
  if (yMin === yMax) yMax += 1;
  mChartAxes(ctx, w, h, pad, yMin, yMax);
  const bw = (w - pad.l - pad.r) / groups.length;
  const y = (v) => pad.t + (h - pad.t - pad.b) * (1 - (v - yMin) / (yMax - yMin));
  groups.forEach((e, i) => {
    const cx = pad.l + bw * (i + 0.5);
    ctx.fillStyle = "#93b4f8";
    const barTop = y(e.s.mean);
    ctx.fillRect(cx - bw * 0.28, barTop, bw * 0.56, y(yMin <= 0 ? 0 : yMin) - barTop);
    if (e.s.ci95_low !== null && e.s.ci95_low !== undefined) {
      ctx.strokeStyle = "#1e3a8a";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(cx, y(e.s.ci95_low));
      ctx.lineTo(cx, y(e.s.ci95_high));
      ctx.stroke();
      for (const v of [e.s.ci95_low, e.s.ci95_high]) {
        ctx.beginPath();
        ctx.moveTo(cx - 4, y(v));
        ctx.lineTo(cx + 4, y(v));
        ctx.stroke();
      }
    }
    ctx.fillStyle = "#334155";
    ctx.font = "10px 'Segoe UI', sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(String(e.g).slice(0, 14), cx, h - 16);
    ctx.fillText(`n=${e.s.n}`, cx, h - 5);
    ctx.textAlign = "left";
  });
}

// ---- detail tabs ----------------------------------------------------------

const METRICS_TABS = [
  ["summary", "Run Summary"],
  ["robots", "Robots"],
  ["jobs", "Jobs & Workstations"],
  ["coordination", "Coordination & Safety"],
  ["routing", "Routing & Capacity"],
  ["localization", "Localization & Control"],
  ["energy", "Energy & Reliability"],
  ["definitions", "Metric Definitions"],
  ["quality", "Data Quality"],
];

function metricsBuildTabs() {
  const bar = document.getElementById("metrics-tabs");
  if (!bar) return;
  bar.innerHTML = METRICS_TABS.map(([key, label], i) =>
    `<button class="secondary${i === 0 ? " active" : ""}" data-mtab="${key}">${i + 1} · ${escapeHtml(label)}</button>`
  ).join("");
  bar.addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-mtab]");
    if (!btn) return;
    _mActiveTab = btn.dataset.mtab;
    for (const b of bar.querySelectorAll("button")) {
      b.className = b === btn ? "secondary active" : "secondary";
    }
    metricsRenderActiveTab();
  });
}

function metricsRenderActiveTab() {
  const box = document.getElementById("metrics-tab-content");
  if (!box) return;
  if (!_mMetrics) {
    box.innerHTML = '<p class="muted">Select a run above.</p>';
    return;
  }
  const renderers = {
    summary: metricsTabSummary, robots: metricsTabRobots,
    jobs: metricsTabJobs, coordination: metricsTabKeyed,
    routing: metricsTabKeyed, localization: metricsTabKeyed,
    energy: metricsTabKeyed, definitions: metricsTabDefinitions,
    quality: metricsTabQuality,
  };
  (renderers[_mActiveTab] || metricsTabSummary)(box);
}

// Metric keys per tab (run-scope table; non-run scopes render grouped below).
const METRICS_TAB_KEYS = {
  summary: ["actual_makespan_sec", "observed_duration_sec",
    "mission_completion_success", "intervention_free_success",
    "safe_completion_success", "job_completion_rate",
    "throughput_jobs_per_min", "observed_throughput_jobs_per_min",
    "job_flow_time_mean_sec", "job_flow_time_median_sec",
    "job_flow_time_std_sec", "job_flow_time_p95_sec", "job_flow_time_max_sec",
    "pickup_response_time_mean_sec", "loaded_transport_time_mean_sec",
    "job_tardiness_mean_sec", "robot_completion_spread_sec",
    "tail_completion_delay_sec", "planned_makespan_sec",
    "makespan_error_sec", "makespan_error_pct"],
  coordination: ["conflict_count", "conflict_rate_per_job",
    "auto_conflict_resolution_rate", "traffic_stop_episodes",
    "mean_traffic_stop_duration_sec", "fleet_traffic_delay_robot_sec",
    "traffic_delay_per_job_sec", "queue_delay_per_job_sec",
    "safety_stop_count", "reroute_count", "reroute_success_rate",
    "reroute_detour_distance_m", "reroute_detour_time_sec",
    "deadlock_count", "mean_deadlock_recovery_sec",
    "unresolved_deadlock_count", "collision_count",
    "min_fleet_separation_m", "near_miss_count",
    "time_below_warning_separation_sec", "emergency_stop_count",
    "manual_intervention_count"],
  routing: ["total_distance_m", "distance_per_job_m", "path_efficiency_pct",
    "detour_ratio_pct", "empty_travel_ratio_pct", "loaded_travel_ratio_pct",
    "payload_distance_utilization_pct", "item_distance_efficiency",
    "route_conformance_pct", "correct_service_rate", "jain_fairness_index",
    "workload_cv", "max_min_workload_ratio"],
  localization: ["vision_availability_pct", "pose_update_rate_hz",
    "pose_age_mean_ms", "pose_age_median_ms", "pose_age_p95_ms",
    "pose_age_max_ms", "vision_dropout_episodes",
    "path_tracking_error_mean_m", "terminal_condition_success_rate",
    "docking_success_rate", "yaw_mean_signed_error_deg", "yaw_mae_deg",
    "yaw_rmse_deg", "yaw_error_std_deg", "yaw_abs_error_median_deg",
    "yaw_abs_error_p95_deg", "yaw_abs_error_max_deg",
    "rotation_completion_rate", "rotation_within_tolerance_rate",
    "rotation_mean_duration_sec", "acceptable_turn_throughput_per_min",
    "rotation_time_error_product", "camera_correction_rate",
    "rotation_failure_rate", "command_ack_latency_sec",
    "command_completion_latency_sec", "replanning_frequency_per_job",
    "replanning_latency_sec", "advisor_decision_latency_sec",
    "decision_validity_rate", "advisor_fallback_rate"],
  energy: ["battery_drain_per_job_pct", "battery_drain_per_meter_pct",
    "battery_drain_imbalance_cv", "energy_per_job_wh", "energy_per_meter_wh",
    "command_success_rate", "robot_fault_count", "mean_recovery_time_sec",
    "manual_intervention_count"],
};

function metricsRowHtml(v) {
  const def = metricsDefFor(v.metric_key);
  const name = def ? def.display_name : v.metric_key;
  const unit = v.unit || (def && def.unit) || "";
  const frac = v.numerator !== null && v.numerator !== undefined &&
    v.denominator !== null && v.denominator !== undefined
    ? `${mFmt(v.numerator)} / ${mFmt(v.denominator)}` : "—";
  const statusCls = v.status === "missing" ? "metrics-value-missing"
    : v.status === "error" ? "metrics-value-error" : "";
  const shown = v.value === null || v.value === undefined ? "null"
    : v.metric_key.endsWith("_m") && typeof v.value === "number"
      ? mFmtMeters(v.value) : mFmt(v.value);
  return `<tr>
    <td title="${escapeHtml(v.metric_key)} v${v.definition_version}">${escapeHtml(name)}${v.scope_id ? ` <span class="muted">[${escapeHtml(v.scope_id)}]</span>` : ""}</td>
    <td class="num ${statusCls}">${escapeHtml(shown)}</td>
    <td>${escapeHtml(unit)}</td>
    <td class="num">${escapeHtml(frac)}</td>
    <td class="num">${v.valid_n ?? "—"}</td>
    <td class="num">${v.missing_n ?? "—"}</td>
    <td class="${statusCls}">${escapeHtml(v.status)}</td>
  </tr>`;
}

function metricsKeyedTableHtml(keys, title) {
  const runRows = [];
  const scopedRows = [];
  for (const key of keys) {
    for (const v of metricsValuesFor(key)) {
      (v.scope_type === "run" ? runRows : scopedRows).push(v);
    }
  }
  const table = (rows) => `
    <div class="metrics-table-wrap"><table>
      <thead><tr><th>metric</th><th>value</th><th>unit</th>
        <th>numerator / denominator</th><th>valid n</th><th>missing n</th>
        <th>status</th></tr></thead>
      <tbody>${rows.map(metricsRowHtml).join("")}</tbody></table></div>`;
  let html = `<h3>${escapeHtml(title)}</h3>`;
  html += runRows.length ? table(runRows)
    : '<p class="muted">No run-scope values in this family for this run.</p>';
  if (scopedRows.length) {
    html += `<h3>Per-robot / per-workstation detail</h3>${table(scopedRows)}`;
  }
  html += `<p class="metrics-note">null means the required source data was
    absent — never substituted with zero. Numerators/denominators are
    preserved for every ratio; failed attempts stay in denominators.</p>`;
  return html;
}

function metricsTabSummary(box) {
  box.innerHTML = metricsKeyedTableHtml(METRICS_TAB_KEYS.summary,
    "Run summary metrics");
}

function metricsTabKeyed(box) {
  const titles = { coordination: "Coordination & safety metrics",
                   routing: "Routing, capacity & workload balance",
                   localization: "Localization, rotation & control",
                   energy: "Energy & reliability" };
  box.innerHTML = metricsKeyedTableHtml(METRICS_TAB_KEYS[_mActiveTab],
    titles[_mActiveTab] || _mActiveTab);
}

async function metricsTabRobots(box) {
  box.innerHTML = '<p class="muted">Loading robots…</p>';
  let robots = [];
  try {
    robots = (await metricsFetch(`/api/runs/${_mSelectedRunId}/robots`)).robots;
  } catch (_) { /* table below still renders metric rows */ }
  const rows = robots.map((r) => `<tr>
    <td>${escapeHtml(r.robot_id)}</td>
    <td>${escapeHtml(r.status)}</td>
    <td class="num">${r.completion_elapsed_ms !== null ? mFmtSec(r.completion_elapsed_ms / 1000) : "—"}</td>
    <td class="num">${r.start_battery_pct ?? "—"} → ${r.end_battery_pct ?? "—"}</td>
    <td class="num">${r.terminal_yaw_deg !== null ? `${mFmt(r.terminal_yaw_deg)}°` : "—"}</td>
    <td>${escapeHtml(r.failure_reason || "")}</td>
  </tr>`).join("");
  const perRobotKeys = ["robot_productive_utilization_pct",
    "occupied_utilization_pct", "state_fraction_pct", "robot_distance_m",
    "battery_drop_pct", "battery_drain_rate_pct_per_min",
    "battery_drain_per_productive_min_pct", "energy_wh",
    "terminal_heading_error_deg", "robot_online_availability_pct",
    "state_time_reconciliation_error_sec"];
  box.innerHTML = `
    <h3>Robots in this run</h3>
    <div class="metrics-table-wrap"><table>
      <thead><tr><th>robot</th><th>status</th><th>completed at</th>
        <th>battery %</th><th>final yaw</th><th>failure reason</th></tr></thead>
      <tbody>${rows || '<tr><td colspan="6" class="muted">No robots registered for this run.</td></tr>'}</tbody>
    </table></div>
    ${metricsKeyedTableHtml(perRobotKeys, "Per-robot metrics")}`;
}

async function metricsTabJobs(box) {
  box.innerHTML = '<p class="muted">Loading jobs…</p>';
  let jobs = [];
  try {
    jobs = (await metricsFetch(`/api/runs/${_mSelectedRunId}/jobs`)).jobs;
  } catch (_) { /* fall through */ }
  const t = (ms) => ms !== null && ms !== undefined ? mFmtSec(ms / 1000) : "—";
  const rows = jobs.map((j) => `<tr>
    <td>${escapeHtml(j.job_id)}</td>
    <td>${escapeHtml(j.assigned_robot_id || "—")}</td>
    <td>${escapeHtml(j.workstation_id || "—")}</td>
    <td>${escapeHtml(j.status)}</td>
    <td class="num">${t(j.release_elapsed_ms)}</td>
    <td class="num">${t(j.delivery_elapsed_ms)}</td>
    <td class="num">${t(j.pickup_elapsed_ms)}</td>
    <td class="num">${t(j.completion_elapsed_ms)}</td>
    <td>${escapeHtml(j.failure_reason || "")}</td>
  </tr>`).join("");
  const wsKeys = ["workstation_processing_utilization_pct",
    "workstation_starvation_pct", "workstation_blocked_pct",
    "mean_workstation_queue_sec", "excess_workstation_dwell_sec"];
  box.innerHTML = `
    <h3>Jobs in this run</h3>
    <div class="metrics-table-wrap"><table>
      <thead><tr><th>job</th><th>robot</th><th>workstation</th><th>status</th>
        <th>released</th><th>delivered</th><th>picked up</th><th>completed</th>
        <th>failure</th></tr></thead>
      <tbody>${rows || '<tr><td colspan="9" class="muted">No jobs recorded for this run (job events come from the supervisor stream).</td></tr>'}</tbody>
    </table></div>
    ${metricsKeyedTableHtml(wsKeys, "Workstation metrics")}`;
}

function metricsDefFor(key) {
  if (!_mDefinitions) return null;
  // highest version wins for display
  let best = null;
  for (const d of _mDefinitions) {
    if (d.metric_key === key && (!best || d.version > best.version)) best = d;
  }
  return best;
}

async function metricsEnsureDefinitions() {
  if (_mDefinitions) return;
  try {
    _mDefinitions = (await metricsFetch("/api/metric-definitions",
      { timeoutMs: 8000 })).definitions;
  } catch (_) {
    _mDefinitions = null;  // stay null so a later call retries
  }
}

async function metricsTabDefinitions(box) {
  box.innerHTML = '<p class="muted">Loading definitions…</p>';
  await metricsEnsureDefinitions();
  if (!_mDefinitions) {
    box.innerHTML = '<p class="muted">Definitions could not be loaded — is the backend running?</p>';
    return;
  }
  const cards = _mDefinitions.map((d) => {
    let fields = [];
    try { fields = JSON.parse(d.required_fields_json || "[]"); } catch (_) {}
    return `<div class="metrics-def-card">
      <b>${escapeHtml(d.display_name)}</b>
      <span class="def-key">${escapeHtml(d.metric_key)} · v${d.version}</span>
      <div class="def-formula">${escapeHtml(d.formula_latex || "—")}</div>
      <div>${escapeHtml(d.description)}</div>
      <div class="def-meta">scope: ${escapeHtml(d.scope)} · unit: ${escapeHtml(d.unit || "—")} ·
        ${d.direction === "lower" ? "lower is better" : d.direction === "higher" ? "higher is better" : "no preferred direction"} ·
        effective ${escapeHtml(String(d.effective_at_utc).slice(0, 10))}</div>
      <div class="def-meta">required source fields: ${escapeHtml(fields.join(", ") || "—")}</div>
      <div class="def-meta">missing-data policy: ${escapeHtml(d.null_policy || "—")}</div>
    </div>`;
  }).join("");
  box.innerHTML = `
    <h3>Metric definition registry (${_mDefinitions.length} definitions)</h3>
    <p class="muted">Definitions are versioned and immutable — a formula change adds a new
      version; historical runs keep resolving to the version they were computed under.</p>
    ${cards}`;
}

async function metricsTabQuality(box) {
  const dq = _mMetrics && _mMetrics.data_quality;
  if (!dq) {
    box.innerHTML = '<p class="muted">No data-quality report available.</p>';
    return;
  }
  const issues = dq.issues.length
    ? dq.issues.map((i) => `<div class="metrics-dq-issue"><b>${escapeHtml(i.kind)}</b> — ${escapeHtml(i.detail)}</div>`).join("")
    : '<div class="metrics-dq-ok">No event-sequence or state-reconciliation issues detected.</div>';
  const acc = dq.attempt_accounting || {};
  const accRows = Object.entries(acc).map(([family, f]) => `<tr>
    <td>${escapeHtml(family)}</td>
    <td class="num">${f.attempted}</td><td class="num">${f.valid}</td>
    <td class="num">${f.completed}</td><td class="num">${f.failed}</td>
    <td class="num">${f.aborted}</td><td class="num">${f.missing}</td>
    <td class="num">${f.excluded_with_reason}</td>
  </tr>`).join("");
  const th = dq.thresholds || {};
  box.innerHTML = `
    <h3>Event-sequence & state checks</h3>
    ${issues}
    <h3>Attempt accounting</h3>
    <div class="metrics-table-wrap"><table>
      <thead><tr><th>family</th><th>attempted</th><th>valid</th>
        <th>completed</th><th>failed</th><th>aborted</th><th>missing</th>
        <th>excluded (with reason)</th></tr></thead>
      <tbody>${accRows}</tbody></table></div>
    <h3>Coverage & thresholds</h3>
    <div class="metrics-table-wrap"><table><tbody>
      <tr><td>telemetry coverage</td><td class="num">${dq.telemetry_coverage_pct !== null ? `${mFmt(dq.telemetry_coverage_pct, 1)}%` : "— (no telemetry stored)"}</td></tr>
      <tr><td>pose freshness limit (a_max)</td><td class="num">${th.pose_fresh_max_ms} ms</td></tr>
      <tr><td>separation warning distance (d_warn)</td><td class="num">${mFmtMeters(th.separation_warn_m)}</td></tr>
      <tr><td>collision distance (d_collision)</td><td class="num">${mFmtMeters(th.separation_collision_m)}</td></tr>
      <tr><td>terminal heading tolerance (tau_psi)</td><td class="num">${th.terminal_heading_tol_deg}°</td></tr>
      <tr><td>state reconciliation tolerance</td><td class="num">${th.state_reconcile_tol_sec} s</td></tr>
      <tr><td>yaw tolerance (tau)</td><td class="num">${th.yaw_tolerance_deg}°</td></tr>
    </tbody></table></div>
    <p class="metrics-note">Failed attempts are never deleted or silently excluded — they are
      classified, retained, and stay in every reliability denominator.</p>`;
}

// Definitions are needed for display names in every tab — warm the cache.
metricsEnsureDefinitions();
