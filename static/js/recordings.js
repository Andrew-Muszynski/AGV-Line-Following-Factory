// ===================== Page 6 · Recordings ==================================
// Per-run videos of the annotated overhead camera (video_recorder.py on the
// Flask side). Two surfaces:
//   Run & Monitor -> the Recording row: an optional name for the next run's
//                    video, auto-record on/off, Record now / Stop recording,
//                    and a live REC status.
//   Recordings    -> the library of saved videos and a player that puts the
//                    run's notable events on the video's timeline.
// Loaded after metrics.js (uses metricsFetch, mFmtEastern, mShortId) and
// before app.js, which calls recordingsInit() through a typeof-guard, so a
// failure here can never break the other pages.

const REC_POLL_MS = 2000;
//: Seek this far before an event, so the lead-up is visible.
const REC_PREROLL_SEC = 2;
const REC_TRAFFIC_LABELS = {
  page: "Page plan", page_wait: "Page plan", astar: "A*", wait: "WAIT",
  pp: "PP", cbs: "CBS",
};
const REC_EVENT_LABELS = {
  RUN_STARTED: "Run started", LAUNCH_COMPLETED: "Left the depot",
  PROCESSING_STARTED: "Processing started",
  PROCESSING_COMPLETED: "Processing done", DROPOFF_COMPLETED: "Drop-off",
  DEPOT_ARRIVED: "Back at depot", RUN_COMPLETED: "Run completed",
  RUN_FAILED: "Run failed", RUN_ABORTED: "Run aborted",
  COLLISION: "Collision", COLLISION_AVOIDED: "Collision avoided",
  DEADLOCK_DETECTED: "Deadlock detected",
  DEADLOCK_RESOLUTION_STARTED: "Deadlock: resolving",
  DEADLOCK_RESOLVED: "Deadlock resolved",
  DEADLOCK_RESOLUTION_DECLINED: "Deadlock: move declined",
  DEADLOCK_RESOLUTION_FAILED: "Deadlock: resolution failed",
  COMMAND_FAILED: "Command failed", ROBOT_OFFLINE: "Robot offline",
};

let _recStatus = null;          // GET /api/recorder
let _recVideos = [];            // GET /api/videos
let _recSelected = null;        // the video row in the player
let _recMarkers = [];           // its run's events, on the video's clock
let _recWasRecordingId = null;  // to refresh the library when one finishes
let _recRowEls = [];

// ---- formatting -------------------------------------------------------------

function recFmtDur(sec) {
  if (sec === null || sec === undefined || Number.isNaN(sec)) return "—";
  const s = Math.max(0, Math.round(sec));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const r = String(s % 60).padStart(2, "0");
  return h ? `${h}:${String(m).padStart(2, "0")}:${r}` : `${m}:${r}`;
}

function recFmtStamp(sec) {
  if (sec === null || sec === undefined) return "—";
  const sign = sec < 0 ? "−" : "";
  const a = Math.abs(sec);
  const m = Math.floor(a / 60);
  return `${sign}${m}:${(a - m * 60).toFixed(1).padStart(4, "0")}`;
}

function recFmtBytes(b) {
  if (!b) return "—";
  return b >= 1e9 ? `${(b / 1e9).toFixed(2)} GB` : `${(b / 1e6).toFixed(0)} MB`;
}

function recTrafficLabel(m) {
  return REC_TRAFFIC_LABELS[m] || m || "";
}

// ---- recorder status (Run & Monitor row + library banner) -------------------

async function recorderPoll() {
  if (document.hidden) return;
  const live = _recStatus && _recStatus.recording;
  if (!["run", "recordings"].includes(state.view) && !live) return;
  try {
    _recStatus = await metricsFetch("/api/recorder");
  } catch (_) {
    _recStatus = null;
  }
  const nowId = _recStatus && _recStatus.recording
    ? _recStatus.recording.video_id : null;
  if (_recWasRecordingId !== null && nowId !== _recWasRecordingId &&
      state.view === "recordings") {
    recordingsLoadList();          // one just finished: show it
  }
  _recWasRecordingId = nowId;
  renderRecorderStatus();
}

function renderRecorderStatus() {
  const st = _recStatus;
  const auto = document.getElementById("rec-auto");
  const toggle = document.getElementById("rec-toggle");
  const label = document.getElementById("rec-status");
  renderRecordingBanner();
  if (!label || !auto || !toggle) return;
  if (!st || !st.enabled) {
    auto.disabled = true;
    toggle.disabled = true;
    label.className = "mode-status muted";
    label.textContent = st
      ? "Video recording is off in this app instance. Start the app with: python app.py"
      : "Recorder unreachable. Is app.py running?";
    return;
  }
  auto.disabled = false;
  toggle.disabled = false;
  const second = document.getElementById("rec-second");
  const sc = st.second || {};
  if (second) {
    second.disabled = !sc.configured;
    second.textContent = !sc.configured ? "Second camera: not set up"
      : sc.on ? "Second camera on" : "Second camera off";
    second.className = sc.configured && sc.on ? "secondary active" : "secondary";
  }
  auto.textContent = st.auto ? "Auto-record on" : "Auto-record off";
  auto.className = st.auto ? "secondary active" : "secondary";
  const r = st.recording;
  toggle.textContent = r ? "Stop recording" : "Record now";
  if (r) {
    let text = `REC ${recFmtDur(r.elapsed_sec)} · ${r.title}`;
    if (r.stalled) {
      text += r.camera_frames
        ? " · camera stream stalled"
        : " · waiting for the camera stream";
    }
    if (r.second_state === "starting") text += " · starting the second camera";
    else if (r.second_state === "recording") text += " · second camera recording";
    else if (r.second_state === "failed") text += ` · second camera: ${r.second_error}`;
    if (r.stopping_in_sec !== null && r.stopping_in_sec !== undefined) {
      text += ` · run ended, saving in ${Math.ceil(r.stopping_in_sec)} s`;
    }
    label.className = `mode-status ${r.stalled ? "rec-warn" : "rec-live"}`;
    label.innerHTML = `<span class="rec-dot"></span>${escapeHtml(text)}`;
    return;
  }
  let text = st.auto
    ? "Idle · the next run records when you press Start"
    : "Idle · auto-record is off; use Record now";
  if (st.auto && st.armed && (st.armed.next_name || st.armed.label)) {
    text = `Idle · the next run records as "${st.armed.next_name || st.armed.label}"`;
  }
  if (st.last && st.last.status === "failed") {
    text += ` · last recording failed: ${st.last.error}`;
  } else if (st.last) {
    text += ` · saved "${st.last.title}" (${recFmtDur(st.last.duration_sec)})`;
  }
  label.className = st.last && st.last.status === "failed"
    ? "mode-status rec-warn" : "mode-status muted";
  label.textContent = text;
}

function renderRecordingBanner() {
  const box = document.getElementById("rec-live");
  if (!box) return;
  const r = _recStatus && _recStatus.recording;
  box.hidden = !r;
  if (!r) return;
  box.innerHTML = `<span><span class="rec-dot"></span>Recording `
    + `<b>${escapeHtml(r.title)}</b> · ${escapeHtml(recFmtDur(r.elapsed_sec))}</span>`
    + `<button class="secondary" id="rec-live-stop">Stop</button>`;
  document.getElementById("rec-live-stop").onclick = recorderToggle;
}

async function recorderToggle() {
  const r = _recStatus && _recStatus.recording;
  const input = document.getElementById("rec-label");
  try {
    if (r) {
      _recStatus = await metricsFetch("/api/recorder/stop",
        { method: "POST", body: {}, timeoutMs: 20000 });
    } else {
      const label = input ? input.value.trim() : "";
      // The session's run, if it has one; the server drops it when that run
      // has already ended, so an unrelated recording never inherits it.
      const runId = typeof _mSessionRunId !== "undefined" ? _mSessionRunId : null;
      _recStatus = await metricsFetch("/api/recorder/start",
        { method: "POST", body: { run_id: runId || null, label: label || null } });
      if (input && label) input.value = "";
    }
  } catch (err) {
    alert(`Recording: ${err.message}`);
  }
  renderRecorderStatus();
  if (state.view === "recordings") recordingsLoadList();
}

async function recorderToggleSecond() {
  const on = !(_recStatus && _recStatus.second && _recStatus.second.on);
  try {
    _recStatus = await metricsFetch("/api/recorder/settings",
      { method: "POST", body: { second: on } });
  } catch (err) {
    alert(`Second camera: ${err.message}`);
  }
  renderRecorderStatus();
}

async function recorderToggleAuto() {
  const on = !(_recStatus && _recStatus.auto);
  try {
    _recStatus = await metricsFetch("/api/recorder/settings",
      { method: "POST", body: { auto: on } });
  } catch (err) {
    alert(`Auto-record: ${err.message}`);
  }
  renderRecorderStatus();
}

// ---- library ----------------------------------------------------------------

async function recordingsViewShown() {
  await recorderPoll();
  await recordingsLoadList();
}

async function recordingsLoadList() {
  const list = document.getElementById("rec-list");
  if (!list) return;
  try {
    const data = await metricsFetch("/api/videos?limit=500", { timeoutMs: 8000 });
    _recVideos = data.videos || [];
  } catch (err) {
    list.innerHTML = `<p class="muted">Could not load recordings: ${escapeHtml(err.message)}</p>`;
    return;
  }
  renderRecList();
  renderRecStorage();
  if (_recSelected) {
    const fresh = _recVideos.find((v) => v.video_id === _recSelected.video_id);
    if (!fresh) {
      recordingsSelect(null);
    } else if (fresh.status !== _recSelected.status ||
               fresh.file_exists !== _recSelected.file_exists) {
      recordingsSelect(fresh);     // e.g. it just finished recording
    } else {
      _recSelected = fresh;
      renderRecMeta();
    }
  } else if (_recVideos.length) {
    recordingsSelect(_recVideos.find((v) => v.status === "ready") || _recVideos[0]);
  } else {
    recordingsSelect(null);
  }
}

function recSearchText(v) {
  return [v.title, v.file_name, v.robot_count ? `${v.robot_count}r` : "",
    v.robot_count ? `${v.robot_count} robots` : "", v.traffic_method,
    recTrafficLabel(v.traffic_method), v.experimental_condition, v.status,
    mFmtEastern(v.started_at_utc), v.run_id ? mShortId(v.run_id) : ""]
    .join(" ").toLowerCase();
}

function renderRecList() {
  const list = document.getElementById("rec-list");
  const count = document.getElementById("rec-count");
  if (!list) return;
  const q = ((document.getElementById("rec-search") || {}).value || "")
    .toLowerCase().split(/\s+/).filter(Boolean);
  const shown = _recVideos.filter((v) => {
    const hay = recSearchText(v);
    return q.every((tok) => hay.includes(tok));
  });
  if (count) {
    count.textContent = q.length
      ? `${shown.length} of ${_recVideos.length}` : `${_recVideos.length}`;
  }
  if (!shown.length) {
    list.innerHTML = `<p class="muted" style="margin:4px 0;">${_recVideos.length
      ? "No recordings match this filter."
      : "Nothing recorded yet."}</p>`;
    return;
  }
  list.innerHTML = shown.map((v) => {
    const sub = [v.robot_count ? `${v.robot_count} robots` : null,
      recTrafficLabel(v.traffic_method) || null,
      v.actual_makespan_sec !== null && v.actual_makespan_sec !== undefined
        ? `makespan ${recFmtDur(v.actual_makespan_sec)}` : null]
      .filter(Boolean).join(" · ") || (v.run_id ? "" : "not linked to a run");
    const meta = [mFmtEastern(v.started_at_utc),
      v.status === "recording" ? "recording now" : recFmtDur(v.duration_sec),
      v.bytes ? recFmtBytes(v.bytes) : null].filter(Boolean).join(" · ");
    const pill = v.status === "ready" ? ""
      : `<span class="rec-pill rec-pill-${escapeHtml(v.status)}">${escapeHtml(v.status)}</span>`;
    const active = _recSelected && _recSelected.video_id === v.video_id;
    return `<button class="rec-item${active ? " active" : ""}" data-video="${v.video_id}"`
      + ` aria-pressed="${active}">`
      + `<span class="rec-item-title">${escapeHtml(v.title)}</span>${pill}`
      + `<span class="rec-item-sub">${escapeHtml(sub)}</span>`
      + `<span class="rec-item-meta">${escapeHtml(meta)}</span></button>`;
  }).join("");
}

function renderRecStorage() {
  const p = document.getElementById("rec-storage");
  if (!p) return;
  const st = _recStatus;
  if (!st || !st.enabled) { p.textContent = ""; return; }
  const total = _recVideos.reduce((sum, v) => sum + (v.bytes || 0), 0);
  p.textContent = `Saved in ${st.video_dir} · ${recFmtBytes(total)} used`
    + (st.disk_free_gb !== null && st.disk_free_gb !== undefined
      ? ` · ${st.disk_free_gb >= 1000 ? (st.disk_free_gb / 1000).toFixed(1) + " TB"
        : st.disk_free_gb.toFixed(0) + " GB"} free` : "");
}

// Open a video from anywhere (Page 4's Watch video, the history table).
async function recordingsOpenVideo(videoId) {
  if (state.view !== "recordings") setView("recordings");
  if (!_recVideos.some((v) => v.video_id === videoId)) await recordingsLoadList();
  const v = _recVideos.find((x) => x.video_id === videoId);
  if (v) recordingsSelect(v);
}

// ---- player -----------------------------------------------------------------

function recordingsSelect(v) {
  _recSelected = v;
  _recMarkers = [];
  for (const b of document.querySelectorAll("#rec-list .rec-item")) {
    const on = v && Number(b.dataset.video) === v.video_id;
    b.classList.toggle("active", Boolean(on));
    b.setAttribute("aria-pressed", String(Boolean(on)));
  }
  const empty = document.getElementById("rec-empty");
  const detail = document.getElementById("rec-detail");
  const video = document.getElementById("rec-video");
  if (!detail || !video) return;
  if (!v) {
    if (empty) empty.hidden = false;
    detail.hidden = true;
    video.removeAttribute("src");
    video.load();
    recLoadSecond(null, false);
    return;
  }
  if (empty) empty.hidden = true;
  detail.hidden = false;
  const title = document.getElementById("rec-title");
  title.value = v.title;
  document.getElementById("rec-save-title").hidden = true;

  const note = document.getElementById("rec-stage-note");
  const playable = v.status === "ready" && v.file_exists && v.browser_playable;
  let why = "";
  if (v.status === "recording") {
    why = "Still recording. The video can be played as soon as the recording stops.";
  } else if (v.status === "failed") {
    why = `This recording failed: ${v.error || "unknown error"}`;
  } else if (v.status === "interrupted") {
    why = "Interrupted: the app stopped while this was recording, so the file "
      + "was never finished and will not play.";
  } else if (!v.file_exists) {
    why = "The video file is missing from the videos folder.";
  } else if (!v.browser_playable) {
    why = `This file was saved as ${String(v.codec || "").toUpperCase()}, which `
      + "browsers cannot play. Use Download to watch it.";
  }
  note.hidden = !why;
  note.textContent = why;
  if (playable) {
    const src = `${v.url}`;
    if (video.getAttribute("src") !== src) {
      video.src = src;
      video.playbackRate = recCurrentRate();
    }
  } else {
    video.removeAttribute("src");
    video.load();
  }
  recLoadSecond(v, playable);
  document.getElementById("rec-download").disabled = !(v.file_exists && v.status !== "recording");
  document.getElementById("rec-delete").disabled = v.status === "recording";
  document.getElementById("rec-open-run").disabled = !v.run_id;
  renderRecMeta();
  recLoadMarkers(v.video_id);
}

// ---- two cameras ---------------------------------------------------------------
// The second overhead camera records on its own laptop; its file is played
// straight from there. The timeline, events and seeking all run on the MAIN
// video's clock: second-video time = main time - second_offset_sec. Whichever
// video is on screen with controls is the master; in Side by side the second
// follows the main one.
let _recCam = "main";
let _recOutside = false;     // main time is beyond the second clip

function recMainVideo() { return document.getElementById("rec-video"); }
function recSecondVideo() { return document.getElementById("rec-video-2"); }
function recOffset2() {
  return (_recSelected && Number(_recSelected.second_offset_sec)) || 0;
}
function recHasSecond() {
  const s = recSecondVideo();
  return Boolean(s && s.getAttribute("src"));
}
function recMaster() {
  return _recCam === "second" && recHasSecond() ? recSecondVideo() : recMainVideo();
}

// Current time on the main video's clock.
function recNow() {
  const m = recMaster();
  return m === recSecondVideo() ? m.currentTime + recOffset2() : m.currentTime;
}

function recSetNow(t, play) {
  const m = recMaster();
  if (!m || !m.getAttribute("src")) return;
  const local = m === recSecondVideo() ? t - recOffset2() : t;
  const dur = Number.isFinite(m.duration) && m.duration > 0 ? m.duration : null;
  m.currentTime = Math.max(0, dur ? Math.min(local, dur) : local);
  if (play) m.play().catch(() => {});
  recFollow(true);
}

function recFollow(force) {
  if (_recCam !== "both" || !recHasSecond()) return;
  const m = recMainVideo(), s = recSecondVideo();
  const want = m.currentTime - recOffset2();
  s.playbackRate = m.playbackRate;
  const end = Number.isFinite(s.duration) ? s.duration : Infinity;
  if (want < 0 || want > end) {      // this moment is outside the second clip
    if (!s.paused) s.pause();
    s.currentTime = Math.max(0, Math.min(want, Number.isFinite(end) ? end : 0));
    // Say so, rather than leave a frozen frame that looks like a still.
    if (!_recOutside) {
      _recOutside = true;
      const off = recOffset2();
      recRenderSync(Number.isFinite(end)
        ? `second camera only covers ${recFmtStamp(Math.max(0, off))}–${recFmtStamp(off + end)} of this video`
        : "second camera has no footage at this point");
    }
    return;
  }
  if (_recOutside) { _recOutside = false; recRenderSync(); }
  if (force || Math.abs(s.currentTime - want) > 0.3) s.currentTime = want;
  if (m.paused && !s.paused) s.pause();
  if (!m.paused && s.paused) s.play().catch(() => {});
}

function recLoadSecond(v, mainPlayable) {
  const s = recSecondVideo();
  const row = document.getElementById("rec-cam-row");
  if (!s || !row) return;
  const src = v && mainPlayable && v.second_camera_file && v.second_camera_url
    ? `${v.second_camera_url}/videos/${encodeURIComponent(v.second_camera_file)}` : "";
  row.hidden = !src;
  if (!src) {
    s.pause();
    s.removeAttribute("src");
    s.load();
    recSetCam("main", true);
    return;
  }
  if (s.getAttribute("src") !== src) s.src = src;
  recRenderSync();
  recSetCam(_recCam, true);
}

function recRenderSync(problem) {
  const span = document.getElementById("rec-sync");
  if (!span) return;
  const off = recOffset2();
  span.textContent = problem
    || `second camera ${off >= 0 ? "starts" : "started"} ${Math.abs(off).toFixed(1)} s ${off >= 0 ? "into" : "before"} the main video`;
  span.className = problem ? "mode-status rec-warn" : "mode-status muted";
}

function recSetCam(cam, quiet) {
  const m = recMainVideo(), s = recSecondVideo();
  if (!m || !s) return;
  if (!recHasSecond()) cam = "main";
  const t = quiet ? null : recNow();
  const playing = !quiet && !recMaster().paused;
  if (!quiet) { m.pause(); s.pause(); }
  _recCam = cam;
  for (const b of document.querySelectorAll("#rec-cam button")) {
    b.className = b.dataset.cam === cam ? "secondary active" : "secondary";
  }
  m.hidden = cam === "second";
  s.hidden = cam === "main";
  m.controls = cam !== "second";
  s.controls = cam === "second";
  const stage = m.closest(".rec-stage");
  if (stage) stage.classList.toggle("cam-both", cam === "both");
  if (t !== null) recSetNow(t, playing);
}

async function recNudge(delta) {
  const v = _recSelected;
  if (!v) return;
  const next = Math.round((recOffset2() + delta) * 10) / 10;
  try {
    const row = await metricsFetch(`/api/videos/${v.video_id}`,
      { method: "PATCH", body: { second_offset_sec: next } });
    v.second_offset_sec = row.second_offset_sec;
  } catch (err) {
    alert(`Sync: ${err.message}`);
    return;
  }
  recRenderSync();
  if (_recCam === "second") recSetNow(recNow(), !recMaster().paused);
  recFollow(true);
}

function recCurrentRate() {
  const on = document.querySelector("#rec-speed button.active");
  return on ? Number(on.dataset.rate) : 1;
}

function recDuration() {
  const video = document.getElementById("rec-video");
  if (video && Number.isFinite(video.duration) && video.duration > 0) return video.duration;
  return (_recSelected && _recSelected.duration_sec) || 0;
}

function renderRecMeta() {
  const dl = document.getElementById("rec-meta");
  const v = _recSelected;
  if (!dl || !v) return;
  const effFps = v.duration_sec ? (v.camera_frames || 0) / v.duration_sec : null;
  const items = [
    ["Run", v.run_id ? `${mShortId(v.run_id)} · ${v.run_status || "?"}` : "not linked to a run"],
    ["Traffic planner", recTrafficLabel(v.traffic_method) || "—"],
    ["Robots", v.robot_count ?? "—"],
    ["Makespan", v.actual_makespan_sec !== null && v.actual_makespan_sec !== undefined
      ? `${recFmtDur(v.actual_makespan_sec)}${v.planned_makespan_sec
        ? ` (plan ${recFmtDur(v.planned_makespan_sec)})` : ""}` : "—"],
    ["Condition", v.experimental_condition || "—"],
    ["Recorded", mFmtEastern(v.started_at_utc, { seconds: true, tzName: true })],
    ["Length", v.status === "recording" ? "recording now"
      : `${recFmtDur(v.duration_sec)} · ${recFmtBytes(v.bytes)}`],
    ["Camera frames", v.camera_frames !== null && v.camera_frames !== undefined
      ? `${v.camera_frames} distinct${effFps ? ` (${effFps.toFixed(1)} fps)` : ""}` : "—"],
    ["Stream stalls", v.stall_count ? `${v.stall_count} · longest ${v.max_stall_sec} s` : (v.status === "ready" ? "none" : "—"), v.stall_count > 0],
    ["Ended because", v.stop_reason || "—"],
    ["Run clock", v.run_t0_offset_sec === null || v.run_t0_offset_sec === undefined
      ? "no run start received"
      : `run starts at ${recFmtStamp(v.run_t0_offset_sec)} (${v.offset_source})`],
    ["File", v.file_name || "—"],
    ["Second camera", v.second_camera_file
      || (v.second_error ? `not recorded: ${v.second_error}` : "not used"), Boolean(v.second_error)],
  ];
  dl.innerHTML = items.map(([k, val, bad]) =>
    `<div><dt>${escapeHtml(k)}</dt><dd${bad ? ' class="rec-bad"' : ""}>${escapeHtml(String(val))}</dd></div>`
  ).join("");
}

async function recLoadMarkers(videoId) {
  const body = document.getElementById("rec-events");
  if (body) body.innerHTML = '<tr><td colspan="5" class="muted">Loading events…</td></tr>';
  let data = null;
  try {
    data = await metricsFetch(`/api/videos/${videoId}/markers`, { timeoutMs: 8000 });
  } catch (err) {
    if (body) body.innerHTML = `<tr><td colspan="5" class="muted">Could not load events: ${escapeHtml(err.message)}</td></tr>`;
    return;
  }
  if (!_recSelected || _recSelected.video_id !== videoId) return;   // moved on
  _recMarkers = data.markers || [];
  renderRecMarkers();
}

function recVisibleMarkers() {
  const show = {
    safety: (document.getElementById("rec-show-safety") || {}).checked !== false,
    progress: (document.getElementById("rec-show-progress") || {}).checked !== false,
  };
  return _recMarkers.filter((m) => show[m.category] && m.t_video !== null);
}

function renderRecMarkers() {
  const body = document.getElementById("rec-events");
  const ticks = document.getElementById("rec-ticks");
  if (!body || !ticks) return;
  const v = _recSelected;
  const shown = recVisibleMarkers();
  const dur = recDuration();

  // Timeline: the run's span as a band, then one tick per event.
  let html = "";
  if (dur > 0 && v && v.run_t0_offset_sec !== null && v.run_t0_offset_sec !== undefined) {
    const end = _recMarkers.filter((m) => /^RUN_(COMPLETED|FAILED|ABORTED)$/.test(m.event_type)
      && m.t_video !== null).map((m) => m.t_video).pop();
    const a = Math.max(0, v.run_t0_offset_sec) / dur * 100;
    const b = Math.min(dur, end ?? dur) / dur * 100;
    if (b > a) html += `<span class="rec-run-span" style="left:${a.toFixed(2)}%;width:${(b - a).toFixed(2)}%"></span>`;
  }
  if (dur > 0) {
    html += shown.filter((m) => m.t_video >= 0 && m.t_video <= dur).map((m) =>
      `<span class="rec-tick rec-tick-${m.category}" style="left:${(m.t_video / dur * 100).toFixed(2)}%"`
      + ` title="${escapeHtml(recFmtStamp(m.t_video))} ${escapeHtml(REC_EVENT_LABELS[m.event_type] || m.event_type)}`
      + `${m.robots ? " · " + escapeHtml(m.robots) : ""}"></span>`).join("");
  }
  ticks.innerHTML = html;

  if (!_recMarkers.length) {
    const why = !v || !v.run_id
      ? "This recording is not linked to a run, so there are no events to show."
      : v.run_t0_offset_sec === null || v.run_t0_offset_sec === undefined
        ? "No run start reached the recorder, so events cannot be placed on this video."
        : "No notable events were recorded for this run.";
    body.innerHTML = `<tr><td colspan="5" class="muted">${escapeHtml(why)}</td></tr>`;
    _recRowEls = [];
    return;
  }
  body.innerHTML = shown.map((m, i) =>
    `<tr class="rec-ev-${m.category}" data-i="${i}">`
    + `<td class="col-num">${escapeHtml(recFmtStamp(m.t_video))}</td>`
    + `<td class="col-num">${m.elapsed_sec === null ? "—" : "+" + escapeHtml(recFmtStamp(m.elapsed_sec))}</td>`
    + `<td class="col-left">${escapeHtml(REC_EVENT_LABELS[m.event_type] || m.event_type)}</td>`
    + `<td class="col-left">${escapeHtml(m.robots || "")}</td>`
    + `<td class="col-left">${escapeHtml(m.text || "")}</td></tr>`).join("")
    || '<tr><td colspan="5" class="muted">Every event type is filtered out.</td></tr>';
  _recRowEls = [...body.querySelectorAll("tr[data-i]")];
  for (const tr of _recRowEls) {
    tr.addEventListener("click", () => {
      recSeek(shown[Number(tr.dataset.i)].t_video - REC_PREROLL_SEC, true);
    });
  }
  recSyncPlayhead();
}

function recSeek(t, play) {
  recSetNow(Math.max(0, Math.min(t, recDuration() || t)), play);
}

function recJump(direction) {
  if (!recMaster()) return;
  const now = recNow() + REC_PREROLL_SEC;
  const times = recVisibleMarkers().map((m) => m.t_video);
  const target = direction > 0
    ? times.find((t) => t > now + 0.25)
    : [...times].reverse().find((t) => t < now - 0.75);
  if (target !== undefined) recSeek(target - REC_PREROLL_SEC, false);
}

function recSyncPlayhead(ev) {
  const head = document.getElementById("rec-playhead");
  if (!head || !recMaster()) return;
  if (ev && ev.target && ev.target !== recMaster()) return;   // the follower
  const now = recNow();
  const dur = recDuration();
  head.style.left = dur > 0 ? `${(Math.min(now, dur) / dur * 100).toFixed(3)}%` : "0";
  // Highlight the latest event at or before the playhead.
  const shown = recVisibleMarkers();
  let idx = -1;
  for (let i = 0; i < shown.length; i += 1) {
    if (shown[i].t_video <= now + 0.05) idx = i; else break;
  }
  _recRowEls.forEach((tr, i) => tr.classList.toggle("rec-now", i === idx));
}

// ---- actions ------------------------------------------------------------------

async function recSaveTitle() {
  const v = _recSelected;
  const input = document.getElementById("rec-title");
  if (!v || !input) return;
  try {
    const row = await metricsFetch(`/api/videos/${v.video_id}`,
      { method: "PATCH", body: { title: input.value } });
    Object.assign(v, { title: row.title });
    document.getElementById("rec-save-title").hidden = true;
    renderRecList();
  } catch (err) {
    alert(`Rename: ${err.message}`);
  }
}

async function recDelete() {
  const v = _recSelected;
  if (!v) return;
  const size = v.bytes ? ` (${recFmtBytes(v.bytes)})` : "";
  if (!confirm(`Delete "${v.title}"${size}? The video file is removed from disk and cannot be recovered. The run's metrics are kept.`)) return;
  try {
    await metricsFetch(`/api/videos/${v.video_id}`, { method: "DELETE" });
  } catch (err) {
    alert(`Delete: ${err.message}`);
    return;
  }
  _recSelected = null;
  await recordingsLoadList();
}

function recOpenRun() {
  const v = _recSelected;
  if (!v || !v.run_id) return;
  _mSelectedRunId = v.run_id;
  setView("metrics");
}

// ---- wiring -----------------------------------------------------------------

function recordingsInit() {
  const on = (id, ev, fn) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener(ev, fn);
  };
  on("rec-toggle", "click", recorderToggle);
  on("rec-auto", "click", recorderToggleAuto);
  on("rec-refresh", "click", () => { recorderPoll(); recordingsLoadList(); });
  on("rec-search", "input", renderRecList);
  on("rec-list", "click", (ev) => {
    const b = ev.target.closest("button.rec-item");
    if (!b) return;
    const v = _recVideos.find((x) => x.video_id === Number(b.dataset.video));
    if (v) recordingsSelect(v);
  });
  on("rec-title", "input", () => {
    const changed = _recSelected &&
      document.getElementById("rec-title").value.trim() !== _recSelected.title;
    document.getElementById("rec-save-title").hidden = !changed;
  });
  on("rec-title", "keydown", (ev) => {
    if (ev.key === "Enter") recSaveTitle();
    if (ev.key === "Escape" && _recSelected) {
      ev.target.value = _recSelected.title;
      document.getElementById("rec-save-title").hidden = true;
    }
  });
  on("rec-save-title", "click", recSaveTitle);
  on("rec-download", "click", () => {
    if (_recSelected) window.location.href = `${_recSelected.url}?download=1`;
  });
  on("rec-delete", "click", recDelete);
  on("rec-open-run", "click", recOpenRun);
  on("rec-prev", "click", () => recJump(-1));
  on("rec-next", "click", () => recJump(1));
  on("rec-show-safety", "change", renderRecMarkers);
  on("rec-show-progress", "change", renderRecMarkers);
  on("rec-speed", "click", (ev) => {
    const b = ev.target.closest("button[data-rate]");
    if (!b) return;
    for (const x of document.querySelectorAll("#rec-speed button")) {
      x.className = x === b ? "secondary active" : "secondary";
    }
    recMainVideo().playbackRate = Number(b.dataset.rate);
    recSecondVideo().playbackRate = Number(b.dataset.rate);
  });
  on("rec-cam", "click", (ev) => {
    const b = ev.target.closest("button[data-cam]");
    if (b) recSetCam(b.dataset.cam, false);
  });
  on("rec-sync-minus", "click", () => recNudge(0.1));
  on("rec-sync-plus", "click", () => recNudge(-0.1));
  on("rec-second", "click", recorderToggleSecond);
  const v2 = recSecondVideo();
  if (v2) {
    v2.addEventListener("timeupdate", recSyncPlayhead);
    v2.addEventListener("seeked", recSyncPlayhead);
    v2.addEventListener("loadedmetadata", () => {
      v2.playbackRate = recCurrentRate();
      recRenderSync();
      recFollow(true);
    });
    v2.addEventListener("error", () => {
      if (!v2.getAttribute("src")) return;
      recRenderSync("second camera's video unavailable: is its laptop (192.168.0.73) on and its recorder running?");
    });
  }
  on("rec-timeline", "click", (ev) => {
    const box = ev.currentTarget.getBoundingClientRect();
    recSeek((ev.clientX - box.left) / box.width * recDuration(), false);
  });
  const video = document.getElementById("rec-video");
  if (video) {
    video.addEventListener("timeupdate", (ev) => { recSyncPlayhead(ev); recFollow(false); });
    video.addEventListener("seeked", (ev) => { recSyncPlayhead(ev); recFollow(true); });
    for (const evName of ["play", "pause", "ratechange"]) {
      video.addEventListener(evName, () => recFollow(true));
    }
    // Ticks are placed against the real duration once the file reports it.
    video.addEventListener("loadedmetadata", () => {
      video.playbackRate = recCurrentRate();
      renderRecMarkers();
    });
  }
  on("metrics-watch-video", "click", (ev) => {
    const id = Number(ev.currentTarget.dataset.video);
    if (id) recordingsOpenVideo(id);
  });
  recorderPoll();
  setInterval(recorderPoll, REC_POLL_MS);
}
