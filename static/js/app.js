document.getElementById("build-grid").addEventListener("click", buildGrid);
document.getElementById("select-all").addEventListener("click", selectAll);
document.getElementById("clear-bays").addEventListener("click", clearBays);
document.getElementById("solve").addEventListener("click", solve);
document.getElementById("open-report").addEventListener("click", showReport);
document.getElementById("close-report").addEventListener("click", closeReport);
document.getElementById("download-json").addEventListener("click", downloadJson);
document.getElementById("dispatch-generate").addEventListener("click", dispatchGenerate);
document.getElementById("dispatch-start").addEventListener("click", dispatchStart);
document.getElementById("dispatch-hold").addEventListener("click", dispatchHold);
document.getElementById("dispatch-abort").addEventListener("click", dispatchAbort);
document.getElementById("sup-send").addEventListener("click", supervisorSendMission);
document.getElementById("sup-start").addEventListener("click", () => {
  if (supPublish("fleet_control", "START")) {
    state.missionStartMs = Date.now();  // anchors Real-mode status badges
    // Page 4 metrics: mark the registered run as started (fire-and-forget;
    // the supervisor's own RUN_STARTED event is the authoritative record).
    if (typeof metricsNotifyRunStarted === "function") {
      try { metricsNotifyRunStarted(); } catch (_) {}
    }
  }
});
document.getElementById("sup-abort").addEventListener("click", () => {
  if (supPublish("fleet_control", "ABORT")) {
    state.missionStartMs = 0;
    if (typeof metricsNotifyRunAborted === "function") {
      try { metricsNotifyRunAborted(); } catch (_) {}
    }
  }
});
document.getElementById("agents").addEventListener("input", recomputeIfSolved);
document.getElementById("capacity").addEventListener("input", recomputeIfSolved);
document.getElementById("rpd-mode").addEventListener("change", recomputeIfSolved);
document.getElementById("process-sec").addEventListener("input", recomputeIfSolved);
document.getElementById("import-plan").addEventListener("change", (ev) => {
  const file = ev.target.files && ev.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    try {
      importPlanFromJson(JSON.parse(reader.result), file.name);
    } catch (err) {
      alert(`Plan import failed: ${err.message}`);
    }
    ev.target.value = "";  // allow re-importing the same file
  };
  reader.readAsText(file);
});
document.getElementById("route-mode").addEventListener("change", recomputeIfSolved);
document.getElementById("seed").addEventListener("change", recomputeIfSolved);
document.getElementById("rows").addEventListener("change", buildGrid);
document.getElementById("cols").addEventListener("change", buildGrid);
document.getElementById("play-route").addEventListener("click", togglePlayback);
document.getElementById("step-route").addEventListener("click", stepRoute);
document.getElementById("timestep-slider").addEventListener("input", (event) => setTimestep(event.target.value));
document.getElementById("play-speed").addEventListener("change", () => {
  if (state.timer) startPlaybackTimer();
});
document.getElementById("mode-sim").addEventListener("click", () => setMode("sim"));
document.getElementById("mode-real").addEventListener("click", () => setMode("real"));
document.getElementById("mode-live").addEventListener("click", () => setMode("live"));
document.getElementById("drive-mode-color").addEventListener("click", () => setDriveMode("color"));
document.getElementById("drive-mode-vision").addEventListener("click", () => setDriveMode("vision"));
document.getElementById("fuse-turns-toggle").addEventListener("click", toggleFuseTurns);
document.getElementById("view-setup").addEventListener("click", () => setView("setup"));
document.getElementById("view-plan").addEventListener("click", () => setView("plan"));
document.getElementById("view-run").addEventListener("click", () => setView("run"));
document.getElementById("view-metrics").addEventListener("click", () => setView("metrics"));
document.getElementById("edit-setup").addEventListener("click", () => setView("setup"));
document.getElementById("decision-log-save").addEventListener("click", saveDecisionLog);
document.getElementById("decision-log-clear").addEventListener("click", clearDecisionLog);
renderDecisionLog();  // paint the empty-state hint on load
document.getElementById("side-tabs").addEventListener("click", (ev) => {
  const btn = ev.target.closest("button[data-tab]");
  if (!btn) return;
  for (const b of document.querySelectorAll("#side-tabs button")) {
    b.className = b === btn ? "secondary active" : "secondary";
  }
  for (const pane of document.querySelectorAll(".side-tab-pane")) {
    pane.classList.toggle("active", pane.id === `tab-${btn.dataset.tab}`);
  }
});
document.getElementById("toggle-edit-route").addEventListener("click", () => {
  state.editMode = !state.editMode;
  render();
});
document.getElementById("clear-route-edit").addEventListener("click", () => {
  setManualWaypoints(state.editAgent, []);
  rescheduleCurrentRoutes(true);
  render();
});
document.getElementById("route-modal").addEventListener("click", (event) => {
  if (event.target.id === "route-modal") closeReport();
});
document.getElementById("close-robot-topic-modal").addEventListener("click", closeRobotTopicModal);
document.getElementById("robot-topic-modal").addEventListener("click", (event) => {
  // event.target is the click's actual origin; a click on the backdrop
  // itself (not the card or anything inside it) has the modal wrapper as
  // its target, exactly like #route-modal's own click-outside handling.
  if (event.target.id === "robot-topic-modal") closeRobotTopicModal();
});
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" &&
      document.getElementById("robot-topic-modal").classList.contains("open")) {
    closeRobotTopicModal();
  }
});

// MAX_ROBOTS is the single source of truth for the depot's real physical
// slot count -- the #agents input's static `max` HTML attribute can't
// reference a JS constant directly, so it's overwritten here at startup
// instead of duplicating the number in the markup.
document.getElementById("agents").max = String(MAX_ROBOTS);

applyDefaultBays();
solve(false);

// Page 4 metrics bootstrap: probes the backend once so the nav button state
// and unavailable-banner are ready before the tab is first opened. Guarded --
// a missing/broken metrics.js can never affect the other three views.
if (typeof metricsInit === "function") {
  try { metricsInit(); } catch (err) { console.warn("metrics init error:", err); }
  try { vlmInit(); } catch (err) { console.warn("vlm init error:", err); }
}
