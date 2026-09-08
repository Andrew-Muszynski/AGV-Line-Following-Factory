"""Metric-definition registry for the AGV testbed metrics backend.

Every metric displayed on Page 4 references one (metric_key, version) row
here. Definitions are IMMUTABLE once effective: if a formula changes, add a
new version row -- never edit or delete the old one, because historical runs
store the definition_version they were computed under and must keep
resolving to the formula that produced them.

Canonical units: seconds, meters, degrees, percent, watt-hours. Rates name
their basis explicitly in the unit string.

Scope legend: run | robot | job | workstation | conflict | aggregate.
The engine may additionally emit per-robot rows for run-scope families
(metric_values.scope_type distinguishes them); `scope` here names the
PRIMARY scope the definition is written for.
"""
from __future__ import annotations

import json

DEFINITION_SET_VERSION = 1
EFFECTIVE_AT = "2026-08-20T00:00:00+00:00"

# Default missing-data policy (spec section 21): a metric whose required
# source fields are absent is stored as NULL with status='missing' and the
# missing fields reported -- never substituted with zero, and failed
# attempts always stay in reliability denominators.
NULL_POLICY = ("null-when-missing; failed attempts stay in denominators; "
               "no zero substitution")


def _d(key, name, scope, unit, direction, formula, desc, fields,
       null_policy=NULL_POLICY):
    return {
        "metric_key": key,
        "version": 1,
        "display_name": name,
        "scope": scope,
        "description": desc,
        "formula_latex": formula,
        "unit": unit,
        "direction": direction,
        "required_fields": fields,
        "null_policy": null_policy,
        "effective_at_utc": EFFECTIVE_AT,
    }


DEFINITIONS: list[dict] = [
    # ---- 7. Run and system -------------------------------------------------
    _d("actual_makespan_sec", "Actual makespan", "run", "s", "lower",
       r"T = t_f - t_0",
       "Monotonic seconds from the first real mission command (RUN_STARTED) "
       "to the final required robot reaching its depot slot inside tolerance "
       "and facing the terminal heading. NULL for failed/aborted runs.",
       ["events.RUN_STARTED", "events.RUN_COMPLETED"]),
    _d("observed_duration_sec", "Observed run duration", "run", "s", "neutral",
       r"T_{obs} = t_{stop} - t_0",
       "Duration of an aborted or failed run, from start to the abort/failure "
       "event. Not a makespan; never compared against successful-run makespans.",
       ["events.RUN_STARTED", "events.RUN_FAILED|RUN_ABORTED"]),
    _d("mission_completion_success", "Mission completion", "run", "0/1", "higher",
       r"S_r \in \{0,1\}",
       "1 only if every required job completed correctly AND every required "
       "robot returned to its depot location AND met the final-heading "
       "tolerance. Automatic recovery is permitted.",
       ["events.RUN_COMPLETED", "jobs.status", "run_robots.status"]),
    _d("intervention_free_success", "Intervention-free completion", "run",
       "0/1", "higher", r"S_r \cdot \mathbf{1}(\text{no manual intervention})",
       "1 when the mission completed with no manual repositioning, no manual "
       "command, no operator reset, and no unresolved deadlock.",
       ["events.MANUAL_INTERVENTION", "conflicts.status"]),
    _d("safe_completion_success", "Safe completion", "run", "0/1", "higher",
       r"S_r \cdot \mathbf{1}(N_{collision}=0)",
       "1 when the mission completed and the collision count was zero.",
       ["events.COLLISION"]),
    _d("run_success_rate", "Run success rate", "aggregate", "ratio", "higher",
       r"\frac{\sum_r S_r}{R}",
       "Successful runs over all runs in the aggregate; shown with "
       "numerator, denominator, and a 95% confidence interval.",
       ["runs.status"]),
    _d("job_completion_rate", "Job completion rate", "run", "ratio", "higher",
       r"\frac{J_C}{J_A}",
       "Completed jobs over attempted/released jobs. Failed jobs stay in the "
       "denominator.",
       ["jobs.status"]),
    _d("throughput_jobs_per_min", "Throughput", "run", "jobs/min", "higher",
       r"60\frac{J_C}{T}",
       "Completed jobs per minute of actual makespan. NULL for unsuccessful "
       "runs (see observed_throughput_jobs_per_min).",
       ["jobs.status", "actual_makespan_sec"]),
    _d("observed_throughput_jobs_per_min", "Observed throughput (incomplete run)",
       "run", "jobs/min", "neutral", r"60\frac{J_C}{T_{obs}}",
       "Explicitly-labeled throughput over the observed duration of an "
       "unsuccessful run. Never presented as completed-run throughput.",
       ["jobs.status", "observed_duration_sec"]),
    _d("job_flow_time_mean_sec", "Job flow time (mean)", "run", "s", "lower",
       r"\overline{F_j},\; F_j = t^C_j - t^R_j",
       "Mean completed-job flow time (release to completion).",
       ["jobs.release_elapsed_ms", "jobs.completion_elapsed_ms"]),
    _d("job_flow_time_median_sec", "Job flow time (median)", "run", "s",
       "lower", r"\mathrm{median}(F_j)", "Median completed-job flow time.",
       ["jobs.release_elapsed_ms", "jobs.completion_elapsed_ms"]),
    _d("job_flow_time_std_sec", "Job flow time (std dev)", "run", "s",
       "lower", r"s(F_j)", "Sample standard deviation of job flow time.",
       ["jobs.release_elapsed_ms", "jobs.completion_elapsed_ms"]),
    _d("job_flow_time_max_sec", "Job flow time (max)", "run", "s", "lower",
       r"\max_j F_j", "Maximum completed-job flow time.",
       ["jobs.release_elapsed_ms", "jobs.completion_elapsed_ms"]),
    _d("job_flow_time_p95_sec", "Job flow time (95th pct)", "run", "s",
       "lower", r"P_{95}(F_j)", "95th percentile of job flow time.",
       ["jobs.release_elapsed_ms", "jobs.completion_elapsed_ms"]),
    _d("pickup_response_time_mean_sec", "Pickup response time (mean)", "run",
       "s", "lower", r"\overline{R_j},\; R_j = t^P_j - t^R_j",
       "Mean time from job release to pickup.",
       ["jobs.release_elapsed_ms", "jobs.pickup_elapsed_ms"]),
    _d("loaded_transport_time_mean_sec", "Loaded transport time (mean)",
       "run", "s", "lower", r"\overline{L_j},\; L_j = t^D_j - t^P_j",
       "Mean time from pickup to delivery.",
       ["jobs.pickup_elapsed_ms", "jobs.delivery_elapsed_ms"]),
    _d("job_tardiness_mean_sec", "Job tardiness (mean)", "run", "s", "lower",
       r"\overline{\max(0, t^C_j - t^{due}_j)}",
       "Mean tardiness over jobs that HAVE due times. Not displayed when no "
       "due times exist.",
       ["jobs.due_elapsed_ms", "jobs.completion_elapsed_ms"]),
    _d("robot_completion_spread_sec", "Robot completion spread", "run", "s",
       "lower", r"\max_i C_i - \min_i C_i",
       "Spread between the first and last robot completion times.",
       ["run_robots.completion_elapsed_ms"]),
    _d("tail_completion_delay_sec", "Tail completion delay", "run", "s",
       "lower", r"\max_i C_i - \mathrm{median}_i(C_i)",
       "How much the last robot lags the median -- identifies a makespan "
       "controlled by one straggler.",
       ["run_robots.completion_elapsed_ms"]),

    # ---- 8. Utilization and time loss --------------------------------------
    _d("robot_productive_utilization_pct", "Robot productive utilization",
       "robot", "%", "higher", r"U_i = 100\frac{P_i}{T}",
       "Productive mission time (travel empty/loaded + pickup + dropoff "
       "service) as a share of run makespan.",
       ["events.ROBOT_STATE_CHANGED", "actual_makespan_sec"]),
    _d("fleet_productive_utilization_pct", "Fleet productive utilization",
       "run", "%", "higher", r"U_{fleet} = 100\frac{\sum_i P_i}{N T}",
       "Fleet-wide productive share of total robot-time.",
       ["events.ROBOT_STATE_CHANGED", "actual_makespan_sec"]),
    _d("occupied_utilization_pct", "Occupied utilization", "robot", "%",
       "neutral",
       r"O_i = 100\frac{P_i + W^{process}_i + W^{traffic}_i + W^{queue}_i + R_i}{T}",
       "Share of the run during which the robot was committed or "
       "unavailable, even when not productive.",
       ["events.ROBOT_STATE_CHANGED", "actual_makespan_sec"]),
    _d("state_fraction_pct", "State time fraction", "robot", "%", "neutral",
       r"100\frac{X_i}{T}",
       "Share of run time in one mutually-exclusive robot state; the state "
       "name is carried in scope_id as '<robot>:<STATE>'.",
       ["events.ROBOT_STATE_CHANGED"]),
    _d("fleet_traffic_delay_robot_sec", "Fleet traffic delay", "run",
       "robot-s", "lower", r"D_{traffic} = \sum_i W^{traffic}_i",
       "Total robot-seconds spent in TRAFFIC_WAIT. Three robots waiting ten "
       "seconds simultaneously = 30 robot-seconds.",
       ["events.ROBOT_STATE_CHANGED"]),
    _d("traffic_delay_per_job_sec", "Traffic delay per completed job", "run",
       "robot-s/job", "lower", r"\frac{\sum_i W^{traffic}_i}{J_C}",
       "Fleet traffic delay normalized by completed jobs.",
       ["events.ROBOT_STATE_CHANGED", "jobs.status"]),
    _d("processing_hold_per_job_sec", "Processing hold per completed job",
       "run", "robot-s/job", "neutral", r"\frac{\sum_i W^{process}_i}{J_C}",
       "Workstation-processing hold time per completed job.",
       ["events.ROBOT_STATE_CHANGED", "jobs.status"]),
    _d("queue_delay_per_job_sec", "Queue delay per completed job", "run",
       "robot-s/job", "lower", r"\frac{\sum_i W^{queue}_i}{J_C}",
       "Resource-queue wait time per completed job.",
       ["events.ROBOT_STATE_CHANGED", "jobs.status"]),

    # ---- 9. Coordination ---------------------------------------------------
    _d("conflict_count", "Detected conflict episodes", "run", "count",
       "lower", r"N_{conflict}",
       "Distinct conflict episodes (hysteresis/cooldown applied at "
       "detection), never video frames or repeated warnings.",
       ["conflicts"]),
    _d("conflict_rate_per_job", "Conflict rate", "run", "conflicts/job",
       "lower", r"\frac{N_{conflict}}{J_C}", "Conflicts per completed job.",
       ["conflicts", "jobs.status"]),
    _d("auto_conflict_resolution_rate", "Automatic conflict-resolution rate",
       "run", "ratio", "higher",
       r"\frac{N_{resolved\ automatically}}{N_{conflict}}",
       "Conflicts resolved without manual intervention over all conflicts.",
       ["conflicts.status", "conflicts.resolution"]),
    _d("traffic_stop_episodes", "Traffic-stop episodes", "run", "count",
       "lower", r"N_{\text{TRAFFIC\_WAIT entries}}",
       "Transitions INTO TRAFFIC_WAIT, not repeated STOP messages.",
       ["events.ROBOT_STATE_CHANGED"]),
    _d("mean_traffic_stop_duration_sec", "Mean traffic-stop duration", "run",
       "s", "lower",
       r"\frac{\sum_i W^{traffic}_i}{N_{traffic\ stop\ episodes}}",
       "Average duration of one traffic-stop episode.",
       ["events.ROBOT_STATE_CHANGED"]),
    _d("safety_stop_count", "Safety-stop count", "run", "count", "lower",
       r"N_{safety\ stop}",
       "Deliberate collision-avoidance stops (SAFETY_STOP events), counted "
       "separately from faults and operator stops. NOT labeled 'prevented "
       "collisions' -- no counterfactual analysis exists.",
       ["events.SAFETY_STOP"]),
    _d("reroute_count", "Reroute count", "run", "count", "neutral",
       r"N_{reroutes\ issued}", "Reroute plans actually accepted and issued.",
       ["events.REROUTE_APPLIED"]),
    _d("reroute_success_rate", "Reroute success rate", "run", "ratio",
       "higher", r"\frac{N_{reroutes\ completed}}{N_{reroutes\ issued}}",
       "Issued reroutes that completed.",
       ["events.REROUTE_APPLIED", "events.REROUTE_FAILED"]),
    _d("reroute_detour_distance_m", "Reroute detour distance (mean)", "run",
       "m", "lower", r"D^{reroute}_g - D^{original\ remaining}_g",
       "Mean extra distance per reroute vs the original remaining route.",
       ["events.REROUTE_APPLIED.details"]),
    _d("reroute_detour_time_sec", "Reroute detour time (mean)", "run", "s",
       "lower", r"T^{reroute}_g - T^{original\ remaining}_g",
       "Mean extra time per reroute vs the original remaining route.",
       ["events.REROUTE_APPLIED.details"]),
    _d("deadlock_count", "Deadlock episodes", "run", "count", "lower",
       r"N_{deadlock}", "Distinct deadlock episodes.",
       ["events.DEADLOCK_DETECTED"]),
    _d("deadlock_run_rate", "Deadlock run rate", "aggregate", "ratio",
       "lower", r"\frac{N_{runs\ containing\ deadlock}}{R}",
       "Share of runs containing at least one deadlock.",
       ["events.DEADLOCK_DETECTED"]),
    _d("mean_deadlock_recovery_sec", "Mean deadlock-recovery time", "run",
       "s", "lower", r"\mathrm{mean}(t^{resolved}_g - t^{detected}_g)",
       "Mean recovery time over RESOLVED deadlocks; unresolved deadlocks "
       "stay visible as unresolved and are never dropped.",
       ["conflicts.detected_elapsed_ms", "conflicts.resolved_elapsed_ms"]),
    _d("unresolved_deadlock_count", "Unresolved deadlocks", "run", "count",
       "lower", r"N_{deadlock\ unresolved}",
       "Deadlock episodes never resolved during the run.",
       ["conflicts.status"]),

    # ---- 10. Safety --------------------------------------------------------
    _d("collision_count", "Collision count", "run", "count", "lower",
       r"N_{collision}", "Physical robot-robot contacts as distinct events.",
       ["events.COLLISION"]),
    _d("min_fleet_separation_m", "Minimum fleet separation", "run", "m",
       "higher", r"d_{min} = \min_{t, i<\ell} d_{i\ell}(t)",
       "Smallest pairwise robot distance across the run; pose source and "
       "accuracy metadata are stored alongside.",
       ["telemetry_samples"]),
    _d("near_miss_count", "Near-miss episodes", "run", "count", "lower",
       r"N: d_{collision} < d_{i\ell}(t) \le d_{warn}",
       "Distinct near-miss episodes (hysteresis/cooldown prevents "
       "frame-by-frame double counting).",
       ["events.NEAR_MISS_STARTED"]),
    _d("time_below_warning_separation_sec", "Time below warning separation",
       "run", "s", "lower",
       r"\int \mathbf{1}(\min_{i<\ell} d_{i\ell}(t) \le d_{warn})\,dt",
       "Total time the closest pair was inside the warning distance.",
       ["telemetry_samples"]),
    _d("conflict_detection_latency_sec", "Conflict-detection latency (mean)",
       "run", "s", "lower",
       r"t_{detected} - t_{first\ qualifying\ observation}",
       "Latency from first qualifying trajectory observation to detection.",
       ["events.CONFLICT_DETECTED.details"]),
    _d("stop_command_latency_sec", "Stop-command latency (mean)", "run", "s",
       "lower", r"t_{STOP\ issued} - t_{conflict\ detected}",
       "Latency from conflict detection to STOP being issued.",
       ["events.SAFETY_STOP.details"]),
    _d("motion_cessation_latency_sec", "Motion-cessation latency (mean)",
       "run", "s", "lower", r"t_{motion\ stopped} - t_{STOP\ issued}",
       "Latency from STOP issued to physical standstill.",
       ["events.SAFETY_STOP.details", "telemetry_samples"]),
    _d("emergency_stop_count", "Emergency-stop count", "run", "count",
       "lower", r"N_{e\text{-}stop}",
       "Emergency stops, split by type in scope_id (physical / software / "
       "operator / automatic).",
       ["events.SAFETY_STOP", "events.MANUAL_INTERVENTION"]),

    # ---- 11. Routing, distance, capacity -----------------------------------
    _d("total_distance_m", "Total distance traveled", "run", "m", "neutral",
       r"D_{fleet} = \sum_i D_i",
       "Sum of actual per-robot path length integrated from valid "
       "telemetry positions.",
       ["telemetry_samples"]),
    _d("robot_distance_m", "Robot distance traveled", "robot", "m",
       "neutral", r"D_i", "Actual distance traveled by one robot.",
       ["telemetry_samples"]),
    _d("distance_per_job_m", "Distance per completed job", "run", "m/job",
       "lower", r"\frac{\sum_i D_i}{J_C}",
       "Fleet distance normalized by completed jobs.",
       ["telemetry_samples", "jobs.status"]),
    _d("path_efficiency_pct", "Path efficiency", "run", "%", "higher",
       r"\eta_{path} = 100\frac{\sum_i D^*_i}{\sum_i D_i}",
       "Shortest feasible reference distance over actual distance. The "
       "reference performs the same assigned work under the same physical "
       "constraints.",
       ["telemetry_samples", "runs.plan_json"]),
    _d("detour_ratio_pct", "Detour ratio", "run", "%", "lower",
       r"100\frac{\sum_i (D_i - D^*_i)}{\sum_i D^*_i}",
       "Excess distance over the reference, as a share of the reference.",
       ["telemetry_samples", "runs.plan_json"]),
    _d("empty_travel_ratio_pct", "Empty-travel ratio", "run", "%", "lower",
       r"100\frac{\sum_i D^E_i}{\sum_i D_i}",
       "Share of distance traveled empty. Requires loaded/empty state "
       "classification of travel.",
       ["events.ROBOT_STATE_CHANGED", "telemetry_samples"]),
    _d("loaded_travel_ratio_pct", "Loaded-travel ratio", "run", "%",
       "neutral", r"100\frac{\sum_i D^L_i}{\sum_i D_i}",
       "Share of distance traveled loaded.",
       ["events.ROBOT_STATE_CHANGED", "telemetry_samples"]),
    _d("payload_distance_utilization_pct", "Payload-distance utilization",
       "run", "%", "higher", r"100\frac{\sum_g L_g D_g}{\sum_g K_g D_g}",
       "Utilized carrying capacity over distance (segment load x distance "
       "over capacity x distance).",
       ["events.ROBOT_STATE_CHANGED.details.load", "telemetry_samples"]),
    _d("item_distance_efficiency", "Item-distance efficiency", "run",
       "items/robot-m", "higher", r"\frac{\sum_g L_g D_g}{\sum_i D_i}",
       "Average items transported per robot-meter.",
       ["events.ROBOT_STATE_CHANGED.details.load", "telemetry_samples"]),
    _d("route_conformance_pct", "Route-conformance ratio", "run", "%",
       "higher", r"100(1 - \frac{D_{unplanned}}{D_{actual}})",
       "Displayed value bounded to 0-100%; the raw value is retained in the "
       "numerator/denominator fields for debugging.",
       ["telemetry_samples", "runs.plan_json"]),
    _d("correct_service_rate", "Correct workstation-service rate", "run",
       "ratio", "higher",
       r"\frac{N_{correct\ services}}{N_{attempted\ services}}",
       "Wrong workstations and wrong service order count as failures.",
       ["events.PICKUP_COMPLETED", "events.DROPOFF_COMPLETED",
        "events.COMMAND_FAILED"]),

    # ---- 12. Workload balance ----------------------------------------------
    _d("jain_fairness_index", "Jain's fairness index", "run", "index",
       "higher", r"J_f = \frac{(\sum_i x_i)^2}{N \sum_i x_i^2}",
       "Workload-balance index over the selected nonnegative workload "
       "measure x_i (completed jobs by default; selectable). 1 = equal. "
       "Valid only for nonnegative x_i with a nonzero total.",
       ["run_robots", "jobs"]),
    _d("workload_cv", "Workload coefficient of variation", "run", "ratio",
       "lower", r"CV_x = \frac{s(x_i)}{\overline{x}}",
       "NULL if the mean workload is zero.",
       ["run_robots", "jobs"]),
    _d("max_min_workload_ratio", "Max/min workload ratio", "run", "ratio",
       "lower", r"\frac{\max_i x_i}{\min_i x_i}",
       "NULL when the minimum is zero; the zero-workload robot is displayed "
       "explicitly.",
       ["run_robots", "jobs"]),

    # ---- 13. Plan and digital-twin fidelity --------------------------------
    _d("planned_makespan_sec", "Planned makespan", "run", "s", "lower",
       r"T^{plan}",
       "The route solver's makespan() result, stored at run creation. Never "
       "displayed as actual makespan.",
       ["runs.planned_makespan_sec"]),
    _d("makespan_error_sec", "Makespan prediction error", "run", "s",
       "lower", r"T - T^{plan}",
       "Actual minus planned makespan (signed).",
       ["actual_makespan_sec", "runs.planned_makespan_sec"]),
    _d("makespan_error_pct", "Relative makespan prediction error", "run",
       "%", "lower", r"100\frac{T - T^{plan}}{T^{plan}}",
       "Signed relative prediction error.",
       ["actual_makespan_sec", "runs.planned_makespan_sec"]),
    _d("event_timing_mae_sec", "Matched-event timing MAE", "run", "s",
       "lower", r"\frac{1}{H}\sum_h |t^{actual}_h - t^{plan}_h|",
       "Mean absolute timing error over matched planned/actual events; "
       "reported separately per event family in scope_id (depot departure, "
       "workstation arrival, pickup, drop-off, processing completion, depot "
       "return).",
       ["runs.plan_json", "events"]),
    _d("replanning_frequency_per_job", "Replanning frequency", "run",
       "replans/job", "lower", r"\frac{N_{replans}}{J_C}",
       "Reroute/replan events per completed job.",
       ["events.REROUTE_REQUESTED", "jobs.status"]),
    _d("replanning_latency_sec", "Replanning latency (mean)", "run", "s",
       "lower", r"t_{plan\ available} - t_{replan\ requested}",
       "Mean/median/p95/max reported over replan request-to-availability "
       "pairs.",
       ["events.REROUTE_REQUESTED", "events.REROUTE_APPLIED"]),
    _d("advisor_decision_latency_sec", "Advisor decision latency (mean)",
       "run", "s", "lower", r"t_{decision\ returned} - t_{requested}",
       "LLM/advisor decision round-trip latency.",
       ["events with details.advisor_latency_ms"]),
    _d("decision_validity_rate", "Decision-validity rate", "run", "ratio",
       "higher",
       r"\frac{N_{valid\ executable\ decisions}}{N_{decision\ requests}}",
       "Advisor decisions that were valid and executable.",
       ["events with details.advisor"]),
    _d("advisor_fallback_rate", "Advisor fallback rate", "run", "ratio",
       "lower", r"\frac{N_{fallbacks}}{N_{advisor\ requests}}",
       "Advisor requests that fell back to the deterministic path.",
       ["events with details.advisor_fallback"]),
    _d("command_ack_latency_sec", "Command acknowledgement latency (mean)",
       "run", "s", "lower", r"t_{ack} - t_{command\ sent}",
       "COMMAND_SENT to COMMAND_ACKNOWLEDGED latency.",
       ["events.COMMAND_SENT", "events.COMMAND_ACKNOWLEDGED"]),
    _d("command_completion_latency_sec", "Command completion latency (mean)",
       "run", "s", "lower", r"t_{completed} - t_{command\ sent}",
       "COMMAND_SENT to COMMAND_COMPLETED latency.",
       ["events.COMMAND_SENT", "events.COMMAND_COMPLETED"]),

    # ---- 14. Localization, navigation, terminal accuracy -------------------
    _d("vision_availability_pct", "Vision availability", "run", "%",
       "higher", r"100\frac{T_{fresh\ pose\ coverage}}{T}",
       "Share of the run covered by poses no older than the freshness "
       "threshold a_max.",
       ["telemetry_samples.pose_age_ms"]),
    _d("pose_update_rate_hz", "Pose update rate", "run", "Hz", "higher",
       r"\frac{N_{valid\ pose\ updates}}{T}",
       "Valid pose updates per second over the run.",
       ["telemetry_samples"]),
    _d("pose_age_mean_ms", "Pose age (mean)", "run", "ms", "lower",
       r"\overline{a}", "Mean pose age across valid samples.",
       ["telemetry_samples.pose_age_ms"]),
    _d("pose_age_median_ms", "Pose age (median)", "run", "ms", "lower",
       r"\mathrm{median}(a)", "Median pose age.",
       ["telemetry_samples.pose_age_ms"]),
    _d("pose_age_p95_ms", "Pose age (95th pct)", "run", "ms", "lower",
       r"P_{95}(a)", "95th percentile pose age.",
       ["telemetry_samples.pose_age_ms"]),
    _d("pose_age_max_ms", "Pose age (max)", "run", "ms", "lower",
       r"\max(a)", "Maximum pose age.",
       ["telemetry_samples.pose_age_ms"]),
    _d("vision_dropout_episodes", "Vision-dropout episodes", "run", "count",
       "lower", r"N: a > a_{max}",
       "Continuous intervals with pose age above a_max; count plus "
       "mean/max/total duration reported.",
       ["telemetry_samples.pose_age_ms"]),
    _d("path_tracking_error_mean_m", "Path-tracking error (mean)", "run",
       "m", "lower", r"\overline{e_{path}},\; e_{path}(t)=\|p(t)-p^*(t)\|",
       "Mean distance between measured position and the closest point on "
       "the planned path (RMS/p95/max reported alongside).",
       ["telemetry_samples", "runs.plan_json"]),
    _d("depot_position_error_m", "Depot position error", "robot", "m",
       "lower", r"e_{depot,i} = \|p_i^{final} - p_i^*\|",
       "Final measured depot position vs the required position.",
       ["run_robots.terminal_x_m", "run_robots.terminal_y_m"]),
    _d("terminal_heading_error_deg", "Terminal heading error", "robot",
       "deg", "lower",
       r"e_{\psi,i} = |\mathrm{wrap}(\psi_i^{final} - \psi^*)|",
       "Final heading error vs the required depot heading (wrap to "
       "[-180,180)).",
       ["run_robots.terminal_yaw_deg"]),
    _d("terminal_condition_success_rate", "Terminal-condition success rate",
       "run", "ratio", "higher",
       r"\frac{N_{robots\ in\ position+heading\ tolerance}}{N}",
       "Robots satisfying both terminal position and heading tolerances.",
       ["run_robots"]),
    _d("docking_success_rate", "Docking success rate", "run", "ratio",
       "higher", r"\frac{N_{successful\ docking}}{N_{docking\ attempts}}",
       "Successful docking attempts over all attempts.",
       ["events.DEPOT_ARRIVED", "events.COMMAND_FAILED"]),

    # ---- 15. Rotation control ----------------------------------------------
    _d("yaw_mean_signed_error_deg", "Mean signed yaw error", "run", "deg",
       "lower", r"\frac{1}{n_V}\sum_q e_q",
       "Systematic overshoot/undershoot across rotations with a valid final "
       "measurement.",
       ["events.ROTATION_ATTEMPT_COMPLETED.details.error_deg"]),
    _d("yaw_mae_deg", "Mean absolute yaw error", "run", "deg", "lower",
       r"MAE = \frac{1}{n_V}\sum_q |e_q|",
       "Mean absolute rotation error over valid completions; n shown.",
       ["events.ROTATION_ATTEMPT_COMPLETED.details.error_deg"]),
    _d("yaw_rmse_deg", "Yaw RMSE", "run", "deg", "lower",
       r"RMSE = \sqrt{\frac{1}{n_V}\sum_q e_q^2}",
       "Root-mean-square rotation error over valid completions.",
       ["events.ROTATION_ATTEMPT_COMPLETED.details.error_deg"]),
    _d("yaw_error_std_deg", "Yaw-error std dev (within-target groups)",
       "run", "deg", "lower", r"s(e_q)\ \text{within commanded-angle groups}",
       "Standard deviation computed within commanded-angle groups so target "
       "variation is not mistaken for poor repeatability (pooled).",
       ["events.ROTATION_ATTEMPT_COMPLETED.details"]),
    _d("yaw_abs_error_median_deg", "Yaw |error| (median)", "run", "deg",
       "lower", r"\mathrm{median}(|e_q|)", "Median absolute rotation error.",
       ["events.ROTATION_ATTEMPT_COMPLETED.details.error_deg"]),
    _d("yaw_abs_error_p95_deg", "Yaw |error| (95th pct)", "run", "deg",
       "lower", r"P_{95}(|e_q|)", "95th percentile absolute rotation error.",
       ["events.ROTATION_ATTEMPT_COMPLETED.details.error_deg"]),
    _d("yaw_abs_error_max_deg", "Yaw |error| (max)", "run", "deg", "lower",
       r"\max |e_q|", "Maximum absolute rotation error.",
       ["events.ROTATION_ATTEMPT_COMPLETED.details.error_deg"]),
    _d("rotation_completion_rate", "Rotation completion rate", "run",
       "ratio", "higher", r"\frac{n_V}{n_A}",
       "Rotations with a valid final measurement over all attempts. Failed "
       "attempts stay in the denominator with final error NULL, not zero.",
       ["events.ROTATION_ATTEMPT_STARTED", "events.ROTATION_ATTEMPT_COMPLETED",
        "events.ROTATION_ATTEMPT_FAILED"]),
    _d("rotation_within_tolerance_rate", "Within-tolerance rotation rate",
       "run", "ratio", "higher", r"\frac{n_\tau}{n_A}",
       "Attempts completing within the yaw tolerance over ALL attempts; "
       "failures count as not-within-tolerance.",
       ["events.ROTATION_ATTEMPT_COMPLETED.details.within_tolerance"]),
    _d("rotation_mean_duration_sec", "Mean completed-rotation duration",
       "run", "s", "lower",
       r"\frac{1}{n_V}\sum_{q \in valid} t_q",
       "Mean elapsed time of valid rotation attempts.",
       ["events.ROTATION_ATTEMPT_COMPLETED.details.duration_sec"]),
    _d("acceptable_turn_throughput_per_min", "Acceptable-turn throughput",
       "run", "rotations/min", "higher",
       r"60\frac{n_\tau}{\sum_{q=1}^{n_A} t_q}",
       "Within-tolerance rotations per minute of total rotation time, "
       "INCLUDING time consumed by failed attempts.",
       ["events.ROTATION_ATTEMPT_*"]),
    _d("rotation_time_error_product", "Time-error product (TEP)", "run",
       "deg-s", "lower", r"TEP = MAE \times \overline{t}_{valid}",
       "Secondary composite metric; error and duration are always reported "
       "separately as well.",
       ["yaw_mae_deg", "rotation_mean_duration_sec"]),
    _d("camera_correction_rate", "Camera-correction rate", "run", "ratio",
       "lower",
       r"\frac{N_{corrective\ rotations}}{N_{camera\text{-}assist\ eligible}}",
       "Share of camera-assist-eligible rotations that needed a corrective "
       "rotation.",
       ["events.ROTATION_ATTEMPT_COMPLETED.details.corrected"]),
    _d("rotation_failure_rate", "Rotation failure rate", "run", "ratio",
       "lower", r"\frac{n_A - n_V}{n_A}",
       "Attempts without a valid final measurement over all attempts. One "
       "failure out of 256 never discards the other 255.",
       ["events.ROTATION_ATTEMPT_*"]),

    # ---- 16. Battery and energy --------------------------------------------
    _d("battery_drop_pct", "Battery-charge decrease", "robot", "pct-points",
       "lower", r"\Delta B_i = B^0_i - B^1_i",
       "Start minus end charge. Negative values (estimate rebound) are NOT "
       "forced to zero -- they stay visible and auditable.",
       ["run_robots.start_battery_pct", "run_robots.end_battery_pct"]),
    _d("battery_drain_rate_pct_per_min", "Battery-drain rate", "robot",
       "pct-points/min", "lower", r"\frac{\Delta B_i}{T/60}",
       "Charge decrease per minute of run time.",
       ["battery_drop_pct", "actual_makespan_sec"]),
    _d("battery_drain_per_job_pct", "Battery drain per completed job", "run",
       "pct-points/job", "lower", r"\frac{\sum_i \Delta B_i}{J_C}",
       "Fleet charge decrease per completed job.",
       ["battery_drop_pct", "jobs.status"]),
    _d("battery_drain_per_meter_pct", "Battery drain per meter", "run",
       "pct-points/m", "lower", r"\frac{\sum_i \Delta B_i}{\sum_i D_i}",
       "Fleet charge decrease per meter traveled.",
       ["battery_drop_pct", "telemetry_samples"]),
    _d("battery_drain_per_productive_min_pct",
       "Battery drain per productive minute", "robot", "pct-points/min",
       "lower", r"\frac{\Delta B_i}{P_i/60}",
       "Charge decrease per minute of productive time.",
       ["battery_drop_pct", "events.ROBOT_STATE_CHANGED"]),
    _d("energy_wh", "Electrical energy", "robot", "Wh", "lower",
       r"E_i = \frac{1}{3600}\sum_k V_{ik} I^{elec}_{ik} \Delta t_k",
       "Integrated electrical energy when voltage and current telemetry "
       "exist. (I^elec is battery current -- never confused with idle time.)",
       ["telemetry_samples.voltage_v", "telemetry_samples.current_a"]),
    _d("energy_per_job_wh", "Energy per completed job", "run", "Wh/job",
       "lower", r"\frac{\sum_i E_i}{J_C}", "Fleet energy per completed job.",
       ["energy_wh", "jobs.status"]),
    _d("energy_per_meter_wh", "Energy per meter", "run", "Wh/m", "lower",
       r"\frac{\sum_i E_i}{\sum_i D_i}", "Fleet energy per meter traveled.",
       ["energy_wh", "telemetry_samples"]),
    _d("battery_drain_imbalance_cv", "Battery-drain imbalance (CV)", "run",
       "ratio", "lower", r"CV(\Delta B_i / D_i)",
       "Coefficient of variation of normalized drain (per meter) across "
       "robots -- raw charge decrease alone is never compared when robots "
       "performed different workloads.",
       ["battery_drop_pct", "telemetry_samples"]),

    # ---- 17. Reliability and recovery --------------------------------------
    _d("command_success_rate", "Command success rate", "run", "ratio",
       "higher", r"\frac{Q_C}{Q_A}",
       "Completed commands over attempted commands. Timeouts, rejections, "
       "emergency stops, and aborts all stay in the denominator.",
       ["events.COMMAND_SENT", "events.COMMAND_COMPLETED",
        "events.COMMAND_FAILED"]),
    _d("robot_fault_count", "Robot fault episodes", "run", "count", "lower",
       r"N_{fault}",
       "Distinct fault episodes, not repeated status messages.",
       ["events.COMMAND_FAILED", "events.ROBOT_OFFLINE"]),
    _d("mean_recovery_time_sec", "Mean recovery time", "run", "s", "lower",
       r"\mathrm{mean}(t_{recovered} - t_{fault})",
       "Over resolved faults; unresolved faults are retained as censored "
       "observations, reported separately.",
       ["events.ROBOT_OFFLINE", "events.ROBOT_RECOVERED"]),
    _d("manual_intervention_count", "Manual interventions", "run", "count",
       "lower", r"N_{manual}",
       "Operator commands, physical repositioning, resets, or other manual "
       "assistance.",
       ["events.MANUAL_INTERVENTION"]),
    _d("robot_online_availability_pct", "Robot online availability", "robot",
       "%", "higher", r"100\frac{T - F_i - T^{offline}_i}{T}",
       "Share of the run the robot was neither faulted nor offline (the two "
       "never overlap by construction of the state model).",
       ["events.ROBOT_STATE_CHANGED"]),

    # ---- 18. Workstations --------------------------------------------------
    _d("workstation_processing_utilization_pct",
       "Workstation processing utilization", "workstation", "%", "higher",
       r"100\frac{T^{processing}_s}{T}",
       "Share of the run the workstation spent processing.",
       ["events.PROCESSING_STARTED", "events.PROCESSING_COMPLETED"]),
    _d("workstation_starvation_pct", "Workstation starvation ratio",
       "workstation", "%", "lower", r"100\frac{T^{starved}_s}{T}",
       "Share of the run the workstation waited for a delivery.",
       ["events", "jobs"]),
    _d("workstation_blocked_pct", "Workstation blocked ratio", "workstation",
       "%", "lower", r"100\frac{T^{blocked}_s}{T}",
       "Share of the run the workstation was blocked (finished part waiting "
       "for pickup).",
       ["events", "jobs"]),
    _d("mean_workstation_queue_sec", "Mean workstation queue time",
       "workstation", "s", "lower",
       r"\mathrm{mean}(t_{service\ start} - t_{queue\ entry})",
       "Mean robot wait between queue entry and service start.",
       ["events.ROBOT_STATE_CHANGED"]),
    _d("excess_workstation_dwell_sec", "Excess workstation dwell",
       "workstation", "s", "lower",
       r"T^{observed\ dwell} - T^{prescribed\ process}",
       "Observed minus prescribed processing time. Negative values remain "
       "visible as possible timing/data-quality issues.",
       ["events.PROCESSING_*", "runs.process_sec"]),

    # ---- 19. Data quality --------------------------------------------------
    _d("telemetry_coverage_pct", "Telemetry coverage", "run", "%", "higher",
       r"100\frac{T_{covered\ by\ valid\ telemetry}}{T}",
       "Share of the run covered by valid telemetry.",
       ["telemetry_samples"]),
    _d("event_sequence_issues", "Event-sequence issues", "run", "count",
       "lower", r"N_{issues}",
       "Missing seq numbers + duplicates + nonmonotonic elapsed timestamps "
       "+ end-without-start + overlapping exclusive states + completion "
       "anomalies + invalid pose values. Itemized in the Data Quality tab.",
       ["events"]),
    _d("state_time_reconciliation_error_sec", "State-time reconciliation error",
       "robot", "s", "lower",
       r"|T - (W^{launch}_i + P_i + W^{process}_i + W^{traffic}_i + "
       r"W^{queue}_i + I_i + F_i + R_i + H_i)|",
       "Per-robot gap between the run duration and the sum of "
       "mutually-exclusive state durations; flagged as a data-quality error "
       "above the configured tolerance.",
       ["events.ROBOT_STATE_CHANGED"]),

    # ---- 20. Aggregate scalability -----------------------------------------
    _d("throughput_speedup", "Throughput speedup", "aggregate", "ratio",
       "higher", r"S_N = \frac{Q_N}{Q_{reference}}",
       "Throughput at fleet size N over the reference fleet size, same "
       "workload and conditions only.",
       ["throughput_jobs_per_min"]),
    _d("scaling_efficiency", "Scaling efficiency", "aggregate", "ratio",
       "higher", r"E_N = \frac{S_N}{N / N_{reference}}",
       "Speedup normalized by fleet-size ratio.",
       ["throughput_speedup"]),
    _d("marginal_robot_benefit", "Marginal robot benefit", "aggregate",
       "jobs/min", "higher", r"Q_N - Q_{N-1}",
       "Throughput gained by the Nth robot, same workload and conditions "
       "only.",
       ["throughput_jobs_per_min"]),
]

# ---------------------------------------------------------------------------
# Movement breakdown (added 2026-09-01)
#
# Every dispatched plan item already arrives as COMMAND_SENT with
# details.kind in {move, turn, dwell} and closes as COMMAND_COMPLETED with
# the same command_id, so a per-robot per-kind timing breakdown needs no new
# instrumentation -- only that the engine stop collapsing all commands into
# one command_completion_latency_sec average.
#
# The point is comparison BETWEEN robots. fleetSupervisor's own timing
# calibration already shows Alvik4 taking ~1.48x its predicted move duration
# against Alvik1's ~1.14x, and nothing in the metrics captured that. These
# make it a measured quantity: per-robot means, and a run-scope spread that
# says how unlike each other the robots are.
MOVEMENT_DEFINITIONS = [
    {"metric_key": "movement_count", "version": 1,
     "display_name": "Movements executed", "scope": "robot",
     "description": "Completed plan items for this robot, split by kind "
                    "(scope_id is '<robot>:<kind>' for the per-kind rows and "
                    "'<robot>' for the total). Counts only commands that "
                    "actually completed; a failed or aborted command stays in "
                    "the denominator of movement_success_rate instead.",
     "formula_latex": "n_{r,k}", "unit": "count", "direction": "neutral",
     "required_fields": ["events.COMMAND_SENT", "events.COMMAND_COMPLETED"],
     "null_policy": "null-when-missing; no zero substitution",
     "effective_at_utc": "2026-09-01"},

    {"metric_key": "movement_duration_mean_sec", "version": 1,
     "display_name": "Mean movement duration", "scope": "robot",
     "description": "Mean seconds from COMMAND_SENT to COMMAND_COMPLETED for "
                    "this robot, split by kind. This is wall-clock execution "
                    "time including any settle and verification, not the "
                    "planner's predicted duration.",
     "formula_latex": "\\bar{d}_{r,k} = \\frac{1}{n}\\sum (t_{done} - t_{sent})",
     "unit": "s", "direction": "lower",
     "required_fields": ["events.COMMAND_SENT", "events.COMMAND_COMPLETED"],
     "null_policy": "null-when-missing; incomplete commands excluded from the "
                    "mean but reported in missing_n",
     "effective_at_utc": "2026-09-01"},

    {"metric_key": "movement_duration_median_sec", "version": 1,
     "display_name": "Median movement duration", "scope": "robot",
     "description": "Median seconds per completed movement for this robot, by "
                    "kind. Reported alongside the mean because a single "
                    "stalled leg skews the mean badly on short runs.",
     "formula_latex": "\\tilde{d}_{r,k}", "unit": "s", "direction": "lower",
     "required_fields": ["events.COMMAND_SENT", "events.COMMAND_COMPLETED"],
     "null_policy": "null-when-missing; no zero substitution",
     "effective_at_utc": "2026-09-01"},

    {"metric_key": "movement_duration_std_sec", "version": 1,
     "display_name": "Movement duration spread (this robot)", "scope": "robot",
     "description": "Population standard deviation of this robot's completed "
                    "movement durations, by kind. High spread on one kind is "
                    "how an intermittent fault shows up before it becomes a "
                    "failure.",
     "formula_latex": "\\sigma_{r,k}", "unit": "s", "direction": "lower",
     "required_fields": ["events.COMMAND_SENT", "events.COMMAND_COMPLETED"],
     "null_policy": "null-when-missing; needs at least 2 completed movements",
     "effective_at_utc": "2026-09-01"},

    {"metric_key": "movement_success_rate", "version": 1,
     "display_name": "Movement success rate", "scope": "robot",
     "description": "Completed / dispatched commands for this robot, by kind. "
                    "Failed and never-completed commands stay in the "
                    "denominator -- a robot that aborts half its turns must "
                    "not score 100%.",
     "formula_latex": "n_{done} / n_{sent}", "unit": "ratio",
     "direction": "higher",
     "required_fields": ["events.COMMAND_SENT", "events.COMMAND_COMPLETED",
                         "events.COMMAND_FAILED"],
     "null_policy": "null-when-missing; failed attempts stay in denominators",
     "effective_at_utc": "2026-09-01"},

    {"metric_key": "fleet_movement_duration_mean_sec", "version": 1,
     "display_name": "Fleet mean movement duration", "scope": "run",
     "description": "Mean movement duration across all robots for one kind "
                    "(scope_id is the kind). The fleet baseline each robot is "
                    "compared against.",
     "formula_latex": "\\bar{d}_k", "unit": "s", "direction": "lower",
     "required_fields": ["events.COMMAND_SENT", "events.COMMAND_COMPLETED"],
     "null_policy": "null-when-missing; no zero substitution",
     "effective_at_utc": "2026-09-01"},

    {"metric_key": "movement_duration_delta_sec", "version": 1,
     "display_name": "Movement duration vs fleet mean", "scope": "robot",
     "description": "This robot's mean movement duration for a kind minus the "
                    "fleet mean for that kind. Positive means slower than its "
                    "peers. This is the per-robot difference the breakdown "
                    "exists to expose -- a robot consistently +0.4s on turns "
                    "is a mechanical or calibration difference, not noise.",
     "formula_latex": "\\bar{d}_{r,k} - \\bar{d}_k", "unit": "s",
     "direction": "lower",
     "required_fields": ["events.COMMAND_SENT", "events.COMMAND_COMPLETED"],
     "null_policy": "null-when-missing; needs >= 2 robots with data for the "
                    "kind, otherwise there is no fleet to differ from",
     "effective_at_utc": "2026-09-01"},

    {"metric_key": "movement_duration_robot_spread_sec", "version": 1,
     "display_name": "Between-robot spread", "scope": "run",
     "description": "Max minus min of the per-robot mean movement durations "
                    "for one kind (scope_id is the kind). Answers 'how unlike "
                    "each other are these robots' in one number; near zero "
                    "means the fleet is homogeneous for that movement.",
     "formula_latex": "\\max_r \\bar{d}_{r,k} - \\min_r \\bar{d}_{r,k}",
     "unit": "s", "direction": "lower",
     "required_fields": ["events.COMMAND_SENT", "events.COMMAND_COMPLETED"],
     "null_policy": "null-when-missing; needs >= 2 robots with data for the kind",
     "effective_at_utc": "2026-09-01"},
]

DEFINITIONS.extend(MOVEMENT_DEFINITIONS)

DEFINITIONS_BY_KEY = {d["metric_key"]: d for d in DEFINITIONS}

assert len(DEFINITIONS_BY_KEY) == len(DEFINITIONS), "duplicate metric_key"


def seed_definitions(conn) -> int:
    """Insert any registry rows not already in metric_definitions. Existing
    (metric_key, version) rows are never modified."""
    count = 0
    for d in DEFINITIONS:
        cur = conn.execute(
            """INSERT OR IGNORE INTO metric_definitions
                   (metric_key, version, display_name, scope, description,
                    formula_latex, unit, direction, required_fields_json,
                    null_policy, effective_at_utc)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (d["metric_key"], d["version"], d["display_name"], d["scope"],
             d["description"], d["formula_latex"], d["unit"], d["direction"],
             json.dumps(d["required_fields"]), d["null_policy"],
             d["effective_at_utc"]))
        count += cur.rowcount
    conn.commit()
    return count
