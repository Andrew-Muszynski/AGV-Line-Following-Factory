import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _vision_execute_method() -> ast.FunctionDef:
    tree = ast.parse((ROOT / "fleet" / "fleetSupervisor.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "VisionLegWorker":
            for child in node.body:
                if isinstance(child, ast.FunctionDef) and child.name == "_execute":
                    return child
    raise AssertionError("VisionLegWorker._execute not found")


def test_live_supervisor_uses_fast_rotate_rel_at_all_turn_sites():
    method = _vision_execute_method()
    called_methods = [
        node.func.attr
        for node in ast.walk(method)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    ]

    # Departure-after-dwell, arrival square-up, and turn-toward-next-leg.
    assert called_methods.count("turn_to_heading_rotate_rel") == 3
    assert "turn_to_heading" not in called_methods


def test_live_supervisor_accepts_four_degree_turn_tolerance():
    source = (ROOT / "fleet" / "fleetSupervisor.py").read_text()
    assert "turn_tol_deg=4.0" in source


def test_firmware_brakes_for_camera_verification_instead_of_preemptive_error():
    source = (ROOT / "AGV_Factory_camera_correction" /
              "AGV_Factory_camera_correction.ino").read_text()
    assert 'publish_status("ROTATE_REL BRAKED_UNCONFIRMED")' in source
    assert 'publish_status("ERROR ROTATE_REL_TARGET_TIMEOUT")' not in source


def test_firmware_services_queued_wheel_commands_before_watchdog():
    source = (ROOT / "AGV_Factory_camera_correction" /
              "AGV_Factory_camera_correction.ino").read_text()
    loop_source = source[source.index("void loop() {"):]

    final_spin = "rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));"
    assert final_spin in loop_source
    assert loop_source.index(final_spin) < loop_source.index(
        "update_state_machine();")
    assert "ERROR WHEEL_CMD_TIMEOUT gap=%lums" in source
    assert "const unsigned long WHEEL_CMD_BRAKE_MS = 300;" in source
    assert "const unsigned long WHEEL_CMD_ABORT_MS = 1000;" in source
    assert "WHEEL_CMD_STALL_RECOVERED gap=%lums" in source
