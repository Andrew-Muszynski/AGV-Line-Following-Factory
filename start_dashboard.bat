@echo off
rem One-click startup for the Windows side of the testbed:
rem   - plan server   (vrp_rpd algorithms for the Route-mode dropdown)
rem   - vision        (AprilTag localization + rosbridge publish + Live-mode
rem                    camera stream; window shows an error if no camera)
rem   - the dashboard itself in the default browser
rem The Linux laptop side (micro-ROS agent, rosbridge, and dispatcher) is
rem started separately over SSH.
cd /d "%~dp0"
rem BRKGA runs real GPU evolution (validated on the RTX 5060, ~55s/solve);
rem the other dropdown algorithms ignore this flag.
start "plan server (close after session)" cmd /k python plan_server.py --brkga num_gpus=1,total_generations=300
start "vision + camera stream (close after session)" cmd /k python apriltag_localize.py --rosbridge --stream
start "" "%~dp0agv_grid_workstation_solver.html"
