@echo off
REM ===========================================================================
REM  Double-click to bring up the whole testbed as TABS in one Windows
REM  Terminal window.
REM
REM  STACK=linux (default, and what the evidence supports)
REM      camera bridge    Windows   camera -> TCP 8765
REM      metrics app      Windows   Flask + SQLite on 8000
REM      vision           Windows   apriltag_localize -> rosbridge
REM      microROS agent   LINUX     via ssh, UDP 8888, robots connect here
REM      rosbridge        LINUX     via ssh, TCP 9090
REM      supervisor       LINUX     via ssh, fleetSupervisor --listen
REM      ros shell        LINUX     via ssh, sourced
REM
REM  STACK=wsl -- the everything-on-this-machine arrangement, kept so the two
REM  can still be compared.
REM
REM  WHY THE AGENT IS ON THE LINUX LAPTOP (2026-09-29). ERROR
REM  WHEEL_CMD_TIMEOUT gap=1001ms was root-caused to the micro-ROS agent /
REM  XRCE-DDS layer by eliminating everything else with measurements: relay
REM  bandwidth (moved 16MB/s off the WSL2 NIC, no change), vision CPU (11-25
REM  percent across the whole --threads sweep), navigator stalls (596 turns,
REM  worst control-loop gap 85ms, none over 300ms), and the WiFi link (ping
REM  monitor during a live run: zero loss within +/-3s of the abort, both
REM  robots answering at 6.6-8.5ms). See memory: wheel_cmd_timeout_rootcause.
REM
REM  THE SUPERVISOR GOES WHEREVER THE AGENT GOES. The agent publishes
REM  Alvik*_pose / Alvik*_status as DDS topics and fleetSupervisor subscribes
REM  to them. Splitting them across the WSL2 boundary is the "Alvik1_pose
REM  subscription never matched a publisher" failure. There is deliberately
REM  no toggle that puts them on different machines.
REM
REM  VISION STAYS ON WINDOWS either way -- it cut frame-read median 32
REM  percent (6.9 -> 4.7ms) and is proven not to be the stall mechanism.
REM
REM  BEFORE THE FIRST STACK=linux RUN:
REM    1. bash tools/deploy_linux.sh     (the Linux copy is weeks stale)
REM    2. flash the robots with v2       (AGENT_IP=192.168.0.212)
REM    3. ssh-copy-id romeshprasad@192.168.0.212   (or type the password 4x)
REM
REM  WHY `wt -w agv` PER LINE rather than one `wt` with `;` separators:
REM  Windows Terminal treats `;` as its own tab separator, and cmd mangles it
REM  again, so a single chained command is fragile to quote. Naming the window
REM  with -w means the first call creates it and every later call adds a tab to
REM  that same window -- no semicolons anywhere.
REM
REM  Ordering is enforced by the scripts themselves, not by sleeps here:
REM  vision waits for the camera bridge AND rosbridge, supervisor waits for
REM  rosbridge.
REM ===========================================================================
setlocal

set "REPO=C:\Users\muszy\VRP"
set "WSLREPO=/mnt/c/Users/muszy/VRP"

REM If this is wrong, every WSL tab will fail. Check with:  wsl -l -v
set "DISTRO=Ubuntu-24.04"

REM Window name: all tabs land in this one window.
set "WIN=agv"

REM --- WHERE THE ROS SIDE RUNS -------------------------------------------
REM   linux = agent + rosbridge + supervisor on the Linux laptop, reached
REM           over ssh so their output still lands in tabs here.
REM           Robots must be flashed with v2 (AGENT_IP=192.168.0.212).
REM   wsl   = all three inside WSL2 on this machine.
REM           Robots must be flashed with v3 (AGENT_IP=192.168.0.162).
REM CHANGING THIS MEANS RE-FLASHING THE ROBOTS. The two sketches differ by
REM exactly one line, so a diff of the two folders shows only AGENT_IP.
set "STACK=linux"

set "LINUXUSER=romeshprasad"
set "LINUXIP=192.168.0.212"

REM --- load shedding ------------------------------------------------------
REM These were the reduced-load configuration from when CPU contention was
REM still a live hypothesis. It is now ELIMINATED (11-25 percent utilisation
REM measured across the whole sweep), so VTHREADS=8 is no longer justified as
REM load shedding -- the measured sweep on this CPU says 16 is both faster
REM and more stable. Raise it once the agent move is confirmed good, one
REM change at a time.
set "NOLLM=1"
set "VTHREADS=8"

where wt.exe >nul 2>&1
if errorlevel 1 (
  echo Windows Terminal ^(wt.exe^) not found.
  echo Install it from the Microsoft Store, or run the roles by hand.
  pause
  exit /b 1
)

if /I "%STACK%"=="linux" goto :stack_linux
if /I "%STACK%"=="wsl"   goto :stack_wsl
echo STACK must be "linux" or "wsl", got "%STACK%".
pause
exit /b 1

REM ===========================================================================
:stack_linux
set "ROSHOST=%LINUXIP%"
echo Starting testbed as tabs in Windows Terminal window "%WIN%"...
echo   ROS side:    %LINUXUSER%@%LINUXIP%  (agent, rosbridge, supervisor)
echo   Windows:     camera bridge, vision, metrics app
echo   robots:      must be flashed with v2  ^(AGENT_IP=%LINUXIP%^)
echo   LLM advisor: NOLLM=%NOLLM%   vision threads: %VTHREADS%
echo.

REM Windows-native first: the camera bridge LISTENS and vision connects to it.
wt.exe -w %WIN% new-tab --title "camera bridge" -d "%REPO%" cmd /k python camera_bridge_windows.py --camera 1 --backend msmf --port 8765 --jpeg-quality 95
timeout /t 1 /nobreak >nul

wt.exe -w %WIN% new-tab --title "metrics app" -d "%REPO%" cmd /k python app.py
timeout /t 1 /nobreak >nul

REM ssh -t forces a TTY so Ctrl+C reaches the remote process instead of just
REM killing ssh and orphaning it. No quotes around the remote command: ssh
REM joins the trailing args itself, which sidesteps the cmd/wt/ssh/bash
REM quoting pile-up entirely.
wt.exe -w %WIN% new-tab --title "microROS agent" ssh -t %LINUXUSER%@%LINUXIP% bash run_role_linux.sh agent
timeout /t 3 /nobreak >nul

wt.exe -w %WIN% new-tab --title "rosbridge" ssh -t %LINUXUSER%@%LINUXIP% bash run_role_linux.sh rosbridge
timeout /t 2 /nobreak >nul

REM Vision runs HERE but publishes to the Linux rosbridge, so it needs that
REM machine's LAN IP, not 127.0.0.1.
wt.exe -w %WIN% new-tab --title "vision" -d "%REPO%" cmd /k tools\run_vision_windows.bat %VTHREADS% %ROSHOST%
timeout /t 1 /nobreak >nul

wt.exe -w %WIN% new-tab --title "supervisor" ssh -t %LINUXUSER%@%LINUXIP% bash run_role_linux.sh supervisor
timeout /t 1 /nobreak >nul

wt.exe -w %WIN% new-tab --title "ros shell" ssh -t %LINUXUSER%@%LINUXIP% bash run_role_linux.sh shell
goto :launched

REM ===========================================================================
:stack_wsl
set "ROSHOST=127.0.0.1"
echo Starting testbed as tabs in Windows Terminal window "%WIN%"...
echo   ROS side:    WSL2 %DISTRO%  (agent, rosbridge, supervisor)
echo   robots:      must be flashed with v3  ^(AGENT_IP=192.168.0.162^)
echo   LLM advisor: NOLLM=%NOLLM%   vision threads: %VTHREADS%
echo.

wt.exe -w %WIN% new-tab --title "camera bridge" -d "%REPO%" cmd /k python camera_bridge_windows.py --camera 1 --backend msmf --port 8765 --jpeg-quality 95
timeout /t 1 /nobreak >nul

wt.exe -w %WIN% new-tab --title "metrics app" -d "%REPO%" cmd /k python app.py
timeout /t 1 /nobreak >nul

wt.exe -w %WIN% new-tab --title "microROS agent" wsl.exe -d %DISTRO% bash -lc "cd %WSLREPO% && bash tools/run_role.sh agent"
timeout /t 3 /nobreak >nul

wt.exe -w %WIN% new-tab --title "rosbridge" wsl.exe -d %DISTRO% bash -lc "cd %WSLREPO% && bash tools/run_role.sh rosbridge"
timeout /t 2 /nobreak >nul

wt.exe -w %WIN% new-tab --title "vision" -d "%REPO%" cmd /k tools\run_vision_windows.bat %VTHREADS% %ROSHOST%
timeout /t 1 /nobreak >nul

wt.exe -w %WIN% new-tab --title "supervisor" wsl.exe -d %DISTRO% bash -lc "cd %WSLREPO% && AGV_NO_LLM=%NOLLM% bash tools/run_role.sh supervisor"
timeout /t 1 /nobreak >nul

wt.exe -w %WIN% new-tab --title "ros shell" wsl.exe -d %DISTRO% bash -lc "cd %WSLREPO% && bash tools/run_role.sh shell"
goto :launched

REM ===========================================================================
:launched
echo All tabs launched into window "%WIN%".
echo.
echo   Dashboard:  http://127.0.0.1:8000/?rosbridge=%ROSHOST%
echo   Stop:       STOP_TESTBED.bat
echo.
timeout /t 4 /nobreak >nul
