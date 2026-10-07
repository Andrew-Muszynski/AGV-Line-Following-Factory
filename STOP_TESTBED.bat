@echo off
REM ===========================================================================
REM  Stop everything START_TESTBED.bat started.
REM
REM  Kills the WSL2 processes by name, then closes the Windows-side ones.
REM  Leaves WSL itself running -- use 'wsl --shutdown' if you want it gone
REM  entirely, but note that also drops the mirrored network interface.
REM ===========================================================================
setlocal
set "DISTRO=Ubuntu-24.04"

REM Must match STACK in START_TESTBED.bat.
set "STACK=linux"
set "LINUXUSER=romeshprasad"
set "LINUXIP=192.168.0.212"

if /I not "%STACK%"=="linux" goto :skip_linux
echo Stopping processes on the Linux laptop...
ssh -o ConnectTimeout=8 %LINUXUSER%@%LINUXIP% "pkill -f micro_ros_agent; pkill -f rosbridge_websocket; pkill -f fleetSupervisor; pkill -f camera_grid_navigate; source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; ros2 daemon stop >/dev/null 2>&1; echo done"
REM camera_grid_navigate is included on purpose: the supervisor spawns
REM one per robot, and killing only the supervisor leaves them driving.
:skip_linux

echo Stopping WSL2 processes...
wsl.exe -d %DISTRO% bash -lc "pkill -f micro_ros_agent; pkill -f rosbridge_websocket; pkill -f apriltag_localize; pkill -f fleetSupervisor; source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; ros2 daemon stop >/dev/null 2>&1; echo done"

echo Stopping Windows processes...
REM Only the two python processes this launcher started listen on these ports;
REM taskkill by window title avoids killing unrelated python you may be running.
taskkill /FI "WINDOWTITLE eq camera bridge*" /T /F >nul 2>&1
taskkill /FI "WINDOWTITLE eq metrics app*"   /T /F >nul 2>&1
REM Only matches when VISIONHOST=windows; harmless otherwise, and the
REM WSL2 pkill above already covers the in-VM case.
taskkill /FI "WINDOWTITLE eq vision*"       /T /F >nul 2>&1

echo.
echo Stopped. Terminal windows may remain open -- close them, or leave them
echo to read the last output.
echo.
pause
