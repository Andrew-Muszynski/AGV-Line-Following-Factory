@echo off
REM ===========================================================================
REM  apriltag_localize.py, WINDOWS-NATIVE instead of inside WSL2.
REM
REM  WHY THIS EXISTS (2026-09-29). The WHEEL_CMD_TIMEOUT gap=1001ms stalls
REM  appeared only after the micro-ROS agent moved into WSL2 on this machine.
REM  The first hypothesis was CPU contention -- but the --threads sweep
REM  recorded in apriltag_localize.py measured 11-25 percent CPU across the
REM  entire range on this 24-logical-processor chip, which starves nothing.
REM
REM  The remaining shared resource is the WSL2 virtual network path. Running
REM  vision inside WSL2 pushes ~16 MB/s of JPEG across it (320 KB/frame at
REM  ~52 fps) alongside the micro-ROS agent's best-effort UDP. That UDP has
REM  no retransmit -- lose a second of it and the firmware watchdog aborts
REM  the route, which is exactly what gap=1001ms is.
REM
REM  Running vision HERE keeps that 16 MB/s entirely on Windows loopback:
REM  camera_bridge_windows.py LISTENS on 8765 and this CONNECTS to it, so
REM  with both on Windows the frame stream never enters the VM at all. Only
REM  the ~320 pose messages/sec to rosbridge still cross, which is nothing.
REM
REM  SINGLE VARIABLE: the micro-ROS agent stays in WSL2. Nothing else moves.
REM  Launched by START_TESTBED.bat when VISIONHOST=windows; run it by hand as
REM     tools\run_vision_windows.bat [threads]
REM ===========================================================================
setlocal EnableExtensions
cd /d "%~dp0.."

if not "%~1"=="" set "AGV_VISION_THREADS=%~1"
if not defined AGV_VISION_THREADS set "AGV_VISION_THREADS=8"

REM %2 is where rosbridge lives. 127.0.0.1 when the ROS side is in WSL2
REM on this machine; the Linux laptop's LAN IP when it is over there.
REM Vision publishes every pose through this, so getting it wrong means
REM a silent no-publish run -- the publisher checks is_connected once.
if not "%~2"=="" set "ROSHOST=%~2"
if not defined ROSHOST set "ROSHOST=127.0.0.1"

echo ===========================================================================
echo  apriltag_localize -- WINDOWS-NATIVE   threads=%AGV_VISION_THREADS%  rosbridge=%ROSHOST%
echo ===========================================================================
echo.
echo Waiting for camera bridge (127.0.0.1:8765) and rosbridge (%ROSHOST%:9090)...

REM Vision connects to both exactly ONCE and never retries -- the publisher
REM checks is_connected a single time, prints "not connected, poses not being
REM published", then silently never publishes again. Starting a second early
REM costs a whole run, so block until both are actually accepting.
powershell -NoProfile -ExecutionPolicy Bypass -Command "$d=(Get-Date).AddSeconds(90); foreach($hp in @(@('127.0.0.1',8765),@('%ROSHOST%',9090))){ $h=$hp[0]; $p=$hp[1]; while($true){ try{ $c=New-Object Net.Sockets.TcpClient; $c.Connect($h,$p); $c.Close(); Write-Host ('  ' + $h + ':' + $p + ' up'); break } catch { if((Get-Date) -gt $d){ Write-Host ('  TIMEOUT waiting for ' + $h + ':' + $p); exit 1 }; Start-Sleep -Milliseconds 400 } } }; exit 0"
if errorlevel 1 goto :noports

echo.
REM python3, NOT python: the Store Python carries roslibpy / cv2 /
REM pupil_apriltags (verified 2026-09-29); the conda one does not.
python3 apriltag_localize.py --frame-source-port 8765 --rosbridge=%ROSHOST%:9090 --threads %AGV_VISION_THREADS% --freeze-calib 45 --freeze-min-refs 4 --stream --no-preview
goto :done

:noports
echo.
echo NOT starting vision -- it connects once and never retries, so starting it
echo now would give you a silent no-publish run. Check the "camera bridge" and
echo "rosbridge" tabs, then run this again.
echo.
pause

:done
