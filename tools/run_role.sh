#!/usr/bin/env bash
# Launch ONE testbed process with the right environment for it.
#
# Called once per terminal window by START_TESTBED.bat, so that all the
# environment knowledge lives here in bash instead of being duplicated across
# .bat quoting. Also usable by hand:
#
#     bash tools/run_role.sh agent
#     bash tools/run_role.sh supervisor
#
# WHY IT MATTERS WHICH ROLE GETS WHAT
#   agent       needs ROS *and* the micro-ROS overlay (~/microros_ws)
#   rosbridge   needs ROS only
#   supervisor  needs ROS only (it imports rclpy)
#   vision      needs NEITHER -- apriltag_localize.py talks to rosbridge over
#               a websocket via roslibpy, so it is not a DDS participant
#
# Getting this wrong fails SILENTLY: a process in the wrong scope simply does
# not see the others, with nothing printed. 2026-09-29 was largely lost to
# that.
# NO `set -u`. /opt/ros/jazzy/setup.bash reads unset variables
# (AMENT_TRACE_SETUP_FILES on line 8), so `set -u` makes sourcing ROS fail
# instantly -- every window printed "unbound variable" and dropped to a
# prompt without starting anything. Learned the hard way 2026-09-29.
set -o pipefail

REPO=/mnt/c/Users/muszy/VRP
ROS_SETUP=/opt/ros/jazzy/setup.bash
MICROROS_SETUP="$HOME/microros_ws/install/setup.bash"
CAMERA_PORT=8765
ROSBRIDGE_HOSTPORT=127.0.0.1:9090
SUPERVISOR_LOG="$HOME/supervisor_live.log"
: "${OLLAMA_URL:=http://127.0.0.1:11434}"
: "${METRICS_URL:=http://127.0.0.1:8000}"

# --- load-shedding knobs, set by START_TESTBED.bat -----------------------
# 2026-09-29: Alvik1 died with ERROR WHEEL_CMD_TIMEOUT gap=1004ms. The
# control loop was NOT at fault -- the navigator reported "0ms since this
# side's last send_wheel()" and the worst tick gap all run was 33ms. The
# commands were sent and did not arrive, i.e. the micro-ROS agent was not
# getting scheduled promptly enough to pass them on. Zero such stalls ever
# occurred while the agent had the Linux laptop to itself; seven appeared in
# the first run after it moved onto this machine alongside AprilTag
# detection, camera capture, Ollama, Flask and rosbridge.
#
# These two default to the reduced-load configuration so the next run tests
# that theory. Set them back in START_TESTBED.bat once it is settled --
# the LLM dispatch advisor is part of the thesis, not scaffolding.
: "${AGV_NO_LLM:=1}"            # 1 = pass --no-llm to the supervisor
: "${AGV_VISION_THREADS:=8}"    # was 16; AprilTag detection is pure CPU

# Explicitly UNSET, not set. Restricting discovery to LOCALHOST was tried on
# 2026-09-29 and made things worse -- `ros2 topic list` went completely empty,
# i.e. the CLI stopped seeing any participant at all. Clearing it here means
# every window is in the same known scope regardless of ~/.bashrc leftovers.
unset ROS_AUTOMATIC_DISCOVERY_RANGE

role="${1:-}"

banner() { printf '\n=== %s ===\n\n' "$*"; }
die()    { printf '\nERROR: %s\n\n' "$*" >&2; exit 1; }

need_ros() {
  [ -f "$ROS_SETUP" ] || die "missing $ROS_SETUP"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
}

wait_for_port() {  # host port label timeout
  local host=$1 port=$2 label=$3 secs=${4:-30} i
  for ((i = 0; i < secs; i++)); do
    timeout 1 bash -c "</dev/tcp/$host/$port" 2>/dev/null && return 0
    [ "$i" = 0 ] && printf 'waiting for %s on %s' "$label" "$port"
    printf '.'; sleep 1
  done
  echo
  return 1
}

case "$role" in

  agent)
    banner "micro-ROS agent  (UDP 8888)"
    need_ros
    [ -f "$MICROROS_SETUP" ] || die "micro-ROS overlay not built: $MICROROS_SETUP"
    # shellcheck disable=SC1090
    source "$MICROROS_SETUP"
    # The daemon caches the ROS graph and inherits the environment of whatever
    # first starts it. A leftover daemon from a differently-configured shell
    # reports a stale or empty graph, which reads exactly like a dead network.
    # The agent window comes up first, so clear it here, once.
    ros2 daemon stop >/dev/null 2>&1
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    ;;

  rosbridge)
    banner "rosbridge websocket  (TCP 9090)"
    need_ros
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ;;

  vision)
    banner "apriltag_localize  (no ROS sourcing -- roslibpy over websocket)"
    # Vision CONNECTS to both of these and does not retry: the publisher
    # checks is_connected once, prints "not connected, poses not being
    # published", and then silently never publishes again.
    wait_for_port 127.0.0.1 "$CAMERA_PORT" "camera bridge (Windows)" 30 \
      || die "camera bridge never came up on $CAMERA_PORT -- start it on Windows"
    wait_for_port 127.0.0.1 9090 "rosbridge" 40 \
      || die "rosbridge never came up on 9090"
    echo
    cd "$REPO" || die "no $REPO"
    echo "threads=$AGV_VISION_THREADS"
    python3 apriltag_localize.py \
      --frame-source-port "$CAMERA_PORT" \
      --rosbridge="$ROSBRIDGE_HOSTPORT" \
      --threads "$AGV_VISION_THREADS" \
      --freeze-calib 45 --freeze-min-refs 4 --stream --no-preview
    ;;

  supervisor)
    banner "fleetSupervisor  (listen mode)"
    need_ros
    wait_for_port 127.0.0.1 9090 "rosbridge" 40 \
      || die "rosbridge never came up on 9090"
    echo
    cd "$REPO/fleet" || die "no $REPO/fleet"
    echo "log -> $SUPERVISOR_LOG"
    llm_flag=""
    if [ "$AGV_NO_LLM" = "1" ]; then
      llm_flag="--no-llm"
      echo "LLM advisor: OFF (AGV_NO_LLM=1)"
    else
      echo "LLM advisor: ON  -> $OLLAMA_URL"
    fi
    # No exec: the tee pipeline has to stay in this shell.
    OLLAMA_URL="$OLLAMA_URL" METRICS_URL="$METRICS_URL" \
      python3 -u fleetSupervisor.py --listen $llm_flag 2>&1 \
      | tee -a "$SUPERVISOR_LOG"
    ;;

  shell)
    banner "sourced shell  (ros2 topic list, camera_grid_navigate.py, ...)"
    need_ros
    cd "$REPO/fleet" || die "no $REPO/fleet"
    exec bash
    ;;

  status)
    need_ros
    echo "ROS graph:"
    ros2 topic list 2>/dev/null | sed 's/^/    /' || echo "    (ros2 topic list failed)"
    echo
    for hp in "9090 rosbridge" "$CAMERA_PORT camera-bridge(Windows)" "8000 app.py(Windows)"; do
      set -- $hp
      if timeout 1 bash -c "</dev/tcp/127.0.0.1/$1" 2>/dev/null; then
        echo "    UP    $2  ($1)"
      else
        echo "    DOWN  $2  ($1)"
      fi
    done
    echo "    (agent is UDP 8888 -- not probeable this way, check its window)"
    ;;

  *)
    sed -n '2,20p' "$0"
    echo
    echo "roles: agent | rosbridge | vision | supervisor | shell | status"
    exit 2
    ;;
esac

# The process has exited (crash, Ctrl-C, or a clean finish). Keep the tab
# alive with a usable shell so the error stays on screen instead of the tab
# vanishing -- that is the whole reason these are separate tabs.
status=$?
echo
echo "--- '$role' exited (status $status); shell kept open ---"
exec bash
