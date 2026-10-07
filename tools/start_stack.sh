#!/usr/bin/env bash
# Bring up the whole WSL2 side of the testbed in one tmux session.
#
# WHY
# ---
# The stack is four processes with three different environments: two need ROS
# sourced, one of those also needs the micro-ROS overlay, and one needs no ROS
# at all. Getting that wrong fails SILENTLY -- a process in the wrong
# discovery scope simply cannot see the others, with no error printed -- and
# 2026-09-29 was largely spent chasing exactly that.
#
# USE
#   bash tools/start_stack.sh          # start everything, attach
#   bash tools/start_stack.sh start    # same, without attaching
#   bash tools/start_stack.sh stop     # kill the session
#   bash tools/start_stack.sh status   # what is up, and the ROS graph
#
# Then: tmux attach -t agv     ctrl-b n / ctrl-b p    switch window
#       ctrl-b d              detach, leaving it all running
#
# NOT STARTED HERE -- these are native Windows, run them yourself first:
#   python camera_bridge_windows.py --camera 1 --backend msmf --port 8765 --jpeg-quality 95
#   python app.py
set -uo pipefail

SESSION=agv
REPO=/mnt/c/Users/muszy/VRP
ROS_SETUP=/opt/ros/jazzy/setup.bash
MICROROS_SETUP="$HOME/microros_ws/install/setup.bash"
CAMERA_PORT=8765
ROSBRIDGE_HOSTPORT=127.0.0.1:9090
SUPERVISOR_LOG="$HOME/supervisor_live.log"
: "${OLLAMA_URL:=http://127.0.0.1:11434}"
: "${METRICS_URL:=http://127.0.0.1:8000}"

# ROS_AUTOMATIC_DISCOVERY_RANGE is explicitly UNSET, not set. Restricting it
# to LOCALHOST was tried on 2026-09-29 and made things worse -- `ros2 topic
# list` went completely empty, i.e. the CLI could no longer see any
# participant at all. Unsetting it here means the session is in a known scope
# whatever happens to be in ~/.bashrc.
ENV_ROS="unset ROS_AUTOMATIC_DISCOVERY_RANGE; source $ROS_SETUP"
ENV_AGENT="$ENV_ROS; source $MICROROS_SETUP"

red()  { printf '\033[31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[32m%s\033[0m\n' "$*"; }
ylw()  { printf '\033[33m%s\033[0m\n' "$*"; }

port_open() {  # host port -- true if something is listening
  timeout 1 bash -c "</dev/tcp/$1/$2" 2>/dev/null
}

preflight() {
  local bad=0
  [ -f "$ROS_SETUP" ]      || { red "missing $ROS_SETUP"; bad=1; }
  [ -f "$MICROROS_SETUP" ] || { red "missing $MICROROS_SETUP -- the micro-ROS overlay is not built"; bad=1; }
  [ -d "$REPO/fleet" ]     || { red "missing $REPO/fleet"; bad=1; }
  command -v tmux >/dev/null || { red "tmux not installed:  sudo apt install tmux"; bad=1; }
  [ "$bad" = 0 ] || exit 1

  # camera_bridge_windows.py runs natively on Windows and apriltag_localize
  # CONNECTS to it. Starting vision against a dead port just prints nothing
  # useful, so say so plainly rather than letting it look like a vision bug.
  if port_open 127.0.0.1 "$CAMERA_PORT"; then
    grn "camera bridge is up on $CAMERA_PORT"
  else
    ylw "camera bridge NOT listening on $CAMERA_PORT -- start it on WINDOWS first:"
    ylw "   python camera_bridge_windows.py --camera 1 --backend msmf --port 8765 --jpeg-quality 95"
    ylw "   (starting anyway; vision will publish nothing until it is up)"
  fi
  if port_open 127.0.0.1 8000; then
    grn "metrics backend (app.py) is up on 8000"
  else
    ylw "app.py NOT listening on 8000 -- metrics will not be recorded"
  fi
}

start() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    red "session '$SESSION' already exists -- 'bash tools/start_stack.sh stop' first"
    exit 1
  fi
  preflight

  # The ros2 daemon caches the discovery graph and INHERITS the environment of
  # whatever first starts it. A daemon left over from a shell with different
  # ROS env will report a stale or empty graph, which reads exactly like a
  # broken network. Kill it so it respawns in this session's scope.
  bash -lc "$ENV_ROS; ros2 daemon stop" >/dev/null 2>&1

  tmux new-session  -d -s "$SESSION" -n agent
  tmux send-keys -t "$SESSION:agent" \
    "$ENV_AGENT; ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888" C-m

  tmux new-window -t "$SESSION" -n rosbridge
  tmux send-keys -t "$SESSION:rosbridge" \
    "$ENV_ROS; ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

  # rosbridge has to be accepting websockets before vision connects: the
  # publisher checks is_connected once, prints "not connected, poses not being
  # published", and never retries.
  printf 'waiting for rosbridge on 9090'
  for _ in $(seq 1 30); do
    port_open 127.0.0.1 9090 && break
    printf '.'; sleep 1
  done
  echo
  port_open 127.0.0.1 9090 && grn "rosbridge up" || red "rosbridge did NOT come up -- check the rosbridge window"

  # No ROS sourcing here on purpose: apriltag_localize talks to rosbridge over
  # a websocket via roslibpy, not rclpy, so it is not a DDS participant.
  tmux new-window -t "$SESSION" -n vision
  tmux send-keys -t "$SESSION:vision" \
    "cd $REPO && python3 apriltag_localize.py --frame-source-port $CAMERA_PORT \
--rosbridge=$ROSBRIDGE_HOSTPORT --threads 16 --freeze-calib 45 \
--freeze-min-refs 4 --stream --no-preview" C-m

  tmux new-window -t "$SESSION" -n supervisor
  tmux send-keys -t "$SESSION:supervisor" \
    "$ENV_ROS; cd $REPO/fleet && OLLAMA_URL=$OLLAMA_URL METRICS_URL=$METRICS_URL \
python3 -u fleetSupervisor.py --listen 2>&1 | tee -a $SUPERVISOR_LOG" C-m

  # A spare shell, already sourced, for ros2 topic list / echo and for running
  # camera_grid_navigate.py by hand.
  tmux new-window -t "$SESSION" -n shell
  tmux send-keys -t "$SESSION:shell" "$ENV_ROS; cd $REPO/fleet" C-m

  grn "started: agent, rosbridge, vision, supervisor, shell"
  echo "  supervisor log: $SUPERVISOR_LOG"
  echo "  attach:         tmux attach -t $SESSION"
}

status() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    grn "tmux session '$SESSION':"
    tmux list-windows -t "$SESSION" | sed 's/^/    /'
  else
    red "no tmux session '$SESSION'"
  fi
  for p in "8888/udp agent" "9090 rosbridge" "$CAMERA_PORT camera-bridge(Windows)" "8000 app.py(Windows)"; do
    set -- $p
    case "$1" in
      *udp) echo "    (udp 8888 not probed -- check the agent window)" ;;
      *) port_open 127.0.0.1 "$1" && grn "    $2 listening on $1" \
                                  || red  "    $2 NOT listening on $1" ;;
    esac
  done
  echo
  echo "ROS graph:"
  bash -lc "$ENV_ROS; ros2 topic list" 2>/dev/null | sed 's/^/    /' \
    || red "    ros2 topic list failed"
}

case "${1:-attach}" in
  start)  start ;;
  attach) start; tmux attach -t "$SESSION" ;;
  stop)   tmux kill-session -t "$SESSION" 2>/dev/null && grn "stopped" \
            || red "no session to stop"
          bash -lc "$ENV_ROS; ros2 daemon stop" >/dev/null 2>&1 ;;
  status) status ;;
  *)      sed -n '2,25p' "$0"; exit 2 ;;
esac
