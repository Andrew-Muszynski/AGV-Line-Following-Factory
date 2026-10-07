#!/bin/bash
# Per-role launcher for the LINUX LAPTOP (192.168.0.212).
#
# Counterpart to tools/run_role.sh, which does the same job inside WSL2.
# Deployed to $HOME by tools/deploy_linux.sh and invoked over SSH from
# START_TESTBED.bat, so each process runs here but its output lands in an
# ordinary Windows Terminal tab -- no tmux, copyable, all in one window.
#
# WHY THE AGENT IS BACK HERE (2026-09-29). ERROR WHEEL_CMD_TIMEOUT
# gap=1001ms was root-caused to the micro-ROS agent / XRCE-DDS layer by
# eliminating everything else with measurements:
#
#   relay bandwidth  moved ~16MB/s of JPEG off the WSL2 NIC -- no change
#   vision CPU       11-25 percent across the whole --threads sweep
#   navigator stall  596 turns, worst control-loop gap 85ms, none over 300
#   WiFi / RF        ping monitor during a live run: ZERO loss within +/-3s
#                    of the abort, both robots answering at 6.6-8.5ms
#
# The navigator published 0-120ms before each abort, the firmware caught the
# 1000ms crossing within 1ms, and losses arrived simultaneously across
# robots (5 within 28.8ms). Everything below the agent was healthy.
#
# THE SUPERVISOR MUST LIVE ON THE SAME MACHINE AS THE AGENT. The agent
# publishes Alvik*_pose / Alvik*_status as DDS topics and fleetSupervisor
# subscribes to them; splitting them across the WSL2 boundary is the
# "Alvik1_pose subscription never matched a publisher" failure. Do not run
# the agent here and the supervisor in WSL2.
#
# NO `set -u`. /opt/ros/jazzy/setup.bash reads unset variables, so `set -u`
# makes sourcing ROS fail instantly.
set -o pipefail

# The deploy is FLATTENED: fleet/*.py land directly in this directory, no
# fleet/ subdirectory. See tools/deploy_linux.sh.
REPO="$HOME/agv_testbed_1/agv_testbed/ORACLE_VM"
ROS_SETUP=/opt/ros/jazzy/setup.bash
MICROROS_SETUP="$HOME/microros_ws/install/setup.bash"
SUPERVISOR_LOG="$HOME/supervisor_live.log"

# Both of these live on the WINDOWS machine. Ollama especially: CPU
# inference on this laptop starved the micro-ROS agent and dropped sessions
# (TERMINALS_CHEATSHEET.md), which is the same sensitivity that brought the
# agent back here in the first place. Keep heavy work off this box.
WINDOWS_IP=192.168.0.162
: "${OLLAMA_URL:=http://$WINDOWS_IP:11434}"
: "${METRICS_URL:=http://$WINDOWS_IP:8000}"
: "${AGV_NO_LLM:=1}"

# Explicitly unset, not set. Restricting discovery to LOCALHOST was tried on
# 2026-09-29 and emptied `ros2 topic list` entirely.
unset ROS_AUTOMATIC_DISCOVERY_RANGE

role="${1:-}"

banner() { printf '\n=== %s ===\n\n' "$*"; }
die()    { printf '\nERROR: %s\n\n' "$*" >&2; read -r -p "press enter..."; exit 1; }

need_ros() {
  [ -f "$ROS_SETUP" ] || die "no $ROS_SETUP -- is ROS 2 Jazzy installed?"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
}

wait_for_port() {  # host port label timeout
  local h="$1" p="$2" label="$3" t="${4:-30}" i=0
  printf 'waiting for %s on %s:%s' "$label" "$h" "$p"
  while ! (exec 3<>"/dev/tcp/$h/$p") 2>/dev/null; do
    i=$((i + 1)); [ "$i" -ge "$((t * 2))" ] && { printf ' TIMEOUT\n'; return 1; }
    printf '.'; sleep 0.5
  done
  printf ' up\n'; return 0
}

case "$role" in
  agent)
    banner "micro-ROS agent  (robots connect here, UDP 8888)"
    need_ros
    [ -f "$MICROROS_SETUP" ] || die "no $MICROROS_SETUP -- micro_ros_agent
 is not built on this machine. Build it, or the robots have nothing to
 connect to."
    # shellcheck disable=SC1090
    source "$MICROROS_SETUP"
    # A stale daemon caches the previous run's graph and makes topics look
    # present when nothing is behind them.
    ros2 daemon stop >/dev/null 2>&1
    echo "robots must be flashed with v2 (AGENT_IP=192.168.0.212)."
    echo "a green LED only means a robot reached SOME agent once -- after"
    echo "restarting this, power-cycle the robots."
    echo
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    ;;

  rosbridge)
    banner "rosbridge websocket  (TCP 9090)"
    need_ros
    echo "vision (on Windows) and the HTML page both connect here, so this"
    echo "machine's LAN IP -- not 127.0.0.1 -- is what they must point at."
    echo
    exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ;;

  supervisor)
    banner "fleetSupervisor  (listen mode)"
    need_ros
    wait_for_port 127.0.0.1 9090 "rosbridge" 40 \
      || die "rosbridge never came up on 9090 -- start that tab first"
    cd "$REPO" || die "no $REPO -- run tools/deploy_linux.sh from Windows"
    [ -f fleetSupervisor.py ] || die "no fleetSupervisor.py in $REPO --
 run tools/deploy_linux.sh from Windows first"
    echo "log -> $SUPERVISOR_LOG"
    llm_flag=""
    if [ "$AGV_NO_LLM" = "1" ]; then
      llm_flag="--no-llm"
      echo "LLM advisor: OFF (AGV_NO_LLM=1)"
    else
      echo "LLM advisor: ON  -> $OLLAMA_URL"
    fi
    echo "metrics -> $METRICS_URL"
    echo
    # No exec: the tee pipeline has to stay in this shell.
    OLLAMA_URL="$OLLAMA_URL" METRICS_URL="$METRICS_URL" \
      python3 -u fleetSupervisor.py --listen $llm_flag 2>&1 \
      | tee -a "$SUPERVISOR_LOG"
    ;;

  shell)
    banner "ROS shell on the Linux laptop"
    need_ros
    cd "$REPO" 2>/dev/null || cd "$HOME" || exit 1
    echo "try: ros2 topic list | sort"
    echo "     ros2 topic hz /Alvik1_pose"
    exec bash
    ;;

  wifi-check)
    # The link to this machine measured 28-181ms RTT (avg 104ms) from
    # Windows on 2026-09-29, against 6.7ms to the robots. That is WiFi
    # power-saving, and this box is about to host the agent.
    banner "WiFi power-save check"
    # `iw` is NOT installed on this machine -- iwconfig (wireless-tools) is,
    # so use that. Confirmed 2026-09-29: Power Management:on, iwlmvm
    # power_scheme=2, with a strong signal (-43 dBm, quality 67). The high
    # idle RTT is the radio sleeping between beacons, not a weak link.
    for d in $(ls /sys/class/net | grep -E '^wl'); do
      echo "interface $d:"
      iwconfig "$d" 2>/dev/null | grep -i 'power man' | sed 's/^/  /'         || echo "  (iwconfig not available either)"
      echo "  disable with:  sudo iwconfig $d power off"
    done
    echo
    echo "signal (level should be better than about -70 dBm):"
    tail -2 /proc/net/wireless | sed 's/^/  /'
    echo
    echo "This does NOT persist across reboots -- re-run after any restart."
    read -r -p "press enter..."
    ;;

  *)
    echo "usage: bash run_role_linux.sh {agent|rosbridge|supervisor|shell|wifi-check}"
    exit 2
    ;;
esac
