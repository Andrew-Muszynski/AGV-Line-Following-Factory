#!/bin/bash
# Push the current fleet code to the LINUX LAPTOP, from Windows Git Bash.
#
#   bash tools/deploy_linux.sh
#
# The Linux laptop's copy predates everything changed on 2026-09-29 --
# standing holds, collision detection + the parked-neighbour exemption,
# settle_heading(), robot_online()/PING, the corridor_key conflict fix. None
# of that is there yet, so moving the supervisor back WITHOUT this deploy
# silently reverts weeks of work.
#
# The destination is FLATTENED on purpose: fleet/*.py go straight into
# ORACLE_VM/ with no fleet/ subdirectory, which is how that checkout has
# always been laid out.
set -o pipefail

LINUX_USER="${LINUX_USER:-romeshprasad}"
LINUX_IP="${LINUX_IP:-192.168.0.212}"
DEST="${DEST:-agv_testbed_1/agv_testbed/ORACLE_VM}"
HOST="$LINUX_USER@$LINUX_IP"

cd "$(dirname "$0")/.." || exit 1

# fleetSupervisor.py is GITIGNORED (.gitignore line 8), so it never shows in
# `git status` and is easy to forget. It is the single most important file
# here.
FILES=(
  fleet/fleetSupervisor.py
  fleet/camera_grid_navigate.py
  fleet/dispatch_advisor.py
  fleet/localization_modes.py
  fleet/metrics_events.py
)

echo "deploying to $HOST:~/$DEST"
echo
for f in "${FILES[@]}"; do
  [ -f "$f" ] || { echo "MISSING: $f"; exit 1; }
  printf '  %-40s %8s bytes  %s\n' "$f" "$(stat -c%s "$f")" \
         "$(date -r "$f" '+%Y-%m-%d %H:%M')"
done
echo

read -r -p "proceed? [y/N] " ok
[ "$ok" = "y" ] || { echo "aborted"; exit 1; }

ssh "$HOST" "mkdir -p ~/$DEST" || exit 1

# timing_calibration.json is LEARNED per-robot state, not source. The Linux
# laptop has its own, and overwriting it throws away whatever that machine
# measured. Back it up there before touching it, and copy ours separately
# so this is a deliberate choice rather than a side effect.
echo "backing up the remote timing calibration..."
ssh "$HOST" "cd ~/$DEST && [ -f timing_calibration.json ] && cp timing_calibration.json timing_calibration.json.bak.\$(date +%Y%m%d_%H%M%S) && echo '  backed up' || echo '  none there yet'"

echo
echo "copying source..."
scp "${FILES[@]}" "$HOST:~/$DEST/" || exit 1

echo
echo "copying the role launcher to \$HOME..."
scp tools/run_role_linux.sh "$HOST:~/" || exit 1
ssh "$HOST" "chmod +x ~/run_role_linux.sh"

echo
echo "verifying what landed..."
ssh "$HOST" "cd ~/$DEST && ls -l fleetSupervisor.py camera_grid_navigate.py dispatch_advisor.py localization_modes.py metrics_events.py && echo && python3 -c 'import ast,sys
for f in [\"fleetSupervisor.py\",\"camera_grid_navigate.py\",\"dispatch_advisor.py\"]:
    ast.parse(open(f).read())
    print(\"  parses OK\", f)'"

echo
echo "NOT copied: timing_calibration.json (learned state -- the remote keeps"
echo "its own). To push ours anyway:"
echo "  scp fleet/timing_calibration.json $HOST:~/$DEST/"
echo
echo "done."
