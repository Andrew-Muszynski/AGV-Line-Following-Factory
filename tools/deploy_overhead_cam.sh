#!/bin/bash
# Push the overhead camera recorder to the secondary-camera laptop and
# (re)start it as a systemd user service. Run from Windows Git Bash:
#
#   bash tools/deploy_overhead_cam.sh
#
# Then open http://192.168.0.73:8090/ from any machine on the lab network.
set -o pipefail

CAM_USER="${CAM_USER:-industrious}"
CAM_IP="${CAM_IP:-192.168.0.73}"
HOST="$CAM_USER@$CAM_IP"

cd "$(dirname "$0")/.." || exit 1

echo "deploying to $HOST:~/overhead_cam"
ssh "$HOST" "mkdir -p ~/overhead_cam ~/.config/systemd/user ~/Videos/testbed" || exit 1
scp overhead_cam/overhead_recorder.py overhead_cam/overhead_recorder.html \
    overhead_cam/README.txt "$HOST:overhead_cam/" || exit 1
scp overhead_cam/overhead-recorder.service "$HOST:.config/systemd/user/" || exit 1

ssh "$HOST" 'command -v ffmpeg >/dev/null || { echo "ffmpeg missing: run  sudo apt install ffmpeg  on that laptop"; exit 1; }
systemctl --user daemon-reload &&
systemctl --user enable overhead-recorder >/dev/null 2>&1 &&
systemctl --user restart overhead-recorder &&
sleep 2 &&
systemctl --user --no-pager --lines=5 status overhead-recorder | head -12
if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != "yes" ]; then
  echo
  echo "NOTE: the service stops when you log out of that laptop. To keep it"
  echo "running after logout and start it at boot, run once ON THAT LAPTOP:"
  echo "  sudo loginctl enable-linger $USER"
fi' || exit 1

echo
echo "open: http://$CAM_IP:8090/"
