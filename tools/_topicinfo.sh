#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
for t in Alvik1_status Alvik1_pose Alvik1_vision_pose; do
  echo "=== /$t ==="
  ros2 topic info "/$t" --verbose 2>/dev/null \
    | grep -E "Publisher count|Subscription count|Reliability|Node name|Endpoint type"
  echo
done
echo "=== is anything publishing status at all? (5s) ==="
timeout 5 ros2 topic echo /Alvik1_status --once 2>/dev/null || echo "  (no status message in 5s)"
echo "=== pose? (5s) ==="
timeout 5 ros2 topic echo /Alvik1_pose --once 2>/dev/null | head -2 || echo "  (no pose message in 5s)"
