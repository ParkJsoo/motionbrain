#!/usr/bin/env bash
set -euo pipefail

container="${MOTIONBRAIN_RVIZ_CONTAINER:-motionbrain-rviz}"
dashboard_url="${MOTIONBRAIN_DASHBOARD_URL:-http://motionbrain-pi.local:8765}"
mirror_src="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/motionbrain_http_mirror.py"
mirror_dst="/tmp/motionbrain_http_mirror.py"

docker cp "${mirror_src}" "${container}:${mirror_dst}"
docker exec "${container}" pkill -f "/tmp/motionbrain_http_mirror.py" >/dev/null 2>&1 || true
docker exec "${container}" pkill -f "/usr/local/bin/motionbrain_http_mirror.py" >/dev/null 2>&1 || true
docker exec -d \
  -e "MOTIONBRAIN_DASHBOARD_URL=${dashboard_url}" \
  "${container}" \
  bash -lc "set +u; source /opt/ros/jazzy/setup.bash; source /opt/motionbrain/ros2_ws/install/setup.bash; exec python3 ${mirror_dst}"

for _ in $(seq 1 12); do
  value="$(docker exec "${container}" bash -lc "set +u; source /opt/ros/jazzy/setup.bash; source /opt/motionbrain/ros2_ws/install/setup.bash; timeout 3 ros2 param get /motionbrain_http_mirror dashboard_url" 2>/dev/null || true)"
  if printf "%s" "${value}" | grep -q 'String value is: http'; then
    printf "%s\n" "${value}"
    exit 0
  fi
  sleep 1
done

echo "motionbrain_http_mirror did not become visible in ROS graph" >&2
exit 1
