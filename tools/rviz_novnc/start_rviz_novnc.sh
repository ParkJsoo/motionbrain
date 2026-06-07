#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RVIZ_RESOLUTION="${RVIZ_RESOLUTION:-1600x1000x24}"

display_number="${DISPLAY#:}"
display_number="${display_number%%.*}"
if ! pgrep -f "Xvfb ${DISPLAY}" >/dev/null 2>&1; then
    rm -f "/tmp/.X${display_number}-lock" "/tmp/.X11-unix/X${display_number}"
fi

Xvfb "${DISPLAY}" -screen 0 "${RVIZ_RESOLUTION}" -ac +extension GLX +render -noreset &
xvfb_pid=$!

for _ in $(seq 1 50); do
    if [ -S "/tmp/.X11-unix/X${display_number}" ]; then
        break
    fi
    sleep 0.1
done

openbox >/tmp/openbox.log 2>&1 &
x11vnc -display "${DISPLAY}" -forever -shared -nopw -listen 0.0.0.0 -rfbport 5900 -xkb >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900 >/tmp/novnc.log 2>&1 &

set +u
. /opt/ros/jazzy/setup.bash
. /opt/motionbrain/ros2_ws/install/setup.bash
set -u

mirror_script="${MOTIONBRAIN_HTTP_MIRROR_SCRIPT:-/usr/local/bin/motionbrain_http_mirror.py}"
mirror_pid=""
if [[ -f "${mirror_script}" ]]; then
    export MOTIONBRAIN_DASHBOARD_URL="${MOTIONBRAIN_DASHBOARD_URL:-http://motionbrain-pi.local:8765}"
    python3 "${mirror_script}" >/tmp/motionbrain_http_mirror.log 2>&1 &
    mirror_pid=$!
fi

ros2 launch motionbrain_description display.launch.py use_rviz:=true "$@" &
launch_pid=$!

trap 'kill "${launch_pid}" "${xvfb_pid}" ${mirror_pid:+"${mirror_pid}"} 2>/dev/null || true' INT TERM EXIT
wait "${launch_pid}"
