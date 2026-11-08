#!/usr/bin/env bash
set -euo pipefail
export DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 QT_QPA_PLATFORM=xcb
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
# A stopped container retains Xvfb's lock/socket while its processes are gone.
# This entrypoint owns display :1 in a fresh container PID namespace.
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
Xvfb :1 -screen 0 1440x950x24 +extension GLX +render -noreset &
for _ in $(seq 1 50); do [[ -S /tmp/.X11-unix/X1 ]] && break; sleep .1; done
openbox >/tmp/openbox.log 2>&1 &
x11vnc -display :1 -forever -shared -nopw -listen 127.0.0.1 -rfbport 5900 >/tmp/vnc.log 2>&1 &
websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900 >/tmp/novnc.log 2>&1 &
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ "${VIEWER:-rviz}" == gazebo ]]; then
    export GZ_PARTITION=motionbrain_offline
    python3 - <<'PYGUI'
from pathlib import Path
source=Path('/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config').read_text()
source=source.replace('<camera_pose>-6 0 6 0 0.5 0</camera_pose>', '<camera_pose>0.055 0.36 0.136 0 0.1 -1.57079632679</camera_pose>')
source=source.replace('<width>1000</width>', '<width>1360</width>').replace('<height>845</height>', '<height>900</height>')
Path('/tmp/motionbrain-gui.config').write_text(source)
PYGUI
    gz sim -r --gui-config /tmp/motionbrain-gui.config -v 4 /workspace/.local/visual-tools/world.sdf --render-engine ogre2 >/tmp/gazebo.log 2>&1 &
else
    python3 /opt/motionbrain/rviz_appearance.py /workspace/ros2_ws/src/motionbrain_description/urdf/motionbrain.urdf >/tmp/motionbrain-rviz.urdf
    ros2 run robot_state_publisher robot_state_publisher --ros-args \
      -p robot_description:="$(cat /tmp/motionbrain-rviz.urdf)" >/tmp/robot-state.log 2>&1 &
    mapfile -t pose_args < <(python3 - <<'PYPOSE'
import json
for name, value in json.load(open('/opt/motionbrain/display-pose.json'))['joints'].items():
    print('-p')
    print(f'zeros.{name}:={value}')
PYPOSE
    )
    ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args "${pose_args[@]}" >/tmp/joints.log 2>&1 &
    rviz2 -d /workspace/ros2_ws/src/motionbrain_description/rviz/motionbrain.rviz >/tmp/rviz.log 2>&1 &
fi
trap 'kill $(jobs -pr) 2>/dev/null || true' EXIT INT TERM
wait -n
