# MotionBrain Demo Runbook

[한국어](DEMO_RUNBOOK.md)

This runbook is the public, reproducible procedure for capturing MotionBrain
portfolio evidence. It intentionally uses placeholders for local IP addresses,
Wi-Fi credentials, and command tokens.

## Goal

Capture a concise demo that proves this chain:

```text
STM32 handheld safety/teleop
  -> ESP32 motion controller
  -> ESP32-CAM vision input
  -> Raspberry Pi ROS2 bridge
  -> ROS2 command
  -> real SearchLight output
```

The demo should show both successful behavior and safety/authorization
boundaries.

## Safety And Privacy Rules

- Do not show real Wi-Fi passwords.
- Do not show the real `MOTIONBRAIN_HTTP_TOKEN`.
- Do not expose router admin pages in recordings.
- Keep live motion commands conservative and opt-in.
- Use `/light?action=toggle` as the ROS2 command demo. It is the safe
  non-motion actuator path.
- Use timed base nudge only for the vision alignment segment.
- Do not use `/base?action=angle` until a base-mounted IMU or encoder exists.

## Hardware Setup

Required:

- ESP32 MotionBrain controller
- STM32 handheld teleop/safety board
- ESP32-CAM
- Raspberry Pi 4 running Ubuntu Server 24.04 and ROS2 Jazzy
- Robot arm hardware and search light
- Mac or other operator machine for SSH, browser, and recording

Network:

- Controller, ESP32-CAM, Raspberry Pi, and Mac should be on the same trusted
  Home Wi-Fi network.
- Prefer `.local` hostnames when they resolve.
- Use router DHCP leases or serial logs for IP fallback.

Observed during 2026-05-26 validation:

```text
Raspberry Pi: 192.168.219.105
ESP32 controller: 192.168.219.113
ESP32-CAM: 192.168.219.114
```

Observed during 2026-05-27 typed ROS2 message validation after reboot:

```text
Raspberry Pi: 192.168.219.111
ESP32 controller: 192.168.219.109
ESP32-CAM: 192.168.219.110
```

These are DHCP observations and may change.

## Preflight

From the Mac, confirm the controller and camera are reachable:

```bash
ping -c 1 motionbrain.local
ping -c 1 motionbrain-cam.local
```

If hostnames fail, use the IP addresses from the router or serial logs:

```bash
curl -sS http://<controller-ip>/status
curl -I http://<camera-ip>/capture
```

From the Pi, confirm ROS2 and the workspace:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
printenv ROS_DISTRO
ros2 pkg list | grep motionbrain
```

Expected:

```text
jazzy
motionbrain_msgs
motionbrain_control
motionbrain_mission
motionbrain_ros_bridge
```

If the package is missing:

```bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description
source install/setup.bash
```

If the systemd bridge is already running, capture public-safe text evidence in
one pass. By default, this records the health check, topic list, typed topic
samples, and JSON compatibility samples without publishing any actuator
command.

```bash
cd ~/develop/arduino/motionbrain
tools/raspi/capture_ros2_evidence.sh
```

The 2026-05-30 Pi validation on `99154d2` produced
`/tmp/motionbrain_ros2_evidence_helper_99154d2.txt` with `Result: OK`.

To include the mission command boundary, use the opt-in mode. This publishes
only `start` and `reset`; it does not publish `confirm`.

```bash
cd ~/develop/arduino/motionbrain
CAPTURE_MISSION_BOUNDARY=1 tools/raspi/capture_ros2_evidence.sh
```

Default output path:

```text
/tmp/motionbrain_ros2_evidence_<timestamp>.txt
```

## Recording Plan

Capture these short clips or screenshots:

1. Hardware overview
   - Raspberry Pi
   - ESP32 controller
   - ESP32-CAM
   - STM32 handheld controller
   - robot arm and search light

2. ROS2 bridge evidence
   - `printenv ROS_DISTRO`
   - `ros2 topic list`
   - `/motionbrain/status`
   - `/motionbrain/status_typed`
   - `/camera/detection`
   - `/camera/detection_typed`
   - `/motionbrain/light_cmd` and `/motionbrain/light_result`
   - `/motionbrain/light_cmd_typed` and `/motionbrain/light_result_typed`
   - `/joint_states`
   - `/motionbrain/end_effector_pose`
   - `/motionbrain/kinematics_typed`
   - `/motionbrain/control_guard_typed`
   - `/motionbrain/mission_state_typed`
   - optional RViz RobotModel/TF view
   - real search light turning on

3. Safety/authorization evidence
   - token missing or wrong returns `HTTP Error 403: Forbidden`
   - deadman release stops teleop motion
   - optional: stale/safety block in `/status`

4. Vision evidence
   - ESP32-CAM capture or dashboard camera view
   - red target detection
   - `LEFT`, `CENTER`, `RIGHT`, or `LOST` alignment state
   - optional timed nudge with immediate stop

## Segment 1: ROS2 Bridge And Light Command

Open three SSH terminals to the Raspberry Pi.

### Terminal 1: Launch Bridge

Use hostnames if they work:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
```

Use IP fallback if `.local` resolution fails on the Pi:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=<controller-ip> \
  camera_url:=http://<camera-ip>
```

Expected bridge log:

```text
MotionBrain ROS2 bridge polling http://<controller-ip>:80
```

### Terminal 2: Observe Topics

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list
```

Expected topics:

```text
/camera/detection
/camera/detection_typed
/joint_states
/motionbrain/control_guard
/motionbrain/control_guard_typed
/motionbrain/end_effector_pose
/motionbrain/events
/motionbrain/events_typed
/motionbrain/kinematics
/motionbrain/kinematics_typed
/motionbrain/mission_cmd
/motionbrain/mission_cmd_typed
/motionbrain/mission_state
/motionbrain/mission_state_typed
/motionbrain/light_cmd
/motionbrain/light_cmd_typed
/motionbrain/light_result
/motionbrain/light_result_typed
/motionbrain/status
/motionbrain/status_typed
```

Capture status:

```bash
ros2 topic echo /motionbrain/status --once
ros2 topic echo /motionbrain/status_typed --once
ros2 topic echo /motionbrain/kinematics_typed --once
ros2 topic echo /motionbrain/control_guard_typed --once
ros2 topic echo /motionbrain/mission_state_typed --once
```

Capture camera detection:

```bash
ros2 topic echo /camera/detection --once
ros2 topic echo /camera/detection_typed --once
```

Start result listener before publishing the command:

```bash
ros2 topic echo /motionbrain/light_result --once
```

### Terminal 3: Publish Light Command

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub --once --wait-matching-subscriptions 1 /motionbrain/light_cmd std_msgs/msg/String "{data: toggle}"
```

Typed command alternative:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /motionbrain/light_cmd_typed motionbrain_msgs/msg/LightCommand "{action: toggle}"
```

Expected:

- Terminal 2 prints a `/motionbrain/light_result` JSON payload.
- The real search light toggles.

## Segment 2: Token Gate Check

This is optional but useful because it proves the command boundary is enforced.

Stop the bridge, then relaunch it without `MOTIONBRAIN_HTTP_TOKEN` or with an
incorrect token:

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=<controller-ip> \
  camera_url:=http://<camera-ip>
```

Run the same `/motionbrain/light_result` echo and `/motionbrain/light_cmd`
publish steps.

Expected result:

```text
HTTP Error 403: Forbidden
```

This means the ROS2 graph and bridge are working, but the ESP32 correctly
rejected the state-changing HTTP command.

## Segment 2-B: URDF / TF / Joint State Check

With the ROS2 bridge running, start the robot description launch in another
terminal:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch motionbrain_description display.launch.py
```

Check:

```bash
ros2 topic echo /joint_states --once
```

On a desktop with RViz2 installed:

```bash
ros2 launch motionbrain_description display.launch.py use_rviz:=true
```

Capture points:

- `/joint_states` includes `base_yaw_joint`, `shoulder_pitch_joint`,
  `elbow_pitch_joint`, `wrist_pitch_joint`, and `gripper_joint`.
- RViz shows the RobotModel.
- The TF tree follows `world -> base_link -> ... -> gripper_link`.

## Segment 3: ESP32-CAM Vision Evidence

From the Mac:

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/vision_host_mvp.py \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --once
```

Expected log fields:

```text
detected=Y
red_ratio=<non-zero>
align=LEFT|CENTER|RIGHT
suggest=base_left|hold|base_right
```

Optional dashboard view:

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/motionbrain_dashboard.py \
  --host 0.0.0.0 \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --timeout 6
```

Open:

```text
http://127.0.0.1:8765
http://<pi-ip>:8765
```

The `TRACKED` camera mode in `motionbrain.local` uses this dashboard API too.
Confirm that the on-page `API` field is `http://motionbrain-pi.local:8765` or
`http://<pi-ip>:8765`.

## Segment 4: Timed Vision Nudge

Use this only if the robot area is clear and the system is in a known safe
state.

Start with:

- send stop
- arm immediately before the test
- keep the red target visible
- use conservative settings
- the default `250ms`/`25%` setting is conservative; if movement is too subtle
  on video, after checking clearance and stop behavior, use about `600ms`/`40%`

Command:

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/vision_host_mvp.py \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --align-mode nudge \
  --align-nudge-ms 250 \
  --align-percent 25 \
  --enable-align-action \
  --once
```

Expected:

```text
ACTION base.left nudge=250ms success=True stopped=True
```

or:

```text
ACTION base.right nudge=250ms success=True stopped=True
```

Afterward, verify the controller is safe:

```bash
curl -sS http://<controller-ip>/status
```

Expected state for the end of a demo:

```text
state=IDLE
motors off
fault=false
```

## Segment 5: Teleop Safety Evidence

Record:

- controller armed
- deadman held
- small handheld tilt produces conservative motion
- deadman released
- robot stops

Useful status fields:

```bash
curl -sS http://<controller-ip>/status
```

Look for:

- `teleop`
- `deadman`
- `sensor`
- `state`
- `faultLatched`

## Troubleshooting

### `.local` Works On Mac But Not On Pi

Use IP fallback in the launch command:

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=<controller-ip> \
  camera_url:=http://<camera-ip>
```

### `light_result` Does Not Print

`/motionbrain/light_result` is not latched. Start the echo command before
publishing `/motionbrain/light_cmd`.

### `403 Forbidden`

The token gate is active. Relaunch the bridge with:

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
```

Then publish the command again.

### Camera Detection Is Slow Or Missing

Use the hardened defaults:

```bash
python3 tools/vision_host_mvp.py \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --timeout 6 \
  --capture-retries 2 \
  --interval 3 \
  --once
```

### Before Ending A Demo

Return the robot to a quiet state:

```bash
curl -X POST -H "X-MotionBrain: 1" -H "X-MotionBrain-Token: <local-controller-token>" \
  "http://<controller-ip>/command?cmd=stop"
```

Then confirm:

```bash
curl -sS http://<controller-ip>/status
```

## Portfolio Captions

Use short captions like:

- Raspberry Pi 4 running ROS2 Jazzy as MotionBrain host bridge.
- ESP32 motion controller remains the real-time motor and safety boundary.
- ESP32-CAM publishes color-target detection into JSON and typed ROS2 topics
  through the bridge.
- ROS2 `/motionbrain/light_cmd_typed` reaches the token-gated ESP32 command
  path.
- Real search light output confirms end-to-end command execution.
- Deadman release and token rejection demonstrate safety and authorization
  boundaries.
