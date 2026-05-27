# MotionBrain ROS2 Bridge

MVP ROS2 bridge for the Phase 4 host path. It has been validated on Raspberry
Pi 4 with Ubuntu Server 24.04 and ROS2 Jazzy.

It keeps the current ESP32 HTTP boundary intact and exposes it as ROS2 topics.
The original JSON topics remain available for debugging, while typed
`motionbrain_msgs` topics provide portfolio-grade ROS2 interfaces.

| ROS2 name | Direction | Type | Payload |
| --- | --- | --- | --- |
| `/joint_states` | publish | `sensor_msgs/msg/JointState` | MotionStatus joint fields mapped to URDF joints |
| `/motionbrain/status` | publish | `std_msgs/String` | Raw `GET /status` JSON |
| `/motionbrain/status_typed` | publish | `motionbrain_msgs/msg/MotionStatus` | Stable status fields plus raw JSON |
| `/motionbrain/events` | publish | `std_msgs/String` | Raw `GET /events?limit=N` JSON |
| `/motionbrain/events_typed` | publish | `motionbrain_msgs/msg/MotionEvent` | One typed message per ESP32 event |
| `/camera/detection` | publish | `std_msgs/String` | Color detection JSON from ESP32-CAM `/capture` |
| `/camera/detection_typed` | publish | `motionbrain_msgs/msg/CameraDetection` | Stable detection/alignment fields plus raw JSON |
| `/motionbrain/light_cmd` | subscribe | `std_msgs/String` | `on`, `off`, `toggle`, or `{"action":"toggle"}` |
| `/motionbrain/light_cmd_typed` | subscribe | `motionbrain_msgs/msg/LightCommand` | Typed search-light command |
| `/motionbrain/light_result` | publish | `std_msgs/String` | Raw `/light` command result JSON |
| `/motionbrain/light_result_typed` | publish | `motionbrain_msgs/msg/LightResult` | Stable command result fields plus raw JSON |

`/camera/detection` includes color ratio plus vision-alignment fields: `centerX`, `centerY`, `centroidX`, `centroidY`, `areaRatio`, `offsetX`, `offsetY`, `alignDeadband`, `alignment` (`LEFT`, `CENTER`, `RIGHT`, or `LOST`), and `commandSuggestion` (`base_left`, `hold`, `base_right`, or `none`).

## Build

From this repository root:

```bash
cd ros2_ws
colcon build --packages-select motionbrain_msgs motionbrain_ros_bridge
source install/setup.bash
```

## Run Directly

With the Mac or Raspberry Pi connected to `MotionBrain-AP`:

```bash
ros2 run motionbrain_ros_bridge motionbrain_status_node \
  --ros-args \
  -p motion_host:=192.168.4.1 \
  -p camera_url:=http://192.168.4.2
```

When Home Wi-Fi mode is enabled, hostnames can be used instead:

```bash
export MOTIONBRAIN_HTTP_TOKEN="your-local-command-token"
ros2 run motionbrain_ros_bridge motionbrain_status_node \
  --ros-args \
  -p motion_host:=motionbrain.local \
  -p camera_url:=http://motionbrain-cam.local
```

If `.local` names do not resolve on the Pi, use IP addresses from the router or
ESP32 serial logs.

## Run With Launch

For the Raspberry Pi Home Wi-Fi portfolio path:

```bash
export MOTIONBRAIN_HTTP_TOKEN="your-local-command-token"
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
```

Override hostnames or timing when needed:

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=192.168.1.50 \
  camera_url:=http://192.168.1.51 \
  poll_interval:=2.0 \
  http_timeout:=6.0
```

Watch bridge output:

```bash
ros2 topic echo /motionbrain/status
ros2 topic echo /motionbrain/status_typed
ros2 topic echo /joint_states
ros2 topic echo /motionbrain/events
ros2 topic echo /motionbrain/events_typed
ros2 topic echo /camera/detection
ros2 topic echo /camera/detection_typed
```

Toggle the search light through ROS2:

```bash
ros2 topic echo /motionbrain/light_result
```

In another terminal:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /motionbrain/light_cmd std_msgs/msg/String "{data: toggle}"
```

Typed command path:

```bash
ros2 topic echo /motionbrain/light_result_typed
```

In another terminal:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /motionbrain/light_cmd_typed \
  motionbrain_msgs/msg/LightCommand "{action: toggle}"
```

Validation on 2026-05-26 confirmed this path on real hardware:

```text
Raspberry Pi ROS2 /motionbrain/light_cmd
  -> motionbrain_ros_bridge
  -> token-gated ESP32 POST /light?action=toggle
  -> real SearchLight output
  -> /motionbrain/light_result
```

Full Raspberry Pi setup and verification notes are in
[docs/RASPBERRY_PI_ROS2_BRINGUP.md](../../../docs/RASPBERRY_PI_ROS2_BRINGUP.md).
