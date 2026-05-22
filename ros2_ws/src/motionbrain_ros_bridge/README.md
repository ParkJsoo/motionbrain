# MotionBrain ROS2 Bridge

MVP ROS2 bridge for the Phase 4 host path.

It keeps the current ESP32 HTTP boundary intact and exposes it as ROS2 topics:

| ROS2 name | Direction | Type | Payload |
| --- | --- | --- | --- |
| `/motionbrain/status` | publish | `std_msgs/String` | Raw `GET /status` JSON |
| `/motionbrain/events` | publish | `std_msgs/String` | Raw `GET /events?limit=N` JSON |
| `/camera/detection` | publish | `std_msgs/String` | Color detection JSON from ESP32-CAM `/capture` |
| `/motionbrain/light_cmd` | subscribe | `std_msgs/String` | `on`, `off`, `toggle`, or `{"action":"toggle"}` |
| `/motionbrain/light_result` | publish | `std_msgs/String` | Raw `/light` command result JSON |

`/camera/detection` includes color ratio plus vision-alignment fields: `centerX`, `centerY`, `centroidX`, `centroidY`, `areaRatio`, `offsetX`, `offsetY`, `alignDeadband`, `alignment` (`LEFT`, `CENTER`, `RIGHT`, or `LOST`), and `commandSuggestion` (`base_left`, `hold`, `base_right`, or `none`).

## Build

From this repository root:

```bash
cd ros2_ws
colcon build --packages-select motionbrain_ros_bridge
source install/setup.bash
```

## Run

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

Watch bridge output:

```bash
ros2 topic echo /motionbrain/status
ros2 topic echo /motionbrain/events
ros2 topic echo /camera/detection
```

Toggle the search light through ROS2:

```bash
ros2 topic pub --once /motionbrain/light_cmd std_msgs/msg/String "{data: toggle}"
ros2 topic echo /motionbrain/light_result
```
