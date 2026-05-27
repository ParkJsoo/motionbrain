# motionbrain_control

C++ ROS2 control guard package for MotionBrain.

The first node, `motionbrain_control_guard_node`, subscribes to typed robot
status and camera detection topics, then publishes a JSON readiness decision on
`/motionbrain/control_guard`.

This node does not command hardware directly. It is a C++ control boundary for
future mission/action logic: it checks whether status data is fresh, the robot is
available, no fault is latched, the robot is not already moving, and optionally
whether the robot is armed and a target is detected.

## Topics

| ROS2 name | Direction | Type |
| --- | --- | --- |
| `/motionbrain/status_typed` | subscribe | `motionbrain_msgs/msg/MotionStatus` |
| `/camera/detection_typed` | subscribe | `motionbrain_msgs/msg/CameraDetection` |
| `/motionbrain/control_guard` | publish | `std_msgs/msg/String` JSON |

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `status_topic` | `/motionbrain/status_typed` | Typed status input |
| `detection_topic` | `/camera/detection_typed` | Typed camera detection input |
| `output_topic` | `/motionbrain/control_guard` | Guard decision output |
| `stale_timeout_sec` | `3.0` | Maximum allowed age for required inputs |
| `require_armed` | `false` | Require the ESP32 controller state to be armed |
| `require_detection` | `false` | Require a fresh detected camera target |
| `publish_rate_hz` | `2.0` | Guard output rate |

## Build

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_ros_bridge motionbrain_description
source install/setup.bash
```

## Run

The home Wi-Fi bridge launch starts this node by default:

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
```

Direct run:

```bash
ros2 run motionbrain_control motionbrain_control_guard_node
```

Watch the guard output:

```bash
ros2 topic echo /motionbrain/control_guard --once
```
