# motionbrain_mission

Lightweight mission supervisor for MotionBrain ROS2 demos.

This package is intentionally smaller than Nav2. It provides the portfolio
shape of a behavior flow without pretending the current robot arm is a mobile
base:

```text
detect -> align -> operator confirm -> act
```

The supervisor never publishes live motion commands. It watches typed detection
and the C++ control guard, publishes `/motionbrain/mission_state_typed`, and
only publishes a typed light command after an operator sends `confirm`.

## Topics

| ROS2 name | Direction | Type |
| --- | --- | --- |
| `/motionbrain/control_guard_typed` | subscribe | `motionbrain_msgs/msg/ControlGuard` |
| `/motionbrain/control_guard` | subscribe | `std_msgs/msg/String` JSON compatibility |
| `/camera/detection_typed` | subscribe | `motionbrain_msgs/msg/CameraDetection` |
| `/motionbrain/status_typed` | subscribe | `motionbrain_msgs/msg/MotionStatus` |
| `/motionbrain/mission_cmd_typed` | subscribe | `motionbrain_msgs/msg/MissionCommand` |
| `/motionbrain/mission_cmd` | subscribe | `std_msgs/msg/String` compatibility |
| `/motionbrain/mission_state_typed` | publish | `motionbrain_msgs/msg/MissionState` |
| `/motionbrain/mission_state` | publish | `std_msgs/msg/String` JSON compatibility |
| `/motionbrain/light_cmd_typed` | publish | `motionbrain_msgs/msg/LightCommand` |

## Commands

Publish typed commands on `/motionbrain/mission_cmd_typed`:

- `start`
- `confirm`
- `cancel`
- `reset`

Example:

```bash
ros2 topic pub --once /motionbrain/mission_cmd_typed motionbrain_msgs/msg/MissionCommand "{command: start}"
ros2 topic echo /motionbrain/mission_state_typed --once
ros2 topic pub --once /motionbrain/mission_cmd_typed motionbrain_msgs/msg/MissionCommand "{command: confirm}"
```

The legacy `/motionbrain/mission_cmd` string input remains available for demos
and simple terminal tests.

## Build

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description
source install/setup.bash
```
