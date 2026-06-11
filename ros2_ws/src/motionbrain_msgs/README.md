# MotionBrain ROS2 Messages

Typed ROS2 interfaces for the MotionBrain Raspberry Pi bridge.

These messages promote the stable parts of the ESP32 and ESP32-CAM HTTP JSON
payloads into first-class ROS2 topics while preserving each raw JSON payload for
debugging and schema evolution.

## Messages

| Message | Purpose |
| --- | --- |
| `motionbrain_msgs/msg/MotionStatus` | Robot state, motion flags, base angle estimate, and raw `/status` JSON |
| `motionbrain_msgs/msg/MotionEvent` | One typed event from `/events?limit=N` |
| `motionbrain_msgs/msg/CameraDetection` | ESP32-CAM or Pi perception selected target, label/confidence, and alignment result |
| `motionbrain_msgs/msg/KinematicsState` | FK diagnostics and optional IK suggestion fields |
| `motionbrain_msgs/msg/ControlGuard` | Control readiness guard output from typed status and detection |
| `motionbrain_msgs/msg/MissionCommand` | Operator mission command request |
| `motionbrain_msgs/msg/MissionState` | Detect-align-confirm-act mission supervisor state |
| `motionbrain_msgs/msg/LightCommand` | Search-light command request |
| `motionbrain_msgs/msg/LightResult` | Search-light command result |
| `motionbrain_msgs/msg/RoutineStatus` | Guarded routine readiness, executor policy/state, sensor/teleop/safety diagnostics, and raw `/routine` JSON |

## Build

```bash
cd ros2_ws
colcon build --packages-select motionbrain_msgs
source install/setup.bash
ros2 interface show motionbrain_msgs/msg/MotionStatus
```
