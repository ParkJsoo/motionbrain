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
| `motionbrain_msgs/msg/NodeLifecycleStatus` | Lifecycle-style primary state heartbeat for bridge-side nodes |
| `motionbrain_msgs/msg/LightCommand` | Search-light command request |
| `motionbrain_msgs/msg/LightResult` | Search-light command result |
| `motionbrain_msgs/msg/RoutineCommand` | Guarded routine `status`, `dry_run`, or `abort` request |
| `motionbrain_msgs/msg/RoutineResult` | Guarded routine command result, bridge policy outcome, and raw response JSON |
| `motionbrain_msgs/msg/RoutineStatus` | Guarded routine readiness, executor policy/state, feedback/base-yaw readiness, sensor/teleop/safety diagnostics, and raw `/routine` JSON |

## Actions

| Action | Purpose |
| --- | --- |
| `motionbrain_msgs/action/GuardedRoutine` | Long-running-client boundary for guarded routine `status` and `dry_run` requests with feedback and the same bridge-local `run` rejection policy |

## Services

| Service | Purpose |
| --- | --- |
| `motionbrain_msgs/srv/GuardedRoutineCommand` | Request/response guarded routine command boundary for `status`, `dry_run`, and `abort`; `run`/`execute` remains locally rejected by bridge policy |

## Build

```bash
cd ros2_ws
colcon build --packages-select motionbrain_msgs
source install/setup.bash
ros2 interface show motionbrain_msgs/msg/MotionStatus
ros2 interface show motionbrain_msgs/action/GuardedRoutine
ros2 interface show motionbrain_msgs/srv/GuardedRoutineCommand
```
