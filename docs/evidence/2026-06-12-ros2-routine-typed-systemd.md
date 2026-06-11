# 2026-06-12 ROS2 Routine Typed Systemd Evidence

This note summarizes public-safe text evidence captured from the Raspberry Pi
after the guarded routine diagnostics were added to the ROS2 bridge.

## Runtime

- Host: `motionbrain-pi`
- Pi address used from the Mac: `192.168.219.113`
- OS: Ubuntu 24.04.4 LTS, Linux `6.8.0-1057-raspi`, `aarch64`
- ROS2: `jazzy`
- Git commit: `24d43a6 Add typed routine status topic`
- Service: `motionbrain-ros-bridge.service`
- Service state: `active (running)`
- Systemd launch inputs:
  - controller: `192.168.219.110`
  - ESP32-CAM: `http://192.168.219.111`
  - perception: `http://127.0.0.1:8766`

The service cgroup contained:

- `motionbrain_status_node`
- `motionbrain_joint_state_node`
- `motionbrain_kinematics_node`
- `motionbrain_control_guard_node`
- `motionbrain_mission_supervisor`

## Build And Interface Check

The Pi workspace was fast-forwarded to `24d43a6`, then rebuilt with:

```bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description
```

The generated interface inventory included:

```text
motionbrain_msgs/msg/CameraDetection
motionbrain_msgs/msg/ControlGuard
motionbrain_msgs/msg/KinematicsState
motionbrain_msgs/msg/LightCommand
motionbrain_msgs/msg/LightResult
motionbrain_msgs/msg/MissionCommand
motionbrain_msgs/msg/MissionState
motionbrain_msgs/msg/MotionEvent
motionbrain_msgs/msg/MotionStatus
motionbrain_msgs/msg/RoutineStatus
```

## Health Check

Command:

```bash
CHECK_SERVICE=1 SAMPLE_TIMEOUT_SECONDS=25 tools/raspi/check_ros_bridge_health.sh
```

Captured result:

```text
OK service active: motionbrain-ros-bridge.service
OK topic: /motionbrain/status_typed
OK topic: /motionbrain/routine
OK topic: /motionbrain/routine_typed
OK topic: /camera/detection_typed
OK topic: /joint_states
OK topic: /motionbrain/end_effector_pose
OK topic: /motionbrain/kinematics_typed
OK topic: /motionbrain/control_guard_typed
OK topic: /motionbrain/mission_state_typed
OK status typed sample
OK routine diagnostics sample
OK routine typed diagnostics sample
OK camera detection typed sample
OK joint state sample
OK end-effector pose sample
OK kinematics typed sample
OK control guard typed sample
OK mission state typed sample
```

## Topic Inventory

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
/motionbrain/light_cmd
/motionbrain/light_cmd_typed
/motionbrain/light_result
/motionbrain/light_result_typed
/motionbrain/mission_cmd
/motionbrain/mission_cmd_typed
/motionbrain/mission_state
/motionbrain/mission_state_typed
/motionbrain/routine
/motionbrain/routine_typed
/motionbrain/status
/motionbrain/status_typed
```

## Typed Samples

Representative fields from captured `ros2 topic echo --once` samples:

- `/motionbrain/status_typed`
  - `available: true`
  - `state: IDLE`
  - `armed: false`
  - `moving: false`
  - `faulted: false`
- `/motionbrain/routine_typed`
  - `available: true`
  - `controller_state: IDLE`
  - `dry_run_only: true`
  - `execute_implemented: false`
  - `executor_enabled: false`
  - `executor_mode: skeleton_disabled_by_default`
  - `queue_apply_allowed: false`
  - `executor_state: idle`
  - `sensor_connected: true`
  - `sensor_fresh: true`
  - `teleop_connected: true`
  - `safety_motion_blocked: false`
  - `safety_fault_latched: false`
  - `recovery_action: none`
  - `routine_count: 5`
  - routine names: `inspect`, `open_gripper_check`, `stow`,
    `center_target_dry_run`, `soft_home_reference`
- `/camera/detection_typed`
  - `available: true`
  - `detected: false`
  - `target_type: object`
  - `label: cup`
  - `alignment: LOST`
  - `camera_url: http://192.168.219.111`
  - `reason: no_objects`
- `/joint_states`
  - `base_yaw_joint`
  - `shoulder_pitch_joint`
  - `elbow_pitch_joint`
  - `wrist_pitch_joint`
  - `gripper_joint`
- `/motionbrain/end_effector_pose`
  - `frame_id: world`
  - position approximately `x=0.82`, `y=0.0`, `z=0.09`
  - orientation `w=1.0`
- `/motionbrain/kinematics_typed`
  - `within_joint_limits: true`
  - `radial_reach_m: 0.82`
  - `ik_enabled: false`
- `/motionbrain/control_guard_typed`
  - `ready: true`
  - `reason: ready`
  - `status_fresh: true`
  - `detection_fresh: true`
  - `state: IDLE`
  - `camera_available: true`
  - `alignment: LOST`
- `/motionbrain/mission_state_typed`
  - `state: IDLE`
  - `reason: idle`
  - `guard_ready: true`
  - `guard_reason: ready`
  - `status_fresh: true`
  - `detection_fresh: true`
  - `alignment: LOST`

## Safety Boundary

The capture used read-only status, health, and `ros2 topic echo --once` checks.
`CAPTURE_MISSION_BOUNDARY=0` was used, so no mission command was published. No
ARM, motor, light, gripper, nudge, grasp, or physical routine execute command
was sent.

The raw capture file was kept locally under `.codex/tmp/evidence/` and on the
Pi at:

```text
/home/motionbrain/develop/arduino/motionbrain/docs/evidence/2026-06-12-ros2-routine-typed-systemd.txt
```
