# 2026-05-30 ROS2 Typed Systemd Evidence

This note summarizes the public-safe text evidence captured from the Raspberry
Pi after the typed ROS2 interface cleanup landed.

## Runtime

- Host: `motionbrain-pi`
- OS: Ubuntu 24.04.4 LTS, Linux `6.8.0-1056-raspi`, `aarch64`
- ROS2: `jazzy`
- Git commit: `2874df7 Use typed ROS2 guard and mission topics`
- Service: `motionbrain-ros-bridge.service`
- Service state: `active (running)`
- Launch IPs used by systemd:
  - controller: `192.168.219.110`
  - ESP32-CAM: `http://192.168.219.113`

The service cgroup contained:

- `motionbrain_status_node`
- `motionbrain_joint_state_node`
- `motionbrain_kinematics_node`
- `motionbrain_control_guard_node`
- `motionbrain_mission_supervisor`

## Health Check

Command:

```bash
CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh
```

Captured result:

```text
OK service active: motionbrain-ros-bridge.service
OK topic: /motionbrain/status_typed
OK topic: /camera/detection_typed
OK topic: /joint_states
OK topic: /motionbrain/end_effector_pose
OK topic: /motionbrain/kinematics_typed
OK topic: /motionbrain/control_guard_typed
OK topic: /motionbrain/mission_state_typed
OK status typed sample
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
/motionbrain/status
/motionbrain/status_typed
/parameter_events
/rosout
```

## Typed Samples

Representative fields from captured `ros2 topic echo --once` samples:

- `/motionbrain/status_typed`
  - `available: true`
  - `state: IDLE`
  - `armed: false`
  - `moving: false`
  - `faulted: false`
  - joint angles all `0.0`
- `/camera/detection_typed`
  - `available: true`
  - `color: red`
  - `alignment: LOST`
  - `width: 320`
  - `height: 240`
  - `camera_url: http://192.168.219.113`
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

The raw capture file was kept on the Pi at:

```text
/tmp/motionbrain_ros2_typed_evidence_20260530.txt
```

## TF Text Evidence

`motionbrain_description` was launched without RViz to verify the URDF/TF path
in text form:

```bash
ros2 launch motionbrain_description display.launch.py \
  start_joint_state_bridge:=false \
  use_rviz:=false
```

Captured topics:

```text
/tf
/tf_static
```

Representative `/tf_static` frames:

```text
world -> base_link
base_link -> camera_link
```

Representative `/tf` frames:

```text
base_link -> shoulder_link
shoulder_link -> upper_arm_link
upper_arm_link -> forearm_link
forearm_link -> wrist_link
wrist_link -> gripper_link
```

Launch log:

```text
robot_state_publisher: Robot initialized
```

The raw TF capture was kept on the Pi at:

```text
/tmp/motionbrain_tf_evidence_20260530.txt
```

## Typed Mission Command Boundary

The typed mission command path was checked without publishing `confirm`, so no
physical actuator command was triggered.

Command topic:

```text
/motionbrain/mission_cmd_typed
```

Published `start`:

```text
motionbrain_msgs.msg.MissionCommand(command='start')
```

Observed `/motionbrain/mission_state_typed`:

```text
state: WAIT_DETECTION
reason: target_not_detected
next_step: wait_for_detection
guard_ready: true
status_fresh: true
detection_fresh: true
alignment: LOST
```

Published `reset`:

```text
motionbrain_msgs.msg.MissionCommand(command='reset')
```

Observed `/motionbrain/mission_state_typed`:

```text
state: IDLE
reason: reset
next_step: none
guard_ready: true
status_fresh: true
detection_fresh: true
alignment: LOST
```

The raw mission command capture was kept on the Pi at:

```text
/tmp/motionbrain_mission_cmd_evidence_20260530.txt
```
