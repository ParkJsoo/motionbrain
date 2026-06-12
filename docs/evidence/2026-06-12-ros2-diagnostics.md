# 2026-06-12 ROS2 Diagnostics Evidence

This note summarizes public-safe evidence captured from the Raspberry Pi after
adding the read-only ROS diagnostics topic.

## Runtime

- Host: `motionbrain-pi`
- Pi address used from the Mac: `192.168.219.110`
- OS: Ubuntu 24.04.4 LTS, Linux `6.8.0-1057-raspi`, `aarch64`
- ROS2: `jazzy`
- Git commit: `1136952 Add ROS2 diagnostics topic`
- Service: `motionbrain-ros-bridge.service`
- Service state after rebuild/reload: `active (running)`
- Controller used by the service: `192.168.219.108`
- ESP32-CAM used by the service: `http://192.168.219.109`

## Build And Capture

The Pi repository was fast-forwarded to `1136952`, then the ROS2 workspace was
rebuilt:

```bash
colcon build --packages-select \
  motionbrain_msgs motionbrain_control motionbrain_mission \
  motionbrain_ros_bridge motionbrain_description
```

Build result:

```text
Summary: 5 packages finished
```

The bridge service was reloaded and the evidence helper was run with read-only
bag capture enabled:

```bash
CAPTURE_ROSBAG=1 \
CAPTURE_COMPAT_JSON=0 \
SAMPLE_TIMEOUT_SECONDS=15 \
ROSBAG_DURATION_SECONDS=10 \
MOTIONBRAIN_EVIDENCE_OUTPUT=/tmp/motionbrain_ros2_diagnostics_evidence.txt \
MOTIONBRAIN_ROSBAG_OUTPUT=/tmp/motionbrain_ros2_diagnostics_bag \
tools/raspi/capture_ros2_evidence.sh
```

Captured result:

```text
Output: /tmp/motionbrain_ros2_diagnostics_evidence.txt
ROS2 bag: /tmp/motionbrain_ros2_diagnostics_bag
Result: OK
```

The raw text capture is kept locally under
`.codex/tmp/evidence/2026-06-12-ros2-diagnostics.txt`. The bag was left on the
Pi under `/tmp/motionbrain_ros2_diagnostics_bag`.

## Topic Coverage

The bridge advertised the new topic:

```text
/motionbrain/diagnostics
```

The health check passed the diagnostics sample:

```text
OK diagnostics sample
```

The diagnostics sample included four read-only status groups:

```text
motionbrain/controller
motionbrain/routine_executor
motionbrain/teleop_sensor
motionbrain/camera_perception
```

Observed diagnostic messages:

```text
controller ready
routine executor disabled by policy
teleop and sensor fresh
camera available, target not found
```

The routine readiness sample still reported the disabled executor boundary:

```text
controller_state: IDLE
executor_enabled: false
queue_apply_allowed: false
executor_state: idle
routine_count: 5
```

## Rosbag Evidence

The opt-in bag capture subscribed only to read-only topics, including the new
diagnostics topic:

```text
/motionbrain/status_typed
/motionbrain/routine_typed
/motionbrain/diagnostics
/motionbrain/events_typed
/camera/detection_typed
/joint_states
/motionbrain/end_effector_pose
/motionbrain/kinematics_typed
/motionbrain/control_guard_typed
/motionbrain/mission_state_typed
```

Bag output files:

```text
/tmp/motionbrain_ros2_diagnostics_bag/metadata.yaml
/tmp/motionbrain_ros2_diagnostics_bag/motionbrain_ros2_diagnostics_bag_0.mcap
```

## Safety Boundary

The diagnostics path is read-only. It reuses the latest bridge poll results and
does not publish ARM, motor, light, gripper, nudge, grasp, routine command, or
physical routine execution requests.

A post-capture controller check showed the controller still in a safe read-only
state:

```text
state: IDLE
motorEnabled: false
all motor speeds: 0
all motors enabled: false
light: false
blockReason: NONE
faultReason: NONE
queueApplyAllowed: false
executorEnabled: false
executor state: idle
```
