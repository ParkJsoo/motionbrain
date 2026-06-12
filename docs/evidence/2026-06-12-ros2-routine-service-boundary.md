# 2026-06-12 ROS2 Routine Service Boundary Evidence

This note summarizes public-safe evidence captured from the Raspberry Pi after
adding the guarded routine request/response service boundary.

## Runtime

- Host: `motionbrain-pi`
- Pi address used from the Mac: `192.168.219.110`
- OS: Ubuntu 24.04.4 LTS, Linux `6.8.0-1057-raspi`, `aarch64`
- ROS2: `jazzy`
- Git commit: `cbbeef5 Add guarded routine ROS2 service boundary`
- Service: `motionbrain-ros-bridge.service`
- Service state after rebuild/restart: `active (running)`
- Controller used by the service: `192.168.219.108`
- ESP32-CAM used by the service: `http://192.168.219.109`

## Build And Capture

The Pi repository was fast-forwarded to `cbbeef5`, then the ROS2 workspace was
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

The bridge service was restarted and the evidence helper was run with both the
service-boundary and read-only bag options:

```bash
CAPTURE_ROUTINE_SERVICE_BOUNDARY=1 \
CAPTURE_ROSBAG=1 \
CAPTURE_COMPAT_JSON=0 \
SAMPLE_TIMEOUT_SECONDS=15 \
ROSBAG_DURATION_SECONDS=10 \
MOTIONBRAIN_EVIDENCE_OUTPUT=/tmp/motionbrain_ros2_service_boundary_evidence.txt \
MOTIONBRAIN_ROSBAG_OUTPUT=/tmp/motionbrain_ros2_service_boundary_bag \
tools/raspi/capture_ros2_evidence.sh
```

Captured result:

```text
Output: /tmp/motionbrain_ros2_service_boundary_evidence.txt
ROS2 bag: /tmp/motionbrain_ros2_service_boundary_bag
Result: OK
```

The raw text capture is kept locally under
`.codex/tmp/evidence/2026-06-12-ros2-routine-service-boundary.txt`. The bag was
left on the Pi under `/tmp/motionbrain_ros2_service_boundary_bag`.

## Interface Coverage

The interface inventory included the new service:

```text
motionbrain_msgs/srv/GuardedRoutineCommand
```

The service inventory included:

```text
/motionbrain/routine_command
```

The bridge health check passed the new service sample:

```text
OK routine command service status sample
```

## Service Boundary Results

Read-only routine status through the service returned success and was forwarded
to the existing ESP32 `GET /routine` boundary:

```text
success=True
action='status'
result='status'
message='routine status fetched'
forwarded=True
```

Routine execution through the service was rejected locally by the ROS2 bridge:

```text
success=False
action='run'
routine_name='inspect'
result='routine_execute_disabled_by_bridge_policy'
forwarded=False
```

The typed routine readiness sample still reported the disabled executor
boundary:

```text
executor_enabled: false
queue_apply_allowed: false
executor_state: idle
routine_count: 5
```

## Rosbag Evidence

The opt-in bag capture subscribed only to read-only topics:

```text
/motionbrain/status_typed
/motionbrain/routine_typed
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
/tmp/motionbrain_ros2_service_boundary_bag/metadata.yaml
/tmp/motionbrain_ros2_service_boundary_bag/motionbrain_ros2_service_boundary_bag_0.mcap
```

## Safety Boundary

The capture did not publish ARM, motor, light, gripper, nudge, grasp, or
physical routine execution commands. The only `run` request was sent to
`/motionbrain/routine_command` and was rejected locally with `forwarded=False`.

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
