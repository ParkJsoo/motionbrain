# 2026-06-13 ROS2 Lifecycle Heartbeat Evidence

This note summarizes public-safe text evidence captured from the Raspberry Pi
after adding the MotionBrain lifecycle-style heartbeat interface.

## Runtime

- Host: `motionbrain-pi`
- Pi address used from the Mac: `192.168.219.110`
- OS: Ubuntu 24.04.4 LTS, Linux `6.8.0-1057-raspi`, `aarch64`
- ROS2: `jazzy`
- Git commit during capture: `1f5ec92 Parameterize ROS bridge health sample timeouts`
- Service: `motionbrain-ros-bridge.service`
- Service state: `active (running)`
- Controller: `192.168.219.108`
- ESP32-CAM: `http://192.168.219.109`

## Build And Capture

The Pi repository was fast-forwarded through the lifecycle heartbeat commits,
then the ROS2 workspace was rebuilt:

```bash
colcon build --packages-select \
  motionbrain_msgs motionbrain_control motionbrain_mission \
  motionbrain_ros_bridge motionbrain_description \
  motionbrain_ros2_control_mock
```

Build result:

```text
Summary: 6 packages finished [12min 19s]
```

The bridge service was restarted from the rebuilt workspace. After the health
check timeout helper fix was fast-forwarded to `1f5ec92`, the evidence helper
was run with typed-only compatibility output:

```bash
CAPTURE_COMPAT_JSON=0 \
SAMPLE_TIMEOUT_SECONDS=25 \
MOTIONBRAIN_EVIDENCE_OUTPUT=/tmp/motionbrain_ros2_lifecycle_evidence.txt \
CHECK_SERVICE=1 \
ROS_DISTRO=jazzy \
MOTIONBRAIN_ROS_WS=$HOME/develop/arduino/motionbrain/ros2_ws \
tools/raspi/capture_ros2_evidence.sh
```

Captured result:

```text
Output: /tmp/motionbrain_ros2_lifecycle_evidence.txt
Result: OK
```

The raw text capture is kept locally under
`.codex/tmp/evidence/2026-06-13-ros2-lifecycle.txt`.

## Interface Coverage

The typed lifecycle status message was available in the ROS2 interface
inventory:

```text
motionbrain_msgs/msg/NodeLifecycleStatus
```

The runtime topic inventory included both lifecycle status topics:

```text
/motionbrain/lifecycle
/motionbrain/lifecycle_typed
```

The heartbeat is intentionally a lifecycle-style primary-state heartbeat, not a
full managed `LifecycleNode` conversion. It keeps the current systemd launch and
startup behavior stable while exposing a typed active/error/detail boundary for
monitoring.

## Runtime Coverage

The bridge service was active and running the five expected nodes:

```text
motionbrain_status_node
motionbrain_joint_state_node
motionbrain_kinematics_node
motionbrain_control_guard_node
motionbrain_mission_supervisor
```

The health check passed the lifecycle topic and active publisher check:

```text
OK topic: /motionbrain/lifecycle_typed
OK lifecycle active samples
```

The lifecycle sample captured from `/motionbrain/lifecycle_typed` reported an
active status node:

```text
node_name: motionbrain_status_node
state_id: 3
state_label: active
active: true
error: false
detail: polling status/routine/camera and serving routine command/action boundaries
```

The active sample health check streamed lifecycle messages long enough to verify
all five expected lifecycle publishers.

## Existing Boundary Checks

The same evidence run also confirmed that the existing read-only ROS2 boundary
remained healthy:

```text
OK topic: /motionbrain/status_typed
OK topic: /motionbrain/routine_typed
OK topic: /motionbrain/diagnostics
OK topic: /camera/detection_typed
OK topic: /joint_states
OK topic: /motionbrain/end_effector_pose
OK topic: /motionbrain/kinematics_typed
OK topic: /motionbrain/control_guard_typed
OK topic: /motionbrain/mission_state_typed
OK service: /motionbrain/routine_command
OK action: /motionbrain/guarded_routine
OK routine command service status sample
OK guarded routine action status sample
```

The typed routine readiness sample still reported the disabled executor
boundary:

```text
dry_run_only: true
execute_implemented: false
executor_enabled: false
queue_apply_allowed: false
executor_state: idle
routine_count: 5
```

## Safety Boundary

The lifecycle heartbeat path is read-only. This capture did not publish ARM,
motor, light, gripper, nudge, grasp, or physical routine execution commands.
The routine service and action samples used status-only requests.

A post-capture controller status check showed the physical controller still in
a safe read-only state:

```text
state: IDLE
motorEnabled: false
M1-M5 enabled: false
M1-M5 speed: 0
M1-M5 direction: stopped
light: false
dryRunOnly: true
executeImplemented: false
executor.enabled: false
executor.queueApplyAllowed: false
executor.status.state: idle
executor.status.lastResult: not_requested
```
