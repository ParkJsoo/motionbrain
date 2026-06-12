# 2026-06-12 ROS2 Routine Action Boundary Evidence

This note summarizes public-safe text evidence captured from the Raspberry Pi
after adding the guarded routine ROS2 action interface and optional
`ros2_control` mock demo package.

## Runtime

- Host: `motionbrain-pi`
- Pi address used from the Mac: `192.168.219.110`
- OS: Ubuntu 24.04.4 LTS, Linux `6.8.0-1057-raspi`, `aarch64`
- ROS2: `jazzy`
- Git commit during capture: `e0194cb Fix guarded routine action evidence matching`
- Feature commits covered by this capture:
  - `16d9ac8 Add guarded routine ROS2 action boundary`
  - `a9a26ea Add optional ros2_control mock demo`
- Service: `motionbrain-ros-bridge.service`
- Service state: `active (running)`
- Controller: `192.168.219.108`
- ESP32-CAM: `http://192.168.219.109`

## Build And Capture

The Pi repository was fast-forwarded through the action and mock demo commits,
then the ROS2 workspace was rebuilt:

```bash
colcon build --packages-select \
  motionbrain_msgs motionbrain_control motionbrain_mission \
  motionbrain_ros_bridge motionbrain_description \
  motionbrain_ros2_control_mock
```

Build result:

```text
Summary: 6 packages finished
```

The bridge service was restarted from the rebuilt workspace. After a script-only
helper fix was fast-forwarded to `e0194cb`, the evidence helper was run with the
action-boundary option:

```bash
CAPTURE_ROUTINE_ACTION_BOUNDARY=1 \
CAPTURE_COMPAT_JSON=0 \
SAMPLE_TIMEOUT_SECONDS=20 \
MOTIONBRAIN_EVIDENCE_OUTPUT=/tmp/motionbrain_ros2_action_boundary_evidence.txt \
CHECK_SERVICE=1 \
ROS_DISTRO=jazzy \
MOTIONBRAIN_ROS_WS=$HOME/develop/arduino/motionbrain/ros2_ws \
tools/raspi/capture_ros2_evidence.sh
```

Captured result:

```text
Output: /tmp/motionbrain_ros2_action_boundary_evidence.txt
Result: OK
```

The raw text capture is kept locally under
`.codex/tmp/evidence/2026-06-12-ros2-action-boundary.txt`.

## Interface Coverage

The package inventory included the new optional mock package:

```text
motionbrain_ros2_control_mock
```

The action interface inventory included:

```text
motionbrain_msgs/action/GuardedRoutine
```

The runtime action inventory included:

```text
/motionbrain/guarded_routine
```

The bridge health check passed the action server and status sample:

```text
OK action: /motionbrain/guarded_routine
OK guarded routine action status sample
```

## Action Boundary Results

Read-only routine status through the action returned success and was forwarded
to the existing ESP32 `GET /routine` boundary:

```text
success: true
action: status
result: status
message: routine status fetched
forwarded: true
Goal finished with status: SUCCEEDED
```

Routine execution through the action was rejected locally by the ROS2 bridge:

```text
success: false
action: run
routine_name: inspect
result: routine_execute_disabled_by_bridge_policy
error: routine_execute_disabled_by_bridge_policy
forwarded: false
Goal finished with status: ABORTED
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

## Mock Control Demo Status

The optional `motionbrain_ros2_control_mock` package was included in the Pi
workspace build and appears in `ros2 pkg list`. It is intentionally not part of
the systemd bridge launch and does not connect to ESP32, STM32, motors, gripper,
or light hardware.

The Pi does not currently have the runtime `ros2_control` packages installed:

```text
missing controller_manager
missing joint_state_broadcaster
missing joint_trajectory_controller
missing ros2_control
missing ros2_controllers
missing hardware_interface
```

Because of that, this capture verifies the package build/install boundary but
does not include a running `controller_manager` mock launch. Running the mock
launch and capturing `ros2 control list_controllers`,
`ros2 control list_hardware_interfaces`, and `/joint_states` remains the next
runtime step after those packages are installed.

## Safety Boundary

The evidence helper did not publish ARM, motor, light, gripper, nudge, grasp, or
physical routine execution commands. The only `run` goal was sent to the ROS2
bridge action server and was rejected locally with `forwarded=false`, so it was
not forwarded to the ESP32 HTTP routine command boundary.

A post-capture controller status check showed the controller still in a safe
read-only state:

```text
state: IDLE
all motor speeds: 0
all motors enabled: false
light: false
faultLatched: false
faultReason: NONE
dryRunOnly: true
executeImplemented: false
executor.enabled: false
executor.queueApplyAllowed: false
executor.status.state: idle
executor.status.lastResult: not_requested
```
