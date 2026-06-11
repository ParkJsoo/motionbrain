# 2026-06-12 ROS2 Routine Command Boundary Evidence

This note summarizes public-safe text evidence captured from the Raspberry Pi
after adding the opt-in routine command/result capture path to the ROS2
evidence helper.

## Runtime

- Host: `motionbrain-pi`
- Pi address used from the Mac: `192.168.219.113`
- OS: Ubuntu 24.04.4 LTS, Linux `6.8.0-1057-raspi`, `aarch64`
- ROS2: `jazzy`
- Git commit: `a475ede Add routine command evidence capture`
- Service: `motionbrain-ros-bridge.service`
- Service state: `active (running)`
- Controller: `192.168.219.110`
- ESP32-CAM: `http://192.168.219.111`

The service was already running from the installed ROS2 workspace. This change
only updated the reusable evidence helper and documentation; no ROS2 rebuild was
required for this script-only follow-up.

## Capture Command

The Pi repository was fast-forwarded to `a475ede`, then the helper was run from
the tracked repo path:

```bash
CAPTURE_ROUTINE_COMMAND_BOUNDARY=1 \
CAPTURE_COMPAT_JSON=0 \
SAMPLE_TIMEOUT_SECONDS=15 \
MOTIONBRAIN_EVIDENCE_OUTPUT=docs/evidence/2026-06-12-ros2-routine-command-boundary.txt \
tools/raspi/capture_ros2_evidence.sh
```

Captured result:

```text
Output: docs/evidence/2026-06-12-ros2-routine-command-boundary.txt
Result: OK
```

The raw capture file is kept locally under `.codex/tmp/evidence/` and on the Pi
as an untracked text file.

## Interface And Topic Coverage

The interface inventory included the routine command boundary messages:

```text
motionbrain_msgs/msg/RoutineCommand
motionbrain_msgs/msg/RoutineResult
motionbrain_msgs/msg/RoutineStatus
```

The topic inventory included:

```text
/motionbrain/routine
/motionbrain/routine_cmd
/motionbrain/routine_cmd_typed
/motionbrain/routine_result
/motionbrain/routine_result_typed
/motionbrain/routine_typed
```

The health check also passed the existing read-only and typed samples:

```text
OK routine diagnostics sample
OK routine typed diagnostics sample
OK control guard typed sample
OK mission state typed sample
```

## Routine Command Results

The helper first published a typed routine status command:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 \
  /motionbrain/routine_cmd_typed \
  motionbrain_msgs/msg/RoutineCommand \
  "{action: status}"
```

The typed result showed that the bridge fetched routine status:

```text
success: true
action: status
result: status
message: routine status fetched
forwarded: true
```

The helper then published a typed routine run request to prove the bridge policy
boundary:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 \
  /motionbrain/routine_cmd_typed \
  motionbrain_msgs/msg/RoutineCommand \
  "{action: run, routine_name: inspect, confirm_code: confirm-inspect}"
```

The typed result showed local rejection by the ROS2 bridge:

```text
success: false
action: run
routine_name: inspect
result: routine_execute_disabled_by_bridge_policy
error: routine_execute_disabled_by_bridge_policy
forwarded: false
```

## Safety Boundary

The evidence helper did not publish ARM, motor, light, gripper, nudge, grasp, or
physical routine execution commands. The only `run` message was sent to the
ROS2 bridge command topic and was rejected locally with `forwarded=false`, so it
was not forwarded to the ESP32 HTTP routine command boundary.

A post-capture controller status check showed the controller still in a safe
read-only state:

```text
state: IDLE
motorEnabled: false
all motor speeds: 0
all motors enabled: false
light: false
blockReason: NONE
faultReason: NONE
```
