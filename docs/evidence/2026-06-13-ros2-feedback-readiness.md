# 2026-06-13 Feedback Readiness Mirror Evidence

This note summarizes read-only evidence captured after mirroring the firmware
hardware feedback scaffold into the dashboard, ROS2 typed routine status, ROS
diagnostics, and the Pi evidence helper.

## Runtime

- Host: `motionbrain-pi`
- Pi address used from the Mac: `192.168.219.110`
- Controller address: `192.168.219.108`
- ROS2: `jazzy`
- Git commit: `2f35319 Mirror feedback readiness to dashboard and ROS2`
- Service: `motionbrain-ros-bridge.service`
- Service state during capture: `active (running)`

## Build And Upload

Local validation passed before deployment:

```text
python3.12 -m unittest tests.test_guarded_routine_contract
python3.12 -m unittest tests.test_ros2_workspace_contract
python3.12 -m unittest discover -s tests
PYTHONPATH=ros2_ws/src/motionbrain_ros_bridge python3.12 -m unittest discover -s ros2_ws/src/motionbrain_ros_bridge/test
PYTHONPATH=ros2_ws/src/motionbrain_ros_bridge python3.12 -m py_compile ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/motionbrain_status_node.py
bash -n tools/raspi/check_ros_bridge_health.sh tools/raspi/capture_ros2_evidence.sh
pio run
git diff --check
```

The ESP32 controller was uploaded through `/dev/cu.usbserial-1110`:

```text
pio run -t upload --upload-port /dev/cu.usbserial-1110
========================= [SUCCESS] Took 61.86 seconds =========================
```

The Pi repository was fast-forwarded to `2f35319`, then the ROS2 workspace was
rebuilt for the changed message interface:

```bash
colcon build --packages-select motionbrain_msgs motionbrain_ros_bridge
```

Build result:

```text
Summary: 2 packages finished [9min 49s]
```

## ESP32 Read-Only Smoke

Only read-only HTTP endpoints were sampled after upload.

`GET /status`:

```text
messageType=status
state=IDLE
motorEnabled=false
feedback.selectedClosureTarget=base_yaw_reference
feedback.readyForRoutineExecution=false
feedback.physicalRoutineExecutionAllowed=false
feedback.baseYaw.installed=false
feedback.baseYaw.fault=not_installed
```

`GET /routine`:

```text
messageType=routine_list
state=IDLE
dryRunOnly=true
executeImplemented=false
executor.queueApplyAllowed=false
feedback.selectedClosureTarget=base_yaw_reference
feedback.readyForRoutineExecution=false
feedback.physicalRoutineExecutionAllowed=false
feedback.baseYaw.installed=false
feedback.baseYaw.fault=not_installed
```

The raw JSON samples are kept locally under:

```text
.codex/tmp/evidence/2026-06-13-esp32-status-feedback.json
.codex/tmp/evidence/2026-06-13-esp32-routine-feedback.json
```

## ROS2 Health And Evidence

The Pi health check passed and now validates typed feedback readiness:

```text
OK routine typed feedback readiness sample
OK diagnostics sample
```

The evidence helper was run in read-only mode:

```bash
CAPTURE_COMPAT_JSON=0 \
SAMPLE_TIMEOUT_SECONDS=25 \
MOTIONBRAIN_EVIDENCE_OUTPUT=/tmp/motionbrain_ros2_feedback_evidence.txt \
CHECK_SERVICE=1 \
ROS_DISTRO=jazzy \
MOTIONBRAIN_ROS_WS=$HOME/develop/arduino/motionbrain/ros2_ws \
tools/raspi/capture_ros2_evidence.sh
```

Captured result:

```text
Output: /tmp/motionbrain_ros2_feedback_evidence.txt
Result: OK
```

The raw text capture is kept locally under:

```text
.codex/tmp/evidence/2026-06-13-ros2-feedback-readiness.txt
```

## ROS2 Observations

`/motionbrain/routine_typed` included the feedback mirror:

```text
feedback_selected_target: base_yaw_reference
feedback_ready: false
physical_routine_execution_allowed: false
feedback_block_reason: feedback_required
base_yaw_feedback_installed: false
base_yaw_feedback_available: false
base_yaw_feedback_connected: false
base_yaw_feedback_fresh: false
base_yaw_feedback_referenced: false
base_yaw_feedback_faulted: true
base_yaw_feedback_stop_reason: NOT_INSTALLED
base_yaw_feedback_fault: not_installed
```

`/motionbrain/diagnostics` included the feedback diagnostic:

```text
name: motionbrain/feedback
message: feedback not ready for physical routines
selected_target: base_yaw_reference
feedback_ready: false
physical_routine_execution_allowed: false
block_reason: feedback_required
base_yaw_fault: not_installed
```

## Safety Boundary

This capture did not publish ARM, motor, light, gripper, nudge, grasp, routine
run, or physical routine execution requests. The ESP32 checks were limited to
`GET /status` and `GET /routine`; ROS2 evidence was topic/service/action status
sampling only.
