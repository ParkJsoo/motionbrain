# Hardware Feedback Gap Before Physical Routine Execution

This document defines the hardware feedback gap that must be closed before
MotionBrain can treat guarded routine `run` or `execute` as a physical motion
feature.

The current repository already supports guarded routine dry-run, status,
preflight, service/action boundaries, diagnostics, event logs, and a disabled
executor skeleton. That is intentionally not the same as physical autonomous
routine execution.

## Current Boundary

The current physical execution boundary remains closed:

- `GET /routine` reports `dryRunOnly=true`.
- The executor policy reports `executeImplemented=false`.
- The materialization gate reports `queueApplyAllowed=false`.
- ROS2 routine topics, service, and action reject `run` and `execute` locally
  with `routine_execute_disabled_by_bridge_policy`.
- `soft_home_reference` is an operator-confirmed software reference routine,
  not encoder-grade homing.
- No routine executor code is allowed to call `MotionSequence::addCommand`,
  `MotionSequence::addBaseAngleCommand`, or `MotionSequence::run` until this
  gap is explicitly closed.

Manual teleoperation and guarded routine execution are different safety cases.
Manual teleoperation is operator-held, deadman-bounded, and immediately visible
to the operator. A routine executor can continue through multiple steps unless
the software proves that feedback, stop conditions, and recovery behavior are
fresh and reliable.

## Current Feedback Reality

MotionBrain has useful safety and status signals, but they do not yet prove
joint pose or contact state well enough for physical routine execution.

| Area | Current state | Gap |
| --- | --- | --- |
| Shoulder, elbow, wrist, gripper position | No encoder-grade joint feedback | Timed pulses cannot prove final pose, stall, overshoot, or drift |
| Base yaw | Relative angle controller integrates gyro rate and stops on timeout/no-feedback | Not an absolute or indexed joint reference |
| Homing/reference | `soft_home_reference` is operator-confirmed and records no absolute pose | No limit switch, index pulse, or hard reference |
| Motor load/contact | No current sensing, force sensing, or contact switch feedback | Cannot prove stall, collision, or gripper contact |
| Sensor freshness | STM32 teleop/safety freshness is reported | No per-joint feedback freshness exists |
| Vision-to-motion | Detection and alignment are available as perception input | No calibrated hand-eye/depth contract that proves target pose for grasping |

These gaps do not invalidate the existing demo. They define why routine
execution remains disabled.

## Selected First Closure Target

The first feedback upgrade should close exactly one physical feedback gap:

```text
base_yaw_reference
```

Reason:

- Base yaw already has the most complete partial control path through
  `AngleController`.
- A single-axis reference is easier to validate than all five joints at once.
- Base yaw has clear failure modes: stale feedback, no progress, overshoot,
  timeout, and unreferenced startup.
- It can be reflected cleanly through HTTP status, diagnostics, ROS2 topics,
  and event evidence before any multi-step routine executor is enabled.

The implementation can use the hardware that fits the build best, but the
software contract should treat it as a base yaw reference/position feedback
source. Acceptable physical approaches include an indexed encoder, an absolute
magnetic angle sensor, or a dedicated home/index switch plus bounded relative
feedback. The repo should not claim closed-loop base routine execution until
one concrete implementation is wired, reported, tested, and evidenced.

Optional future hardware candidate:

```text
GPIO36 active-low base index/reference input
```

The repository has an optional, disabled-by-default adapter for a read-only
digital index/reference input on ESP32 `GPIO36`. No GPIO36 reference sensor is
installed in the current bench setup. If a future build intentionally adds one,
the expected electrical path is an external pull-up to 3V3 and an active-low
sensor or switch that pulls the pin to GND at the base reference. Possible
future parts include a base-mounted magnet with a digital Hall/index output, or
a simple normally-open switch/reed switch for bench validation. `GPIO36` is
input-only and has no internal pull-up, so the pull-up must be external. This
would be an index reference, not a continuous absolute encoder.

Firmware defaults stay conservative:

- `MOTIONBRAIN_BASE_YAW_REFERENCE_ENABLED=0`
- `MOTIONBRAIN_BASE_YAW_REFERENCE_PIN=36`
- `MOTIONBRAIN_BASE_YAW_REFERENCE_ACTIVE_LOW=1`
- `MOTIONBRAIN_BASE_YAW_REFERENCE_PHYSICAL_ROUTINE_ALLOWED=0`

With the input disabled, which is the current real setup, status remains
`not_installed`. If the optional input is enabled for future bench work, the
adapter may report `installed`, `connected`, `fresh`, `signalActive`,
`referenced`, and `hardwareReady`; physical routine execution still remains
disabled unless the separate physical routine gate is explicitly opened in a
later evidence-backed change.

## Required Telemetry Contract

Before physical routine execution can be considered, the chosen feedback path
must be visible as structured telemetry. The exact field names can change when
implemented, but the status contract must include these concepts:

```json
{
  "feedback": {
    "schemaVersion": "feedback.v0",
    "selectedClosureTarget": "base_yaw_reference",
    "physicalRoutineExecutionAllowed": false,
    "readyForRoutineExecution": false,
    "blockReason": "feedback_required",
    "baseYaw": {
      "installed": false,
      "available": false,
      "connected": false,
      "fresh": false,
      "referenced": false,
      "faulted": true,
      "readyForRoutineExecution": false,
      "ageMs": 0,
      "positionDeg": 0.0,
      "velocityDps": 0.0,
      "lastStopReason": "NOT_INSTALLED",
      "fault": "not_installed"
    }
  }
}
```

The same state must be reflected in:

- `GET /status`
- `GET /routine`
- routine preflight responses
- event log entries
- dashboard readiness UI (`Routine Readiness` feedback tile)
- ROS2 diagnostics or typed status (`motionbrain/feedback`,
  `RoutineStatus.feedback_ready`, `RoutineStatus.base_yaw_feedback_fault`,
  `RoutineStatus.base_yaw_feedback_hardware_ready`,
  `RoutineStatus.base_yaw_feedback_signal_active`,
  `RoutineStatus.base_yaw_feedback_pin`, and
  `RoutineStatus.base_yaw_feedback_active_low`)
- evidence helper output (`/motionbrain/routine` and typed feedback capture)

## Current Firmware Scaffold

The current firmware scaffold implements the first read-only status layer:

- `src/control/hardware_feedback.h`
- `src/control/hardware_feedback.cpp`

It reports the selected closure target as `base_yaw_reference` and exposes the
current state as `not_installed`. This is intentionally conservative:

```json
{
  "feedback": {
    "schemaVersion": "feedback.v0",
    "selectedClosureTarget": "base_yaw_reference",
    "physicalRoutineExecutionAllowed": false,
    "readyForRoutineExecution": false,
    "blockReason": "feedback_required",
    "baseYaw": {
      "installed": false,
      "available": false,
      "connected": false,
      "fresh": false,
      "referenced": false,
      "faulted": true,
      "readyForRoutineExecution": false,
      "lastStopReason": "NOT_INSTALLED",
      "fault": "not_installed"
    }
  }
}
```

`GET /status`, `GET /routine`, and routine command responses expose this
read-only object. Motion-step routines now include feedback readiness in
`executePreflight`; if all earlier gates pass but `base_yaw_reference` is not
ready, the routine is blocked with `feedback_required` and the event log uses
`ROUTINE_FEEDBACK_BLOCK`.

The default scaffold does not read real feedback hardware yet and does not
enable the executor.

The current firmware now also includes a disabled-by-default read-only adapter
for the selected GPIO36 active-low index path:

- `HardwareFeedback::initBaseYawReference()`
- `HardwareFeedback::updateBaseYawReference()`
- default macro gate:
  `MOTIONBRAIN_BASE_YAW_REFERENCE_ENABLED=0`
- physical routine gate:
  `MOTIONBRAIN_BASE_YAW_REFERENCE_PHYSICAL_ROUTINE_ALLOWED=0`

When enabled for bench validation, the adapter debounces the digital input,
marks the axis `referenced` after the index signal is observed, reports
`UNREFERENCED` until that happens, reports `STALE` if the sampling loop stops
updating, and keeps `readyForRoutineExecution=false` while
`MOTIONBRAIN_BASE_YAW_REFERENCE_PHYSICAL_ROUTINE_ALLOWED=0`.
The wiring note is mirrored in `PIN_MAP.md`.

Read-only bench evidence should be captured with
`tools/raspi/capture_base_yaw_reference_evidence.sh`. The helper records ESP32
`/status`, ESP32 `/routine`, dashboard `/api/status`, ROS2 health, and ROS2
readiness evidence without sending actuator or routine commands.

## Mandatory Blocks

Routine `run` or `execute` must remain blocked if any of these are true:

- Feedback hardware is not installed.
- Feedback is disconnected.
- Feedback is stale.
- Feedback is unreferenced after boot.
- Feedback reports a fault.
- The selected axis is outside configured bounds.
- The selected axis reports no progress after motion starts.
- The selected axis overshoots the command envelope.
- A motion step times out before reaching its stop condition.
- The operator has not confirmed the routine-specific confirmation phrase.
- The controller is not `ARMED`.
- Safety monitor reports motion blocked or a latched fault.
- A previous sequence or routine executor state is still active.

The preferred block reason for routine preflight is:

```text
feedback_required
```

The preferred event code is:

```text
ROUTINE_FEEDBACK_BLOCK
```

## Evidence Required Before Enabling Execution

A future patch may only change `executeImplemented` or `queueApplyAllowed` after
all of this evidence exists:

1. Firmware tests prove stale, disconnected, unreferenced, faulted, timeout, and
   no-progress feedback all block routine execution.
2. `GET /status` shows the feedback path when healthy and when faulted.
3. `GET /routine` shows routine preflight blocked by feedback when feedback is
   not ready.
4. The dashboard shows feedback readiness without sending a command.
5. ROS2 diagnostics or typed status mirrors the feedback readiness state.
6. The evidence helper captures the feedback state and the blocked routine
   result without publishing physical motion commands.
7. A bench log shows stop-after-step verification for the selected axis.
8. The user explicitly approves a physical routine execution test.

Until then, physical routine execution remains disabled by design.

## Non-Goals

The first closure target does not enable:

- autonomous grasping
- arbitrary object manipulation
- multi-joint closed-loop motion
- force-controlled contact
- hard-stop seeking
- blind timed-pulse routines
- perception-only motion approval

Those require additional feedback and evidence after the base yaw reference gap
is closed.

## Implementation Order

Recommended next implementation sequence:

1. Add a firmware-side read-only feedback status model for
   `base_yaw_reference`. Done for the `not_installed` scaffold.
2. Publish that model through `GET /status` and `GET /routine`. Done for the
   read-only scaffold.
3. Add block reasons and events for stale, disconnected, unreferenced, timeout,
   no-progress, and faulted feedback. Done for the enum/contract surface;
   hardware-backed transitions remain future work.
4. Mirror feedback readiness into dashboard and ROS2 diagnostics. Done for the
   `not_installed` scaffold; see
   `docs/evidence/2026-06-13-ros2-feedback-readiness.md`.
5. Extend tests so routine execution remains blocked while feedback is not
   ready. Done for the scaffold contract.
6. Capture read-only evidence on the Pi. Done for the scaffold contract.
7. If a GPIO36 reference sensor/switch is intentionally added later, wire the
   active-low index/reference input and capture bench evidence for inactive,
   active/reference, unreferenced, stale, and reboot states. Skip this step
   while no such sensor exists.
8. Only after explicit approval, perform a short single-axis physical validation
   with stop-after-step status verification.

The executor should stay disabled until step 8 has explicit approval and
hardware-backed feedback evidence.
