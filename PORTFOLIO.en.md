# MotionBrain Portfolio One-Pager

[한국어 README](README.md) | [English README](README.en.md)

## Summary

MotionBrain is a multi-MCU robotic arm control project built around an ESP32 motion controller, STM32 sensor/teleop layer, ESP32-CAM vision input, Mac host-side perception, and a Raspberry Pi ROS2 bridge path validated on real hardware. The project focuses on reliable embedded control, safety state management, sensor feedback, and a clean command boundary that can be exposed to ROS2 without rewriting the embedded firmware.

## Engineering Problem

The goal is not just to move motors. The project treats the robot arm as a layered control system:

- Low-level motor output must be isolated from unsafe commands.
- Sensor and teleop inputs must be handled as structured feedback, not ad hoc serial text.
- Host-side control should be able to consume status and events without depending on internal firmware details.
- The system should evolve from bench hardware to camera-driven and ROS2-driven demos without rewriting the command model.

## My Role

I designed and implemented the embedded control structure, including:

- ESP32 motor control for a 5-axis robotic arm
- Safety state machine and fault handling
- Unified serial and HTTP command dispatch
- STM32 UART sensor bridge integration
- Safety monitor for sensor-based blocking and latching
- Wired handheld teleop path with deadman and frame freshness timeout
- ESP32-CAM capture/stream firmware
- Mac host-side OpenCV red-target detection MVP
- Trusted Home Wi-Fi workflow for the controller and ESP32-CAM
- Phone-browser manual control through the ESP32-hosted `MotionBrain Control` page
- Safe host-triggered `/light` action through the existing MotionBrain command boundary
- Raspberry Pi 4 ROS2 Jazzy validation for status, events, ESP32-CAM detection, and token-gated light command forwarding
- Bench simulation commands for hardware-independent validation
- Documentation for pin mapping, message boundaries, Phase 4 host-side integration, and Raspberry Pi ROS2 bring-up

## System Architecture

```text
STM32 Sensor / Teleop Layer
  -> UART JSON sensor and teleop frames
  -> ESP32 Motion Controller
  -> Dispatcher + SafetyGate
  -> RobotArm / MotionSequence
  -> TB6612FNG motor drivers
  -> 5-axis DC motor robotic arm

ESP32-CAM Vision Node
  -> HTTP /status, /capture, /stream
  -> Mac host OpenCV target detection
  -> MotionBrain /status safety check
  -> safe /light demo action
  -> opt-in base nudge alignment action

Phone / Browser Operator
  -> ESP32-hosted MotionBrain Control
  -> runtime command token prompt
  -> manual ARM / STOP / joint commands

Mac Ops Dashboard
  -> status, events, camera, detection, action log
  -> token-gated one-shot vision nudge

Raspberry Pi ROS2 Host
  -> motionbrain_ros_bridge
  -> /motionbrain/status, /motionbrain/events
  -> /camera/detection
  -> /motionbrain/light_cmd, /motionbrain/light_result
  -> token-gated /light command
  -> real SearchLight output
```

## Technical Highlights

### Safety State Machine

The ESP32 firmware uses explicit system states:

```text
BOOT -> IDLE -> ARMED -> FAULT
```

Motion commands are accepted only when they pass state and safety checks. Fault conditions can latch the system into `FAULT`, forcing the operator to stop and recover intentionally.

### Unified Command Boundary

Serial and HTTP inputs are routed through a common command path instead of separate behavior branches. This reduces drift between local bench operation, web control, and future host/ROS2 control.

State-changing HTTP commands require the MotionBrain command header and can be protected with a provisioned command token. The token is entered at runtime on the ESP32-hosted control page and kept only in the current browser page memory.

Current control routes include:

- `POST /command`
- `POST /motor`
- `POST /joint`
- `POST /base`
- `POST /sequence`
- `POST /light`
- `GET /status`
- `GET /events`

### STM32 Sensor and Teleop Layer

The STM32 side handles IMU/range sensing and wired handheld teleop experiments. The ESP32 receives structured UART frames and converts them into safe motion intent.

Implemented teleop behavior includes:

- Deadman hold-to-move control
- Frame freshness timeout
- Neutral reset on new deadman session
- `reach`, `lift`, `twist`, and gripper inputs
- Parse-error counters and status reporting

### Bench Simulation

The firmware includes `sensor sim ...` commands so safety and status behavior can be tested without the full sensor stack connected.

Examples:

```text
sensor sim healthy
sensor sim obstacle 10
sensor sim vibration 9
sensor sim stale
sensor sim off
```

### ESP32-CAM Host Vision MVP

The Phase 4 MVP proves the first camera-to-host-to-controller loop before moving the host role onto Raspberry Pi:

- ESP32-CAM joins the `MotionBrain-AP` network as a camera node.
- For normal bench work, the ESP32 controller and ESP32-CAM can also join a trusted Home Wi-Fi LAN to avoid switching the Mac between networks.
- Mac host fetches ESP32-CAM `/capture` frames and MotionBrain `/status`.
- OpenCV detects a red target in the camera frame.
- The host computes target centroid, normalized horizontal offset, and an alignment decision.
- The host only triggers an action when MotionBrain status allows it.
- The default demo action uses the safe non-motion `/light?action=toggle` path.
- Base alignment is opt-in. On the current handheld-teleop hardware it uses a short safety-gated base nudge with an immediate stop; closed-loop `/base?action=angle` is kept for future base-mounted gyro or encoder feedback.

### ROS2 Bridge Path

The repository includes a ROS2 bridge package validated on Raspberry Pi 4 with Ubuntu 24.04 and ROS2 Jazzy:

- `motionbrain_ros_bridge` polls ESP32 `GET /status` and publishes `/motionbrain/status`.
- `motionbrain_msgs` promotes stable status, event, detection, and light command/result fields into typed ROS2 messages.
- The bridge also publishes typed topics such as `/motionbrain/status_typed`, `/motionbrain/events_typed`, `/camera/detection_typed`, and `/motionbrain/light_result_typed`.
- It polls ESP32 `GET /events?limit=N` and publishes `/motionbrain/events`.
- It fetches ESP32-CAM `/capture`, runs color-target detection, and publishes `/camera/detection`.
- It subscribes to `/motionbrain/light_cmd` and forwards safe `on`, `off`, or `toggle` commands to the token-gated ESP32 `/light` endpoint.
- It also accepts `/motionbrain/light_cmd_typed` for typed command-path validation.
- The Home Wi-Fi launch path was run with IP fallback when `.local` name resolution was unavailable on the Pi.
- The command-channel test toggled the real search light and published `/motionbrain/light_result`.

This keeps ROS2 integration at the host boundary while preserving the existing embedded command model.

### Operator Interfaces

The project separates manual control from observability:

- `MotionBrain Control`, served directly by the ESP32, is the primary manual control surface and has been verified from a phone browser.
- The local Mac ops dashboard is used for status, events, camera/detection, action logs, and a gated one-shot vision nudge.
- Command tokens are provisioned on-device and entered only at runtime; real Wi-Fi credentials and tokens are not committed to the repository.

## Validation So Far

- ESP32 firmware builds with PlatformIO.
- `TB6612FNG x3` and `M1~M5` motor outputs were physically connected and bench-tested.
- STM32 `MPU-6050 + HC-SR04 + UART` sensor stream was bench-validated.
- ESP32 sensor bridge received STM32 sensor packets in bench testing.
- Wired handheld teleop produced real motor output under deadman control.
- Deadman release stopped teleop-controlled motion.
- `teleop.parseErrors=0` and `sensor.parseErrors=0` were observed in the current bench path.
- ESP32-CAM firmware was uploaded and joined `MotionBrain-AP` at `192.168.4.2`.
- ESP32-CAM `/status`, `/capture`, and `/stream` were verified from a phone viewer.
- Mac host fetched MotionBrain `/status` and ESP32-CAM `/capture` in the same loop.
- Host action MVP succeeded with `ACTION light.toggle success=True`.
- OpenCV red target detection succeeded with `red_ratio=0.254` in dry-run.
- Red target action test toggled the real search light; removing the target produced `detected=N red_ratio=0.000`.
- Vision alignment now exposes centroid, horizontal offset, `LEFT|CENTER|RIGHT|LOST` decisions, and command suggestions in the host loop, dashboard, and ROS2 detection payload.
- Vision alignment nudge mode was physically validated in both directions: right target -> `base.right`, left target -> `base.left`, each with a 250 ms / 25% base nudge and confirmed stop.
- The dashboard includes a token-gated one-shot nudge control that revalidates the current camera frame and controller status server-side before motion.
- Home Wi-Fi operation was verified with the ESP32 controller and ESP32-CAM on the same trusted LAN.
- Phone-browser control was verified on 2026-05-25: token prompt appeared, the token was accepted, and commands executed.
- Raspberry Pi 4 ROS2 host validation was completed on 2026-05-26:
  - Ubuntu Server 24.04 arm64 and ROS2 Jazzy on Raspberry Pi 4
  - `motionbrain_ros_bridge` built successfully with `colcon`
  - `/motionbrain/status`, `/motionbrain/events`, and `/camera/detection` published real ESP32/ESP32-CAM data
  - `/motionbrain/light_cmd` forwarded through the token-gated ESP32 `/light` endpoint
  - `/motionbrain/light_result` returned the ESP32 command result
  - the real search light turned on from a ROS2 command
- Typed ROS2 message validation was completed on 2026-05-27:
  - `motionbrain_msgs` and `motionbrain_ros_bridge` built successfully on the Pi
  - `ros2 interface show` and `ros2 interface list` confirmed all five custom messages
  - `/motionbrain/status_typed` returned real controller status
  - `/camera/detection_typed` returned real ESP32-CAM detection data
- A first-pass URDF/TF/joint-state path was validated on the Raspberry Pi:
  - `motionbrain_description` provides a 5-axis arm URDF, display launch file, and RViz config
  - `motionbrain_joint_state_node` maps `/motionbrain/status_typed` into `/joint_states`
  - `robot_state_publisher` publishes `/tf` and `/tf_static` from the MotionBrain URDF
- CI now runs synthetic host-side vision alignment tests alongside ESP32 and ESP32-CAM PlatformIO builds.

## Current Limitations

- Final hardware enclosure and wiring layout are not locked yet.
- Public demo images and videos are not included yet.
- The URDF/TF/joint-state path is validated on the Raspberry Pi, but public RViz evidence capture is still pending.
- Vision-based base alignment is implemented as an explicit opt-in nudge step. It is validated for left/right correction nudges, but not yet for full pick behavior.
- AI planning integration is planned but not complete.
- Teleop sign, deadzone, and speed weights need final tuning after mechanical mounting is fixed.

## Why It Is Relevant

This project demonstrates practical embedded robotics engineering:

- Multi-MCU system decomposition
- Safety-gated motor control
- UART sensor integration
- HTTP and serial command APIs
- State and event observability
- Hardware bench validation
- ROS2 host bridge validation that preserves embedded command boundaries

## Next Steps

1. Capture public demo media showing teleop, safety/status, camera capture, red detection, ROS2 topic output, and ROS2-triggered light action.
2. Capture demo media showing red target localization and left/right base nudge alignment.
3. Capture public RViz/TF evidence for the validated URDF, TF, and `joint_states` path.
4. Extend camera or host decisions from alignment nudges toward semi-autonomous pick behavior.
5. Finalize wiring, obstacle-safety placement, and teleop tuning.

The planned public capture sequence is tracked in
[docs/DEMO_RUNBOOK.en.md](docs/DEMO_RUNBOOK.en.md), with supporting system
diagrams in [docs/ARCHITECTURE_DIAGRAMS.en.md](docs/ARCHITECTURE_DIAGRAMS.en.md).
