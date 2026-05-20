# MotionBrain Portfolio One-Pager

[한국어 README](README.md) | [English README](README.en.md)

## Summary

MotionBrain is a multi-MCU robotic arm control project built around an ESP32 motion controller, STM32 sensor/teleop layer, ESP32-CAM vision input, and a Mac host-side perception loop. The project focuses on reliable embedded control, safety state management, sensor feedback, and a clean command boundary that can later be bridged into ROS2 and AI-driven planning.

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
- Safe host-triggered `/light` action through the existing MotionBrain command boundary
- Bench simulation commands for hardware-independent validation
- Documentation for pin mapping, message boundaries, and Phase 4 host-side integration

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

The Phase 4 MVP proves the first camera-to-host-to-controller loop before moving to Raspberry Pi or ROS2:

- ESP32-CAM joins the `MotionBrain-AP` network as a camera node.
- Mac host fetches ESP32-CAM `/capture` frames and MotionBrain `/status`.
- OpenCV detects a red target in the camera frame.
- The host only triggers an action when MotionBrain status allows it.
- The demo action uses the safe non-motion `/light?action=toggle` path.

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

## Current Limitations

- Final hardware enclosure and wiring layout are not locked yet.
- Public demo images and videos are not included yet.
- Phase 4 currently uses a Mac host; Raspberry Pi deployment is not complete yet.
- The first vision action is intentionally limited to a safe search-light command, not motor motion.
- Raspberry Pi, ROS2, and AI integration are planned but not complete.
- Teleop sign, deadzone, and speed weights need final tuning after mechanical mounting is fixed.

## Why It Is Relevant

This project demonstrates practical embedded robotics engineering:

- Multi-MCU system decomposition
- Safety-gated motor control
- UART sensor integration
- HTTP and serial command APIs
- State and event observability
- Hardware bench validation
- Forward-compatible architecture for ROS2 and host-side perception

## Next Steps

1. Prepare public demo media showing teleop, safety/status, camera capture, red detection, and host-triggered light action.
2. Harden the host vision demo with capture retry and clearer action modes.
3. Extend camera or host decisions from light-only actions toward alignment or semi-autonomous pick behavior.
4. Finalize wiring, obstacle-safety placement, and teleop tuning.
5. Move the host-side boundary toward Raspberry Pi + ROS2.
