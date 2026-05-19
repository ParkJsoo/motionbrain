# MotionBrain Portfolio One-Pager

[Korean README](README.md) | [English README](README.en.md)

## Summary

MotionBrain is a multi-MCU robotic arm control project built around an ESP32 motion controller, STM32 sensor/teleop layer, and a planned ESP32-CAM + host-side robotics stack. The project focuses on reliable embedded control, safety state management, sensor feedback, and a clean command boundary that can later be bridged into ROS2 and AI-driven planning.

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

ESP32-CAM and host-side vision are planned as the next input layer.
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

## Validation So Far

- ESP32 firmware builds with PlatformIO.
- `TB6612FNG x3` and `M1~M5` motor outputs were physically connected and bench-tested.
- STM32 `MPU-6050 + HC-SR04 + UART` sensor stream was bench-validated.
- ESP32 sensor bridge received STM32 sensor packets in bench testing.
- Wired handheld teleop produced real motor output under deadman control.
- Deadman release stopped teleop-controlled motion.
- `teleop.parseErrors=0` and `sensor.parseErrors=0` were observed in the current bench path.

## Current Limitations

- Final hardware enclosure and wiring layout are not locked yet.
- Public demo images and videos are not included yet.
- ESP32-CAM streaming and host-side vision input are still in Phase 4 MVP work.
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

1. Connect ESP32-CAM capture/streaming to a Mac host MVP.
2. Use host-side status and event polling to make safe demo decisions.
3. Route camera or host decisions through existing safe command paths.
4. Finalize wiring, obstacle-safety placement, and teleop tuning.
5. Add public demo media once the physical layout is stable.
