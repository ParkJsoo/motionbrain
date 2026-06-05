# MotionBrain Portfolio One-Pager

[Korean README](README.md) | [English README](README.en.md) | [Korean Portfolio](PORTFOLIO.md)

## Summary

MotionBrain is an embedded robotics portfolio project that integrates an ESP32 motion controller, STM32 sensor/teleop layer, ESP32-CAM vision input, and a Raspberry Pi + ROS2 host bridge into one robotic-arm system.

The project is designed to demonstrate reliable embedded control, not just motor movement. The main engineering signal is the separation between low-level motion execution, safety state, structured sensor feedback, host-side perception, and ROS2 orchestration.

## Engineering Problem

The current arm is a low-cost 5-axis DC motor platform without encoders, force feedback, or reliable absolute joint pose. Claiming fully autonomous grasping on this hardware would be misleading.

MotionBrain therefore uses a constrained architecture:

- The ESP32 owns motor output and the safety boundary.
- The STM32 turns sensors and wired teleop inputs into structured UART frames.
- Serial, HTTP, dashboard, and ROS2-facing commands share the same command semantics.
- Camera and AI workloads run on the Raspberry Pi.
- Physical motion remains safety-gated and operator-confirmed.

## My Role

I designed and implemented:

- ESP32 firmware for 5-axis DC motor control
- `BOOT -> IDLE -> ARMED -> FAULT` safety state machine
- Shared serial/HTTP command dispatch
- Token-gated HTTP state-changing commands
- STM32 `MPU-6050 + HC-SR04 + UART` sensor/teleop firmware
- Deadman handling, frame freshness timeout, and sensor fault latching
- ESP32-CAM capture/stream firmware
- Raspberry Pi dashboard and perception service
- OpenCV-based red target detection and target overlay
- Safety-gated short base nudge actions
- ROS2 Jazzy bridge, typed messages, C++ control guard, and mission supervisor
- Raspberry Pi systemd deployment and health checks
- GitHub Actions quality gates for PlatformIO, Python tests, and ROS2 build/test

## Operator Screens

The final public demo video is still pending. The images below are static documentation captures from the current controller and dashboard path, showing that MotionBrain includes operator-facing and observability surfaces rather than only firmware code.

![MotionBrain Control web console](docs/assets/motionbrain-control-stream.png)

The ESP32-hosted control console provides `STREAM` camera feedback, token-gated state-changing commands, manual motor/joint control, and current system state in one local surface.

![MotionBrain Pi dashboard](docs/assets/motionbrain-dashboard.png)

The Pi dashboard observes status, safety, teleop, events, camera frames, and detection/alignment results. Physical action buttons remain constrained by token and safety-state checks.

## System Architecture

```text
STM32 Sensor / Teleop
  -> UART sensor and teleop frames
  -> ESP32 Motion Controller
  -> Dispatcher + SafetyGate
  -> RobotArm / MotionSequence
  -> TB6612FNG motor drivers
  -> 5-axis DC motor arm

ESP32-CAM
  -> HTTP capture / stream
  -> Raspberry Pi perception service
  -> dashboard target overlay
  -> ROS2 /camera/detection(_typed)

Raspberry Pi ROS2 Host
  -> motionbrain_msgs
  -> motionbrain_ros_bridge
  -> motionbrain_control C++ guard
  -> motionbrain_mission supervisor
  -> typed status, detection, kinematics, guard, and mission topics
```

## Technical Highlights

### Safety-Gated Motion

Motion commands must pass state checks, fault checks, sensor checks, and token checks before reaching motor output. Browser manual commands use a controller-side lease so missed refreshes force a hard stop.

### Multi-MCU Decomposition

The ESP32 is the motion and safety controller. The STM32 is the sensor and wired teleop layer. This keeps manual input, sensor feedback, and host commands behind one embedded safety boundary.

### Vision Without Bypassing Safety

The ESP32-CAM acts as a camera node. The Raspberry Pi runs detection and overlay logic. The dashboard and ROS2 bridge consume the same selected-target payload instead of each opening competing camera loops.

### ROS2 Host Boundary

ROS2 does not replace the embedded controller. It promotes ESP32 status/events/camera detection into typed topics and adds host-side guard and mission-state logic while preserving the ESP32 command boundary.

### Demo-Ready Scope

The reliable demo path is `STREAM`-based manual camera feedback, Pi-hosted dashboard, red-target or known-object overlay, ROS2 typed topics, a safety-gated short nudge, and a token-gated command path to physical hardware. Autonomous grasping remains out of scope for the current hardware state.

## Validation

- ESP32 controller and ESP32-CAM PlatformIO builds pass.
- `TB6612FNG x3` and `M1~M5` motor outputs were physically tested.
- STM32 `MPU-6050 + HC-SR04 + UART` bench path was validated.
- Wired teleop produced real motor output under deadman control and stopped on release.
- ESP32-CAM `/status`, `/capture`, and `/stream` were verified.
- Home Wi-Fi operation was validated across ESP32 controller, ESP32-CAM, and Raspberry Pi.
- The ESP32-hosted `MotionBrain Control` page accepted a runtime token and executed state-changing commands.
- The Pi dashboard showed the camera feed, red target box, and physical nudge behavior.
- The Pi perception service recognized the `cup` target with ESP32-CAM `qvga` / JPEG quality `4` and YOLOv5s.
- Raspberry Pi 4 + Ubuntu 24.04 + ROS2 Jazzy `colcon build/test` passed.
- Health checks passed for `/motionbrain/status_typed`, `/camera/detection_typed`, `/joint_states`, `/motionbrain/kinematics_typed`, `/motionbrain/control_guard_typed`, and `/motionbrain/mission_state_typed`.
- Pi perception service output was verified through ROS2 `/camera/detection_typed`.
- GitHub Actions validates PlatformIO firmware builds and ROS2 workspace build/test.

## Object Detection Status

The Pi object-detection path exists: OpenCV DNN/ONNX backend loading, explicit model/label paths, selected-target JSON, dashboard overlay, and ROS2 typed detection publishing. Model weights are intentionally not committed.

The current reliable known-object bench path is constrained `cup` recognition with ESP32-CAM `qvga` / JPEG quality `4`, YOLOv5s, `--object-target cup`, and a confidence baseline of `0.25`. This path returned `cup` through the Pi dashboard/perception API. Manual camera operation is separated into `STREAM`, while `TRACKED` is used as the slower recognition/confirmation view.

Current honest positioning:

- Implemented: Pi-hosted object-detection pipeline, selected-target contract, ROS2/dashboard integration, constrained `cup` recognition
- Stable demo: red target tracking/overlay, `STREAM` manual camera feedback, cup recognition checks
- Not yet solved: arbitrary-object recognition, marker/object-assisted automatic grasping, and continuous visual servoing without richer feedback

## Current Limitations

- No joint encoders, force feedback, or reliable absolute joint pose
- HC-SR04 is currently part of the teleop/safety layer, not a gripper-mounted range sensor
- ESP32-CAM QVGA input limits general object detection quality
- No general text-prompt object search
- Public photos and videos still need final capture

## Next Steps

1. Capture demo media showing `STREAM` manual camera feedback, the Pi dashboard, and ROS2 typed topics together.
2. Capture red-target or `cup` recognition plus safety-gated short nudge behavior.
3. Design marker- or fixed-known-object-assisted grasping as a separate plan.
4. Improve object detection with a better camera or a validated edge-runtime model.
5. Add range/contact sensing before attempting more autonomous grasp sequences.

## Related Documents

- [README.en.md](README.en.md): project entry point
- [MESSAGE_INTERFACE.md](MESSAGE_INTERFACE.md): command and status boundary
- [docs/DEMO_RUNBOOK.en.md](docs/DEMO_RUNBOOK.en.md): demo capture runbook
- [docs/RASPBERRY_PI_DEPLOYMENT.en.md](docs/RASPBERRY_PI_DEPLOYMENT.en.md): Pi systemd deployment
- [docs/PHYSICAL_AI_OBJECT_DETECTION_PLAN.md](docs/PHYSICAL_AI_OBJECT_DETECTION_PLAN.md): object detection and constrained physical-AI plan
- [docs/EMBEDDED_FIRMWARE_EVIDENCE.md](docs/EMBEDDED_FIRMWARE_EVIDENCE.md): embedded firmware validation evidence
