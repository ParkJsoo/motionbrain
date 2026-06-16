# MotionBrain Portfolio One-Pager

[Korean README](README.md) | [English README](README.en.md) | [Korean Portfolio](PORTFOLIO.md)

## Summary

MotionBrain is an embedded robotics portfolio project that integrates an ESP32 motion controller, STM32 sensor/teleop layer, ESP32-CAM vision input, and a Raspberry Pi + ROS2 host bridge into one robotic-arm system.

The project is designed to demonstrate reliable embedded control, not just motor movement. The main engineering signal is the separation between low-level motion execution, safety state, structured sensor feedback, host-side perception, and ROS2 orchestration.

The stable portfolio snapshot is tagged as `demo-ready-20260608`.

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
- STM32 `MPU-6050 + UART` sensor/teleop firmware and bench-validated HC-SR04 path
- Deadman handling, frame freshness timeout, and sensor fault latching
- ESP32-CAM capture/stream firmware
- Raspberry Pi dashboard and perception service
- OpenCV-based red target detection and target overlay
- Safety-gated short base nudge actions
- ROS2 Jazzy bridge, typed messages, C++ control guard, and mission supervisor
- `ros2_control` mock demo and safe open-loop `SystemInterface` scaffold
- Raspberry Pi systemd deployment and health checks
- GitHub Actions quality gates for PlatformIO, Python tests, and ROS2 build/test

## Operator Screens

The final physical teleoperation video is published as the README GIF/MP4. The images below are separate static evidence from the controller, dashboard, and RViz path, showing that MotionBrain includes operator-facing surfaces, observability, and ROS2 visualization rather than only firmware code.

![MotionBrain Control web console](docs/assets/motionbrain-control-stream.png)

The ESP32-hosted control console provides `STREAM` camera feedback, token-gated state-changing commands, manual motor/joint control, and current system state in one local surface.

![MotionBrain Pi dashboard](docs/assets/motionbrain-dashboard.png)

The Pi dashboard observes status, safety, teleop, events, camera frames, and detection/alignment results. Physical action buttons remain constrained by token and safety-state checks.

![MotionBrain RViz RobotModel](docs/assets/motionbrain-rviz-robotmodel.png)

The RViz view shows the visualization path for live ROS2 topics mirrored from the Pi dashboard, plus `RobotModel` and TF.

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

The public demo is physical teleoperation. Supporting evidence covers `STREAM`-based manual camera feedback, Pi-hosted dashboard, red-target or known-object overlay, ROS2 typed topics, a safety-gated nudge path, and token-gated light/search commands. Autonomous grasping remains out of scope for the current hardware state.

## Validation

- ESP32 controller and ESP32-CAM PlatformIO builds pass.
- `TB6612FNG x3` and `M1~M5` motor outputs were physically tested.
- STM32 `MPU-6050 + UART` teleop and the HC-SR04 bench path were validated.
- Wired teleop produced real motor output under deadman control and stopped on release.
- The final physical teleoperation demo was captured and published as README GIF/MP4 assets.
- ESP32-CAM `/status`, `/capture`, and `/stream` were verified.
- Home Wi-Fi operation was validated across ESP32 controller, ESP32-CAM, and Raspberry Pi.
- The ESP32-hosted `MotionBrain Control` page accepted a runtime token and executed state-changing commands.
- The Pi dashboard verified the camera feed, red target box, and safety-gated nudge path.
- The Pi perception service recognized the `cup` target with ESP32-CAM `qvga` / JPEG quality `4` and YOLOv5s.
- Raspberry Pi 4 + Ubuntu 24.04 + ROS2 Jazzy `colcon build/test` passed.
- Health checks passed for `/motionbrain/status_typed`, `/camera/detection_typed`, `/joint_states`, `/motionbrain/kinematics_typed`, `/motionbrain/control_guard_typed`, and `/motionbrain/mission_state_typed`.
- Pi perception service output was verified through ROS2 `/camera/detection_typed`.
- Mac Docker/noVNC RViz visualized RobotModel/TF and live ROS2 topics mirrored from the Pi dashboard.
- GitHub Actions validates PlatformIO firmware builds, Python tests, and ROS2 workspace build/test.

## Object Detection Status

The Pi object-detection path exists: OpenCV DNN/ONNX backend loading, explicit model/label paths, selected-target JSON, dashboard overlay, and ROS2 typed detection publishing. Model weights are intentionally not committed.

The current reliable known-object bench path is constrained `cup` recognition with ESP32-CAM `qvga` / JPEG quality `4`, YOLOv5s, `--object-target cup`, and a confidence baseline of `0.25`. This path returned `cup` through the Pi dashboard/perception API. Manual camera operation is separated into `STREAM`, while `TRACKED` is used as the slower recognition/confirmation view.

Current honest positioning:

- Implemented: Pi-hosted object-detection pipeline, selected-target contract, ROS2/dashboard integration, constrained `cup` recognition
- Stable demo: red target tracking/overlay, `STREAM` manual camera feedback, cup recognition checks
- Not yet solved: arbitrary-object recognition, marker/object-assisted automatic grasping, and continuous visual servoing without richer feedback

## Current Limitations

- No joint encoders, force feedback, or reliable absolute joint pose
- HC-SR04 is removed for the final physical demo, with range telemetry handled as disabled/nonblocking
- ESP32-CAM QVGA input limits general object detection quality
- No general text-prompt object search
- The README physical teleop video and non-motion screenshots/evidence are captured; additional SearchLight or object-correction video should be captured only for a specific follow-up goal

## Next Steps

1. Use the `demo-ready-20260608` snapshot and README GIF/MP4 as the reference links for applications.
2. Tune the emphasis per role: embedded safety, multi-MCU teleop, Pi perception/dashboard, or ROS2 bridge.
3. Design marker- or fixed-known-object-assisted grasping as a separate plan.
4. Revisit autonomy only after adding a better camera, range/contact sensing, or a validated edge runtime.
5. Do not expand arbitrary-object recognition or autonomous grasping claims without new hardware validation.

## Related Documents

- [README.en.md](README.en.md): project entry point
- [README.md](README.md): Korean project entry point
- [ROBOTICS_SYSTEM_READINESS.md](ROBOTICS_SYSTEM_READINESS.md): ROS2 and hardware-boundary summary for robotics system roles
- [EMBEDDED_BRINGUP.md](EMBEDDED_BRINGUP.md): STM32/ESP32 bring-up and measurement checklist
- [OPERATIONS.md](OPERATIONS.md): Pi/systemd/health-check operations notes
