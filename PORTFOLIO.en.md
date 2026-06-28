# MotionBrain Portfolio One-Pager

[Korean README](README.md) | [English README](README.en.md) | [Korean Portfolio](PORTFOLIO.md)

## Summary

MotionBrain is an embedded robotics system project that integrates an ESP32 motion controller, STM32 sensor/teleop layer, ESP32-CAM vision input, and a Raspberry Pi + ROS2 host bridge into one robotic-arm system.

The project is designed to demonstrate reliable embedded control, not just motor movement. The main engineering signal is the separation between low-level motion execution, safety state, structured sensor feedback, host-side perception, and ROS2 orchestration.

The physical teleoperation demo media snapshot is tagged as `demo-ready-20260608`.

## Screening Signals

- Real hardware: ESP32 motion controller, STM32 wired teleop layer, ESP32-CAM,
  and Raspberry Pi were integrated in an actual networked robot stack.
- Safety boundary: state changes and physical output must pass token checks,
  state-machine checks, `SafetyGate`, deadman handling, and freshness timeouts.
- ROS2 systems work: typed messages, ROS2 bridge, C++ control guard, mission
  supervisor, URDF/RViz, and a `ros2_control` dry-run mock/open-loop
  `SystemInterface` with no physical actuation through `ros2_control`.
- Operations readiness: Pi systemd services, SSH/DNS recovery notes, health
  checks, runtime evidence, `ros2_control` evidence, and CI validation are
  documented.
- Honest boundaries: the project separates the bounded M4 single-axis
  closed-loop result from full-arm position control and does not overclaim
  arbitrary-object recognition or autonomous grasping.

## Fast Evidence

- [Physical teleoperation GIF](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)
  / [MP4](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4)
- [`ros2_control` dry-run evidence](docs/evidence/2026-06-16-ros2-control-open-loop.en.md):
  controller manager, hardware-interface plugin, command/state interfaces,
  `FollowJointTrajectory`, and `/joint_states` dry-run state mirror
- [Pi/systemd/ROS2 health evidence](docs/evidence/2026-06-16-pi-system-health.en.md):
  dashboard, perception, ROS2 bridge services, typed topics, service, and action
- [Pi runtime measurements](docs/evidence/2026-06-17-runtime-measurements.en.md):
  HTTP endpoints returned `200`, 15 s bounded ROS2 topic acquisition passed,
  `/joint_states` was about 4.9-5.0 Hz, and the capture was read-only/no actuation
- [M4 shoulder closed-loop evidence](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md):
  AS5600 absolute I2C feedback, sensor-fault stops, and bounded target convergence

## Engineering Problem

The arm is a low-cost 5-axis DC motor platform that still lacks force feedback
and reliable full-arm absolute joint pose. On 2026-06-28, the M4 shoulder alone
was validated with AS5600 absolute feedback and closed-loop targets over a
narrow trial range. That result does not justify a full-arm position-control or
autonomous-grasping claim.

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
- M4 shoulder AS5600 I2C absolute-angle feedback, bounded closed-loop targets,
  and HTTP/dashboard/ROS2 status exposure
- ESP32-CAM capture/stream firmware
- Raspberry Pi dashboard and perception service
- OpenCV-based red target detection and target overlay
- Safety-gated bounded base-nudge control surface
- ROS2 Jazzy bridge, typed messages, C++ control guard, and mission supervisor
- `ros2_control` dry-run mock demo and safe open-loop `SystemInterface` scaffold
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

The public demo is physical teleoperation. Supporting evidence covers `STREAM`-based manual camera feedback, Pi-hosted dashboard, red-target or known-object overlay, ROS2 typed topics, a safety-gated bounded nudge control surface, and token-gated search-light on/off commands. Autonomous grasping remains out of scope for the current hardware state.

## Validation

- ESP32 controller and ESP32-CAM PlatformIO builds pass.
- `TB6612FNG x3` and `M1~M5` motor outputs were physically tested.
- M4 shoulder AS5600 angle/magnet status was validated over I2C; bounded targets
  settled at 238.10/233.96 deg initially and 238.09/234.14 deg in the remount
  regression.
- Shoulder teleoperation cancelled an active target as `OVERRIDDEN` within
  53 ms, followed by a shared soft-limit guard for direct, sequence, and
  teleoperation M4 paths.
- M4 calibrated/raw angle, magnet health, sensor freshness, controller state,
  and manual guard state are exposed through ESP32 `/status`, the Pi dashboard,
  ROS2 typed status, and diagnostics.
- Two fixed-mount regressions each covered five 232-243 deg and 75/100% cases
  plus six repeated 238-to-234 deg cycles, passing 22/22 in total. The second
  complete matrix had 0.191 deg mean absolute error and 0.31 deg maximum
  absolute error, with settled-error revalidation and an explicit
  `TARGET_MISSED` failure state.
- A 23.1 g short upward move exposed a mismatch between PWM ramp time and the
  correction pulse. Increasing only the upward correction maximum to 500 ms
  and adding a separate 0.40 deg internal success margin produced final 11/11
  regressions both without load (0.132 deg mean, 0.36 deg max) and at 23.1 g
  (0.155 deg mean, 0.31 deg max).
- STM32 `MPU-6050 + UART` teleop and the HC-SR04 bench path were validated.
- Wired teleop produced real motor output under deadman control and stopped on release.
- The final physical teleoperation demo was captured and published as README GIF/MP4 assets.
- ESP32-CAM `/status`, `/capture`, and `/stream` were verified.
- Local LAN operation was validated across ESP32 controller, ESP32-CAM, and Raspberry Pi.
- The ESP32-hosted `MotionBrain Control` page accepted a runtime token and executed state-changing commands.
- The Pi dashboard verified the camera feed, red target box, and safety-gated bounded nudge control surface.
- The Pi perception service recognized the `cup` target with ESP32-CAM `qvga` / JPEG quality `10` and YOLOv5s.
- Raspberry Pi 4 + Ubuntu 24.04 + ROS2 Jazzy `colcon build/test` passed.
- Health checks passed for `/motionbrain/status_typed`, `/camera/detection_typed`, status-derived/open-loop `/joint_states`, `/motionbrain/kinematics_typed`, `/motionbrain/control_guard_typed`, and `/motionbrain/mission_state_typed`.
- Pi perception service output was verified through ROS2 `/camera/detection_typed`.
- `motionbrain_ros2_control_mock` and `motionbrain_hardware_interface` validated the `ros2_control` mock/controller and hardware-interface dry-run boundary, not physical `ros2_control` actuation.
- Docker/noVNC RViz validation visualized RobotModel/TF and live ROS2 topics mirrored from the Pi dashboard.
- GitHub Actions validates PlatformIO firmware builds, Python tests, and ROS2 workspace build/test.

## Object Detection Status

The Pi constrained known-object detection path exists: OpenCV DNN/ONNX backend loading, explicit model/label paths, selected-target JSON, dashboard overlay, and ROS2 typed detection publishing. Model weights are intentionally not committed.

The current reliable known-object bench path is constrained `cup` recognition with ESP32-CAM `qvga` / JPEG quality `10`, YOLOv5s, `--object-target cup`, and a configured confidence gate. This path returned `cup` through the Pi dashboard/perception API. Manual camera operation is separated into `STREAM`, while `TRACKED` is used as the slower recognition/confirmation view.

Current honest positioning:

- Implemented: Pi-hosted object-detection pipeline, selected-target contract, ROS2/dashboard integration, constrained `cup` recognition
- Stable demo: red target tracking/overlay, `STREAM` manual camera feedback, cup recognition checks
- Not yet solved: arbitrary-object recognition, marker/object-assisted automatic grasping, and continuous visual servoing without richer feedback

## Current Limitations

- Only the M4 shoulder has mechanically secured AS5600 position feedback. The other
  four axes have no position feedback, and there is no full-arm absolute pose
  or physical closed-loop `ros2_control` path.
- M4 GPIO0/GPIO15 is the supported allocation under the current pin budget but
  requires boot-strap discipline. The sensor and magnet are mechanically
  secured; the 230-245 deg calibrated range and directional stop leads still
  require long-duration, vibration, load, and battery-voltage validation.
- The current `-24.35 deg` angle offset is specific to the current fixed mount and requires
  recalibration after remounting.
- HC-SR04 is removed for the final physical demo, with range telemetry handled as disabled/nonblocking
- ESP32-CAM QVGA input limits general object detection quality
- No general text-prompt object search
- The README physical teleop video and non-motion screenshots/evidence are captured; additional search-light toggle or bounded-nudge evidence should be captured only for a specific follow-up goal

## Next Steps

1. Keep the `demo-ready-20260608` physical teleoperation demo media and README GIF/MP4 as the stable public reference links.
2. Tune the emphasis by context: embedded safety, multi-MCU teleop, Pi perception/dashboard, or ROS2 bridge.
3. Design marker- or fixed-known-object-assisted grasping as a separate plan.
4. Revisit autonomy only after adding a better camera, range/contact sensing, or a validated edge runtime.
5. Do not expand arbitrary-object recognition or autonomous grasping claims without new hardware validation.

## Related Documents

- [README.en.md](README.en.md): project entry point
- [README.md](README.md): Korean project entry point
- [ROBOTICS_SYSTEM_READINESS.en.md](ROBOTICS_SYSTEM_READINESS.en.md): ROS2 and hardware-boundary summary for robotics system roles
- [PIN_MAP.en.md](PIN_MAP.en.md): ESP32 allocation and M4 AS5600 wiring policy
- [docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md): physical M4 single-axis closed-loop evidence
- [docs/evidence/2026-06-16-ros2-control-open-loop.en.md](docs/evidence/2026-06-16-ros2-control-open-loop.en.md): ros2_control dry-run evidence summary
- [docs/evidence/2026-06-16-pi-system-health.en.md](docs/evidence/2026-06-16-pi-system-health.en.md): Pi/systemd/ROS2 health evidence summary
- [docs/evidence/2026-06-17-runtime-measurements.en.md](docs/evidence/2026-06-17-runtime-measurements.en.md): Pi runtime and ROS2 measurement note
- [EMBEDDED_BRINGUP.md](EMBEDDED_BRINGUP.md): STM32/ESP32 bring-up and measurement checklist
- [OPERATIONS.md](OPERATIONS.md): Pi/systemd/health-check operations notes
