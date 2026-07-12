# MotionBrain

[Korean README](README.md) | [Korean Portfolio](PORTFOLIO.md) | [Portfolio One-Pager](PORTFOLIO.en.md)

MotionBrain is an embedded robotics system project that starts with an ESP32-based 5-axis robotic arm controller and extends into an STM32 sensor/teleop layer, ESP32-CAM vision input, and a Raspberry Pi + ROS2 host bridge.

The project is not just a motor demo. It is structured around safety state management, clear command boundaries, sensor feedback, camera input, and ROS2 integration on real hardware.

```text
input -> decision -> state -> motion -> feedback
```

## Evidence At A Glance

| Capability | Evidence |
| --- | --- |
| Real robot integration | ESP32 5-axis motion controller, STM32 wired teleop layer, ESP32-CAM, and Raspberry Pi host integrated into one arm stack |
| Embedded safety boundary | `BOOT -> IDLE -> ARMED -> FAULT`, `Dispatcher` + `SafetyGate`, token-gated commands, deadman release stop, and frame timeouts |
| Single-axis position feedback | M4 shoulder AS5600 sensing, magnet/sensor health, bounded 230-245 deg proven closed-loop targets, explicit `TARGET_MISSED`, 22/22 no-added-load plus 11/11 at 23.1 g on the fixed mount, and HTTP/dashboard/ROS2 telemetry |
| ROS2 system software | ROS2 Jazzy typed topics, C++ control guard, mission supervisor, URDF/RViz, dry-run/read-only modes, and operator-confirmed physical `ros2_control` writes for M4 |
| Operations and validation | Pi systemd services, health-check scripts, runtime evidence, `ros2_control` evidence, PlatformIO/Python/ROS2 GitHub Actions, and a physical teleoperation demo |

Good first evidence links for reviewers:

- [PORTFOLIO.en.md](PORTFOLIO.en.md)
- [ROBOTICS_SYSTEM_READINESS.en.md](ROBOTICS_SYSTEM_READINESS.en.md)
- [docs/evidence/claim-to-evidence-matrix.md](docs/evidence/claim-to-evidence-matrix.md)
- [OPERATIONS.md](OPERATIONS.md)
- [PIN_MAP.en.md](PIN_MAP.en.md)
- [docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md)
- [docs/evidence/2026-06-16-ros2-control-open-loop.en.md](docs/evidence/2026-06-16-ros2-control-open-loop.en.md)
- [docs/evidence/2026-06-16-pi-system-health.en.md](docs/evidence/2026-06-16-pi-system-health.en.md)
- [docs/evidence/2026-06-17-runtime-measurements.en.md](docs/evidence/2026-06-17-runtime-measurements.en.md)

## Robotics System Fit

The strongest system-level evidence in this repository is the combination of
real hardware integration and ROS2 boundary design: ROS2 Jazzy typed
interfaces, C++ guard logic, mission supervision, RViz/TF visualization,
`ros2_control` dry-run/read-only modes, and an M4-only physical-write path.
Proposals are not forwarded automatically; only an operator-confirmed 20-second
one-shot can reach the authenticated ESP32 `/shoulder` endpoint and `SafetyGate`.
A 248.20 deg start reached 249.96 deg for a 250.00 deg target
(`TARGET_REACHED`, -0.04 deg), and proposal reuse was rejected. [Detailed evidence](docs/evidence/2026-07-13-m4-physical-ros2-control.en.md)

## Demo Video

The GIF below shows the final physical teleoperation demo directly in the README.

![MotionBrain demo video](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)

[Download the MP4 file](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4)

Physical teleoperation demo media snapshot: `demo-ready-20260608`

## Operator Screens

These are documentation captures of the real controller and Pi dashboard UI surfaces, not the final public demo video.

![MotionBrain Control web console](docs/assets/motionbrain-control-stream.png)

The ESP32-hosted `MotionBrain Control` page brings manual operation, token-gated command boundaries, direct capture / Pi tracked-frame camera feedback, and motor/joint controls into one local operator surface.

![MotionBrain Pi dashboard](docs/assets/motionbrain-dashboard.png)

The Pi-hosted dashboard observes controller state, teleop, events, camera frames, and target detection while exposing only operator-triggered, token/safety-gated bounded base-nudge controls.

![MotionBrain RViz RobotModel](docs/assets/motionbrain-rviz-robotmodel.png)

The Docker/noVNC RViz view mirrors Pi dashboard status and detection through read-only HTTP polling into ROS2 topics, then visualizes the `RobotModel` and TF path.

## Scope And Limits

This README is the project entry point for structure, demo media, and build/run
commands. Detailed validation results and portfolio claim boundaries live in
[PORTFOLIO.en.md](PORTFOLIO.en.md) and the
[claim-to-evidence matrix](docs/evidence/claim-to-evidence-matrix.md).

- Physical motion authority remains inside the ESP32 firmware `SafetyGate`.
  ROS2 physical writes are limited to an operator-confirmed one-shot M4 target.
- Position feedback is limited to the M4 shoulder AS5600. The other four axes do
  not have encoder-grade feedback.
- The matrix-proven M4 target range is 230-245 deg. 122.08-301.02 deg is only a
  provisional current-posture soft range, not an equivalently validated range.
- Direct ESP32-CAM `/stream` returns HTTP 410 by design; current camera viewing
  uses `/capture` or Pi tracked frames.
- Pi perception is scoped to constrained known-object `cup` handling and red
  target alignment evidence. Arbitrary object recognition and autonomous
  grasping are not claimed.
- Physical guarded routine `run/execute` remains disabled because
  `base_yaw_reference` is not installed.

## System Layout

```text
[STM32 Sensor / Teleop]
  MPU-6050, UART frames
  HC-SR04 firmware path bench-validated, not installed in the final demo
        ->
[ESP32 Motion Controller]
  SafetyMonitor, Dispatcher, SafetyGate
  RobotArm, MotionSequence, EventLog
        ->
[TB6612FNG x3]
        ->
5-axis DC motor robotic arm

[ESP32-CAM]
  /capture, /status, /camera profile
  /stream returns HTTP 410 by design
        ->
[Raspberry Pi]
  perception service
  dashboard
  ROS2 bridge
        ->
ROS2 typed topics, control guard, mission supervisor
```

## Repository Map

- `src/`: ESP32 motion-controller firmware
- `firmware/esp32cam/`: ESP32-CAM firmware
- `firmware/stm32/MotionBrainSensor/`: STM32 sensor/teleop firmware
- `tools/`: dashboard, perception service, watcher, STM32 helper scripts
- `ros2_ws/src/`: ROS2 messages, bridge, control guard, mission, URDF, and `ros2_control` packages
- `docs/assets/`: public demo images and video used by the README and portfolio
- `config/`: runtime config such as vision labels

## Quick Start

Build the ESP32 motion controller:

```bash
pio run
```

Build the ESP32-CAM firmware:

```bash
pio run -d firmware/esp32cam
```

Run host tests:

```bash
python3 -m unittest discover -s tests
```

Check Raspberry Pi access:

```bash
python3 tools/raspi/check_pi_ssh_target.py
ssh motionbrain-pi 'hostname; hostname -I; systemctl is-active ssh'
```

The Pi SSH alias should primarily follow `motionbrain-pi.local`, not a DHCP IP
literal. Treat `.davolink` as a router-DNS fallback. See
[OPERATIONS.md](OPERATIONS.md) for the access and recovery flow.

Build and test the ROS2 workspace on Raspberry Pi:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_hardware_interface motionbrain_mission motionbrain_ros_bridge motionbrain_description motionbrain_ros2_control_mock
colcon test --packages-select motionbrain_msgs motionbrain_control motionbrain_hardware_interface motionbrain_mission motionbrain_ros_bridge motionbrain_description motionbrain_ros2_control_mock
colcon test-result --verbose
```

The Pi dashboard and perception service can run as boot-time systemd services
using the unit files in `deploy/systemd/`.

Manual fallback dashboard command:

The real `MOTIONBRAIN_HTTP_TOKEN` is a local device command token. Do not expose
the real value in the repository, logs, or screen captures. Dashboard POST
controls also require a separate `MOTIONBRAIN_DASHBOARD_TOKEN`. To expose the
dashboard on a LAN, use `--host 0.0.0.0` only with a dashboard token.

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
export MOTIONBRAIN_DASHBOARD_TOKEN="<local-dashboard-token>"
python3 tools/motionbrain_dashboard.py \
  --host 127.0.0.1 \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --timeout 6
```

Manual fallback command for running the Pi perception service separately and
proxying it through the dashboard:

```bash
python3 tools/motionbrain_perception_service.py \
  --host 127.0.0.1 \
  --port 8766 \
  --camera-url http://<camera-ip> \
  --detector-mode color \
  --detect-color red \
  --timeout 6

export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
export MOTIONBRAIN_DASHBOARD_TOKEN="<local-dashboard-token>"
python3 tools/motionbrain_dashboard.py \
  --host 127.0.0.1 \
  --motion-host <controller-ip> \
  --perception-url http://127.0.0.1:8766 \
  --timeout 6
```

The current cup known-object path uses Pi YOLOv5s object mode, a configured
confidence gate, and dashboard proxy mode. The stable CAM service profile is
ESP32-CAM `qvga` / JPEG quality `15`. The systemd wrappers re-apply this camera
profile after an ESP32-CAM reboot and raise lower JPEG quality settings to the
stable minimum.

## Documentation

- [PORTFOLIO.en.md](PORTFOLIO.en.md): English portfolio summary
- [PORTFOLIO.md](PORTFOLIO.md): Korean portfolio summary
- [ROBOTICS_SYSTEM_READINESS.en.md](ROBOTICS_SYSTEM_READINESS.en.md): robotics system and ROS2 hardware-boundary summary
- [docs/evidence/claim-to-evidence-matrix.md](docs/evidence/claim-to-evidence-matrix.md): claim/evidence/limitation/non-claim matrix
- [PIN_MAP.en.md](PIN_MAP.en.md): ESP32 allocation and M4 AS5600 I2C boot conditions
- [docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md): M4 shoulder AS5600 absolute feedback and bounded physical closed-loop evidence
- [docs/evidence/2026-06-16-ros2-control-open-loop.en.md](docs/evidence/2026-06-16-ros2-control-open-loop.en.md): public ros2_control dry-run evidence note
- [docs/evidence/2026-07-13-m4-physical-ros2-control.en.md](docs/evidence/2026-07-13-m4-physical-ros2-control.en.md): M4 physical `ros2_control` one-shot evidence
- [docs/evidence/2026-06-16-pi-system-health.en.md](docs/evidence/2026-06-16-pi-system-health.en.md): public Pi/systemd/ROS2 health evidence note
- [docs/evidence/2026-06-17-runtime-measurements.en.md](docs/evidence/2026-06-17-runtime-measurements.en.md): Pi runtime endpoint latency, ROS2 topic/status probe, and instrument inventory record
- [docs/evidence/2026-06-16-embedded-bench-checks.en.md](docs/evidence/2026-06-16-embedded-bench-checks.en.md): recovered DMM-level embedded bench sanity-check evidence
- [docs/evidence/physical-safety-validation-plan.en.md](docs/evidence/physical-safety-validation-plan.en.md): physical safety planned-evidence procedure for hard cutoff, deadman latency, PWM/UART/I2C, and motor sag
- [EMBEDDED_BRINGUP.md](EMBEDDED_BRINGUP.md): STM32/ESP32 bring-up and measurement checklist
- [OPERATIONS.md](OPERATIONS.md): Pi/systemd/health-check operations notes

## License

Project code is MIT licensed; see [LICENSE](LICENSE). STM32 HAL/CMSIS vendor
files retain their upstream licenses under `firmware/stm32/MotionBrainSensor/Drivers/`.
