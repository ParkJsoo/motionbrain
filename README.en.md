# MotionBrain

[한국어 README](README.md) | [English README](README.en.md) | [Portfolio One-Pager](PORTFOLIO.en.md)

MotionBrain is an embedded robotics portfolio project that starts with an ESP32-based 5-axis robotic arm controller and expands toward an STM32 sensor hub, ESP32-CAM vision input, and a Raspberry Pi + ROS2 host bridge for higher-level robot orchestration.

The project is designed to show more than basic motor movement. It focuses on layered control, safety state management, sensor feedback, command boundaries, and a clear path from low-level embedded control to host-side robotics orchestration.

Core idea:

```text
input -> decision -> state -> motion -> feedback
```

## Current Status

### Implemented

- ESP32 5-axis DC motor control kernel
- Safety-oriented state machine: `BOOT -> IDLE -> ARMED -> FAULT`
- Serial command interface
- Wi-Fi AP and HTTP web control surface
- Joint abstraction layer through `RobotArm`
- Non-blocking motion sequence queue through `MotionSequence`
- `SearchLight` peripheral control
- Physical bench test of `TB6612FNG x3` with `M1~M5`
- STM32 `MotionBrainSensor` project with `MPU-6050 + HC-SR04 + UART` sensor stream bench validation
- ESP32 `Stm32Bridge`, `SafetyMonitor`, `Dispatcher`, and `SafetyGate`
- Unified command path for serial and HTTP inputs
- Experimental base relative-angle control through `base angle ...` and `POST /base`
- Recent event log and `GET /events` API
- Bench simulation path through `sensor sim ...`
- Wired handheld teleoperation v1:
  - ESP32 `teleop_adapter`
  - STM32 `APP_MODE_TELEOP_REMOTE`
  - JSON `teleop` frames with embedded safety telemetry
  - deadman handling
  - frame freshness timeout
  - LED edge counter
  - initial teleop mixer
- Physical wired teleop bench validation:
  - embedded teleop safety telemetry clears `SENSOR_STALE`
  - real motor output from deadman + IMU input
  - stop on `DEADMAN_RELEASE`
- trusted home Wi-Fi station mode for the ESP32 controller and ESP32-CAM
- token-aware host commands and a runtime token prompt in the ESP32-hosted `MotionBrain Control` page
- local ops dashboard for status/events, ESP32-CAM capture, color detection, and token-gated one-shot vision nudge control
- Raspberry Pi 4 + Ubuntu 24.04 + ROS2 Jazzy bridge validation:
  - `motionbrain_ros_bridge` builds and launches on the Pi
  - ESP32 controller status/events publish to ROS2 topics
  - ESP32-CAM capture publishes `/camera/detection`
  - `/motionbrain/light_cmd` reaches token-gated ESP32 `/light` and toggles the real search light

### Current Focus

- Capture Raspberry Pi ROS2 validation logs and demo evidence
- Capture portfolio demo media for the verified teleop, safety, Home Wi-Fi, phone-control, and vision-alignment flows
- Keep live motion demos conservative and explicitly opt-in
- Add the next layer of portfolio depth: kinematics FK/IK, ROS2 message refinement, and AI planning integration

## Architecture

### Implemented Layers

```text
[STM32 Sensor / Teleop Layer]
  MPU-6050
  HC-SR04
  UART teleop stream + embedded safety telemetry
        ->
[ESP32 Motion Controller]
  Stm32Bridge
  SafetyMonitor
  TeleopAdapter
  Dispatcher + SafetyGate
  AngleController
  RobotArm + MotionSequence
  EventLog
        ->
TB6612FNG x3
        ->
5-axis DC motors
```

### Target Architecture

```text
[STM32 Sensor / Teleop Layer]
  HC-SR04 safety input
  GY-521 handheld remote
  UART sensor/teleop stream
        ->
[ESP32 Motion Controller]
  Safety state machine
  Command processing
  Motion execution
        <-
[ESP32-CAM Vision Node]
  Camera streaming
  Visual target input
        ->
[Raspberry Pi + ROS2 + AI]
  High-level planning
  Vision processing
  Message bridge
  Portfolio demo orchestration
```

### Validated Raspberry Pi ROS2 Layer

As of 2026-05-28, the Raspberry Pi 4 ROS2 host path has been validated on real hardware:

```text
[Raspberry Pi 4 / Ubuntu 24.04 / ROS2 Jazzy]
  motionbrain_msgs
  motionbrain_ros_bridge
  /motionbrain/status
  /motionbrain/status_typed
  /motionbrain/events
  /motionbrain/events_typed
  /camera/detection
  /camera/detection_typed
  /joint_states
  /motionbrain/end_effector_pose
  /motionbrain/kinematics
  /motionbrain/control_guard
  /motionbrain/light_cmd
  /motionbrain/light_cmd_typed
  /motionbrain/light_result
  /motionbrain/light_result_typed
        <->
[ESP32 Motion Controller + ESP32-CAM on Home Wi-Fi]
  GET /status
  GET /events
  GET /capture
  POST /light?action=toggle
        ->
real SearchLight output
```

## Hardware

Main components:

- `ESP32 DevKit`
- `ESP32-CAM`
- `TB6612FNG x3`
- 5-axis DC motor robotic arm
- `STM32 B-F446E-96B01A`
- `MPU-6050 (GY-521)`
- `HC-SR04`
- `1602 LCD + I2C backpack`
- Jumper wires, power modules, and breadboard wiring

The current hardware source of truth is [PIN_MAP.md](PIN_MAP.md).

Wired teleop wiring and bench bring-up notes are documented in [docs/TELEOP_BRINGUP.md](docs/TELEOP_BRINGUP.md).

## Software Structure

The main ESP32 entry point is [src/main.cpp](src/main.cpp).

- `system/`: state machine and system initialization
- `motor/`: `TB6612FNG` motor driver layer
- `motion/`: `RobotArm`, `MotionSequence`
- `bridge/`: STM32 sensor bridge and simulation input
- `safety/`: sensor-based blocking and fault latching
- `control/`: `Dispatcher`, `SafetyGate`, `AngleController`, `EventLog`
- `input/`: serial command and teleop input handling
- `network/`: Wi-Fi AP and HTTP server
- `peripheral/`: `SearchLight`
- `debug/`: serial debug logging

Current HTTP routes:

- `GET /status`
- `GET /events`
- `POST /command`
- `POST /motor`
- `POST /joint`
- `POST /base`
- `POST /sequence`
- `POST /light`

### ROS2 Bridge MVP

The Phase 4 host path is exposed through a minimal ROS2 package:

```text
ros2_ws/src/motionbrain_ros_bridge
```

The bridge keeps the ESP32 HTTP API unchanged while publishing robot status,
event logs, and ESP32-CAM detection results into ROS2:

- `/joint_states`
- `/motionbrain/status`
- `/motionbrain/status_typed`
- `/motionbrain/events`
- `/motionbrain/events_typed`
- `/camera/detection`
- `/camera/detection_typed`
- `/motionbrain/end_effector_pose`
- `/motionbrain/kinematics`
- `/motionbrain/control_guard`
- `/motionbrain/light_cmd`
- `/motionbrain/light_cmd_typed`
- `/motionbrain/light_result`
- `/motionbrain/light_result_typed`

The workspace also includes `motionbrain_description`, a lightweight URDF,
`robot_state_publisher` launch path, and RViz config for TF/joint-state
visualization.

`motionbrain_control` adds a small C++ ROS2 guard node that subscribes to typed
status and camera-detection topics, then publishes `/motionbrain/control_guard`
as a readiness/suggested-action JSON state. This adds a real C++ ROS2 component
while keeping unsafe motion decisions outside the unverified host layer.

`motionbrain_kinematics_node` subscribes to `/joint_states`, publishes an FK
end-effector pose on `/motionbrain/end_effector_pose`, and publishes kinematics
diagnostics on `/motionbrain/kinematics`. The pure Python kinematics module also
includes a tested IK suggestion path for reachable target points and joint-limit
checks.

Raspberry Pi bring-up is documented in
[docs/RASPBERRY_PI_ROS2_BRINGUP.md](docs/RASPBERRY_PI_ROS2_BRINGUP.md). The
portfolio validation path has been run on Raspberry Pi 4 with ROS2 Jazzy:
Home Wi-Fi access to the ESP32 controller and ESP32-CAM, JSON and typed topic
echo verification, and a ROS2 command-channel test that toggled the real search
light through the ESP32 `/light` endpoint.
For service-style operation on the Pi, see
[docs/RASPBERRY_PI_DEPLOYMENT.en.md](docs/RASPBERRY_PI_DEPLOYMENT.en.md).

```bash
cd ros2_ws
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_ros_bridge motionbrain_description
source install/setup.bash
export MOTIONBRAIN_HTTP_TOKEN="CHANGE_ME_TO_LOCAL_TOKEN"
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
ros2 launch motionbrain_description display.launch.py
```

If mDNS is unavailable on the Pi, pass IP addresses with `motion_host:=...` and
`camera_url:=http://...`.

## Development Environment

### ESP32

- PlatformIO
- `esp32dev`
- Arduino framework

Build:

```bash
pio run
```

Upload:

```bash
pio run -t upload
```

Serial monitor:

```bash
pio device monitor
```

### ESP32-CAM

The ESP32-CAM firmware lives in:

```bash
firmware/esp32cam
```

Build:

```bash
pio run -d firmware/esp32cam
```

See [PHASE4_MVP.md](PHASE4_MVP.md) for the current camera-to-host MVP plan. The dry-run vision loop reports target center, normalized offset, `LEFT|CENTER|RIGHT|LOST` alignment, and a command suggestion before any optional motion command is enabled. On the current handheld-teleop hardware, opt-in physical alignment uses a short safety-gated base nudge; closed-loop base angle mode is reserved for future base-mounted gyro or encoder feedback.

For bench work on a trusted home LAN, see [docs/HOME_WIFI_MODE.md](docs/HOME_WIFI_MODE.md). The ESP32-hosted `MotionBrain Control` page is the primary manual control surface and works from a phone browser on the same Wi-Fi network. If a command token is configured, the page prompts for it at runtime on the first state-changing command and keeps it only in current page memory. The local ops dashboard at `http://127.0.0.1:8765` is used for observability, camera/detection, and the token-gated one-shot vision nudge.

### STM32

- STM32CubeIDE
- HAL / CubeMX
- Project path: `firmware/stm32/MotionBrainSensor`

Helper scripts:

- `tools/stm32_build.sh`: STM32CubeIDE headless build
- `tools/stm32_upload.sh`: upload with STM32CubeProgrammer CLI over ST-LINK
- `tools/stm32_build_upload.sh`: build then upload

## Message Boundary

The project intentionally keeps serial, HTTP, teleop, and ROS2-facing semantics aligned.

See [MESSAGE_INTERFACE.md](MESSAGE_INTERFACE.md) for the current command and status boundary. This is one of the key design documents because it separates embedded motor execution from host-side planning and ROS2 integration.

## Host-Side Monitor

The host watcher polls `GET /status` and `GET /events` and prints state, safety, base-angle, and teleop information.

```bash
python3 tools/motionbrain_watch.py --host 192.168.4.1 --interval 1.0
```

The local ops dashboard combines status/events with ESP32-CAM capture, color detection, light command logging, and the one-shot vision nudge control. Manual driving remains on the ESP32 `MotionBrain Control` page so the phone browser can act as the wireless controller.

## Why This Project Matters

This repository is structured as an embedded robotics portfolio project. The most important engineering signals are:

- Multi-MCU role separation between ESP32 motion control and STM32 sensor/teleop input
- Safety-first motor control with explicit state transitions and fault latching
- Unified command dispatch across serial and HTTP control paths
- Sensor feedback and simulation hooks for bench validation
- Teleoperation with deadman and frame freshness handling
- Home Wi-Fi phone control with runtime token entry that avoids committing secrets
- Clear path from embedded control to camera input, host-side decision logic, ROS2, and AI integration
- Real Raspberry Pi ROS2 bridge validation with a token-gated command reaching physical hardware

## Related Documents

- [README.md](README.md): Korean project overview and current status
- [PORTFOLIO.en.md](PORTFOLIO.en.md): English portfolio one-pager
- [PHASE4_MVP.md](PHASE4_MVP.md): ESP32-CAM + Mac host MVP
- [docs/RASPBERRY_PI_ROS2_BRINGUP.md](docs/RASPBERRY_PI_ROS2_BRINGUP.md): Raspberry Pi ROS2 bring-up and validation notes
- [docs/ARCHITECTURE_DIAGRAMS.en.md](docs/ARCHITECTURE_DIAGRAMS.en.md): architecture and demo diagrams for portfolio explanation
- [MESSAGE_INTERFACE.md](MESSAGE_INTERFACE.md): command, teleop, and status message boundary
- [PIN_MAP.md](PIN_MAP.md): ESP32 motor pin mapping
- [docs/TELEOP_BRINGUP.md](docs/TELEOP_BRINGUP.md): wired handheld teleop bring-up notes
- [docs/DEMO_RUNBOOK.en.md](docs/DEMO_RUNBOOK.en.md): portfolio demo capture procedure
- [docs/DEMO_RUNBOOK.md](docs/DEMO_RUNBOOK.md): Korean demo capture procedure
- [로드맵.md](%EB%A1%9C%EB%93%9C%EB%A7%B5.md): Korean project roadmap

## License

Personal research, learning, making, and portfolio project.
