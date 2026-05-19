# MotionBrain

[Korean README](README.md) | [English README](README.en.md) | [Portfolio One-Pager](PORTFOLIO.en.md)

MotionBrain is an embedded robotics portfolio project that starts with an ESP32-based 5-axis robotic arm controller and expands toward an STM32 sensor hub, ESP32-CAM vision input, and Raspberry Pi + ROS2 + AI high-level control.

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
  - JSON `teleop` frames
  - deadman handling
  - frame freshness timeout
  - LED edge counter
  - initial teleop mixer
- Physical wired teleop bench validation:
  - `sensor sim healthy -> arm`
  - real motor output from deadman + IMU input
  - stop on `DEADMAN_RELEASE`
  - `teleop.parseErrors=0`, `sensor.parseErrors=0`

### Current Focus

- Connect ESP32-CAM streaming to the Mac host-side vision MVP
- Finalize the physical layout and wiring table
- Revalidate sensor, obstacle-safety, and teleop behavior after final mounting
- Define the Raspberry Pi + ROS2 + AI high-level control path

## Architecture

### Implemented Layers

```text
[STM32 Sensor / Teleop Layer]
  MPU-6050
  HC-SR04
  UART sensor/teleop stream
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

See [PHASE4_MVP.md](PHASE4_MVP.md) for the current camera-to-host MVP plan.

### STM32

- STM32CubeIDE
- HAL / CubeMX
- Project path: `firmware/stm32/MotionBrainSensor`

Helper scripts:

- `tools/stm32_build.sh`: STM32CubeIDE headless build
- `tools/stm32_upload.sh`: upload with STM32CubeProgrammer CLI over ST-LINK
- `tools/stm32_build_upload.sh`: build then upload

## Message Boundary

The project intentionally keeps serial, HTTP, teleop, and future ROS2-facing semantics aligned.

See [MESSAGE_INTERFACE.md](MESSAGE_INTERFACE.md) for the current command and status boundary. This is one of the key design documents because it separates embedded motor execution from host-side planning and future ROS2 integration.

## Host-Side Monitor

The host watcher polls `GET /status` and `GET /events` and prints state, safety, base-angle, and teleop information.

```bash
python3 tools/motionbrain_watch.py --host 192.168.4.1 --interval 1.0
```

## Why This Project Matters

This repository is structured as an embedded robotics portfolio project. The most important engineering signals are:

- Multi-MCU role separation between ESP32 motion control and STM32 sensor/teleop input
- Safety-first motor control with explicit state transitions and fault latching
- Unified command dispatch across serial and HTTP control paths
- Sensor feedback and simulation hooks for bench validation
- Teleoperation with deadman and frame freshness handling
- Clear path from embedded control to camera input, host-side decision logic, ROS2, and AI integration

## Related Documents

- [README.md](README.md): Korean project overview and current status
- [PORTFOLIO.en.md](PORTFOLIO.en.md): English portfolio one-pager
- [PHASE4_MVP.md](PHASE4_MVP.md): ESP32-CAM + Mac host MVP
- [MESSAGE_INTERFACE.md](MESSAGE_INTERFACE.md): command, teleop, and status message boundary
- [PIN_MAP.md](PIN_MAP.md): ESP32 motor pin mapping
- [docs/TELEOP_BRINGUP.md](docs/TELEOP_BRINGUP.md): wired handheld teleop bring-up notes
- [로드맵.md](%EB%A1%9C%EB%93%9C%EB%A7%B5.md): Korean project roadmap

## License

Personal research, learning, making, and portfolio project.
