# MotionBrain

[Korean README](README.md) | [Korean Portfolio](PORTFOLIO.md) | [Portfolio One-Pager](PORTFOLIO.en.md)

MotionBrain is an embedded robotics portfolio project that starts with an ESP32-based 5-axis robotic arm controller and extends into an STM32 sensor/teleop layer, ESP32-CAM vision input, and a Raspberry Pi + ROS2 host bridge.

The project is not just a motor demo. It is structured around safety state management, clear command boundaries, sensor feedback, camera input, and ROS2 integration on real hardware.

```text
input -> decision -> state -> motion -> feedback
```

## Current Status

Validated:

- ESP32 5-axis DC motor control and `BOOT -> IDLE -> ARMED -> FAULT` safety state machine
- Shared serial/HTTP command path through `Dispatcher` and `SafetyGate`
- STM32 `MPU-6050 + HC-SR04 + UART` sensor and teleop stream
- Wired handheld teleop with deadman, frame timeout, and embedded safety telemetry
- ESP32-CAM `/status`, `/capture`, and `/stream`
- Home Wi-Fi operation across the ESP32 controller, ESP32-CAM, and Raspberry Pi
- ESP32-hosted `MotionBrain Control` UI with token-gated state-changing commands
- Pi-hosted dashboard for status, events, camera feed, target overlay, and safety-gated nudge actions
- Raspberry Pi 4 + Ubuntu 24.04 + ROS2 Jazzy bridge
- ROS2 typed topics for status, events, camera detection, joint states, kinematics, control guard, and mission state
- Pi perception service feeding `/camera/detection(_typed)`
- GitHub Actions checks for PlatformIO builds, Python tests, and ROS2 `colcon build/test`

Important current limits:

- Red target tracking is the reliable demo path.
- The general object-detection pipeline is implemented on the Pi, but the current ESP32-CAM QVGA input and tested YOLO-family models did not reliably detect a cup.
- Autonomous grasping is not enabled. The current physical-AI path is limited to safety-gated perception, alignment, and operator confirmation.

## System Layout

```text
[STM32 Sensor / Teleop]
  MPU-6050, HC-SR04, UART frames
        ->
[ESP32 Motion Controller]
  SafetyMonitor, Dispatcher, SafetyGate
  RobotArm, MotionSequence, EventLog
        ->
[TB6612FNG x3]
        ->
5-axis DC motor robotic arm

[ESP32-CAM]
  /capture, /stream
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
- `ros2_ws/src/`: ROS2 messages, bridge, control guard, mission, and URDF packages
- `docs/`: demo runbooks, Raspberry Pi deployment, architecture notes, validation records
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

Build and test the ROS2 workspace on Raspberry Pi:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description
colcon test --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description
colcon test-result --verbose
```

Start the Pi dashboard:

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/motionbrain_dashboard.py \
  --host 0.0.0.0 \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --timeout 6
```

Run the Pi perception service separately and proxy it through the dashboard:

```bash
python3 tools/motionbrain_perception_service.py \
  --host 0.0.0.0 \
  --port 8766 \
  --camera-url http://<camera-ip> \
  --detector-mode color \
  --detect-color red \
  --timeout 6

export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/motionbrain_dashboard.py \
  --host 0.0.0.0 \
  --motion-host <controller-ip> \
  --perception-url http://127.0.0.1:8766 \
  --timeout 6
```

## Documentation

- [PORTFOLIO.en.md](PORTFOLIO.en.md): English portfolio summary
- [PORTFOLIO.md](PORTFOLIO.md): Korean portfolio summary
- [MESSAGE_INTERFACE.md](MESSAGE_INTERFACE.md): serial, HTTP, and ROS2 command/status boundary
- [PIN_MAP.md](PIN_MAP.md): pin and wiring reference
- [docs/DEMO_RUNBOOK.en.md](docs/DEMO_RUNBOOK.en.md): demo capture runbook
- [docs/RASPBERRY_PI_DEPLOYMENT.en.md](docs/RASPBERRY_PI_DEPLOYMENT.en.md): Raspberry Pi systemd deployment
- [docs/RASPBERRY_PI_ROS2_BRINGUP.md](docs/RASPBERRY_PI_ROS2_BRINGUP.md): Raspberry Pi ROS2 bring-up notes
- [docs/PHYSICAL_AI_OBJECT_DETECTION_PLAN.md](docs/PHYSICAL_AI_OBJECT_DETECTION_PLAN.md): object detection and constrained physical-AI plan
- [docs/EMBEDDED_FIRMWARE_EVIDENCE.md](docs/EMBEDDED_FIRMWARE_EVIDENCE.md): embedded firmware validation evidence

## License

Personal research, learning, making, and portfolio project.
