# MotionBrain

[Korean README](README.md) | [Korean Portfolio](PORTFOLIO.md) | [Portfolio One-Pager](PORTFOLIO.en.md)

MotionBrain is an embedded robotics portfolio project that starts with an ESP32-based 5-axis robotic arm controller and extends into an STM32 sensor/teleop layer, ESP32-CAM vision input, and a Raspberry Pi + ROS2 host bridge.

The project is not just a motor demo. It is structured around safety state management, clear command boundaries, sensor feedback, camera input, and ROS2 integration on real hardware.

```text
input -> decision -> state -> motion -> feedback
```

## Operator Screens

These are documentation captures of the real controller and Pi dashboard UI surfaces, not the final public demo video.

![MotionBrain Control web console](docs/assets/motionbrain-control-stream.png)

The ESP32-hosted `MotionBrain Control` page brings manual operation, token-gated command boundaries, `STREAM` camera feedback, and motor/joint controls into one local operator surface.

![MotionBrain Pi dashboard](docs/assets/motionbrain-dashboard.png)

The Pi-hosted dashboard observes controller state, teleop, events, camera frames, and target detection while exposing only safety-gated corrective actions.

## Current Status

Validated:

- ESP32 5-axis DC motor control and `BOOT -> IDLE -> ARMED -> FAULT` safety state machine
- Shared serial/HTTP command path through `Dispatcher` and `SafetyGate`
- STM32 `MPU-6050 + HC-SR04 + UART` sensor and teleop stream
- Wired handheld teleop with deadman, frame timeout, and embedded safety telemetry
- ESP32-CAM `/status`, `/capture`, `/stream`, and `/camera` profile control
- Home Wi-Fi operation across the ESP32 controller, ESP32-CAM, and Raspberry Pi
- ESP32-hosted `MotionBrain Control` UI with token-gated state-changing commands
- Pi-hosted dashboard for status, events, camera feed, target overlay, and safety-gated nudge actions
- Raspberry Pi 4 + Ubuntu 24.04 + ROS2 Jazzy bridge
- ROS2 typed topics for status, events, camera detection, joint states, kinematics, control guard, and mission state
- Pi perception service feeding `/camera/detection(_typed)`
- ESP32-hosted camera mode split: `STREAM` for manual operation, `TRACKED` for recognition checks
- GitHub Actions checks for PlatformIO builds, Python tests, and ROS2 `colcon build/test`

Important current limits:

- Red target tracking is the reliable demo path.
- The general object-detection pipeline is implemented on the Pi, and the current bench validates constrained known-object `cup` detection with ESP32-CAM `qvga` / JPEG quality `4` plus YOLOv5s. The current physical-AI demo uses only `cup` as the active target.
- A label-less dark bottle, a sticker-heavy iPhone back side, and the secondary Z Flip phone target are out of scope for the current demo. Describe this as constrained workcell known-object detection/alignment, not arbitrary object recognition.
- Autonomous grasping is not enabled. The current cup dry-run path revalidates safety state and CENTER alignment, then returns a gripper open/close plan for operator review only.
- Manual arm operation uses `STREAM` by default. `TRACKED` is a slower Pi-recognition view for checking fixed or slow-moving targets.

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
  /capture, /stream, /camera
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

The Pi dashboard and perception service can run as boot-time systemd services.
Use `docs/RASPBERRY_PI_DEPLOYMENT.en.md` for the install procedure.

Manual fallback dashboard command:

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/motionbrain_dashboard.py \
  --host 0.0.0.0 \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --timeout 6
```

Manual fallback command for running the Pi perception service separately and
proxying it through the dashboard:

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

The current cup known-object demo uses ESP32-CAM `qvga` / JPEG quality `4`, Pi
YOLOv5s object mode, and dashboard proxy mode. The systemd wrappers re-apply
this camera profile after an ESP32-CAM reboot. Use `docs/HOME_WIFI_MODE.md` and
`docs/DEMO_RUNBOOK.en.md` for the exact run commands.

## Documentation

- [PORTFOLIO.en.md](PORTFOLIO.en.md): English portfolio summary
- [PORTFOLIO.md](PORTFOLIO.md): Korean portfolio summary
- [MESSAGE_INTERFACE.md](MESSAGE_INTERFACE.md): serial, HTTP, and ROS2 command/status boundary
- [PIN_MAP.md](PIN_MAP.md): pin and wiring reference
- [docs/DEMO_RUNBOOK.en.md](docs/DEMO_RUNBOOK.en.md): demo capture runbook
- [docs/RASPBERRY_PI_DEPLOYMENT.en.md](docs/RASPBERRY_PI_DEPLOYMENT.en.md): Raspberry Pi systemd deployment
- [docs/RASPBERRY_PI_ROS2_BRINGUP.md](docs/RASPBERRY_PI_ROS2_BRINGUP.md): Raspberry Pi ROS2 bring-up notes
- [docs/PHYSICAL_AI_OBJECT_DETECTION_PLAN.md](docs/PHYSICAL_AI_OBJECT_DETECTION_PLAN.md): object detection and constrained physical-AI plan
- [docs/VISION_DATASET_EVALUATION.md](docs/VISION_DATASET_EVALUATION.md): vision frame capture and offline detector evaluation
- [docs/EMBEDDED_FIRMWARE_EVIDENCE.md](docs/EMBEDDED_FIRMWARE_EVIDENCE.md): embedded firmware validation evidence

## License

Personal research, learning, making, and portfolio project.
