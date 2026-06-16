# Robotics System Readiness

[README](README.md) | [PORTFOLIO](PORTFOLIO.md)

This note maps MotionBrain to robotics system-software roles. It separates
implemented evidence from mock, scaffold, and known limitations.

## Role Match

MotionBrain is strongest as a real-hardware robotics integration project:

- ESP32 firmware drives five DC motor axes through TB6612FNG drivers.
- STM32F446 firmware publishes structured sensor and teleoperation frames.
- Raspberry Pi hosts dashboard, perception, and ROS2 Jazzy bridge processes.
- ROS2 packages expose typed status, event, detection, kinematics, guard,
  mission, URDF, RViz, and `ros2_control` surfaces.
- `ros2_control` integration is intentionally safe: mock and dry-run surfaces
  are validated, while physical actuation stays behind embedded safety gates.

This is evidence for robotics platform work: embedded safety boundaries,
hardware integration, ROS2 system software, and real-robot issue analysis.

## Evidence Already In Repo

- Physical controller and dashboard overview: [README.md](README.md)
- Portfolio problem framing and honest limitations: [PORTFOLIO.md](PORTFOLIO.md)
- Public `ros2_control` dry-run evidence note:
  [docs/evidence/2026-06-16-ros2-control-open-loop.md](docs/evidence/2026-06-16-ros2-control-open-loop.md)
- Public Pi/systemd/ROS2 health evidence note:
  [docs/evidence/2026-06-16-pi-system-health.md](docs/evidence/2026-06-16-pi-system-health.md)
- Public Pi runtime measurement evidence:
  [docs/evidence/2026-06-16-runtime-measurements.md](docs/evidence/2026-06-16-runtime-measurements.md)
- ESP32 safety gate and dispatcher: `src/control/`, `src/safety/`
- ESP32 motor driver and pin mapping: `src/motor/motor_driver.*`
- STM32 HAL sensor/teleop firmware: `firmware/stm32/MotionBrainSensor/`
- ROS2 typed messages and bridge: `ros2_ws/src/motionbrain_msgs/`,
  `ros2_ws/src/motionbrain_ros_bridge/`
- C++ ROS2 control guard: `ros2_ws/src/motionbrain_control/`
- URDF/RViz description: `ros2_ws/src/motionbrain_description/`
- `ros2_control` mock demo: `ros2_ws/src/motionbrain_ros2_control_mock/`
- safe open-loop `SystemInterface` scaffold:
  `ros2_ws/src/motionbrain_hardware_interface/`

## ros2_control Boundary

There are two separate `ros2_control` surfaces:

| Surface | Package | Purpose | Physical actuation |
| --- | --- | --- | --- |
| Mock controller | `motionbrain_ros2_control_mock` | Controller-manager, joint-state, and trajectory-controller bring-up with `mock_components/GenericSystem` | No |
| Hardware interface scaffold | `motionbrain_hardware_interface` | Standard `hardware_interface::SystemInterface` shape, joint command/state interfaces, timeout, finite-command guard, launch/config/URDF surface | No direct actuation yet |

The hardware interface scaffold is intentionally safe. Its `write()` method does
not POST to the ESP32 controller. Physical motion remains behind the firmware
`SafetyGate`, token-gated operator UI, deadman/teleop timeout, and routine
execution policy.

Use this claim:

```text
Implemented a safe open-loop ros2_control SystemInterface scaffold and mock
controller setup; physical ESP32 actuation remains guarded by firmware and is
not exposed as an unchecked ros2_control write path.
```

Avoid these claims:

```text
Completed closed-loop ros2_control hardware interface, vendor-specific smart
actuator integration, full-platform motion control, or encoder-grade joint
feedback.
```

## Commands

Host tests:

```bash
python3 -m unittest discover -s tests
```

Firmware builds:

```bash
pio run
pio run -d firmware/esp32cam
```

ROS2 build on Raspberry Pi or Jazzy container:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  motionbrain_msgs \
  motionbrain_control \
  motionbrain_hardware_interface \
  motionbrain_mission \
  motionbrain_ros_bridge \
  motionbrain_description \
  motionbrain_ros2_control_mock
```

Open-loop hardware-interface scaffold launch:

```bash
source install/setup.bash
ros2 launch motionbrain_hardware_interface hardware_interface.launch.py
```

Mock controller evidence launch:

```bash
source install/setup.bash
ros2 launch motionbrain_ros2_control_mock mock_control.launch.py
```

Open-loop hardware-interface evidence capture on the Pi:

```bash
tools/raspi/capture_ros2_control_hardware_evidence.sh
```

## Latest ros2_control Evidence

Captured on Raspberry Pi 4 / ROS2 Jazzy on 2026-06-16. The capture used
`ROS_DOMAIN_ID=43` and the hardware-interface URDF parameter
`transport_mode=dry_run`, so it did not command the ESP32 controller or
physical motors.

| Evidence | Result |
| --- | --- |
| `motionbrain_hardware_interface` plugin load | `MotionBrainOpenLoopSystem` loaded, initialized, configured, and activated |
| Controllers | `joint_state_broadcaster` active, `motionbrain_arm_controller` active |
| Command interfaces | five position command interfaces available and claimed |
| State interfaces | position and velocity state interfaces exported for five joints |
| Open-loop trajectory | `FollowJointTrajectory` goal accepted and completed with `SUCCEEDED` |
| `/joint_states` | changed from all `0.0` to the commanded scaffold positions |

## Latest Runtime Measurement Evidence

Captured on the live Raspberry Pi host on 2026-06-16 with read-only commands.
HTTP dashboard/perception endpoints returned `200` with measured latency, while
bounded ROS2 CLI probes timed out before returning topic/status data. No USB
oscilloscope, logic analyzer, serial adapter, or meter interface was visible to
the Pi, so PWM/UART/I2C waveform and motor-voltage measurements remain gated by
physical instrumentation.

## Claim Boundaries

- I separated host-side ROS2 decision logic from firmware-level motor authority.
- I used typed ROS2 topics for state, routine, detection, kinematics, guard, and
  mission state instead of only string payloads.
- I kept unsafe automation disabled until feedback and physical validation are
  strong enough.
- I added `ros2_control` surfaces without pretending the low-cost arm has
  encoder feedback or production-grade joint control.
- I documented failures and limits: no closed-loop joint feedback, no autonomous
  grasping claim, and no production smart-actuator backend claim.

## Next Work

1. Capture measured embedded evidence: PWM duty/frequency, UART timing,
   deadman release latency, I2C activity, and bounded motor voltage drop.
2. Convert one real ESP32 status field into a read-only `ros2_control`
   diagnostic before exposing any write path to physical motion.
3. If new actuator hardware becomes available, add only a small bench note:
   ping, present-position read, and bounded goal-position write. Do not add
   paper-only actuator claims.
