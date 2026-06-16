# ROBOTIS Readiness

[README](README.md) | [PORTFOLIO](PORTFOLIO.md)

This note is a hiring-focused map for the ROBOTIS humanoid system software role.
It separates implemented evidence from mock, scaffold, and known limitations.

## Role Match

MotionBrain is strongest as a real-hardware robotics integration project:

- ESP32 firmware drives five DC motor axes through TB6612FNG drivers.
- STM32F446 firmware publishes structured sensor and teleoperation frames.
- Raspberry Pi hosts dashboard, perception, and ROS2 Jazzy bridge processes.
- ROS2 packages expose typed status, event, detection, kinematics, guard, mission,
  URDF, RViz, and ros2_control surfaces.

This is not a ROBOTIS SDK or DYNAMIXEL project. It is evidence for adjacent
robotics platform work: embedded safety boundaries, hardware integration,
ROS2 system software, and real-robot issue analysis.

## Evidence Already In Repo

- Physical controller and dashboard overview: [README.md](README.md)
- Portfolio problem framing and honest limitations: [PORTFOLIO.md](PORTFOLIO.md)
- ESP32 safety gate and dispatcher: `src/control/`, `src/safety/`
- ESP32 motor driver and pin mapping: `src/motor/motor_driver.*`
- STM32 HAL sensor/teleop firmware: `firmware/stm32/MotionBrainSensor/`
- ROS2 typed messages and bridge: `ros2_ws/src/motionbrain_msgs/`,
  `ros2_ws/src/motionbrain_ros_bridge/`
- C++ ROS2 control guard: `ros2_ws/src/motionbrain_control/`
- URDF/RViz description: `ros2_ws/src/motionbrain_description/`
- ros2_control mock demo: `ros2_ws/src/motionbrain_ros2_control_mock/`
- safe open-loop SystemInterface scaffold:
  `ros2_ws/src/motionbrain_hardware_interface/`

## ros2_control Boundary

There are now two separate ros2_control surfaces:

| Surface | Package | Purpose | Physical actuation |
| --- | --- | --- | --- |
| Mock controller | `motionbrain_ros2_control_mock` | Controller-manager, joint-state, and trajectory-controller bring-up with `mock_components/GenericSystem` | No |
| Hardware interface scaffold | `motionbrain_hardware_interface` | Standard `hardware_interface::SystemInterface` shape, joint command/state interfaces, timeout, finite-command guard, launch/config/URDF surface | No direct actuation yet |

The hardware interface scaffold is intentionally safe. Its `write()` method does
not POST to the ESP32 controller. Physical motion remains behind the firmware
`SafetyGate`, token-gated operator UI, deadman/teleop timeout, and routine
execution policy.

This is the right claim:

```text
Implemented a safe open-loop ros2_control SystemInterface scaffold and mock
controller setup; physical ESP32 actuation remains guarded by firmware and is
not exposed as an unchecked ros2_control write path.
```

Do not claim:

```text
Completed closed-loop ros2_control hardware interface, DYNAMIXEL integration,
humanoid whole-body control, or encoder-grade joint feedback.
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
`transport_mode=dry_run`, so it did not command the ESP32 controller or physical
motors.

| Evidence | Result |
| --- | --- |
| `motionbrain_hardware_interface` plugin load | `MotionBrainOpenLoopSystem` loaded, initialized, configured, and activated |
| Controllers | `joint_state_broadcaster` active, `motionbrain_arm_controller` active |
| Command interfaces | five position command interfaces available and claimed |
| State interfaces | position and velocity state interfaces exported for five joints |
| Open-loop trajectory | `FollowJointTrajectory` goal accepted and completed with `SUCCEEDED` |
| `/joint_states` | changed from all `0.0` to the commanded scaffold positions |

Local raw evidence path:
`.codex/tmp/evidence/robotis-ros2-control-open-loop-20260616/capture.txt`.

## Interview Talking Points

- I separated host-side ROS2 decision logic from firmware-level motor authority.
- I used typed ROS2 topics for state, routine, detection, kinematics, guard, and
  mission state instead of only string payloads.
- I kept unsafe automation disabled until feedback and physical validation are
  strong enough.
- I added ros2_control surfaces without pretending the low-cost arm has encoder
  feedback or production-grade joint control.
- I documented failures and limits: no DYNAMIXEL, no humanoid stack, no
  closed-loop joint feedback, no autonomous grasping claim.

## Next ROBOTIS-Focused Work

1. Capture Pi operations evidence: SSH alias check, active systemd services,
   and `check_ros_bridge_health.sh` passing on the live robot host.
2. If actual DYNAMIXEL hardware becomes available, add only a small SDK
   ping/read/write bench note. Do not add a paper-only DYNAMIXEL claim.
3. Convert one real ESP32 status field into a read-only ros2_control diagnostic
   before exposing any write path to physical motion.
