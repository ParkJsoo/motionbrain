# Robotics System Readiness

[README](README.en.md) | [PORTFOLIO](PORTFOLIO.en.md) | [Korean](ROBOTICS_SYSTEM_READINESS.md)

This note maps MotionBrain to robotics system-software roles. It separates
implemented evidence from mock, scaffold, and known limitations.

## Role Match

MotionBrain is strongest as a real-hardware robotics integration project:

- ESP32 firmware drives five DC motor axes through TB6612FNG drivers.
- The M4 shoulder alone has physically validated AS5600 absolute I2C feedback
  and bounded closed-loop targets. Other axes have no position feedback, and
  physical `ros2_control` writes remain disabled.
- STM32F446 firmware publishes structured sensor and teleoperation frames.
- Raspberry Pi hosts dashboard, perception, and ROS2 Jazzy bridge processes.
- ROS2 packages expose typed status, event, detection, kinematics, guard,
  mission, URDF, RViz, and `ros2_control` surfaces.
- `ros2_control` integration is intentionally safe: mock, dry-run/open-loop, and
  M4 read-only measured state surfaces are validated, while physical actuation
  stays behind embedded safety gates.

This is evidence for robotics platform work: embedded safety boundaries,
hardware integration, ROS2 system software, and real-robot issue analysis.

## Evidence Already In Repo

- Physical controller and dashboard overview: [README.en.md](README.en.md)
- Portfolio problem framing and honest limitations: [PORTFOLIO.en.md](PORTFOLIO.en.md)
- Claim/evidence/limitation matrix:
  [docs/evidence/claim-to-evidence-matrix.md](docs/evidence/claim-to-evidence-matrix.md)
- Public `ros2_control` dry-run evidence note:
  [docs/evidence/2026-06-16-ros2-control-open-loop.en.md](docs/evidence/2026-06-16-ros2-control-open-loop.en.md)
- Public Pi/systemd/ROS2 health evidence note:
  [docs/evidence/2026-06-16-pi-system-health.en.md](docs/evidence/2026-06-16-pi-system-health.en.md)
- Public Pi runtime measurement evidence:
  [docs/evidence/2026-06-17-runtime-measurements.en.md](docs/evidence/2026-06-17-runtime-measurements.en.md)
- Public embedded bench-check evidence:
  [docs/evidence/2026-06-16-embedded-bench-checks.en.md](docs/evidence/2026-06-16-embedded-bench-checks.en.md)
- Public M4 shoulder closed-loop evidence:
  [docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md)
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

There are three separate `ros2_control` surfaces:

| Surface | Package | Purpose | Physical actuation |
| --- | --- | --- | --- |
| Mock controller | `motionbrain_ros2_control_mock` | Controller-manager, joint-state, and trajectory-controller bring-up with `mock_components/GenericSystem` | No |
| Hardware interface scaffold | `motionbrain_hardware_interface` | Standard `hardware_interface::SystemInterface` shape, joint command/state interfaces, timeout, finite-command guard, launch/config/URDF surface | No direct actuation yet |
| M4 measured state mode | `motionbrain_hardware_interface` | Read-only `m4_state` mode exposing only M4 `shoulder_pitch_joint` state from cached `/motionbrain/status_typed` | No command interface |

The hardware interface scaffold and M4 measured state mode are intentionally
safe. Physical-mode `write()` does not POST to the ESP32 controller, and
`m4_state` exposes no command interface. Physical motion remains behind the
firmware `SafetyGate`, token-gated operator UI, deadman/teleop timeout, and
routine execution policy.

Use this claim:

```text
Implemented a safe open-loop ros2_control SystemInterface scaffold, mock
controller setup, and M4 read-only measured state mode; physical ESP32
actuation remains guarded by firmware and is not exposed as an unchecked
ros2_control write path.
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

M4 read-only measured state launch:

```bash
source install/setup.bash
ros2 launch motionbrain_hardware_interface m4_measured_state.launch.py
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

The initial open-loop evidence was captured on Raspberry Pi 4 / ROS2 Jazzy on
2026-06-16. The capture used
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

M4 read-only measured state mode was added later. It uses only
`joint_state_broadcaster`, exposes no command interface or trajectory controller,
and publishes a finite `shoulder_pitch_joint` state only from valid, fresh cached
`/motionbrain/status_typed` feedback.

## Latest Runtime Measurement Evidence

Captured on the live Raspberry Pi host on 2026-06-17 with read-only commands.
HTTP controller, camera, dashboard, and perception endpoints returned `200`.
ROS2 graph discovery showed the bridge, joint-state, kinematics, control-guard,
and mission nodes. With a 15 s bounded CLI acquisition window, sampled ROS2
topic probes and routine status service/action probes completed successfully.
No USB oscilloscope, logic analyzer, serial adapter, or meter interface was
visible to the Pi, so PWM/UART/I2C waveform and motor-voltage measurements
remain gated by physical instrumentation.

## Recovered Embedded Bench Evidence

Recovered repository history records digital-multimeter-level checks for common
ground continuity, obvious shorts, `3V3` logic rails, TB6612FNG `VCC`/`VM`,
active-low button HIGH/LOW behavior, and simple output voltage sanity. This is
useful embedded bring-up evidence, but it does not support UART timing, PWM
duty/frequency, I2C signal-integrity, transient motor-voltage, or closed-loop
joint-control claims.

## Claim Boundaries

- I separated host-side ROS2 decision logic from firmware-level motor authority.
- I used typed ROS2 topics for state, routine, detection, kinematics, guard, and
  mission state instead of only string payloads.
- I kept unsafe automation disabled until feedback and physical validation are
  strong enough.
- I kept the trial M4 single-axis feedback result separate from full-arm
  encoder feedback, production-grade joint control, and the `ros2_control`
  dry-run surface.
- I documented failures and limits: no position feedback on the other four
  axes, no full-arm closed loop, no autonomous grasping claim, and no
  production smart-actuator backend claim.

## Next Work

1. Capture instrumented embedded evidence beyond DMM-level sanity checks: PWM
   duty/frequency, UART timing, deadman release latency, I2C activity, and
   bounded motor voltage drop.
2. Decide whether the M4 read-only measured state evidence needs a compact public
   refresh after the next stable reconnect.
3. If new actuator hardware becomes available, add only a small bench note:
   ping, present-position read, and bounded goal-position write. Do not add
   paper-only actuator claims.
