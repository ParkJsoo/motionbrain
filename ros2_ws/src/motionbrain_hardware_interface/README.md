# motionbrain_hardware_interface

Safe `ros2_control` hardware-interface scaffold for MotionBrain.

This package exists to show the ROS2 hardware boundary explicitly without
pretending the current low-cost DC arm has encoder-grade joint feedback.

## What It Does

- Exports a `hardware_interface::SystemInterface` plugin:
  `motionbrain_hardware_interface/MotionBrainHardwareInterface`
- Keeps the existing five-joint `transport_mode=dry_run` demo:
  - `base_yaw_joint`
  - `shoulder_pitch_joint`
  - `elbow_pitch_joint`
  - `wrist_pitch_joint`
  - `gripper_joint`
- Provides position command interfaces plus position/velocity state interfaces
  for each dry-run joint.
- Implements lifecycle callbacks, finite-command validation, command timeout,
  and open-loop state interpolation for dry-run controller bring-up.
- Rejects unsupported transport modes at initialization; the current scaffold
  accepts only `transport_mode=dry_run` and read-only `transport_mode=m4_state`.
- Includes launch/config/URDF files for controller-manager smoke tests.

## M4 Measured State Mode

The package also includes a read-only M4 shoulder feedback mode for calibrated
AS5600 state bring-up:

- `transport_mode=m4_state`
- one joint: `shoulder_pitch_joint`
- state interfaces only: `position` and `velocity`
- zero command interfaces
- explicit hardware parameters:
  - `status_topic=/motionbrain/status_typed`
  - `feedback_source=m4_as5600`
  - `shoulder_feedback_calibration_enabled`
  - `shoulder_sensor_zero_deg`
  - `shoulder_direction_sign`
  - `shoulder_ros_joint_zero_rad`

This mode is intended for publishing calibrated measured state through
`joint_state_broadcaster`. It does not load `joint_trajectory_controller` and
does not send physical commands. When calibration is disabled, stale, or
invalid, the state is unavailable instead of falling back to raw AS5600 degrees
or open-loop tilt estimates.

## Safety Boundary

In the current repository state this package is non-physical. `write()` does
not POST to the ESP32 controller, and no STM32, motor, gripper, camera, or light
is contacted by this plugin.

The dry-run URDF uses `transport_mode=dry_run`. The M4 measured URDF uses
`transport_mode=m4_state`, subscribes only to typed status, and exposes no
command interfaces. It does not make HTTP calls from `read()`.

Physical actuation remains behind:

- ESP32 firmware `SafetyGate`
- `BOOT -> IDLE -> ARMED -> FAULT` state boundary
- token-gated HTTP commands
- deadman/teleop timeout paths
- routine executor policy and feedback readiness checks

Use this wording for public claims:

```text
Implemented a safe ros2_control SystemInterface scaffold for the MotionBrain
arm, including an open-loop dry-run demo and a read-only M4 measured-state
mode. It validates the ROS2 hardware boundary and controller integration,
while physical motion remains guarded by ESP32 firmware.
```

Do not claim:

```text
Closed-loop trajectory execution, vendor-specific smart-actuator integration,
encoder-verified motion, full-platform motion control, or production hardware
interface completion.
```

## Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_hardware_interface
colcon test --packages-select motionbrain_hardware_interface
colcon test-result --verbose
```

## Launch

```bash
source install/setup.bash
ros2 launch motionbrain_hardware_interface hardware_interface.launch.py
```

Then inspect:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

For the read-only M4 measured state mode:

```bash
source install/setup.bash
ros2 launch motionbrain_hardware_interface m4_measured_state.launch.py \
  shoulder_feedback_calibration_enabled:=true \
  shoulder_sensor_zero_deg:=<supervised_zero_deg> \
  shoulder_direction_sign:=<1_or_-1>
```

Leave `shoulder_feedback_calibration_enabled=false` until a supervised
read-only preflight confirms the current mount zero and direction.

## Future Physical Backend

A future ESP32 backend should be added behind an explicit mode flag and should
forward only bounded, firmware-accepted commands. Because the current arm has no
per-joint encoders, trajectory completion must be treated as command acceptance
and estimated motion, not physical convergence.
