# motionbrain_hardware_interface

Safe open-loop `ros2_control` hardware-interface scaffold for MotionBrain.

This package exists to show the ROS2 hardware boundary explicitly without
pretending the current low-cost DC arm has encoder-grade joint feedback.

## What It Does

- Exports a `hardware_interface::SystemInterface` plugin:
  `motionbrain_hardware_interface/MotionBrainHardwareInterface`
- Provides five position command interfaces:
  - `base_yaw_joint`
  - `shoulder_pitch_joint`
  - `elbow_pitch_joint`
  - `wrist_pitch_joint`
  - `gripper_joint`
- Provides position and velocity state interfaces for each joint.
- Implements lifecycle callbacks, finite-command validation, command timeout,
  and open-loop state interpolation for ROS2 controller bring-up.
- Includes launch/config/URDF files for controller-manager smoke tests.

## Safety Boundary

In the current repository state this package is non-physical. `write()` does
not POST to the ESP32 controller, and no STM32, motor, gripper, camera, or light
is contacted by this plugin.

Physical actuation remains behind:

- ESP32 firmware `SafetyGate`
- `BOOT -> IDLE -> ARMED -> FAULT` state boundary
- token-gated HTTP commands
- deadman/teleop timeout paths
- routine executor policy and feedback readiness checks

Use this wording in applications:

```text
Implemented a safe open-loop ros2_control SystemInterface scaffold for the
MotionBrain arm. It validates the ROS2 hardware boundary and controller
integration, while physical motion remains guarded by ESP32 firmware.
```

Do not claim:

```text
Closed-loop trajectory execution, DYNAMIXEL integration, encoder-verified
motion, humanoid control, or production hardware interface completion.
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

## Future Physical Backend

A future ESP32 backend should be added behind an explicit mode flag and should
forward only bounded, firmware-accepted commands. Because the current arm has no
per-joint encoders, trajectory completion must be treated as command acceptance
and estimated motion, not physical convergence.
