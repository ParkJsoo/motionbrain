# MotionBrain ros2_control Mock

Optional mock-hardware demo for MotionBrain. This package uses
`mock_components/GenericSystem` and does not connect to the ESP32 controller,
STM32 teleop board, motors, gripper, or light.

Install the ROS2 control runtime packages on a ROS2 Jazzy host before running:

```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-ros2-control-test-assets \
  ros-jazzy-ros2controlcli
```

Build the optional package:

```bash
cd ros2_ws
colcon build --packages-select motionbrain_ros2_control_mock
source install/setup.bash
```

Launch the mock hardware:

```bash
ros2 launch motionbrain_ros2_control_mock mock_control.launch.py
```

Inspect controllers:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Capture a repeatable mock-only evidence bundle on the Raspberry Pi:

```bash
tools/raspi/capture_ros2_control_mock_evidence.sh
```

If a temporary non-root ROS package overlay is being used for evidence capture,
point the helper at that prefix:

```bash
MOTIONBRAIN_ROS2_CONTROL_OVERLAY_PREFIX=$HOME/.local/motionbrain_ros2_control_overlay/opt/ros/jazzy \
  tools/raspi/capture_ros2_control_mock_evidence.sh
```

Send a mock-only joint trajectory:

```bash
ros2 topic pub --once /motionbrain_arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [base_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, gripper_joint], points: [{positions: [0.2, 0.1, -0.1, 0.05, 0.0], time_from_start: {sec: 2}}]}"
```

This is portfolio evidence for ROS2 controller architecture only. It is not a
physical hardware interface and must not be presented as encoder-grade control
of the current DC arm.
