# 2026-06-12 ros2_control Mock Runtime Evidence

This note summarizes public-safe text evidence captured from the Raspberry Pi
after adding the optional MotionBrain `ros2_control` mock hardware demo.

## Runtime

- Host: `motionbrain-pi`
- Pi address used from the Mac: `192.168.219.110`
- OS: Ubuntu 24.04.4 LTS, Linux `6.8.0-1057-raspi`, `aarch64`
- ROS2: `jazzy`
- Git commit during capture: `76cc662 Stabilize ros2_control mock evidence capture`
- Service state during post-capture check: `motionbrain-ros-bridge.service`
  `active`
- Controller checked after capture: `192.168.219.108`
- ESP32-CAM observed in the current bring-up path: `http://192.168.219.109`

## Runtime Package Boundary

Noninteractive `sudo` was not available in this session:

```text
sudo: a password is required
```

The ROS apt packages were available from the configured Jazzy package source,
but were not installed system-wide. To complete a non-root runtime check, the
required ROS `.deb` packages were downloaded and extracted under a user-local
temporary overlay:

```text
/home/motionbrain/.local/motionbrain_ros2_control_overlay/opt/ros/jazzy
```

This overlay was supplied only to the mock evidence helper through
`MOTIONBRAIN_ROS2_CONTROL_OVERLAY_PREFIX`. The preferred persistent setup is
still the normal apt installation documented in the mock package README.

The helper confirmed the required runtime packages through ROS package lookup:

```text
OK package: controller_manager
OK package: hardware_interface
OK package: joint_state_broadcaster
OK package: joint_trajectory_controller
OK package: ros2_control
OK package: ros2_control_test_assets
OK package: ros2controlcli
OK package: motionbrain_ros2_control_mock
```

## Build And Capture

The Pi repository was fast-forwarded to `76cc662`, then the optional mock
package was rebuilt:

```bash
cd ros2_ws
colcon build --packages-select motionbrain_ros2_control_mock
```

Build result:

```text
Summary: 1 package finished
```

The mock-only evidence helper was run with a separate ROS domain and the
temporary overlay prefix:

```bash
MOTIONBRAIN_ROS2_CONTROL_OVERLAY_PREFIX=$HOME/.local/motionbrain_ros2_control_overlay/opt/ros/jazzy \
CAPTURE_MOCK_TRAJECTORY=1 \
SAMPLE_TIMEOUT_SECONDS=20 \
MOCK_STARTUP_TIMEOUT_SECONDS=45 \
MOTIONBRAIN_EVIDENCE_OUTPUT=/tmp/motionbrain_ros2_control_mock_evidence.txt \
tools/raspi/capture_ros2_control_mock_evidence.sh
```

Captured result:

```text
Output: /tmp/motionbrain_ros2_control_mock_evidence.txt
Launch log: /tmp/motionbrain_ros2_control_mock_launch_20260612_235650.log
Result: OK
```

The raw text capture is kept locally under
`.codex/tmp/evidence/2026-06-12-ros2-control-mock.txt`. The launch log is kept
locally under
`.codex/tmp/evidence/2026-06-12-ros2-control-mock-launch.log`.

## Controller Evidence

The mock launch loaded `mock_components/GenericSystem`:

```text
Loaded hardware 'MotionBrainMockSystem' from plugin 'mock_components/GenericSystem'
Successful initialization of hardware 'MotionBrainMockSystem'
Successful 'activate' of hardware 'MotionBrainMockSystem'
```

Both controllers were active:

```text
motionbrain_arm_controller joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster    joint_state_broadcaster/JointStateBroadcaster          active
```

Position command interfaces were available and claimed by the trajectory
controller:

```text
command interfaces
  base_yaw_joint/position [available] [claimed]
  elbow_pitch_joint/position [available] [claimed]
  gripper_joint/position [available] [claimed]
  shoulder_pitch_joint/position [available] [claimed]
  wrist_pitch_joint/position [available] [claimed]
```

The initial mock `/joint_states` sample reported all five joints at zero:

```text
name:
- base_yaw_joint
- elbow_pitch_joint
- gripper_joint
- shoulder_pitch_joint
- wrist_pitch_joint
position:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
```

After publishing a mock-only trajectory, the mock joint state changed:

```text
position:
- 0.2
- -0.1
- 0.0
- 0.1
- 0.05
```

The joint order in that sample was base yaw, elbow pitch, gripper, shoulder
pitch, and wrist pitch.

## Safety Boundary

The mock launch used `ROS_DOMAIN_ID=42`, separate from the running MotionBrain
bridge domain. It did not connect to the ESP32 controller, STM32 teleop board,
motors, gripper, or light. The only trajectory command was published to the
mock `joint_trajectory_controller` topic in that isolated ROS domain.

The helper stopped the mock launch after capture. A process check showed no
remaining mock launch, `ros2_control_node`, or helper processes.

A post-capture controller status check showed the physical controller still in
a safe read-only state:

```text
state: IDLE
motorEnabled: false
all motor speeds: 0
all motors enabled: false
light: false
faultLatched: false
faultReason: NONE
dryRunOnly: true
executeImplemented: false
executor.enabled: false
executor.queueApplyAllowed: false
executor.status.state: idle
executor.status.lastResult: not_requested
```
