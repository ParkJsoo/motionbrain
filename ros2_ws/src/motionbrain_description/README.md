# motionbrain_description

URDF, RViz, and `robot_state_publisher` launch assets for MotionBrain.

The model is intentionally lightweight: it represents the current 5-axis
MotionBrain arm as a ROS2 kinematic tree so TF, `joint_states`, and RViz can be
used in the portfolio path before full encoder feedback exists.

## Build

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_ros_bridge motionbrain_description
source install/setup.bash
```

## Run

Start the ESP32 bridge first:

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=<controller-ip> \
  camera_url:=http://<camera-ip>
```

Then start the robot model and TF publisher:

```bash
ros2 launch motionbrain_description display.launch.py
```

On a desktop with RViz2 installed:

```bash
ros2 launch motionbrain_description display.launch.py use_rviz:=true
```

Useful checks:

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /motionbrain/end_effector_pose --once
ros2 topic echo /motionbrain/kinematics --once
ros2 run tf2_tools view_frames
```

The FK pose and kinematics JSON come from `motionbrain_kinematics_node` in the
bridge package. The display launch remains focused on URDF/TF/RViz; the home
Wi-Fi bridge launch starts the kinematics node by default.
