# Raspberry Pi Deployment

[한국어](RASPBERRY_PI_DEPLOYMENT.md)

This document describes how to run the MotionBrain ROS2 bridge on the Raspberry
Pi as a systemd service. Do not store real Wi-Fi passwords or the real
`MOTIONBRAIN_HTTP_TOKEN` in the repository.

## Goal

Replace manual terminal launch with an operational boundary:

```text
systemd
  -> /etc/motionbrain/ros-bridge.env
  -> tools/raspi/start_ros_bridge.sh
  -> ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
  -> JSON/typed topics + /joint_states + TF display path
```

## Prerequisites

Build the ROS2 workspace on the Pi:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_ros_bridge motionbrain_description
source install/setup.bash
```

## Install Environment File

```bash
sudo mkdir -p /etc/motionbrain
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-ros-bridge.env.example \
  /etc/motionbrain/ros-bridge.env
sudo chmod 600 /etc/motionbrain/ros-bridge.env
sudo nano /etc/motionbrain/ros-bridge.env
```

Set:

- `MOTIONBRAIN_HOST`
- `MOTIONBRAIN_CAMERA_URL`
- `MOTIONBRAIN_HTTP_TOKEN`

Use hostnames when `.local` works. If mDNS is unstable on the Pi, reserve stable
DHCP addresses in the router and use those IPs.

## Install Service

```bash
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-ros-bridge.service \
  /etc/systemd/system/motionbrain-ros-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable motionbrain-ros-bridge.service
sudo systemctl start motionbrain-ros-bridge.service
```

## Check Status

```bash
systemctl status motionbrain-ros-bridge.service --no-pager
journalctl -u motionbrain-ros-bridge.service -n 80 --no-pager
```

Health check:

```bash
~/develop/arduino/motionbrain/tools/raspi/check_ros_bridge_health.sh
```

Expected:

```text
OK service active: motionbrain-ros-bridge.service
OK topic: /motionbrain/status_typed
OK topic: /camera/detection_typed
OK topic: /joint_states
OK status typed sample
OK joint state sample
```

## Operations

```bash
sudo systemctl restart motionbrain-ros-bridge.service
sudo systemctl stop motionbrain-ros-bridge.service
sudo systemctl disable motionbrain-ros-bridge.service
```

## Troubleshooting

Restart after editing the environment file:

```bash
sudo systemctl restart motionbrain-ros-bridge.service
```

If startup fails:

```bash
journalctl -u motionbrain-ros-bridge.service -n 120 --no-pager
```

Token errors appear as `HTTP Error 403: Forbidden` on `/motionbrain/light_result`.
Verify that `MOTIONBRAIN_HTTP_TOKEN` in `/etc/motionbrain/ros-bridge.env`
matches the token provisioned on the ESP32.
