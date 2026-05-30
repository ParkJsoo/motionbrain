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
  -> JSON/typed topics + /joint_states + TF + kinematics + C++ guard + mission state
```

## Prerequisites

Build the ROS2 workspace on the Pi:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description
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

To capture public-safe terminal evidence in one pass:

```bash
cd ~/develop/arduino/motionbrain
tools/raspi/capture_ros2_evidence.sh
```

The default mode records service, health, package/interface/topic inventory,
typed topic samples, and JSON compatibility samples without publishing actuator
commands. Use `CAPTURE_MISSION_BOUNDARY=1` only when the safe mission
`start`/`reset` boundary should be included.

Expected:

```text
OK service active: motionbrain-ros-bridge.service
OK topic: /motionbrain/status_typed
OK topic: /camera/detection_typed
OK topic: /joint_states
OK topic: /motionbrain/end_effector_pose
OK topic: /motionbrain/kinematics_typed
OK topic: /motionbrain/control_guard_typed
OK topic: /motionbrain/mission_state_typed
OK status typed sample
OK camera detection typed sample
OK joint state sample
OK end-effector pose sample
OK kinematics typed sample
OK control guard typed sample
OK mission state typed sample
```

## 2026-05-27 Pi Validation Result

The systemd deployment path was installed and validated on the Raspberry Pi 4.

- Installed `/etc/motionbrain/ros-bridge.env` with mode `600`.
- Installed `/etc/systemd/system/motionbrain-ros-bridge.service`.
- `systemctl enable motionbrain-ros-bridge.service` succeeded.
- After `systemctl restart motionbrain-ros-bridge.service`, the service was
  `active (running)`.
- `motionbrain_status_node` and `motionbrain_joint_state_node` ran under the
  systemd service cgroup.
- `CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh` returned:

```text
OK service active: motionbrain-ros-bridge.service
OK topic: /motionbrain/status_typed
OK topic: /camera/detection_typed
OK topic: /joint_states
OK topic: /motionbrain/end_effector_pose
OK topic: /motionbrain/kinematics
OK status typed sample
OK camera detection typed sample
OK joint state sample
OK end-effector pose sample
OK kinematics sample
```

During validation, `set -u` conflicted with optional environment variable
references inside the ROS2 setup files. `start_ros_bridge.sh` and
`check_ros_bridge_health.sh` now enable `set -u` only after sourcing the ROS
environment.

## 2026-05-28 C++ Control Guard Validation Result

The C++ ROS2 control guard was validated through the Raspberry Pi 4 systemd
path.

- `colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description`
  succeeded.
- Ran `systemctl daemon-reload`, then restarted
  `motionbrain-ros-bridge.service`.
- Service state: `active (running)`.
- The service cgroup contained:
  - `motionbrain_status_node`
  - `motionbrain_joint_state_node`
  - `motionbrain_kinematics_node`
  - `motionbrain_control_guard_node`
- `/motionbrain/control_guard` sample:
  - `ready=true`
  - `reason=ready`
  - `statusFresh=true`
  - `detectionFresh=true`
- `CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh` passed the
  `/motionbrain/control_guard` topic and sample checks. Add
  `STRICT_CAMERA_AVAILABLE=1` when the check must fail unless
  `/camera/detection_typed.available` is `true`.

## 2026-05-28 Mission Supervisor Validation Result

The lightweight mission supervisor was validated through the Raspberry Pi 4
systemd path.

- `colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description`
  succeeded.
- After restarting `motionbrain-ros-bridge.service`, the service was
  `active (running)`.
- The service cgroup contained:
  - `motionbrain_status_node`
  - `motionbrain_joint_state_node`
  - `motionbrain_kinematics_node`
  - `motionbrain_control_guard_node`
  - `motionbrain_mission_supervisor`
- `/motionbrain/mission_state` sample:
  - `state=IDLE`
  - `reason=idle`
  - guard/status/detection freshness fields
- `CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh` passed the
  `/motionbrain/mission_state` topic and sample checks.
- Only safe command boundaries were checked:
  - publishing `start` to `/motionbrain/mission_cmd` moved the mission to
    `WAIT_DETECTION`
  - publishing `reset` to `/motionbrain/mission_cmd` returned the mission to
    `IDLE`

The `confirm` command was intentionally not run during this validation because
it can publish a real `/motionbrain/light_cmd_typed` command. The current
mission supervisor is a structured ROS2 mission layer for detection, alignment
decision, and operator confirmation, not an autonomous motion controller.

## 2026-05-30 Typed Interface Cleanup Validation Result

The typed guard, mission, and kinematics topic path was validated through the
Raspberry Pi 4 systemd service.

- Commit: `2874df7 Use typed ROS2 guard and mission topics`
- Updated `/etc/motionbrain/ros-bridge.env` to the current Home Wi-Fi IPs:
  - `MOTIONBRAIN_HOST=192.168.219.110`
  - `MOTIONBRAIN_CAMERA_URL=http://192.168.219.113`
- After restarting `motionbrain-ros-bridge.service`, the service was
  `active (running)`.
- The service cgroup contained:
  - `motionbrain_status_node`
  - `motionbrain_joint_state_node`
  - `motionbrain_kinematics_node`
  - `motionbrain_control_guard_node`
  - `motionbrain_mission_supervisor`
- `CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh` passed typed topic
  and sample checks for:
  - `/motionbrain/status_typed`
  - `/camera/detection_typed`
  - `/joint_states`
  - `/motionbrain/end_effector_pose`
  - `/motionbrain/kinematics_typed`
  - `/motionbrain/control_guard_typed`
  - `/motionbrain/mission_state_typed`
- Public-safe text evidence:
  [docs/evidence/2026-05-30-ros2-typed-systemd.md](evidence/2026-05-30-ros2-typed-systemd.md)

## 2026-05-30 Evidence Helper Validation Result

The public-safe ROS2 evidence helper was validated on the Raspberry Pi 4 while
the systemd service was running.

- Commit: `99154d2 Fix ROS2 evidence interface listing`
- Script: `tools/raspi/capture_ros2_evidence.sh`
- Default mode does not publish actuator commands.
- The first validation exposed a grep issue because `ros2 interface list`
  prefixes interface names with spaces; `99154d2` fixed the check by matching
  `motionbrain_msgs/msg` anywhere on the line.
- Corrected output file:
  `/tmp/motionbrain_ros2_evidence_helper_99154d2.txt`
- Final result: `Result: OK`
- Pi repo state: clean at `99154d2`
- `motionbrain-ros-bridge.service`: `active`

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
