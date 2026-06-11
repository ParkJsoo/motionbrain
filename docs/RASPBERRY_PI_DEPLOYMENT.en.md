# Raspberry Pi Deployment

[한국어](RASPBERRY_PI_DEPLOYMENT.md)

This document describes how to run the MotionBrain ROS2 bridge, perception
service, and dashboard on the Raspberry Pi as systemd services. Do not store
real Wi-Fi passwords or the real `MOTIONBRAIN_HTTP_TOKEN` in the repository.

## Goal

Replace manual terminal launch with an operational boundary:

```text
systemd
  -> motionbrain-ros-bridge.service
     -> /etc/motionbrain/ros-bridge.env
     -> tools/raspi/start_ros_bridge.sh
     -> ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
     -> JSON/typed topics + /joint_states + TF + kinematics + C++ guard + mission state
  -> motionbrain-perception.service
     -> /etc/motionbrain/perception.env
     -> tools/raspi/start_perception_service.sh
     -> ESP32-CAM capture + target detection API
  -> motionbrain-dashboard.service
     -> /etc/motionbrain/dashboard.env
     -> tools/raspi/start_dashboard_service.sh
     -> LAN dashboard at http://<pi-ip>:8765
```

## Prerequisites

Build the ROS2 workspace on the Pi:
The systemd units in this document assume the `motionbrain` user and a checkout
at `/home/motionbrain/develop/arduino/motionbrain`. If you use another user or
path, update the units and env-file paths together.

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
- `MOTIONBRAIN_PERCEPTION_URL` if the same Pi perception service should provide
  `/camera/detection` instead of direct ESP32-CAM polling

Use hostnames when `.local` works. If mDNS is unstable on the Pi, reserve stable
DHCP addresses in the router and use those IPs.

For object detection or tracked camera overlay through the Pi perception
service, use the same-Pi endpoint
`MOTIONBRAIN_PERCEPTION_URL=http://127.0.0.1:8766`. Leave it empty to keep the
original bridge behavior, where ROS2 polls `MOTIONBRAIN_CAMERA_URL/capture`
directly and runs color detection. Use the Pi LAN IP only when
`MOTIONBRAIN_PERCEPTION_HOST=0.0.0.0` is intentionally enabled for direct LAN
clients.

## Install Dashboard / Perception Environment Files

```bash
sudo mkdir -p /etc/motionbrain
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-perception.env.example \
  /etc/motionbrain/perception.env
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-dashboard.env.example \
  /etc/motionbrain/dashboard.env
sudo chmod 600 /etc/motionbrain/perception.env /etc/motionbrain/dashboard.env
sudo nano /etc/motionbrain/perception.env
sudo nano /etc/motionbrain/dashboard.env
```

Set:

- `MOTIONBRAIN_CAMERA_URL`
- `MOTIONBRAIN_MOTION_HOST`
- `MOTIONBRAIN_HTTP_TOKEN`
- `MOTIONBRAIN_OBJECT_MODEL`
- `MOTIONBRAIN_OBJECT_LABELS`
- `MOTIONBRAIN_OBJECT_TARGET`

The default setup prefers `motionbrain.local`, `motionbrain-cam.local`, and
`motionbrain-pi.local`. If mDNS is unstable, the ROS2 bridge, dashboard, and
perception service wrappers scan local `/status` endpoints and automatically
resolve the current controller and ESP32-CAM IPs. A reconcile timer also checks
once per minute that dashboard/perception/ROS2 bridge still match the currently
discovered device IPs, and restarts the required services when the ESP32 boards
are power-cycled while the Pi stays online. This avoids editing env files or
RViz bridge inputs every time the ESP32 boards receive new DHCP addresses.

The perception API binds to Pi-local `127.0.0.1:8766`; only the dashboard is
exposed on the LAN as `0.0.0.0:8765`. Open
`http://motionbrain-pi.local:8765` in the browser, and use
router DNS or `http://<pi-ip>:8765` only when mDNS is unavailable. If
Mac/browser `.local` lookup is polluted by a public IP result, open the
dashboard by Pi IP and set the ESP32 Control `API` field to the Pi IP or router
DNS. Control `STREAM` reads the current camera URL from the Pi dashboard
`/api/config` endpoint and automatically corrects the default
`motionbrain-cam.local` value.

Set `MOTIONBRAIN_DISCOVERY=0` in `/etc/motionbrain/ros-bridge.env`,
`/etc/motionbrain/perception.env`, and `/etc/motionbrain/dashboard.env` to
disable discovery fallback. Set a specific scan subnet with a value such as
`MOTIONBRAIN_DISCOVERY_CIDR=192.168.219.0/24`.

For the current cup known-object demo, use `MOTIONBRAIN_OBJECT_TARGET=cup`,
`MOTIONBRAIN_OBJECT_MIN_CONFIDENCE=0.25`, and
`MOTIONBRAIN_DISPLAY_HOLD_SECONDS=1.5`. The ESP32-CAM profile should also stay
at `MOTIONBRAIN_CAMERA_FRAMESIZE=qvga` and `MOTIONBRAIN_CAMERA_QUALITY=10`. The
service wrappers apply this profile on startup, and the reconcile timer
re-applies it and restarts dashboard/perception when an ESP32-CAM reboot resets
the profile. Set `MOTIONBRAIN_CAMERA_PROFILE=0` when using another camera. Keep
aliases empty as the baseline; add a known-mislabel alias only when the current
white-cup view flickers into a nearby COCO label.

## Install ROS2 Bridge Service

```bash
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-ros-bridge.service \
  /etc/systemd/system/motionbrain-ros-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable motionbrain-ros-bridge.service
sudo systemctl start motionbrain-ros-bridge.service
```

## Install Dashboard / Perception Services

```bash
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-perception.service \
  /etc/systemd/system/motionbrain-perception.service
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-dashboard.service \
  /etc/systemd/system/motionbrain-dashboard.service
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-dashboard-reconcile.service \
  /etc/systemd/system/motionbrain-dashboard-reconcile.service
sudo cp ~/develop/arduino/motionbrain/deploy/systemd/motionbrain-dashboard-reconcile.timer \
  /etc/systemd/system/motionbrain-dashboard-reconcile.timer
sudo systemctl daemon-reload
sudo systemctl enable --now motionbrain-perception.service
sudo systemctl enable --now motionbrain-dashboard.service
sudo systemctl enable --now motionbrain-dashboard-reconcile.timer
```

`motionbrain-dashboard.service` starts after `motionbrain-perception.service`.
If perception temporarily fails, the dashboard remains available while the
restart policy recovers the companion service.

## Check Status

```bash
systemctl status motionbrain-ros-bridge.service --no-pager
systemctl status motionbrain-perception.service --no-pager
systemctl status motionbrain-dashboard.service --no-pager
systemctl status motionbrain-dashboard-reconcile.timer --no-pager
journalctl -u motionbrain-ros-bridge.service -n 80 --no-pager
journalctl -u motionbrain-perception.service -n 80 --no-pager
journalctl -u motionbrain-dashboard.service -n 80 --no-pager
journalctl -u motionbrain-dashboard-reconcile.service -n 80 --no-pager
```

ROS2 health check:

```bash
~/develop/arduino/motionbrain/tools/raspi/check_ros_bridge_health.sh
```

Dashboard/perception health check:

```bash
CHECK_SERVICE=1 ~/develop/arduino/motionbrain/tools/raspi/check_dashboard_health.sh
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

## 2026-06-04 Pi Dashboard / Perception Validation Result

The Raspberry Pi dashboard and perception service were validated as companion
processes next to the ROS2 bridge for the current camera-mode split.

- Controller: `192.168.219.111`
- ESP32-CAM: `192.168.219.113`
- Raspberry Pi: `192.168.219.114`
- ESP32-CAM profile: `qvga`, JPEG quality `4` in that dated bench check;
  current service wrappers default to JPEG quality `10` for live capture
  stability.
- Perception service: Pi port `8766`, object mode, OpenCV DNN YOLOv5s, target
  `cup`, then-current confidence gate `0.5`, display hold `1.5s`
- Dashboard: Pi port `8765`, `--perception-url http://127.0.0.1:8766`
- Result: dashboard `/api/detection` returned `label=cup` above the configured
  threshold in that scene.
- Browser check: `motionbrain.local`, the controller IP page, and
  `http://192.168.219.114:8765` were opened and visible to the operator.

This validates the current operating split: `STREAM` for responsive manual
camera feedback, `TRACKED` for slower fixed/slow-target recognition checks.

## Operations

```bash
sudo systemctl restart motionbrain-ros-bridge.service
sudo systemctl restart motionbrain-perception.service
sudo systemctl restart motionbrain-dashboard.service
sudo systemctl restart motionbrain-dashboard-reconcile.timer
sudo systemctl stop motionbrain-ros-bridge.service
sudo systemctl stop motionbrain-perception.service
sudo systemctl stop motionbrain-dashboard.service
sudo systemctl stop motionbrain-dashboard-reconcile.timer
sudo systemctl disable motionbrain-ros-bridge.service
sudo systemctl disable motionbrain-perception.service
sudo systemctl disable motionbrain-dashboard.service
sudo systemctl disable motionbrain-dashboard-reconcile.timer
```

## Troubleshooting

Restart after editing the environment file:

```bash
sudo systemctl restart motionbrain-ros-bridge.service
sudo systemctl restart motionbrain-perception.service
sudo systemctl restart motionbrain-dashboard.service
sudo systemctl restart motionbrain-dashboard-reconcile.timer
```

If startup fails:

```bash
journalctl -u motionbrain-ros-bridge.service -n 120 --no-pager
journalctl -u motionbrain-perception.service -n 120 --no-pager
journalctl -u motionbrain-dashboard.service -n 120 --no-pager
journalctl -u motionbrain-dashboard-reconcile.service -n 120 --no-pager
```

Token errors appear as `HTTP Error 403: Forbidden` on `/motionbrain/light_result`.
Verify that `MOTIONBRAIN_HTTP_TOKEN` in `/etc/motionbrain/ros-bridge.env` and
`/etc/motionbrain/dashboard.env` match the token provisioned on the ESP32.
To rotate only the ESP32-side token to match the Pi environment, run
`wifi token <new-command-token>` in the controller serial monitor. The command
preserves the stored Wi-Fi SSID/password, updates only the NVS token field, and
reboots the controller.
