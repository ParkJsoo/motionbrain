# Operations

[README](README.md) | [EMBEDDED_BRINGUP](EMBEDDED_BRINGUP.md)

This is the operator-oriented view of MotionBrain on Raspberry Pi. It is useful
for system-quality, Linux operations, and troubleshooting discussions.

## Pi Access / SSH

Use a hostname-based SSH alias for the Pi. Do not pin the alias to a DHCP IP:
that was the recurring failure mode when the Pi moved from an old address to
`192.168.219.114` while the Mac still tried the stale `192.168.219.110`.

Recommended `~/.ssh/config` entry on the Mac:

```sshconfig
Host motionbrain-pi motionbrain-pi.local motionbrain-pi.davolink
    HostName motionbrain-pi.local
    User motionbrain
    HostKeyAlias motionbrain-pi.local
    AddressFamily inet
```

First check from the Mac:

```bash
python3 tools/raspi/check_pi_ssh_target.py
ssh -o ConnectTimeout=5 motionbrain-pi 'hostname; hostname -I; systemctl is-active ssh'
nc -vz motionbrain-pi.local 22
nc -vz motionbrain-pi.davolink 22
```

Interpretation:

- `motionbrain-pi.local` is the primary Pi SSH target.
- `motionbrain-pi.davolink` is only a router-DNS fallback; it may disappear
  even while `.local` and SSH still work.
- `motionbrain.local` is the ESP32 motion controller, not the Pi.
- `motionbrain-cam.local` is the ESP32-CAM, not the Pi.
- `tools/raspi/discover_device_url.py` discovers ESP32 controller/camera HTTP
  `/status` endpoints. It is not a Pi SSH discovery tool.

On the Pi, Ubuntu 24.04 can expose SSH through `ssh.socket`; `ssh.service` may
show `disabled` while `ssh.socket` is `enabled` and `active`. Verify both before
changing systemd state:

```bash
systemctl is-active ssh
systemctl is-enabled ssh.socket
systemctl is-active ssh.socket
ss -ltnp | grep ':22 '
```

If DNS works but SSH warns about a changed host key after a fresh OS install,
remove only the stale Pi key and reconnect:

```bash
ssh-keygen -R motionbrain-pi.local
ssh-keygen -R motionbrain-pi.davolink
```

## Runtime Services

Systemd units live in `deploy/systemd/`:

| Service | Purpose | Env file |
| --- | --- | --- |
| `motionbrain-perception.service` | Pi perception API for camera detection | `/etc/motionbrain/perception.env` |
| `motionbrain-dashboard.service` | Pi dashboard and controller proxy | `/etc/motionbrain/dashboard.env` |
| `motionbrain-ros-bridge.service` | ROS2 Jazzy bridge and typed topics | `/etc/motionbrain/ros-bridge.env` |
| `motionbrain-dashboard-reconcile.timer` | periodic dashboard/perception reconciliation | service-specific env |

Example install flow on the Pi:

```bash
tools/raspi/install_systemd_units.sh
sudo systemctl enable --now motionbrain-perception.service
sudo systemctl enable --now motionbrain-dashboard.service
sudo systemctl enable --now motionbrain-ros-bridge.service
```

The installer copies service/timer units and writes env examples under
`/etc/motionbrain/`. Override `MOTIONBRAIN_REPO` or `MOTIONBRAIN_SERVICE_USER`
when installing from a different checkout path or Linux account. Edit copied
env files before enabling services on real hardware. Keep
`MOTIONBRAIN_HTTP_TOKEN` out of git, logs, screenshots, and public demos.
Dashboard-side controls and ESP32 command tokens are separate boundaries: the
dashboard can observe read-only state without exposing the controller token, and
state-changing requests must still pass the controller firmware gate.

For object-mode perception on the Raspberry Pi, keep CPU load bounded with
`MOTIONBRAIN_OPENCV_THREADS`, `MOTIONBRAIN_PERCEPTION_INTERVAL`, and
`MOTIONBRAIN_PERCEPTION_STALE_SECONDS` in `/etc/motionbrain/perception.env`.
The default systemd wrapper uses one OpenCV worker thread and a conservative
freshness window for YOLOv5s/OpenCV DNN.

For ESP32 controller/camera `.local` or IP drift, use the discovery and reconcile
helpers on the Pi instead of hard-coding stale device addresses:

```bash
python3 tools/raspi/discover_device_url.py --help
tools/raspi/reconcile_dashboard_services.sh
```

## Health Checks

Dashboard/perception:

```bash
CHECK_SERVICE=1 tools/raspi/check_dashboard_health.sh
```

If the ESP32 controller or ESP32-CAM is intentionally powered off, keep the
dashboard service check separate from motion readiness:

```bash
ALLOW_DASHBOARD_DEGRADED=1 CHECK_SERVICE=1 tools/raspi/check_dashboard_health.sh
```

This accepts a structured dashboard `degraded` response for read-only
observability, but it is not a motion-ready preflight.

ROS2 bridge:

```bash
CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh
```

The ROS2 bridge check validates discovery first and then samples multiple
topics, diagnostics, managed lifecycle state, one service, and one action
sequentially. It also verifies that a physical `run` request is rejected by the
ROS2 bridge policy unless `CHECK_ROUTINE_RUN_REJECTION=0` is set. On the Pi this
can take a few minutes. If topic discovery passes but samples do not arrive
after the configured deadline, inspect recent bridge logs and restart the
service:

```bash
journalctl -u motionbrain-ros-bridge.service -n 120 --no-pager
sudo systemctl restart motionbrain-ros-bridge.service
CHECK_SERVICE=1 SAMPLE_TIMEOUT_SECONDS=25 tools/raspi/check_ros_bridge_health.sh
```

The default diagnostic thresholds allow the current read-only bring-up state:
M4 shoulder feedback may be `WARN` when the measured angle is outside calibrated
limits, and routine feedback may be `WARN` while `base_yaw_reference` is not
installed. For a motion-ready preflight, require clean diagnostics explicitly:

```bash
EXPECTED_SHOULDER_DIAGNOSTIC_MAX_LEVEL=0 \
EXPECTED_FEEDBACK_DIAGNOSTIC_MAX_LEVEL=0 \
CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh
```

ROS2 evidence capture:

```bash
tools/raspi/capture_ros2_evidence.sh
```

ros2_control mock evidence:

```bash
tools/raspi/capture_ros2_control_mock_evidence.sh
```

ros2_control open-loop hardware-interface evidence:

```bash
tools/raspi/capture_ros2_control_hardware_evidence.sh
```

## Logs

```bash
systemctl status motionbrain-perception.service --no-pager
systemctl status motionbrain-dashboard.service --no-pager
systemctl status motionbrain-ros-bridge.service --no-pager

journalctl -u motionbrain-perception.service -n 100 --no-pager
journalctl -u motionbrain-dashboard.service -n 100 --no-pager
journalctl -u motionbrain-ros-bridge.service -n 100 --no-pager
```

## ROS2 Topics To Check

Read-only topics expected from the bridge:

- `/motionbrain/status_typed`
- `/motionbrain/routine_typed`
- `/motionbrain/diagnostics`
- `/motionbrain/events_typed`
- `/camera/detection_typed`
- `/joint_states`
- `/motionbrain/end_effector_pose`
- `/motionbrain/kinematics_typed`
- `/motionbrain/control_guard_typed`
- `/motionbrain/mission_state_typed`

Sample commands:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /motionbrain/status_typed --once
ros2 topic echo /motionbrain/control_guard_typed --once
```

## HTTP Surfaces

ESP32 controller:

- `GET /status`
- `GET /events`
- `GET /routine`
- `POST /command`
- `POST /routine`

ESP32-CAM:

- `GET /status`
- `GET /capture`
- `GET /stream`
- `POST /camera`

Pi services:

- dashboard `GET /api/config`
- dashboard `GET /api/status`
- perception `GET /health`
- perception `GET /api/detection`

State-changing HTTP calls require the local command token where implemented.
Use read-only endpoints for evidence capture unless a physical operator is
present.

## Recovery Playbook

| Symptom | First checks | Recovery |
| --- | --- | --- |
| dashboard unavailable | `check_dashboard_health.sh`, service status | restart dashboard and perception services |
| Pi hostname not resolving | `python3 tools/raspi/check_pi_ssh_target.py --skip-remote`, router DHCP lease, mDNS | use `motionbrain-pi.local` first; treat `.davolink` as fallback |
| Pi port 22 closed | `nc -vz motionbrain-pi.davolink 22`, `ssh.socket` status on console | enable/start `ssh.socket` or `ssh.service` |
| SSH alias reaches old IP | `ssh -G motionbrain-pi`, compare with router/`hostname -I` | remove literal `HostName` IP from `~/.ssh/config` |
| SSH auth denied | `ssh -vv motionbrain-pi`, key path, user | use `User motionbrain` and the expected key |
| SSH host key changed | SSH warning host, recent OS reinstall | `ssh-keygen -R` only the stale Pi hostname/IP |
| SSH works but services fail | `systemctl status ...`, repo path, env files | fix `/home/motionbrain/...` path or service env |
| ROS2 topics missing | `check_ros_bridge_health.sh`, `ros2 topic list` | restart ROS bridge, confirm ROS workspace overlay |
| ROS2 topics listed but samples hang | bridge journal, controller `/routine`, sample timeout | restart ROS bridge, then rerun with `SAMPLE_TIMEOUT_SECONDS=25` |
| camera stale | ESP32-CAM `/status`, Wi-Fi, perception logs | apply camera profile and restart perception |
| Pi load high | `uptime`, `vcgencmd get_throttled`, perception `/health` latency | lower OpenCV threads or increase perception interval, then restart perception |
| controller command rejected | `/status`, fault latch, token, ARMED state | clear fault only after physical inspection |
| routine blocked | `/routine`, feedback readiness, active sequence | keep dry-run, inspect feedback block reason |
| token missing | service env file, dashboard status, ESP32 command rejection | restore `/etc/motionbrain/*.env`, restart service |
| perception stale | perception `/health`, `/api/detection`, camera URL | restart perception, verify camera profile |
| feedback not installed | `/motionbrain/routine_typed`, diagnostics `base_yaw_fault` | keep physical routine execution disabled |

## QA Matrix

| Layer | Verification |
| --- | --- |
| ESP32 firmware | `pio run` |
| ESP32-CAM firmware | `pio run -d firmware/esp32cam` |
| Host contracts | `python3 -m unittest discover -s tests` |
| ROS2 workspace | GitHub Actions Jazzy container `colcon build/test` |
| Pi services | `check_dashboard_health.sh`, `check_ros_bridge_health.sh` |
| Physical demo | operator-held teleop, deadman release, bounded motion evidence |

Residual risk: this is a personal robotics system, not a production fleet. The
right operational claim is CI/systemd/health-check experience for a real
hardware prototype, not enterprise-scale SRE or commercial production service
operation.

Demo boundary: dashboard alignment nudges are token- and safety-gated bounded
commands, not autonomous visual servoing. The current cup path is an
operator-confirmed dry-run plan, not a physical grasp demo.
