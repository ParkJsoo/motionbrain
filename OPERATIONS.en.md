# Build and Operations

[한국어](OPERATIONS.md) · [README](README.en.md) · [Pin map](PIN_MAP.en.md)

These are general repository reproduction steps. Device addresses, credentials and joint calibration belong to each installation. Passing builds/tests does not upload firmware or actuate motors.

## Builds

ESP32 uses PlatformIO. Host tests require Python, NumPy and OpenCV. Replace `<...>` placeholders with installation-specific values. Run from the repository root:

```sh
python3 -m venv .local/venv
.local/venv/bin/python -m pip install numpy opencv-python-headless
.local/venv/bin/python -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

The STM32 project is under `firmware/stm32/MotionBrainSensor/`. Configure local STM32CubeIDE and build with `tools/stm32_build.sh`. Upload helpers are separate from building.

In a ROS2 Jazzy environment:

```sh
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_hardware_interface motionbrain_mission motionbrain_ros_bridge motionbrain_description motionbrain_ros2_control_mock
source install/setup.bash
colcon test --packages-select motionbrain_msgs motionbrain_control motionbrain_hardware_interface motionbrain_mission motionbrain_ros_bridge motionbrain_description motionbrain_ros2_control_mock
colcon test-result --verbose
```

Dependencies are listed in [ROS2 CI](.github/workflows/ros2.yml); firmware/Python checks are defined in [PlatformIO CI](.github/workflows/platformio.yml).

## Pi service configuration

Use the installation's current hostname rather than a fixed DHCP address. `tools/raspi/check_pi_ssh_target.py` checks the SSH target and connectivity. `tools/raspi/discover_device_url.py` discovers ESP32 HTTP services, not Pi SSH.

```sh
python3 tools/raspi/check_pi_ssh_target.py
```

Review the [systemd examples](deploy/systemd/) before installing on Pi. The installer copies units/environment examples and calls `daemon-reload`; it does not automatically enable services.

```sh
tools/raspi/install_systemd_units.sh
```

The installer uses the environment filenames in the table below. It preserves existing files; if only a legacy `motionbrain-*.env` file exists, it copies its contents to the correct name and retains the old file. `.env.example` files are refreshed from the current templates. Configure device URLs, model paths and tokens, then enable only the required services. Set installer variables `MOTIONBRAIN_REPO` and `MOTIONBRAIN_SERVICE_USER` for another checkout/account.

| Service | Environment file | Purpose |
| --- | --- | --- |
| `motionbrain-perception.service` | `/etc/motionbrain/perception.env` | Camera acquisition/perception API |
| `motionbrain-dashboard.service` | `/etc/motionbrain/dashboard.env` | UI and controller proxy |
| `motionbrain-ros-bridge.service` | `/etc/motionbrain/ros-bridge.env` | ROS2 state/message bridge |

```sh
sudo systemctl enable --now motionbrain-perception.service
sudo systemctl enable --now motionbrain-dashboard.service
sudo systemctl enable --now motionbrain-ros-bridge.service
```

Environment directory/file modes are `750`/`640`. Keep actual tokens out of the repository, logs and screenshots. The dashboard binds to `127.0.0.1` by default. LAN exposure requires separate dashboard and controller authentication. ESP32 `/stream` is disabled; use `/capture` or Pi tracked images. The example CAM profile is QVGA / JPEG quality 15; inspect the actual configuration in each environment.

## Health checks and diagnostics

```sh
CHECK_SERVICE=1 tools/raspi/check_dashboard_health.sh
CHECK_SERVICE=1 tools/raspi/check_ros_bridge_health.sh
journalctl -u motionbrain-ros-bridge.service -n 100 --no-pager
```

To inspect the dashboard while devices are intentionally powered off, use the following. It accepts structured `degraded` responses and is not a motion-readiness check.

```sh
ALLOW_DASHBOARD_DEGRADED=1 CHECK_SERVICE=1 tools/raspi/check_dashboard_health.sh
```

ROS2 health checks allow some `WARN` diagnostics by default. Inspect sensor/feedback readiness separately before motion. The bridge check also tests physical routine rejection by default; successful status queries are distinct from physical execution readiness.

Record a controller HTTP failure:

```sh
python3 tools/diagnose_controller_http.py --target 'http://<controller-host>' --output '.local/http-diagnostic/<new-session>'
```

The tool sends at most four sequential GETs (`/status`, `/sequence`, `/actions`, `/action_outputs`) without retries. Acceptance is 0.8s per request, diagnostic collection timeout is 2s, and session budget is 12s. It checks motors OFF, empty queue and matching boot IDs, not electrical stopping or long-term reliability. [Correction and residual timeouts (Korean)](docs/evidence/2026-10-17-idle-http-client.md)

## Command states and recovery

The M4 executor stores a durable journal at `$XDG_STATE_HOME/motionbrain/m4_commands.sqlite3`, defaulting to `~/.local/state/motionbrain/m4_commands.sqlite3`. Use one executor/journal per controller. Duplicate ownership of the same file is blocked; different hosts/paths are outside that protection.

`accepted` means acknowledgment, not arrival. `forwarded` means an attempted or potentially attempted dispatch before interruption. `outcome_unknown` blocks new confirmations. Do not bypass uncertainty by deleting the journal, selecting a new path or automatically rebooting. Duplicate IDs are not resent. `/motionbrain/m4_write_result` reads stored results without contacting the controller.

Experimental `/actions` and the legacy M4 `/shoulder` executor are separate paths. `/actions` is disabled by default; physical activation and collection integration remain incomplete. See the [action/data contract (Korean)](docs/data-pipeline.md).

## Raw archives and training data

```sh
python3 tools/archive_hardware_demo.py '.local/<source-phase>' 'datasets/<new-archive>' --observation placed
```

The archive tool does not contact devices. It copies plans, results, after-images and optional camera receipts to a new destination, checks SHA-256 and durable writes, and leaves sources unchanged. Image suffixes must match actual PNG/JPEG content.

`archiveComplete` means copying finished, not task success. Observation snapshots are not synchronized action frames; image/anchor/source-persistence checks remain `unknown`. `trainingEligible` and `controlEligible` remain false. Offline copy speed cannot establish acquisition-time compliance. Keep raw images, receipts and execution records in ignored `.local/` or `datasets/`, outside Git.

[Training/evaluation reproduction (Korean)](docs/data-pipeline.md) · [Mac camera instrumentation (Korean)](docs/mac-direct-camera.md) · [ROS2 hardware interface](ros2_ws/src/motionbrain_hardware_interface/README.en.md)
