# Build and Operations

[한국어](OPERATIONS.md) · [README](README.en.md) · [Pin map](PIN_MAP.en.md)

These are general repository reproduction steps. Device addresses, credentials and joint calibration belong to each installation. Passing builds/tests does not upload firmware or actuate motors.

Current goals and stages are described in the [visual action learning design (Korean)](docs/visual-action-learning.md), and the collection → dataset → training/evaluation workflow is documented in the [manual joint collection guide (Korean)](docs/manual-data-collection.md). Prediction inputs are pre-command images and planned commands; actual outputs and later sensor readings are reserved for evidence and targets. A learned loop that uses object/gripper relative state to align, close, and lift remains incomplete.

The earlier M2 [action prediction experiment (Korean)](docs/evidence/2026-10-31-first-action-prediction.md) and [single goal-approach trial (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) use a separate contract. Follow the [offline selector (Korean)](docs/offline-action-selection.md) for saved-data comparisons and the [visual action collector (Korean)](docs/visual-action-collection.md) for the scope and journal procedure of a finite physical trial. The general commands below do not execute an entire collection or autonomous-control workflow.

[Finite output profiles (Korean)](docs/finite-output-profile.md) are enabled only in the separate `esp32dev_m2_finite_trial` build; default builds keep them disabled. Four-profile firmware upload and individual output checks were completed, but the new-view U4 trial on 2026-11-04 was rejected by measurement after output completed because post-window variation exceeded its threshold (one command, one completed output, zero eligible transitions). Earlier pre-ARM stops are also preserved. Read the conditions, denominators, and limitations alongside [stationary repeatability diagnostics (Korean)](docs/stationary-repeatability.md); completed output does not establish accurate motion or eligible training data.

## Builds

ESP32 uses PlatformIO. Host tests require Python, NumPy, OpenCV and Node.js (CI: Python 3.12 and Node.js 22). Native C++ checks require a C++ compiler and are skipped when it is unavailable. Replace `<...>` placeholders with installation-specific values. Run from the repository root:

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

STA connection and reconnection disable ESP32 modem sleep to avoid power-save waits during sensor queries. `/status` reports the driver-read `network.stationPowerSave` as `none`, `min_modem`, `max_modem`, `not_station`, or `unknown`. Failure logs a warning without blocking the connection. Even `none` does not guarantee collection within 20ms; continue checking each request. AP behavior and sensor polling intervals are unchanged. [Finite latency measurement (Korean)](docs/mac-direct-camera.md)

## Command states and recovery

The M4 executor stores a durable journal at `$XDG_STATE_HOME/motionbrain/m4_commands.sqlite3`, defaulting to `~/.local/state/motionbrain/m4_commands.sqlite3`. Use one executor/journal per controller. Duplicate ownership of the same file is blocked; different hosts/paths are outside that protection.

`accepted` means acknowledgment, not arrival. `forwarded` means an attempted or potentially attempted dispatch before interruption. `outcome_unknown` blocks new confirmations. Do not bypass uncertainty by deleting the journal, selecting a new path or automatically rebooting. Duplicate IDs are not resent. `/motionbrain/m4_write_result` reads stored results without contacting the controller.

Experimental `/actions` and the legacy M4 `/shoulder` executor are separate paths. `/actions` is disabled by default; physical activation and collection integration remain incomplete. See the [action/data contract (Korean)](docs/data-pipeline.md).

## Raw archives and training data

Closed manual-collection sessions can be converted using the [FFV1 lossless archive and restore procedure (Korean)](docs/manual-data-collection.md). Full restoration and original compressed-file byte hashes are verified before pruning. Restore only the needed sessions to their original paths before existing replay or training tools read archived frames. An archive on the same disk is not an external backup.

A separate [relative-state path (Korean)](docs/relative-action-selection.md) reaudits and trains manual gripper-camera annotations for M1, M2 and M5 under separate joint contracts. This is distinct from M4 sensor-label conversion, automatic perception and physical control support.

```sh
python3 tools/archive_hardware_demo.py '.local/<source-phase>' 'datasets/<new-archive>' --observation placed
```

The archive tool does not contact devices. It copies plans, results, after-images and optional camera receipts to a new destination, checks SHA-256 and durable writes, and leaves sources unchanged. Image suffixes must match actual PNG/JPEG content.

`archiveComplete` means copying finished, not task success. Observation snapshots are not synchronized action frames; image/anchor/source-persistence checks remain `unknown`. `trainingEligible` and `controlEligible` remain false. Offline copy speed cannot establish acquisition-time compliance. Keep raw images, receipts and execution records in ignored `.local/` or `datasets/`, outside Git.

The separate `esp32cam_register_diag` build observes 34 raw OV2640 registers at most three times. Sensor/DMA initialization occurs internally, but there are no image requests or exports and no automatic retry after failure or expiry. A host runner can observe USB power connection for an already installed diagnostic app. It keeps the UART connection through readiness and HTTP collection without uploading firmware or issuing an explicit reset pulse. Both physical attempts on 2026-10-18 failed to complete the diagnostic (0/2). The first stopped at the UART handoff boundary; in the second, the OV2640-only guard rejected driver PID `0x3660` (OV3660) before any additional diagnostic SCCB access. Actual exposure bounds remain unverified, and that diagnostic yielded zero training-eligible physical action transitions. [Finite register diagnostic (Korean)](docs/ov2640-register-diagnostic.md)

[Training/evaluation reproduction (Korean)](docs/data-pipeline.md) · [Mac camera instrumentation (Korean)](docs/mac-direct-camera.md) · [ROS2 hardware interface](ros2_ws/src/motionbrain_hardware_interface/README.en.md)

## Automatic UI recovery after IP changes

Both the controller and Pi must be deployed with a version containing this feature. Merging or pushing source does not update running devices.

On a fresh installation, starting only the three services does not enable periodic rediscovery. Review addresses, tokens and camera settings in `/etc/motionbrain/dashboard.env` and `perception.env`, then enable the timer separately on the Pi. Timer runs discover devices, apply the camera profile by default, and restart affected services when needed.

```sh
sudo systemctl enable --now motionbrain-dashboard-reconcile.timer
systemctl status motionbrain-dashboard-reconcile.timer --no-pager
systemctl list-timers motionbrain-dashboard-reconcile.timer --all
```

- The existing Pi `motionbrain-dashboard-reconcile.timer` rediscovers devices approximately every 60 seconds. It restarts affected services when addresses change and publishes a healthy dashboard endpoint to the controller. Unchanged configurations also renew the publication.
- Publication uses the existing `MOTIONBRAIN_HTTP_TOKEN` to authenticate `POST /vision_endpoint?port=8765`. The host comes from the authenticated TCP peer's private IPv4 address, not an arbitrary supplied URL. The RAM-only lease expires after 180 seconds and does not change controller state or motor outputs. Tokens are absent from URLs and discovery responses.
- The controller UI reads `GET /vision_endpoint` and dashboard `/api/discovery` to recover Pi and camera addresses. Legacy saved IPs become automatic-mode hints. New addresses entered through `CAM` or `API` remain manual overrides; `AUTO` restores automatic discovery. Discovery does not convert a single snapshot into continuous capture.
- After repeated connection failures, the dashboard searches its remembered controller address and `motionbrain.local` for the new Pi endpoint. It verifies the dashboard identity before navigation and never transfers tokens, URL fragments, or commands, or replays commands. A new origin requires re-entering the dashboard control token.
- Discovery requests have a two-second timeout; image requests have a six-second watchdog. Recovery requests do not overlap. Hidden tabs issue no new image/discovery requests. Video connection and `NO TARGET`/detection errors are displayed separately.
- The perception live-image endpoint returns HTTP 503 (`no_fresh_frame`) once the original receive time exceeds `stale_seconds` on the monotonic clock. The dashboard propagates this as HTTP 502, clears the old image, and displays reconnecting until a fresh frame arrives. Historical evidence requested with an explicit `frameId` remains readable until cache eviction; that response does not establish a live connection or control eligibility. Display freshness for images without source metadata is based on host receipt time.

Recovery requires **a reachable controller address or working Pi hostname resolution**. If every device IP changes and all hostname resolution fails, a browser cannot locate the new network addresses. Restore networking/mDNS or enter a current address. The supported scope is a private IPv4 LAN with one MotionBrain setup, not IPv6-only networks, public proxies, or automatic selection among identically named devices. Service startup and discovery intervals mean recovery is not instantaneous.

Run software verification with `python3 -m unittest discover -s tests`. Browser recovery tests require Node.js and use mocked HTTP and virtual timers. They are separate from physical DHCP changes, device reboots, image collection, and motor tests. The existing systemd installer also installs `publish_vision_endpoint.py`, using the existing authentication and timer configuration.

## Supervised collection

The [manual joint data collection console (Korean)](docs/manual-data-collection.md) runs on the Mac and records/replays video, commands, actual outputs, and sensors for one selected joint among M1–M5. M4 permits downward motion only, and starting the server does not start capture or motion. Automated dataset conversion and sensor-delta learning currently target M4. Selecting every joint does not imply completed measurement or learning for every joint.

Models were fixed using 18 M4 downward records across six conditions, then evaluated without refitting on 18 later records. MAE was **0.367°** for the per-command mean, **0.595°** for command-only ridge, and **0.605°** for image-and-command ridge. The coarse image representation provided no demonstrated added predictive value. These are sequential same-day poses after a conditional restart. The four pre-motion stopped batches before the successful 18-record batch, plus initial failures and excluded records, are preserved in the [measurement and evaluation record (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md). Independent generalization, repeated control, and learned-policy grasping remain unverified. Offline candidate comparison accepts a target sensor delta but does not generate an object-approach target automatically.

The earlier [single-joint collection blocks and offline measurement path (Korean)](docs/supervised-action-collection.md) executed and measured 17 of 23 planned M2 actions on 2026-11-03; communication/response failures left six unexecuted before RUN. Those development samples use a settled-endpoint contract and are not automatically mixed with the M4 data above, the older 32 M2 samples, or finite-profile data. New physical execution requires current connection/clearance checks and onsite supervision.
