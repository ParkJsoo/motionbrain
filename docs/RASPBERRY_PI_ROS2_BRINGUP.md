# Raspberry Pi ROS2 Bring-Up

This document is the Raspberry Pi bring-up checklist and validation record for
proving MotionBrain as a ROS2-backed robot portfolio project.

Goal:

```text
Raspberry Pi 4 runs ROS2 Jazzy as the robot host and bridges the ESP32 motion
controller plus ESP32-CAM into ROS2 topics and command channels.
```

## Target Setup

- Board: Raspberry Pi 4, 8GB RAM
- OS: Ubuntu Server 24.04 LTS, 64-bit arm64
- ROS2: Jazzy Jalisco
- Network: trusted Home Wi-Fi shared by Raspberry Pi, ESP32 motion controller,
  and ESP32-CAM
- Robot controller hostname: `motionbrain.local`
- Camera hostname: `motionbrain-cam.local`
- Development machine: Mac, optional after Raspberry Pi host is running

Use ROS2 Jazzy for this portfolio path because it targets Ubuntu 24.04 and is a
long-term support ROS2 release.

## 2026-05-26 Validation Result

Raspberry Pi 4 hardware validation was completed with this path:

- Pi: Raspberry Pi 4, 8GB RAM
- OS: Ubuntu Server 24.04.4 LTS, arm64
- ROS2: Jazzy
- Workspace: `~/develop/arduino/motionbrain/ros2_ws`
- Branch: `feature/raspberry-pi-ros2-bringup`
- ESP32 controller IP used for validation: `192.168.219.113`
- ESP32-CAM IP used for validation: `192.168.219.114`

Validated outputs:

- `colcon build --packages-select motionbrain_ros_bridge` finished
  successfully on the Pi.
- `ros2 pkg list | grep motionbrain` returned `motionbrain_ros_bridge`.
- `ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py`
  started `motionbrain_status_node`.
- `/motionbrain/status` published real ESP32 controller JSON.
- `/camera/detection` published real ESP32-CAM detection JSON.
- `/motionbrain/light_cmd` with `toggle` reached the ESP32 controller through
  the token-gated `/light` endpoint.
- `/motionbrain/light_result` returned a successful command result.
- The physical search light turned on from a ROS2 command.

Notes from the run:

- The Mac resolved `motionbrain.local` and `motionbrain-cam.local`, but the Pi
  did not. The validated Pi launch used IP fallback.
- A missing or wrong `MOTIONBRAIN_HTTP_TOKEN` produced `HTTP Error 403:
  Forbidden`, which confirms the token gate was active.
- Do not paste placeholder token text into `MOTIONBRAIN_HTTP_TOKEN`; HTTP
  headers must be ASCII-safe and must match the token provisioned on the ESP32.

## 2026-05-27 Typed Message Validation Result

After rebooting the Raspberry Pi, the custom ROS2 message package was built and
validated on the real Pi host:

- Pi IP observed after reboot: `192.168.219.111`
- ESP32 controller IP used for validation: `192.168.219.109`
- ESP32-CAM IP used for validation: `192.168.219.110`
- Added missing C++ toolchain dependency with `sudo apt install -y g++`.
- `colcon build --packages-select motionbrain_msgs motionbrain_ros_bridge`
  finished successfully on the Pi.
- `ros2 pkg list | grep motionbrain` returned both `motionbrain_msgs` and
  `motionbrain_ros_bridge`.
- `ros2 interface list | grep motionbrain_msgs` returned all custom message
  types.
- `/motionbrain/status_typed` published real
  `motionbrain_msgs/msg/MotionStatus` data from the ESP32 controller.
- `/camera/detection_typed` published real
  `motionbrain_msgs/msg/CameraDetection` data from the ESP32-CAM bridge.

The bridge still publishes JSON string topics for compatibility and debugging,
but portfolio-facing ROS2 integration should prefer the typed topics.

## Portfolio Evidence Checklist

Capture these artifacts after bring-up:

- Raspberry Pi terminal showing Ubuntu version and ROS2 Jazzy environment
- `colcon build --packages-select motionbrain_msgs motionbrain_ros_bridge`
  success log
- `ros2 topic echo /motionbrain/status` output
- `ros2 topic echo /motionbrain/status_typed` output
- `ros2 topic echo /motionbrain/events` output
- `ros2 topic echo /camera/detection` output
- `ros2 topic echo /camera/detection_typed` output
- `ros2 topic pub --once /motionbrain/light_cmd ...` plus
  `/motionbrain/light_result` output
- Photo or short video showing Raspberry Pi, ESP32 controller, ESP32-CAM, and
  the robot hardware on Home Wi-Fi

## Flash Ubuntu

Use Raspberry Pi Imager:

1. Select Raspberry Pi 4.
2. Select `Ubuntu Server 24.04 LTS (64-bit)`.
3. Configure SSH, hostname, username, Wi-Fi SSID, and locale before writing.
4. Write the image to the microSD card.
5. Boot the Raspberry Pi and wait for first-boot setup to finish.

Recommended hostname:

```text
motionbrain-pi
```

## First Login

From the Mac:

```bash
ssh <user>@motionbrain-pi.local
```

If mDNS is not available, find the Pi IP address from the router DHCP client
list and use:

```bash
ssh <user>@<pi-ip>
```

Confirm OS and architecture:

```bash
lsb_release -a
uname -m
```

Expected:

- Ubuntu 24.04
- `aarch64`

## Base Packages

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release git python3-pip python3-venv avahi-daemon
```

Confirm mDNS service is running:

```bash
systemctl status avahi-daemon --no-pager
```

## Install ROS2 Jazzy

Enable the Ubuntu universe repository:

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

Add the ROS apt repository:

```bash
sudo apt update
sudo apt install -y curl gnupg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

Install ROS2 and build tools:

```bash
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep
```

If `liblz4-dev`, `libzstd-dev`, or `zlib1g-dev` dependency versions are
unavailable, make sure `noble-updates` is enabled:

```bash
sudo add-apt-repository -y "deb http://ports.ubuntu.com/ubuntu-ports noble-updates main restricted universe multiverse"
sudo apt update
```

Initialize rosdep:

```bash
sudo rosdep init
rosdep update
```

Add ROS2 setup to the shell:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify ROS2:

```bash
printenv ROS_DISTRO
ros2 topic list
```

Expected:

```text
jazzy
```

`ros2 --version` is not a valid ROS2 CLI option in this setup; use
`printenv ROS_DISTRO` and `ros2 --help` for a quick sanity check.

## Clone MotionBrain

```bash
mkdir -p ~/develop/arduino
cd ~/develop/arduino
git clone <repo-url> motionbrain
cd motionbrain
```

If the repository is already copied to the Pi, skip the clone and enter the
repo directory.

## Network Verification

The ESP32 motion controller and ESP32-CAM should already be provisioned for
Home Wi-Fi. See `docs/HOME_WIFI_MODE.md` for provisioning.

From the Pi:

```bash
ping -c 3 motionbrain.local
ping -c 3 motionbrain-cam.local
curl -sS http://motionbrain.local/status
curl -I http://motionbrain-cam.local/capture
```

If `.local` names do not resolve, use the IP addresses printed by each ESP32's
serial log or reserve stable IP addresses in the router.

Example validated IP fallback for network checks:

```bash
curl -sS http://192.168.219.109/status
curl -I http://192.168.219.110/capture
```

## Build ROS2 Bridge

Install package dependencies:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

If rosdep prints `Cannot locate rosdep definition for [ament_python]` but then
finishes installing resolvable dependencies, continue with the build. The
validated Pi setup had the required ament Python tooling from the ROS2 apt
installation.

Build the ROS2 packages:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_ros_bridge
source install/setup.bash
```

If CMake reports `No CMAKE_CXX_COMPILER could be found`, install the C++
toolchain and rebuild:

```bash
sudo apt install -y g++
colcon build --packages-select motionbrain_msgs motionbrain_ros_bridge
source install/setup.bash
```

Expected result:

```text
Summary: 2 packages finished
```

## Run Bridge

If the motion controller has a command token, export it before running:

```bash
export MOTIONBRAIN_HTTP_TOKEN="CHANGE_ME_TO_LOCAL_TOKEN"
```

Start the bridge directly:

```bash
ros2 run motionbrain_ros_bridge motionbrain_status_node \
  --ros-args \
  -p motion_host:=motionbrain.local \
  -p camera_url:=http://motionbrain-cam.local \
  -p detect_color:=red \
  -p poll_interval:=1.0 \
  -p http_timeout:=4.0
```

Leave this terminal running.

After direct execution works, verify the launch file:

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
```

Use IP fallback with launch arguments if mDNS is unstable:

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=<controller-ip> \
  camera_url:=http://<camera-ip>
```

## Verify Topics

Open a second SSH terminal to the Pi:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Confirm topics:

```bash
ros2 topic list
```

Expected topics:

```text
/camera/detection
/camera/detection_typed
/motionbrain/events
/motionbrain/events_typed
/motionbrain/light_cmd
/motionbrain/light_cmd_typed
/motionbrain/light_result
/motionbrain/light_result_typed
/motionbrain/status
/motionbrain/status_typed
```

Confirm custom message interfaces:

```bash
ros2 interface show motionbrain_msgs/msg/MotionStatus
ros2 interface show motionbrain_msgs/msg/CameraDetection
```

Check robot status:

```bash
ros2 topic echo /motionbrain/status
ros2 topic echo /motionbrain/status_typed --once
```

Check event stream:

```bash
ros2 topic echo /motionbrain/events
```

Check camera detection:

```bash
ros2 topic echo /camera/detection
ros2 topic echo /camera/detection_typed --once
```

The `/camera/detection` payload should include fields such as `detected`,
`color`, `areaRatio`, `offsetX`, `alignment`, and `commandSuggestion`.
The typed topic carries the same detection state in
`motionbrain_msgs/msg/CameraDetection` and keeps the raw JSON payload in
`raw_json`.

## Verify Command Channel

In the second terminal:

```bash
ros2 topic echo /motionbrain/light_result
```

Open a third SSH terminal and publish the command:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub --once --wait-matching-subscriptions 1 /motionbrain/light_cmd std_msgs/msg/String "{data: toggle}"
```

Typed command alternative:

```bash
ros2 topic pub --once --wait-matching-subscriptions 1 /motionbrain/light_cmd_typed motionbrain_msgs/msg/LightCommand "{action: toggle}"
```

Expected:

- The ESP32 search light toggles.
- `/motionbrain/light_result` publishes the raw HTTP command result plus the
  requested action.

This is important because it proves the bridge is not publish-only. It has a
ROS2 command path back into the embedded controller.

## Raspberry Pi Host Role

For the portfolio demo, the Raspberry Pi host is responsible for:

- Running ROS2 Jazzy
- Polling the ESP32 motion controller over HTTP
- Polling ESP32-CAM capture frames over HTTP
- Publishing robot status, events, and camera detection as JSON and typed ROS2
  topics
- Accepting JSON and typed ROS2 command messages and forwarding safe HTTP
  commands to ESP32

Keep the Mac as the development/dashboard machine during early validation. Move
dashboard or additional vision loops to the Pi only after the bridge path is
stable.

## Troubleshooting

### `.local` Hostnames Do Not Resolve

Check Avahi:

```bash
systemctl status avahi-daemon --no-pager
```

Use IP fallback:

```bash
ros2 run motionbrain_ros_bridge motionbrain_status_node \
  --ros-args \
  -p motion_host:=<controller-ip> \
  -p camera_url:=http://<camera-ip>
```

### ROS2 Package Not Found

Source both ROS2 and workspace setup files:

```bash
source /opt/ros/jazzy/setup.bash
source ~/develop/arduino/motionbrain/ros2_ws/install/setup.bash
```

### Camera Detection Reports `opencv_unavailable`

Install OpenCV and rebuild if needed:

```bash
cd ~/develop/arduino/motionbrain/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select motionbrain_msgs motionbrain_ros_bridge
source install/setup.bash
```

### Light Command Returns Token Error

Export the same command token provisioned on the ESP32 motion controller:

```bash
export MOTIONBRAIN_HTTP_TOKEN="CHANGE_ME_TO_LOCAL_TOKEN"
```

Then restart the ROS2 bridge process.

Observed failure mode:

```text
{"error":"HTTP Error 403: Forbidden","requestedAction":"toggle","success":false}
```

That means ROS2 publish/subscribe is working, but the ESP32 rejected the HTTP
POST because the token was missing or incorrect.

### HTTP Polling Is Unstable

Use longer timeouts:

```bash
ros2 run motionbrain_ros_bridge motionbrain_status_node \
  --ros-args \
  -p motion_host:=motionbrain.local \
  -p camera_url:=http://motionbrain-cam.local \
  -p poll_interval:=2.0 \
  -p http_timeout:=6.0
```

## Done Criteria

The Raspberry Pi ROS2 bring-up is complete when:

- Pi boots Ubuntu 24.04 arm64 and ROS2 Jazzy.
- Pi reaches the ESP32 controller and ESP32-CAM on Home Wi-Fi by `.local`
  hostname or IP fallback.
- `motionbrain_msgs` and `motionbrain_ros_bridge` build successfully on the Pi.
- `/motionbrain/status`, `/motionbrain/events`, `/camera/detection`, and their
  typed equivalents publish real data.
- `/motionbrain/light_cmd` controls the ESP32 through the ROS2 bridge.
- Logs, screenshots, and at least one photo or video are saved for README and
  portfolio use.
