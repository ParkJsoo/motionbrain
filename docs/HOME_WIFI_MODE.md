# MotionBrain Home Wi-Fi Mode

Home Wi-Fi mode lets the ESP32 motion controller and ESP32-CAM join the same
trusted local network as the Mac. This removes the need to switch the Mac to
`MotionBrain-AP` during bench testing.

## Security Rules

- Do not commit real Wi-Fi credentials.
- Do not commit real command tokens.
- Do not expose the ESP32 HTTP port through router port forwarding.
- Use a trusted home/test SSID, not a public or shared guest network.
- Prefer a router DHCP reservation or `.local` hostnames instead of hard-coded
  changing IP addresses.

## Provision Over Serial

The firmware does not require a checked-in or local credential file. On first
boot, each ESP32 asks for Wi-Fi credentials over the serial monitor and stores
them in ESP32 NVS flash.

Motion controller prompts:

```text
Wi-Fi SSID:
Wi-Fi password:
Command token:
```

ESP32-CAM prompts:

```text
Wi-Fi SSID:
Wi-Fi password:
```

Typed values are not written to project files or git. They are stored on the
device. Rebooting uses the stored values automatically.

To erase stored values, type `CLEAR` during the short boot prompt in the serial
monitor. Then reboot or wait for the provisioning prompt.

To rotate only the motion controller command token while preserving the stored
Wi-Fi SSID/password, open the controller serial monitor and run:

```text
wifi token <new-command-token>
```

The firmware redacts this command in serial debug logs, writes only the token
field to ESP32 NVS, and restarts the controller so the web server reloads the
new token.

## Build And Upload

Motion controller:

```bash
pio run -t upload
```

ESP32-CAM:

```bash
pio run -d firmware/esp32cam -t upload
```

After boot, serial logs should show:

```text
WiFi STA: Connected successfully
mDNS: http://motionbrain.local
ESP32-CAM mDNS: http://motionbrain-cam.local
```

## Host Commands

Use hostnames when mDNS works:

```bash
export MOTIONBRAIN_HTTP_TOKEN="CHANGE_ME_TO_A_LONG_RANDOM_LOCAL_TOKEN"
python3 tools/vision_host_mvp.py \
  --motion-host motionbrain.local \
  --camera-url http://motionbrain-cam.local \
  --detect-color red \
  --once
```

If ESP32-CAM mDNS is unreliable, use the IPs printed by each board's serial log. The vision host defaults are intentionally conservative for the ESP32-CAM HTTP server: `--timeout 6`, `--interval 3`, and `--capture-retries 2`.

For the dashboard:

```bash
export MOTIONBRAIN_HTTP_TOKEN="CHANGE_ME_TO_A_LONG_RANDOM_LOCAL_TOKEN"
python3 tools/motionbrain_dashboard.py \
  --motion-host motionbrain.local \
  --camera-url http://motionbrain-cam.local
```

For a Raspberry Pi-hosted dashboard that is reachable from another browser on
the same trusted LAN, bind the dashboard to the Pi's LAN interface:

```bash
export MOTIONBRAIN_HTTP_TOKEN="CHANGE_ME_TO_A_LONG_RANDOM_LOCAL_TOKEN"
python3 tools/motionbrain_dashboard.py \
  --host 0.0.0.0 \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --timeout 6
```

Then open `http://<pi-ip>:8765`. Keep this on a trusted local network only.
The default dashboard Nudge Once setting is conservative (`250ms`/`25%`). For a
visible demo nudge after confirming clearance and stop behavior, restart with
`--align-nudge-ms 600 --align-percent 40`.

If the Pi should own camera polling and detection, run the perception service
next to the dashboard and point the dashboard at `--perception-url`:

```bash
python3 tools/motionbrain_perception_service.py \
  --host 0.0.0.0 \
  --port 8766 \
  --camera-url http://<camera-ip> \
  --detector-mode color \
  --detect-color red \
  --timeout 6

export MOTIONBRAIN_HTTP_TOKEN="CHANGE_ME_TO_A_LONG_RANDOM_LOCAL_TOKEN"
python3 tools/motionbrain_dashboard.py \
  --host 0.0.0.0 \
  --motion-host <controller-ip> \
  --perception-url http://127.0.0.1:8766 \
  --timeout 6
```

Object mode uses explicit local model and label files. For the first live
check, use a YOLO11n detect ONNX export with `config/coco80.labels`:

```bash
python3 tools/motionbrain_perception_service.py \
  --host 0.0.0.0 \
  --camera-url http://<camera-ip> \
  --detector-mode object \
  --object-backend opencv-dnn \
  --object-model <model.onnx> \
  --object-labels config/coco80.labels \
  --object-target cup \
  --object-input-size 640
```

The ESP32-hosted `MotionBrain Control` page also uses this dashboard for
`TRACKED` camera mode. Its default dashboard API is
`http://motionbrain-pi.local:8765`; if mDNS is unreliable, set the page's `API`
field to `http://<pi-ip>:8765`. Older browser storage pointing to the
Mac-hosted `127.0.0.1:8765` dashboard is automatically migrated to the Pi
default.

If `.local` names do not resolve, use the IP addresses printed in serial logs or
reserve fixed IP addresses in the router.

On 2026-05-26, the Mac resolved `motionbrain.local` and
`motionbrain-cam.local`, while Raspberry Pi Ubuntu Server did not. The ROS2
bridge validation used IP fallback:

```bash
export MOTIONBRAIN_HTTP_TOKEN="CHANGE_ME_TO_A_LONG_RANDOM_LOCAL_TOKEN"
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:=<controller-ip> \
  camera_url:=http://<camera-ip>
```

## Control Surface Direction

Use the ESP32-hosted `MotionBrain Control` page at `http://motionbrain.local`
or the controller IP for manual operation. This page works from a phone on the
same Wi-Fi network and is the intended wireless controller surface.

The page prompts for the token on the first state-changing command. The token is
kept only in the current browser page's JavaScript memory; it is not written to
firmware, files, URL query strings, or browser local storage. Reloading or
closing the page clears it. If no token is provisioned, controller POST
endpoints are rejected until provisioning is repeated.

Bench verification on 2026-05-25 confirmed the phone flow: the token prompt
appears, the entered token is accepted, and controller commands execute from the
phone browser.

Use the local ops dashboard at `http://127.0.0.1:8765` locally, or
`http://<pi-ip>:8765` when hosted on the Pi, for status, events, ESP32-CAM,
target overlay, and vision alignment observation. Do not duplicate manual
control there.

The local ops dashboard and controller firmware suppress routine successful
status/event polling logs so serial and dashboard terminals stay focused on
commands, motion, safety transitions, and errors.

## Notes

- `GET /status`, `GET /events`, and camera capture endpoints remain readable on
  the local network.
- Motion controller POST endpoints still require `X-MotionBrain: 1`.
- Motion controller POST endpoints also require `X-MotionBrain-Token`.
- The built-in controller web page prompts for the token only at runtime and
  keeps it in page memory only.
- ROS2 bridge commands use the same token through `MOTIONBRAIN_HTTP_TOKEN`.
- If station connection fails, the motion controller starts a fallback AP named
  `MotionBrain-XXXX` with a device-specific password printed to serial logs.
