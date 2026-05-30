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

Use the local ops dashboard at `http://127.0.0.1:8765` for status, events,
ESP32-CAM, and vision alignment observation. Do not duplicate manual control
there.

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
