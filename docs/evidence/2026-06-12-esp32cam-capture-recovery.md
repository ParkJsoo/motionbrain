# 2026-06-12 ESP32-CAM Capture Recovery Evidence

This note summarizes the non-motion ESP32-CAM recovery work performed after the
Pi ROS2 evidence capture showed `/camera/detection_typed` as unavailable due to
camera capture failures.

## Runtime

- ESP32-CAM address: `192.168.219.111`
- Raspberry Pi address: `192.168.219.113`
- Controller address: `192.168.219.110`
- Git commit: `f26636b Stabilize ESP32-CAM capture recovery`
- Firmware upload port: `/dev/cu.usbserial-1130`
- Pi services checked:
  - `motionbrain-perception.service`
  - `motionbrain-dashboard.service`
  - `motionbrain-ros-bridge.service`

## Firmware Change

The ESP32-CAM firmware now exposes additional `/status` diagnostics:

```text
consecutiveCaptureFailures
cameraRecoveries
lastRecoveryMs
lastRecoveryDurationMs
lastRecoveryOk
lastError
```

If `esp_camera_fb_get()` fails during `/capture`, the firmware records
`camera_capture_failed`, deinitializes the camera driver, waits briefly, and
reinitializes the camera using the active frame size and JPEG quality.

## Profile Selection

The failing live profile was `qvga` with JPEG quality `4`. On the powered bench,
that profile produced repeated `503` responses from `/capture`.

The tested stable profiles were:

```text
quality=10: 3/3 direct captures succeeded
quality=12: 3/3 direct captures succeeded
quality=15: 3/3 direct captures succeeded
```

The Pi service wrapper default was changed to `qvga` / JPEG quality `10` as the
live operating balance between capture stability and object-detection input
quality.

## Live Verification

After uploading the ESP32-CAM firmware and applying the `qvga` / quality `10`
profile, direct capture passed five times in a row:

```text
capture_1 code=200 time=0.158234 size=10155
capture_2 code=200 time=0.182667 size=10126
capture_3 code=200 time=0.319818 size=10102
capture_4 code=200 time=0.283698 size=10118
capture_5 code=200 time=0.316746 size=10103
```

The camera status then showed:

```text
frameSize: qvga
jpegQuality: 10
captures: 13
captureFailures: 0
consecutiveCaptureFailures: 0
cameraRecoveries: 0
lastError: ""
```

The Pi perception service was restarted and picked the direct camera IP:

```text
camera-url http://192.168.219.111
```

Perception health returned:

```text
ok: true
fresh: true
lastError: ""
```

ROS2 `/camera/detection_typed` returned:

```text
available: true
target_type: object
label: cup
alignment: LOST
camera_url: http://192.168.219.111
reason: no_objects
```

## Safety Boundary

This work touched only the ESP32-CAM firmware and Pi camera/perception service
configuration. No ARM, motor, light, gripper, nudge, grasp, or physical routine
execution command was sent.

A post-capture controller status check showed:

```text
state: IDLE
motorEnabled: false
all motor speeds: 0
all motors enabled: false
light: false
blockReason: NONE
faultReason: NONE
```
