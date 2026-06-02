# Physical AI Object Detection MVP Plan

Branch: `feature/pi-object-detection-mvp`

This plan scopes a Raspberry Pi 4B 8GB object-detection and constrained-grasp
upgrade for MotionBrain. The goal is to make the current color-target demo grow
into a credible physical-AI prototype without overstating what the current arm
hardware can safely do.

## Target Outcome

Build a constrained workcell demo:

```text
ESP32-CAM frame
  -> Raspberry Pi perception
  -> selected object target and overlay
  -> timed base alignment
  -> operator-confirmed calibrated grasp sequence
  -> stop/status verification
```

The demo should be framed as:

> Pi-hosted edge-AI perception with safety-gated robotic manipulation in a
> constrained workcell.

Do not describe this as a general-purpose autonomous grasping system.

## Hardware Assumptions

- Raspberry Pi 4B 8GB runs the perception workload.
- ESP32-CAM provides JPEG frames via `/capture`; `vga` with JPEG quality `12`
  is the current object-detection baseline, with `qvga` quality `15` as the
  low-bandwidth fallback.
- ESP32 motion controller remains the actuator and safety boundary.
- STM32 handheld teleop sends embedded sensor/safety telemetry.
- The current HC-SR04 is in the handheld teleop path, not mounted as a robot
  range sensor. Treat it as advisory/sensor health, not object-to-gripper range.
- The OWI-style arm has no joint encoders, no force feedback, and no reliable
  absolute joint pose.

## Architecture Decisions

1. Keep the existing color detector as the fallback and compatibility path.
2. Extract one shared perception contract before adding object model loading.
3. Move model inference to a Pi-side perception service instead of tying
   inference directly to dashboard browser polling.
4. Keep `/api/detection` compatible with the current selected-target contract.
5. Use enriched JSON fields first; add richer ROS2 typed object-array messages
   only after the selected-target path is stable.
6. Keep physical motion behind explicit operator confirmation until more
   position/contact feedback exists.

## Current Branch Status

As of 2026-06-01 on `feature/pi-object-detection-mvp`:

- Milestone 1 is implemented: shared detection contract, selected-target JSON,
  fake object backend contract tests, and color compatibility.
- Milestone 2 is implemented: Pi perception service with cached frame,
  `/api/detection`, `/api/perception`, `/api/vision_frame`, `/health`, and
  dashboard `--perception-url` proxy mode.
- Milestone 3 is in progress: the first real object backend path exists through
  OpenCV DNN/ONNX model loading, label-map loading, SSD-style and Ultralytics
  YOLO raw-output decoding, confidence filtering, NMS, and detector injection
  into the Pi perception service. Model weights are intentionally not
  committed.
- Live hardware validation confirms the Pi perception service and dashboard can
  track red color targets reliably with the ESP32-CAM QVGA feed.
- Earlier live object-mode validation on the QVGA feed loaded YOLOv5n, YOLOv5s,
  and a YOLOv8n ONNX candidate through the Pi OpenCV DNN path, but a white cup
  target was not detected reliably.
- Offline validation after removing the ESP32-CAM lens film and switching to
  `vga` / JPEG quality `12` changed the result: `YOLOv5s` detected the white cup
  in `50/50` saved frames at confidence `0.5`; `YOLOv5n` remained too weak for
  this scene. Keep this as a constrained-object MVP path, not a guaranteed
  arbitrary-object demo.
- A local live perception-service smoke test using the same ESP32-CAM profile
  and `YOLOv5s` returned `label=cup` with confidence around `0.80-0.90` and
  detector latency around `120ms` on the Mac host. Pi latency still needs to be
  measured.
- Live Pi validation on 2026-06-02 KST corrected the YOLO preprocessing path to
  preserve aspect ratio with letterbox padding before OpenCV DNN inference. On
  the current low-angle ESP32-CAM bench, `vga` / JPEG quality `18` captured
  reliably but often mislabeled the cup body as `microwave` or `toilet`.
  Switching the camera profile to `qvga` / JPEG quality `4` produced the
  current physical success case: `YOLOv5s`, `--object-target cup`,
  `--object-min-confidence 0.5`, class id `41`, confidence about `0.55-0.59`,
  `alignment=CENTER`, and area ratio about `0.287` from the Pi dashboard API.
- Background-only evaluation on 2026-06-02 KST produced `0/50` false positives
  at confidence `0.5` and `0.25` with `YOLOv5s`; lowering to `0.1` introduced a
  low-confidence `boat` false positive. Keep the first live cup demo at
  confidence `0.5`.
- A label-less dark cola bottle with a red cap on the same dark background was
  not recognized as COCO `bottle` (`0/50` at confidence `0.05` to `0.5`), but it
  was consistently detected as `vase` (`49/50` at confidence `0.5`, `50/50` at
  confidence `0.25`) with no `vase` false positives on the saved background
  set. Treat this only as a constrained proxy target, not bottle recognition.
- Moving the same bottle to a white background increased the `vase` proxy
  confidence to about `0.72` and produced `50/50` detections at confidence
  `0.5`, but it still produced `0/50` COCO `bottle` detections. Use a more
  typical bottle if the semantic `bottle` label matters.
- An iPhone 13 mini back-side sample on the white background produced `0/50`
  COCO `cell phone` detections at confidence `0.05` to `0.5`. The model mostly
  detected the large sticker/phone silhouette as `tie` (`49/50` at confidence
  `0.25`). Use the screen side or a cleaner phone surface before treating phone
  detection as a viable semantic target.
- A Samsung Z Flip back-side sample on the white background did work as a
  semantic `cell phone` target when the threshold was lowered and the class was
  filtered: `43/50` at confidence `0.1`, `47/50` at confidence `0.05`, and
  `0/50` background false positives for `cell phone` at those thresholds. This
  is deferred as a secondary target; do not include it in the first physical-AI
  demo.
- The immediate object-detection feasibility step is complete: saved-frame
  evaluation and smoke tests show that the current ESP32-CAM can support
  constrained known-object perception for `cup`. Next work should connect the
  cup target to alignment, operator-confirmed motion, and a limited gripper
  sequence rather than adding more object classes; see
  `docs/VISION_DATASET_EVALUATION.md`.
- ROS2 can consume the Pi perception service through `perception_url`, so
  `/camera/detection` and `/camera/detection_typed` can publish the same
  selected target used by the dashboard without opening an additional
  ESP32-CAM connection.

## Perception Design

Current color detection is duplicated in:

- `tools/vision_host_mvp.py`
- `tools/motionbrain_dashboard.py`
- `ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/motionbrain_status_node.py`

Completed first implementation step:

- Add `ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/vision_detection.py`.
- Move shared functions there:
  - `DetectionConfig`
  - `DetectionCandidate`
  - `DetectorBackend`
  - `detect_frame(frame, config, detector=None)`
  - `detect_colored_target(frame, config)`
  - `select_target(candidates, policy)`
  - `payload_from_candidate(...)`
- Keep defaults exactly compatible with the current red color detector.

Completed second implementation step:

- Add object-mode config and fake backend tests.
- Add Pi perception service plus dashboard proxy mode.

Current object-backend step:

- Add OpenCV DNN/ONNX backend behind explicit model paths.
- Decode SSD-style and YOLO-style detector outputs into the existing
  selected-target payload.
- Keep color detection as the runtime fallback by running
  `--detector-mode color`; if object mode is explicitly requested without a
  usable model, fail fast instead of silently changing the requested mode.

Current ROS2 compatibility step:

- Add optional `perception_url` plumbing to the ROS2 status bridge launch and
  systemd startup path.
- When `perception_url` is set, publish the Pi perception service
  `/api/detection` payload to `/camera/detection` and `/camera/detection_typed`
  instead of directly fetching ESP32-CAM frames in the ROS2 node.
- Promote selected target fields (`target_type`, `label`, `class_id`,
  `confidence`) into `CameraDetection.msg`; keep the full payload in
  `raw_json`.

Recommended first live model path:

- Current tested path: YOLO ONNX models through OpenCV DNN. `YOLOv5s` is the
  first useful baseline for the ESP32-CAM VGA cup scene; `YOLOv5n` loads but is
  not reliable enough for this target. The official `YOLO11n` ONNX asset is a
  better modern target, but it did not load through OpenCV DNN on the current Pi
  due ONNX shape handling.
- Use `config/coco80.labels` for COCO class names.
- Start with `--object-input-size 640` for correctness, then benchmark 416/320
  if Pi CPU load is too high.
- Start `--object-min-confidence` at `0.5` for the cup.
- For the current live Pi bench, set the ESP32-CAM to `qvga` / JPEG quality `4`
  before starting perception. The saved-frame cup dataset used VGA, but the live
  low-angle scene currently labels the cup more reliably at high-quality QVGA.
- Current active semantic target: `cup` only.
- Defer the tested Z Flip phone target and avoid presenting the tested dark
  bottle as `bottle`; it only works as a `vase` proxy target.
- Avoid open-vocabulary prompts for the MVP. This phase detects known COCO
  classes; arbitrary text-described object search is a later model family.

Preferred runtime path:

- Primary: small int8 TFLite/LiteRT object detector such as EfficientDet-Lite0
  or SSD MobileNet.
- Fallback: ONNX Runtime CPU in a dedicated venv if TFLite packaging is painful
  on Ubuntu/Python 3.12.
- Avoid making PyTorch/Ultralytics a Pi runtime dependency for the MVP. If a
  YOLO model is used, export offline and run a small exported model.

Expected Pi 4B performance:

- Start target: 1-2 FPS end-to-end including camera fetch and JPEG decode.
- Stretch target after benchmarking: 3-5 FPS.
- Drop old frames. Do not queue inference requests.
- Publish freshness and latency so action gates can reject stale perception.

## Detection Payload Contract

Keep existing selected-target fields:

- `available`
- `detected`
- `color`
- `areaRatio`
- `pixels`
- `width`
- `height`
- `frameBytes`
- `centerX`
- `centerY`
- `centroidX`
- `centroidY`
- `targetBox`
- `offsetX`
- `offsetY`
- `alignDeadband`
- `alignment`
- `commandSuggestion`
- `cameraUrl`
- `ts`
- `reason`

Add compatible fields:

- `targetType`: `color` or `object`
- `label`
- `classId`
- `confidence`
- `detector`: `{mode, backend, model, latencyMs, targetPolicy}`
- `objects`: top candidates, each with label, class id, confidence, box, center,
  area ratio, offset, and alignment fields
- `target`: selected target in a richer nested form
- `stableFrames`: number of consecutive frames that matched the selected target

For object mode, set:

- `targetBox` from the selected object
- `pixels = targetBox.width * targetBox.height`
- `areaRatio = pixels / frame_area`
- `alignment` and `commandSuggestion` using the same horizontal offset logic as
  the color path

This lets the current dashboard, embedded `TRACKED` mode, ROS2 mission flow, and
control guard continue to work while richer object details live in JSON.

## CLI And Config Surface

Current Pi perception service options:

```text
--detector-mode {color,object}
--object-backend {fake,tflite,onnx,opencv-dnn}
--object-model PATH
--object-labels PATH
--object-target LABEL
--object-min-confidence 0.45
--object-nms-threshold 0.45
--object-input-size 640
--target-policy {largest,center,highest-confidence}
```

`--object-config PATH` remains reserved for future backends that need a
separate config file. The dashboard and embedded console consume the resulting
`/api/detection` payload through `--perception-url`; they do not need to own the
model runtime.

Environment variables for Pi/systemd:

```text
MOTIONBRAIN_DETECTOR_MODE
MOTIONBRAIN_OBJECT_BACKEND
MOTIONBRAIN_OBJECT_MODEL
MOTIONBRAIN_OBJECT_LABELS
MOTIONBRAIN_OBJECT_TARGET
MOTIONBRAIN_OBJECT_MIN_CONFIDENCE
```

Do not commit model weights.

## UI Changes

Dashboard:

- Show detector mode/backend/model.
- Show selected label and confidence.
- Show inference latency and target stability.
- Overlay label should use `payload.label || payload.color || "target"`.
- Preserve `Nudge Once` behavior and server-side revalidation.

Embedded `motionbrain.local`:

- Keep existing `TRACKED`, `STREAM`, and `SNAPSHOT` modes.
- No new firmware endpoint is required.
- Read the same `/api/detection` payload from the Pi dashboard/perception API.
- Overlay label should display object label and confidence when present.

## Motion And Grasp Scope

Allowed MVP motion:

1. Detect a selected object class or color target.
2. Require multiple fresh consistent frames.
3. Use timed base nudge only until target is centered.
4. Require operator confirmation before any grasp sequence.
5. Run short, low-speed, pre-calibrated sequence steps.
6. Stop and verify `/status` after each sequence.

Do not automate yet:

- General object recognition for arbitrary user-described objects.
- Full 3D pose estimation.
- IK-driven grasp planning.
- Continuous visual servoing.
- `/base?action=angle` without base-mounted feedback.
- Gripper force/contact inference.
- ROS-published live joint/sequence commands without richer typed safety fields.

## Safety Gates

Before any physical grasp attempt:

- Controller status is fresh.
- `state` is `IDLE` before arming and `ARMED` only immediately before motion.
- `sensor.blocked == false`.
- `faultLatched == false`.
- `baseAngle.active == false`.
- Target is detected and centered for multiple frames.
- Target is in a calibrated workcell or apparent-size band.
- `/command?cmd=stop` has been tested with the current token.
- Operator confirms the action.
- Physical power cutoff is reachable.

Alignment action:

- Use only timed `/joint?joint=base&action=<left|right>` nudge.
- Always send stop after the nudge.
- Limit to a small number of nudges per attempt.

Grasp action:

- Start as dry-run/log-only.
- Then run one calibrated low-speed sequence in an empty work area.
- Then run with one known object in one known position.

## Implementation Milestones

### Milestone 1: Shared Detection Contract

Scope:

- Add shared `vision_detection.py`.
- Refactor CLI, dashboard, and ROS2 bridge to use it.
- Preserve default color behavior and current payload fields.
- Add fake object backend tests but no real model dependency yet.

Validation:

```bash
python3 -m py_compile \
  tools/vision_host_mvp.py \
  tools/motionbrain_dashboard.py \
  ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/*.py
python3 -m unittest discover -s tests
pio run
```

### Milestone 2: Pi Perception Service

Scope:

- Add `tools/motionbrain_perception_service.py`.
- Continuous loop fetches ESP32-CAM `/capture`, decodes once, runs detector,
  stores latest result.
- HTTP endpoints:
  - `/api/perception`
  - `/api/detection` compatibility view
  - `/api/vision_frame` annotated or cached frame
  - `/health`
- Dashboard consumes latest result instead of running inference per browser
  request.

Validation:

```bash
python3 tools/motionbrain_perception_service.py \
  --camera-url http://<camera-ip> \
  --detector-mode color
curl -sS http://127.0.0.1:<port>/api/detection
```

### Milestone 3: Object Backend

Scope:

- Add OpenCV DNN/ONNX backend behind explicit model path.
- Add object labels and class whitelist.
- Add latency/threshold/fallback fields.
- Keep color fallback as the default detector mode; fail fast for explicitly
  requested object mode if the model/runtime is missing.

Validation:

```bash
curl -sS -X POST "http://<camera-ip>/camera?framesize=qvga&quality=4"

python3 tools/motionbrain_perception_service.py \
  --camera-url http://<camera-ip> \
  --detector-mode object \
  --object-backend opencv-dnn \
  --object-model ~/.cache/motionbrain/models/yolov5s.onnx \
  --object-labels config/coco80.labels \
  --object-min-confidence 0.5 \
  --object-target cup
curl -sS http://127.0.0.1:<port>/api/detection
```

### Milestone 4: Dashboard And Embedded Display

Scope:

- Show label/confidence/latency/stability in dashboard.
- Show label/confidence in `motionbrain.local` overlay.
- Keep existing color overlay behavior unchanged.

Validation:

```bash
python3 tools/motionbrain_dashboard.py \
  --host 0.0.0.0 \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detector-mode object \
  --object-target cup
```

### Milestone 5: ROS2 Compatibility

Scope:

- Publish selected object through existing `/camera/detection` JSON and
  `/camera/detection_typed`.
- Keep richer object list in `raw_json`.
- Do not add autonomous motion commands yet.

Validation:

```bash
cd ros2_ws
colcon build --packages-select \
  motionbrain_msgs motionbrain_control motionbrain_mission \
  motionbrain_ros_bridge motionbrain_description
colcon test --packages-select \
  motionbrain_control motionbrain_mission motionbrain_ros_bridge
colcon test-result --verbose
```

### Milestone 6: Constrained Grasp Dry Run

Scope:

- Dashboard exposes an operator-confirmed dry-run plan through
  `/api/cup_grasp_plan`.
- Require selected target `cup`, confidence at least `0.5`, centered alignment,
  a clear ARMED controller state, and base idle.
- No gripper or arm controller POSTs in the first pass.

Output example:

```json
{
  "ok": true,
  "success": true,
  "target": "cup",
  "alignment": "CENTER",
  "dryRun": true,
  "plannedSequence": [
    {"joint": "gripper", "action": "open", "percent": 35, "ms": 300},
    {"joint": "gripper", "action": "stop", "percent": 0, "ms": 0},
    {"joint": "gripper", "action": "close", "percent": 35, "ms": 450},
    {"joint": "gripper", "action": "stop", "percent": 0, "ms": 0}
  ]
}
```

### Milestone 7: Operator-Confirmed Grasp Sequence

Scope:

- Later, run one calibrated sequence after explicit operator confirm and an
  extra execution enable.
- Low speed, short duration, immediate stop/status verification.
- No continuous servoing.

Validation:

- Empty workcell dry run.
- Object in fixed location.
- One known target object.
- Stop command tested before recording.

## First Commit Recommendation

First commit on this branch should be:

```text
Extract shared vision detection contract
```

Included:

- `vision_detection.py`
- CLI/dashboard/ROS2 bridge import changes
- color-mode compatibility tests
- fake object backend contract tests

Excluded:

- Real object model runtime
- systemd service
- grasp motion
- ROS2 message schema changes

This reduces risk before adding model/runtime dependencies on the Raspberry Pi.
