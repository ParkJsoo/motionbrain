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
- ESP32-CAM provides QVGA JPEG frames via `/capture`; this is the stable input.
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

As of 2026-05-31 on `feature/pi-object-detection-mvp`:

- Milestone 1 is implemented: shared detection contract, selected-target JSON,
  fake object backend contract tests, and color compatibility.
- Milestone 2 is implemented: Pi perception service with cached frame,
  `/api/detection`, `/api/perception`, `/api/vision_frame`, `/health`, and
  dashboard `--perception-url` proxy mode.
- Milestone 3 is in progress: the first real object backend path exists through
  OpenCV DNN/ONNX model loading, label-map loading, SSD-style output decoding,
  confidence filtering, NMS, and detector injection into the Pi perception
  service. Model weights are intentionally not committed.
- Live hardware validation is currently color-mode only. Real object-mode
  validation still requires selecting/downloading a small Pi-suitable model and
  labels file outside the repository.

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
- Decode SSD-style detector outputs into the existing selected-target payload.
- Keep color detection as the runtime fallback by running
  `--detector-mode color`; if object mode is explicitly requested without a
  usable model, fail fast instead of silently changing the requested mode.

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
--object-input-size 320
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
python3 tools/motionbrain_perception_service.py \
  --camera-url http://<camera-ip> \
  --detector-mode object \
  --object-backend opencv-dnn \
  --object-model <model> \
  --object-labels <labels> \
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

- Add a dry-run grasp planner that only emits proposed steps.
- Require centered stable target and calibrated workcell assumptions.
- No controller POSTs in the first pass.

Output example:

```json
{
  "ready": true,
  "target": "cup",
  "alignment": "CENTER",
  "rangeBand": "near",
  "plannedSequence": [
    {"joint": "gripper", "action": "open", "percent": 35, "ms": 300},
    {"joint": "shoulder", "action": "down", "percent": 25, "ms": 450},
    {"joint": "gripper", "action": "close", "percent": 35, "ms": 550}
  ]
}
```

### Milestone 7: Operator-Confirmed Grasp Sequence

Scope:

- Run one calibrated sequence after explicit operator confirm.
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
