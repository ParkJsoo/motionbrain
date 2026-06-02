# Vision Dataset Evaluation

This note keeps the object-detection work grounded in repeatable frame captures
instead of live-only model guessing.

## Purpose

The object detector is useful for a constrained known-object workcell demo, not
for claiming general arbitrary-object recognition. This note keeps the work
grounded in repeatable frame captures and records which targets are actually
usable for MotionBrain's next "recognize -> align -> operator confirm -> grasp"
prototype.

Current conclusion:

- ESP32-CAM remains usable after removing the lens film and switching to VGA.
- Use `YOLOv5s` through OpenCV DNN as the first practical model baseline.
- Active semantic target: `cup`, confidence `0.5`.
- Defer the Samsung Z Flip `cell phone` target until after one reliable cup
  perception/alignment sequence is captured.
- Do not present the tested dark bottle or sticker-heavy iPhone back side as
  semantic object-recognition successes.

## Dataset Commit Policy

`datasets/vision/` is a raw local capture area and stays ignored by git. Do not
commit raw capture sessions, model weights, or local ONNX files to this repo.
Commit the repeatable tools and summarized evaluation results instead.

If a future test needs image data in git, add only a small curated public fixture
under a dedicated fixture path such as `tests/fixtures/vision/` after reviewing
size, privacy, and reproducibility. Larger datasets should live outside the repo
or in a release/artifact store.

## ESP32-CAM Profile

The ESP32-CAM firmware exposes a runtime camera profile endpoint:

```bash
curl -sS http://<camera-ip>/camera
curl -sS -X POST "http://<camera-ip>/camera?framesize=vga&quality=12"
```

Current practical baselines:

- Saved-frame evaluation baseline: `vga`, JPEG quality `12`.
- Live Pi/ESP32-CAM cup demo baseline: `qvga`, JPEG quality `4`.
- `vga`, JPEG quality `18`: captured reliably on the current board, but the
  low-angle live cup scene often mislabeled the cup body.
- `svga`: observed capture failures on the current board, so do not use it as
  the default.

Check `/status` after a profile change and watch `captureFailures`,
`lastCaptureMs`, and `lastFrameBytes`.

## Capture Frames

From a Pi dashboard or perception service:

```bash
python3 tools/capture_vision_dataset.py \
  --camera-url http://<camera-ip> \
  --label cup \
  --count 50 \
  --interval 0.08 \
  --notes "white cup on desk, ESP32-CAM QVGA quality=4"
```

If only the dashboard is running, pass its frame endpoint directly:

```bash
python3 tools/capture_vision_dataset.py \
  --frame-url http://<pi-ip>:8765/api/vision_frame \
  --detection-url http://<pi-ip>:8765/api/detection \
  --label bottle \
  --count 50
```

Each session writes:

```text
datasets/vision/<session>/
  manifest.json
  labels.jsonl
  frames/000000.jpg
```

For the current demo branch, use `cup` and `background`. Other labels can be
captured later, but they are not part of the active success path.

## Evaluate Detector Settings

Run the current OpenCV-DNN YOLO path against saved frames:

```bash
python3 tools/evaluate_object_detector.py \
  --dataset datasets/vision/<session> \
  --detector-mode object \
  --object-model ~/.cache/motionbrain/models/yolov5s.onnx \
  --object-labels config/coco80.labels \
  --object-input-size 640 \
  --object-min-confidence 0.5
```

For target-filtered recall, add:

```bash
--filter-target-from-label
```

The summary reports target match rate, wrong labels, false positives, average
confidence, and latency. Use that result to decide whether ESP32-CAM object mode
is worth tuning further or whether the next step should be a better camera path.

## Current Baseline Result

On 2026-06-01, after removing the ESP32-CAM lens protective film and switching
to `vga` / JPEG quality `12`, a 50-frame white cup dataset was captured at:

```text
datasets/vision/20260601T142932Z_cup
```

Offline OpenCV-DNN results on the same frames:

- `yolov5n.onnx`, confidence `0.5`: `0/50` cup detections.
- `yolov5n.onnx`, confidence `0.03`: `8/50` cup detections.
- `yolov5s.onnx`, confidence `0.5`: `50/50` cup detections, average confidence
  about `0.85`.
- Live perception-service smoke test with `yolov5s.onnx` and the current
  ESP32-CAM frame returned `label=cup`, confidence about `0.80-0.90`, and
  detector latency about `120ms` on the local Mac host.

Interpretation: the ESP32-CAM should not be discarded yet. For a constrained
single-object workcell, the camera can support object detection when the lens is
clear, VGA frames are used, and the model is strong enough.

On 2026-06-02 KST, a 50-frame background-only dataset was captured after
removing the cup:

```text
datasets/vision/20260601T151132Z_background
```

Offline `YOLOv5s` false-positive results on the background-only frames:

- Any object, confidence `0.5`: `0/50` false positives.
- Any object, confidence `0.25`: `0/50` false positives.
- Any object, confidence `0.1`: `1/50` false positive, labeled `boat`.
- Any object, confidence `0.05`: `2/50` false positives, labeled `boat`.
- `cup` target only, confidence `0.05` or higher: `0/50` false positives.

Interpretation: confidence `0.5` is a reasonable first live threshold for the
cup demo. Avoid lowering the threshold unless the target object is explicitly
filtered and the background false-positive set is rechecked.

The active dashboard dry-run path uses this same cup target and threshold. It
requires `label=cup`, confidence at least `0.5`, and `alignment=CENTER` before
returning a proposed gripper open/close sequence.

Live Pi validation on 2026-06-02 KST used Raspberry Pi
`192.168.219.115`, controller `192.168.219.113`, and ESP32-CAM
`192.168.219.114`. The Pi service was updated to the letterbox YOLO
preprocessing path and launched with `YOLOv5s`, `--object-target cup`,
`--object-min-confidence 0.5`, and `--object-input-size 640`.

Camera-profile sweep on the live bench showed:

- `qvga`, JPEG quality `4`: `cup` confidence about `0.55-0.60`, centered.
- `qvga`, JPEG quality `6`: `cup` confidence about `0.56`, centered.
- `qvga`, JPEG quality `8`: `cup` confidence about `0.46`, below the dry-run
  threshold.
- `qvga`, JPEG quality `10` and higher: cup confidence degraded or the cup body
  was mislabeled.
- `vga`, JPEG quality `18`: captured reliably, but the cup body was often
  labeled as `microwave` or `toilet`; only a weak partial `cup` candidate
  appeared.

With `qvga` / JPEG quality `4`, the live dashboard/perception API returned
`label=cup`, class id `41`, confidence about `0.55-0.59`,
`alignment=CENTER`, and an area ratio about `0.287`. This is the current
physical cup success profile.

On the same day, a 50-frame black unlabeled cola bottle dataset with a red cap
was captured on the dark cloth background:

```text
datasets/vision/20260601T151554Z_bottle
```

Offline `YOLOv5s` results:

- `bottle` target, confidence `0.5` down to `0.05`: `0/50` correct bottle
  detections.
- Any object, confidence `0.5`: `49/50` detections, all labeled `vase`, average
  confidence about `0.62`.
- Any object, confidence `0.25`: `50/50` detections, all labeled `vase`.
- `vase` target on the background-only dataset, confidence `0.5` and `0.25`:
  `0/50` false positives.

Interpretation: the label-less dark bottle is not semantically recognized as a
COCO `bottle` in this setup. It can still be used as a constrained physical
target if documented as a silhouette/proxy target detected under the `vase`
class, but it should not be presented as robust bottle recognition.

The same bottle was retested on a white background:

```text
datasets/vision/20260601T152136Z_bottle
```

Offline `YOLOv5s` results:

- `bottle` target, confidence `0.5` down to `0.05`: `0/50` correct bottle
  detections.
- Any object, confidence `0.5`: `50/50` detections, all labeled `vase`, average
  confidence about `0.72`.

Interpretation: the white background improves target confidence and silhouette
quality, but it does not fix the semantic class mismatch for this unlabeled dark
cola bottle. For a demo that needs the word `bottle`, use a more typical
COCO-like bottle, such as a clear plastic water bottle or a labeled bottle. For
a constrained alignment/tracking demo, this object can be used as `vase`.

An iPhone 13 mini back-side sample on the white background was also tested:

```text
datasets/vision/20260601T152627Z_cell_phone
```

Offline `YOLOv5s` results:

- `cell phone` target, confidence `0.5` down to `0.05`: `0/50` correct
  detections.
- Any object, confidence `0.5`: `6/50` detections, all labeled `tie`.
- Any object, confidence `0.25`: `50/50` detections, `49` labeled `tie` and `1`
  labeled `remote`, average confidence about `0.45`.

Interpretation: this back-side phone setup is dominated by the large sticker and
is not a useful semantic `cell phone` demo. It may be usable as a proxy target
only if the detected `tie` box is acceptable, but the next phone test should use
the screen side or a cleaner phone back without a large sticker.

A Samsung Z Flip back-side sample on the white background was then tested:

```text
datasets/vision/20260601T153024Z_cell_phone
```

Offline `YOLOv5s` results:

- `cell phone` target, confidence `0.5`: `0/50` correct detections.
- `cell phone` target, confidence `0.25`: `5/50` correct detections.
- `cell phone` target, confidence `0.1`: `43/50` correct detections, average
  confidence about `0.18`.
- `cell phone` target, confidence `0.05`: `47/50` correct detections, average
  confidence about `0.18`.
- Background-only dataset with `cell phone` target, confidence `0.1` and
  `0.05`: `0/50` false positives.
- Live perception-service smoke test at confidence `0.05` returned
  `label=cell phone`, confidence about `0.36`, and detector latency about
  `120ms`.

Interpretation: the Z Flip back-side setup is a possible second semantic target
after the cup, but it needs a lower target-filtered threshold than the cup. It
is deferred until the cup perception/alignment demo is reliable end to end.
