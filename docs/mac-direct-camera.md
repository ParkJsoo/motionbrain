# Mac direct-camera commissioning

`tools/probe_mac_camera.py` runs one finite AVFoundation acquisition and audits the
exact delivered image pixels and native processing results. It does not call a
controller, start a service, generate action transitions, or train a model.
The macOS helper requires macOS 14 or later and an explicitly selected Continuity
Camera; it never falls back to another camera. Python audit is platform independent.

## Build and run

Build with a matching Xcode compiler and SDK. The embedded Info.plist declares the
camera permission purpose and opts into Continuity Camera discovery. Output binaries
and camera data belong in Git-excluded paths.

```sh
mkdir -p .local/bin
bash tools/macos/build_camera_probe.sh .local/bin/mac-camera-probe
python3 tools/probe_mac_camera.py list --helper .local/bin/mac-camera-probe
python3 tools/probe_mac_camera.py capture \
  --helper .local/bin/mac-camera-probe \
  --device-id '<explicit Continuity Camera unique ID>' \
  --view-id '<new stable mount and scene configuration ID>' \
  --output datasets/mac_camera/<new-run>
python3 tools/probe_mac_camera.py audit datasets/mac_camera/<new-run>
```

Run capture through the Python wrapper: native AVFoundation and filesystem calls
can block, so the wrapper supplies the independent process deadline. A fresh output
directory is mandatory; there is no resume, overwrite, automatic retry, or camera
fallback. A new command is a new experiment and requires a separately justified
budget; a failed run must not be silently replaced.

The iPhone must be locked, stably mounted and trusted by the Mac for USB use.
A cable connection alone is not evidence of the actual video transport: raw
AVFoundation `transportType` is recorded, and `transportVerifiedUSB` stays false.
See [Apple's Continuity Camera requirements](https://support.apple.com/en-us/102546).

## Fixed acquisition contract

- One process, startup at most 30 seconds; fixed warmup ends after 1 second or
  30 callbacks, whichever occurs first.
- One reference frame and 20 evaluation frames; measurement at most 10 seconds,
  process at most 45 seconds, followed by at most 2 seconds termination grace.
- Serial callbacks, late video frames discarded and counted. Requested output is
  640×480 BGRA; actual dimensions, row layout, rotation and mirroring are recorded.
- Raw tightly packed BGRA pixels are copied without row padding. This is the
  delivered AVFoundation image, not a sensor RAW or pre-transport image.
- Native processing computes the exact weighted mean luma
  `(77*R + 150*G + 29*B)/256`. Audit recomputes it from the preserved bytes.
  This is image processing, not semantic detection, tracking or learned inference.
- Raw image and frame metadata are exclusively created, fsynced, read back, and
  the directory is fsynced. A separate terminal receipt binds both SHA-256 hashes
  and records the complete transaction time. The 250 ms bound applies to this
  transaction, not to the later terminal receipt, execution log or offline audit.
- Storage failure or duration over 250 ms, invalid/duplicate/regressing PTS,
  device loss, changed dimensions/rotation/mirroring, or deadline ends the run.
  Partial files remain available for audit.

The frame-zero reference is a temporal diagnostic reference. It is **not** the
semantic object anchor required by the existing ESP32 observation pipeline.
Physical mount stability and scene changes cannot be proved by these metadata.
A moved camera or changed scene needs a new view ID and session; old calibration,
anchors and action-training groups must not be reused.

## Timestamp and result meaning

Every image has a unique session/sequence identity, original rational CMTime PTS
(value, timescale, flags and epoch), converted host PTS, callback arrival, processing
completion and storage timing. All live timings use the CoreMedia host clock.
AVCaptureSession's synchronization clock is used for PTS conversion with
`CMSyncConvertTime`; Python monotonic time is used only for subprocess supervision.
See Apple's [synchronizationClock](https://developer.apple.com/documentation/avfoundation/avcapturesession/synchronizationclock)
and [CMSyncConvertTime](https://developer.apple.com/documentation/coremedia/cmsyncconverttime(_:from:to:)) contracts.

`captureProcessingPassed` requires all 21 images to pass integrity and storage
checks, all 20 evaluation images to complete, and at least 16 to have PTS-to-processing
diagnostics within 100 ms and reference-PTS age within 15 seconds. This small sample
is a commissioning smoke test, not a statistical reliability claim.

Converted presentation timestamps and callback arrival do not establish optical
exposure time or its error bound. Therefore the real image-age ≤100 ms and anchor-age
≤15 seconds gates remain `unverified`, `overallPassed` stays false, and
`captureHostBoundsCalibrated`, `controlEligible`, and `trainingEligible` stay false.
Passing pixel-processing diagnostics cannot promote an observation or action dataset.
No actual learning performance is measured by this tool.

Run software regression separately from physical acquisition:

```sh
python3 -m unittest discover -s tests -p 'test_mac_camera_probe.py' -v
```

## Commissioning result (2026-10-16)

The first bounded run found the intended Continuity device during preflight but
could not obtain camera authorization within its startup budget. It exited with
code 3 (`camera permission unavailable`) after 30.265 seconds, with **zero captured
frames**. That attempt did not test the 21-image acquisition or timing criteria on
physical images; it is a failed commissioning attempt, not measured camera
latency. No automatic retry or threshold adjustment was performed.

The failed execution, frozen plan, helper hash, stderr and audit report were
preserved locally. Independent replay rejects the incomplete evidence. Controller
status before and after remained IDLE, all five motor outputs zero, and the bounded
action API disabled with no action records. No actuator command, firmware upload
or service start/restart was issued.

Software validation: 669 regression tests passed, including 33 synthetic/fake-process
camera tests; the macOS helper built successfully. Independent review found and
resolved an audit gap that had allowed missing execution records; final review
reported no remaining P0/P1/P2 findings. Software checks do not establish USB video
transport, optical timing, real detection accuracy or real learning performance.


### Separate run after camera authorization

A separately authorized new session used the same executable, processing code and
fixed criteria. The capture process completed in 3.169 seconds: all 21 raw frames, metadata
and native pixel measurements passed integrity replay. The actual delivered format
was **1920×1080 BGRA**, despite the requested 640×480 session preset; no post-result
resolution change or new capture was performed. Actual frame dimensions, rather
than the requested preset, determine the evidence configuration.

| Diagnostic | Result |
|---|---:|
| Evaluation frames completed | 20/20, plus one reference |
| Evaluation PTS-to-processing ≤100 ms | **12/20; required 16/20 — failed** |
| Evaluation PTS-to-processing min / median / max | 75.453 / 81.840 / 115.418 ms |
| Evaluation callback-to-processing min / median / max | 0.732 / 1.995 / 2.251 ms |
| Maximum reference-PTS age at processing | 747.274 ms |
| Maximum raw+metadata durable transaction, all 21 frames | 12.162 ms |
| Callback-reported dropped frames | 0 |

These timing metrics use presentation timestamps, not verified optical exposure
bounds. The reference-age diagnostic is not a validated semantic-anchor age.
`captureProcessingPassed=false` because the fixed 16/20 diagnostic threshold was
missed; `overallPassed=false` additionally retains the unverified optical timing
limits. Controller outputs remained off and no action record was created. Actual
learning-eligible action data remains **zero**. Raw scene images stay outside Git.
The run ended at the original sample limit; there was no further capture or tuning.

## Optional callback and format diagnostics

New wrapper runs require instrumentation version 1; immutable legacy captures remain
readable without those diagnostics. The helper's `--describe-capabilities` is a
non-capturing compatibility check. Session metadata records the pre-start preset,
output settings and active format. Each frame binds the actual settings, active and
sample formats, delivered pixel format, orientation and device-reported frame-duration
values with a SHA-256 hash; a configuration change aborts the run. Unset device
CMTime duration values remain raw diagnostics and never establish an observed FPS.

Up to 30 warmup callbacks and 64 dropped-sample records are retained, with an explicit
truncation count. Missing warmup timing or dropped-sample reasons remain unknown.
Terminal receipts record callback return-preparation timestamps after persistence
and output flushing, and lifecycle timestamps through delegate-queue draining.
A return-preparation timestamp is sampled **before** callback return; it cannot prove
actual dispatch readiness or isolate camera ISP, transport and upstream queue delay.
These diagnostics do not change the fixed acceptance criteria or establish exposure time.

## Frozen offline object-detector replay

```sh
python3 tools/evaluate_mac_camera_detection.py \
  --capture-dir datasets/mac_camera/<existing-complete-run> \
  --model .local/models/yolov5s.onnx --labels config/coco80.labels \
  --output .local/detector-evaluation/<new-run>
```

The replay validates all original evidence before freezing model, label, implementation
and input hashes. A timing-only failure is allowed for offline analysis and remains
recorded. It executes one CPU worker with a 60-second deadline and 2-second termination
grace, one warmup on frame zero and exactly 21 evaluations. Settings are fixed to
YOLOv5s ONNX, input 640, cup only, confidence 0.20, NMS 0.45, largest target, no aliases.
OpenCV parallel regions are disabled and its effective thread setting must report one.
BGRA is converted to BGR and lossless PNG for the existing detector entry point.
No capture, action, training, retry or parameter search is performed.

Outputs distinguish input preparation, detector-call time and total offline processing.
Detection presence, confidence and boxes are not accuracy: there are no manually
annotated ground-truth boxes or independent test scenes. Replay timing is not live
capture-to-detection latency. All learning/control eligibility fields remain false.
