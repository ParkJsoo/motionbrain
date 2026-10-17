# MotionBrain Portfolio

[한국어](PORTFOLIO.md) · [Project and demo](README.en.md)

An ESP32/STM32/camera/Raspberry Pi/ROS2 integration project that distinguishes sending a command from physically reaching a target. The strongest physical results are M4 single-axis feedback control and system integration. Learned manipulation remains at the data-validation and offline-baseline stage.

## Problem and design

The low-cost DC arm cannot measure all joint positions or grasp force. Time-based actuation varies with load, inertia and friction. An AS5600 was added to the M4 shoulder; ESP32 handles outputs, STM32 handles sensing/teleoperation, and Pi handles perception and ROS2.

- **Shared command boundary:** serial and HTTP inputs use `Dispatcher` and `SafetyGate` with state, sensor-fault and deadman checks. Web manual commands expire; the control loop stops outputs when it detects expiry. A physical stop-latency bound including blocking communication is unverified.
- **Measured versus estimated state:** M4 measured angles are distinguished from open-loop estimates for other joints in ROS2.
- **Confirmed ROS2 writes:** `ros2_control` proposals are not forwarded automatically. A separate executor sends one M4 target after operator confirmation within the default 20-second validity window. This is not a 20-second motion duration.

Implementation: [controller](src/control/), [ROS2 hardware interface](ros2_ws/src/motionbrain_hardware_interface/README.en.md), [command journal](ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/m4_command_journal.py).

## Case 1 — Correcting false M4 target success

The initial controller did not recheck settled error after cutting output: a 234° request stopped at 235.37° but returned success. Settled-error validation, bounded correction and `TARGET_MISSED` were added.

At 23.10g, no-load inertia compensation and PWM ramp timing caused a short upward move to fail. Upward correction duration increased from 250 to 500ms; the internal success window tightened to ±0.40° to allow for a sensor step after stopping. The external ±0.50° acceptance criterion remained unchanged.

| Final regression condition (2026-06-28) | Result | Mean / maximum absolute error |
| --- | ---: | ---: |
| No added load | 11/11 | 0.132° / 0.36° |
| 23.10g | 11/11 | 0.155° / 0.31° |

These results use the same fixed mount and a 230–245° validation range. They do not establish whole-arm accuracy or performance under other loads/battery conditions. [Conditions, failures and final results](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md)

A separate ROS2 physical-path test moved from 248.20° toward 250.00° and ended at 249.96°. Duplicate confirmation and disarmed requests were not forwarded. This is single-axis target validation, not trajectory tracking. [2026-07-13 results](docs/evidence/2026-07-13-m4-physical-ros2-control.en.md)

## Case 2 — Status delays caused by an empty HTTP connection

Status checks intermittently failed around actuation. Instead of replaying uncertain commands, an empty TCP connection and queued request reproduced one failure path. Arduino WebServer's 5000ms initial-byte wait was identified; a project-local adapter now releases only initial connections with no readable data after 100ms.

The same queued GET changed from a reset after 1198.4ms to a successful 39.4ms response. A later status request still timed out at about 830ms on another path, so this is not a complete communication fix. [Reproduction, correction and residual failure (Korean)](docs/evidence/2026-10-17-idle-http-client.md)

## Observed grasping and learning status

On 2026-10-17, an external observer selected manual commands from iPhone views to grasp and lift lipbalm. Two additional grasp/lift/place cycles followed; the first required one extra close correction. Twenty-one command/output steps and 37 images were preserved privately. [Demonstration and limits (Korean)](docs/evidence/2026-10-17-lipbalm-repetitions.md)

Perception runs pretrained YOLOv5s through OpenCV DNN/ONNX; no custom detector-training result is claimed. A NumPy ridge action-outcome model, no-change baseline, session-grouped splits, train-only normalization and separate training/evaluation CLIs are implemented. **Eligible real training data remains zero.**

Action-linked observations meeting 100ms image age, 15-second anchor age and 250ms durable storage are still missing. A no-actuation observation trial completed only one of seven executed requests before stopping on persistence latency. Both October 16 Mac camera runs achieved 12/20 PTS diagnostics against a required 16/20; a separate October 17 run in a new view passed 20/20. Optical exposure timing remains unverified. [Camera measurement conditions (Korean)](docs/mac-direct-camera.md) Grasp demonstrations are neither synchronized training transitions nor learned-policy results. [Data contracts, reproduction and limits (Korean)](docs/data-pipeline.md)

An offline collection review joins saved transitions with lipbalm semantic anchors, optical measurements, original persistence receipts, M4 stability intervals, visual whole-arm settling and output records after OFF, reporting missing evidence. Synthetic regression tests cover these joins; producer instrumentation and physical collection remain outstanding. A passing review does not grant training eligibility or execution approval.

A same-Mac diagnostic now links video PTS and M4 polling markers through bounded host-clock conversion, with independent recomputation from saved originals. The conversion interval was approximately −1.08 to +1.12µs, but HTTP M4 collection age met the 20ms limit in 0/6 samples. A separate wired diagnostic waits for the next regular update, distinguishes MCU waiting time, and offers an information-equivalent 58-byte response. The first compact trial stopped after two attempts because a partial log followed the response; after fixing receive boundaries, a separate six-sample trial produced diagnostic age upper bounds of 20.559–32.929ms, still 0/6 within 20ms. A later six-sample trial with RAM stage instrumentation produced 19.318–27.423ms, with 4/6 within the limit. It separates regular-update waiting after request parsing from angle-read and response stages; one run does not establish a stable improvement. This calculation includes an explicit MCU clock-rate error assumption, not a measured clock calibration. Optical exposure and physical sensor-sample alignment remain unverified. [Conditions and limitations (Korean)](docs/mac-direct-camera.md)

A combined producer now assigns a common session ID before capture to six USB M4 responses and 21 video frames, linking them through the host-clock ID in USB responses and clock-conversion evidence. Its saved-original audit passed. In the first combined run, the PTS-to-processing diagnostic met 100ms in 20/20 evaluation frames, and original persistence met 250ms in 21/21 frames. M4 age met 20ms in 2/6 samples; nearest video-PTS/poll-marker interval distance met 20ms in 6/21 frames. These are different metrics. Clock linkage is implemented, but optical exposure, physical sensor sampling and action-transition alignment remain unverified; physically training-eligible transitions remain zero.

## Validation and remaining work

Software checks run in [PlatformIO/Python](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml) and [ROS2](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml) CI. Hardware results are limited to the dated conditions above. Raw photographs and device-specific operations records are not part of the public documentation.

- The other four joints lack position feedback; full-arm trajectory control and force sensing are absent.
- Electrical cutoff, deadman latency, power transients and long-term reliability still require instrumentation.
- The next step is to connect verified image timing/anchors with actual outputs, collect data, and compare against a no-change baseline using separate sessions. Prediction error and physical manipulation success need separate evaluation.
