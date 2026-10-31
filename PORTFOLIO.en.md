# MotionBrain Portfolio

[한국어](PORTFOLIO.md) · [Project and demo](README.en.md)

An ESP32/STM32/camera/Raspberry Pi/ROS2 integration project that distinguishes sending a command from physically reaching a target. The strongest physical results are M4 single-axis feedback control and system integration. Following detector development training, the first M2 action predictor linking observations and commands has been fitted and evaluated. No learned-policy manipulation result has been demonstrated.

## Problem and design

The low-cost DC arm cannot measure all joint positions or grasp force. Time-based actuation varies with load, inertia and friction. An AS5600 was added to the M4 shoulder; ESP32 handles outputs, STM32 handles sensing/teleoperation, and Pi handles perception and ROS2.

- **Command boundaries:** legacy serial and web manual commands use `Dispatcher` and `SafetyGate` with state, sensor-fault and deadman checks. Experimental `/actions` use a separate bounded executor that checks `SafetyGate` and output state. Web manual commands expire; the control loop stops outputs when it detects expiry. A physical stop-latency bound including blocking communication is unverified.
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

On 2026-10-17, an external observer selected manual commands from external camera views to grasp and lift a small cylindrical target object. Two additional grasp/lift/place cycles followed; the first required one extra close correction. Twenty-one command/output steps and 37 images were preserved privately. [Demonstration and limits (Korean)](docs/evidence/2026-10-17-lipbalm-repetitions.md)

## From detector development to action learning

Faster R-CNN was trained on 12 images of a single target object type from an earlier and a new capture session. Final training-set results were target object TP8/FP1/FN0, gripper TP12/FP0/FN0, and relative position/direction 8/8 each. The four detector-training runs total 460 updates; the remaining duplicate false positive and training-set scope are retained. Before the additional training, the old model detected target object TP0/4 and gripper TP0/6 on the new six images. Those images were subsequently used for training, so they are not a held-out evaluation of the final model. [Data, criteria and results (Korean)](docs/evidence/2026-10-30-cv-development.md)

The current goal is **learning the consequences of the arm's own actions and using those predictions to approach a target**. Fixed external camera observations, recent command history and available sensors condition predictions of arm-feature changes under candidate PWM/duration commands. Still-image detection remains an observation aid; joints without encoders can join the pilot when their motion is observable.

The [visual action recording core (Korean)](docs/visual-action-collection.md) connects before/during/after frames with commands and responses. A separate repeat-collection path produced 32 audited M2 projected-orientation transitions, split 16/8/8 for training/validation/test. Linear/ridge candidates were fitted; validation selected a linear model with test MAE 0.665° (no-change baseline 6.797°) and direction agreement 8/8. This is same-day/view 2D observation prediction, not physical joint-angle accuracy or model-based manipulation. It is distinct from eligibility under the existing object-box/M4 data contract. [Conditions, denominators, failures and evaluation (Korean)](docs/evidence/2026-10-31-first-action-prediction.md) · [Next stages (Korean)](docs/visual-action-learning.md)

Earlier investigations studied camera/USB clock linkage and output evidence under fixed image/sensor-age contracts. Their measurements and failures remain in [Mac camera instrumentation (Korean)](docs/mac-direct-camera.md) and the [existing action-data contract (Korean)](docs/data-pipeline.md). The new slow-action/settled-observation approach needs a separate contract; old criteria are not silently relaxed and manual demonstrations are not promoted into action-training data.

## Validation and remaining work

Software checks run in [PlatformIO/Python](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml) and [ROS2](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml) CI. Hardware results are limited to the dated conditions above. Raw photographs and device-specific operations records are not part of the public documentation.

- The other four joints lack position feedback; full-arm trajectory control and force sensing are absent.
- Electrical cutoff, deadman latency, power transients and long-term reliability still require instrumentation.
- Next, join bounded commands with external camera observations before and after motion, then compare motion predictions against baselines on held-out starting poses/sequences. Goal approach and grasping are evaluated separately afterward.
