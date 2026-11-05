# MotionBrain Portfolio

[한국어](PORTFOLIO.md) · [Project overview and setup](README.en.md) · [Evidence index (Korean)](docs/README.md)

**A personal robotics project that learns motion from images and commands, then compares predicted outcomes to approach a goal.** On a five-axis DC arm, it connects control and instrumentation to a collection UI, raw-data validation, dataset construction, prediction training/evaluation and limited candidate selection.

The current results are a physical-data pipeline and bounded motion-prediction experiments. Learned grasping that derives object/gripper state from images and coordinates alignment, closing and lifting is not complete. This document focuses on engineering decisions, problem solving and measured outcomes.

## 1. Connecting physical robot records to training data

Requested PWM and duration can differ from actual output history, and communication failures can leave execution uncertain. Only the M4 shoulder has an angle sensor; other joint angles cannot be presented as measurements. Saving only video or successful runs would hide these distinctions.

The implemented path addresses them as follows:

- **Collection and replay:** select one of M1–M5, request a bounded pulse, and preserve lossless frames, command intent, driver outputs and sensor responses. Completed, failed and interrupted sessions remain replayable.
- **Execution evidence:** journal intent before dispatch and check boot identity, output sequence and all-axis output state. Uncertain commands are not automatically resent; restarting the server does not erase an incomplete journal.
- **Datasets and evaluation:** successful storage is separate from training eligibility. The M4 path audits raw hashes, image/sensor windows and execution evidence before building datasets, and checks overlap between training and subsequent evaluation data.

Pre-action models use only pre-command images and planned commands. Actual outputs and later sensor values serve execution checks, labels and retrospective analysis, preventing future-information leakage into inputs. Driver records are software PWM values, not electrical waveform measurements. The collection UI supports all joints. Existing sensor-label training covers M4 downward motion; a separate M1 relative-state path uses manual gripper-camera annotations.

Implementation: [collection state management](learning/manual_collection.py) · [transport and command journal](learning/supervised_transport.py) · [dataset audit](learning/manual_collection_dataset.py). [Usage, failures and validation scope (Korean)](docs/manual-data-collection.md)

## 2. Comparing visual prediction against baselines

On 2026-11-04, the experiment covered six M4 downward conditions: 20/25/30% output × 350/500ms requested duration. Models were **trained and frozen on 18 records**, three per condition, then **evaluated without retraining on 18 newly collected records** after repositioning. Image features are an 8×4 grayscale reduction of the entire frame; labels are before/after sensor-angle changes.

| Frozen comparison on the new 18 records | Mean absolute error (MAE) |
| --- | ---: |
| No change | 2.154° |
| Overall training mean | 0.624° |
| Per-command training mean | **0.367°** |
| Command-only ridge | 0.595° |
| Image-and-command ridge | 0.605° |

**Adding these coarse image features did not demonstrate an improvement.** The per-command mean performed better, so a small prediction error was not treated as evidence of visual understanding or autonomous control. The next focus is representing object/gripper relative position and pose changes, then testing whether those observations improve on command-only baselines before increasing model size.

This is same-day, single-device retrospective replay over sequential poses. The new batch had 18 attempts, 18 completions and 18 eligible records with 1,935 frames, but followed four batches stopped before motion; their failures and unstarted slots remain recorded. The result does not establish cross-day/load generalization, sensor accuracy, calibrated prediction intervals or control success rates.

[Conditions, denominators, failures and complete comparison (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) · [Training implementation](learning/manual_collection_learning.py) · [Candidate comparison](learning/manual_action_selection.py)

## 3. How far prediction has been connected to action selection

A separate M2 experiment split 32 image-projected relative-orientation transitions into 16 training, eight validation and eight test records. The validation-selected linear model achieved test MAE **0.665°**, compared with 6.797° for no change. These are small logical groups from the same day/view, not measurements of physical joint-angle accuracy.

One M2 command selected by comparing model predictions reduced image-projected goal error from **5.479° to 0.967°**. Of four trials that started capture, the preceding three stopped before motion. This demonstrates a single observation–prediction–selection–bounded-execution connection, not a repeated policy result or grasp success rate.

M4 also has an offline function comparing predicted candidates against a target change. However, it does not derive that sensor-change target automatically from object location, and is not a visual grasping policy. The 32 M2 transitions, a separate set of 17 settled M2 transitions and the new M4 records use different observation contracts and are not automatically mixed.

[First M2 prediction evaluation (Korean)](docs/evidence/2026-10-31-first-action-prediction.md) · [Single goal approach (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [Selector contract (Korean)](docs/offline-action-selection.md)

The [offline relative-state path (Korean)](docs/relative-action-selection.md) connects manual object/jaw annotations on two M4 transitions to change prediction and candidate ranking. A [gripper-camera pilot (Korean)](docs/evidence/2026-11-04-gripper-relative-learning.md) trained on four new M1 transitions and evaluated one later record, where no-change was best. These are small development evaluations with approximate manual annotations; they do not establish automatic perception or grasping.

A [subsequent joint-specific study (Korean)](docs/evidence/2026-11-05-multi-joint-relative-learning.md) collected 33 transitions across separate M1, M5 and M2 contracts, with 24 training and nine later evaluation records. Range rejections and large errors remain; learned grasping and lifting are unverified.

## 4. Control and communication work supporting learning experiments

**Correcting false target success.** M4 control initially ignored inertial motion after output cutoff, returning success at 235.37° for a 234° target. Settled-error checks, bounded correction and `TARGET_MISSED` were added. Final regressions passed 11/11 each with no added load and with 23.10g, at mean absolute errors of 0.132° and 0.155°, using a fixed mount and a 230–245° range. These do not establish accuracy for the other four axes or the whole workspace. [2026-06-28 measurements, failures and corrections](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md)

**Separating communication failure stages.** On 2026-11-04, status requests were instrumented across address lookup, connection, headers and body. One path closed the connection after a 100ms first-byte wait before TCP could recover; another exceeded 0.8s while reassembling the response body. Subsequent changes adjusted the first-byte wait and reduced collection responses, but intermittent timeouts remain. Commands were not automatically resent to improve completion rates. [Diagnostic evidence (Korean)](docs/evidence/2026-11-04-http-response-gap.md) · [Subsequent collection-path changes (Korean)](docs/manual-data-collection.md)

The control foundation includes ESP32 PWM/state guards, STM32 sensing/teleoperation, M4 AS5600 feedback and ROS2 integration. Physical ROS2 writes were validated only for an operator-confirmed single M4 target, not whole-arm trajectory tracking. [ROS2 physical validation](docs/evidence/2026-07-13-m4-physical-ros2-control.en.md)

## Validation scope and next work

Software regressions and firmware builds run in [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml); ROS2 builds and tests run in [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). Physical results retain their dated conditions and denominators separately.

Earlier [manual grasp demonstrations (Korean)](docs/evidence/2026-10-17-lipbalm-repetitions.md) and [12-image detector development training (Korean)](docs/evidence/2026-10-30-cv-development.md) are supporting experiments. The manual demonstrations are not learned-policy results; detector results evaluate the final model on its training images.

The core remaining work is to connect visual object/gripper state to action prediction, implement and evaluate repeated action selection and reobservation, then separately test grasp/lift successes and failures under bounded conditions. Whole-arm position/force feedback, an electrical emergency-stop latency bound, learned continuous manipulation and generalization to other environments are not established.
