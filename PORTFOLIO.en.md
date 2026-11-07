# MotionBrain Portfolio

[한국어](PORTFOLIO.md) · [Project overview and setup](README.en.md) · [Evidence index (Korean)](docs/README.md)

**A personal project that turns physical robot images and commands into training data, then connects predicted outcomes to bounded action selection.** It combines a collection UI, two cameras, source validation, model training/evaluation and execution paths on an ESP32-controlled five-axis DC arm.

The current task connects joint-specific experiments into **M1 gripper close → M4 shoulder lift**. Automatic alignment and learned grasping are not complete. This document explains the problems addressed, design decisions and observed results.

## Turning physical motion into usable records

Requested output and duration can differ from actual output history. Communication failures can leave execution uncertain, while even a normally stopped motor may lack a reliably measured visual after-state. Only the M4 shoulder has an angle sensor, so other joints cannot be evaluated in the same way.

Collection, execution confirmation and training eligibility are therefore separate.

- **Collection and replay:** a finite single-joint action binds images, command intent, actual outputs and sensors. Failed, interrupted and unstarted records are preserved.
- **Execution confirmation:** a pre-command journal and device boot/output history bind requests to actual actions. Uncertain commands are not automatically resent.
- **Dataset audit:** source hashes, capture times, before/after windows and output history are checked, together with overlap between training and evaluation data.

Pre-action inputs use only visual state available at that time and planned commands. Actual outputs and post-action sensors serve execution checks, labels and retrospective analysis. Driver history records software PWM values, not electrical waveforms. Different joints, poses and camera conditions are not automatically mixed.

Implementation: [collection state management](learning/manual_collection.py) · [command journal and transport](learning/supervised_transport.py) · [source audit](learning/manual_collection_dataset.py). [Usage, failure handling and scope (Korean)](docs/manual-data-collection.md)

## Testing whether images improve prediction

### M4 downward motion: 18 training records and 18 new frozen evaluations

On 2026-11-04, six conditions combined 20/25/30% output with 350/500ms requested duration. Models were **trained and frozen on 18 records**, three per condition, then **evaluated without retraining on 18 new records** after repositioning. Image features are an 8×4 grayscale reduction of the full frame; labels are sensor-angle changes.

| Frozen comparison on the new 18 records | Mean absolute error (MAE) |
| --- | ---: |
| No change | 2.154° |
| Overall training mean | 0.624° |
| Per-command training mean | **0.367°** |
| Command-only ridge | 0.595° |
| Image-and-command ridge | 0.605° |

**Adding these coarse image features did not demonstrate an improvement.** The per-command mean was better, motivating a check of visual information quality and subsequent development of object/gripper relative-state representations.

This is same-day, single-device retrospective replay across sequential poses. The new batch had 18 attempts, 18 completions and 18 eligible records with 1,935 frames; four earlier batches stopped before motion and remain preserved. These figures do not establish cross-day/load generalization, sensor accuracy, calibrated prediction intervals or control success rates.

[Conditions, denominators, failures and full comparison (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) · [Training implementation](learning/manual_collection_learning.py) · [Candidate comparison](learning/manual_action_selection.py)

### M4 upward motion: separating prediction evaluation from observed lifting

With a manually pregrasped object and a fresh reference view, an **M4 model including a 500ms candidate** was trained/frozen on four new records. Two new records were evaluated without retraining. Object-rise MAE was **2px**, tied with the **2px** command mean. Including three returns, nine commands completed normally; physical selection by this new model and the close-to-lift connection remain at zero.

An earlier, separate 650ms model selected up at 50% for 650ms from two candidates, and two cameras observed one physical lift. However, **3.08° of movement exceeded the 3° guard, leaving the execution failed**. Lifting an already manually grasped object was not presented as automatic alignment/closing or repeated success.

[Models, evaluations and observational aborts by experiment (Korean)](docs/m4-up-commissioning.md)

## Connecting model selection to physical closing and lifting

As of 2026-11-08, the first whole-arm connection is **M1 close → fresh contact observation → M4 lift**, with other joints fixed. The connector separately checks the close result, output OFF, dual-camera contact evidence and M4 input conditions. Failed or uncertain outcomes block the next action.

The current pose lay outside the earlier M1 model's support. Instead of forcing that model to accept the input, a separate observation contract registered fixed-view object outlines and jaw features. A new M1 model was frozen on **four records: two open and two close**. The initial close record's failed after-observation was preserved; source execution evidence and eligibility under the new observation contract were audited separately.

This batch used **seven physical commands: four for training and three for preparation**. Preparation stopped when competing observation matches could not be distinguished sufficiently; even the rejected best-match approximation lay outside the frozen model's support. Consequently, **post-freeze evaluations, model-selected closes and connected close/lift trials are all zero**. Removing an observation rejection alone would not establish a valid model input.

This work connects fixed registration, original images, observation methods and model support checks. Current tracking measures limited features for a particular view/object; it does not establish general object perception, depth, contact or physical-length accuracy.

[Current connection attempt, observation failure and remaining conditions (Korean)](docs/evidence/2026-11-08-close-lift-integration.md) · [Sequential trial and connection contracts (Korean)](docs/model-grasp-trial.md) · [Observation implementation](learning/registered_gripper_landmarks.py)

### Earlier single-action selection and development experiments

A separate M2 experiment split 32 image-projected orientation transitions into 16 training, eight validation and eight test records. Test MAE was **0.665°**, versus **6.797°** for no change. One model-selected M2 command reduced image-projected goal error from **5.479° to 0.967°**. Of four trials that started capture, the preceding three stopped before motion. These are small logical groups from the same day/view, not physical joint-angle accuracy or repeated grasping performance.

Other records retain their own observation contracts and limits.

- [Four M1 training transitions and one later evaluation (Korean)](docs/evidence/2026-11-04-gripper-relative-learning.md): no-change was best on that single evaluation.
- [Joint-specific M1/M5/M2 expansion (Korean)](docs/evidence/2026-11-05-multi-joint-relative-learning.md): 33 collected transitions, with 24 training and nine subsequent evaluations. Range rejections and large errors remained.
- [Pose-separated follow-up (Korean)](docs/relative-action-selection.md#2026-11-07-자세를-분리한-학습과-구동-전-예측-확인): separate M1/M4 models used four/two records; one new M1 comparison favored no-change.
- [M3/M4 marker study (Korean)](docs/evidence/2026-11-06-projected-arm-m3-m4-learning.md): nine/four training records, respectively. New post-freeze records were rejected for observation quality or input support, leaving no formal prediction-error metric.

[First M2 prediction evaluation (Korean)](docs/evidence/2026-10-31-first-action-prediction.md) · [Single goal approach (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [Selector contract (Korean)](docs/offline-action-selection.md)

## Control and communication improvements supporting experiments

| Problem | Change and verified scope |
| --- | --- |
| Target success ignored inertial movement after output cutoff | After reporting success at 235.37° for a 234° target, settled-error checks, bounded correction and `TARGET_MISSED` were added. At a fixed mount over 230–245°, no added load and 23.10g each passed 11/11, with MAE 0.132° and 0.155°. These are bounded M4 sensor-reference results |
| Intermittent HTTP timeouts | Requests were instrumented across lookup, connection, headers and body. A 100ms first-byte wait and a response-body reassembly path exceeding 0.8s were distinguished; waits and response size were adjusted. Intermittent latency remains and was not hidden with command retries |
| Physical control through ROS2 | ESP32 output guards, STM32 sensing and M4 AS5600 were connected. Physical ROS2 writes were validated only for operator-confirmed single M4 targets, not whole-arm trajectory tracking |

[Control corrections and measurement conditions](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md) · [HTTP diagnostic (Korean)](docs/evidence/2026-11-04-http-response-gap.md) · [Physical ROS2 validation](docs/evidence/2026-07-13-m4-physical-ros2-control.en.md)

Software regressions and firmware builds run in [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml); ROS2 builds/tests run in [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). Physical results retain their dated conditions and denominators separately. Earlier [manual grasp demonstrations (Korean)](docs/evidence/2026-10-17-lipbalm-repetitions.md) and [12-image detector development training (Korean)](docs/evidence/2026-10-30-cv-development.md) are distinct from learned-policy results and independent perception evaluation.

## Whole-arm next steps

The nearest task is reliable observation and supported M1 inputs at the current pose, followed by separate frozen evaluation, model-selected closing and M4 lifting. Then extend to the necessary approach joints and poses, evaluating repeated successes and failures.

Kinematics will progressively constrain reachability and movement clearance. Whole-arm inverse-kinematics/dynamics control and a reinforcement-learning policy have not been applied; current learning is supervised transition prediction and candidate comparison. Extend RViz, Rerun and W&B use for whole-arm observation and experiment comparison, with Gazebo/MuJoCo contact and dynamics studies following geometry and actuator-response calibration.

Whole-arm position/force feedback, an electrical emergency-stop latency bound, learned continuous manipulation and generalization to other environments remain unverified. Training every joint or completing a simulator is not a prerequisite for the first grasp.
