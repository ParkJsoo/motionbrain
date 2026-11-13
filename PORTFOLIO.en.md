# MotionBrain — Connecting Predictions to Physical Actions

[한국어](PORTFOLIO.md) · [Project overview](README.en.md) · [Design and experiment evidence (Korean)](docs/README.md)

**A personal project that predicts a robot arm's next state from images and planned commands, then compares candidate outcomes to approach a goal.** It connects firmware, observation and collection, dataset audits, supervised learning and bounded execution on an ESP32-controlled five-axis DC arm.

The central questions go beyond whether the arm moved: **Was the outcome predicted using information available before execution? Was it better than a simple command mean? Can that prediction support the next action?** M4 shoulder sensor measurements are distinguished from visual observations of other joints, and a single goal approach from repeatable grasping. An automatic **M1 close → M4 lift** sequence has not completed normally.

## 1. Making success reflect the observed outcome

### Output cutoff and target arrival were different events

M4 position control returned `TARGET_REACHED` for a 234° target even though the arm settled at 235.37° after cutoff. Success reflected the cutoff instant rather than the final state including inertial motion. Direction-dependent stopping lead, post-cutoff settling checks, bounded correction and `TARGET_MISSED` were added. For short loaded rises, the correction pulse was also adjusted to give the PWM ramp time to overcome friction.

| Final regression conditions, 2026-06-28 | Targets reached | Sensor-reference mean absolute error (MAE) |
| --- | ---: | ---: |
| Fixed mount, no added load | 11/11 | 0.132° |
| Same mount, 23.10g load | 11/11 | 0.155° |

These are **bounded M4 sensor-reference tests over 230–245°**. Earlier failures and records preceding the tightened success criterion were not merged into the final denominator. The figures do not establish absolute joint-angle calibration accuracy, other joints' positions, grasp force or whole-arm trajectory tracking.

The decision was to move the success condition from the output command to the **observed settled outcome**. Later learning experiments likewise record motor completion, observation success, evaluation eligibility and goal attainment separately.

[Faults, corrections and load-specific regressions](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md) · [Separate physical validation of single M4 targets through ROS2](docs/evidence/2026-07-13-m4-physical-ros2-control.en.md)

### Separating requests, execution and usable training data

Saving the requested command alone was insufficient. Requested output differs from actual ramp output, and an HTTP timeout cannot distinguish non-execution from a lost response. A saved image may precede the wrong event or contain ambiguous after-state features, making it unsuitable as an input or label.

A finite single action is therefore checked through this chain:

**Pre-command journal → boot and actual output history → source images and times → before/after observations → training/evaluation eligibility**

- Uncertain commands are not automatically resent. Unstarted, interrupted and failed attempts retain their original denominators.
- Source hashes, observation windows, output OFF and an empty queue are checked. File integrity and observation validity remain separate decisions.
- Storage, training and execution eligibility are separate. Successfully saved sources can still lack usable observations; valid training records still require separate model-support and execution checks.
- Prediction inputs contain only planned commands and pre-action images. Actual outputs and post-action sensors are execution evidence and labels. PWM history is a software record, not an electrical waveform measurement.

HTTP faults were instrumented across lookup, connection, headers and body. A 100ms first-byte wait was distinguished from body reassembly exceeding 0.8s, then waits and response size were adjusted. Intermittent latency remains rather than being concealed by retries.

Recent work addresses changes in image scale and placement between capture restarts. Optional arm-reference registration uses **only the first frame of that capture** and leaves an unstarted attempt before any motor command if quality is insufficient. Retrospective comparison reconnects actual HTTP RUN/OFF boundaries, complete output history, source frames and sensors. Nearest-time matching does not establish precise exposure/sensor synchronization.

Implementation: [command journal and transport](learning/supervised_transport.py) · [collection state management](learning/manual_collection.py) · [dataset audit](learning/manual_collection_dataset.py) · [Per-capture registration and retrospective audit conditions (Korean)](docs/arm-calibration.md#촬영별-첫-프레임-기준-등록) · [HTTP diagnostic (Korean)](docs/evidence/2026-11-04-http-response-gap.md)

## 2. Judging image inputs against baselines

### Freezing the model before comparing 18 new records

M4 downward motion used six conditions: 20/25/30% output × 350/500ms requested duration. Models were **trained on 18 records**, three per condition, then frozen. After repositioning, **18 new records were evaluated without retraining**. Image features were an 8×4 grayscale reduction of the full frame; labels were sensor-angle changes.

| Prediction method on the new 18 records | MAE |
| --- | ---: |
| No change | 2.154° |
| Overall training mean | 0.624° |
| Per-command training mean | **0.367°** |
| Command-only ridge | 0.595° |
| Image-and-command ridge | 0.605° |

The image model beat no change but **lost to the per-command mean**. This was not presented as a performance gain from visual learning. The result motivated explicit relative-state observations of the object, jaws and arm markers instead of coarse whole-frame features from a fixed view. Each new representation still requires its own model, baselines and new evaluation data.

The evaluation batch had 18 attempts, 18 completions and 18 eligible records, with 1,935 frames. Four earlier batches stopped before motion and remain preserved. This is same-day, single-device retrospective comparison across sequential poses, not cross-day/load generalization or a control success rate.

[Full conditions, denominators and baselines (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) · [Training implementation](learning/manual_collection_learning.py) · [Relative-state observation and learning design (Korean)](docs/relative-action-selection.md)

### Keeping acceptance criteria after switching to relative state

Instead of whole-frame features, the gripper view supplies **object-center minus visible jaw-midpoint XY and visible jaw-tip separation**. Registered finite jobs connect collection, automatic observation, training, model freeze and new evaluation without per-sample manual coordinates. Human pose setup and collection-positioning actions remain separate from model selection; observation or communication failures stop the job. Unattended whole-arm operation remains unvalidated.

For M3 up/down at 60% for 350ms, **18 records trained the model**. Of **12 new post-freeze evaluations**, **11 common supported records** had mean vector error **1.074px versus 1.341px for command mean**. Only two downward records met support, below the required four, so the evaluation did not pass. Conditional supplementation kept the model fixed: **15/16 completed**, with 14 common supported records yielding **1.190px versus 1.438px**. Completeness still failed. The supplement first established supported starting states, so its population differs from the original evaluation.

Earlier 35%/650ms and 60%/450ms models yielded **1.375px versus 1.809px** on 11 common records and **2.266px versus 1.887px** on ten, respectively. Data and models remain separate by command condition. At the final state, predicted goal improvement stayed below the 4px minimum, selecting **STOP**. That prediction-evaluation cohort executed zero model-selected approach, close, grasp or lift actions; pixel prediction gains do not establish grasping success.

In a separate new-view experiment, V2 was trained on five development transitions plus one valid manual UP return and frozen before the trial. It compared UP with stop and selected **one physical command**, changing `[1,21,124] → [-0.5,18,124]` and reaching the coordinate-wise tolerance boundary. A failed manual starting-position command and an earlier trial that began inside the goal with zero commands remain separate. The manual return is excluded from model-selected results; repeated correction, repeatability, grasping and lifting remain unvalidated. The earlier models and the 15/16 incomplete result are unchanged. [New approach conditions and denominators (Korean)](docs/evidence/2026-11-13-bounded-m3-approach.md).

[M3 conditions, frozen evaluations and failure denominators (Korean)](docs/evidence/2026-11-12-m3-goal-prediction.md) · [Development observation contract and usage (Korean)](docs/development-gripper-observer.md)

[M2/M3 W&B evaluation report (Korean; no login required)](https://forge.coreweave.com/wandb/henrypark-personal-project/motionbrain-fixed-evaluation/reports/-M2-M3---VmlldzoxODI5OTAzOQ?accessToken=5oatt93qha9x95onnncj8tbsdgk6362irr67javlo0nymassqaaal1mj8rjcl1sz) presents baseline comparisons, failures and evaluation denominators for four models, separating the original 350ms evaluation from its conditional supplement. These are prediction evaluations; that prediction-evaluation cohort executed zero model-selected grasp/lift actions.

## 3. Validating the next action separately from prediction quality

### What a single goal approach connected

A separate M2 experiment split 32 image-projected relative-orientation transitions into 16 training, eight validation and eight test records. Test MAE was **0.665°**, versus **6.797°** for no change. In one physical action selected by the frozen model's candidate comparison, image-projected goal error fell from **5.479° to 0.967°**.

Of four attempts that started capture, the preceding three **stopped before motion** on tracking, selection or initial-registration conditions. This demonstrates a connection from observation through prediction, candidate selection and execution to re-observation. Small logical groups from the same day/view and a single action do not establish physical joint-angle accuracy or repeatable grasping success.

[Prediction evaluation (Korean)](docs/evidence/2026-10-31-first-action-prediction.md) · [Goal approach and aborted-attempt denominator (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [Selector hold/abstain contract (Korean)](docs/offline-action-selection.md)

### Separating measured quantities and execution outcomes exposed the close-to-lift bottleneck

The current connector is **model-selected M1 close → fresh contact observation → model-selected M4 lift**. With other joints fixed, it separately checks the close result, output OFF, dual-camera contact evidence and M4 input conditions. Failed or uncertain results block the next action.

Ambiguous object/jaw-tip patches and inputs outside training support blocked further actions. Execution evidence was separated from observation contracts so that sources could be processed again. A separate M1 contract measures shaft separation at a fixed image cross-section, with **a distinct one-dimensional transition format and model**. Missing object-relative XY values are not fabricated, and uncertain shaft measurements remain explicitly rejected. Image cross-section separation is distinct from physical jaw-tip position, object contact and grasp force.

In the separate November 9 experiment, V1 and the augmented V2 were each frozen before evaluating one new close. On that V2 evaluation case, ridge shaft-prediction error was **0.18px**, versus **0.03px** for the command mean. Comparing V1 and V2 on that same evaluation case also failed to establish model improvement. V2 explicitly incorporated the earlier V1 evaluation record into training; that record was not counted again as independent V2 evaluation. Support bounds are preserved, and insufficient observations, unstarted attempts and failures are neither deleted nor accepted by lowering thresholds.

On November 13, a model frozen from nine manual transitions (six OPEN, three CLOSE) under a new direct-row95 observation contract compared CLOSE with stop and selected **one physical close**. Shaft separation fell **126.677 → 113.767px**, with output OFF verified. It stopped at the one-command budget without reaching the manually registered 85.3px reference goal. Absolute prediction error was **1.265px for ridge versus 0.989px for command mean**; the mean was more accurate in this case. The contribution of state input, repeated correction, grasping and lifting remain unvalidated. Nine manual commands and four earlier manual commands (including two observation rejections) are preserved: **13 manual + one model-selected = 14 commands** across these two cohorts. [Conditions, denominators and limits of the new model close (Korean)](docs/evidence/2026-11-13-model-selected-m1-close.md).

A separate **manual contact test** left the object on the table and failed to retain and lift it. The final M4 probe exceeded its stopping criterion and remained failed, with output OFF verified. The contributions of alignment, contact, friction and force were not isolated. These tests are excluded from model-selected execution results. Shaft-transition prediction and physical contact/retention require separate evidence.

[Shaft-model evaluations and manual contact-test conditions and denominators (Korean)](docs/evidence/2026-11-09-shaft-evaluation-and-contact-probe.md) · [Earlier patch-contract training, preparation and observation-abort records (Korean)](docs/evidence/2026-11-08-close-lift-integration.md) · [Repeatable collection, observation and evaluation structure (Korean)](docs/experiment-pipeline.md)

M4 lifting likewise separates observations from execution verdicts.

| Separate M4 experiment | Observed result and remaining condition |
| --- | --- |
| 500ms-candidate model from 2026-11-07 | Manually pregrasped object and fresh reference view; four training records and two frozen evaluations. Rise-prediction MAE was 2px, tied with the 2px command mean. Nine commands including three returns ended normally, but this model has selected zero physical executions |
| Earlier model with a 650ms candidate | Selected up at 50% for 650ms from two candidates. Two cameras observed one lift, but 3.08° of movement exceeded the 3° guard and execution remained failed |

Lifting a manually pregrasped object is not a successful automatic align/close/lift sequence. Visual jaw landmarks and object centers are also not ground truth for depth, contact force or collision clearance.

[Close-to-lift records and observation failures (Korean)](docs/evidence/2026-11-08-close-lift-integration.md) · [M4 evaluations and failures by model (Korean)](docs/m4-up-commissioning.md) · [Connector core](learning/close_lift_trial.py) · [Registered observations](learning/registered_gripper_landmarks.py)

## 4. Comparing the same models and evidence across views

### A plausible virtual arm is different from a calibrated robot

If FK, URDF and simulator dimensions change independently, viewers can agree while disagreeing with the physical arm. A [shared geometry source](ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/arm_geometry.json) records values, measurement methods, uncertainty and evidence hashes. FK defaults are checked against five whole-arm/M4 URDFs. Generating candidate models from a profile does not automatically alter control settings.

Manual measurements, assembly drawings and photographs informed the geometry, while joint-center dimensions remained distinct from estimated appearance. The physical fingertip moves with gripper opening and is not treated as the fixed virtual `tool0`. Comparing dimensions, sensors and photographs at one stationary pose left an **approximately 3° residual**, so the candidate was not adopted as a control correction. Viewer consistency remains distinct from physical measurement accuracy; the [measurement, approximation and calibration-candidate conditions (Korean)](docs/arm-calibration.md) retain the detailed evidence.

### Giving each tool a specific question

| Question | Connected tool | Interpretation boundary |
| --- | --- | --- |
| Are joint frames and geometry consistent? | Virtual joints and the shared URDF in RViz | Slider poses are not the physical arm's measured current state |
| Does the same geometry display in different engines? | Derived Gazebo/MuJoCo models | Mass, inertia and actuators are uncalibrated placeholders; no arm-link collision, jaw-contact or grasp model |
| What was predicted, and what was observed? | Saved dual-camera images, prospective predictions and measurements in Rerun | Replay order is not a synchronized real-time camera/controller clock |
| Does the model improve on baselines? | Offline W&B metrics and selective publication of allowed metrics | Published frozen-model/baseline comparisons include failures and evaluation denominators, separate from physical grasp/lift success; raw images and device information are excluded |

MuJoCo, Gazebo and RViz statically reflect the last measured M4 pose while distinguishing assumptions about other joints. Rerun distinguishes saved replay from following collected files; file following makes no additional camera requests and does not imply real-time joint tracking.

The connection supports **frames → observations → predictions → measurements → baselines** using shared evidence. Preparation verifies source/model hashes and observation contracts, then recomputes denominators and errors. Showing the same records in several viewers does not create new evaluation samples.

FK/IK/URDF position and orientation, plus `tool0` after MuJoCo conversion, are compared in software. GUI display and actual Rerun/W&B SDK records are checked separately. These checks do not establish a real-time digital twin, calibrated dynamics, simulation-trained policies or physical grasping performance.

[Geometry, photographic and sensor-comparison conditions (Korean)](docs/arm-calibration.md) · [Viewer setup and validation scope (Korean)](docs/robot-visualization.md) · [Evaluation replay and metric conversion](learning/robot_visualization.py)

## Remaining validation and next decisions

The remaining question is whether observation and model selection can connect shaft prediction to physical jaw-tip/object alignment, contact and retention. Independent frozen evaluation remains distinct from model-selected close/lift performance. Necessary approach joints and poses will be expanded while repeated results retain aborts, failures and human intervention.

Current learning is supervised transition prediction and candidate comparison. Whole-arm inverse-kinematics/dynamics control, reinforcement-learning policies, whole-arm position/force feedback and cross-environment generalization remain unverified. Kinematics will support reachability and clearance decisions after multi-pose calibration; dynamics and contact models require separate identification and validation.

Software regressions and firmware builds run in [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml); ROS2 builds/tests run in [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). Software validation and physical results tied to dates, mounts and loads do not substitute for each other.
