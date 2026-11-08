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

## 3. Validating the next action separately from prediction quality

### What a single goal approach connected

A separate M2 experiment split 32 image-projected relative-orientation transitions into 16 training, eight validation and eight test records. Test MAE was **0.665°**, versus **6.797°** for no change. In one physical action selected by the frozen model's candidate comparison, image-projected goal error fell from **5.479° to 0.967°**.

Of four attempts that started capture, the preceding three **stopped before motion** on tracking, selection or initial-registration conditions. This demonstrates a connection from observation through prediction, candidate selection and execution to re-observation. Small logical groups from the same day/view and a single action do not establish physical joint-angle accuracy or repeatable grasping success.

[Prediction evaluation (Korean)](docs/evidence/2026-10-31-first-action-prediction.md) · [Goal approach and aborted-attempt denominator (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [Selector hold/abstain contract (Korean)](docs/offline-action-selection.md)

### Observation and model support blocked close-to-lift integration

The current connector is **model-selected M1 close → fresh contact observation → model-selected M4 lift**. With other joints fixed, it separately checks the close result, output OFF, dual-camera contact evidence and M4 input conditions. Failed or uncertain results block the next action.

When the current pose fell outside the earlier M1 model's support, a separate observation contract used fixed-view object arcs and jaw patches. A new model was frozen on **four records: two open and two close**. An original after-observation failure remained a failure while its sources were re-audited under the new contract. Three subsequent preparation opens still left ambiguous correspondences and unsupported inputs. A high best-match score was insufficient if it could not be distinguished from a competing match.

This batch therefore contains **seven physical commands: four for training and three for preparation; zero post-freeze evaluations, zero model-selected closes and zero connected lifts**. Observation thresholds were not lowered to force acceptance, and rejected approximations did not expand model support. The lesson is to establish **reliable observation of the current pose and its relation to the training range** before increasing execution count.

M4 lifting likewise separates observations from execution verdicts.

| Separate M4 experiment | Observed result and remaining condition |
| --- | --- |
| New model with a 500ms candidate | Manually pregrasped object and fresh reference view; four training records and two frozen evaluations. Rise-prediction MAE was 2px, tied with the 2px command mean. Nine commands including three returns ended normally, but this model has selected zero physical executions |
| Earlier model with a 650ms candidate | Selected up at 50% for 650ms from two candidates. Two cameras observed one lift, but 3.08° of movement exceeded the 3° guard and execution remained failed |

Lifting a manually pregrasped object is not a successful automatic align/close/lift sequence. Visual jaw landmarks and object centers are also not ground truth for depth, contact force or collision clearance.

[Close-to-lift records and observation failures (Korean)](docs/evidence/2026-11-08-close-lift-integration.md) · [M4 evaluations and failures by model (Korean)](docs/m4-up-commissioning.md) · [Connector core](learning/close_lift_trial.py) · [Registered observations](learning/registered_gripper_landmarks.py)

## 4. Comparing the same models and evidence across views

### A plausible virtual arm is different from a calibrated robot

If FK, URDF and simulator dimensions change independently, viewers can agree while disagreeing with the physical arm. A [shared geometry source](ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/arm_geometry.json) records values, measurement methods, uncertainty and evidence hashes. FK defaults are checked against five whole-arm/M4 URDFs. Generating candidate models from a profile does not automatically alter control settings.

Manual readings supplied a 70mm shoulder height, 90mm shoulder-to-elbow length and approximately 110mm elbow-to-wrist length. The 110mm wrist-to-gripper-tip reading at the measured opening is approximated as a fixed virtual `tool0` length and divided internally into 45+65mm. The intermediate 45mm is not added again; the moving physical fingertip and fixed `tool0` remain distinct. Appearance was constructed from assembly drawings and photographs. Joint-center dimensions, estimated case geometry and uncalibrated camera frames remain distinct.

Stationary height and sensor readings were also compared. An elbow-axis height of **145mm** and existing dimensions imply a shoulder elevation of **approximately 56.44°**, paired with a **192.83°** sensor report from the same stationary interval. The photograph's projected angle was **approximately 59.44°**, leaving about a 3° residual. A coarse single-pose reading was not used to erase that residual or reapply the sensor mounting offset. The candidate is **read-only, `enabled=false`, `controlEligible=false`**, and has not been applied to control.

### Giving each tool a specific question

| Question | Connected tool | Interpretation boundary |
| --- | --- | --- |
| Are joint frames and geometry consistent? | Virtual joints and the shared URDF in RViz | Slider poses are not the physical arm's measured current state |
| Does the same geometry display in different engines? | Derived Gazebo/MuJoCo models | Mass, inertia and actuators are uncalibrated placeholders; no arm-link collision, jaw-contact or grasp model |
| What was predicted, and what was observed? | Saved dual-camera images, prospective predictions and measurements in Rerun | Replay order is not a synchronized real-time camera/controller clock |
| Does the model improve on baselines? | Offline W&B metrics and selective publication of allowed metrics | Comparison uses two actual frozen evaluations; raw images and device information are excluded from publication |

The connection supports **frames → observations → predictions → measurements → baselines** using shared evidence. Preparation verifies source/model hashes and observation contracts, then recomputes denominators and errors. Showing the same records in several viewers does not create new evaluation samples.

FK/IK/URDF position and orientation, plus `tool0` after MuJoCo conversion, are compared in software. GUI display and actual Rerun/W&B SDK records are checked separately. These checks do not establish a real-time digital twin, calibrated dynamics, simulation-trained policies or physical grasping performance.

[Geometry, photographic and sensor-comparison conditions (Korean)](docs/arm-calibration.md) · [Viewer setup and validation scope (Korean)](docs/robot-visualization.md) · [Evaluation replay and metric conversion](learning/robot_visualization.py)

## Remaining validation and next decisions

The next physical task is reliable observation and supported M1 inputs at the current pose, followed by separate validation of **new frozen evaluation → model-selected close → fresh contact/support confirmation → model-selected M4 lift**. Necessary approach joints and poses will be expanded sequentially, with repeated results retaining aborts, failures and human intervention.

Current learning is supervised transition prediction and candidate comparison. Whole-arm inverse-kinematics/dynamics control, reinforcement-learning policies, whole-arm position/force feedback and cross-environment generalization remain unverified. Kinematics will support reachability and clearance decisions after multi-pose calibration; dynamics and contact models require separate identification and validation.

Software regressions and firmware builds run in [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml); ROS2 builds/tests run in [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). Software validation and physical results tied to dates, mounts and loads do not substitute for each other.
