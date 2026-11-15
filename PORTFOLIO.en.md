# MotionBrain — Connecting Predictions to Physical Actions

[한국어](PORTFOLIO.md) · [Project overview](README.en.md) · [Design and experiment evidence (Korean)](docs/README.md)

**A personal project that uses learned models to compare and select actions from current images, then executes, re-observes and corrects toward approach → grasp → lift and hold → lower → release.** It connects firmware, observation and collection, dataset audits, supervised learning and bounded execution on an ESP32-controlled five-axis DC arm.

The central evaluation asks **whether the arm can make physical progress, correct from post-action observations, and grasp, lift and retain an object within a bounded attempt**. Command counts, elapsed time, failure causes, preparation and stage coordination remain part of the record. Prediction error and command-mean/no-change baselines support decisions about observation and candidate selection.

On November 15, 2026, one prepared scene connected **three model-selected closes → lift → lower → release**, with physical outcomes checked in two cameras. The user prepared the L-shaped starting pose and supervised on site. A coordinating experiment agent defined stage order, goals and candidates, started stages and assessed images; the learned models selected commands within each stage. [Latest physical results and denominators (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md)

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
- Prediction inputs contain only planned commands, visual features and sensor readings available before execution. Actual outputs and post-action sensors are execution evidence and labels. PWM history is a software record, not an electrical waveform measurement.

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

### Connecting model-selected closure through release in a prepared scene

On November 15, stage models were frozen for the same object, external camera view and prepared L-shaped pose. M1 learned candidate responses from four current dual-camera records: two CLOSE and two OPEN. Fresh images drove repeated comparisons of CLOSE, OPEN and stop. **Three closes reduced visible fingertip separation from about 137 to 49.70px.** Image overlap between the object and visible finger components served as an observation condition; physical contact and grasping were assessed from subsequent physical outcomes.

Local response models used a **virtual image point** defined from colored arm markers, image-axis orientation and pre-action M4 measurements. The virtual point does not measure the physical fingertip or depth. The lift applied a model trained on three unloaded UP records to a held object as an explicit transfer hypothesis. Lowering learned the inverse response from those UP records, with **zero direct DOWN training records**. Release was bound to observed table support and compared OPEN with stop against a separate goal: the original opening gap.

| Scope | Result |
| --- | --- |
| Final prepared-scene chain | **Six completed commands: three CLOSE, one UP, one DOWN and one OPEN**, with output OFF verified |
| Physical outcome | Two cameras separately confirmed table clearance, **airborne retention in post-action observations 3.582s apart**, table return and release |
| Model actuations across the whole work | **Eight: seven completed, one failed**. The final six commands are separate from preceding attempts |
| Preparation and collection | **16 actuations whose direction, output and duration were directly specified by the coordinating agent: 14 completed, two failed**, plus two non-starts. The user supplied the initial pose and on-site supervision |

This connects observation, learning and candidate comparison to physical closure, lift, return and release. The coordinating experiment agent handled stage order, goals, candidate sets, stage starts and visual assessment; learned action-prediction models chose commands within each stage. One prepared scene does not establish integrated unattended autonomy, repeated success rates, state-input gains over baselines or load generalization. The cause of the arm/gripper-camera pose change after release also remains unresolved.

The implementation separated communication preparation, active-motion monitoring and post-action retention observation deadlines. New observation contracts were audited and models frozen, while non-starts and failures retained their original outcomes. Actual object motion in both cameras was checked separately from prediction error.

[November 15 conditions, complete denominators and observation evidence (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md) · [Prepared-scene model control W&B report (Korean; no sign-in required)](<https://wandb.ai/henrypark-personal-project/motionbrain-fixed-evaluation/reports/MotionBrain:-수동-준비-후-모델-선택-집기·들기·복귀·해제-(2026-11-15)--VmlldzoxODMxMTk2Nw==?accessToken=wao2ed11330m5jtxjxpz3ifgbxhzad1tf4eavyptcc1ehmvmgt4ix9t96d1ivqw3>) (includes an English summary) · [Virtual image-point observer](learning/marker_virtual_observer.py) · [Near-gripper M1 model](learning/m1_near_action_model.py) · [Experimental lift model](learning/experimental_lift_response.py) · [Inverse-response lowering model](learning/experimental_lower_release.py)

### Preserving earlier evaluations and failures under their original conditions

| Earlier experiment | Result and limits |
| --- | --- |
| **November 9 M1 shaft evaluation** | One new close with frozen V2: ridge error **0.18px**, command mean **0.03px**. V1 evaluation data had entered V2 training and were not counted again as independent evaluation |
| **November 13 first M1 model close** | **126.677 → 113.767px**, short of the 85.3px goal. Ridge **1.265px**, command mean **0.989px**. A separate two-command follow-up also failed at the 45-second deadline before its second after-observation |
| **November 13–14 realigned model close and manual M4 lift** | Model closure **79.185 → 67.908px** and on-site bilateral contact. Later manual lifting produced about **1.387s** of airborne retention, followed by observed manual return and release. The original release trial retains cleanup failure, uncertain output and no normal receipt, with zero formal manual-release completions |
| **November 7 M4 lift models** | The 500ms model used four training and two frozen-evaluation records: MAE **2px**, tied with command mean, and zero physical model-selected executions. A separate 650ms model-selected lift remained failed because **3.08°** exceeded its 3° criterion |

A separate manual contact-and-rise test that left the object on the table without retained lifting also remains failed. These historical cohorts and manual interventions are kept separate from the new November 15 models, observations and execution results.

[Shaft evaluation and manual contact tests (Korean)](docs/evidence/2026-11-09-shaft-evaluation-and-contact-probe.md) · [November 13–14 model closing, alignment and manual-lift denominators (Korean)](docs/evidence/2026-11-13-model-selected-m1-close.md) · [Earlier close-to-lift connection (Korean)](docs/evidence/2026-11-08-close-lift-integration.md) · [M4 evaluations and failures by model (Korean)](docs/m4-up-commissioning.md)

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

The next step is to integrate initial approach and stage transitions, then repeat trials under the same conditions to measure success rates, corrections and elapsed time. Failures, non-starts and directly specified preparation actions remain in the record. Load transfer during lifting and inverse-response lowering require separate physical validation before expanding start poses, views and objects. Frozen prediction evaluation remains distinct from actual grasping and retention.

Current learning is supervised transition prediction and candidate comparison. Whole-arm inverse-kinematics/dynamics control, reinforcement-learning policies, whole-arm position/force feedback and cross-environment generalization remain unverified. Kinematics will support reachability and clearance decisions after multi-pose calibration; dynamics and contact models require separate identification and validation.

Software regressions and firmware builds run in [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml); ROS2 builds/tests run in [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). Software validation and physical results tied to dates, mounts and loads do not substitute for each other.
