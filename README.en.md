# MotionBrain

[한국어](README.md) · [Portfolio](PORTFOLIO.en.md) · [Build and operations](OPERATIONS.en.md) · [Documentation and evidence (Korean)](docs/README.md)

**A personal robotics project that uses learned response models to compare and select actions from current images, then executes, re-observes and corrects.** The goal is to connect approach → grasp → lift and hold → lower → release, evaluating actual task completion and correction after failure. Candidate predictions use pre-action visual features, sensor readings and planned commands.

An ESP32-controlled five-axis DC arm connects an external camera, a gripper-mounted camera and sensors. The project combines a physical **collection → training → prediction → bounded execution** pipeline with **RViz, Gazebo, MuJoCo, Rerun and W&B** for inspecting robot models and evaluation results.

![MotionBrain physical records, prediction, bounded execution and five visualization tools](docs/assets/overview.en.svg)

*An overview of the architecture and data flow in the public source. It is not a physical photograph or a live integration screen.*

**On November 15, 2026, one prepared scene connected three model-selected CLOSE commands → one UP → one DOWN → one OPEN.** Two cameras confirmed table clearance, **airborne retention in post-action observations 3.582 seconds apart**, table return and release. The user prepared the L-shaped starting pose and supervised on site. A coordinating experiment agent defined stage order, goals and candidates, started stages and assessed images; the learned action-prediction models selected commands within each stage. This is one prepared-scene result; integrated unattended autonomy and repeated success rates remain unvalidated. [Physical results and full attempt denominators (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md)

[Prepared-scene model control W&B report (Korean; no sign-in required)](<https://wandb.ai/henrypark-personal-project/motionbrain-fixed-evaluation/reports/MotionBrain:-수동-준비-후-모델-선택-집기·들기·복귀·해제-(2026-11-15)--VmlldzoxODMxMTk2Nw==?accessToken=wao2ed11330m5jtxjxpz3ifgbxhzad1tf4eavyptcc1ehmvmgt4ix9t96d1ivqw3>) — Includes an English summary.

Registered single-joint jobs connect **collection → automatic observation → training → model freeze → evaluation on new actions** without per-sample manual coordinates. Observation rejections and communication failures retain their own outcomes.

A development path connects relative observations under bounded translation and isotropic scale changes to candidate comparison and the existing finite execution path. Approach/closing features remain distinct from external height measurements used for lifting. A supported prefix is reported separately from a full cycle. See [view adaptation and stage-specific models (Korean)](docs/visual-action-learning.md#구도가-달라진-영상에서-후보-비교로-이어지는-경로); software checks do not establish physical grasp success.

## From collection to bounded execution

| Stage | Implemented path | Decision criteria |
| --- | --- | --- |
| **1. Collect** | A local Mac UI and finite-job runners collect one joint at a time from M1–M5, recording and replaying images, commands, actual outputs and sensors | Distinguish requests from actual execution; preserve failures, interruptions and unstarted actions |
| **2. Train** | Audit raw hashes, observation windows and output history, then train change predictors on data separated by joint, pose and view | Separate completed storage from training eligibility; check training/evaluation overlap |
| **3. Predict** | Predict each candidate's final state or change from pre-action visual features, sensor state and planned commands | Compare with command-mean and no-change baselines; freeze the model before evaluating new data |
| **4. Execute within limits** | Check supported inputs, observation quality and stop state before executing an eligible candidate and observing again | Do not automatically resend failed or uncertain actions; hold the next step |

Prediction uses inputs available before execution. Actual outputs and post-action sensors are kept separate as labels, execution checks and retrospective evidence. Current learning is **supervised action-change prediction and candidate comparison**, rather than a reinforcement-learning policy. The first connection uses only the joints it needs, sequentially; approach poses and motion coverage come afterward.

[Collection and replay (Korean)](docs/manual-data-collection.md) · [Learning design (Korean)](docs/visual-action-learning.md) · [Model selection and sequential execution (Korean)](docs/model-grasp-trial.md)

The [repeatable experiment pipeline (Korean)](docs/experiment-pipeline.md) separates raw single-action collection from later observation and reruns source audits, context-specific training, baseline evaluation and model-version comparisons from an explicit manifest. Software and historical replay verification remain separate from physical operation in a new camera view.

## Inspecting models and experiments

- **RViz, Gazebo and MuJoCo:** Compare arm models derived from shared geometry and approximate static poses. The last measured M4 value can update the display pose; other joints remain assumed. Live pose following, calibrated dynamics and gripper contact models remain incomplete.
- **Rerun and W&B:** Compare source observations, pre-action predictions, measurements and baselines. Rerun distinguishes saved replay from following recorded collection files; only selected metrics useful for decisions are published to W&B.

[Setup, display and validation scope (Korean)](docs/robot-visualization.md) · [Experiment pipeline (Korean)](docs/experiment-pipeline.md)

## Representative results and current status

These experiments use different joints, models and observation conditions. MAE (mean absolute error) averages the magnitude of prediction errors; lower is better. Image-plane angles, sensor angles and pixel errors are not combined into one accuracy measure.

| Experiment | Observed result | Scope |
| --- | --- | --- |
| **Prepared-scene model-selected grasp, lift, lower and release (Nov 15)** | The final chain completed **six commands**: three CLOSE, one UP, one DOWN and one OPEN. Two cameras confirmed table clearance, retention observations **3.582s apart**, return and release | Across the whole work: **eight model actuations, seven completed and one failed**; **16 preparation/collection actuations directly specified by the coordinating agent, 14 completed and two failed**; two additional non-starts. One case with agent-managed stages and visual assessment; autonomous completion rate and repeatability remain unvalidated |
| **M2 model-selected goal approach** | One command reduced image-plane goal error from **5.479° to 0.967°** | Three of four attempts that started recording stopped before actuation. A single same-day, same-view result; repeated grasping remains unvalidated |
| **M4 downward prediction** | Six conditions, **18 training** records and **18 new post-freeze evaluations**. Per-command mean MAE **0.367°**; image-plus-command ridge **0.605°** | Adding this visual representation did not improve prediction |
| **Earlier M1 shaft prediction (Nov 9)** | On **one new close** with the model frozen then: error **0.18px**, command mean **0.03px** | No improvement over the previous model established. Image-plane shaft separation, not physical fingertip, contact or grasp performance |
| **M3 relative-state prediction near the goal** | **18 training** records; among **12 new post-freeze evaluations**, 11 common supported records had mean vector error **1.074px** for ridge and **1.341px** for command mean | Insufficient downward evaluations initially; conditional supplementation still completed only **15/16**, so no overall pass. Zero model-selected approach/grasp/lift actions in that prediction-evaluation cohort |
| **Physical approach with an M3 development model (Nov 13)** | Frozen V2 selected **one UP command**; a fresh image reached the coordinate-wise goal tolerances | One manual return was separate. A single result on the tolerance boundary; repeated correction, repeatability, grasping and lifting remain unvalidated |
| **Model-selected M1 close: initial cohort (Nov 13)** | A frozen model compared CLOSE with stop and selected **one physical command**: shaft separation **126.677 → 113.767px**, output OFF verified | Nine manual preparation/collection commands were separate. The 85.3px goal was not reached. Ridge error **1.265px**, command mean **0.989px**: the mean was more accurate. Repeated correction, grasping and lifting remain unvalidated |
| **Model-selected M1 closure and manual M4 lift after realignment (Nov 13–14)** | A new frozen model executed **one command**, separate from four manual commands: **79.185 → 67.908px**, reference-goal error **0.152px**, OFF verified. The user confirmed bilateral jaw contact on site. A later manual M4 rise of +2.28° produced observed short-term airborne retention | Two manual DOWN commands returned the object to the table; one later physical manual OPEN produced observed release. That trial retains its cleanup failure and uncertain-output result, separate from emergency OFF and two later observations. M4: seven attempts/five actuations (three UP, two DOWN); manual release: two separate attempts/zero formal completions. Fully AI-controlled execution, formal end-to-end success and repeatability remain unvalidated |
| **Manual contact-and-rise experiment (Nov 9)** | Closing and rising were observed, but the object stayed on the table: **no retained lift** | Separate from model-selection results. The final rise failed the observed-change criterion |

[November 15 prepared-scene physical sequence (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md) · [M2 goal approach (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [M4 frozen evaluation (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) · [Lift models, observations and failures (Korean)](docs/m4-up-commissioning.md) · [M1 evaluation and physical contact experiment (Korean)](docs/evidence/2026-11-09-shaft-evaluation-and-contact-probe.md)

[M1 close, observation-deadline failure and bilateral contact after realignment: conditions and denominators (Korean)](docs/evidence/2026-11-13-model-selected-m1-close.md)

A separate follow-up model reselected from a fresh observation and executed **two commands with OFF verified**, but failed at the 45-second deadline before the second after-observation. Across those earlier two trials, three model commands were confirmed; goal completion, grasping and lifting were not established.

[New M3 physical approach: conditions, manual preparation and failure denominators (Korean)](docs/evidence/2026-11-13-bounded-m3-approach.md)

[M3 automatic observation, frozen evaluation and incomplete denominators (Korean)](docs/evidence/2026-11-12-m3-goal-prediction.md)

[M2/M3 W&B prediction evaluation report (Korean; no login required)](https://forge.coreweave.com/wandb/henrypark-personal-project/motionbrain-fixed-evaluation/reports/-M2-M3---VmlldzoxODI5OTAzOQ?accessToken=5oatt93qha9x95onnncj8tbsdgk6362irr67javlo0nymassqaaal1mj8rjcl1sz) records model/baseline comparisons, failures and evaluation denominators. That prediction-evaluation cohort executed zero model-selected grasp/lift actions; prediction evaluation remains separate from physical grasp/lift success.

## Physical system

ESP32 manages commands, outputs and stop state for five DC axes; the host handles collection, validation, training and evaluation. The system also connects STM32 sensing/teleoperation and Raspberry Pi perception, dashboard and ROS2 paths.

- **Observation:** An external camera and an ESP32-CAM above the gripper. Current feature tracking is limited to registered views and reference points; general object recognition, depth and contact detection remain unvalidated.
- **Feedback:** Only the M4 shoulder has an AS5600 angle sensor. The other four axes lack position feedback.
- **Physical ROS2 path:** Operator-confirmed single M4 targets have been validated. Whole-arm trajectory tracking remains incomplete.

[Hardware wiring](PIN_MAP.en.md) · [ROS2 hardware interface](ros2_ws/src/motionbrain_hardware_interface/README.en.md)

## Getting started

Start with software checks and firmware builds without connecting a device. Host checks use Python 3.12, `numpy`, `opencv-python-headless`, Node.js (CI: 22) and a C++ compiler. Optional PyTorch checks are skipped without their dependencies; firmware builds require PlatformIO.

```sh
python3 -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

These commands test and build; they do not upload firmware or drive motors. Raw captures and model weights are not in the public repository, so reevaluating experiment scores requires those data.

- **Collection and learning:** [Manual collection guide (Korean)](docs/manual-data-collection.md) for the UI, replay, datasets and training
- **Model and evaluation viewers:** [Visualization guide (Korean)](docs/robot-visualization.md) for virtual-arm display and local evaluation replay
- **Installation, services and physical execution:** [Operations](OPERATIONS.en.md) for execution conditions and stop verification
- **Automated checks:** [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml) · [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml)

Physical execution requires checking current connections, cameras and movement clearance. Intermittent HTTP latency remains, and the collection UI's abort request does not guarantee an immediate physical emergency stop. Follow the collection and operations guides for details. Passing software checks and validating physical performance are separate outcomes.

## Remaining validation

The prepared-scene result is a starting point for integrating initial approach and stage transitions, then repeating trials with all failures retained. The November 15 lift transferred an unloaded response model to a held object; lowering used an inverse UP-response model with zero direct DOWN training records. Load, view and start-pose support, plus gains over simple baselines, require separate validation. Pixel gaps and virtual image points remain distinct from physical contact, gripping force and observed airborne retention. The cause of the arm/gripper-camera pose change after release remains unresolved.

Shared geometry supports software comparisons of FK, IK and URDF, but precision physical joint-zero, camera-extrinsic and dynamics calibration remain incomplete. The current approximate model does not establish physical collision clearance.

[Roles of observation, prediction and kinematics (Korean)](docs/visual-action-learning.md#전체-팔에서-기구학동역학을-쓰는-방법) · [Calibration evidence and limitations (Korean)](docs/arm-calibration.md)

## Code and documentation

| Path | Role |
| --- | --- |
| `learning/`, `tools/` | Collection/replay, source validation, visual measurement, prediction/evaluation, candidate selection and visualization |
| `src/`, `firmware/` | ESP32 motor control, camera and STM32 firmware |
| `ros2_ws/src/` | Messages, bridges, control guards, URDF and hardware interface |
| [Portfolio](PORTFOLIO.en.md) | Problems addressed, design decisions and representative results |
| [Documentation and evidence index (Korean)](docs/README.md) | Guides and dated measurement conditions, failures and denominators |

## Existing teleoperation demo

![2026-06-08 wired teleoperation demonstration](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)

[Watch MP4](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4) · **Teleoperation at the fixed 2026-06-08 tag.** This is not an autonomous grasping demonstration by the current learned model.

## License

Project code is [MIT](LICENSE). STM32 HAL/CMSIS files retain their upstream licenses under `firmware/stm32/MotionBrainSensor/Drivers/`.
