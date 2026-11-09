# MotionBrain

[한국어](README.md) · [Portfolio](PORTFOLIO.en.md) · [Build and operations](OPERATIONS.en.md) · [Documentation and evidence (Korean)](docs/README.md)

**A personal robotics project that records physical arm motion and predicts the next change from images and planned commands.** Its goal is to compare candidate outcomes to approach, grasp and lift an object.

An ESP32-controlled five-axis DC arm connects an external camera, a gripper-mounted camera and sensors. The project combines a physical **collection → training → prediction → bounded execution** pipeline with **RViz, Gazebo, MuJoCo, Rerun and W&B** for inspecting robot models and evaluation results.

![MotionBrain physical records, prediction, bounded execution and five visualization tools](docs/assets/overview.en.svg)

*An overview of the architecture and data flow in the public source. It is not a physical photograph or a live integration screen.*

The current stage connects joint-specific prediction into **M1 gripper closing → M4 shoulder lifting**. **Continuous autonomous whole-arm grasping and lifting remain incomplete.** Post-freeze evaluation and connected execution of the current M1 integration model are both still at zero.

## From collection to bounded execution

| Stage | Implemented path | Decision criteria |
| --- | --- | --- |
| **1. Collect** | A local Mac UI runs a finite pulse on one of M1–M5, recording and replaying images, commands, actual outputs and sensors | Distinguish requests from actual execution; preserve failures, interruptions and unstarted actions |
| **2. Train** | Audit raw hashes, observation windows and output history, then train change predictors on data separated by joint, pose and view | Separate completed storage from training eligibility; check training/evaluation overlap |
| **3. Predict** | Predict each candidate's final state or change from pre-action images and planned commands | Compare with command-mean and no-change baselines; freeze the model before evaluating new data |
| **4. Execute within limits** | Check supported inputs, observation quality and stop state before executing an eligible candidate and observing again | Do not automatically resend failed or uncertain actions; hold the next step |

Prediction uses inputs available before execution. Actual outputs and post-action sensors are kept separate as labels, execution checks and retrospective evidence. Current learning is **supervised action-change prediction and candidate comparison**, rather than a reinforcement-learning policy. The first connection uses only the joints it needs, sequentially; approach poses and motion coverage come afterward.

[Collection and replay (Korean)](docs/manual-data-collection.md) · [Learning design (Korean)](docs/visual-action-learning.md) · [Model selection and sequential execution (Korean)](docs/model-grasp-trial.md)

The [repeatable experiment pipeline (Korean)](docs/experiment-pipeline.md) separates raw single-action collection from later observation and reruns source audits, context-specific training, baseline evaluation and model-version comparisons from an explicit manifest. Software and historical replay verification remain separate from physical operation in a new camera view.

## Five tools for inspecting models and experiments

The approximate virtual arm and actual experiment records provide complementary views. **RViz, Gazebo and MuJoCo cover robot models; Rerun and W&B cover observations, predictions and evaluation.**

| Tool | Connected content | What to inspect |
| --- | --- | --- |
| **RViz** | URDF appearance, joints, frames and virtual joint sliders | Link placement and joint changes |
| **Gazebo** | A virtual arm converted from the shared URDF, with display-pose targets | Appearance and pose in a virtual scene |
| **MuJoCo** | A conversion of the same arm with virtual position actuators | Model structure and virtual joint manipulation |
| **Rerun** | External/gripper-camera recordings from saved frozen evaluation, with pre-action predictions and measurements | Before/after changes and prediction error per case |
| **W&B** | Offline model/baseline metrics and selective web publishing of allowed metrics | Training/evaluation counts and method/case comparisons |

The virtual arm is an **approximation of appearance and static pose** informed by OWI-535 photos, assembly drawings and manual dimension readings. It has no live physical-pose connection, calibrated dynamics, arm-link collision or gripper-contact model; virtual training is deferred. Rerun replays saved evaluations, and W&B does not automatically publish every run.

[Setup, viewers and validation scope (Korean)](docs/robot-visualization.md) · [Shared geometry and physical calibration (Korean)](docs/arm-calibration.md)

## Representative results and current status

These experiments use different joints, models and observation conditions. MAE (mean absolute error) averages the magnitude of prediction errors; lower is better. Image-plane angles, sensor angles and pixel errors are not combined into one accuracy measure.

| Experiment | Observed result | Scope |
| --- | --- | --- |
| **M2 model-selected goal approach** | One command reduced image-plane goal error from **5.479° to 0.967°** | Three of four attempts that started recording stopped before actuation. A single same-day, same-view result; repeated grasping remains unvalidated |
| **M4 downward prediction** | Six conditions, **18 training** records and **18 new post-freeze evaluations**. Per-command mean MAE **0.367°**; image-plus-command ridge **0.605°** | Adding this visual representation did not improve prediction |
| **M4 lift prediction** | With a manually pregrasped object: **4 training** records and **2 post-freeze evaluations** for a 500ms-candidate model. Rise MAE **2px**, tied with command mean | **Zero** physical selections by this model. An earlier, separate 650ms model produced one observed lift, but execution failed the observational guard at **3.08° > 3°** |
| **M1 close → M4 lift integration** | New M1 model frozen on **4 records**. **7 physical commands**: four training and three preparation | Stopped for ambiguous observation and unsupported model inputs. Post-freeze evaluations, model-selected closes and connected executions: **zero each** |

[M2 goal approach (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [M4 frozen evaluation (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) · [Lift models, observations and failures (Korean)](docs/m4-up-commissioning.md) · [Close-to-lift attempt and stop (Korean)](docs/evidence/2026-11-08-close-lift-integration.md)

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

## Calibration and next steps

The immediate task is to establish reliable observation and supported M1 inputs at the current pose, then evaluate the frozen model and connect **model-selected closing → fresh contact review → M4 lifting**. Extend the necessary approach joints and poses afterward, evaluating repeated successes and failures.

Kinematic display uses positive upward pitch and a fixed `tool0` independent of jaw opening. Software checks compare FK, IK and URDF positions and orientations against shared geometry; precision physical geometry, joint-zero and camera-extrinsic calibration remain incomplete. The current approximate model is not used to establish collision clearance or permission for physical commands.

An optional implemented path registers the first frame of each recording before motor commands and retrospectively audits raw records and output history. Static sensor/height observations also produced a coarse M4 reference candidate pairing a sensor reading of **192.83°** with an elevation of **approximately 56.44°**. The same photo gives approximately **59.44°**, leaving an approximately **3°** residual, and the candidate is disabled. This does not establish precision zero calibration or apply control settings.

[Coordinate conventions and the role of kinematics (Korean)](docs/visual-action-learning.md#전체-팔에서-기구학동역학을-쓰는-방법) · [Calibration procedure and remaining work (Korean)](docs/arm-calibration.md)

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
