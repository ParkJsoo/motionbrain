# MotionBrain

[한국어](README.md) · [Portfolio](PORTFOLIO.en.md) · [Physical trial video · 35.5 s](docs/assets/whole-arm-cycle.mp4) · [Build and operations](OPERATIONS.en.md) · [Documentation and experiments (Korean)](docs/README.md)

**MotionBrain is a personal physical AI project that aims to teach a real arm what its actions do and use those predictions to reach a goal.** Given an image of the arm and object, the model predicts the effects of candidate joint commands. The controller can then choose a move that brings the arm closer to its target, observe what actually moved, and adjust its next choice.

The platform is an OWI-535 arm with five DC motors. Only the shoulder has a joint-angle sensor. Command duration alone tells us little about where the wrist stopped or whether an object rose with the gripper. To measure those outcomes, I connected motor firmware, two cameras, recording and learning software, and a ROS2 hardware interface. From a single move toward a target to a multi-stage manipulation task, the question is **how much a prediction helps reduce physical goal error**.

![Physical control, visual observation, learning and evaluation in MotionBrain](docs/assets/overview.en.svg)

## From the arm to the host

| Component | Role |
| --- | --- |
| **STM32, ESP32 and motor drivers** | The default STM32 firmware sends remote-control commands over UART using IMU tilt and button inputs. The browser also provides joystick and joint-button controls. The ESP32 drives five motors through three TB6612FNGs, controlling direction, PWM and stop state. On the M4 shoulder it reads an AS5600 angle sensor to check motion and settling after power cutoff. [Wiring and sensors](PIN_MAP.en.md) |
| **External camera and ESP32-CAM** | The external view shows the arm and object moving together. The gripper-mounted camera shows the jaws relative to the object. Experiments include object and gripper detection in a fixed view. Joint-response predictors use features such as markers, relative coordinates and object height validated for the recording setup. [Perception experiments (Korean)](docs/evidence/2026-10-30-cv-development.md) |
| **Raspberry Pi and ROS2** | The Pi hosts perception, a dashboard and a ROS2 bridge. After operator confirmation of an M4 target proposed through `ros2_control`, the physical controller drives the shoulder to its sensor target. The other four joints have no direct angle feedback. [ROS2 hardware interface](ros2_ws/src/motionbrain_hardware_interface/README.en.md) |
| **Host and learning software** | The host records video alongside teleoperation input, command receipt and actual motor output. It builds datasets from recorded episodes and compares models with baselines on the same samples. [Collection and replay (Korean)](docs/manual-data-collection.md) · [Experiment pipeline (Korean)](docs/experiment-pipeline.md) |

## How an observation informs the next action

1. **Collect.** Record a bounded movement of a single joint or a human teleoperation episode using the whole arm. Group keepalive requests under the original press-and-release gesture, then align driver output with capture times from both cameras. Retain interruptions and attempts that stopped before actuation.
2. **Learn.** Use image features, sensor readings and the planned command available before motion as inputs. Use the observed change after motion as the target. Record the joint, pose and camera conditions, freeze the model, then compare it with command-mean and no-change baselines on new data.
3. **Select and correct.** Compare candidate outcomes with the target within the model's supported range. Execute one bounded action, check that output stopped, and observe the new scene before choosing again. In the demonstration-based whole-arm experiment, human demonstrations, policy selections, rule-driven phase transitions and actual outputs were recorded separately.

[Visual action learning design (Korean)](docs/visual-action-learning.md) · [Model selection and sequential execution (Korean)](docs/model-grasp-trial.md) · [Research and design rationale (Korean)](docs/research/visual-action-learning.md)

## Results on physical hardware

Each experiment measures a different quantity. Joint sensor angles, image-space errors and task completion have separate meanings.

| Question | Result and conditions |
| --- | --- |
| Does the shoulder stop at its target angle? | In M4 closed-loop tests over 230–245°, the shoulder reached **11/11** targets unloaded and **11/11** with a 23.10 g load. Mean absolute sensor error was **0.132° and 0.155°**, respectively. In a separate ROS2 hardware test it stopped at 249.96° for a 250.00° target. [Shoulder control](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md) · [Physical ROS2 target](docs/evidence/2026-07-13-m4-physical-ros2-control.en.md) |
| Do images improve motion prediction? | On 18 new M4 downward motions, error was **0.605°** for the image-and-command model and **0.367°** for the command mean. For M3, both methods supported 11 of 12 new cases, with **1.074 px** error for the relative-state model and **1.341 px** for the command mean. Too few downward M3 cases met the original evaluation criteria. [M4 evaluation (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) · [M3 evaluation (Korean)](docs/evidence/2026-11-12-m3-goal-prediction.md) |
| Does the selected action move the arm toward a goal? | One model-selected M2 action reduced image-space angular error from **5.479° to 0.967°**. Three of four capture attempts stopped before actuation. Later experiments measured M3 target approach and M1 closure under their own conditions. [M2 approach (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [M3 approach (Korean)](docs/evidence/2026-11-13-bounded-m3-approach.md) · [M1 closure (Korean)](docs/evidence/2026-11-13-model-selected-m1-close.md) |
| Do several actions move the arm and object as intended? | In a prepared scene, the models selected actions for each supplied stage goal, progressing through grasp, lift, placement and release. In a later whole-arm development trial (attempt 5), the arm approached the object and proceeded through release and retreat without joint-button input. It missed the demonstration lower bound for the retreat stop angle by **1.67°**, failing the criterion set for that trial. [Stage-by-stage experiment (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md) · [Five whole-arm development trials (Korean)](docs/whole-arm-cycle.md) |

![Start pose, lift and post-release retreat from a tabletop trial](docs/assets/whole-arm-cycle.jpg)

[Whole-arm policy on hardware · 35.5 s](docs/assets/whole-arm-cycle.mp4) · [Early wired teleoperation demo](docs/assets/demo/motionbrain-demo.mp4)

## Kinematics, simulation and evaluation tools

Measured dimensions define a common arm geometry for checking FK, IK and URDF coordinates. **RViz** displays links and coordinate frames, and **Gazebo and MuJoCo** display simulated poses using that geometry. In two physical demonstrations, registration used the first frame of each. On eight other frames, mean image-space error was **1.56 px at the elbow, 4.72 px at the wrist and 46.58 px at the jaw tip**. These are 2D projection errors. Physical dynamics, contact calibration and policy transfer from simulation remain open. [Kinematic measurements (Korean)](docs/arm-calibration.md)

| RViz | Gazebo | MuJoCo |
| --- | --- | --- |
| [![Current OWI-535 link model in RViz](docs/assets/arm-rviz.png)](docs/assets/arm-rviz.png) | [![OWI-535 arm in the Gazebo scene](docs/assets/arm-gazebo.png)](docs/assets/arm-gazebo.png) | [![MuJoCo rendering of the OWI-535 model](docs/assets/arm-mujoco.png)](docs/assets/arm-mujoco.png) |

**Rerun** replays both camera views alongside predictions and measurements. A separate read-only view displays the latest frames from both cameras and the M4 sensor reading. **W&B** compares frozen models with baselines, preserving failures and evaluation counts so that readers can see which conditions benefited from a model change. [Live preview (Korean)](docs/live-observation.md) · [Tool setup (Korean)](docs/robot-visualization.md) · [Experiment record with the public W&B report (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md#모델-비교-보고서)

| Rerun | W&B |
| --- | --- |
| [![Recorded physical observations in Rerun](docs/assets/arm-rerun.png)](docs/assets/arm-rerun.png) | [![W&B chart of observed jaw gap against the image target after successive closes](docs/assets/arm-wandb.png)](docs/assets/arm-wandb.png) |

In this Rerun replay, the predicted rise was 17 px and the observed rise was 21 px for a shoulder UP command (50%, 500 ms). The W&B report shows a separate gripper experiment. Across three CLOSE commands, the observed jaw gap fell from 137.24 px to 49.70 px against a 35 px image target.

## Getting started without hardware

Host tests use Python 3.12, NumPy, OpenCV, Node.js and a C++ compiler. Optional PyTorch tests may be skipped depending on installed dependencies. Firmware builds use PlatformIO.

Try the gripper release check with synthetic observations. The test checks whether the jaws have opened far enough, whether the object is at the table reference position, and whether both camera observations are fresh and follow confirmed motor output.

```sh
python3 -m unittest discover -s tests -p 'test_whole_arm_release.py'
```

To run the full host test suite and build both firmware targets, use the following commands.

```sh
python3 -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

The [operations guide](OPERATIONS.en.md) covers setup, services and physical execution conditions. Source checks and builds can be reproduced from the public repository. Recomputing the original experimental metrics requires private recordings, models and camera registration data. The physical whole-arm trial also used a local host runner. [PlatformIO CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml) · [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml)

## Next evaluations

Score task completion and similarity to a demonstrated pose separately. Repeat trials from the same starting conditions, then vary pose, object and camera view to compare goal attainment and correction counts. Further work includes evaluating visual state assessment and imitation-based action choice on independent episodes, and measuring simulated and physical trajectory differences. VLA and reinforcement learning policies and sim-to-real performance are future research comparisons.

## License

Project code uses the [MIT License](LICENSE). Third-party STM32 HAL/CMSIS files under `firmware/stm32/MotionBrainSensor/Drivers/` retain their original licenses.
