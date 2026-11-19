# MotionBrain

[한국어](README.md) · [Portfolio](PORTFOLIO.en.md) · [Physical trial video · 35.5 s](docs/assets/whole-arm-cycle.mp4) · [Build and operations](OPERATIONS.en.md) · [Documentation and experiments (Korean)](docs/README.md)

**MotionBrain is a personal physical AI project that predicts how a robot arm and its surroundings will change given the current scene and a planned command.** The goal is to compare candidate moves, execute one, then observe what actually moved before choosing again.

For the fifth hardware development trial, the operator set up the object and the starting arm pose. Without joint-button input, the arm approached, grasped and lifted the object, lowered it, opened the gripper and withdrew. The final view shows the object standing on the table with the empty arm clear of it. [35.5-second hardware video](docs/assets/whole-arm-cycle.mp4) · [Five development trials and their outcomes (Korean)](docs/whole-arm-cycle.md)

![Start pose, lift and post-release retreat from a tabletop trial](docs/assets/whole-arm-cycle.jpg)

## From one joint to a whole-arm task

Of the OWI-535’s five DC motors, only the shoulder (M4) has joint-angle feedback. The project began with STM32 tilt and button inputs for teleoperation and an ESP32 driving the motors. Measuring how far the shoulder coasted after power cutoff led to a controller that reads the angle again after settling. Across 230–245°, it reached 11/11 targets unloaded and 11/11 with a 23.10 g load. Mean absolute error between the target and the settled sensor reading was 0.132° and 0.155°, respectively. Through the Raspberry Pi’s ROS2 interface, the shoulder also settled at 249.96° for an operator-confirmed 250.00° target. [Early teleoperation video](docs/assets/demo/motionbrain-demo.mp4) · [Shoulder tests under load](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md) · [ROS2 hardware interface](ros2_ws/src/motionbrain_hardware_interface/README.en.md)

For joints without an angle sensor, such as the wrist, an external camera measured motion. The design draws on the approach in *Visual Foresight* to predict the outcome of each move, choose one, then observe again. On this arm, the model learns how far image features move in response to short motor commands. In a wrist (M2) test, one model-selected action reduced image-space angular error from **5.479° to 0.967°**. It was the only one of four capture attempts that reached actuation. On 18 new shoulder-lowering actions, prediction error was 0.605° for the image-and-command model and 0.367° for the command-mean baseline. [Physical M2 trial (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [M4 baseline comparison (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) · [Research behind the design (Korean)](docs/research/visual-action-learning.md)

Object manipulation added a second view from above the gripper to track the jaws relative to the object. In teleoperated demonstrations, each press-and-release gesture was aligned with both views and actual motor output. An **offline imitation learning** experiment used images at the time of each button press to predict the operator’s choice of joint and direction. The policy that drove the hardware in the video compared the current observations with demonstrated poses and drove the shoulder and gripper in sequence, using a prescribed stage order and visual stopping conditions. [Demonstration collection (Korean)](docs/manual-data-collection.md) · [Stage-by-stage model selection (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md) · [Whole-arm execution and evaluation (Korean)](docs/whole-arm-cycle.md)

![Physical control, visual observation, learning and evaluation in MotionBrain](docs/assets/overview.en.svg)

## Comparing the physical arm with its model

Measured link dimensions define a common arm model. Forward and inverse kinematics were checked against URDF coordinates. **RViz** shows the links and coordinate frames, while **Gazebo** and **MuJoCo** show poses built from the same geometry. In physical footage, tracking started from the first frame of each demonstration and used shoulder sensor readings and the visible forearm direction. The estimated position agreed more closely with the image at the elbow and wrist than at the jaw tip. [Arm model and physical image measurements (Korean)](docs/arm-calibration.md)

| RViz | Gazebo | MuJoCo |
| --- | --- | --- |
| [![Current OWI-535 link model in RViz](docs/assets/arm-rviz.png)](docs/assets/arm-rviz.png) | [![OWI-535 arm in the Gazebo scene](docs/assets/arm-gazebo.png)](docs/assets/arm-gazebo.png) | [![MuJoCo rendering of the OWI-535 model](docs/assets/arm-mujoco.png)](docs/assets/arm-mujoco.png) |

In **Rerun**, replaying the video alongside the measurements shows how closely a move matched its prediction. In the shoulder-lift example below, the predicted object rise was 17 px and the observed rise was 21 px. **W&B** records comparisons that inform later model choices. The report on the right shows a separate gripper experiment in which the jaw gap fell from 137.24 px to 49.70 px over three closes against a 35 px image target. [Observation and tool setup (Korean)](docs/robot-visualization.md) · [W&B report and experiment conditions (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md#모델-비교-보고서)

| Rerun | W&B |
| --- | --- |
| [![Recorded physical observations in Rerun](docs/assets/arm-rerun.png)](docs/assets/arm-rerun.png) | [![W&B chart of observed jaw gap against the image target after successive closes](docs/assets/arm-wandb.png)](docs/assets/arm-wandb.png) |

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

## Next experiments

The next whole-arm trials will score whether the object stands on the table and the empty arm clears it separately from how closely the final pose matches a demonstration. A vision-language model (VLM) identified the upright object after release in one recorded demonstration using both camera views. New demonstrations will test whether that judgment holds. A later vision-language-action (VLA) experiment will ask whether different instructions for the same scene lead to different actions. For sim-to-real work, the first comparison will be simulated and physical trajectories under the same command, followed by a test of whether a policy learned through reinforcement learning reaches its goal on hardware. [Evaluation design (Korean)](docs/visual-action-learning.md)

## License

Project code uses the [MIT License](LICENSE). Third-party STM32 HAL/CMSIS files under `firmware/stm32/MotionBrainSensor/Drivers/` retain their original licenses.
