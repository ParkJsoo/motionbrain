# MotionBrain | Connecting visual action selection to a physical robot arm

[한국어](PORTFOLIO.md) · [Project overview and setup](README.en.md) · [Design and experiment index (Korean)](docs/README.md)

**A personal physical AI project exploring how a robot can anticipate the effects of its next action, observe the actual movement and approach a goal.** Starting with teleoperation of a five-axis DC robot arm, I connected images, commands and outputs into training data, then extended action selection to physical grasping, lifting and release. I implemented the STM32 input device, ESP32 controller, Raspberry Pi ROS2 interface and software for two-camera collection, learning and evaluation.

[![The starting pose, the object lifted and the arm withdrawn after release](docs/assets/whole-arm-cycle.jpg)](docs/assets/whole-arm-cycle.mp4)

This 35.5-second video records a development trial with fixed camera views and an operator-prepared object and starting pose. From there, the system used both cameras and the shoulder angle to drive **approach → grasp → lift → lower → release → empty-arm retreat** sequentially without joint-button input. The final view shows the object standing on the table and the arm withdrawn. [Physical trial video](docs/assets/whole-arm-cycle.mp4) · [Execution method and trial conditions (Korean)](docs/whole-arm-cycle.md)

The project connects three areas:

- **Embedded control and ROS2:** sensor feedback, correction after settling and command lifecycle management to handle physical output and stopping.
- **Robot learning data and action policies:** reconstruction of actions from teleoperated demonstrations, with separate implementations for offline imitation learning, action-effect prediction and the physical policy.
- **Physical and virtual models with evaluation:** a shared arm model inspected in RViz, Gazebo and MuJoCo, with observations, predictions and model comparisons analyzed in Rerun and W&B.

## Closing the loop from sensor feedback to physical stopping

The shoulder (M4) is the only one of the OWI-535's five DC axes with direct joint-angle feedback. I measured coasting after output cutoff and implemented direction-dependent early cutoff, followed by an AS5600 reading and correction after settling. Across 230–245°, the controller reached its targets in 11/11 trials unloaded and 11/11 with a 23.10 g load. Mean absolute errors between the targets and settled sensor readings were **0.132° and 0.155°**, respectively. Through the Raspberry Pi's physical `ros2_control` path, an operator-confirmed 250.00° target also brought the shoulder to rest at 249.96°. [Controller measurements](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md) · [Physical ROS2 interface](docs/evidence/2026-07-13-m4-physical-ros2-control.en.md)

For teleoperation, I added lifecycle management that treats start, keep-alive and stop requests as one gesture. The controller rejects renewals for closed gestures so a late request after button release cannot restart the motor. I verified that request sequence in a native regression test. This control path became the basis for connecting demonstrator input to actual motor activity. [Regression test](tests/native/test_manual_control_lease.cpp) · [Controller behavior](OPERATIONS.en.md)

## Turning teleoperation footage into imitation learning data

Learning required more than a video of a successful lift: **which observation led to which action, and what output was actually applied?** I implemented a recorder that aligns the external view of the arm, the gripper-mounted view of the jaws, button presses and releases, and driver output records. Each action retains its pre-action observation, proposed command, output and cutoff, and post-action observation. Prediction inputs use only information available before execution. [Demonstration recorder](tools/record_controller_ui.py) · [Action boundaries and labels (Korean)](docs/manual-data-collection.md)

I recorded nine whole-arm teleoperation demonstrations. Reconstructing seven actions in the first demonstration involved grouping 98 keep-alive requests with their original gestures. The first **imitation learning (IL)** experiment used offline Behavior Cloning: a classification task predicting the demonstrator's joint and direction from both views at the moment of a button press. Physical whole-arm execution used the demonstration-pose comparison policy described below.

### Which parts of the arm did the camera pipeline locate?

![Part contours overlaid on a physical frame from the manual demonstration](docs/assets/wide-arm-overlay.jpg)

With a wider field of view, the old overlay missed the upper arm and forearm even when both were clearly visible. I located the upper arm using the elbow housing and both rail edges, and the forearm using surface markers and the material connecting them to the wrist. On seven frames from a manual grasp, I checked 28 premarked points per part. Correctly colored points rose from zero on each link to 18 on the upper arm and 24 on the forearm. Some boundaries still disappear at high poses. The overlay lets an observer inspect detections. The action model used the two original camera images. [Part-level results and pose limitations (Korean)](docs/live-observation.md#광각-부품-표시-검증) · [Front and rear policy trials and manual demonstrations (Korean)](docs/relative-action-selection.md#2026-11-21-앞뒤-배치와-광각-수동-시연)

## Translating a research idea into action selection and a physical policy

The design principle I drew from Ebert et al.'s *Visual Foresight* was to compare candidate outcomes, execute one action and decide again from a new image. On this arm, I predicted changes in observable states such as object position, arm markers and jaw spacing. Comparing image-and-command models with per-command means and no-change baselines helped assess the information that each state representation added to action selection. [Research and design decisions (Korean)](docs/research/visual-action-learning.md) · [Model and baseline comparison (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md)

In one physical wrist (M2) trial with a fixed view, a single command selected by a frozen model reduced image-space target-direction error from **5.479° to 0.967°**. For object manipulation, I connected joint-specific models that predict the next change from current observations and candidate commands, then compare movement with stopping against each stage's goal. [Physical M2 selection (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [Model-selected grasp, lift and release (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md)

The whole-arm execution policy combines **comparison of current observations with demonstrated poses, a prescribed stage order and visual stopping conditions**. The host selects shoulder and gripper actions; the ESP32 enforces output, time and angle limits and stops the motors. I connected the sequence through visually stopping gripper opening, checking release on both camera images captured after cutoff, and withdrawing the empty arm. [Whole-arm execution (Korean)](docs/whole-arm-cycle.md) · [Release assessment code](learning/whole_arm_release.py) · [Device stage and output handling](src/control/whole_arm_service.cpp)

In **Rerun**, I replayed both camera views alongside measurements to compare predictions with physical responses. In **W&B**, I recorded model and baseline comparisons and stage-level outcomes in reports. The screens below show predicted and observed object rise after a shoulder command, and changing jaw spacing over three closes. [Replay and experiment comparison tools (Korean)](docs/robot-visualization.md)

| Rerun — video, predictions and measurements | W&B — model and stage comparisons |
| --- | --- |
| [![Rerun replay of predicted and observed rise after a shoulder command](docs/assets/arm-rerun.png)](docs/assets/arm-rerun.png) | [![W&B report comparing three closes with the jaw-gap target](docs/assets/arm-wandb.png)](docs/assets/arm-wandb.png) |

## Comparing the physical arm and simulators through shared geometry

I assembled a common geometry file from physical measurements, assembly drawings and photographs, then compared forward and inverse kinematics (FK and IK) with URDF coordinates. For two demonstrations, I used apparent link lengths in each first frame as references, then estimated joint positions in later frames from shoulder sensor changes and the visible forearm direction. This comparison distinguished the fixed tool frame from the actual jaw tip, whose position changes as the gripper opens. [Dimensions and image comparison (Korean)](docs/arm-calibration.md) · [Shared geometry file](ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/arm_geometry.json)

I inspected the same arm model in three tools. **RViz** showed links, joint axes and coordinate frames. **Gazebo** provided a scene for checking the geometry of the imported SDF model. In **MuJoCo**, I rendered the model and numerically compared its end-effector frame (`tool0`) with the URDF and kinematics calculations. The screens below display the arm built from this shared geometry. [Tool setup and validation (Korean)](docs/robot-visualization.md)

| RViz — links and frames | Gazebo — virtual scene | MuJoCo — model and end-effector comparison |
| --- | --- | --- |
| [![OWI-535 link model in RViz](docs/assets/arm-rviz.png)](docs/assets/arm-rviz.png) | [![OWI-535 arm in a Gazebo scene](docs/assets/arm-gazebo.png)](docs/assets/arm-gazebo.png) | [![OWI-535 model in MuJoCo](docs/assets/arm-mujoco.png)](docs/assets/arm-mujoco.png) |

## From task-state interpretation to language-conditioned actions and sim-to-real

I applied a **vision-language model (VLM)** to interpreting task states in a recorded demonstration. For offline assessment, I grouped 55 camera pairs from one demonstration into eight clips and compared the final “released and standing” reading with the on-site observation. This is the starting point for a **vision-language-action (VLA)** direction connecting observations, language instructions and actions. The planned evaluation asks whether changing only the instruction in the same scene changes action selection accordingly. [Visual state assessment and action learning design (Korean)](docs/visual-action-learning.md)

The next use of simulation is to compare physical and virtual trajectories under the same motor commands and calibrate load, delay and contact responses. Building on this, I plan a sim-to-real evaluation comparing a policy learned through **reinforcement learning (RL)** with the current physical policy on the same task. The existing connection between control, data, models and physical observations provides the foundation for this extension. [Research and evaluation plan (Korean)](docs/research/visual-action-learning.md) · [Public source checks and builds](OPERATIONS.en.md)
