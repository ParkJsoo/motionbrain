# MotionBrain | Learning the effects of actions on a physical robot arm

[한국어](PORTFOLIO.md) · [Project overview and setup](README.en.md) · [Design and experiment index (Korean)](docs/README.md)

**Given this scene, where will the arm and the object move if I send this command?** MotionBrain's goal is to learn those changes from images, sensor readings and proposed commands, then choose an action that brings the arm closer to a target. The shoulder is the only one of the five DC axes with direct angle feedback. I built the STM32 teleoperation input, ESP32 motor control, Raspberry Pi ROS2 interface and the software that aligns both camera views with physical motor outputs for training and evaluation.

[![The arm before approaching, during the lift and after releasing the object and retreating](docs/assets/whole-arm-cycle.jpg)](docs/assets/whole-arm-cycle.mp4)

In this 35.5-second recording, the arm picks up an upright object, lowers it back onto the table, opens the gripper and retreats. A person prepared the object and starting pose. From there, the controller used the two camera views and shoulder angle to drive the shoulder and gripper sequentially without joint-button input. [Physical trial video](docs/assets/whole-arm-cycle.mp4) · [Conditions and outcomes of five development trials (Korean)](docs/whole-arm-cycle.md)

## Why start with physical control?

When I requested 234°, the controller reported that the shoulder had reached its target, but inertia carried it to 235.37° after cutoff. I added direction-dependent early cutoff and a correction based on the settled AS5600 reading. Over 230–245°, the arm reached its targets in 11/11 trials without added load and 11/11 with 23.10 g. The mean absolute errors against the sensor were 0.132° and 0.155°, respectively. Later, a physical `ros2_control` trial sent an operator-confirmed 250.00° target to the same controller. The shoulder stopped at 249.96°. [Firmware measurements](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md) · [Physical ROS2 command path](docs/evidence/2026-07-13-m4-physical-ros2-control.en.md)

Tracing actual output also mattered for demonstration collection. During manual gripper control, a late keep-alive request turned the motor back on for 90 ms after a stop. I changed the controller to reject renewals for closed commands. Replaying the same timing in a native regression test eliminated the pulse. After flashing the physical ESP32, I also checked that all five outputs were off at rest. [Regression test](tests/native/test_manual_control_lease.cpp) · [Controller behavior](OPERATIONS.en.md)

## Reconstructing one action from images and motor output

Early teleoperation photographs captured the object in the air, but their timestamps alone could not establish which command had moved it. I aligned the external view of the arm, the gripper-mounted view of the jaws and the operator's button presses and releases with motor output. Each action now has a pre-action observation, a proposed command, driver output and cutoff, and a post-action observation. Only information available before execution enters the prediction model. [Early collection records (Korean)](docs/evidence/2026-10-17-lipbalm-repetitions.md) · [Demonstration recorder](tools/record_controller_ui.py)

I collected nine whole-arm teleoperation demonstrations. In the first, I reconstructed seven actions and grouped the 98 keep-alive requests generated while buttons were held with their original actions. My first imitation learning experiment used offline behavior cloning. Given the two views at the time of a button press, the classifier predicted which joint and direction the operator had chosen. [Action boundaries and labels (Korean)](docs/manual-data-collection.md)

## Which predictions helped the arm choose an action?

I drew on Ebert et al.'s *Visual Foresight* for the idea of comparing candidate outcomes, executing one action and replanning from a new image. On this arm I predicted measurable changes such as object position or jaw spacing. I then tested whether adding image features improved on command-only baselines for the same actions. [Original paper](https://arxiv.org/abs/1812.00568) · [Design choices for this arm (Korean)](docs/research/visual-action-learning.md)

- **Shoulder (M4).** On 18 new actions, an 8×4 grayscale image plus the command predicted the angle change with an error of 0.605°, compared with 0.367° for the per-command mean. I next measured the relative positions of arm markers, the object and the jaws. [M4 baseline comparison (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md)
- **Elbow (M3).** On 11 of 12 new records for which both methods produced a prediction, relative position features gave a mean vector error of 1.074 px compared with 1.341 px for the per-command mean. That initial evaluation had two of the four downward samples required for the comparison by direction. [Evaluation by direction (Korean)](docs/evidence/2026-11-12-m3-goal-prediction.md)
- **Wrist (M2).** In a separate physical trial, one command chosen by a frozen model reduced the image-space direction error from 5.479° to 0.967°. Of four attempts that started recording, this was the one in which the motor ran. [Physical goal approach (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md)

| Rerun | W&B |
| --- | --- |
| [![Rerun replay of predicted and observed rise after a shoulder command](docs/assets/arm-rerun.png)](docs/assets/arm-rerun.png) | [![W&B comparison of jaw spacing across three close commands](docs/assets/arm-wandb.png)](docs/assets/arm-wandb.png) |

In one Rerun replay, the model predicted a 17 px rise from a shoulder command and the object rose 21 px in the recording. In a separate experiment I used W&B to track whether successive close commands brought the jaws closer to their target. Across three commands the gap fell from 137.24 px to 49.70 px against a 35 px image target. [Replay and reports (Korean)](docs/robot-visualization.md)

## From a single command to a whole-arm action policy

At first, each stage had a supplied goal. The frozen models compared candidate actions, including stopping, against the current images. On November 15, the two cameras showed the object held in the air, returned to the table and released. Seven of the eight model-selected commands completed. Lowering that day used an estimate based on reversing the lift response. [Stage decisions and later direct measurements (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md)

For the whole-arm task, I used demonstration poses to guide action selection. The physical runner compared current images with demonstrated poses and applied a predefined stage order and visual stopping rules. In development trial five, it stopped the motor when the jaws opened, checked release on a fresh pair of images, then moved the empty arm away. The object stood upright in the last image, with a vertical gap of 1.42 object heights between it and the gripper. When the policy chose to stop retreating, the shoulder angle was 1.67° below the lower bound from the demonstrations, missing the original trial criterion. The next repeatability trial will score upright placement, clearance and post-stop stability separately from reproduction of the demonstrated pose. [Physical execution and termination criteria (Korean)](docs/whole-arm-cycle.md)

For offline assessment I gave a vision-language model (VLM) 55 camera pairs from one demonstration, grouped into eight clips. Its reading of the last clip, “released and standing,” matched the operator's observation. The next test is whether it can read states on other demonstrations. For a future vision-language-action (VLA) experiment, the question is whether changing only the instruction in the same scene changes the selected action accordingly. [Recorded evaluation and research plan (Korean)](docs/visual-action-learning.md)

## Bringing the measured arm into simulation

With an angle sensor on only one axis, I needed points visible in the video to evaluate the kinematics. I assembled a shared geometry file from physical measurements, assembly drawings and photographs, and compared forward and inverse kinematics (FK and IK). I kept the apparent link lengths measured in the first frame of each demonstration and estimated joint positions in the other eight frames using the change in the shoulder sensor reading and the forearm direction visible in the image. Mean image errors were 1.56 px at the elbow, 4.72 px at the wrist and 46.58 px at the jaw tip. Opening the jaws changed the silhouette endpoint, so locating the grasp point required a separate observation. [Dimensions and image-space measurements (Korean)](docs/arm-calibration.md)

| RViz | Gazebo | MuJoCo |
| --- | --- | --- |
| [![OWI-535 link model in RViz](docs/assets/arm-rviz.png)](docs/assets/arm-rviz.png) | [![OWI-535 arm in a Gazebo scene](docs/assets/arm-gazebo.png)](docs/assets/arm-gazebo.png) | [![OWI-535 model in MuJoCo](docs/assets/arm-mujoco.png)](docs/assets/arm-mujoco.png) |

I inspected the frames and joint axes in RViz. In Gazebo I loaded the SDF model into a scene to check its geometry. In MuJoCo I compared the end-effector frame (`tool0`) numerically against the URDF and kinematics calculations. The rendered pose is for display. [Shared geometry](ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/arm_geometry.json) · [Validation in each simulator (Korean)](docs/robot-visualization.md)

How do load, delay and contact change the physical and simulated arms' responses to the same motor command? Measuring those differences is the next step in calibrating the dynamics. The planned sim-to-real evaluation will compare a policy trained with reinforcement learning (RL) in simulation against the existing physical policy on the same task. [Research behind the plan (Korean)](docs/research/visual-action-learning.md) · [Checks and builds available in the public repository](OPERATIONS.en.md)
