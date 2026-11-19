# MotionBrain | From images to physical actions

[한국어](PORTFOLIO.md) · [Project overview and build](README.en.md) · [Experiment index (Korean)](docs/README.md)

The engineering question behind this project is **whether a physical arm with angle feedback at only the shoulder can learn what its actions do and use successive observations to correct its motion toward a goal**. To investigate this, I connected an ESP32 motor controller, image and output recording, candidate-action prediction and a policy running on the physical arm. In the tabletop experiment below, I recorded human demonstrations, model decisions, rule-based transitions and actual motor outputs separately to determine how the arm and object moved.

![External-camera frames from a whole-arm autonomous trial](docs/assets/whole-arm-cycle.jpg)

[Autonomous trial video (MP4)](docs/assets/whole-arm-cycle.mp4) · [Comparing lift height, jaw clearance and terminal angle (Korean)](docs/whole-arm-cycle.md)

## Why did the trial fail after the arm retreated?

A human prepared the starting scene for trial 5. During execution the host used fresh images and the shoulder angle to approach, close, lift, lower, open and retreat. The bottom of the object rose **166px** above the table reference and returned to that height. During opening, the system checked the object position in the external view and the change in jaw separation in the close view before stopping the output. **A new pair of images after the stop allowed the empty arm to retreat.** This transition was exercised on the physical arm. [Release observer](learning/whole_arm_release.py) · [Firmware action execution](src/control/whole_arm_action.cpp)

During retreat the policy chose to end at **214.98°**. The trial required the shoulder to reach the demonstrations’ lower endpoint of **216.65°**, so it recorded a failure. In the last saved image, the vertical gap from the jaw tip to the top of the object was **1.42 object heights**. Measured the same way, manual demonstrations 7, 8 and 9 ended at **1.74, 2.44 and 2.44 object heights**. The evidence does not establish that the model computed this gap when it chose to stop.

This motivates two evaluation measures. **Task completion** asks whether the object stays upright and clear of the empty gripper after stopping. **Demonstration fidelity** measures how closely the arm matches the human terminal pose. The trial targeted a 226.25px lift and lacks the three-second post-stop observation. The proposed criteria still need to be fixed on independent recordings. [Original trial criteria and outcome (Korean)](docs/whole-arm-cycle.md)

## Turning teleoperation into action labels

Holding a joint button sends multiple renewals. Treating each request as an action would teach the learner that the operator repeatedly started the same motion. I used the browser press and release as action boundaries and aligned them with both camera streams and the controller’s output and stop records. The first whole-arm demonstration yielded **seven actions** from approach to post-release retreat. **98 renewals** remained associated with those actions. The onset of table support stayed uncertain where the images could not resolve it. [Recorder](tools/record_controller_ui.py) · [Input capture](tools/controller_ui_capture.js) · [Controller renewal validation](src/network/manual_control_lease.cpp)

The collection reached nine manual whole-arm demonstrations. An initial behavioral cloning experiment matched two-camera observations against demonstration images to predict the operator’s next joint and direction. That offline comparison was given human press times. The physical executor makes additional stage-transition and stop decisions. For approach, it checks the scene, shoulder angle and object placement against demonstrations before reusing a waypoint previously crossed on hardware. Performance on new placements remains unverified. [Approach waypoint selection](learning/approach_exemplar.py) · [Demonstration recording (Korean)](docs/manual-data-collection.md)

In an earlier November 15 experiment, a coordinating agent ordered and started stages while models chose among candidates within each one. Its final six commands were three CLOSE actions and one each of UP, DOWN and OPEN. Lowering used an inverse-response hypothesis with no direct DOWN training examples. Later direct collections are recorded separately. [Earlier trial and counts of model-selected and manually specified commands (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md)

## When a simple baseline wins

| Experimental question | Measurement | Engineering decision |
| --- | --- | --- |
| Does a pre-action image improve prediction of downward M4 motion? | Six conditions, 18 training cases and 18 fresh evaluations. Mean absolute error in sensor-angle change was **0.367°** for the per-command mean and **0.605°** for low-resolution image-plus-command ridge regression. | The command mean was more accurate. Subsequent work explicitly tracked the relative positions of the jaws and object. [Evaluation conditions (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) |
| Does relative position help predict M3 motion? | At 60% for 350ms, 18 training cases and 12 fresh evaluations. Across 11 cases supported by both methods, mean vector error was **1.074px** for the model and **1.341px** for the command mean. | Only two DOWN cases qualified against a minimum of four. The original evaluation retained its failed status. [Denominators and follow-up (Korean)](docs/evidence/2026-11-12-m3-goal-prediction.md) |
| Did the shoulder reach the target after output cutoff? | At 230–245°, early cutoff and settling checks yielded **11/11 targets, 0.132°** sensor-reference mean absolute error unloaded and **11/11, 0.155°** with a 23.10g load. | Record output cutoff, sensor settling and target arrival as separate events. [Physical M4 regression](docs/evidence/2026-06-28-m4-shoulder-closed-loop.en.md) |

## What images and kinematics can establish

- **Multimodal state assessment.** I split 55 paired camera observations from one saved episode into eight clips and assessed them in shuffled order. The terminal “released and standing” label agreed with the operator. The episode description was known during this development assessment. Independent VLM checkpoint accuracy and the instant of contact before opening remain unverified.
- **Arm pose error.** I registered the first frame of each of two episodes from the same external view and evaluated the remaining eight frames. With motion changes alone, mean image errors for the elbow proxy, wrist and jaw tip were **1.56, 4.72 and 46.58px** respectively. Opening the jaws changes the silhouette endpoint, so matching the elbow gives limited information about grasp-point accuracy. [Geometry source](ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/arm_geometry.json) · [Calibration methods and limits (Korean)](docs/arm-calibration.md)
- **Simulation and replay.** FK, IK and URDF share dimensions that can be cross-checked, and poses can be inspected in RViz, Gazebo and MuJoCo. Rerun replays observations and W&B compares model and baseline metrics. Mass, inertia, gripper contact and the state of the four other joints still require physical calibration. [Tools and verification scope (Korean)](docs/robot-visualization.md)

The next hardware evaluation will fix task-completion criteria, count successes, failures and human interventions from the same starting condition, then vary object placement and starting pose. That supplies a common task for comparing demonstration-based policies with future VLA and reinforcement-learning policies. Sim-to-real work next requires measuring the difference between simulated and physical trajectories under the same commands.
