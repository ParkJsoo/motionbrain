# MotionBrain

[한국어](README.md) · [Portfolio](PORTFOLIO.en.md) · [Build and operations](OPERATIONS.en.md)

**A personal robotics project that learns arm motion from images and commands, then compares predicted outcomes to approach a goal.** An ESP32-controlled five-axis DC arm records images, commands, actual outputs and sensors, connecting data validation to training, evaluation and candidate action selection.

Current validation covers constrained single-joint prediction and one-command goal approach. Learned manipulation that uses object/gripper image positions to align, grasp and lift is not yet complete.

## Goal and current scope

The intended loop is `current visual state + candidate command → predict change → compare with goal → execute → observe again`. The first task is to approach, grasp and lift a small object. Current implementation and measurements cover the following scope.

| Stage | Implementation and validation |
| --- | --- |
| Collection and replay | A local Mac UI selects one of M1–M5 at a time and records/replays video, planned commands, actual driver outputs, sensors and failures |
| Datasets | Existing sensor labels target M4. Separate M1, M2 and M5 paths use manually annotated object/jaw state from the gripper camera. Retrospective outputs are excluded from inputs |
| Training and evaluation | Six M4 downward conditions; models frozen after 18 training records and evaluated on 18 new records. Per-command mean MAE **0.367°** versus image-plus-command ridge **0.605°**. No benefit from the current visual representation demonstrated |
| Candidate selection | An offline selector compares predicted residuals for supported commands and stopping. A separate M2 experiment verified projected goal approach with one model-selected command |
| Remaining integration | Automatic object/gripper state estimation, physical integration and repeated evaluation of the implemented trial core, and learned grasp/lift evaluation |

The M4 figures compare predictions with sensor labels from sequential poses on one device on the same day. They do not establish generalization to other days/loads, sensor accuracy or grasping performance. Actual PWM and output duration are retrospective records and are excluded from pre-execution prediction inputs. Data from different joints or observation contracts are not automatically combined.

[M4 sensor evaluation and failure denominators (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) · [M2 single-command goal approach and limits (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [Learning goals and design (Korean)](docs/visual-action-learning.md)

The [relative-state development path (Korean)](docs/relative-action-selection.md) connects manual image annotations to prediction and candidate ranking on two M4 transitions. A [gripper-camera pilot (Korean)](docs/evidence/2026-11-04-gripper-relative-learning.md) trained on four M1 transitions and evaluated one new record; no-change was best on that one record. Generalization to other days or loads, automatic state estimation and learned physical grasping remain unverified.

A [subsequent joint-specific study (Korean)](docs/evidence/2026-11-05-multi-joint-relative-learning.md) collected 33 transitions across separate M1, M5 and M2 contracts, with 24 training and nine later evaluation records. Range rejections and large errors remain; learned grasping and lifting are unverified.

A separate [external-camera marker learning contract (Korean)](docs/projected-arm-learning.md) represents arm motion without an object using four pixel values from a fixed background point and two link landmarks. Its states remain distinct from the object/jaw model and calibrated joint angles.

The [M3/M4 marker-learning follow-up (Korean)](docs/evidence/2026-11-06-projected-arm-m3-m4-learning.md) trained on 9 and 4 transitions, respectively. After freezing, both new M3 transitions were rejected by observation-quality checks and the new M4 transition was outside the training-state range, leaving no formal prediction-error metric. These were not model-selected physical control trials.

A [bounded positioning/close trial core (Korean)](docs/model-grasp-trial.md) connects frozen-model selection, one action and actual re-observation; the core was validated in simulation. A gripper-camera adapter binds original images, timestamps and manually reviewed landmarks, with a diagnostic tool that sends no motor commands. Automatic perception, depth/retention verification and physical grasping/lifting through this path remain unverified.

On 2026-11-07, two cameras confirmed [one physical object lift using a separate fixed manual command (Korean)](docs/m4-up-commissioning.md#2026-11-07-수동-물체-들림1회와-관측-중단). The M4 upward 50%/750 ms request exceeded the 1° observational abort criterion and stopped; the original trial remains failed. The subsequent camera observation of free-space retention does not establish model-selected grasping/lifting or a repeated success rate.

A [pose-separated M1/M4 follow-up and pre-command prediction check (Korean)](docs/relative-action-selection.md#2026-11-07-자세를-분리한-학습과-구동-전-예측-확인) trained separate models on four M1 and two M4 transitions. No-change was best on one new frozen-model M1 comparison, and the next prediction from the resulting state was rejected as outside the training range. That relative-state path had zero model-selected physical commands and zero verified grasps at the time.

In a follow-up on 2026-11-07, an M4 model trained on four fresh records and then frozen selected up at 50% for 650ms from two candidates for an object already grasped manually. Two cameras confirmed one physical lift. The original execution nevertheless failed because the observed change of 3.08° exceeded its 3° guard; all outputs were OFF and the queue was empty. This does not establish autonomous positioning/closing, normal control completion or repeatability. [Evaluation conditions and failure distinction (Korean)](docs/m4-up-commissioning.md#2026-11-07-고정-모델-선택-들림과-실패-구분).

A [close-to-lift connector (Korean)](docs/model-grasp-trial.md#닫기에서-들기로-넘기는-별도-연결) requires a normal close/OFF result and fresh dual-camera contact review before selection by a separate 500ms lift model. Connection failure paths were tested in simulation; physical validation of the complete connection remains separate.

## Architecture

```text
External camera → Mac collection UI → image/command/output/sensor records
                              │                          │
                     bounded single-joint command   validation → training/evaluation
                              ↓                          ↓
STM32 sensing/teleoperation → ESP32 → 5 DC axes     offline candidate comparison
                              ↑
                        AS5600: M4 angle

ESP32-CAM → Raspberry Pi perception/dashboard/ROS2 (existing integration path)
```

Candidate comparison is not connected to automatic control of every joint. ESP32 manages commands, outputs and stop state; the host handles collection, training and evaluation. Only M4 has an angle sensor; the other four axes lack position feedback. The existing ROS2 path has been validated for operator-confirmed single M4 targets.

## Getting started

Host checks use Python 3.12, `numpy`, `opencv-python-headless`, Node.js (CI: 22) and a C++ compiler. Optional PyTorch checks are skipped without their dependencies. Firmware builds require PlatformIO.

```sh
python3 -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

These commands test and build; they do not upload firmware or drive motors. Raw captures and model weights are not included in the public repository, so reproducing measured experiment scores requires those data.

- **Collection, replay, datasets and training:** [Manual collection guide (Korean)](docs/manual-data-collection.md)
- **Installation, firmware, services and ROS2:** [Operations](OPERATIONS.en.md), [wiring](PIN_MAP.en.md), [ROS2 hardware interface](ros2_ws/src/motionbrain_hardware_interface/README.en.md)
- **Checks:** [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml), [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). Software checks are separate from physical performance.

Physical collection requires checking current connections, camera setup and movement clearance. Intermittent HTTP latency remains; the collection UI's abort request does not guarantee an immediate physical emergency stop. The guide specifies execution conditions and stop verification.

## Code and documentation

| Path | Role |
| --- | --- |
| `learning/`, `tools/` | Collection/replay, data validation, perception/motion prediction, evaluation and candidate selection |
| `src/`, `firmware/` | ESP32 motor control, camera and STM32 firmware |
| `ros2_ws/src/` | Messages, bridges, control guards, URDF and hardware interface |
| [Portfolio](PORTFOLIO.en.md) | Problems addressed, design decisions, results and limits |
| [Documentation and evidence index (Korean)](docs/README.md) | Current guides and dated measurements |

## Existing teleoperation demo

![Wired teleoperation demo](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)

[MP4](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4) · Teleoperation at the fixed 2026-06-08 tag, not a grasping demonstration by the current learned model.

## License

Project code is [MIT](LICENSE). STM32 HAL/CMSIS files retain their upstream licenses under `firmware/stm32/MotionBrainSensor/Drivers/`.
