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
| Remaining integration | Object/gripper state estimation, repeated action selection and observation toward a goal, and learned grasp/lift evaluation |

The M4 figures compare predictions with sensor labels from sequential poses on one device on the same day. They do not establish generalization to other days/loads, sensor accuracy or grasping performance. Actual PWM and output duration are retrospective records and are excluded from pre-execution prediction inputs. Data from different joints or observation contracts are not automatically combined.

[Latest M4 evaluation and failure denominators (Korean)](docs/evidence/2026-11-04-m4-action-prediction.md) · [M2 single-command goal approach and limits (Korean)](docs/evidence/2026-10-31-first-model-goal-approach.md) · [Learning goals and design (Korean)](docs/visual-action-learning.md)

The [relative-state development path (Korean)](docs/relative-action-selection.md) connects manual image annotations to prediction and candidate ranking on two M4 transitions. A [gripper-camera pilot (Korean)](docs/evidence/2026-11-04-gripper-relative-learning.md) trained on four M1 transitions and evaluated one new record; no-change was best on that one record. Generalization to other days or loads, automatic state estimation and learned physical grasping remain unverified.

A [subsequent joint-specific study (Korean)](docs/evidence/2026-11-05-multi-joint-relative-learning.md) collected 33 transitions across separate M1, M5 and M2 contracts, with 24 training and nine later evaluation records. Range rejections and large errors remain; learned grasping and lifting are unverified.

A separate [external-camera marker learning contract (Korean)](docs/projected-arm-learning.md) represents arm motion without an object using four pixel values from a fixed background point and two link landmarks. Its states remain distinct from the object/jaw model and calibrated joint angles.

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
