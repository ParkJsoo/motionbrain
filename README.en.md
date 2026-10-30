# MotionBrain

[한국어](README.md) · [Portfolio](PORTFOLIO.en.md) · [Build and operations](OPERATIONS.en.md)

A personal robotics project integrating an ESP32-controlled five-axis DC arm, STM32 sensing and teleoperation, ESP32-CAM, Raspberry Pi, and ROS2 Jazzy. Motor control is separated from host decisions and evaluated using sensor feedback and recorded failures. The current research direction is **learning action outcomes from camera observations and motor commands, then using those predictions to approach a goal**.

## Current goal and status

Record arm features and changes around commands from a fixed iPhone view, then learn `current observations and recent history + candidate command → next motion`. Kinematics and existing control provide structure; experience models friction, backlash and pose-dependent responses.

- **Demonstrated:** M4 single-axis control, manually guided grasping and detector development training. All 12 images in the final detector result were training data; one duplicate lip-balm false positive remains.
- **Next implementation:** a collector joining iPhone video with bounded commands and outcomes, arm-feature motion prediction, and command selection followed by re-observation.
- **Not yet demonstrated:** zero eligible physical action transitions. The new visual action model, autonomous approach and grasping remain unimplemented/unverified. Perfect still-image detection is not a prerequisite for the action pilot.

[Goals, stages and evaluation (Korean)](docs/visual-action-learning.md) · [Research basis (Korean)](docs/research/visual-action-learning.md) · [Detector development results (Korean)](docs/evidence/2026-10-30-cv-development.md)

## Demo

![Wired teleoperation demo](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)

[MP4](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4) · Teleoperation from the fixed 2026-06-08 tag; this is not an autonomous grasp demonstration.

## Implementation and validation

| Area | Demonstrated result | Limit |
| --- | --- | --- |
| Embedded control | Five-axis ESP32 PWM, STM32 UART teleoperation, state machine, deadman and sensor-fault gating | Electrical stop latency and certified functional safety are unverified |
| M4 shoulder feedback | AS5600 target control; final no-added-load and 23.10g regressions each passed 11/11 | Fixed mount and limited range on 2026-06-28; no position feedback on the other four axes |
| ROS2 integration | Typed messages, URDF/RViz, control guard, `ros2_control` | Physical writes validated only for operator-confirmed single M4 targets |
| Observed manipulation | Lipbalm grasp/lift followed by two additional grasp/place cycles | An external observer selected manual commands from camera views; first repeat required correction; not a learned policy |
| Detector development | Faster R-CNN fitting: on 12 training images, lip-balm TP8/FP1/FN0, gripper TP12/FP0/FN0, relative position/direction 8/8 | Training-set fit only. Final-model generalization and action learning unverified |
| Action-learning foundation | Bounded commands and outcome logs, NumPy ridge/no-change baselines and session splits | iPhone action collector and new arm-feature model are planned; zero eligible physical action transitions |

## Existing architecture and research path

```text
STM32 (IMU / teleoperation) ── UART ──> ESP32 (commands / safety / PWM) ──> 5 DC axes
                                          ↑ AS5600 feedback on M4 only
ESP32-CAM ── HTTP /capture ──> Raspberry Pi (perception / dashboard / ROS2)
                                  └─ M4 proposal → operator confirmation → ESP32
```

The iPhone-to-Mac observation/training path is distinct from the existing ESP32-CAM-to-Pi path. The new iPhone visual action model is not connected to the device-control architecture shown above.

- Physical outputs pass through the ESP32 `Dispatcher` and `SafetyGate`. Command acceptance is distinct from target attainment.
- Direct camera `/stream` is disabled; images use `/capture` or Pi tracked frames.
- Full-arm closed-loop trajectories and autonomous picking are not implemented. Intermittent HTTP latency remains unresolved.

![Control UI example](docs/assets/motionbrain-control-stream.png)

An existing static UI example, not the current device state.

## Code and documentation

| Path | Contents |
| --- | --- |
| `src/`, `firmware/` | ESP32 controller/camera and STM32 firmware |
| `ros2_ws/src/` | Messages, bridge, control guard, model and hardware interface |
| `learning/`, `tools/` | Perception, data contracts, offline learning/evaluation and operations tools |
| [PORTFOLIO.en.md](PORTFOLIO.en.md) | Design decisions, failure analysis and qualified measurements |
| [OPERATIONS.en.md](OPERATIONS.en.md) | Builds, service configuration, health checks and diagnostics |
| [PIN_MAP.en.md](PIN_MAP.en.md) | Motor/sensor wiring and boot-pin constraints |
| [ROS2 hardware interface](ros2_ws/src/motionbrain_hardware_interface/README.en.md) | Dry-run, measured-state and M4 proposal modes |
| [Visual action learning (Korean)](docs/visual-action-learning.md) | Current goals, stages and proposed observation contract, separated from existing implementation |
| [Existing data and learning pipeline (Korean)](docs/data-pipeline.md) | Existing M4/object-box contract, eligibility, splits and CLI reproduction |
| [Bounded CV development trainer (Korean)](docs/bounded-cv-training.md) | Explicit execution, local weights, step/time caps and failure records; development fitting is distinct from generalization evaluation |
| [Offline lip-balm/gripper CV (Korean)](docs/offline-cv.md) | Approved data loader with an optional PyTorch input adapter, candidate previews, and detection/direction/relative-position evaluation of saved predictions |
| [Saved-checkpoint evaluation runner (Korean)](docs/checkpoint-evaluation.md) | Variable numbers of approved images, strict model restoration, time limits and retained failure denominators; software checks and actual model evaluation results are reported separately |

## Local builds and checks

Host Python dependencies are `numpy` and `opencv-python-headless`; firmware builds require PlatformIO. See [operations](OPERATIONS.en.md) for ROS2 and service setup.

```sh
python3 -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

[PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml) · [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). Results are commit-specific software checks, separate from physical performance.

## License

Project code is [MIT](LICENSE). STM32 HAL/CMSIS files retain their upstream licenses under `firmware/stm32/MotionBrainSensor/Drivers/`.
