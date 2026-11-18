# MotionBrain

[한국어](README.md) · [Portfolio](PORTFOLIO.en.md) · [Build and operations](OPERATIONS.en.md) · [Documentation index (Korean)](docs/README.md)

MotionBrain is a **five-axis DC robot arm that uses images to choose its next action**. Only the shoulder reports a joint angle. An external camera and a gripper-mounted camera observe the rest of the motion. The task starts with a human teleoperation demonstration. The arm must approach an object, grasp and lift it, set it down, release it and retreat.

![Frames from a physical trial showing the arm and the pink object at different stages](docs/assets/whole-arm-cycle.jpg)

[Physical trial video (MP4)](docs/assets/whole-arm-cycle.mp4) · [Trial conditions and outcomes by stage (Korean)](docs/whole-arm-cycle.md)

**In autonomous trial 5, the arm executed all six stages without an operator pressing joint controls during the run.** The pink object rose and returned to the table. The controller stopped opening the jaws on visual feedback, and the policy received fresh images before moving the empty arm away. The object stood upright in the last observation and all five motor outputs were off.

**The trial failed its original completion criterion.** The object rose 166px against a 226.25px target. During retreat, the model chose to end at a shoulder angle of 214.98°, below the demonstrations’ lower endpoint of 216.65°. In the last image, the vertical gap between the jaws and the object was about 1.42 object heights. Stability over three seconds after stopping and repeatability across starting poses remain to be evaluated.

## What moves the arm?

| Component | Role |
| --- | --- |
| Demonstration recorder | Aligns human joint-button presses and releases with both camera streams and actual PWM outputs recorded by the controller. Renewals of a held button belong to the same action. |
| Host action policy | Compares current images with demonstrations and uses stage-specific models and rules to select an action or stop. Fresh observations inform the next transition. |
| ESP32 and STM32 | The ESP32 manages motor output and stopping. The STM32 participates in sensing and teleoperation. The AS5600 on the M4 shoulder is the only joint-angle sensor. |
| Raspberry Pi and ROS2 | Provide perception, dashboard and ROS2 interfaces. A separate hardware test exercised a single M4 target through ROS2. |

Action prediction uses images and sensor readings available before execution together with the proposed command. The recorded output and subsequent images establish what happened and provide training targets. [Collection and replay (Korean)](docs/manual-data-collection.md) · [Whole-arm learning design (Korean)](docs/visual-action-learning.md) · [Wiring](PIN_MAP.en.md) · [ROS2 hardware interface](ros2_ws/src/motionbrain_hardware_interface/README.en.md)

## Start without hardware

Host checks use Python 3.12, NumPy, OpenCV, Node.js and a C++ compiler. Firmware builds use PlatformIO. Optional PyTorch checks may skip when its dependencies are unavailable.

```sh
python3 -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

The public repository supports source checks and builds. Recalculating experimental metrics requires the original recording, model and camera registration data. The physical trial also used a local host runner. [Operations guide](OPERATIONS.en.md) · [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml) · [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml)

## Explore the evidence

- **What did the policy learn?** The portfolio covers demonstration-based action selection, per-joint response prediction and comparisons with simple baselines. [Engineering decisions and measurements](PORTFOLIO.en.md)
- **How did earlier trials work?** On November 15 a coordinating experiment agent advanced the stages, and a model selected actions within each stage. Lowering used an inverse-UP hypothesis because the training set contained no direct DOWN examples. [Physical outcomes and attempt counts (Korean)](docs/evidence/2026-11-15-prepared-scene-model-grasp-lift-release.md)
- **How closely does the model match the arm?** RViz, Gazebo and MuJoCo share arm geometry. Rerun replays observations, and W&B compares frozen models against baselines. Calibrated dynamics and sim-to-real performance remain open. [Kinematic calibration (Korean)](docs/arm-calibration.md) · [Visualization and evaluation tools (Korean)](docs/robot-visualization.md)

## License

Project code uses the [MIT License](LICENSE). Third-party STM32 HAL/CMSIS files under `firmware/stm32/MotionBrainSensor/Drivers/` retain their original licenses.
