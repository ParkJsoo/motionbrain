# 로보틱스 시스템 준비도

[README](README.md) | [PORTFOLIO](PORTFOLIO.md) | [English](ROBOTICS_SYSTEM_READINESS.en.md)

이 문서는 MotionBrain을 로보틱스 시스템 소프트웨어 관점에서 정리한다.
구현된 증거와 mock/scaffold, 그리고 아직 주장하면 안 되는 한계를 분리한다.

## 역할 적합도

MotionBrain의 강점은 실제 하드웨어를 붙여 운영한 로보틱스 통합 프로젝트라는 점이다.

- ESP32 펌웨어가 TB6612FNG 드라이버를 통해 5축 DC 모터 출력을 담당한다.
- M4 어깨 한 축은 AS5600 절대각 I2C 피드백과 제한 폐루프 목표각 제어를
  실물에서 검증했다. 나머지 축과 `ros2_control` 물리 경로는 개루프다.
- STM32F446 펌웨어가 구조화된 센서/텔레오퍼레이션 프레임을 보낸다.
- Raspberry Pi가 dashboard, perception, ROS2 Jazzy bridge 프로세스를 운영한다.
- ROS2 패키지는 typed status, event, detection, kinematics, guard, mission,
  URDF, RViz, `ros2_control` 표면을 제공한다.
- `ros2_control`은 안전 경계 안에서만 검증했다. mock과 dry-run 표면은
  검증되어 있지만, 실제 물리 구동은 embedded safety gate 뒤에 남겨뒀다.

따라서 이 프로젝트는 embedded safety boundary, 하드웨어 통합, ROS2 시스템
소프트웨어, 실제 로봇 문제 분석에 대한 증거로 쓰는 것이 맞다.

## Repo 안의 증거

- 물리 컨트롤러와 dashboard 개요: [README.md](README.md)
- 포트폴리오 문제 정의와 한계: [PORTFOLIO.md](PORTFOLIO.md)
- 공개 `ros2_control` dry-run 증거:
  [docs/evidence/2026-06-16-ros2-control-open-loop.md](docs/evidence/2026-06-16-ros2-control-open-loop.md)
- 공개 Pi/systemd/ROS2 health 증거:
  [docs/evidence/2026-06-16-pi-system-health.md](docs/evidence/2026-06-16-pi-system-health.md)
- 공개 Pi runtime 측정 증거:
  [docs/evidence/2026-06-17-runtime-measurements.md](docs/evidence/2026-06-17-runtime-measurements.md)
- 공개 embedded bench check 증거:
  [docs/evidence/2026-06-16-embedded-bench-checks.md](docs/evidence/2026-06-16-embedded-bench-checks.md)
- 공개 M4 어깨 폐루프 증거:
  [docs/evidence/2026-06-28-m4-shoulder-closed-loop.md](docs/evidence/2026-06-28-m4-shoulder-closed-loop.md)
- ESP32 safety gate와 dispatcher: `src/control/`, `src/safety/`
- ESP32 motor driver와 pin mapping: `src/motor/motor_driver.*`
- STM32 HAL sensor/teleop firmware: `firmware/stm32/MotionBrainSensor/`
- ROS2 typed messages와 bridge:
  `ros2_ws/src/motionbrain_msgs/`, `ros2_ws/src/motionbrain_ros_bridge/`
- C++ ROS2 control guard: `ros2_ws/src/motionbrain_control/`
- URDF/RViz description: `ros2_ws/src/motionbrain_description/`
- `ros2_control` mock demo: `ros2_ws/src/motionbrain_ros2_control_mock/`
- 안전한 open-loop `SystemInterface` scaffold:
  `ros2_ws/src/motionbrain_hardware_interface/`

## ros2_control 경계

`ros2_control` 표면은 두 가지로 분리되어 있다.

| 표면 | 패키지 | 목적 | 물리 구동 |
| --- | --- | --- | --- |
| Mock controller | `motionbrain_ros2_control_mock` | `mock_components/GenericSystem`으로 controller-manager, joint-state, trajectory-controller bring-up 검증 | 없음 |
| Hardware interface scaffold | `motionbrain_hardware_interface` | 표준 `hardware_interface::SystemInterface` 형태, joint command/state interface, timeout, finite-command guard, launch/config/URDF 표면 | 아직 직접 구동 없음 |

Hardware interface scaffold는 의도적으로 안전하게 막아둔 상태다. `write()`는
ESP32 controller로 POST하지 않는다. 물리 motion authority는 firmware
`SafetyGate`, token-gated operator UI, deadman/teleop timeout, routine execution
policy 뒤에 남아 있다.

사용 가능한 표현:

```text
안전한 open-loop ros2_control SystemInterface scaffold와 mock controller setup을
구현했다. 물리 ESP32 actuation은 firmware safety boundary 뒤에 남아 있으며,
unchecked ros2_control write path로 노출하지 않았다.
```

피해야 할 표현:

```text
closed-loop ros2_control hardware interface 완료, vendor-specific smart actuator
통합, 전체 플랫폼 motion control 완료, encoder-grade joint feedback 확보.
```

## 명령

Host 테스트:

```bash
python3 -m unittest discover -s tests
```

Firmware build:

```bash
pio run
pio run -d firmware/esp32cam
```

Raspberry Pi 또는 Jazzy container에서 ROS2 build:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  motionbrain_msgs \
  motionbrain_control \
  motionbrain_hardware_interface \
  motionbrain_mission \
  motionbrain_ros_bridge \
  motionbrain_description \
  motionbrain_ros2_control_mock
```

Open-loop hardware-interface scaffold launch:

```bash
source install/setup.bash
ros2 launch motionbrain_hardware_interface hardware_interface.launch.py
```

Mock controller evidence launch:

```bash
source install/setup.bash
ros2 launch motionbrain_ros2_control_mock mock_control.launch.py
```

Pi에서 open-loop hardware-interface evidence capture:

```bash
tools/raspi/capture_ros2_control_hardware_evidence.sh
```

## 최신 ros2_control 증거

2026-06-16에 Raspberry Pi 4 / ROS2 Jazzy에서 캡처했다. 캡처는
`ROS_DOMAIN_ID=43`과 hardware-interface URDF parameter
`transport_mode=dry_run`을 사용했다. 따라서 ESP32 controller나 물리 모터를
명령하지 않았다.

| 증거 | 결과 |
| --- | --- |
| `motionbrain_hardware_interface` plugin load | `MotionBrainOpenLoopSystem` loaded, initialized, configured, activated |
| Controllers | `joint_state_broadcaster` active, `motionbrain_arm_controller` active |
| Command interfaces | 5개 position command interface available/claimed |
| State interfaces | 5개 joint position/velocity state interface exported |
| Open-loop trajectory | `FollowJointTrajectory` goal accepted/completed with `SUCCEEDED` |
| `/joint_states` | all `0.0`에서 commanded scaffold position으로 변경 |

## 최신 runtime 측정 증거

2026-06-17에 live Raspberry Pi host에서 read-only command로 캡처했다.
HTTP controller, camera, dashboard, perception endpoint가 `200`을 반환했다.
ROS2 graph discovery에서는 bridge, joint-state, kinematics, control-guard,
mission node가 보였다. 15초 bounded CLI acquisition window 안에서 ROS2 topic
sample과 routine status service/action probe가 성공했다. Pi에서 USB
oscilloscope, logic analyzer, USB serial adapter, meter interface는 보이지
않았으므로 PWM/UART/I2C waveform과 motor-voltage 측정은 여전히 외부 계측
장비가 필요하다.

## 복구된 embedded bench 증거

Repository history에서 digital-multimeter 수준의 bench check 기록을 복구했다.
공통 GND continuity, obvious short, `3V3` logic rail, TB6612FNG `VCC`/`VM`,
active-low button HIGH/LOW 동작, 단순 output voltage sanity를 확인한 기록이다.
이 증거는 embedded bring-up에는 유용하지만 UART timing, PWM duty/frequency,
I2C signal integrity, transient motor voltage, closed-loop joint-control
주장에는 사용할 수 없다.

## 주장 경계

- Host-side ROS2 decision logic과 firmware-level motor authority를 분리했다.
- 문자열 payload만 쓰지 않고 state, routine, detection, kinematics, guard,
  mission state를 typed ROS2 topic으로 노출했다.
- Feedback과 physical validation이 충분해질 때까지 unsafe automation을
  비활성화했다.
- 저가 DC arm의 M4 단일축 시험 피드백을 전체 관절 encoder feedback이나
  production-grade joint control처럼 포장하지 않고 `ros2_control` 표면과
  분리했다.
- 한계를 문서화했다: 나머지 네 축과 전체 로봇팔 폐루프 없음, autonomous
  grasping 주장 없음, production smart-actuator backend 주장 없음.

## 다음 작업

1. DMM 수준을 넘는 embedded 계측 증거를 캡처한다: PWM duty/frequency, UART
   timing, deadman release latency, I2C activity, bounded motor voltage drop.
2. 물리 motion write path를 열기 전에 ESP32 status field 하나를 read-only
   `ros2_control` diagnostic으로 연결한다.
3. 새 actuator hardware가 생기면 작은 bench note만 추가한다: ping,
   present-position read, bounded goal-position write. 종이 위 주장만 추가하지
   않는다.
