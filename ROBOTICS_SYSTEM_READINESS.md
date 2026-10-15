# 로보틱스 시스템 준비도

[README](README.md) | [PORTFOLIO](PORTFOLIO.md) | [English](ROBOTICS_SYSTEM_READINESS.en.md)

이 문서는 MotionBrain을 로보틱스 시스템 소프트웨어 관점에서 정리한다.
구현된 증거와 mock/scaffold, 그리고 아직 주장하면 안 되는 한계를 분리한다.

## 역할 적합도

MotionBrain의 강점은 실제 하드웨어를 붙여 운영한 로보틱스 통합 프로젝트라는 점이다.

- ESP32 펌웨어가 TB6612FNG 드라이버를 통해 5축 DC 모터 출력을 담당한다.
- M4 어깨 한 축은 AS5600 절대각 I2C 피드백과 230-245° matrix 검증 목표
  범위의 제한 폐루프 제어를 실물에서 검증했다. 나머지 축에는 위치 피드백이
  없다. M4는 작업자 승인 단일 목표의 물리 경로가 있고 전체 팔 write는 비활성이다.
- STM32F446 펌웨어가 구조화된 센서/텔레오퍼레이션 프레임을 보낸다.
- Raspberry Pi가 dashboard, perception, ROS2 Jazzy bridge 프로세스를 운영한다.
- ROS2 패키지는 typed status, event, detection, kinematics, guard, mission,
  URDF, RViz, `ros2_control` 표면을 제공한다.
- `ros2_control`은 안전 경계 안에서만 검증했다. mock, dry-run/open-loop,
  M4 read-only measured state 표면은 검증되어 있지만, 실제 물리 구동은
  embedded safety gate 뒤에 남겨뒀다.

따라서 이 프로젝트는 embedded safety boundary, 하드웨어 통합, ROS2 시스템
소프트웨어, 실제 로봇 문제 분석에 대한 증거로 쓰는 것이 맞다.

## Repo 안의 증거

- 물리 컨트롤러와 dashboard 개요: [README.md](README.md)
- 포트폴리오 문제 정의와 한계: [PORTFOLIO.md](PORTFOLIO.md)
- 주장/증거/한계 matrix:
  [docs/evidence/claim-to-evidence-matrix.md](docs/evidence/claim-to-evidence-matrix.md)
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

`ros2_control` 표면은 네 가지로 분리되어 있다.

| 표면 | 패키지 | 목적 | 물리 구동 |
| --- | --- | --- | --- |
| Mock controller | `motionbrain_ros2_control_mock` | `mock_components/GenericSystem`으로 controller-manager, joint-state, trajectory-controller bring-up 검증 | 없음 |
| Hardware interface scaffold | `motionbrain_hardware_interface` | 표준 `hardware_interface::SystemInterface` 형태, joint command/state interface, timeout, finite-command guard, launch/config/URDF 표면 | 아직 직접 구동 없음 |
| M4 measured state mode | `motionbrain_hardware_interface` | `/motionbrain/status_typed` cache에서 M4 `shoulder_pitch_joint` state만 노출하는 read-only `m4_state` mode | command interface 없음 |
| M4 proposal + executor | `motionbrain_hardware_interface`, `motionbrain_ros_bridge` | typed proposal과 별도 작업자 승인 | M4 한정 20초 one-shot; [실물 검증](docs/evidence/2026-07-13-m4-physical-ros2-control.md) |

Hardware interface scaffold와 M4 measured state mode는 의도적으로 안전하게
막아둔 상태다. Physical mode `write()`는 ESP32 controller로 POST하지 않고,
`m4_state` mode는 command interface를 노출하지 않는다. 물리 motion authority는
firmware `SafetyGate`, token-gated operator UI, deadman/teleop timeout, routine
execution policy 뒤에 남아 있다.

사용 가능한 표현:

```text
안전한 open-loop ros2_control SystemInterface scaffold, mock controller setup,
M4 read-only measured state mode를 구현했다. 물리 ESP32 actuation은 firmware
safety boundary 뒤에 남아 있으며 unchecked ros2_control write path로 노출하지
않았다.
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

M4 read-only measured state launch:

```bash
source install/setup.bash
ros2 launch motionbrain_hardware_interface m4_measured_state.launch.py
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

## 2026년 6월 ros2_control 증거

초기 open-loop 증거는 2026-06-16에 Raspberry Pi 4 / ROS2 Jazzy에서 캡처했다. 캡처는
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

이후 M4 read-only measured state mode가 추가됐다. 이 모드는 command interface와
trajectory controller 없이 `joint_state_broadcaster`만 사용하며, cached
`/motionbrain/status_typed`에서 유효하고 fresh한 M4 measured state만
`shoulder_pitch_joint` state로 노출한다.

## 2026년 6월 runtime 측정 증거

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

## 학습 통합 경계

Mac CPU의 NumPy dynamics 기준선과 검증된 데이터셋 경로를 구현했다.
합성 검증은 실물 정확도를 입증하지 않으며 적격 실제 행동 데이터는 0개다.
실물 수집에는 명령 journal 연결과 한 축 제한 동작·출력 off·정착 검증이 남아 있다.
[학습 계약](docs/evidence/2026-10-15-offline-visual-dynamics.md)과
[관측 사용 가능성 미통과 결과](docs/evidence/2026-10-15-bounded-observation.md)를 참고한다.
