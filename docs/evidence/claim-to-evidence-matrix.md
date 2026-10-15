# Claim-to-Evidence Matrix

[PORTFOLIO](../../PORTFOLIO.md) | [Portfolio](../../PORTFOLIO.en.md) | [로보틱스 시스템 준비도](../../ROBOTICS_SYSTEM_READINESS.md) | [Robotics readiness](../../ROBOTICS_SYSTEM_READINESS.en.md)

이 문서는 MotionBrain을 외부에 설명할 때 쓸 수 있는 주장과 증거, 한계,
주장하지 말아야 할 내용을 연결한다. 공개된 구현과 검증 기록을 근거로 한다.

## Matrix

| Claim | Evidence | Limitation | Not claimed |
| --- | --- | --- | --- |
| 실제 하드웨어 arm stack을 통합했다: ESP32 모션 제어기, STM32 센서/텔레오퍼레이션, ESP32-CAM, Raspberry Pi/ROS2 host | `demo-ready-20260608` 물리 텔레오퍼레이션 GIF/MP4, [embedded bench check](2026-06-16-embedded-bench-checks.md), [Pi health](2026-06-16-pi-system-health.md), [runtime measurement](2026-06-17-runtime-measurements.md), `src/`, `firmware/`, `ros2_ws/` | 공개 데모는 작업자 텔레오퍼레이션이며, 장기 반복/production uptime 증거가 아니다 | Autonomous grasping, production reliability guarantee |
| 모터 authority는 ESP32 firmware safety boundary 뒤에 남아 있다 | `src/control/`, `src/safety/`, `tests/test_guarded_routine_contract.py`, `tests/test_motionbrain_mission_flow.py`, [PORTFOLIO 검증 결과](../../PORTFOLIO.md#검증-결과) | 코드/계약 테스트와 제한된 bench evidence다. 독립 safety certification이나 safety-channel validation은 아니다 | Certified functional safety, unattended operation |
| M4 어깨 한 축에서 AS5600 기반 제한 폐루프 목표각 제어를 검증했다 | [M4 어깨 AS5600 폐루프 벤치 검증](2026-06-28-m4-shoulder-closed-loop.md): 230-245도 matrix 검증 목표 범위, 무부하/23.10g 회귀, safety block 0건, 센서 freshness/자석 상태 기록 | M4 한 축, 제한 하중 조건이다. 이후 적용한 ROS zero `222.80 deg`, sign `+1`, `122.08-301.02 deg`는 현재 elbow/wrist/gripper 자세 조건부 임시 소프트 범위이며 230-245도와 동등하게 검증된 범위나 global hard-stop model이 아니다. 나머지 네 축 위치 피드백과 장기 진동/전압별 반복은 미검증이다 | Full-arm closed loop, encoder-grade feedback on all joints, production joint control |
| ROS2 typed bridge, status/diagnostics, kinematics, guard, mission topic surface를 운영했다 | [Pi health](2026-06-16-pi-system-health.md), [runtime measurement](2026-06-17-runtime-measurements.md), `ros2_ws/src/motionbrain_msgs/`, `ros2_ws/src/motionbrain_ros_bridge/`, `ros2_ws/src/motionbrain_control/`, `ros2_ws/src/motionbrain_mission/` | Health/runtime capture는 point-in-time이고 read-only/status probe 중심이다. Routine service/action은 `status`만 호출했다 | Autonomous motion execution, physical routine execution from ROS2 |
| `ros2_control` dry-run/read-only 경계와 operator-confirmed M4 physical single-target 경로를 검증했다 | [open-loop evidence](2026-06-16-ros2-control-open-loop.md), [M4 physical evidence](2026-07-13-m4-physical-ros2-control.md), hardware interface와 executor tests | M4 한 축, 20초 one-shot 확인에 한정된다. executor만 systemd 자동 기동하며 proposal controller는 명시 launch한다 | Full-arm physical ros2_control, unattended actuation, trajectory tracking |
| Pi dashboard/perception pipeline과 제한 known-object `cup` 확인 경로가 있다 | [Pi health](2026-06-16-pi-system-health.md), [runtime measurement](2026-06-17-runtime-measurements.md), `tools/motionbrain_dashboard.py`, `tools/motionbrain_perception_service.py`, [PORTFOLIO 객체 인식 현황](../../PORTFOLIO.md#객체-인식-현황) | YOLOv5s/OpenCV DNN 기반의 제한된 bench 경로다. 모델 weight는 repository에 없고, active target은 `cup` 중심이다. 최신 안정 CAM 서비스 프로필은 QVGA/JPEG quality `15`이며, direct `/stream`은 비활성화되어 `/capture` 또는 Pi tracked frame을 사용한다 | Arbitrary object recognition, marker/object-based autonomous grasping, continuous visual servoing |
| Pi/systemd 운영 표면과 SSH/DNS recovery 절차가 있다 | [Pi health](2026-06-16-pi-system-health.md), [runtime measurement](2026-06-17-runtime-measurements.md), `deploy/systemd/`, `tools/raspi/`, `OPERATIONS.md` | Captured Pi runtime은 point-in-time이다. 과거 `vcgencmd get_throttled=0x50005`가 기록됐고, 전원 어댑터/케이블 교체 후 최신 로컬 확인은 `0x0`이었다. 성능/soak evidence 전후에는 `get_throttled`를 다시 확인해야 한다 | Production uptime, power-integrity closure, thermal/performance guarantee |
| Embedded bring-up의 전원/GND/버튼/출력 sanity check 기록이 있다 | [embedded bench check](2026-06-16-embedded-bench-checks.md), `EMBEDDED_BRINGUP.md` | Digital multimeter 수준 기록이다. Pi에서 USB oscilloscope, logic analyzer, USB serial adapter, meter interface가 보이지 않았다는 runtime inventory도 있다 | PWM duty/frequency, UART bit timing, I2C signal integrity, motor transient sag |
| M4 하중 조건에서 전원 강하와 STM32 MPU boot recovery 이슈를 관찰하고 firmware retry를 보강했다 | [M4 전원 계측과 STM32 MPU 부팅 복구](2026-06-28-m4-shoulder-closed-loop.md#전원-계측과-stm32-mpu-부팅-복구) | 휴대폰 촬영과 일반 멀티미터 기반 근사 계측이다. 배터리/XL4015/배선/부하 영향을 완전히 분리하지 못했다 | Root-cause closure for all power faults, wiring/pull-up/sensor-module defect exclusion |
| 유한 관측 요청과 실패 중단을 구현하고 고정 기준으로 평가했다 | [관측 프로토콜·실측](2026-10-15-bounded-observation.md) | 20요청 중 7개 실행·1개 성공 후 저장 초과로 중단. 사용 가능성 관문 미통과 | Reliable continuous observation, control-ready capture |
| 원본·행동 검증, 그룹 분할, NumPy 선형 dynamics 학습·평가를 구현했다 | [데이터셋 계약](2026-10-14-dataset-manifest-software.md), [특징·CLI·검증](2026-10-15-offline-visual-dynamics.md) | 합성 smoke만 확인. 적격 실물 데이터 0개, production 학습 거부 | Real-data model accuracy, learned manipulation improvement, PyTorch dynamics model |

## 절대 주장하지 않을 항목

- Full-arm closed loop: 미완료. M4 어깨 한 축만 제한 폐루프다.
- Full-arm physical `ros2_control`: 미완료. M4 operator-confirmed single-target만
  실물 검증했으며 trajectory tracking으로 주장하지 않는다.
- Autonomous grasping: 미완료. 현재 공개 데모는 작업자 텔레오퍼레이션과 제한
  perception 확인이다.
- 합성 fixture의 오차 감소를 실물 학습·조작 개선으로 제시하지 않는다.
- 사전 학습 YOLO 추론을 자체 검출기 학습 성과로 제시하지 않는다.

## 사용 기준

이 matrix는 portfolio 문구를 고를 때의 guardrail이다. 위 표의 Evidence에
연결된 항목만 주장하고, Limitation과 Not claimed를 같은 문맥에서 함께 둔다.
새로운 범위는 해당 조건의 실물 검증을 확보한 뒤에만 주장한다.
