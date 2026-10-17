# ROS2 하드웨어 인터페이스

[English](README.en.md) · [운영 안내](../../../OPERATIONS.md)

`motionbrain_hardware_interface/MotionBrainHardwareInterface`는 `ros2_control`의 `SystemInterface` 플러그인이다. 전체 관절에 엔코더가 있다고 가정하지 않고, 추정 상태·M4 실측·실물 실행 제안을 구분한다.

## 모드

| `transport_mode` | 관절과 인터페이스 | 동작 |
| --- | --- | --- |
| `dry_run` | 5관절 위치 명령, 위치·속도 상태 | 개루프 상태 보간. 실물 통신 없음 |
| `m4_state` | `shoulder_pitch_joint` 위치·속도 상태만 | 타입이 정의된 M4 상태 구독. 명령 인터페이스 없음 |
| `m4_proposal` | M4 위치 명령과 상태 | 별도 실행기로 타입이 정의된 제안 발행. 플러그인에서 HTTP 전송하지 않음 |

수명주기 콜백, 유한한 명령값 검사, 명령 타임아웃을 구현한다. 지원하지 않는 모드는 초기화에서 거부한다. `dry_run`의 다섯 관절은 `base_yaw_joint`, `shoulder_pitch_joint`, `elbow_pitch_joint`, `wrist_pitch_joint`, `gripper_joint`다.

M4 측정 모드는 `status_topic=/motionbrain/status_typed`, `feedback_source=m4_as5600`을 사용한다. `shoulder_feedback_calibration_enabled`, `shoulder_sensor_zero_deg`, `shoulder_direction_sign`, `shoulder_ros_joint_zero_rad`로 보정을 지정한다. `state_stale_timeout_sec` 기본값은 2.0초이며 상태 브리지 조회 간격보다 커야 한다. 보정이 꺼져 있거나 피드백이 오래됐거나 유효하지 않으면 상태를 사용 불가로 처리한다. 원본 각도나 개루프 추정으로 대체하지 않는다.

## 빌드와 조회

ROS2 Jazzy 및 [CI의 의존 패키지](../../../.github/workflows/ros2.yml)가 필요하다. `--packages-up-to`로 메시지 패키지 의존성도 함께 빌드한다.

```sh
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to motionbrain_hardware_interface
source install/setup.bash
colcon test --packages-select motionbrain_hardware_interface
colcon test-result --verbose
ros2 launch motionbrain_hardware_interface hardware_interface.launch.py
```

별도 터미널에서 환경을 불러온 뒤 조회한다.

```sh
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

읽기 전용 실측 모드는 `m4_measured_state.launch.py`다. 기본 보정은 비활성화돼 있다. 현재 장착의 영점·방향을 확인한 뒤에만 보정을 활성화한다.

```sh
ros2 launch motionbrain_hardware_interface m4_measured_state.launch.py \
  shoulder_feedback_calibration_enabled:=true \
  shoulder_sensor_zero_deg:='<verified_zero_deg>' \
  shoulder_direction_sign:='<1_or_-1>'
```

## 실물 실행 경계

`m4_proposal.launch.py`는 제안용 제어기를 띄운다. `write()`는 ESP32에 POST하지 않으며, 별도 M4 실행기가 작업자 확인 후 기존 `/shoulder` 명령을 전송한다. 제안 확인의 기본 유효기간 20초는 모터 구동 시간이 아니다. `read()`도 HTTP를 호출하지 않는다.

제안 ID는 무작위 생성자 세션과 순서 번호를 포함한다. 실행기는 영속 저널로 소비된 ID와 결과 불명을 재시작 뒤에도 보존한다. 이는 MCU 중복 제거 또는 실물 완료 증명이 아니다. [저널·행동 계약](../../../docs/data-pipeline.md)과 [운영 복구](../../../OPERATIONS.md#명령-상태와-복구)를 참고한다.

실제 출력은 ESP32의 상태 머신, `SafetyGate`, 토큰 인증, 데드맨·타임아웃과 피드백 검사를 거친다. 호스트 접수와 출력 OFF는 각각 목표 도달·전기적 정지를 보장하지 않는다. 전체 관절 폐루프 궤적 실행과 기능 안전은 미검증이다.

[2026-07-13 M4 단일 목표 실측](../../../docs/evidence/2026-07-13-m4-physical-ros2-control.md)은 이 경로의 제한된 검증 결과다. 나머지 네 축의 엔코더 기반 제어로 확대하지 않는다.

![RViz 모델 예시](../../../docs/assets/motionbrain-rviz-robotmodel.png)

URDF/RViz 모델 표시 예시이며 실물 추종 정확도의 증거는 아니다.
