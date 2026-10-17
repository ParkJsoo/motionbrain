# motionbrain_hardware_interface

[English](README.en.md)

MotionBrain용 `ros2_control` 하드웨어 인터페이스 기본 구현이다. 현재 저가 DC 로봇팔의 전체 관절에 엔코더 수준 피드백이 있다고 가정하지 않고 ROS2 하드웨어 경계를 명시한다.

## 구현 범위

- `hardware_interface::SystemInterface` 플러그인 `motionbrain_hardware_interface/MotionBrainHardwareInterface`를 제공한다.
- 기존 5관절 `transport_mode=dry_run` 시연을 유지한다.
  - `base_yaw_joint`
  - `shoulder_pitch_joint`
  - `elbow_pitch_joint`
  - `wrist_pitch_joint`
  - `gripper_joint`
- 각 dry-run 관절에 위치 명령과 위치·속도 상태 인터페이스를 제공한다.
- 수명주기 콜백, 유한한 명령값 검사, 명령 타임아웃, dry-run 제어기 초기 구동용 개루프 상태 보간을 구현한다.
- 초기화 시 지원하지 않는 전송 모드는 거부한다. 지원 모드는 `transport_mode=dry_run`, 읽기 전용 `transport_mode=m4_state`, 별도 실행기로 제안을 전달하는 `transport_mode=m4_proposal`이다.
- controller-manager 기본 동작 점검용 launch·config·URDF 파일을 포함한다.

## M4 측정 상태 모드

보정된 AS5600 상태의 초기 구동을 위한 읽기 전용 M4 어깨 피드백 모드를 제공한다.

- `transport_mode=m4_state`
- 관절 하나: `shoulder_pitch_joint`
- 상태 인터페이스만 제공: `position`, `velocity`
- 명령 인터페이스 0개
- 명시적 하드웨어 파라미터:
  - `status_topic=/motionbrain/status_typed`
  - `feedback_source=m4_as5600`
  - `shoulder_feedback_calibration_enabled`
  - `shoulder_sensor_zero_deg`
  - `shoulder_direction_sign`
  - `shoulder_ros_joint_zero_rad`
  - `state_stale_timeout_sec` 기본값은 `2.0`이며 예상 상태 브리지 조회 간격보다 커야 한다.

이 모드는 `joint_state_broadcaster`로 보정된 측정 상태를 발행한다. `joint_trajectory_controller`를 불러오거나 실물 명령을 보내지 않는다. 보정이 비활성화됐거나 오래됐거나 유효하지 않으면 원본 AS5600 각도나 개루프 기울기 추정으로 대체하지 않고 상태를 사용 불가로 처리한다.

## 안전 경계

`write()`는 ESP32 제어기에 POST하지 않는다. `m4_proposal` 모드에서는 별도의 작업자 확인형 실물 실행기에 타입이 정의된 제안을 발행한다. 플러그인 자체가 모터·STM32·카메라·조명에 접속하지 않는다.

제안 ID에는 무작위 생성자 세션과 순서 번호가 들어간다. 실행기는 영속 호스트 저널로 소비된 ID와 결과 불명을 재시작 뒤에도 유지한다. MCU 수준 중복 제거나 실물 완료 근거를 제공하는 것은 아니다. [명령 저널 검증](../../../docs/evidence/2026-10-14-command-journal-software.md)과 [제한 행동 계약](../../../docs/evidence/2026-10-14-bounded-actions-software.md)을 참고한다.

dry-run URDF는 `transport_mode=dry_run`을 사용한다. M4 측정 URDF는 `transport_mode=m4_state`로 타입이 정의된 상태만 구독하고 명령 인터페이스를 노출하지 않는다. `read()`에서 HTTP를 호출하지 않는다.

실물 구동은 다음 검사를 통과해야 한다.

- ESP32 펌웨어 `SafetyGate`
- `BOOT -> IDLE -> ARMED -> FAULT` 상태 경계
- 토큰으로 제한한 HTTP 명령
- 데드맨·원격조작 타임아웃
- 루틴 실행기 정책과 피드백 준비 검사

공개 문서에서 설명할 수 있는 범위:

> MotionBrain 로봇팔용 ros2_control SystemInterface 기본 구현에 개루프 dry-run 시연과 읽기 전용 M4 측정 상태 모드를 포함했다. ROS2 하드웨어 경계와 제어기 통합을 검증하며, 실물 구동은 ESP32 펌웨어가 보호한다.

폐루프 궤적 실행, 특정 업체 스마트 액추에이터 통합, 전체 관절 엔코더 검증, 플랫폼 전체 모션 제어, 양산 하드웨어 인터페이스 완성으로 설명하지 않는다.

## 빌드

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_hardware_interface
colcon test --packages-select motionbrain_hardware_interface
colcon test-result --verbose
```

## 실행

```bash
source install/setup.bash
ros2 launch motionbrain_hardware_interface hardware_interface.launch.py
```

실행 후 확인:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

읽기 전용 M4 측정 상태 모드:

```bash
source install/setup.bash
ros2 launch motionbrain_hardware_interface m4_measured_state.launch.py \
  shoulder_feedback_calibration_enabled:=true \
  shoulder_sensor_zero_deg:=<supervised_zero_deg> \
  shoulder_direction_sign:=<1_or_-1>
```

감독하의 읽기 전용 사전 점검으로 현재 장착 영점과 방향을 확인할 때까지 `shoulder_feedback_calibration_enabled=false`를 유지한다.

## 향후 실물 백엔드

향후 ESP32 백엔드는 명시적인 모드 플래그 뒤에 추가하고 펌웨어가 허용한 제한 명령만 전달해야 한다. 현재 로봇팔은 모든 관절에 엔코더가 있는 구성이 아니므로 궤적 완료를 실물 수렴으로 해석하지 않는다. 명령 접수와 추정 동작으로 구분해야 한다.
