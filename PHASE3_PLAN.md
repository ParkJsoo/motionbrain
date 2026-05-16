# MotionBrain Phase 3 Plan

Phase 3는 MotionBrain을 "움직이는 ESP32 프로젝트"에서 "센서 피드백과 판단 계층을 가진 로봇 제어 시스템"으로 끌어올리는 단계다.

이 문서는 아이디어 메모가 아니라 실제 구현 순서와 완료 기준을 정리하는 실행 계획 문서로 유지한다.

## Phase 3의 목표

Phase 3에서 반드시 만들어야 하는 결과는 다음 네 가지다.

- STM32 기반 센서/teleop 계층이 `HC-SR04` safety input과 `GY-521` handheld input을 ESP32로 보낸다.
- ESP32가 센서를 받아 safety에 반영한다.
- 입력 채널이 직접 모터를 때리지 않고 공통 명령 경로를 지난다.
- 유선 handheld teleop 입력을 안전한 motion 입력 채널로 붙인다.

## 현재 진행 상태

기준 날짜: `2026-05-16`

- `Phase 1`: 완료
- `Phase 2`: 사실상 완료
- `Phase 3`: 진행 중
- `3-A Sensor Feedback Layer`: bench 기준 거의 완료, 최종 실장 후 재검증만 남음
- `3-B Decision Layer`: 1차 완료
- `3-C Base Closed-Loop Motion`: 1차 구현 완료, 현재는 optional/보류 기능
- `3-C Teleop Motion Input`: 유선 MVP 구현과 parser 실기 검증 완료, 최종 배치 후 mixer 튜닝 대기
- `3-D Message Bridge`: 부분 완료

현재 해석은 다음과 같다.

- 코드 구조상 Phase 3의 핵심 모듈은 이미 들어가 있다.
- 지금 병목은 새 기능 추가보다 최종 배치, 배선, 통합 검증 준비다.
- `GY-521`은 handheld remote 입력용으로 쓰기로 했으므로 base 상대각 폐루프 물리 검증은 현재 Phase 3 gate에서 제외한다.
- 기존 `base angle` 기능은 구현/문서화된 optional 실험 기능으로 남기며, 실제 폐루프를 다시 목표로 잡으려면 별도 base-mounted IMU/엔코더 같은 피드백 센서가 필요하다.
- teleop mixer의 부호와 비중은 로봇팔 최종 자세, 링크 배치, 리모컨 장착/파지 방향이 확정된 뒤 튜닝한다.

## 현재 확정 사실

### ESP32 측

- 엔트리포인트: `src/main.cpp`
- 현재 연결된 모듈:
  - `SystemStateManager`
  - `MotorControl`
  - `RobotArm`
  - `MotionSequence`
  - `SerialCommand`
  - `WiFiAP`
  - `MotionBrainWebServer`
  - `SearchLight`
  - `Stm32Bridge`
  - `SafetyMonitor`
  - `CommandBus`
  - `SafetyGate`
  - `Dispatcher`
  - `AngleController`
  - `EventLog`
- 웹 라우트:
  - `GET /status`
  - `GET /events`
  - `POST /command`
  - `POST /motor`
  - `POST /joint`
  - `POST /base`
  - `POST /sequence`
  - `POST /light`
- `SerialCommand`와 `MotionBrainWebServer`는 공통 `Command` 경로를 사용한다.
- 현재 `/status`는 시스템 상태, 모터 상태, 센서 health, base angle 상태를 모두 포함한다.

### STM32 측

- 보드: `B-F446E-96B01A`
- 프로젝트 경로:
  - `firmware/stm32/MotionBrainSensor`
- `MPU-6050` 통신 확인 완료
- `WHO_AM_I = 0x68` 확인 완료
- 자이로 바이어스 캘리브레이션 추가 완료
- 200Hz 샘플링 동작 확인 완료
- `HC-SR04` 측정과 UART 센서 송신을 bench에서 확인했다.
- `STM32 -> ESP32` 센서 브리지는 bench 기준으로 수신 확인이 끝났다.

### 검증된 STM32 핀

- `MPU-6050`
  - `I2C2_SCL = PB10`
  - `I2C2_SDA = PC12`
- `USART1`
- `USART2`
  - `TX = PD5`
  - `RX = PD6`
- `HC-SR04` 예약 핀
  - `TRIG = PD4 = Arduino D2`
  - `ECHO = PC8 = Arduino D3`
- `TIM3`
  - 현재 5ms 주기 기반 샘플 타이머로 사용 중

## 설계 원칙

- 첫 목적은 "센서값 표시"가 아니라 "센서 기반 safety와 구조 분리"다.
- 최소 경로가 먼저다. `STM32 -> ESP32` 단방향 센서 스트림이 우선이다.
- 현재 코드 구조에 무리하게 큰 리팩터링을 걸지 않는다. `3-A`와 `3-B`는 단계적으로 분리한다.
- 현재 보유 `MPU-6050`/`GY-521`은 handheld remote 입력용으로 취급한다.
- 로봇 본체의 base 상대각 폐루프나 vibration fault를 다시 활성 데모로 잡으려면 별도 본체 IMU 또는 회전 피드백 센서가 필요하다.
- `ESP32-CAM`과 `RPi/ROS2 + AI`는 Phase 3의 구현 대상이 아니라, Phase 3 결과를 받는 다음 단계다.

## 현재 코드 기준 삽입 지점

### `src/main.cpp`

현재 `loop()`는 아래 흐름으로 묶여 있다.

```text
systemState.update()
stm32Bridge.update()
safetyMonitor.update()
angleController.update()
dispatcher.dispatchPending()
motorControl.update()
motionSequence.update()
serialCommand.update()
wifiAP.update()
webServer.update()
```

즉, Phase 3에서 계획했던 계층은 이미 메인 루프에 삽입된 상태다.

### `src/input/serial_command.*`

- 현재는 문자열 파싱 후 `Command`를 생성하고 공통 경로로 넘긴다.
- base 상대각용 `base angle ...`, `base stop` 명령도 지원한다.

### `src/network/web_server.*`

- 현재는 HTTP 파라미터를 검증한 뒤 `Command`를 생성하고 공통 경로로 넘긴다.
- `/status`는 센서 상태와 `baseAngle` 상태를 포함한다.
- `/events`와 `/base`가 추가돼 메시지 경계가 확장됐다.

### `src/system/system_init.*`

- 기본 상태 머신은 여전히 핵심 safety 축이다.
- 센서 기반 차단 이유는 `SafetyMonitor`와 `EventLog`를 통해 노출된다.
- `/status`와 `/events`에서 safety reason을 확인할 수 있다.

## Phase 3 범위

### 이번 단계에 포함

- `MPU-6050 + HC-SR04 + UART` 센서 스트림
- ESP32 센서 수신
- 거리/진동 기반 safety
- 공통 명령 경로 분리
- 유선 handheld teleop 입력
- 베이스 상대각 제어는 optional 구현 상태로 유지
- Phase 4로 넘길 메시지 규격 초안

### 이번 단계에서 제외

- LCD 표시
- 양방향 STM32 명령 채널
- 장기 절대 yaw 기반 정렬
- 본격적인 비전 처리
- ROS2 노드 구현

## 3-A. Sensor Feedback Layer

목표는 STM32가 센서를 읽고, ESP32가 그 값을 받아 safety에 반영하는 최소 피드백 루프를 만드는 것이다.

### 3-A STM32 작업

1. `MPU-6050`에서 sensor stream/teleop에 필요한 값만 정리한다.
   - `roll`
   - `pitch`
   - `gyro_x`, `gyro_y`, `gyro_z`
   - 필요 시 진동 판단용 magnitude
2. `HC-SR04` 거리 측정을 추가한다.
3. 센서 샘플과 거리값을 주기 패킷으로 합친다.
4. `USART2 TX (PD5, Arduino D1)`로 line-delimited JSON 송신을 붙인다.

### 3-A STM32 주기 제안

- IMU 내부 샘플링: 기존 200Hz 유지 가능
- ESP32 송신 패킷: 10Hz ~ 20Hz
- 초음파 측정: 10Hz 수준부터 시작

Phase 3 MVP에서는 "센서를 빨리 읽는 것"보다 "안정적으로 송신되고 ESP32가 소비 가능한 것"이 더 중요하다.

### 3-A UART/배선 원칙

첫 구현은 단방향이 우선이다.

```text
STM32 USART2_TX (PD5 / Arduino D1) -> ESP32 RX
GND common
```

주의:

- ESP32 수신 핀은 현재 미사용 핀 중에서 잡되, strapping pin 리스크를 다시 확인하고 최종 확정한다.
- 현재 `PIN_MAP.md`에 있는 `GPIO15`는 미사용이지만 strapping pin 주의 대상이므로 배선 전 재검토가 필요하다.

### 3-A 패킷 초안

첫 버전은 JSON 한 줄 패킷으로 충분하다.

```json
{
  "type": "sensor",
  "ts_ms": 12345,
  "imu_ok": true,
  "range_ok": true,
  "roll": -2.1,
  "pitch": 1.4,
  "gyro_x": 0.2,
  "gyro_y": -0.1,
  "gyro_z": 14.6,
  "vibe": 1.8,
  "dist_cm": 18.3
}
```

이벤트 패킷은 필요 시 나중에 추가한다. MVP에서는 주기 패킷 하나로 시작하는 편이 낫다.

### 3-A ESP32 작업

추가 대상 모듈 초안:

- `src/bridge/stm32_bridge.h`
- `src/bridge/stm32_bridge.cpp`
- `src/safety/sensor_snapshot.h`
- `src/safety/safety_monitor.h`
- `src/safety/safety_monitor.cpp`

최소 책임은 다음과 같다.

- `stm32_bridge`
  - UART 버퍼 수신
  - line 단위 패킷 분리
  - JSON 파싱
  - 최신 센서 스냅샷 갱신
  - freshness timestamp 관리
- `safety_monitor`
  - 최신 센서 스냅샷 평가
  - 거리 임계값, 진동 임계값 비교
  - `stop` 또는 `FAULT` 트리거
  - stale sensor 감지

### 3-A 상태 노출

`/status` 또는 로그에 최소한 다음 정보가 보여야 한다.

- `sensor.connected`
- `sensor.last_update_ms`
- `sensor.dist_cm`
- `sensor.vibe`
- `sensor.blocked`
- `sensor.block_reason`

### 3-A 완료 기준

- STM32가 `MPU-6050 + HC-SR04`를 읽고 주기적으로 송신한다.
- ESP32가 센서 패킷을 수신하고 파싱한다.
- 센서 신호가 끊기면 stale 상태가 감지된다.
- `distance < threshold -> stop`
- `abnormal vibration -> FAULT`
- 수신 센서 상태를 `/status` 또는 로그로 확인할 수 있다.

### 3-A 현재 상태

- `Stm32Bridge`와 `SafetyMonitor`는 ESP32 코드에 반영 완료
- `/status.sensor`에 `connected`, `packetsReceived`, `imuOk`, `rangeOk`, `distCm`, `vibe`, `blockReason`, `faultLatched` 노출 완료
- bench 기준으로 아래 동작을 확인했다.
  - `STM32 -> ESP32` UART 센서 브리지 수신
  - `OBSTACLE` 차단
  - `VIBRATION`만 `FAULT` latch
  - `SENSOR_STALE` 감지
  - `OBSTACLE` 또는 `SENSOR_STALE` 중 `ARM` 거부

### 3-A 남은 작업

- 최종 실장 상태에서 센서 스트림 안정성 재검증
- 임시 점퍼선이 아닌 실제 배선 상태에서 `RANGE_FAULT` 간헐 개입 여부 재확인
- 조립 후 센서 하우징/방향에 따라 임계값 재조정 필요 여부 확인

## 3-B. Decision Layer

목표는 입력이 직접 모터를 때리지 않게 만드는 것이다.

### 필요한 구조

- `Command`
- `CommandBus`
- `SafetyGate`
- `Dispatcher`

권장 파일 초안:

- `src/control/command.h`
- `src/control/command_bus.h`
- `src/control/command_bus.cpp`
- `src/control/safety_gate.h`
- `src/control/safety_gate.cpp`
- `src/control/dispatcher.h`
- `src/control/dispatcher.cpp`

### 입력 채널 역할 분리

- `SerialCommand`
  - 문자열 파싱
  - `Command` 생성
  - 버스 enqueue
- `MotionBrainWebServer`
  - HTTP 파라미터 검증
  - `Command` 생성
  - 버스 enqueue
- `Dispatcher`
  - 실행 대상 라우팅
- `SafetyGate`
  - 현재 시스템 상태, 센서 차단 상태, stale sensor 여부 검사

### 초기 Command 범위

처음부터 모든 것을 일반화할 필요는 없다. 아래 정도면 충분하다.

- `ARM`
- `DISARM`
- `STOP`
- `MOTOR_RUN`
- `MOTOR_STOP`
- `JOINT_RUN`
- `JOINT_STOP`
- `SEQUENCE_ADD`
- `SEQUENCE_RUN`
- `SEQUENCE_STOP`
- `LIGHT_SET`

### 3-B 단계적 전환 순서

1. 내부 `Dispatcher`를 먼저 만들고 기존 로직을 옮긴다.
2. `SerialCommand`를 `Command` 생산자 구조로 바꾼다.
3. `WebServer`도 같은 경로로 옮긴다.
4. 마지막에 direct path를 제거한다.

이 순서를 지키면 한 번에 전부 뜯지 않아도 된다.

### 3-B 완료 기준

- 시리얼과 웹이 같은 명령 경로를 사용한다.
- 센서 차단 상태에서는 명령이 실행되지 않는다.
- 거부된 명령은 이유가 로그에 남는다.
- 직접 실행 경로가 정리된다.

### 3-B 현재 상태

- `Command`, `CommandBus`, `SafetyGate`, `Dispatcher` 구현 완료
- 시리얼과 웹 입력 모두 공통 `Command` 경로를 사용한다.
- 거부 사유는 `CommandResult`, 로그, `/events`로 확인할 수 있다.
- `MOTOR`, `JOINT`, `SEQUENCE`, `LIGHT`, `BASE_ANGLE_RUN`까지 공통 경로에 포함됐다.

### 3-B 남은 작업

- 직접 실행 경로가 남아 있지 않은지 조립 전 한 번 더 점검
- 최종 통합 검증 중 발견되는 예외 케이스를 `SafetyGate` 메시지와 로그에 보강
- 필요 시 시퀀스와 base 상대각 명령의 충돌 규칙을 더 명확히 문서화

## 3-C. Closed-Loop Motion

목표는 센서를 실제 동작 제어에 연결하는 것이다.

1차 대상은 베이스 회전만 잡는다.

### 범위

- `M5` 베이스 회전만 대상
- 짧은 상대각 회전만 지원
- 절대 heading 제어는 하지 않음

### 필요한 구성

- `AngleController`
- 목표각, 현재 추정각, 오차, 허용 오차 관리
- 타임아웃과 최대 회전 시간 관리

권장 파일 초안:

- `src/control/angle_controller.h`
- `src/control/angle_controller.cpp`

### 입력 형태 초안

시리얼:

```text
joint base left 40
joint base right 40
```

Phase 3-C 이후 확장:

```text
base angle left 45
base angle right 30
```

또는 시퀀스 확장:

```text
sequence add base left 40 angle=45
```

MVP에서는 새 명령을 과하게 늘리지 말고 베이스 전용 상대각 명령 하나만 추가하는 편이 낫다.

### 3-C 제어 기준

- 목표각 도달 시 자동 정지
- 허용 오차 예: `+-3 deg`
- 타임아웃 시 정지 후 경고 로그
- 센서 stale 또는 vibration 시 즉시 중단

### 3-C 완료 기준

- 베이스에 상대각 회전 명령을 줄 수 있다.
- 목표각 근처에서 자동 정지한다.
- 로그에서 목표각, 현재각, 종료 이유를 확인할 수 있다.

### 3-C 현재 상태

- `AngleController` 구현 완료
- 시리얼 `base angle <left|right> <deg> [percent]` 지원 완료
- HTTP `POST /base?action=angle...` 및 `POST /base?action=stop` 지원 완료
- `/status.baseAngle`에서 현재 추정각, 남은 각도, 처리 샘플 수, 마지막 종료 이유를 확인할 수 있다.
- 센서 미실장 상태 bench에서는 `NO_ROTATION_FEEDBACK` 보호 종료가 정상 동작하는 것을 확인했다.
- 이후 `GY-521`을 handheld remote 입력용으로 쓰기로 했으므로 base 상대각 물리 검증은 현재 활성 gate에서 제외한다.

### 3-C 남은 작업

- 현재 로드맵에서는 없음
- base 상대각 폐루프를 다시 목표로 잡으면 별도 base-mounted IMU/엔코더를 추가한 뒤 물리 검증과 튜닝을 재개한다.

## 3-D. Message Bridge

목표는 향후 Raspberry Pi + ROS2 연결 전에 메시지 경계를 정리하는 것이다.

### Phase 3에서 정리할 것

- 센서 메시지 필드
- 상태 메시지 필드
- 이벤트 메시지 필드
- stale/heartbeat 기준
- 상위 호스트가 어떤 주기로 poll 또는 subscribe할지 기준

### 최소 문서화 대상

- `sensor`
  - `ts_ms`
  - `dist_cm`
  - `vibe`
  - `imu_ok`
  - `range_ok`
- `status`
  - `state`
  - `motorEnabled`
  - `sensor.connected`
  - `sensor.blocked`
  - `sensor.block_reason`
- `event`
  - `OBSTACLE`
  - `VIBRATION`
  - `SENSOR_STALE`
  - `FAULT_ENTERED`

### 3-D 완료 기준

- ESP32 내부와 상위 호스트 간 메시지 경계가 문서로 정리된다.
- Phase 4에서 ROS2 메시지로 옮길 때 재설계 비용이 크지 않다.

### 3-D 현재 상태

- `MESSAGE_INTERFACE.md`에 `phase3.v1` 메시지 경계 정리 완료
- `GET /status`, `GET /events`, 명령 응답 envelope, base angle 종료 이유를 문서화했다.
- `GET /events` 최근 이벤트 API가 구현돼 있다.

### 3-D 남은 작업

- Phase 4 진입 전 host-side poll 주기와 상태/이벤트 소비 방식 확정
- `ESP32-CAM`, `RPi/ROS2 + AI`가 사용할 최소 명령/상태 집합 최종 확정
- 필요 시 event stream 또는 host bridge 형식 초안 추가

## 구현 순서와 현재 상태

- [x] STM32 `HC-SR04` 추가
- [x] STM32 UART JSON 송신
- [x] ESP32 `stm32_bridge`
- [x] ESP32 `safety_monitor`
- [x] `/status` 센서 상태 노출
- [x] `Dispatcher` 도입
- [x] `SerialCommand`를 `Command` 생산자로 전환
- [x] `WebServer`를 같은 경로로 전환
- [x] 베이스 상대각 제어 추가, 현재 optional/보류
- [x] 유선 handheld teleop v1 구현
- [x] 메시지 인터페이스 1차 정리
- [x] handheld remote 버튼/배선 확정
- [x] teleop UART 파서 경고 출력 정리
- [x] teleop UART 파서 경고 실기 재확인
- [x] `sensor sim off` AUTO 해제 확인
- [ ] 최종 배치도와 배선표 확정
- [ ] teleop mixer 부호와 비중 실기 튜닝
- [ ] 조립 후 safety/teleop bring-up 체크리스트에 따라 통합 검증
- [ ] Phase 4 진입용 host-side 경계 최종 확정

## 조립 전에 할 일

- 최종 부품 배치안 확정
- 전원, 공통 GND, 모터선, UART, I2C, 초음파 배선을 포함한 배선표 확정
- 조립 후 1차 bring-up 절차 문서화
- `PHASE3_PLAN.md`, `README.md`, `MESSAGE_INTERFACE.md` 간 상태 설명 불일치 제거
- 실장 후 바로 확인할 핵심 시나리오 고정
  - `sensor stream`
  - `OBSTACLE`
  - `SENSOR_STALE`
  - `VIBRATION`
  - `arm/disarm/recover`
  - `teleop deadman/freshness`
  - `reach/lift/twist/grip`

## 검증 체크리스트

### Bench Simulation Gate

하드웨어 재배치 전에는 시리얼 `sensor sim ...` 명령으로 아래 항목을 재현 가능해야 한다.

- `healthy -> arm` 에서 차단 없이 진입
- `obstacle` 에서 `ARM` 거부 또는 동작 중 즉시 차단
- `vibration` 에서 `FAULT` latch
- `stale` 에서 `SENSOR_STALE`

이 gate는 실기 검증을 대체하지는 않지만, Phase 3의 safety/state 경로가 코드상으로 유지되는지 빠르게 확인하는 용도다.

### Gate 1. 센서 스트림

- 현재 상태: bench 기준 1차 통과
- 남은 것: 최종 실장 후 1분 이상 연속 수신과 간헐 `RANGE_FAULT` 여부 재검증

### Gate 2. Safety

- 현재 상태: bench 기준 1차 통과
- 확인한 것:
  - 장애물 접근 시 즉시 차단
  - `VIBRATION`만 `FAULT` latch
  - `RECOVER` 후 `IDLE` 복귀
  - non-motion 명령 중에도 `AUTO_SAFE_TIMEOUT` 동작
- 남은 것: 최종 조립 상태에서 재현성 확인

### Gate 3. 명령 경로 통합

- 현재 상태: 구현 완료
- 남은 것: 최종 통합 중 발견되는 예외 케이스 점검

### Gate 4. Handheld Teleop

- 현재 상태: ESP32/STM32 코드 구현과 bench 실기 확인 완료
- 확인한 것:
  - `sensor sim healthy -> arm`
  - deadman hold + IMU 입력으로 실제 모터 출력
  - release 시 `DEADMAN_RELEASE`
  - `teleop.parseErrors=0`, `sensor.parseErrors=0`
  - `sensor sim off` 후 `Simulation: OFF`, `SENSOR_STALE` 복귀
- 남은 것:
  - 최종 실장 기준 부품 배치와 배선 고정
  - 최종 배치 상태에서 `deadman`, `FRAME_TIMEOUT`, `reach/lift/twist/grip` 재확인
  - 최종 배치 상태에서 mixer 부호와 비중 조정

## 현재 핵심 리스크

- ESP32 수신 핀을 잘못 고르면 boot strapping 이슈가 생길 수 있다.
- `GY-521`을 remote에 쓰면 로봇 본체 vibration fault와 base 폐루프는 별도 센서 없이는 활성 데모가 아니다.
- teleop는 deadman/freshness fail-safe가 실제 배선 노이즈와 함께 검증돼야 한다.
- Command 구조를 너무 크게 설계하면 실제 구현보다 구조 논의만 길어질 수 있다.

## Phase 4 진입 조건

Phase 4는 아래 조건이 충족된 뒤 시작한다.

- STM32 센서 스트림이 안정적이다.
- ESP32 safety가 센서 기반으로 동작한다.
- 시리얼/웹 입력이 공통 명령 경로를 쓴다.
- `/status`에서 센서 health와 block reason을 확인할 수 있다.
- 유선 handheld teleop가 deadman/freshness fail-safe 아래에서 동작한다.

현재 판정:

- 센서/safety/command/teleop 경로는 bench 기준으로 대부분 충족
- Phase 4 전에 최종 실장 배치, safety 재검증, host-side 상태/이벤트 경계 확정이 남아 있다.

## Phase 4 연결 방향

Phase 4의 기본 구조는 다음과 같다.

```text
[STM32 Sensor Hub] -> [ESP32 Motion Controller] <-> [Raspberry Pi + ROS2 + AI]
                         ^
                         |
                   [ESP32-CAM]
```

역할 분리는 다음처럼 고정한다.

- STM32: 센서 수집, 저수준 센서 전처리
- ESP32: 실시간 제어, safety, 모션 실행
- ESP32-CAM: 영상 입력
- Raspberry Pi: 비전/AI 처리, ROS2 메시지 브리지, 데모 오케스트레이션

## Phase 5에서 정리할 것

Phase 3를 마치면 Phase 5에서 아래 산출물로 묶을 수 있어야 한다.

- 센서 허브와 모션 제어가 분리된 아키텍처 다이어그램
- 센서 기반 safety 데모 로그 또는 영상
- handheld teleop 데모
- 명령 흐름 다이어그램
- 한계와 리스크 설명

## 한 줄 요약

Phase 3의 목적은 센서를 붙이는 것이 아니라, STM32 센서 허브와 ESP32 모션 제어를 분리하고 그 사이에 safety와 공통 명령 경로를 세워서 Phase 4와 Phase 5로 자연스럽게 넘길 수 있는 구조를 만드는 것이다.
