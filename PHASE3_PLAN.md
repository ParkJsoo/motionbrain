# MotionBrain Phase 3 Plan

Phase 3는 MotionBrain을 "움직이는 ESP32 프로젝트"에서 "센서 피드백과 판단 계층을 가진 로봇 제어 시스템"으로 끌어올리는 단계다.

이 문서는 아이디어 메모가 아니라 실제 구현 순서와 완료 기준을 정리하는 실행 계획 문서로 유지한다.

## Phase 3의 목표

Phase 3에서 반드시 만들어야 하는 결과는 다음 네 가지다.

- STM32 센서 허브가 `MPU-6050 + HC-SR04`를 읽고 ESP32로 보낸다.
- ESP32가 센서를 받아 safety에 반영한다.
- 입력 채널이 직접 모터를 때리지 않고 공통 명령 경로를 지난다.
- 베이스 회전에 한정한 최소 폐루프 제어를 만든다.

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
- 웹 라우트:
  - `/status`
  - `/command`
  - `/motor`
  - `/joint`
  - `/sequence`
  - `/light`
- 현재 구조에서는 `SerialCommand`와 `MotionBrainWebServer`가 비교적 직접 실행 경로를 가진다.
- 현재 `/status`는 시스템 상태와 모터 상태 중심이며 센서 health는 포함하지 않는다.

### STM32 측

- 보드: `B-F446E-96B01A`
- 프로젝트 경로:
  - `/Users/jeongsoopark/STM32CubeIDE/workspace_2.1.1/MotionBrainSensor`
- `MPU-6050` 통신 확인 완료
- `WHO_AM_I = 0x68` 확인 완료
- 자이로 바이어스 캘리브레이션 추가 완료
- 200Hz 샘플링 동작 확인 완료

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
- `MPU-6050` yaw는 절대 방향 기준이 아니라 상대 회전 보조용으로만 취급한다.
- `ESP32-CAM`과 `RPi/ROS2 + AI`는 Phase 3의 구현 대상이 아니라, Phase 3 결과를 받는 다음 단계다.

## 현재 코드 기준 삽입 지점

### `src/main.cpp`

현재 `setup()`과 `loop()`는 아래 흐름으로 묶여 있다.

```text
systemState.update()
motorControl.update()
motionSequence.update()
serialCommand.update()
wifiAP.update()
webServer.update()
```

Phase 3에서는 여기에 최소한 다음 계층이 추가돼야 한다.

- `stm32_bridge.update()`
- `safety_monitor.update()`
- 이후 `command_bus` 또는 `dispatcher.update()`

권장 순서는 다음과 같다.

```text
systemState.update()
stm32_bridge.update()
safety_monitor.update()
dispatcher/update command processing
motorControl.update()
motionSequence.update()
serial/web update
wifiAP.update()
webServer.update()
```

### `src/input/serial_command.*`

- 현재는 시리얼 입력을 읽고 바로 명령을 실행한다.
- Phase 3-B 이후에는 "파싱 -> Command 생성"까지만 담당하도록 줄인다.

### `src/network/web_server.*`

- 현재는 HTTP 요청이 직접 상태 전환, 모터, 관절, 시퀀스 실행으로 이어진다.
- Phase 3-B 이후에는 웹도 동일한 `Command` 경로를 사용해야 한다.
- `/status`는 센서 상태를 포함하도록 확장해야 한다.

### `src/system/system_init.*`

- 현재는 타임아웃 중심 안전 상태 머신만 있다.
- 센서 기반 차단 이유를 표현할 방법이 아직 없다.
- 최소한 로그와 `/status`에 safety reason을 남길 경로가 필요하다.

## Phase 3 범위

### 이번 단계에 포함

- `MPU-6050 + HC-SR04 + UART` 센서 스트림
- ESP32 센서 수신
- 거리/진동 기반 safety
- 공통 명령 경로 분리
- 베이스 상대각 제어
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

1. `MPU-6050`에서 safety와 폐루프에 필요한 값만 정리한다.
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

## 구현 순서

1. STM32 `HC-SR04` 추가
2. STM32 UART JSON 송신
3. ESP32 `stm32_bridge`
4. ESP32 `safety_monitor`
5. `/status` 센서 상태 노출
6. `Dispatcher` 도입
7. `SerialCommand`를 `Command` 생산자로 전환
8. `WebServer`를 같은 경로로 전환
9. 베이스 상대각 제어 추가
10. 메시지 인터페이스 정리

## 검증 체크리스트

### Gate 1. 센서 스트림

- STM32 UART에서 JSON 라인이 안정적으로 나온다.
- ESP32가 1분 이상 패킷 누락 없이 수신한다.
- 센서 끊김 시 stale 감지가 동작한다.

### Gate 2. Safety

- 장애물 접근 시 모터 또는 시퀀스가 멈춘다.
- 진동 조건에서 `FAULT` 전환이 일어난다.
- safety 해제 전에는 재실행이 거부된다.

### Gate 3. 명령 경로 통합

- 시리얼과 웹이 같은 경로를 사용한다.
- 거부 사유가 일관되게 보인다.

### Gate 4. 폐루프 데모

- 베이스 상대각 회전이 재현 가능하다.
- 종료 이유가 `target reached`, `timeout`, `sensor block` 중 하나로 설명 가능하다.

## 현재 핵심 리스크

- ESP32 수신 핀을 잘못 고르면 boot strapping 이슈가 생길 수 있다.
- `MPU-6050` 진동/각속도 임계값은 실측 튜닝이 필요하다.
- `MPU-6050` 단독 yaw 적분은 장기 드리프트가 커서 Phase 3-C 범위를 짧은 회전에 제한해야 한다.
- Command 구조를 너무 크게 설계하면 실제 구현보다 구조 논의만 길어질 수 있다.

## Phase 4 진입 조건

Phase 4는 아래 조건이 충족된 뒤 시작한다.

- STM32 센서 스트림이 안정적이다.
- ESP32 safety가 센서 기반으로 동작한다.
- 시리얼/웹 입력이 공통 명령 경로를 쓴다.
- `/status`에서 센서 health와 block reason을 확인할 수 있다.
- 베이스 상대각 회전 데모가 가능하다.

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
- 베이스 상대각 제어 데모
- 명령 흐름 다이어그램
- 한계와 리스크 설명

## 한 줄 요약

Phase 3의 목적은 센서를 붙이는 것이 아니라, STM32 센서 허브와 ESP32 모션 제어를 분리하고 그 사이에 safety와 공통 명령 경로를 세워서 Phase 4와 Phase 5로 자연스럽게 넘길 수 있는 구조를 만드는 것이다.
