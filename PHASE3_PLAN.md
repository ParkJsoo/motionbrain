# MotionBrain Phase 3 Plan

Phase 3는 MotionBrain을 "움직이는 ESP32 프로젝트"에서 "센서 피드백과 판단 계층을 가진 로봇 제어 시스템"으로 끌어올리는 단계다.

## Phase 3의 역할

Phase 3에서 만들고 싶은 것은 다음 네 가지다.

- STM32 센서 허브
- ESP32 센서 수신 및 safety 연동
- 입력/판단/모션 계층 분리
- 폐루프 제어와 향후 ROS2 연동을 위한 메시지 기반 구조

## 확정된 사실

아래 항목은 이미 검증되었거나 현재 코드/보드에서 확정된 사실이다.

### ESP32 측

- 5축 모터 제어 동작 확인 완료
- `RobotArm`, `MotionSequence`, `SearchLight` 구현 완료
- 웹/시리얼 입력 채널 존재

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
  - `TX = PA9`
  - `RX = PA10`
- `USART2`
  - `TX = PD5`
  - `RX = PD6`
- `HC-SR04` 예약 핀
  - `TRIG = PA8`
  - `ECHO = PC7`
- `TIM3`
  - 현재 5ms 주기 기반 샘플 타이머로 사용 중

## 설계 원칙

- Phase 3의 첫 목적은 "센서값 표시"가 아니라 "센서 기반 safety와 구조 분리"다.
- 확정 사실과 예정 구현을 분리해서 관리한다.
- 최소 경로가 먼저다. LCD나 복잡한 양방향 프로토콜보다 `STM32 -> ESP32` 단방향 센서 스트림이 우선이다.
- `MPU-6050` yaw는 절대 방향 기준이 아니라 상대 회전 보조용으로 취급한다.
- ROS2 확장 가능성을 고려하되, 지금 당장 ROS2를 구현 목표로 삼지 않는다.
- Phase 4에서는 `ESP32-CAM`을 비전 입력 노드로 추가하고, 상위 노드에서 vision/AI를 처리하는 구조를 기본 방향으로 둔다.

## Phase 3 구성

### 3-A. Sensor Feedback Layer

목표:

- STM32가 센서를 읽는다.
- ESP32가 그 값을 받아 safety에 반영한다.
- 최소한의 피드백 루프를 만든다.

#### 3-A MVP 범위

STM32:

- `MPU-6050` roll/pitch/gyro 값 생성
- `HC-SR04` 거리 측정 추가
- 라인 단위 UART 패킷 송신

ESP32:

- UART 센서 수신기 `stm32_bridge`
- 최근 센서 상태 저장
- `distance < threshold -> stop`
- `abnormal vibration -> FAULT`

#### 3-A UART 전략

첫 구현은 단방향이 우선이다.

```text
STM32 USART1_TX -> ESP32 RX
```

이유:

- MVP에서 ESP32가 STM32에 명령을 줄 필요가 없다.
- 배선과 디버깅을 단순화할 수 있다.
- 양방향은 이후 `3-D Message Bridge`에서 추가해도 늦지 않다.

ESP32 수신 핀은 현재 미사용 GPIO를 기준으로 선정한다. 현 시점 기준 후보는 `GPIO15`이며, 실제 전환 시 [PIN_MAP.md](/Users/jeongsoopark/develop/arduino/motionbrain/PIN_MAP.md:1)와 배선을 함께 갱신한다.

#### 3-A 패킷 초안

초기 버전은 line-delimited JSON으로 충분하다.

```json
{"type":"sensor","ts":12345,"roll":-2.1,"pitch":1.4,"gyro_x":0.2,"gyro_y":-0.1,"accel":0.98,"dist_cm":18.3}
```

이벤트 패킷은 필요 시 별도로 추가한다.

```json
{"type":"event","code":"VIBRATION","value":3.2}
{"type":"event","code":"OBSTACLE","value":4.7}
```

#### 3-A 완료 기준

- STM32에서 `MPU-6050` + `HC-SR04` 값을 주기적으로 송신
- ESP32가 센서 패킷 수신 및 파싱
- 거리 임계값으로 시퀀스 정지 또는 모터 정지
- 진동 조건으로 `FAULT` 전환
- 수신 센서 상태를 `/status` 또는 로그에서 확인 가능

### 3-B. Decision Layer

목표:

- 입력이 직접 모터를 때리지 않게 만든다.
- 구조를 포트폴리오 수준으로 정리한다.

구성:

- `Command`
- `CommandBus`
- `SafetyGate`
- `Dispatcher`

핵심 변화:

- `SerialCommand`와 `WebServer`는 직접 실행 대신 명령 생성만 담당
- 실제 실행은 `SafetyGate`를 통과한 뒤에만 가능

완료 기준:

- 모든 입력 채널이 동일한 명령 경로 사용
- `FAULT`나 센서 차단 상태에서 명령 거부 이유가 로그로 설명됨

### 3-C. Closed-Loop Motion

목표:

- 센서를 실제 동작 제어에 연결한다.

1차 대상:

- 베이스 회전 상대각 제어

예상 구현:

- `AngleController`
- 목표각 대비 오차 기반 제어
- 시퀀스 명령에 angle 옵션 추가

주의:

- `MPU-6050` 단독 yaw는 드리프트가 있기 때문에 장기 절대방향 제어용이 아니다.
- 이 단계는 "짧은 상대 회전 자동 정지"를 목표로 한다.

완료 기준:

- 예: `base left angle:45` 후 자동 정지
- 센서값과 실제 동작이 로그로 설명 가능

### 3-D. Message Bridge

목표:

- 향후 Raspberry Pi + ROS2 연결 전, 메시지 인터페이스를 정리한다.

범위:

- 센서/상태/이벤트 메시지 구조 확정
- heartbeat/watchdog 초안
- ROS2로 올렸을 때 어떤 메시지로 바뀔지 매핑 기준 정리

완료 기준:

- ESP32 내부와 상위 호스트 간 메시지 구조가 문서화됨
- 이후 RPi/ROS2 연결 시 프로토콜을 다시 뜯어고칠 필요가 적음

## 구현 우선순위

1. STM32 `HC-SR04` 추가
2. STM32 UART 센서 송신
3. ESP32 `stm32_bridge`
4. 센서 기반 safety 연동
5. `CommandBus / SafetyGate / Dispatcher`
6. 베이스 상대각 제어
7. 메시지 인터페이스 정리

## Phase 4와의 연결

Phase 3를 끝내는 이유는 단순히 기능을 늘리기 위해서가 아니다. Phase 4에서 `ESP32-CAM + Raspberry Pi + ROS2 + AI`를 붙였을 때 구조가 자연스럽게 이어지게 만들기 위해서다.

예상 구조:

```text
[STM32 Sensor Hub] -> [ESP32 Motion Controller] <-> [Raspberry Pi + ROS2 + AI]
                         ^
                         |
                   [ESP32-CAM]
```

이때 역할은 다음처럼 나눈다.

- STM32: 센서 수집, 저수준 센서 전처리
- ESP32: 실시간 제어, safety, 모션 실행
- ESP32-CAM: 영상 스트리밍, 시각 입력
- Raspberry Pi: 고수준 판단, 비전/AI 처리, ROS2 노드, 데모 오케스트레이션

## 포트폴리오 관점의 의미

Phase 3는 취업용 포트폴리오에서 가장 설명력이 큰 단계가 될 가능성이 높다.

- 멀티 MCU 역할 분리
- 센서 허브와 모션 제어 분리
- 안전 인터록 설계
- 계층 기반 명령 처리
- 폐루프 제어의 한계와 현실적인 타협

면접에서 중요한 것은 "무엇을 만들었나"보다 "왜 이런 구조를 택했고 어떤 한계를 알고 있었나"다. Phase 3는 그 설명 재료를 만드는 단계다.
