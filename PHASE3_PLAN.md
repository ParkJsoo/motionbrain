# MotionBrain Phase 3 — 실행 계획

> **Phase 3 목표**: ESP32를 "센서 기반 폐루프 모션 제어 노드"로 진화시키고,
> ROS2 노드처럼 동작 가능한 메시지 계층을 미리 구축한다.

---

## 시스템 아키텍처 (확정)

```
[STM32F446 Nucleo — 센서 허브]        [ESP32 MotionBrain — 모션 제어]
  MPU-6050  I2C (PB8/PB9)               TB6612FNG × 3 → 5축 모터
  HC-SR04   TRIG:PA8 / ECHO:PC7  UART   Safety 상태머신 (IDLE/ARMED/FAULT)
  LCD 1602  I2C (PB8/PB9 공유)  ←────→  웹 UI / 시리얼 명령
  USART1    TX:PA9 / RX:PA10            UART2: TX=GPIO2 / RX=GPIO15
```

**ESP32 신규 핀 배정:**
| GPIO | 용도 | 변경 전 |
|------|------|---------|
| GPIO 2 | UART2 TX → STM32 | 미사용 |
| GPIO 15 | UART2 RX ← STM32 | TB6612 BIN1_3 (미사용, 와이어 제거) |

**Phase 4 확장 경로:**
```
STM32 역할 → Raspberry Pi로 이관 (아키텍처 동일, 보드만 교체)
RPi에서 ROS2 노드 실행 → ESP32는 Safety Gate + 모터 제어 유지
```

---

## 진행 순서

```
Phase 3-A → Phase 3-B → Phase 3-C + Phase 3-D (병행)
```

---

## Phase 3-A: Sensor Feedback Layer (STM32 기반)

> STM32가 센서를 담당하고 UART로 ESP32에 데이터 전달

### 목표

- STM32에서 MPU-6050 / HC-SR04 / LCD 1602 구동
- UART JSON 메시지로 ESP32에 센서 데이터 전달
- ESP32는 센서 데이터 수신 → Safety 판단에 활용

### STM32 측 구현 (STM32CubeIDE / HAL)

#### Step 1: 프로젝트 생성 및 핀 설정 (CubeMX)

- Board: NUCLEO-F446RE
- I2C1: PB8(SCL) / PB9(SDA) — MPU-6050 + LCD 1602 공유
- USART1: PA9(TX) / PA10(RX) — ESP32 통신용, 115200 bps
- USART2: PA2(TX) / PA3(RX) — ST-Link 디버그 출력용 (기본 활성화)
- GPIO OUT: PA8 — HC-SR04 TRIG
- GPIO IN: PC7 — HC-SR04 ECHO (5V tolerant)
- TIM2 또는 TIM3: HC-SR04 echo 시간 측정용 (μs 타이머)

#### Step 2: MPU-6050 드라이버 (HAL I2C)

- HAL_I2C_Mem_Read 기반 레지스터 직접 접근
- 100Hz 샘플링, Complementary Filter → roll / pitch 추정
- 진동 임계값 (가속도 벡터 크기) 계산

#### Step 3: HC-SR04 비차단 드라이버 (타이머 + 인터럽트)

- TIM 입력 캡처 (Input Capture) 방식
- TRIG: 10μs HIGH 펄스 출력
- ECHO: 상승 / 하강 엣지 타임스탬프 → 거리 계산
- 50ms 주기 측정

#### Step 4: LCD 1602 I2C 드라이버

- I2C 백팩 (PCF8574, 0x27) 통해 제어
- Line 1: 시스템 상태 수신 표시 (ESP32로부터 수신)
- Line 2: IMU roll/pitch 또는 거리 값

#### Step 5: UART JSON 송신 (STM32 → ESP32)

- 50ms 주기 상태 패킷 전송:
```json
{"type":"sensor","ts":12345,"roll":1.2,"pitch":-0.3,"yaw":45.0,"dist_cm":18}
```
- 이벤트 패킷 (임계값 초과 시 즉시):
```json
{"type":"event","code":"VIBRATION","val":3.2}
{"type":"event","code":"OBSTACLE","val":4}
```

### ESP32 측 구현

#### Step 6: UART2 수신 (GPIO2=TX, GPIO15=RX)

- `Serial2.begin(115200, SERIAL_8N1, 15, 2)`
- `src/input/stm32_bridge.h / .cpp` — JSON 파싱 → 센서 데이터 구조체
- `loop()`에서 `stm32Bridge.update()` 호출

#### Step 7: 센서 데이터 → Safety 통합

- IMU 진동 초과 → `systemState.transitionTo(FAULT)`
- 거리 < 5cm → `motionSequence.stop()`
- `GET /status` JSON에 `"imu"`, `"distance_cm"` 추가

#### Step 8: TB6612FNG BIN1_3 와이어 제거

- GPIO 15 → TB6612FNG #3 BIN1 점퍼 와이어 제거
- `PIN_BIN1_3` 상수 → `PIN_UNUSED` 로 변경
- GPIO 15 UART2 RX로 전환

### 완료 기준

- [ ] STM32 USART2(ST-Link)로 MPU-6050 roll/pitch 출력 확인
- [ ] HC-SR04 거리 측정 ±2cm 오차 확인
- [ ] LCD에 센서 값 표시 확인
- [ ] STM32 → ESP32 UART JSON 수신 확인 (시리얼 로그)
- [ ] 팔 흔들어서 진동 → ESP32 FAULT 자동 전환 확인
- [ ] 그리퍼 앞 장애물 → ESP32 시퀀스 자동 정지 확인

---

## Phase 3-B: Decision Layer 분리 + Web UI 고도화

> "입력이 직접 모터를 제어하지 않는다" — 3계층 아키텍처 완성

### 목표

- Input / Decision / Motion 3계층 분리
- 모든 입력 채널이 단일 Command 객체를 통해 Safety Gate 통과
- 웹 UI 시퀀스 빌더 구현

### 구현 항목

#### Step 1: Command 객체 정의

- `src/decision/command.h`
```cpp
enum class CommandSource { SERIAL, WEB, BLUETOOTH, INTERNAL };
enum class CommandType   { ARM, DISARM, STOP, JOINT_MOVE, SEQUENCE_ADD,
                           SEQUENCE_RUN, SEQUENCE_STOP, SEQUENCE_CLEAR,
                           LIGHT, SENSOR_FAULT };
struct Command {
  CommandSource source;
  CommandType   type;
  char          payload[64];  // JSON-like 파라미터
  uint32_t      timestampMs;
};
```

#### Step 2: CommandBus 구현

- `src/decision/command_bus.h / .cpp`
- `enqueue(Command)` / `dequeue(Command&)` — 링 버퍼 (capacity=8)
- `loop()`에서 CommandBus → Dispatcher 호출

#### Step 3: SafetyGate (Policy) 구현

- `src/decision/safety_gate.h / .cpp`
- 검사 항목: 시스템 상태 ARMED 여부, 센서 임계값, 타임아웃
- 통과 → Dispatcher → Motion Layer
- 거부 → WARN 로그 + 거부 이유 반환

#### Step 4: Dispatcher 구현

- `src/decision/dispatcher.h / .cpp`
- Command type → 해당 Motion Layer 메서드 매핑
- `RobotArm`, `MotionSequence`, `SearchLight` 참조

#### Step 5: 기존 입력 채널 마이그레이션

- `SerialCommand` → Command 생성 → CommandBus enqueue
- `WebServer` → Command 생성 → CommandBus enqueue
- 기존 직접 호출 제거

#### Step 6: 웹 UI 시퀀스 빌더

- `POST /sequence` 개선 — 빌더 UI에서 스텝 목록 한 번에 전송
- 시퀀스 빌더 카드: 관절 선택 → 방향 선택 → 속도/시간 설정 → 큐에 추가
- 현재 큐 목록 표시 (스텝 번호, 관절, 방향, 속도, 시간)
- 실행 중 진행 상태 표시 (현재 스텝 하이라이트)
- SPIFFS JSON으로 프리셋 저장 / 불러오기

### 완료 기준

- [ ] 시리얼 명령 → CommandBus → SafetyGate → Motion Layer 흐름 추적 가능
- [ ] FAULT 상태에서 웹 joint move 명령 → SafetyGate 거부 로그 확인
- [ ] 웹 UI 시퀀스 빌더에서 3스텝 추가 → 실행 → 진행 상태 표시 확인
- [ ] 프리셋 저장 후 ESP32 재부팅 → 불러오기 확인

---

## Phase 3-C: Closed-Loop Motion

> PWM 오픈루프 → IMU 피드백 폐루프 제어

### 목표

- IMU yaw 피드백으로 베이스 회전 목표 각도 자동 정지
- HC-SR04 기반 충돌 회피 인터록
- 엔코더 없는 환경의 추정 기법 구현 (시간 + IMU 결합)

### 구현 항목

#### Step 1: IMU yaw 추정

- Complementary Filter 확장 — yaw 누적 (자이로 적분)
- 자이로 드리프트 보정 (부팅 시 바이어스 캘리브레이션 5초)
- `ImuSensor::getYaw()` API

#### Step 2: 목표 각도 기반 베이스 회전 P 제어기

- `src/motion/angle_controller.h / .cpp`
- `AngleController::moveTo(float targetDeg, uint8_t maxSpeed)`
- 오차 = 목표각 - 현재 yaw → P 게인 적용 → PWM
- 오차 < 임계값 (예: ±3°) → 자동 정지

#### Step 3: MotionSequence 각도 명령 확장

- `MotionCommand` 확장 — `targetDeg` 옵션 필드 추가
- `sequence add base left 50 angle:90` 형태 시리얼 명령
- 시간 기반 / 각도 기반 혼용 가능

#### Step 4: HC-SR04 충돌 회피 인터록

- 거리 < 임계값 → `MotionSequence::pause()`
- Decision Layer에서 재개 정책 결정:
  - 자동 재개 (장애물 제거 후 2초 대기)
  - FAULT 전환 (충돌 위험 레벨 초과 시)
- 웹 UI에 인터록 상태 표시

### 완료 기준

- [ ] `sequence add base left 50 angle:45` → 45° 회전 후 자동 정지
- [ ] 베이스 회전 중 손으로 장애물 → 일시정지 → 제거 후 재개 확인
- [ ] IMU 부팅 캘리브레이션 로그 출력 확인

---

## Phase 3-D: Message Bridge (ROS2 Ready)

> Phase 4 RPi + ROS2 연동을 위한 통신 프로토콜 완성

### 목표

- JSON 메시지 스키마 동결
- Heartbeat / Watchdog 구현
- ROS2 매핑 문서 작성

### 구현 항목

#### Step 1: JSON 메시지 스키마 정의

```
명령 메시지 (Host → ESP32):
{ "type": "cmd", "id": 42, "joint": "base", "dir": "left", "speed": 50 }

상태 메시지 (ESP32 → Host, 50Hz):
{ "type": "state", "ts": 12345, "sys": "ARMED",
  "joints": {"gripper":0,"wrist":0,"elbow":0,"shoulder":0,"base":0},
  "imu": {"roll":1.2,"pitch":-0.3,"yaw":45.0},
  "distance_cm": 18.5 }

이벤트 메시지 (ESP32 → Host, 비주기):
{ "type": "event", "ts": 12345, "code": "FAULT", "reason": "vibration" }
```

#### Step 2: MessageBridge 클래스

- `src/network/message_bridge.h / .cpp`
- `init(Stream* stream)` — 시리얼 또는 다른 스트림
- `sendState()` — 50Hz 주기 상태 전송
- `sendEvent(code, reason)` — 이벤트 즉시 전송
- `parseCommand(json)` → Command 생성 → CommandBus enqueue

#### Step 3: Heartbeat + Watchdog

- ESP32 → Host: 100ms 주기 `{"type":"hb","ts":n}` 전송
- Host → ESP32: 1초 내 heartbeat 미수신 → 자동 DISARM
- ROS2 노드 단절 시 안전 보장

#### Step 4: ROS2 매핑 문서

- `docs/ros2_mapping.md`
- ESP32 메시지 ↔ ROS2 표준 메시지 대응표:
  - `state.joints` → `sensor_msgs/JointState`
  - `state.imu` → `sensor_msgs/Imu`
  - `cmd.joint` → custom `motionbrain_msgs/JointCommand`
  - `event.FAULT` → `std_msgs/String` on `/motionbrain/fault`

### 완료 기준

- [ ] 50Hz 상태 JSON 시리얼 출력 확인
- [ ] Python 스크립트로 heartbeat 수신 → 1초 중단 → ESP32 DISARM 전환 확인
- [ ] `docs/ros2_mapping.md` 작성 완료

---

## 전체 마일스톤

| 서브 Phase | 예상 소요 | 의존성 | 포트폴리오 핵심 어필 |
|---|---|---|---|
| 3-A | 1~2주 | 없음 (독립) | 센서 통합, 안전 시스템 |
| 3-B | 1~2주 | 3-A | 계층 분리 아키텍처, 웹 UI |
| 3-C | 1~2주 | 3-A, 3-B | 폐루프 제어, 엔코더 없는 추정 |
| 3-D | 1주 | 3-B | 분산 시스템 통신 설계 |

---

## Phase 4 연계 계획 (참고)

### Phase 3 완료 시 구조

```
[STM32F446 Nucleo]                    [ESP32 MotionBrain]
  MPU-6050 (I2C)                        TB6612FNG × 3
  HC-SR04  (GPIO)        UART           Safety Gate
  LCD 1602 (I2C)        ←────→         웹 UI / 시리얼
  센서 데이터 처리                        Decision Layer
```

### Phase 4: STM32 → Raspberry Pi 이관

```
[Raspberry Pi 4]                      [ESP32 MotionBrain]
  ROS2 노드                              TB6612FNG × 3
    ├─ sensor_node  (STM32 역할 대체)     Safety Gate (ESP32에 잔존)
    ├─ motion_planner_node               모터 실시간 제어
    ├─ camera_node  (ESP32-CAM)  UART
    └─ ai_node      (LLM 연계)  ←────→  MessageBridge (동일 프로토콜)
  micro-ROS → STM32 직결도 가능
```

**이관 원칙**:
- UART JSON 프로토콜은 Phase 3에서 동결 → RPi도 동일 포맷 그대로 사용
- Safety Gate는 항상 ESP32에 잔존 (RPi 다운돼도 모터 안전 보장)
- STM32는 micro-ROS 노드로 전환 가능 (Phase 4 선택지)

---

## 포트폴리오 어필 포인트

| 항목 | 증명 위치 | 면접 소재 |
|------|---------|---------|
| 멀티 MCU 아키텍처 설계 | Phase 3-A STM32+ESP32 | "역할 분리 기준과 UART 프로토콜 설계" |
| STM32 HAL/CubeMX | Phase 3-A STM32 코드 | "타이머 입력 캡처로 HC-SR04 비차단 구현" |
| 실시간 임베디드 안전 | Safety Gate, Watchdog | "UART 단절 시 자동 DISARM 흐름" |
| 폐루프 제어 | Phase 3-C IMU P제어기 | "엔코더 없는 환경의 추정 기법과 한계" |
| ROS2 연계 준비 | Phase 3-D 프로토콜 | "Phase 4에서 보드만 교체하면 되는 이유" |
| 계층 분리 아키텍처 | Phase 3-B Decision Layer | "입력이 직접 모터를 제어하지 않는 이유" |

**한 줄 요약**:
> "STM32를 센서 허브로, ESP32를 실시간 모션 제어 노드로 분리하는 멀티 MCU 아키텍처를 설계했고,
> Phase 4에서 STM32 자리에 Raspberry Pi + ROS2를 이관할 수 있도록 UART 프로토콜을 미리 설계했습니다."
