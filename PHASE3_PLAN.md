# MotionBrain Phase 3 — 실행 계획

> **Phase 3 목표**: ESP32를 "센서 기반 폐루프 모션 제어 노드"로 진화시키고,
> ROS2 노드처럼 동작 가능한 메시지 계층을 미리 구축한다.

---

## 진행 순서

```
Phase 3-A → Phase 3-B → Phase 3-C + Phase 3-D (병행)
```

---

## Phase 3-A: Sensor Feedback Layer

> "보고 듣는 ESP32" — 센서 입력을 시스템 상태에 통합

### 목표

- MPU-6050 (IMU), HC-SR04 (초음파), LCD 1602 통합
- 센서 이상 → 자동 FAULT 트리거
- 모터 없는 환경에서도 가시적 피드백 확보

### 구현 항목

#### Step 1: MPU-6050 (GY-521) IMU 드라이버

- `src/peripheral/imu_sensor.h / .cpp`
- I2C 통신 (SDA/SCL 핀 — 기존 핀맵 확인 후 결정)
- 100Hz 샘플링, Complementary Filter로 roll / pitch 추정
- 진동 임계값 초과 시 `onFaultTrigger()` 콜백

#### Step 2: HC-SR04 초음파 거리 센서

- `src/peripheral/distance_sensor.h / .cpp`
- 비차단 방식 (echo timeout 기반, `update()` 패턴)
- TRIG / ECHO 핀 설정
- 전방 장애물 감지 → `motionSequence.pause()` 또는 FAULT 트리거

#### Step 3: 16x2 LCD I2C 상태 표시

- `src/peripheral/lcd_display.h / .cpp`
- LiquidCrystal_I2C 라이브러리 활용
- Line 1: 시스템 상태 (IDLE / ARMED / FAULT)
- Line 2: 현재 시퀀스 진행률 또는 센서 값

#### Step 4: 센서 → SystemStateManager 통합

- `src/system/system_init.cpp` — 센서 FAULT 조건 등록
- IMU 과진동 → `transitionTo(SystemState::FAULT)`
- HC-SR04 < 5cm → MotionSequence 강제 정지

#### Step 5: 시리얼 / 웹 status에 센서 데이터 노출

- `status` 시리얼 명령 — IMU roll/pitch, 전방 거리 추가
- `GET /status` JSON — `"imu": {...}, "distance_cm": n` 추가

### 완료 기준

- [ ] MPU-6050 I2C 통신 확인, roll/pitch 로그 출력
- [ ] HC-SR04 거리 측정 오차 ±2cm 이내
- [ ] LCD에 IDLE/ARMED/FAULT 상태 실시간 표시
- [ ] 팔 흔들어서 진동 → FAULT 자동 전환 확인
- [ ] 그리퍼 앞 장애물 → 시퀀스 자동 정지 확인

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

```
[Raspberry Pi 4]
  └─ ROS2 노드
      ├─ serial_bridge_node   ← MessageBridge 프로토콜 그대로 수신
      ├─ motion_planner_node  ← Decision Layer를 RPi로 위탁 가능
      ├─ camera_node          ← ESP32-CAM 스트림
      └─ ai_node              ← 자연어 → 시퀀스 변환 (Phase 4 목표)
           │ USB Serial
[ESP32 MotionBrain]
  ├─ Safety Gate (항상 ESP32에 잔존)
  ├─ Motor Control (실시간)
  └─ Sensor Feedback (실시간)
```

**핵심 설계 원칙**: RPi가 판단을 위탁받더라도,  
**Safety Gate는 항상 ESP32에 잔존**하여 하드웨어 레벨 안전 보장.

---

## 포트폴리오 어필 한 줄

> "오픈루프 PWM 제어기를 센서 피드백 기반 폐루프 + ROS2 호환 메시지 노드로 진화시켰고,
> 안전 계층을 ESP32에 잔존시키는 분산 아키텍처를 직접 설계했습니다."
