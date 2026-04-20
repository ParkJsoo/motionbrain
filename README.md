# MotionBrain

ESP32 기반 5축 로봇팔 제어 시스템에서 출발해, STM32 센서 허브, ESP32-CAM 비전 입력, Raspberry Pi + ROS2 + AI 상위 제어까지 확장하는 로봇 개발 포트폴리오 프로젝트.

## 프로젝트 목표

이 프로젝트의 최종 목표는 단순히 모터를 움직이는 데서 끝나지 않는다.

- ESP32로 실시간 모터 제어와 안전 상태 관리를 구현한다.
- STM32로 센서 허브를 구성해 피드백 계층을 분리한다.
- ESP32-CAM으로 영상 입력 채널을 추가한다.
- Raspberry Pi + ROS2 + AI로 상위 판단과 메시지 계층을 연결한다.
- 전체 구조를 설명 가능한 형태로 정리해 취업용 포트폴리오로 완성한다.

핵심 철학은 다음 한 줄로 요약된다.

> 입력 -> 판단 -> 상태 -> 움직임 -> 피드백

## 현재 상태

### 완료된 것

- ESP32 5축 모터 제어 커널 구현
- 상태 머신: `BOOT -> IDLE -> ARMED -> FAULT`
- 시리얼 명령 인터페이스
- Wi-Fi AP + 웹 UI
- 관절 추상화 계층 `RobotArm`
- 비차단 시퀀스 큐 `MotionSequence`
- `SearchLight` 제어
- `TB6612FNG x3` + `M1~M5` 실물 연결 및 정상 동작 테스트 완료
- STM32 `MotionBrainSensor` 프로젝트에서 `MPU-6050` 통신 검증 완료

### 현재 핵심 미완료

- STM32 센서 허브에서 `HC-SR04`와 UART 센서 브리지 구현
- ESP32에 `stm32_bridge`와 센서 기반 safety 연동 추가
- 입력/판단/모션 계층 분리
- IMU 기반 베이스 상대각 제어
- ESP32-CAM 영상 스트리밍 및 비전 입력 연동
- Raspberry Pi + ROS2 + AI 연동

## 현재 아키텍처

### 이미 구현된 계층

```text
Serial / Web UI
    ->
SystemState + MotorControl
    ->
RobotArm + MotionSequence
    ->
TB6612FNG x3
    ->
5-axis DC motors
```

### 목표 아키텍처

```text
[STM32 Sensor Hub]
  MPU-6050
  HC-SR04
  UART sensor stream
        ->
[ESP32 Motion Controller]
  Safety state machine
  Command processing
  Motion execution
        <-
[ESP32-CAM Vision Node]
  Camera streaming
  Visual target input
        ->
[Raspberry Pi + ROS2 + AI]
  High-level planning
  Vision processing
  Message bridge
  Portfolio demo orchestration
```

## 하드웨어 구성

### 현재 핵심 구성품

- `ESP32 DevKit`
- `ESP32-CAM`
- `TB6612FNG x3`
- 5축 DC 모터
- `STM32 B-F446E-96B01A`
- `MPU-6050 (GY-521)`
- `HC-SR04`
- `1602 LCD + I2C backpack`
- 점퍼선, 전원 모듈, 브레드보드

### STM32 센서 허브 현재 확인 사실

- 보드: `B-F446E-96B01A`
- `MPU-6050` 응답 확인 완료
- 실제 I2C 매핑:
  - `D15 = PB10 = I2C2_SCL`
  - `D14 = PC12 = I2C2_SDA`
- UART 센서 송신 기준:
  - `D1 = PD5 = USART2_TX`
- `HC-SR04` 기준 핀:
  - `D2 = PD4 = TRIG`
  - `D3 = PC8 = ECHO`
- 자이로 바이어스 캘리브레이션 및 200Hz 샘플링 동작 확인

STM32 센서 프로젝트 경로:

- `/Users/jeongsoopark/STM32CubeIDE/workspace_2.1.1/MotionBrainSensor`

## 소프트웨어 구성

현재 ESP32 메인 진입점은 [src/main.cpp](/Users/jeongsoopark/develop/arduino/motionbrain/src/main.cpp:1)이며, 주요 모듈은 다음과 같다.

- `system/`: 상태 머신과 시스템 초기화
- `motor/`: `TB6612FNG` 기반 모터 제어
- `motion/`: `RobotArm`, `MotionSequence`
- `input/`: 시리얼 명령 처리
- `network/`: Wi-Fi AP, 웹 서버
- `peripheral/`: `SearchLight`
- `debug/`: 로그 출력

현재 웹 서버 라우트:

- `/status`
- `/command`
- `/motor`
- `/joint`
- `/base`
- `/sequence`
- `/light`

## 개발 환경

### ESP32

- PlatformIO
- `esp32dev`
- Arduino framework

### STM32

- STM32CubeIDE
- HAL / CubeMX

### 향후 상위 제어

- Raspberry Pi
- ROS2
- 카메라/비전 처리
- AI 연동

## 빠른 시작

### ESP32 빌드

```bash
pio run
```

### ESP32 업로드

```bash
pio run -t upload
```

### ESP32 시리얼 모니터

```bash
pio device monitor
```

### 기본 사용 흐름

1. ESP32 부팅
2. Wi-Fi AP `MotionBrain-AP` 접속 또는 USB 시리얼 연결
3. `arm`
4. `joint`, `motor`, `sequence`, `light` 명령 사용
5. 상대각 회전이 필요하면 `base angle left 45 40` 같은 명령 사용
6. 필요 시 `stop` 또는 `disarm`

## 문서 구조

### 현재 기준으로 유지하는 문서

- [README.md](/Users/jeongsoopark/develop/arduino/motionbrain/README.md:1): 프로젝트 개요와 현재 상태
- [로드맵.md](/Users/jeongsoopark/develop/arduino/motionbrain/로드맵.md:1): 포트폴리오 기준 전체 단계
- [PHASE3_PLAN.md](/Users/jeongsoopark/develop/arduino/motionbrain/PHASE3_PLAN.md:1): 현재 핵심 작업인 센서/브리지 계획
- [MESSAGE_INTERFACE.md](/Users/jeongsoopark/develop/arduino/motionbrain/MESSAGE_INTERFACE.md:1): 시리얼/HTTP/status 메시지 경계 정리
- [PIN_MAP.md](/Users/jeongsoopark/develop/arduino/motionbrain/PIN_MAP.md:1): ESP32 모터 핀 연결
- [.codex/START_HERE.md](/Users/jeongsoopark/develop/arduino/motionbrain/.codex/START_HERE.md:1): 다음 세션용 빠른 복구 메모

### 현재 기준에서 제외하는 문서

- `/arduino/doc/*`: 초기 조사 자료
- `.omc/*`: 이전 OMC/Claude 작업 흔적

## 다음 우선순위

1. STM32에서 `MPU-6050 + HC-SR04` 센서 패킷 송신
2. ESP32에서 센서 수신 및 `distance -> stop`, `vibration -> FAULT`
3. `CommandBus / SafetyGate / Dispatcher` 구조 분리
4. 베이스 상대각 폐루프 제어
5. `ESP32-CAM` 스트리밍과 비전 입력 연결
6. Raspberry Pi + ROS2 + AI 연동
7. 데모 시나리오, 문서, 포트폴리오 정리

## 포트폴리오 관점에서의 핵심 어필 포인트

- 멀티 MCU 역할 분리 설계
- 안전 상태 머신 기반 모터 제어
- 센서 피드백을 통한 폐루프 제어 확장
- 웹/시리얼/센서/카메라/ROS2까지 이어지는 입력 계층 설계
- 카메라 기반 인식과 로봇 동작 연결
- 단순 동작이 아니라 구조와 진화 경로를 설명 가능한 프로젝트

## 라이선스

개인 연구, 학습, 제작, 포트폴리오 프로젝트.
