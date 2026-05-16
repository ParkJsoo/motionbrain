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
- STM32 `MotionBrainSensor` 프로젝트에서 `MPU-6050 + HC-SR04 + UART` 센서 스트림 bench 검증 완료
- ESP32 `stm32_bridge`, `safety_monitor`, `Dispatcher`, `SafetyGate` 추가
- 시리얼/HTTP 입력을 공통 `Command` 경로로 통합
- 베이스 상대각 제어 1차 구현 완료 (`base angle ...`, `POST /base`) 단, 현재는 optional 실험 기능
- 최근 이벤트 로그와 `GET /events` API 추가
- `sensor sim ...` 기반 bench simulation 경로 추가
- 유선 handheld teleop v1 골격 구현 완료
  - ESP32 `teleop_adapter`
  - STM32 `APP_MODE_TELEOP_REMOTE`
  - `teleop` JSON frame / `deadman` / freshness timeout / LED edge / primitive mixer

### 현재 핵심 미완료

- 최종 부품 배치와 배선표 확정
- 최종 실장 상태에서 센서 스트림 안정성 재검증
- handheld remote provisional 버튼 핀과 UART 배선 확정
- teleop 실기에서 `reach/lift/twist` 부호와 비중 조정
- `HC-SR04` 기반 obstacle safety 최종 배치 검증
- Phase 4 진입 전 host-side 상태/이벤트 소비 경계 최종 확정
- ESP32-CAM 영상 스트리밍 및 비전 입력 연동
- Raspberry Pi + ROS2 + AI 연동

## 현재 아키텍처

### 이미 구현된 계층

```text
[STM32 Sensor Hub]
  MPU-6050
  HC-SR04
  UART sensor stream
        ->
[ESP32 Motion Controller]
  Stm32Bridge
  SafetyMonitor
  TeleopAdapter
  Dispatcher + SafetyGate
  AngleController
  RobotArm + MotionSequence
  EventLog
        ->
TB6612FNG x3
        ->
5-axis DC motors
```

### 목표 아키텍처

```text
[STM32 Sensor / Teleop Layer]
  HC-SR04 safety input
  GY-521 handheld remote
  UART sensor/teleop stream
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
- 현재 `MPU-6050 (GY-521)`은 handheld remote 입력용으로 쓰는 방향이다.
- 따라서 base 상대각 폐루프나 본체 vibration fault를 활성 데모로 다시 잡으려면 별도 본체 센서가 필요하다.
- 실제 I2C 매핑:
  - `D15 = PB10 = I2C2_SCL`
  - `D14 = PC12 = I2C2_SDA`
- UART 센서 송신 기준:
  - `D1 = PD5 = USART2_TX`
- `HC-SR04` 기준 핀:
  - `D2 = PD4 = TRIG`
  - `D3 = PC8 = ECHO`
- 자이로 바이어스 캘리브레이션 및 200Hz 샘플링 동작 확인
- `HC-SR04` 거리 측정 수신 확인
- UART JSON 센서 송신 확인
- `STM32 -> ESP32` 센서 브리지 bench 수신 확인

STM32 센서 프로젝트 경로:

- `firmware/stm32/MotionBrainSensor`

## 소프트웨어 구성

현재 ESP32 메인 진입점은 [src/main.cpp](/Users/jeongsoopark/develop/arduino/motionbrain/src/main.cpp:1)이며, 주요 모듈은 다음과 같다.

- `system/`: 상태 머신과 시스템 초기화
- `motor/`: `TB6612FNG` 기반 모터 제어
- `motion/`: `RobotArm`, `MotionSequence`
- `bridge/`: STM32 센서 수신과 simulation
- `safety/`: 센서 기반 차단과 fault latch
- `control/`: `Dispatcher`, `SafetyGate`, `AngleController`, `EventLog`
- `input/`: 시리얼 명령 처리
- `network/`: Wi-Fi AP, 웹 서버
- `peripheral/`: `SearchLight`
- `debug/`: 로그 출력

현재 웹 서버 라우트:

- `/status`
- `/events`
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

### STM32 CLI Helpers

추가된 스크립트:

- `tools/stm32_build.sh`: STM32CubeIDE headless build
- `tools/stm32_upload.sh`: STM32CubeProgrammer CLI로 ST-LINK 업로드
- `tools/stm32_build_upload.sh`: 빌드 후 바로 업로드

현재 macOS에서는 `STM32_Programmer_CLI`를 arm64로 직접 실행하면 Qt/neon 오류가 나므로, 스크립트에서 자동으로 `arch -x86_64`를 사용한다.

예시:

```bash
tools/stm32_build.sh
tools/stm32_upload.sh
tools/stm32_build_upload.sh
```

업로드는 ST-LINK가 연결되어 있어야 한다. 현재 기준 미연결 상태에서는 `Error: No debug probe detected.` 가 정상이다.

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

### Host-Side 상태 감시

```bash
python3 tools/motionbrain_watch.py --host 192.168.4.1 --interval 1.0
```

이 스크립트는 `GET /status`, `GET /events` 를 주기적으로 읽어 `state`, `blockReason`, `faultReason`, `baseAngle`, `teleop` 상태를 한 줄로 보여준다.

### Wired Handheld Teleop Bring-Up

유선 handheld remote v1은 STM32에서 `teleop` JSON frame을 만들고, ESP32가 별도 `teleop_adapter`로 수신하는 구조다.

현재 기준 배선:

- `STM32 USART2 TX = PD5 = D1` -> `ESP32 GPIO34 = Serial1 RX`
- `STM32 GND` -> `ESP32 GND`

현재 ESP32 teleop 수신 기준:

- `Serial1`
- `RX only`
- `GPIO34`
- frame timeout: 약 `200ms`
- 권장 frame rate: 약 `25Hz`

현재 STM32 teleop 버튼 매핑:

- `PE4 = D10`: `deadman`
- `PB4 = D9`: `LED toggle`
- `PE2 = D13`: `grip open`
- `PE6 = D11`: `grip close`

현재 버튼 배선 기준:

- 각 버튼은 한쪽만 해당 STM32 GPIO로 연결
- 버튼 다른 쪽은 `STM32 GND` 공통 rail로 연결
- 내부 pull-up + active-low 기준이므로, 미입력은 HIGH, 눌리면 LOW다

주의:

- 위 버튼 핀은 2026-04-27 실기 기준으로 확인 완료된 source of truth다.
- 이 보드의 Arduino `A0~A5` 헤더는 `PA0/PA1/PA4/PB0`가 아니다.
- `D2/D3`는 현재 `HC-SR04`가 쓰므로 버튼에서는 제외한다.
- `D1`은 teleop UART TX, `D14/D15`는 I2C2라서 버튼에서 제외한다.
- `PE3(D8)`는 선을 뽑아도 `deadman=YES`가 유지돼 deadman 후보에서 제외했다.
- `PE5(D12)`는 선을 뽑아도 `gripOpen=YES`가 유지돼 grip open 후보에서 제외했다.
- 현재는 이미 쓰는 핀을 피하기 위해 digital header `D9/D10/D11/D13`를 버튼 입력으로 사용한다.
- 실제 handheld 배선이 정해지면 STM32 `main.c` 상단 매크로만 바꾸면 된다.
- teleop v1은 `ARM/DISARM`을 직접 처리하지 않는다. 먼저 ESP32를 `ARMED` 상태로 올린 뒤 사용해야 한다.

빠른 실기 체크 순서:

1. ESP32 펌웨어 업로드 후 `status` 또는 웹 `/status` 확인
2. STM32 teleop remote 펌웨어 업로드
3. `STM32 PD5 -> ESP32 GPIO34`, `GND common` 연결
4. 센서 허브 없이 단일 STM32 remote만 bench 테스트한다면 ESP32 시리얼에서 `sensor sim healthy` 실행
5. ESP32를 `arm`
6. deadman을 누른 채 STM32를 중립 자세로 잡기
7. deadman을 떼고 다시 누르며 새 중립이 잡히는지 SWV 로그 확인
8. deadman을 누른 채 앞/뒤/좌/우/비틀기 입력으로 teleop 반응 확인
9. `/status.teleop` 또는 시리얼 `status`에서 `connected`, `deadman`, `reach`, `lift`, `twist`, `gripOpen`, `gripClose`, `lastStopReason` 확인
10. deadman release 또는 선 분리 시 `FRAME_TIMEOUT` / `DEADMAN_RELEASE` 정지 확인

주의:

- 현재 STM32 펌웨어는 `APP_MODE_TELEOP_REMOTE`와 `APP_MODE_SENSOR_BRIDGE` 중 하나로 동작한다.
- 한 개 STM32를 remote 모드로 쓰는 bench에서는 ESP32 sensor bridge가 실제 센서 패킷을 받지 못하므로 `sensor sim healthy`가 필요하다.
- 따라서 현재 single-STM32 remote bench에서는 `GY-521`이 active handheld 입력이고, `HC-SR04`는 연결돼 있어도 active safety stream에 올라오지 않는다.
- 최종 실장에서는 `HC-SR04` safety stream을 별도 sensor bridge로 유지하거나, 동등한 본체 safety 입력 채널을 따로 확보해야 한다.

시리얼만으로 safety 상태를 bench에서 재현하려면 아래 simulation 명령을 사용할 수 있다. `base angle` 관련 simulation은 구현 검사용으로 남아 있지만, 현재 하드웨어 로드맵의 필수 gate는 아니다.

```text
sensor sim healthy
sensor sim obstacle 10
sensor sim vibration 9
sensor sim rotate left 15
sensor sim stale
sensor sim off
```

### 시뮬레이션 검증 절차

하드웨어 없이 safety 경로를 빠르게 점검할 때는 아래 순서가 기준이다.

1. `sensor sim off`
2. `sensor sim healthy`
3. `status`
기대 결과:
`sensor.connected=true`, `blockReason=NONE`, `faultReason=NONE`

4. `sensor sim obstacle 10`
5. `arm`
기대 결과:
`OBSTACLE` 때문에 `ARM` 거부

6. `sensor sim healthy`
7. `arm`
8. `sensor sim vibration 9`
기대 결과:
`VIBRATION`으로 `FAULT` latch

9. `stop`
10. `sensor sim stale`
11. 잠시 대기 후 `status`
기대 결과:
`SENSOR_STALE` 감지

12. `sensor sim off`

상태를 더 보기 쉽게 보려면 다른 터미널에서 아래 watcher를 같이 실행하면 된다.

```bash
python3 tools/motionbrain_watch.py --host 192.168.4.1 --interval 1.0
```

### 기본 사용 흐름

1. ESP32 부팅
2. Wi-Fi AP `MotionBrain-AP` 접속 또는 USB 시리얼 연결
3. `arm`
4. `joint`, `motor`, `sequence`, `light` 명령 사용
5. 유선 handheld teleop를 쓸 때는 deadman을 누른 상태에서 `reach/lift/twist/grip` 입력 사용
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

1. 최종 부품 배치도와 배선표 확정
2. teleop 실기에서 `reach/lift/twist/grip` 부호, deadzone, 속도 비중 조정
3. 최종 실장 후 `HC-SR04` safety와 `RANGE_FAULT` 간헐 개입 여부 재검증
4. `GET /status`, `GET /events`, host watcher 기준으로 teleop/safety 이벤트 확인
5. `ESP32-CAM` 스트리밍과 비전 입력 연결
6. Raspberry Pi + ROS2 + AI 연동
7. 데모 시나리오, 문서, 포트폴리오 정리

## 포트폴리오 관점에서의 핵심 어필 포인트

- 멀티 MCU 역할 분리 설계
- 안전 상태 머신 기반 모터 제어
- 센서 피드백과 handheld teleop를 통한 입력/안전 계층 확장
- 웹/시리얼/센서/카메라/ROS2까지 이어지는 입력 계층 설계
- 카메라 기반 인식과 로봇 동작 연결
- 단순 동작이 아니라 구조와 진화 경로를 설명 가능한 프로젝트

## 라이선스

개인 연구, 학습, 제작, 포트폴리오 프로젝트.
