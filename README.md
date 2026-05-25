# MotionBrain

[한국어 README](README.md) | [English README](README.en.md) | [Portfolio One-Pager](PORTFOLIO.en.md)

MotionBrain is an embedded robotics portfolio project built around an ESP32-based 5-axis robotic arm controller, an STM32 sensor/teleop layer, ESP32-CAM vision input, and a future Raspberry Pi + ROS2 + AI high-level control stack.

Key engineering areas: safety state machine, multi-MCU role separation, UART sensor feedback, unified serial/HTTP command dispatch, wired handheld teleoperation, and a clear path toward host-side robotics orchestration.

ESP32 기반 5축 로봇팔 제어 시스템에서 출발해, STM32 센서 허브, ESP32-CAM 비전 입력, Raspberry Pi + ROS2 + AI 상위 제어까지 확장하는 로봇 개발 포트폴리오 프로젝트.

핵심 구조는 다음 한 줄로 요약된다.

```text
입력 -> 판단 -> 상태 -> 움직임 -> 피드백
```

## 프로젝트 목표

- ESP32에서 실시간 모터 제어와 안전 상태 관리를 구현한다.
- STM32 센서/teleop 계층을 분리해 피드백과 조작 입력을 구조화한다.
- 시리얼, HTTP, teleop, 향후 ROS2 입력이 같은 명령 경계를 공유하도록 설계한다.
- ESP32-CAM, host-side decision, ROS2, AI 상위 제어로 확장 가능한 구조를 만든다.
- 전체 설계와 검증 과정을 취업용 포트폴리오로 설명 가능한 형태로 정리한다.

## 현재 상태

### 구현 완료

- ESP32 5축 DC 모터 제어 커널
- 안전 상태 머신: `BOOT -> IDLE -> ARMED -> FAULT`
- 시리얼 명령 인터페이스와 Wi-Fi AP 기반 HTTP 제어
- `RobotArm` 관절 추상화와 `MotionSequence` 비차단 시퀀스 큐
- `Dispatcher` + `SafetyGate` 기반 공통 명령 경로
- STM32 `MPU-6050 + HC-SR04 + UART` 센서 스트림 bench 검증
- ESP32 `Stm32Bridge`, `SafetyMonitor`, `EventLog`
- `GET /status`, `GET /events` 기반 상태/이벤트 관측
- `sensor sim ...` 기반 bench simulation 경로
- 유선 handheld teleop v1: deadman, frame freshness timeout, LED edge, initial mixer, embedded safety telemetry
- `TB6612FNG x3` + `M1~M5` 실물 연결 및 모터 출력 확인
- 유선 teleop deadman + IMU 입력으로 실제 모터 출력 및 release 정지 확인
- trusted home Wi-Fi station mode와 token-aware host command path
- GitHub Actions 기반 PlatformIO 빌드와 host vision alignment synthetic test
- ESP32-CAM + Mac host Phase 4 MVP
  - `/status`, `/capture`, `/stream` 실기 확인
  - Mac host에서 MotionBrain `/status`와 ESP32-CAM frame 동시 fetch 확인
  - OpenCV red target detection 확인
  - red target 감지 시 안전한 `/light?action=toggle` command path 확인
  - 실제 search light 점등 확인
  - Vision-Based Alignment dry-run `LEFT/CENTER/RIGHT` 판정 확인
  - opt-in timed nudge mode로 `base.left` / `base.right` 실제 짧은 보정 동작 확인

### 현재 집중 작업

- Vision-Based Alignment 1차 실기 검증 결과 정리
- 포트폴리오용 데모 시나리오와 영상 캡처 준비
- Tests / CI 보강
- 최종 부품 배치와 배선표 확정
- Raspberry Pi + ROS2 + AI 상위 제어 연동 설계

## 아키텍처

### 구현된 계층

```text
[STM32 Sensor / Teleop Layer]
  MPU-6050
  HC-SR04
  UART teleop stream + embedded safety telemetry
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

### 목표 계층

```text
[STM32 Sensor / Teleop Layer]
  Safety input
  Handheld remote input
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

### Phase 4 MVP 검증 완료 계층

```text
[ESP32-CAM Vision Node]
  /status
  /capture
  /stream
        ->
[Mac Host Script]
  OpenCV red target detection
  MotionBrain /status safety check
        ->
[ESP32 Motion Controller]
  Dispatcher + SafetyGate
  /light?action=toggle
        ->
SearchLight
```

## 하드웨어

세부 GPIO와 전원 기준은 [PIN_MAP.md](PIN_MAP.md)를 기준으로 한다. Teleop 배선과 bring-up 절차는 [docs/TELEOP_BRINGUP.md](docs/TELEOP_BRINGUP.md)에 정리한다.

주요 구성품:

- `ESP32 DevKit`
- `ESP32-CAM`
- `TB6612FNG x3`
- 5축 DC 모터 로봇팔
- `STM32 B-F446E-96B01A`
- `MPU-6050 (GY-521)`
- `HC-SR04`
- `1602 LCD + I2C backpack`
- 점퍼선, 전원 모듈, 브레드보드

## 소프트웨어 구조

현재 ESP32 메인 진입점은 [src/main.cpp](src/main.cpp)이다.

- `system/`: 상태 머신과 시스템 초기화
- `motor/`: `TB6612FNG` 기반 모터 제어
- `motion/`: `RobotArm`, `MotionSequence`
- `bridge/`: STM32 센서 수신과 simulation
- `safety/`: 센서 기반 차단과 fault latch
- `control/`: `Dispatcher`, `SafetyGate`, `AngleController`, `EventLog`
- `input/`: 시리얼 명령과 teleop 입력 처리
- `network/`: Wi-Fi AP, 웹 서버
- `peripheral/`: `SearchLight`
- `debug/`: 로그 출력

현재 HTTP 경계:

- `GET /status`
- `GET /events`
- `POST /command`
- `POST /motor`
- `POST /joint`
- `POST /base`
- `POST /sequence`
- `POST /light`

### ROS2 Bridge MVP

Phase 4 host path is also exposed through a minimal ROS2 package:

```text
ros2_ws/src/motionbrain_ros_bridge
```

It keeps the ESP32 HTTP API unchanged and publishes JSON payloads on:

- `/motionbrain/status`
- `/motionbrain/events`
- `/camera/detection`

It also subscribes to `/motionbrain/light_cmd` and forwards `on`, `off`, or `toggle` to `POST /light`.

Raspberry Pi bring-up is documented in [docs/RASPBERRY_PI_ROS2_BRINGUP.md](docs/RASPBERRY_PI_ROS2_BRINGUP.md). The intended portfolio validation path is ROS2 Jazzy on Raspberry Pi 4, Home Wi-Fi access to the ESP32 controller and ESP32-CAM, topic echo verification, and one ROS2 command-channel test.

Build and run directly:

```bash
cd ros2_ws
colcon build --packages-select motionbrain_ros_bridge
source install/setup.bash
ros2 run motionbrain_ros_bridge motionbrain_status_node --ros-args -p motion_host:=192.168.4.1 -p camera_url:=http://192.168.4.2
```

Run on Home Wi-Fi with the launch file:

```bash
ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py
```

Package notes are in [ros2_ws/src/motionbrain_ros_bridge/README.md](ros2_ws/src/motionbrain_ros_bridge/README.md).

## 개발 환경

### ESP32 Motion Controller

- PlatformIO
- `esp32dev`
- Arduino framework

```bash
pio run
pio run -t upload
pio device monitor
```

### ESP32-CAM

ESP32-CAM 펌웨어 경로:

```bash
firmware/esp32cam
```

빌드:

```bash
pio run -d firmware/esp32cam
```

카메라와 Mac host를 연결하는 Phase 4 MVP 절차는 [PHASE4_MVP.md](PHASE4_MVP.md)를 기준으로 한다.

집 Wi-Fi에서 controller와 ESP32-CAM을 같은 LAN에 붙여 Mac Wi-Fi 전환 없이 테스트하려면 [docs/HOME_WIFI_MODE.md](docs/HOME_WIFI_MODE.md)를 따른다. 실제 SSID/password와 command token은 serial monitor에서 입력하고 ESP32 NVS flash에만 저장한다.

검증된 host-side vision loop:

```bash
python3 tools/vision_host_mvp.py --camera-url http://192.168.4.2 --detect-color red --once
python3 tools/vision_host_mvp.py --camera-url http://192.168.4.2 --detect-color red --enable-action --once
python3 tools/vision_host_mvp.py --camera-url http://192.168.4.2 --detect-color red --enable-align-action
```

기본 vision loop는 target center, horizontal offset, `LEFT|CENTER|RIGHT|LOST` alignment, `commandSuggestion`을 dry-run으로 출력한다. ESP32-CAM 캡처 안정성을 위해 기본 요청 간격은 3초, timeout은 6초, capture retry는 2회다. `--enable-align-action`을 켠 경우 현재 하드웨어 구성에서는 안전 조건 확인 후 짧은 `/joint?joint=base` nudge를 보내고 즉시 stop한다. base-mounted gyro feedback이 생기면 `--align-mode angle`로 `/base?action=angle` 폐루프를 쓸 수 있다.

### STM32 Sensor / Teleop Layer

- STM32CubeIDE
- HAL / CubeMX
- 프로젝트 경로: `firmware/stm32/MotionBrainSensor`

Helper scripts:

- `tools/stm32_build.sh`
- `tools/stm32_upload.sh`
- `tools/stm32_build_upload.sh`

## 빠른 확인

Host watcher는 `GET /status`, `GET /events`를 주기적으로 읽어 state, safety, base-angle, teleop 상태를 한 줄로 보여준다.

```bash
python3 tools/motionbrain_watch.py --host 192.168.4.1 --interval 1.0
```

Local ops dashboard는 같은 상태/이벤트 경계에 ESP32-CAM capture, 색상 감지, light command log, token-gated one-shot vision nudge control을 더해 브라우저에서 보여준다. 수동 조작은 ESP32가 직접 제공하는 `MotionBrain Control`을 사용한다. Command token이 설정된 경우 `MotionBrain Control`은 첫 state-changing command에서 token을 물어보고, 입력값은 현재 브라우저 페이지 메모리에만 유지한다. 2026-05-25 bench에서는 phone browser에서 token prompt와 command 동작을 확인했다.

```bash
python3 tools/motionbrain_dashboard.py --camera-url http://192.168.4.2
```

브라우저에서 연다.

```text
http://127.0.0.1:8765
```

STM32 teleop remote가 연결된 bench 구성에서는 teleop frame 안의 embedded safety telemetry로 `SENSOR_STALE`을 해제한다. 하드웨어 없이 safety 상태를 재현하거나 fault case를 강제로 만들 때는 simulation 명령을 사용할 수 있다.

```text
sensor sim healthy
sensor sim obstacle 10
sensor sim vibration 9
sensor sim stale
sensor sim off
```

기본 사용 흐름:

1. ESP32 부팅
2. Wi-Fi AP `MotionBrain-AP` 접속 또는 USB 시리얼 연결
3. `arm`
4. `joint`, `motor`, `sequence`, `light` 명령 사용
5. 필요 시 `stop` 또는 `disarm`

## 포트폴리오 관점

이 프로젝트에서 강조하는 역량:

- 멀티 MCU 역할 분리 설계
- 안전 상태 머신 기반 모터 제어
- 센서 피드백과 handheld teleop를 통한 입력/안전 계층 확장
- 시리얼/HTTP/status/event 경계 설계
- 카메라 입력, host-side decision, ROS2로 확장 가능한 구조
- ESP32-CAM + OpenCV 기반 host-side perception/action loop 실기 검증
- 하드웨어 bench 검증과 문서화

## 문서

- [README.en.md](README.en.md): 영어 프로젝트 개요
- [PORTFOLIO.en.md](PORTFOLIO.en.md): 영어 포트폴리오 one-pager
- [로드맵.md](%EB%A1%9C%EB%93%9C%EB%A7%B5.md): 포트폴리오 기준 전체 단계
- [PHASE3_PLAN.md](PHASE3_PLAN.md): 센서/브리지 계획
- [PHASE4_MVP.md](PHASE4_MVP.md): ESP32-CAM + Mac host 비전 MVP
- [MESSAGE_INTERFACE.md](MESSAGE_INTERFACE.md): 시리얼/HTTP/status 메시지 경계
- [PIN_MAP.md](PIN_MAP.md): ESP32 모터 핀 연결
- [docs/TELEOP_BRINGUP.md](docs/TELEOP_BRINGUP.md): 유선 handheld teleop bring-up 절차

## 라이선스

개인 연구, 학습, 제작, 포트폴리오 프로젝트.
