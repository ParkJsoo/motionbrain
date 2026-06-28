# MotionBrain

[영어 README](README.en.md) | [포트폴리오 요약](PORTFOLIO.md) | [영어 포트폴리오 요약](PORTFOLIO.en.md)

MotionBrain은 ESP32 기반 5축 로봇팔 제어기에서 시작해 STM32 센서/텔레오퍼레이션 계층, ESP32-CAM 비전 입력, Raspberry Pi + ROS2 호스트 브리지까지 확장한 임베디드 로보틱스 시스템 프로젝트다.

핵심 목표는 단순히 모터를 움직이는 것이 아니라, 실제 하드웨어에서 안전 상태, 명령 경계, 센서 피드백, 비전 입력, ROS2 연동을 한 흐름으로 검증하는 것이다.

```text
입력 -> 판단 -> 상태 -> 움직임 -> 피드백
```

## 한눈에 보는 핵심 증거

| 역량 | 구현 증거 |
| --- | --- |
| 실물 로봇 통합 | ESP32 5축 모션 제어기, STM32 유선 텔레오퍼레이션, ESP32-CAM, Raspberry Pi 호스트를 하나의 arm stack으로 통합 |
| 임베디드 안전 경계 | `BOOT -> IDLE -> ARMED -> FAULT`, `Dispatcher` + `SafetyGate`, 토큰 기반 명령, deadman release stop, 프레임 타임아웃 |
| 단일축 위치 피드백 | M4 어깨 AS5600 절대각 측정, 센서/자석 상태 감시, 230-245° 제한 폐루프 목표각 제어, `TARGET_MISSED` 실패 판정, 동일 고정 장착 무부하 22/22와 23.1g 하중 11/11 회귀, HTTP/대시보드/ROS2 텔레메트리 |
| ROS2 시스템 소프트웨어 | ROS2 Jazzy typed topics, C++ control guard, mission supervisor, URDF/RViz, `ros2_control` dry-run mock/open-loop `SystemInterface` |
| 운영/검증 | Pi systemd 서비스, health-check 스크립트, runtime evidence, `ros2_control` evidence, PlatformIO/Python/ROS2 GitHub Actions, 물리 텔레오퍼레이션 데모 |

검토자가 먼저 볼 만한 상세 근거:

- [PORTFOLIO.md](PORTFOLIO.md)
- [ROBOTICS_SYSTEM_READINESS.md](ROBOTICS_SYSTEM_READINESS.md)
- [OPERATIONS.md](OPERATIONS.md)
- [PIN_MAP.md](PIN_MAP.md)
- [docs/evidence/2026-06-28-m4-shoulder-closed-loop.md](docs/evidence/2026-06-28-m4-shoulder-closed-loop.md)
- [docs/evidence/2026-06-16-ros2-control-open-loop.md](docs/evidence/2026-06-16-ros2-control-open-loop.md)
- [docs/evidence/2026-06-16-pi-system-health.md](docs/evidence/2026-06-16-pi-system-health.md)
- [docs/evidence/2026-06-17-runtime-measurements.md](docs/evidence/2026-06-17-runtime-measurements.md)

## 로보틱스 시스템 역량

이 저장소의 핵심 증거는 실제 하드웨어 통합과 ROS2 기반 시스템 경계 설계다.
ROS2 Jazzy typed interface, C++ control guard, mission supervisor, RViz/TF
시각화, `ros2_control` dry-run mock bring-up, 안전한 open-loop `SystemInterface`
scaffold를 포함한다.

실제 물리 모션은 ESP32 firmware `SafetyGate` 뒤에 남겨 두었고,
`ros2_control` hardware-interface 경로는 현재 `dry_run` 전용이다. 검증
요약은 [2026-06-16 ros2_control evidence note](docs/evidence/2026-06-16-ros2-control-open-loop.md)를 본다.

## 데모 영상

아래 GIF는 최종 물리 텔레오퍼레이션 데모를 README 안에서 바로 보여준다.

![MotionBrain 데모 영상](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)

[MP4 파일 다운로드](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4)

물리 텔레오퍼레이션 데모 미디어 스냅샷: `demo-ready-20260608`

## 운영 화면

아래 이미지는 최종 데모 영상이 아니라, 제어기와 Pi 대시보드가 제공하는 실제 운영 UI를 문서용 정적 상태로 캡처한 것이다.

![MotionBrain Control 웹 콘솔](docs/assets/motionbrain-control-stream.png)

ESP32 내장 `MotionBrain Control`은 수동 조작, 토큰 기반 명령 경계, `STREAM` 카메라 피드백, 모터/조인트 조작 표면을 한 화면에 모은다.

![MotionBrain Pi 대시보드](docs/assets/motionbrain-dashboard.png)

Pi 호스트 대시보드는 제어기 상태, 텔레오퍼레이션, 이벤트, 카메라 프레임, 타겟 감지 상태를 관찰하고 작업자가 명시적으로 누르는 토큰/안전 게이트 기반 bounded base nudge 제어만 제한적으로 노출한다.

![MotionBrain RViz RobotModel](docs/assets/motionbrain-rviz-robotmodel.png)

Docker/noVNC RViz 화면은 Pi 대시보드 상태와 감지 결과를 읽기 전용 HTTP mirror로 받아 ROS2 topic, `RobotModel`, TF 시각화까지 이어지는 경로를 보여준다.

## 현재 상태

검증 완료:

- ESP32 5축 DC 모터 제어와 `BOOT -> IDLE -> ARMED -> FAULT` 안전 상태 머신
- `Dispatcher` + `SafetyGate` 기반 시리얼/HTTP 공통 명령 경로
- STM32 `MPU-6050 + UART` 센서/텔레오퍼레이션 스트림과 HC-SR04 bench 검증 경로
- 유선 핸드헬드 텔레오퍼레이션: 데드맨, 프레임 타임아웃, 안전 텔레메트리
- 멀티미터 기반 전원/GND/버튼/출력 sanity check 기록과 claim boundary 문서화
- ESP32-CAM `/status`, `/capture`, `/stream`, `/camera` 프로필 제어
- 로컬 LAN 기반 ESP32 제어기, ESP32-CAM, Raspberry Pi 연결
- ESP32 내장 `MotionBrain Control` 웹 UI와 토큰 기반 상태 변경 명령
- Pi 호스트 대시보드: 상태, 이벤트, 카메라, 타겟 오버레이, 안전 게이트 기반 bounded base nudge 제어 표면
- Raspberry Pi 4 + Ubuntu 24.04 + ROS2 Jazzy 브리지
- ROS2 타입 지정 토픽: 상태, 이벤트, 카메라 감지, 상태 기반/open-loop 조인트 상태, 기구학 진단, 제어 guard, mission 상태
- `ros2_control` dry-run mock 데모와 안전한 open-loop `SystemInterface` 스캐폴드
- Pi 인식 서비스를 통한 `/camera/detection(_typed)` 연동
- ESP32 내장 제어 페이지의 카메라 모드 분리: 수동 조작은 `STREAM`, 인식 확인은 `TRACKED`
- Docker/noVNC RViz 검증 환경에서 RobotModel/TF와 Pi dashboard mirror 기반 live ROS2 topic 시각화
- GitHub Actions 기반 PlatformIO 빌드, Python 테스트, ROS2 `colcon build/test`

현재 주의점:

- 빨간 타겟 추적은 별도 비전/정렬 검증에서 안정적으로 동작한 경로다.
- 객체 인식 흐름은 Pi에서 구현됐고, 현재 bench에서는 ESP32-CAM `qvga` / JPEG quality `10` + YOLOv5s 조합으로 제한된 known-object `cup` 인식이 검증됐다. 현재 인식 데모는 `cup` 하나만 활성 타겟으로 사용한다.
- 어두운 저텍스처 물체와 반사가 강한 phone-like 타겟은 현재 데모 범위에서 제외한다. 이 결과는 임의 객체 인식이 아니라 제한된 작업공간의 known-object 인식/정렬 데모로 설명해야 한다.
- 자동 grasp는 아직 하지 않는다. 현재 cup dry-run 경로는 안전 상태와 CENTER 정렬을 재확인한 뒤 작업자 확인용 그리퍼 open/close 계획만 반환한다.
- 로봇팔을 조종하면서 카메라를 보는 작업은 `STREAM`이 기본이다. `TRACKED`는 Pi 인식 결과를 확인하는 느린 뷰로만 쓴다.
- HC-SR04는 최종 물리 데모에서 장착하지 않았고, range telemetry는 disabled/nonblocking demo state로 처리한다.

## 시스템 구성

```text
[STM32 센서/텔레오퍼레이션]
  MPU-6050, UART 프레임
  HC-SR04 펌웨어 경로는 bench 검증됨, 최종 데모 미장착
        ->
[ESP32 모션 제어기]
  SafetyMonitor, Dispatcher, SafetyGate
  RobotArm, MotionSequence, EventLog
        ->
[TB6612FNG x3]
        ->
5축 DC 모터 로봇팔

[ESP32-CAM]
  /capture, /stream, /camera
        ->
[Raspberry Pi]
  인식 서비스
  대시보드
  ROS2 bridge
        ->
ROS2 타입 지정 토픽, control guard, mission supervisor
```

## 주요 디렉터리

- `src/`: ESP32 모션 제어기 펌웨어
- `firmware/esp32cam/`: ESP32-CAM 펌웨어
- `firmware/stm32/MotionBrainSensor/`: STM32 센서/텔레오퍼레이션 펌웨어
- `tools/`: 대시보드, 인식 서비스, 상태 감시기, STM32 보조 스크립트
- `ros2_ws/src/`: ROS2 메시지, 브리지, 제어 guard, mission, URDF, `ros2_control` 패키지
- `docs/assets/`: README/포트폴리오용 공개 데모 이미지와 영상
- `config/`: 비전 모델 라벨 등 런타임 설정 파일

## 빠른 실행

ESP32 모션 제어기 빌드:

```bash
pio run
```

ESP32-CAM 빌드:

```bash
pio run -d firmware/esp32cam
```

호스트 테스트:

```bash
python3 -m unittest discover -s tests
```

Raspberry Pi 접속 확인:

```bash
python3 tools/raspi/check_pi_ssh_target.py
ssh motionbrain-pi 'hostname; hostname -I; systemctl is-active ssh'
```

Pi SSH alias는 DHCP IP가 아니라 `motionbrain-pi.local`을 우선 따라가도록
설정한다. `.davolink`은 라우터 DNS fallback으로만 쓴다. 자세한 접속/복구 절차는
[OPERATIONS.md](OPERATIONS.md)를 본다.

Raspberry Pi ROS2 빌드:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_hardware_interface motionbrain_mission motionbrain_ros_bridge motionbrain_description motionbrain_ros2_control_mock
colcon test --packages-select motionbrain_msgs motionbrain_control motionbrain_hardware_interface motionbrain_mission motionbrain_ros_bridge motionbrain_description motionbrain_ros2_control_mock
colcon test-result --verbose
```

Pi 대시보드/인식 서비스는 `deploy/systemd/`의 unit 파일로 부팅 자동 실행할 수 있다.

수동 fallback 대시보드 예시:

실제 `MOTIONBRAIN_HTTP_TOKEN`은 로컬 장비용 명령 토큰이다. 실제 값을 repo,
로그, 화면 캡처에 노출하지 않는다.

```bash
export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/motionbrain_dashboard.py \
  --host 0.0.0.0 \
  --motion-host <controller-ip> \
  --camera-url http://<camera-ip> \
  --detect-color red \
  --timeout 6
```

Pi 인식 서비스를 분리해서 쓰는 수동 fallback 예시:

```bash
python3 tools/motionbrain_perception_service.py \
  --host 127.0.0.1 \
  --port 8766 \
  --camera-url http://<camera-ip> \
  --detector-mode color \
  --detect-color red \
  --timeout 6

export MOTIONBRAIN_HTTP_TOKEN="<local-controller-token>"
python3 tools/motionbrain_dashboard.py \
  --host 0.0.0.0 \
  --motion-host <controller-ip> \
  --perception-url http://127.0.0.1:8766 \
  --timeout 6
```

현재 cup known-object demo는 ESP32-CAM `qvga` / JPEG quality `10`, Pi
YOLOv5s object mode, 설정된 confidence gate, dashboard proxy 조합을 사용한다.
systemd wrapper는 ESP32-CAM 재부팅 후에도 이 카메라 프로필을 다시 적용하고,
낮은 JPEG quality 설정은 안정 최소값으로 올린다.

## 문서

- [PORTFOLIO.md](PORTFOLIO.md): 한국어 포트폴리오 요약
- [PORTFOLIO.en.md](PORTFOLIO.en.md): 영어 포트폴리오 요약
- [ROBOTICS_SYSTEM_READINESS.md](ROBOTICS_SYSTEM_READINESS.md): 로보틱스 시스템/ROS2 하드웨어 경계 요약
- [PIN_MAP.md](PIN_MAP.md): ESP32 핀 점유와 M4 AS5600 I2C 부트 조건
- [docs/evidence/2026-06-28-m4-shoulder-closed-loop.md](docs/evidence/2026-06-28-m4-shoulder-closed-loop.md): M4 어깨 AS5600 절대각 피드백과 제한 폐루프 실물 검증
- [docs/evidence/2026-06-16-ros2-control-open-loop.md](docs/evidence/2026-06-16-ros2-control-open-loop.md): `ros2_control` dry-run 검증 요약
- [docs/evidence/2026-06-16-pi-system-health.md](docs/evidence/2026-06-16-pi-system-health.md): Pi/systemd/ROS2 health 검증 요약
- [docs/evidence/2026-06-17-runtime-measurements.md](docs/evidence/2026-06-17-runtime-measurements.md): Pi 런타임 endpoint latency, ROS2 topic/status probe, 계측 장비 inventory 측정 기록
- [docs/evidence/2026-06-16-embedded-bench-checks.md](docs/evidence/2026-06-16-embedded-bench-checks.md): 멀티미터 기반 embedded bench sanity check 복구 기록
- [EMBEDDED_BRINGUP.md](EMBEDDED_BRINGUP.md): STM32/ESP32 bring-up 및 측정 체크리스트
- [OPERATIONS.md](OPERATIONS.md): Pi/systemd/health-check 운영 절차

## 라이선스

Project code is MIT licensed; see [LICENSE](LICENSE). STM32 HAL/CMSIS vendor
files retain their upstream licenses under `firmware/stm32/MotionBrainSensor/Drivers/`.
