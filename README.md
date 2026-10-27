# MotionBrain

[English](README.en.md) · [포트폴리오](PORTFOLIO.md) · [빌드·운영](OPERATIONS.md)

ESP32 기반 5축 DC 로봇팔에 STM32 센서·원격조작, ESP32-CAM, Raspberry Pi와 ROS2 Jazzy를 연결한 개인 로보틱스 프로젝트다. 모터 제어와 호스트 판단을 분리하고, 실제 센서 피드백과 실패 기록으로 구현을 검증한다.

## 데모

![유선 원격조작 시연](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)

[MP4](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4) · 2026-06-08 고정 태그의 원격조작 시연. 자율 파지 영상이 아니다.

## 구현과 검증 범위

| 범위 | 확인한 결과 | 한계 |
| --- | --- | --- |
| 임베디드 제어 | ESP32 5축 PWM, STM32 UART 원격조작, 상태 머신·데드맨·센서 고장 차단 | 전기적 정지 지연과 기능 안전 인증은 미검증 |
| M4 어깨 피드백 | AS5600 목표각 제어. 최종 무부하·23.10g 회귀 각각 11/11 통과 | 2026-06-28의 고정 장착·제한 범위 결과. 나머지 네 축 위치 피드백 없음 |
| ROS2 연결 | 타입이 정의된 메시지, URDF/RViz, 제어 가드, `ros2_control` | 실물 쓰기는 작업자 확인형 M4 단일 목표만 검증 |
| 관찰 기반 조작 | 립밤 파지·들기 후 추가 잡기·놓기 2회 | 외부 화면을 보고 수동 명령을 선택한 시연. 첫 반복은 보정 필요, 학습 정책 아님 |
| 인식·학습 코드 | 사전 학습 YOLOv5s 추론, NumPy ridge 학습·평가, 립밤·집게 오프라인 데이터·방향 표시·상대 위치 평가 | 실물 행동 학습 적격 전이 0건. 새 CV 도구는 합성 예측 검증이며 자체 모델 학습·자율 파지·학습 효과 미검증 |

## 시스템 구성

```text
STM32 (IMU·원격조작) ── UART ──> ESP32 (명령·안전 상태·PWM) ──> DC 모터 5축
                                   ↑ AS5600: M4 어깨만 피드백
ESP32-CAM ── HTTP /capture ──> Raspberry Pi (인식·대시보드·ROS2)
                                   └─ M4 제안 → 작업자 확인 → ESP32
```

- 물리 출력은 ESP32의 `Dispatcher`와 `SafetyGate`를 거친다. 호스트의 명령 접수를 목표 도달로 취급하지 않는다.
- 카메라 직접 `/stream`은 비활성화돼 있으며 `/capture` 또는 Pi 추적 프레임을 사용한다.
- 전체 관절 폐루프 궤적 제어와 자동 집기는 구현하지 않았다. 간헐 HTTP 지연도 남아 있다.

![제어 UI 예시](docs/assets/motionbrain-control-stream.png)

기존 제어 UI의 정적 예시다. 현재 장치 상태를 표시하는 화면은 아니다.

## 코드와 문서

| 경로 | 내용 |
| --- | --- |
| `src/`, `firmware/` | ESP32 제어기·카메라 및 STM32 펌웨어 |
| `ros2_ws/src/` | 메시지, 브리지, 제어 가드, 모델, 하드웨어 인터페이스 |
| `learning/`, `tools/` | 인식, 데이터 계약, 오프라인 학습·평가, 운영 도구 |
| [PORTFOLIO.md](PORTFOLIO.md) | 설계 선택, 실패 분석, 조건이 명시된 실측 결과 |
| [OPERATIONS.md](OPERATIONS.md) | 빌드, 서비스 설정, 상태 점검과 진단 |
| [PIN_MAP.md](PIN_MAP.md) | 모터·센서 배선과 부팅 핀 제약 |
| [ROS2 하드웨어 인터페이스](ros2_ws/src/motionbrain_hardware_interface/README.md) | dry-run·측정 상태·M4 제안 모드 |
| [데이터·학습 파이프라인](docs/data-pipeline.md) | 행동 기록, 적격성, 분할, 학습·평가 재현 |
| [립밤·집게 오프라인 CV](docs/offline-cv.md) | 승인 데이터 로더, 후보 표시, 저장 예측의 검출·방향·상대 위치 평가 |

## 로컬 빌드와 검사

Python 호스트 의존성은 `numpy`, `opencv-python-headless`이며 펌웨어 빌드에는 PlatformIO가 필요하다. ROS2와 서비스 설치는 [운영 안내](OPERATIONS.md)를 따른다.

```sh
python3 -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

[PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml) · [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). 빌드·테스트 결과는 커밋별로 확인하며 실물 성능과 구분한다.

## 라이선스

프로젝트 코드는 [MIT](LICENSE)다. STM32 HAL/CMSIS 외부 제공 파일은 `firmware/stm32/MotionBrainSensor/Drivers/`의 원래 라이선스를 따른다.
