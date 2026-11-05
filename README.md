# MotionBrain

[English](README.en.md) · [포트폴리오](PORTFOLIO.md) · [빌드·운영](OPERATIONS.md)

**영상과 명령으로 로봇팔의 움직임을 학습하고, 예측한 결과를 비교해 목표에 접근하는 개인 로보틱스 프로젝트**다. ESP32 기반 5축 DC 로봇팔에서 영상·명령·실제 출력·센서를 함께 기록하고, 데이터 검증부터 학습·평가·후보 명령 선택까지 연결한다.

현재는 제한된 한 관절의 동작 예측과 단발 목표 접근을 검증한 단계다. 물체와 집게의 영상상 위치를 이용해 정렬·파지·들기를 이어가는 학습 기반 조작은 아직 완성하지 않았다.

## 목표와 현재 범위

목표하는 흐름은 `현재 영상 상태 + 후보 명령 → 변화 예측 → 목표와 비교 → 실행 → 재관측`이다. 첫 과제는 작은 물체에 접근해 잡고 드는 것이다. 현재 구현과 실측은 다음 범위에 한정된다.

| 단계 | 현재 구현·검증 |
| --- | --- |
| 수집·재생 | Mac 로컬 UI에서 M1~M5 중 한 관절씩 선택. 영상·예정 명령·실제 드라이버 출력·센서와 실패 기록을 저장하고 재생 |
| 데이터셋 | 기존 센서 라벨은 M4 대상. 별도 M1·M2·M5 경로는 집게 위 영상의 물체·양턱 상대 상태를 수동 주석. 사후 출력은 입력에서 제외 |
| 학습·평가 | M4 아래 방향 6조건, 학습 18건 이후 모델을 고정하고 새 18건 평가. 명령별 평균 MAE **0.367°**, 영상·명령 ridge **0.605°**. 현재 영상 표현의 이득은 확인되지 않음 |
| 후보 선택 | 지원하는 명령과 정지의 예측 잔차를 비교하는 오프라인 선택기. 별도 M2 실험에서는 모델 선택 명령 한 번의 영상상 목표 접근 확인 |
| 남은 연결 | 물체·집게 상태 추정, 목표에 맞는 행동 선택과 재관측을 연결한 반복 제어, 학습 기반 집기·들기 평가 |

M4 수치는 같은 날·같은 장치의 순차 자세에서 센서 라벨과 비교한 결과다. 다른 날짜·하중의 일반화, 센서 자체 정확도 또는 집기 성능을 뜻하지 않는다. 실제 PWM과 출력 기간은 사후 기록이며 실행 전 예측 입력에 넣지 않는다. 서로 다른 관절·관측 계약의 자료는 자동 혼합하지 않는다.

[최신 M4 평가·실패 분모](docs/evidence/2026-11-04-m4-action-prediction.md) · [M2 단발 목표 접근과 한계](docs/evidence/2026-10-31-first-model-goal-approach.md) · [학습 목표·설계](docs/visual-action-learning.md)

[상대 상태 개발 경로](docs/relative-action-selection.md)는 수동 영상 주석으로 M4 전이 2건의 예측·후보 계산을 연결했다. 집게 위 영상으로 [M1 전이 4건 학습·새 1건 평가](docs/evidence/2026-11-04-gripper-relative-learning.md)도 수행했으며, 그 1건에서는 무변화 기준이 최선이었다. 독립 평가·자동 상태 추정·실물 집기는 미검증이다.

후속 [관절별 자료 확대·고정 평가](docs/evidence/2026-11-05-multi-joint-relative-learning.md)에서는 M1·M5·M2를 분리해 33건을 확보하고 24건 학습·9건 후속 평가를 수행했다. 예측 범위 거절과 큰 오차가 남아 있으며 학습 정책의 집기·들기는 미검증이다.

## 시스템 구성

```text
외부 카메라 ──> Mac 수집 UI ──> 영상·명령·출력·센서 기록
                           │                     │
                     한 관절 제한 명령       검증 → 학습·평가
                           ↓                     ↓
STM32 센서·원격조작 ──> ESP32 ──> DC 모터 5축   오프라인 후보 비교
                           ↑
                     AS5600: M4 각도

ESP32-CAM ──> Raspberry Pi 인식·대시보드·ROS2 (기존 통합 경로)
```

후보 비교 결과를 모든 관절의 자동 제어로 연결한 구조는 아니다. ESP32는 명령·출력·정지 상태를 관리하고, 호스트는 수집과 학습·평가를 담당한다. M4에만 각도 센서가 있으며 다른 네 축에는 위치 피드백이 없다. 기존 ROS2 경로는 작업자 확인형 M4 단일 목표 실행을 검증했다.

## 시작하기

호스트 검사는 Python 3.12 기준으로 `numpy`, `opencv-python-headless`, Node.js(CI: 22), C++ 컴파일러를 사용한다. 선택적 PyTorch 검사는 관련 의존성이 없으면 건너뛰며, 펌웨어 빌드에는 PlatformIO가 필요하다.

```sh
python3 -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

위 명령은 검사·빌드이며 펌웨어 업로드나 모터 구동을 하지 않는다. 원본 수집 자료와 모델 가중치는 공개 저장소에 포함하지 않으므로 실제 실험 수치의 재평가에는 해당 자료가 필요하다.

- **수집·재생·데이터셋·학습:** [관절별 수동 수집 안내](docs/manual-data-collection.md)
- **설치·펌웨어·서비스·ROS2:** [운영 안내](OPERATIONS.md), [배선](PIN_MAP.md), [ROS2 하드웨어 인터페이스](ros2_ws/src/motionbrain_hardware_interface/README.md)
- **검증 결과:** [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml), [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). 소프트웨어 검사와 실물 성능은 구분한다.

실물 수집 전에는 현재 연결·카메라·이동 여유를 확인해야 한다. 간헐 HTTP 지연이 남아 있으며, 수집 UI의 중단 요청은 물리적 즉시 비상 정지를 보장하지 않는다. 실행 조건과 종료 확인 절차는 수집 안내에 명시했다.

## 코드와 문서

| 경로 | 역할 |
| --- | --- |
| `learning/`, `tools/` | 수집·재생, 데이터 검증, 인식·동작 예측, 평가와 후보 선택 |
| `src/`, `firmware/` | ESP32 모터 제어, 카메라 및 STM32 펌웨어 |
| `ros2_ws/src/` | 메시지·브리지·제어 가드·URDF·하드웨어 인터페이스 |
| [포트폴리오](PORTFOLIO.md) | 해결한 문제, 설계 판단, 결과와 한계 |
| [문서·실험 증거 색인](docs/README.md) | 현재 사용법과 날짜별 실측 근거 |

## 기존 원격조작 데모

![유선 원격조작 시연](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)

[MP4](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4) · 2026-06-08 고정 태그의 원격조작 영상이다. 현재 학습 모델의 집기 시연은 아니다.

## 라이선스

프로젝트 코드는 [MIT](LICENSE)다. STM32 HAL/CMSIS 외부 제공 파일은 `firmware/stm32/MotionBrainSensor/Drivers/`의 원래 라이선스를 따른다.
