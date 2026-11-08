# MotionBrain

[English](README.en.md) · [포트폴리오](PORTFOLIO.md) · [빌드·운영](OPERATIONS.md)

기구학 표시의 양의 pitch는 위쪽 상승각으로 통일하고, 집게 개방과 독립적인 `tool0`를 사용한다. FK·IK와 URDF 위치·방향의 소프트웨어 일치 검증은 실물 치수·영점 보정을 대신하지 않는다. [좌표계와 한계](docs/visual-action-learning.md#전체-팔에서-기구학동역학을-쓰는-방법)를 참고한다.

**영상과 예정 명령으로 로봇팔의 움직임을 예측하고, 후보 결과를 비교해 물체를 잡고 드는 개인 로보틱스 프로젝트**다. ESP32 기반 5축 DC 로봇팔에 두 카메라와 센서를 연결해 수집·원본 검증·학습·평가·제한 실행 경로를 개발한다.

현재는 **관절별 동작 예측에서 M1 집게 닫기 → M4 어깨 들기로 연결하는 단계**다. 자동 정렬이나 전체 팔의 학습 기반 집기·들기는 아직 완성하지 않았다.

## 목표와 현재 단계 (2026-11-08)

목표하는 흐름은 `현재 영상 상태 + 후보 명령 → 변화 예측 → 목표와 비교 → 실행 → 재관측`이다. 먼저 필요한 관절을 순서대로 연결하며, 모든 관절의 동시 제어를 첫 집기의 선행 조건으로 두지 않는다.

| 구분 | 구현과 확인한 결과 |
| --- | --- |
| 수집·검증 | Mac 로컬 UI에서 M1~M5 중 한 관절씩 유한 단발 명령을 실행하고 영상·명령 의도·실제 출력·센서·실패를 저장·재생한다. 저장 완료와 학습 적격을 구분한다 |
| 기준선 비교 | M4 아래 방향 6조건, 학습 18건·고정 후 새 평가 18건. 명령별 평균 MAE **0.367°**, 영상·명령 ridge **0.605°**로 이 영상 표현의 이득은 확인되지 않았다 |
| 들기 예측 | 사람이 미리 파지한 물체와 새 기준 구도에서 M4 500ms 후보 모델을 새 4건으로 학습·고정하고 2건 평가했다. 상승 예측 MAE **2px**로 명령 평균과 동률이며, 이 모델의 선택 구동은 0회다 |
| 닫기→들기 연결 | 현재 자세의 새 M1 자료 4건(열기 2·닫기 2)으로 모델을 고정했다. 이 묶음의 실물 명령은 학습 4·준비 3, 총 7회이며, **고정 후 평가·모델 선택 닫기·닫기→들기 연결은 모두 0회**다. 관측 모호함과 좁은 모델 입력 범위 때문에 중단했다 |

현재 집게 카메라 관측은 사전에 등록한 구도와 기준점에 한정한 특징 추적이다. 일반적인 물체 인식·깊이·접촉 판정으로 확대하지 않는다. 실행 전 입력은 그때 확보한 영상 상태와 예정 명령이며, 실제 출력과 동작 후 센서는 실행 검증·정답·사후 분석에 사용한다. 관절·자세·관측 방법이 다른 자료는 자동 혼합하지 않는다.

과거 별도 M4 650ms 모델이 명령을 골라 실제 들림 1회를 관측한 적은 있다. 다만 변화량 **3.08°가 3° 관측 기준을 넘어 원래 실행은 실패**다. 사람이 미리 파지한 조건이며, 현재 500ms 모델의 성과나 자동 집기 완료를 뜻하지 않는다.

대표 근거: [현재 닫기→들기 연결과 중단](docs/evidence/2026-11-08-close-lift-integration.md) · [M4 학습·들림·실패 구분](docs/m4-up-commissioning.md) · [18건 고정 평가와 기준선](docs/evidence/2026-11-04-m4-action-prediction.md) · [M2 단발 모델 선택 목표 접근](docs/evidence/2026-10-31-first-model-goal-approach.md). 과거 관절별·표식 실험은 [증거 색인](docs/README.md)에서 조건별로 확인할 수 있다.

## 시스템 구성

```text
외부 카메라 ──> Mac 수집 UI ──> 영상·명령·출력·센서 기록
                           │                     │
                     한 관절 제한 명령       검증 → 학습·평가
                           ↓                     ↓
STM32 센서·원격조작 ──> ESP32 ──> DC 모터 5축   후보 예측 비교
                           ↑
                     AS5600: M4 각도

ESP32-CAM ──> 집게 위 관측 / Raspberry Pi 인식·대시보드·ROS2
```

ESP32는 명령·출력·정지 상태를 관리하고 호스트는 수집과 학습·평가를 담당한다. M4에만 각도 센서가 있으며 다른 네 축에는 위치 피드백이 없다. 기존 ROS2 실물 경로는 작업자 확인형 M4 단일 목표 실행을 검증했으며 전체 팔 궤적 추종은 아니다. 후보 비교를 모든 관절의 자동 제어로 연결한 상태도 아니다.

## 시작하기

호스트 검사는 Python 3.12 기준으로 `numpy`, `opencv-python-headless`, Node.js(CI: 22), C++ 컴파일러를 사용한다. 선택적 PyTorch 검사는 관련 의존성이 없으면 건너뛰며, 펌웨어 빌드에는 PlatformIO가 필요하다.

```sh
python3 -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

위 명령은 검사·빌드이며 펌웨어 업로드나 모터 구동을 하지 않는다. 원본 수집 자료와 모델 가중치는 공개 저장소에 포함하지 않으므로 실험 수치 재평가에는 해당 자료가 필요하다.

- **수집·재생·데이터셋·학습:** [관절별 수동 수집 안내](docs/manual-data-collection.md), [학습 목표·설계](docs/visual-action-learning.md)
- **제한 실행과 관측 계약:** [상대 상태 예측](docs/relative-action-selection.md), [모델 선택 순차 시험](docs/model-grasp-trial.md), [외부 영상 표식 학습](docs/projected-arm-learning.md)
- **설치·펌웨어·서비스·ROS2:** [운영 안내](OPERATIONS.md), [배선](PIN_MAP.md), [ROS2 하드웨어 인터페이스](ros2_ws/src/motionbrain_hardware_interface/README.md)
- **소프트웨어 검사:** [PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml), [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). 소프트웨어 검사와 실물 성능은 구분한다.

실물 수집 전에는 현재 연결·카메라·이동 여유를 확인한다. 실패하거나 실행 여부가 불명확한 동작은 자동 재전송하지 않는다. 간헐 HTTP 지연이 남아 있으며, 수집 UI의 중단 요청은 물리적 즉시 비상 정지를 보장하지 않는다. 실행 조건과 종료 확인 절차는 수집 안내에 명시했다.

## 전체 팔을 기준으로 이어갈 일

먼저 현재 자세에서 신뢰할 수 있는 관측과 M1 입력 범위를 확보하고, 고정 후 평가를 거쳐 M1 닫기·접촉 확인·M4 들기를 연결한다. 이후 필요한 접근 관절과 자세를 순차적으로 넓힌다.

기구학은 도달 가능 영역과 이동 여유 제약에 점진적으로 결합한다. 전체 팔 역기구학·동역학 기반 제어나 강화학습 정책을 도입한 단계는 아니며, 현재 학습은 동작 변화의 지도학습과 후보 비교다. Gazebo·MuJoCo의 접촉·동역학 검토는 기구 치수와 구동 반응을 보정한 뒤 진행한다. 시뮬레이터 도입을 첫 실물 연결의 선행 조건으로 두지 않는다.

RViz의 가상 관절 표시, Gazebo·MuJoCo의 개략 팔 모델, Rerun의 두 카메라 고정 평가 재생, W&B의 오프라인 지표 기록과 선택적 웹 게시 경로를 연결했다. [로컬 화면 실행](docs/robot-visualization.md)에서 검증 범위와 미보정 물리 모델의 한계를 설명한다.

## 코드와 문서

| 경로 | 역할 |
| --- | --- |
| `learning/`, `tools/` | 수집·재생, 원본 검증, 영상 상태 계측, 예측·평가와 후보 선택 |
| `src/`, `firmware/` | ESP32 모터 제어, 카메라 및 STM32 펌웨어 |
| `ros2_ws/src/` | 메시지·브리지·제어 가드·URDF·하드웨어 인터페이스 |
| [포트폴리오](PORTFOLIO.md) | 해결한 문제, 설계 판단, 대표 결과와 한계 |
| [문서·실험 증거 색인](docs/README.md) | 사용법과 날짜별 실측 근거 |

## 기존 원격조작 데모

![유선 원격조작 시연](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)

[MP4](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4) · 2026-06-08 고정 태그의 원격조작 영상이다. 현재 학습 모델의 집기 시연은 아니다.

## 라이선스

프로젝트 코드는 [MIT](LICENSE)다. STM32 HAL/CMSIS 외부 제공 파일은 `firmware/stm32/MotionBrainSensor/Drivers/`의 원래 라이선스를 따른다.
