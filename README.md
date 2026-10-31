# MotionBrain

[English](README.en.md) · [포트폴리오](PORTFOLIO.md) · [빌드·운영](OPERATIONS.md)

ESP32 기반 5축 DC 로봇팔에 STM32 센서·원격조작, ESP32-CAM, Raspberry Pi와 ROS2 Jazzy를 연결한 개인 로보틱스 프로젝트다. 모터 제어와 호스트 판단을 분리하고, 실제 센서 피드백과 실패 기록으로 구현을 검증한다. 현재 연구 방향은 **카메라와 모터 명령으로 자신의 움직임을 학습하고, 그 예측으로 목표에 접근하는 시각 행동 학습**이다.

## 현재 목표와 진행 상태

고정 외부 카메라 영상에서 팔의 특징점과 명령 전후 변화를 기록하고, `현재 영상·최근 이력 + 후보 명령 → 다음 움직임`을 학습한다. 기구학과 기존 제어를 활용하면서 실제 마찰·유격·자세별 반응을 경험으로 보완하는 방향이다.

- **확인한 결과:** M4 단일축 제어·수동 관찰 파지·인식 개발 학습. 인식의 최종 12장은 모두 학습자료이며 대상 물체 중복 오탐 1건이 남았다.
- **수집·학습:** 별도 M2 반복 수집과 원본 심사로 32개 투영 방향 전이를 확보하고, [첫 행동 예측 모델](docs/evidence/2026-10-31-first-action-prediction.md)을 학습했다. 학습 16/검증 8/시험 8이며 시험 평균 절대오차는 영상상 상대 방향 0.665°, 방향 일치는 8/8이다.
- **남은 범위:** 같은 날·같은 구도·M2의 초기 예측 결과다. [오프라인 행동 선택기](docs/offline-action-selection.md)는 저장 상태에서 후보 명령을 비교한다. 실제 관절각 정확도, 다른 날짜·구도 일반화, 선택한 명령의 실물 목표 접근·파지는 미검증이다. 단일 목표 시험 연결은 모의 검증을 마쳤다. 정지사진 검출의 완성을 행동 학습의 선행 조건으로 삼지 않는다.

[목표·단계·평가 설계](docs/visual-action-learning.md) · [연구 근거](docs/research/visual-action-learning.md) · [인식 개발 결과](docs/evidence/2026-10-30-cv-development.md)

## 데모

![유선 원격조작 시연](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.gif)

[MP4](https://raw.githubusercontent.com/ParkJsoo/motionbrain/demo-ready-20260608/docs/assets/demo/motionbrain-demo.mp4) · 2026-06-08 고정 태그의 원격조작 시연. 자율 파지 영상이 아니다.

## 구현과 검증 범위

| 범위 | 확인한 결과 | 한계 |
| --- | --- | --- |
| 임베디드 제어 | ESP32 5축 PWM, STM32 UART 원격조작, 상태 머신·데드맨·센서 고장 차단 | 전기적 정지 지연과 기능 안전 인증은 미검증 |
| M4 어깨 피드백 | AS5600 목표각 제어. 최종 무부하·23.10g 회귀 각각 11/11 통과 | 2026-06-28의 고정 장착·제한 범위 결과. 나머지 네 축 위치 피드백 없음 |
| ROS2 연결 | 타입이 정의된 메시지, URDF/RViz, 제어 가드, `ros2_control` | 실물 쓰기는 작업자 확인형 M4 단일 목표만 검증 |
| 관찰 기반 조작 | 단일 종류의 대상 물체 파지·들기 후 추가 잡기·놓기 2회 | 외부 화면을 보고 수동 명령을 선택한 시연. 첫 반복은 보정 필요, 학습 정책 아님 |
| 인식 개발 | Faster R-CNN 개발 학습: 학습 12장 대상 물체 TP8/FP1/FN0·집게 TP12/FP0/FN0, 상대 위치·방향 8/8 | 단일 종류의 대상 물체에 대한 학습자료 적합성 결과. 최종 검출기의 미학습 일반화는 미검증 |
| 행동 예측 | M2 실물 32개 전이로 첫 선형·릿지 후보 학습과 보류 그룹 평가 | 같은 날·구도의 영상상 상대 방향 예측. 실물 모델 제어와 다른 조건의 일반화는 미검증 |

## 기존 시스템과 연구 경로

```text
STM32 (IMU·원격조작) ── UART ──> ESP32 (명령·안전 상태·PWM) ──> DC 모터 5축
                                   ↑ AS5600: M4 어깨만 피드백
ESP32-CAM ── HTTP /capture ──> Raspberry Pi (인식·대시보드·ROS2)
                                   └─ M4 제안 → 작업자 확인 → ESP32
```

외부 카메라→Mac 영상·학습 경로는 기존 ESP32-CAM→Pi 경로와 구분한다. 위 그림은 기존 장치 제어 구성이다. 새 M2 행동 예측 모델은 오프라인 학습·평가를 마쳤으며 장치 제어에는 연결하지 않았다.

- 기존 수동 명령은 ESP32의 `Dispatcher`와 `SafetyGate`를 거친다. 실험용 `/actions`는 별도 제한 실행기에서 `SafetyGate`를 확인하고 출력한다. 호스트의 명령 접수를 목표 도달로 취급하지 않는다.
- 카메라 직접 `/stream`은 비활성화돼 있으며 `/capture` 또는 Pi 추적 프레임을 사용한다.
- 전체 관절 폐루프 궤적 제어와 자동 집기는 구현하지 않았다. 간헐 HTTP 지연도 남아 있다.

## 코드와 문서

[전체 문서·실험 증거·미디어 안내](docs/README.md)

| 경로 | 내용 |
| --- | --- |
| `src/`, `firmware/` | ESP32 제어기·카메라 및 STM32 펌웨어 |
| `ros2_ws/src/` | 메시지, 브리지, 제어 가드, 모델, 하드웨어 인터페이스 |
| `learning/`, `tools/` | 인식, 데이터 계약, 오프라인 학습·평가, 운영 도구 |
| [PORTFOLIO.md](PORTFOLIO.md) | 설계 선택, 실패 분석, 조건이 명시된 실측 결과 |
| [OPERATIONS.md](OPERATIONS.md) | 빌드, 서비스 설정, 상태 점검과 진단 |
| [PIN_MAP.md](PIN_MAP.md) | 모터·센서 배선과 부팅 핀 제약 |
| [ROS2 하드웨어 인터페이스](ros2_ws/src/motionbrain_hardware_interface/README.md) | dry-run·측정 상태·M4 제안 모드 |
| [시각 행동 학습](docs/visual-action-learning.md) | 현재 목표·단계·새 관측 계약 설계와 기존 구현 구분 |
| [기존 데이터·학습 파이프라인](docs/data-pipeline.md) | 기존 M4·물체 박스 계약, 적격성, 분할과 CLI 재현 |
| [제한된 CV 개발 학습 실행기](docs/bounded-cv-training.md) | 명시적 실행·로컬 가중치·횟수/시간 제한·실패 기록. 개발용 학습과 일반화 성능 검증 구분 |
| [대상 물체·집게 오프라인 CV](docs/offline-cv.md) | 승인 데이터 로더·선택적 PyTorch 입력 어댑터, 후보 표시, 저장 예측의 검출·방향·상대 위치 평가 |
| [저장 체크포인트 평가 실행기](docs/checkpoint-evaluation.md) | 가변 개수의 확정 이미지, 엄격한 모델 복원·시간 제한·실패 분모 보존. 소프트웨어 검사와 실제 모델 평가 결과를 구분 |

## 로컬 빌드와 검사

Python 호스트 의존성은 `numpy`, `opencv-python-headless`이며 브라우저 복구 회귀에는 Node.js(CI: 22)가 필요하다. 네이티브 C++ 검사를 실행하려면 C++ 컴파일러도 필요하며, 없으면 해당 검사는 건너뛴다. 펌웨어 빌드에는 PlatformIO가 필요하다. ROS2와 서비스 설치는 [운영 안내](OPERATIONS.md)를 따른다.

```sh
python3 -m unittest discover -s tests
pio run
pio run -d firmware/esp32cam
```

[PlatformIO/Python CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/platformio.yml) · [ROS2 CI](https://github.com/ParkJsoo/motionbrain/actions/workflows/ros2.yml). 빌드·테스트 결과는 커밋별로 확인하며 실물 성능과 구분한다.

## 라이선스

프로젝트 코드는 [MIT](LICENSE)다. STM32 HAL/CMSIS 외부 제공 파일은 `firmware/stm32/MotionBrainSensor/Drivers/`의 원래 라이선스를 따른다.
