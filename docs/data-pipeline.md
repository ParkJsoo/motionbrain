# 행동 데이터와 오프라인 학습

[README](../README.md) · [운영 안내](../OPERATIONS.md)

이 파이프라인은 행동 전후 영상·M4 실측·실제 출력 기록을 검증하고, 같은 세션이 학습·평가에 섞이지 않게 분할한 뒤 행동 결과를 예측한다. **실물 학습 적격 전이는 현재 0건**이다. 아래 CLI는 오프라인·합성 검증을 재현할 수 있지만, 비공개 실물 원본의 측정 결과까지 저장소만으로 재현할 수 있는 것은 아니다.

## 기록과 실행 경계

| 구성 | 구현 | 핵심 조건 |
| --- | --- | --- |
| 원본·전이 | `learning/schema.py`, `artifacts.py`, `provenance.py` | 프레임·이미지 해시·행동·출력 이력 연결. 스키마 유효성과 학습 적격성 분리 |
| 제한 행동 실행 | `learning/action_execution.py`, `action_port.py` | 정확히 확인한 요청, 영속 예약, 전송 직전 신선도·저장 가드. 불확실한 결과 재전송 금지 |
| 실물 초기 수집 계약 | `learning/device_capture.py` | 정확한 프레임 GET·원본 저장·독립 M4 조회. 생산자 시계 검증이 없으면 전송 거부 |
| 매니페스트·분할 | `learning/dataset.py`, `splits.py` | 모든 시도·제외 사유 보존, 원본 재검증, 세션·에피소드·프레임·이미지·명령 누수 차단 |
| 특징·모델 | `learning/features.py`, `models.py`, `visual_dynamics.py` | 명령 전에 알 수 있는 입력만 사용, train 전용 정규화, 별도 평가 |

MCU `/actions`는 기본 비활성화(`MOTIONBRAIN_ENABLE_BOUNDED_ACTIONS=0`)다. 부팅 ID와 명령 ID로 구분하며 활성 행동 하나와 퇴출 없는 32개 기록을 유지한다. 결과 불명·용량 초과는 새 구동을 막는다. 기존 비상 정지 경로는 유지하며, 재부팅이나 기록 삭제로 차단 상태를 자동 우회하지 않는다. 기존 M4 `/shoulder` 전송과 별개의 경로다.

각 호스트 실행기는 지정한 영속 저널을 사용하며 같은 파일의 동시 소유를 거부한다. 서로 다른 저널·호스트·직접 HTTP 요청까지 중복 방지하지는 않는다. 운영 시 제어기당 실행기·저널 하나를 사용한다. 기존 M4 저널의 기본 한도는 대기·소비 기록을 포함해 10,000개다. 재시작 시 대기는 취소되고 예약된 전송은 결과 불명이 된다. 새 제한 행동 실행기는 별도 저널을 사용한다. `request_stop()`은 취소 신호이며 블로킹 I/O를 끊거나 하드웨어 비상 정지를 대신하지 않는다. 출력 기록의 OFF는 소프트웨어 쓰기 근거이며 전기적 차단·물리적 정착을 뜻하지 않는다.

실물 초기 수집은 원본 나이 100ms, M4 상태 나이 20ms, 영속 저장 250ms를 요구한다. 출력 OFF 뒤 최대 500ms·여섯 샘플에서 50ms의 M4 안정 구간을 확인하고 마지막 영상 뒤 출력·센서를 다시 검사한다. 이 계약의 `physicalSettledVerified`와 `trainingEligible`은 false로 유지하며 전체 팔의 정착을 추론하지 않는다. 실물 실행 CLI는 제공하지 않는다.

## 영상 시각과 관측 품질

ESP32-CAM은 부팅 ID와 첫 DMA 시각으로 원본 버퍼를 식별한다. 인식 서비스의 `frameId` 조회는 정확한 JPEG·메타데이터를 반환하고 만료 ID는 404로 거부한다. 최신 영상으로 대체하지 않는다. HTTP 수신 시각·호스트 callback·PTS를 검증된 광학 노출 시각으로 대신할 수 없다.

영상 나이 **100ms**, 의미 기반 앵커 나이 **15초**, 원본 영속 저장 **250ms**를 유지한다. 시계 변환의 오차 한도와 경과 시간을 포함해 소비 직전에도 검사한다. 카메라 구도·물체·장착이 바뀌면 새 view/session이며 과거 앵커·보정·학습 그룹을 재사용하지 않는다. ROI 외관 일치만으로 같은 물체의 의미·위치를 검증했다고 볼 수 없다.

2026-10-15 관측 경로의 대표 결과:

| 검사 | 결과와 해석 |
| --- | --- |
| 같은 원본 108쌍의 ROI 계산 | 병렬화 전후 중앙값 33.59→21.25ms, 계산 결과 동등. 전체 촬영→소비 지연과는 다름 |
| 원본 저장 경로 | 12장 저장 중앙값 42.72→34.93ms, `fsync` 유지. 간헐 최악값까지 보장하지 않음 |
| 시계 가정을 바꾼 소비 검사 | 기존 가정의 21/24와 이를 제거한 5/24는 다른 조건. 같은 실험의 개선 성공률이 아님 |
| 저장 장애 주입 | 세 실패 사례에서 각각 후속 촬영 0회 |
| 최종 유한 관측 | 20요청 계획 중 7실행·1성공, 저장 268.89ms로 중단. 사용성 기준 실패 |

[관측 시험의 분모·실패 조건](evidence/2026-10-15-bounded-observation.md) · [Mac 카메라 계측](mac-direct-camera.md) · [CAM 통신·부팅 진단](evidence/2026-10-15-camera-boot-watchdog.md)

## 합성 수집과 매니페스트 재현

저장소 루트에서 새 출력 경로를 사용하고 `<...>`는 실제 경로로 바꾼다. 원본과 매니페스트를 함께 보존하며 기존 실행을 덮어쓰지 않는다.

```sh
python3 tools/capture_action_transitions.py --dry-run
python3 tools/capture_action_transitions.py --audit-run '<saved-run>'
python3 tools/build_action_dataset.py --runs '<saved-run>' --output-dir '<new-manifest-dir>'
python3 tools/build_action_dataset.py --validate '<new-manifest-dir>/manifest.json'
python3 tools/build_action_dataset.py --validate '<new-manifest-dir>/manifest.json' --require-ready
```

수집기는 합성 JPEG·관측·실행 의도와 부분 실패를 저장하며 실물 실행 스위치가 없다. 원본은 Git에서 제외한 `datasets/`에 둔다. 감사는 행동을 재개하거나 재시도하지 않는다.

`manifest.json`은 모든 시도와 제외 사유를 담는다. `--validate`는 저장된 적격 플래그를 믿지 않고 원본·해시·저널·출처·분할을 재검증한다. 기본 종료 코드 0은 목록 정합성이다. 적격 샘플 0개일 수 있다. 학습에는 `--require-ready`가 필요하며 잘못된 목록이나 빈 실물 train/validation/test 분할은 종료 코드 2로 거부한다. 출처 선언과 SHA-256은 하드웨어 인증이나 원본 전체 변조 방지가 아니다.

## 모델과 평가 계약

특징 버전은 `motionbrain.planned_visual_m4.v1`, 모델은 NumPy CPU 선형 ridge와 무변화 기준선이다. `alpha=1.0`으로 고정하며 test 결과로 모델을 선택하지 않는다.

- 입력 16개: before 박스의 정규화 중심·크기, M4 각도의 sin/cos, 요청 축 one-hot 5개, 어깨 명령 여부, 요청 PWM/255·기간초·목표와 before M4의 최단 각도차/180·percent/100.
- 목표 5개: after−before 박스 중심·크기 변화, M4의 최단 부호 각도 변화 `[-180,180)`. 180° 이상 회전·다회전 복원은 지원하지 않는다.
- 실제 출력·소요 시간·종료 상태·after 관측은 예측 입력에서 제외한다. 출력 이력은 행동 근거 검증에 사용한다.
- 전후 유효 박스, 동일 크기·label·classId, M4 실측을 요구한다. 누락을 0이나 화면 중앙으로 채우지 않는다. label 일치만으로 물체 인스턴스가 같다고 볼 수 없다.
- 평균·표준편차·모델은 train으로만 적합한다. 상수 열 scale은 1.0, intercept는 train 목표 평균이다.
- 학습 CLI는 test 점수를 계산하지 않는다. 평가 CLI가 고정 모델·동일 매니페스트·분할·코드·runtime으로 분할별 MAE/RMSE와 분모를 기록한다. 시각값과 각도를 하나의 점수로 합치지 않는다.

```sh
python3 tools/train_visual_dynamics.py --manifest '<manifest.json>' --output-dir '<new-training-dir>'
python3 tools/evaluate_visual_dynamics.py --manifest '<manifest.json>' --model '<new-training-dir>/model.json' --output-dir '<new-evaluation-dir>'
python3 tools/smoke_visual_dynamics.py --output-dir '<new-smoke-dir>'
```

모델 자료에는 특징·단위, seed, 원본·분할·학습 행렬·소스 해시, Python/NumPy 버전, 정규화 통계, 계수, alpha, 적격·제외·분할 개수를 남긴다. 다른 매니페스트·코드·runtime 또는 합성 모델을 실물 평가에 넣으면 거부한다. 거부는 `refusal.json` 또는 `unavailable.json`과 종료 코드 2로 기록한다.

합성 점검은 고정된 수학적 fixture 120행을 train94/validation11/test15로 나눈 결과다. 실물 동역학 모사가 아니며 `trainingEligible=false`, `controlEligible=false`다. 실물 학습 적격 행을 성공적으로 학습·평가하는 경로와 조작 개선은 아직 검증하지 않았다. 수동 파지 자료도 별도 원본 보관 형식이며 이 계약을 통과한 전이로 취급하지 않는다.
