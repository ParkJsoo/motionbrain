# 기존 행동 데이터 계약과 오프라인 학습

[README](../README.md) · [운영 안내](../OPERATIONS.md)

이 파이프라인은 행동 전후 영상·M4 실측·실제 출력 기록을 검증하고, 같은 세션이 학습·평가에 섞이지 않게 분할한 뒤 행동 결과를 예측한다. **실물 학습 적격 전이는 현재 0건**이다. 아래 CLI는 오프라인·합성 검증을 재현할 수 있지만, 비공개 실물 원본의 측정 결과까지 저장소만으로 재현할 수 있는 것은 아니다.

## 현재 연구 방향과의 관계

현재 주력 목표는 [영상과 명령으로 배우는 동작 모델](visual-action-learning.md)이다. 이 문서는 기존 `motionbrain.planned_visual_m4.v1`과 실제 구현된 CLI의 재현 안내다. 기존 입력은 물체 박스와 필수 M4 실측이며, 새 팔 특징점·이력 모델과 다르다.

여기서 대상 물체는 기존 계약의 고정 라벨 `lipbalm`을 가리키며, 임의의 물체 라벨을 허용하지 않는다.

아래 100ms/20ms 등 조건은 기존 계약에 적용된다. 새 저속 행동·정착 관측 수집기는 별도 버전으로 설계 중이며, 이 문서나 현재 CLI가 이미 그 경로를 제공하는 것은 아니다. 새 계획을 위해 기존 가드나 과거 자료의 적격 판정을 변경하지 않는다.

## 기록과 실행 경계

| 구성 | 구현 | 핵심 조건 |
| --- | --- | --- |
| 원본·전이 | `learning/schema.py`, `artifacts.py`, `provenance.py` | 프레임·이미지 해시·행동·출력 이력 연결. 스키마 유효성과 학습 적격성 분리 |
| 제한 행동 실행 | `learning/action_execution.py`, `action_port.py` | 정확히 확인한 요청, 영속 예약, 전송 직전 신선도·저장 가드. 불확실한 결과 재전송 금지 |
| 실물 초기 수집 계약 | `learning/device_capture.py` | 정확한 프레임 GET·원본 저장·독립 M4 조회. 생산자 시계 검증이 없으면 전송 거부 |
| 수집 근거 사전 검토 | `learning/collection_readiness.py` | 저장된 전이와 대상 물체 앵커·광학 시각·원본 저장 영수증·시각 정착·OFF 이후 출력 연결. 학습 승격·실물 실행 없음 |
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

## 수동 시연에서 수집 계약으로 연결하기

수동 시연의 원본 보관 형식과 제한 행동 전이 형식은 다르다. 이전 단계의 사후 이미지를 다음 단계의 사전 이미지로 바꾸거나, 수동 명령의 PWM·시간을 `/actions` 실행 결과로 바꾸지 않는다.

| 필요한 근거 | 수동 시연 자료의 누락 | 오프라인 검토에서 요구하는 연결 |
| --- | --- | --- |
| 전후 영상 | 비동기 사후 스냅샷만 보관 | 전이·프레임 ID와 이미지 SHA-256 일치, 행동 전·OFF 후 촬영 순서 |
| 광학 시각 | PTS·callback·HTTP 수신 시각으로 노출을 입증하지 못함 | 같은 session/view/카메라 boot에 적용되는 광학 계측 자료·유효 구간·검토자. 불확실성을 포함한 노출 구간 |
| M4 실측 | 이미지와 독립적으로 읽은 상태 | 원래 센서 샘플 구간·출처를 보존. 소비 및 행동 직전까지 20ms 검사 |
| 대상 물체 앵커 | 의미·인스턴스·구도에 고정된 앵커 없음 | `lipbalm`·물체 인스턴스·새 view·사람의 박스 검토·원본 이미지 연결. 모든 사용 시점까지 15초 |
| 원본 저장 | 사후 아카이브 복사 시간만으로 입증 불가 | 원래 촬영 트랜잭션의 시작·완료·파일 및 디렉터리 fsync·재읽기 영수증. 250ms 이내, 소비 전 완료 |
| 정착·결과 | OFF와 최종 한 장만으로 전체 팔 정착 입증 불가 | OFF 후 서로 다른 3–6프레임, 최소 50ms 구간의 M4 실측, 이후 최종 영상. 전체 팔·물체의 시각 정착과 작업 성공/실패/불명을 별도 기록 |
| 실제 출력 | manual lease이며 제한 행동 ordinal과 다름 | 기존 행동 스키마 검사와 최종 영상 소비 이후 읽은 출력 이력. OFF 기준 행, 연속 sequence·시각, 누락 0·재출력 없음 |

새 CLI는 **이미 저장된 파일만** 읽는다. 아래 경로는 실제 저장 경로로 바꾸며 결과는 Git 제외 경로에 쓴다. `--output-dir`은 새 디렉터리여야 한다.

```sh
# 수동 시연: 모든 시도의 분모·실패·영상 유무·누락 사유를 보존
python3 tools/review_collection_readiness.py '<saved-demo>' --demo --output-dir '.local/<new-demo-review>'
# 초기 수집: 저장된 DeviceCaptureSession만 감사, 시계 변환이나 전이 변환 없음
python3 tools/review_collection_readiness.py '<saved-device-capture>' --device-capture
# CaptureRunner 결과에 연결할 미작성 scene-template.json과 차단 보고서 생성
python3 tools/review_collection_readiness.py '<saved-run>' --transition 'attempts/<transition-id>.json' --output-dir '.local/<new-review>'
# 근거가 채워진 scene JSON 재검사. 모든 참조는 saved-run 내부의 상대 경로
python3 tools/review_collection_readiness.py '<saved-run>' --transition 'attempts/<transition-id>.json' --scene '<scene.json>'
```

`scene-template.json`의 미확인 항목은 `null` 또는 빈 목록이다. 자동으로 과거 구도·박스·앵커를 채우지 않는다. `schemaVersion`은 `motionbrain.collection_scene.v1`이며 다음 구조를 사용한다. 완전한 **합성** 예시는 `tests/test_collection_readiness.py`에 있다.

- 상위: 전이 파일의 `{artifactRef, sha256, bytes}`, `transitionId`, `sessionId`, `viewId`, `objectInstanceId`, `targetLabel=lipbalm`, 고정 `policy`, 명시한 `m4ToleranceDegrees`(0–1°). 허용오차는 현장 검증을 대신하는 기본값이 없다.
- `anchor`, `before`, `after`, `settling[]`: 동일 session/view/물체·프레임 ID·카메라 boot·`capturedUs`, 이미지 descriptor, 이미지 크기·박스, 검토자와 `annotationMethod=human_semantic_review`. `exposureHostBounds={clockId,earliest,latest}`는 불확실성이 포함된 호스트 단조 시각(초)이며 원래 수집기의 구간보다 좁힐 수 없다. `consumedMonotonic`은 같은 시계다.
- `opticalCalibration`: JSON descriptor. 그 원본에는 session/view/카메라 boot, `method=optical_exposure_bounds`, `opticalExposureVerified=true`, 검토자, 원본 `measurement` descriptor, `validHostBounds`가 필요하다. 문자열 선언만으로 광학 검증이 성립하는 것은 아니다.
- 각 프레임의 `persistence`: `basis=original_capture_transaction`, 같은 `clockId`, 시작·완료 단조 시각, 이미지 descriptor·프레임 ID, `fileFsynced`, `directoryFsynced`, `readbackVerified`, 이 값들을 담은 원래 JSON의 `receipt` descriptor. 감사·복사 시간을 대신 넣지 않는다.
- 앵커 외 프레임의 `m4`: 센서 JSON descriptor. `motor=4`, `sensor=AS5600`, `measured=true`, `angleDegrees`, `sampledHostBounds`가 필요하며 전후 프레임은 원래 전이의 M4 기록과 같아야 한다. 정착 샘플 구간도 중복 없이 50ms 이상이어야 한다.
- `visualReview`: 전이 descriptor·session/view/물체 ID·프레임별 이미지 descriptor를 고정한 JSON. 검토자, `method=human_sequence_review`, `wholeArmStationary`, `objectStationary`, `taskOutcome=success|failure|unknown`을 별도로 기록한다.
- `finalOutputLog`: 기존 `driver_write` 이력에 조회 요청의 `readHostBounds`를 함께 저장한 JSON descriptor. 촬영 시각과 MCU `atMs`를 직접 비교하지 않는다.

`recordedEvidenceComplete`는 기록 간 일관성 검사이며 장치 인증·광학 계측의 진위·물리적 정착을 보증하지 않는다. **`trainingEligible`, `controlEligible`, `executionAllowed`는 항상 false**다. 합성 출처·알려진 합성 이미지도 차단한다. 종료 코드 2는 누락·불일치·합성 자료이며, 코드 0도 학습·구동 허가가 아니다. 기존 데이터셋 빌더로 자동 전달하거나 적격 전이로 변환하지 않는다.

Mac 진단 생산자는 PTS와 센서 폴링 표식의 호스트 시계 구간을 연결할 수 있다([계측 조건과 실측](mac-direct-camera.md)). 광학 노출·AS5600 물리 샘플 시각·대상 물체 의미 앵커·다중 시각 정착 자료는 아직 검증하지 못한다. `DeviceCaptureSession`의 독립 M4 조회와 영상은 시계 영역을 그대로 보존하므로 이 도구가 동시 측정으로 재해석하지 않는다. 이 기존 계약으로 적격 전이를 생성하려면 해당 계측·생산자 연결과 원본/저널/출처/분할 검증을 모두 충족해야 한다. 이번 사전 검토 도구의 합성 검증이나 기존 21단계 아카이브 무결성 통과로 실물 학습 적격 수가 증가하지 않는다.

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
