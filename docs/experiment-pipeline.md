# 반복 수집·관측·학습 파이프라인

단발 수집과 사후 처리를 분리한다. 수집기는 정해진 사전·사후 구간의 원본을 먼저
닫는다. 이후 관측기가 점을 찾지 못해도 원본의 수집 결과를 바꾸지 않는다.
관측 계약이나 주석을 수정한 뒤 같은 원본을 다시 처리할 수 있다.

원본 재처리 경로는 개발 재생용이며, 아래 M1 shaft 경로는 사전 예측을 남기는
고정 후 평가를 별도로 지원한다. 모델을 생성했다는 이유로 실물 제어에 적격하다고
판단하지 않는다. 장치 연결·촬영·실물 반복 운영의 검증은 소프트웨어 검사 및 과거
자료 재생과 구분한다.

## 전체 관절 계획과 구현 범위

이 설명서는 공통 수집·감사·재처리와 개별 관측·모델·실행기를 설명한다. 전체 M1~M5의 수집 조건·학습 순서·완료 판단은 [전체 팔 학습 계획](visual-action-learning.md#전체-관절의-수집학습-계획)에 둔다. 날짜별 결과나 실제 실행 허용을 이 계획으로 대체하지 않는다.

| 경로 | 재사용할 구현 | 현재 한계·확장 시 확인할 것 |
| --- | --- | --- |
| 공통 단발 수집 | `collect_pipeline_episode.py` → `pipeline_collection.collect_once` → `ManualCollection`의 원본·journal·OFF 보존 | M1~M5 선택과 모든 방향의 허용은 다름. 일반 M4 수집의 아래 방향·기록당 1회 제한은 유지하며 위 방향은 전용 경로 사용 |
| 관측·재처리 | `run_learning_pipeline.py`, `pipeline_observations`의 등록 관측·수동 주석과 해시 감사 | 임의의 전체 팔 상태 벡터를 자동 처리하는 범용 관측기는 아님. 턱끝·본체 기준 이동 등 새 상태는 계약과 어댑터 보강 필요 |
| 학습·개발 비교 | `pipeline_learning`의 관절·조건별 분할, 상태 ridge·명령 평균·무변화 비교 | 재생은 개발 평가. 모든 관절의 센서 정답·관측기·고정 후 평가가 자동 완성된 것은 아님 |
| 묶음 운영 | `batch_campaign`·`batch_runtime`의 영속 진행·조건 격리·재개 | 모든 관절의 일반 지원 방향을 대상으로 하며 실제 구도·표식 검증이 필요. 접촉과 M4 위 방향은 이 어댑터로 실행하지 않음 |
| 사전 예측·고정 후 평가 | 수집 전 `before_command` 콜백, M1 shaft campaign의 고정·사전 기록 | 콜백은 예약 명령을 바꾸지 않음. shaft campaign은 1차원 M1 전용이며 다른 관절·새 상태에 그대로 사용하지 않음 |
| 후보 선택·순차 실행 | `model_grasp_trial`·`close_lift_trial`의 고정 모델·접촉 검토·정지 검사 | 새 관절·상태의 연결과 적용 범위를 각각 검증해야 함. 코드 존재가 전체 팔 순차 집기 성공의 증거는 아님 |

각 관절 자료는 초기 상태, 다른 관절 자세, 물체·하중, 구도·관측 계약, 예정 명령을 결합해 관리한다. 센서 없는 관절 각도나 가려진 턱끝을 임의로 채우지 않는다. 새 상태 도입 시 관측→전이→학습→범위 검사→사전 예측→후보 선택의 연결과 실패 경로를 검증한다. 기존 출력·방향 제한을 문서상의 계획으로 넓히지 않는다.

## 전체 관절 묶음 수집 실행기

`learning/batch_campaign.py`는 M1~M5의 조건을 순차 선택하는 유한 실행기다. 단발 실행은 기존 `collect_once`·`ManualCollection`·`SequencePort`를 재사용한다. 자동 조건 순회는 수집 정책이며 학습 모델이 목표 동작을 선택한 성과가 아니다. 현재 실제 장치 어댑터는 물체 없는 무부하 조건을 대상으로 한다. 접촉·파지·들기 검토를 자동 통과시키지 않으며, 일반 경로가 지원하지 않는 M4 위 방향은 미지원 조건으로 사전 분리한다.

### 계획과 진행 관리

비공개 계획의 형식은 `motionbrain.batch_campaign.v1`이다. 다음을 실행 전에 고정한다.

| 필드 | 내용 |
| --- | --- |
| `campaignId`, `viewId`, `controllerBootId` | 캠페인·촬영 구도·실제 부팅 식별 |
| `nodes` | 관측기로 구분하는 상태 구간 식별자. 관절각이나 완전한 3차원 자세의 보증이 아님 |
| `maxObservationAgeSeconds` | 실행에 사용할 관측의 신선도 한도 |
| `budgets` | `maxPhysicalCommands`, `maxPreparationCommands`, `maxAttempts`, `maxSeconds` |
| `conditions` | 각 조건의 `id`, `command`, `fromNodes`, `phase`, `captureGroup`, `target`, `maxAttempts` |
| `runtime` | 카메라 도구·관측 명세·원장·제어기 연결 설정. 실제 주소와 실행 계획은 비공개 보관 |

`phase`는 `training`, `preparation`, `evaluation`이다. 준비 동작도 명령 예산을 소비하지만 학습 전이 수에는 넣지 않는다. 예약은 실행 전에 SQLite에 저장한다. 결과 보고의 `reservedCommands`는 확정 미구동·불명 시도도 포함하는 보수적인 예산 사용량이며 실제 완료 실행 수와 다르다. 유효 전이 목표를 채우려고 무제한 동작하지 않는다.

상태 관측 후 현재 `fromNodes`에 맞는 부족 조건을 선택한다. 준비·되돌림도 명시한 조건이어야 하며 반대 방향·같은 시간으로 원상복구됐다고 가정하지 않는다. 연결 가능한 조건이 없으면 이유를 남기며 임의 복구 명령을 만들지 않는다. 일부 조건의 관측 실패는 그 조건을 격리하고 다른 실행 가능한 조건으로 진행한다. 부팅·출력·실행 여부 불명은 다음 구동을 막는다.

원본·실행·OFF·관측은 별도 상태로 저장한다. 재시작하면 미완료 예약의 기존 원본과 실행 증거만 대조하고 재전송하지 않는다. 확정할 증거가 없으면 보류한다. 같은 제어기의 캠페인은 실행 잠금을 공유하며, 원본·세션·명령·프레임의 중복으로 유효 수량을 늘리지 않는다. 시간 한도는 준비가 끝난 뒤 최종 RUN 직전에도 검사한다.

### 자동 관측과 학습

[색 표식 관측기](../learning/batch_observation.py)는 명시한 색상·면적 구간의 연결 영역으로 표식 중심을 찾는다. 여러 후보·누락·화면 경계 잘림·색상 중복은 거절한다. 관절별 상태는 표식 간 XY·거리이며, 전체 자세를 구별하기 위해 선언한 여러 팔 표식의 XY를 각 관절 상태에 포함한다. 고정 배경 표식 최소 두 개의 절대 영상 위치도 검사해 등록 구도의 이동을 거절한다. 색 표식의 실제 링크 배치·가림·불확실성은 현장에서 따로 검증해야 한다.

특정 관절만 사용하는 표식의 가림은 그 관절 상태를 반려하며 다른 유효 관절 상태는 유지한다. 공통 자세·배경 표식의 실패는 해당 관측 전체를 반려한다. 노드 일치는 모든 특징의 선언 불확실성 구간이 해당 범위 안에 들어갈 때만 인정한다. 이것이 깊이·접촉·전체 자세·이동 여유의 물리 보증은 아니다. 턱 끝을 측정하려면 실제 턱 끝을 나타내는 표식을 사용해야 하며 턱축을 같은 의미로 바꾸지 않는다. 새 물체·구도에 과거 모델을 자동 적용하지 않는다.

[관절별 학습기](../learning/batch_learning.py)는 같은 관절·관측 계약의 임의 차원 상태 전이를 학습한다. 명령별 최소 두 전이의 ridge, 명령 평균, 무변화를 비교하고 모델을 고정한다. 학습·평가의 원본·세션·동작·프레임·그룹 중복을 거절한다. 학습 상태의 성분별 최소·최대 안에 있다는 사실만으로 모든 상태 조합이 충분히 관측됐다고 보증하지 않는다. 모델 파일 생성이나 낮은 재생 오차를 제어 적격·집기 성공으로 표시하지 않는다.

학습 입력 프레임은 실행 전에 관측·저장한 프레임으로 고정한다. 원본 전체 감사 통과에 더해 그 프레임이 감사된 사전 구간 안에 있고, 저장의 보수적 가용시각이 명령보다 앞서는지 확인한다. 이는 공통 감사기가 기본 선택한 마지막 입력 프레임과 다를 수 있는 별도 계약이며 원본의 기본 입력을 덮어쓰지 않는다. 평가에서는 그 프레임·모델·예정 명령의 예측을 실행 전에 저장하고 사후에 바인딩을 대조한다.

오프라인 학습에는 `motionbrain.batch_training_manifest.v1`의 `training`·`evaluation` 목록을 사용한다. 각 항목에 `transitionPath`, `transitionSha256`, `sourcePath`, `sourceSha256`를 명시하며 모든 경로는 `.local/` 안에 둔다. `training` 대신 `frozenModels: [{"path": "...", "sha256": "..."}]`를 주면 재학습 없이 같은 평가 자료를 비교한다.

```bash
python3 tools/train_batch_campaign.py --manifest .local/pipeline/batch-training.json \
  --output .local/pipeline/batch-training-output
```

전이 파일과 원본을 다시 감사하고 관절·관측 계약별 모델을 새 출력 폴더에 저장한다. 이 CLI의 사후 비교는 `developmentReplay`이며 원래 고정 후 평가 증거를 새로 만들어 내지 않는다. 실제 고정 후 평가에는 수집 실행기의 사전 예측 증거가 별도로 필요하다.

### 실행과 검증 경계

```bash
python3 tools/run_batch_campaign.py --plan .local/pipeline/batch-plan.json
```

기본 명령은 계획·파일·관측 설정을 검사하며 키체인·카메라·제어기에 접근하지 않는다. 실제 실행은 현재 현장 조건과 명시한 범위가 확인된 뒤 `--execute`, `--onsite-confirmed`, 검사한 `--plan-sha`를 지정한다. 기존 macOS 키체인의 명시한 항목을 메모리로 읽으며 값은 계획·인수·결과에 기록하지 않는다. 실행 전 조건 검토를 매 동작 채팅 응답으로 대신하지 않는다.

묶음 실행기·색 관측·학습의 모의 검사는 다섯 관절의 수량 관리, 관측 반려 후 다른 조건 진행, 중복·재개·시간 한도·원본 누출을 검증한다. 모의 장치의 180개 전이 완주는 실제 180회 구동이나 새 구도의 자동 관측 정확도 검증이 아니다. 실물 수집은 작은 승인 묶음에서 시작해 유효 전이율·준비 비용·실패 조건을 확인한 뒤 확대한다.

## 상태를 구분하는 방법

| 단계 | 판단 내용 | 불합격 처리 |
| --- | --- | --- |
| 실행 증거 | 명령·실제 출력·단일 관절·OFF·빈 큐·boot·HTTP 순서 | 미실행과 결과 불명을 구분하며 재전송하지 않음 |
| 원본 녹화 | 모든 프레임·메타데이터 해시, 시각, 종료 기록, 누락 | 실행 증거를 지우지 않고 녹화 상태만 거절 |
| 관측창 | 명령 전 사용 가능했던 영상과 OFF 후 고정 구간 | 자료 부족을 표시하고 보간하지 않음 |
| 센서 | 각 창의 유효 표본·원본 시각 범위 | 영상 전이와 별도로 거절 사유 기록 |
| 관측 | 두 프레임의 물체·양턱과 동일한 관측 계약 | 부분점·모호함을 기록하고 학습 전이 생성 보류 |
| 학습·평가 | 구도·관절별 학습, 명시적 분할, 기준선·이전 모델 비교 | 부족한 명령·범위·평가 자료를 각각 보고 |

기존 실패 원본은 실패로 보존한다. `allowFailedDevelopment: true`는 정상적인
출력·영상 감사를 다시 통과한 자료를 **개발 재가공**에 쓰는 옵션이다. 부족한
사후 녹화나 잘못된 픽셀을 복구된 것처럼 취급하지 않는다.

## 단발 원본 수집

기존 [수집 경로](manual-data-collection.md)와 [Mac 카메라 도구](mac-direct-camera.md)를
사용한다. `tools/collect_pipeline_episode.py`는 기본 실행에서 계획만 검사한다.
계획은 다음 필드를 갖는 비공개 JSON이다.

```json
{
  "schemaVersion": "motionbrain.pipeline_collection_plan.v1",
  "helperPath": ".local/bin/mac-camera-stream",
  "helperSha256": "실제 도구 파일의 SHA256",
  "controllerUrl": "컨트롤러 HTTP origin",
  "deviceId": "선택한 외부 카메라 ID",
  "viewId": "고정 구도 식별자",
  "sessionRoot": ".local/pipeline/sessions",
  "requestRoot": ".local/pipeline/requests",
  "poseContext": "실제 고정 자세·물체·부하 조건",
  "command": {"joint": "gripper", "direction": "close", "percent": 40, "durationMs": 350}
}
```

예시 문자열은 실제 값으로 바꾼다. 계획 검사에는 모터·카메라 접근이 없다.

```bash
python3 tools/collect_pipeline_episode.py --plan .local/pipeline/collection-plan.json
```

실제 단발 수집은 현재 현장 조건을 확인한 뒤 별도로 실행한다.

```bash
python3 tools/collect_pipeline_episode.py \
  --plan .local/pipeline/collection-plan.json --plan-sha '<검사한 계획 SHA256>' \
  --request-id '<이 단발의 고유 ID>' --execute --onsite-confirmed
```

인증값은 비공개 TTY에서 입력하며 계획이나 명령 인수에 넣지 않는다. 같은 요청 ID는
동일 결과 또는 미완료 예약을 반환하고 다시 움직이지 않는다. 같은 ID에 다른 명령을
넣으면 거절한다. 실패·불명 동작을 새 ID로 자동 대체하는 반복 기능은 없다.

현재 구현은 수동 자료 수집용이다. 사전 3.5초, OFF 이후 6.5초까지 수집을 유지하며
기존 출력·방향·저장·정지 검사와 공유 실행 journal을 사용한다. 모델 선택 제어기와
접촉·깊이·파지 성공 판단을 대신하지 않는다.

집게 카메라 전후 사진이 필요하면 계획에 `gripper: {"url": "카메라 HTTP origin",
"bootId": "확인한 카메라 부팅 ID"}`를 추가한다. 카메라의 지원 기한 6초로
사전 사진을 저장한 뒤 3.5초 동안 센서와 외부 영상을 수집하고 명령을 실행한다.
OFF 후 6.5초 동안 센서와 외부 영상을 수집한 다음 사후 사진을 요청한다. 외부
녹화는 사후 사진 요청 종료까지 유지해 느린 요청이 센서의 고정 관측창을 막지 않는다.
사전 사진 실패는 구동 전에 멈추고, 사후 사진 실패는 별도 상태로 보존하며 외부
녹화를 조기 종료하지 않는다. 원본은 세션의 `gripper-camera/`에, 기존
`bind_gripper_pair` 입력은 `gripper-pair/`에 같은 세션 ID와 원본 메타데이터로 남긴다.
두 카메라의 노출 동기화를 증명하는 것은 아니다. 관측 manifest에는 해당
`gripper-pair/`의 asset 루트 상대 경로를 `observer.gripperPairRoot`로 지정한다.

Python API의 선택적 `before_command(manager, gripper_pair)`는 사전 수집 구간이
끝난 뒤 실제 명령 직전에 호출한다. 관측·고정 모델·예측의 해시와 시각을 먼저
저장하는 용도이며, 반환값으로 예약된 명령을 바꾸지 않는다. 예외가 발생하면
구동 전에 중단하고 원본·요청 결과를 보존한다. 콜백 사용 여부도 요청 해시에
결합하며, 기존 콜백 없는 요청의 중복 조회 형식은 유지한다.

## 원본 재처리

`tools/run_learning_pipeline.py`에 manifest 하나와 비공개 출력 경로를 준다.

```bash
python3 tools/run_learning_pipeline.py \
  --manifest .local/pipeline/manifest.json --output .local/pipeline/output
```

manifest의 최상위 필드는 다음과 같다.

| 필드 | 의미 |
| --- | --- |
| `schemaVersion` | `motionbrain.experiment_pipeline.v1` |
| `artifactRoot` | 원본·등록·참조 파일을 읽을 명시적 루트. 비공개 자료는 `.local` 디렉터리를 루트로 지정 |
| `sources` | 고유 `id`, 루트 상대 `path`, `session.json`의 `sha256`, 개발 재가공 옵션, `observer` |
| `split` | `trainGroups`, `evaluationGroups`, 개발 재가공 학습과 OOD 진단의 명시적 옵션 |
| `previousModels` | 선택 사항. 비교할 기존 모델과 canonical JSON SHA256 |

source 경로는 정규화한 뒤 `artifactRoot` 내부인 경우에만 허용한다. 관측 asset의
상대 경로에서는 `..`와 심볼릭 링크 우회를 거절한다. 누락·변조된 source도 보고서
분모에서 삭제하지 않는다. 매 실행에서 원본을 다시 감사한다.

관측기의 `kind`는 `registered_scene`, `registered_rigid`, `registered_gripper`,
`registered_gripper_crosssection`, `analyst_points`다. 등록 관측은 등록·참조 파일
경로와 각각의 SHA256을 요구한다.
`context`에는 실제 `physicalView`, `object`, `poseRegime`을 지정하고 `captureGroup`으로
분할 그룹을 연결한다. M4 전용 강체 턱 관측을 M1 명령에 사용하면 거절한다.

`registered_gripper_crosssection`은 320×240 집게 영상의 고정된 상부 단면에서
양쪽 턱축을 각각 관측하는 M1 전용 계약이다. 턱끝의 작은 템플릿에 물체가 겹치는
문제에 대응하며, 어두운 영역의 양쪽 경계·폭·여러 행의 일관성을 검사한다.
복수 후보·대비 부족·가림은 반대쪽 턱이나 고정 간격으로 보충하지 않는다.
세 번째 상태는 `shaftSeparationPx`이며 기존 `jawOpeningPx`와 구분한다.
이 간격은 영상 단면의 턱축 중심선 사이 거리로, 턱끝 간격·물리 개구폭·접촉·
안전 여유가 아니다. 기존 관측 모델과 자동 혼합하지 않으며 M4에는 사용하지 않는다.

수동 주석은 별도 관측 계약이다. 각 프레임의 `frameId`, `pixelSha256`,
`sourceFrameIds: [그 프레임 ID]`, `inputFrameOnly: true`, `accepted: true`,
`method: "analyst_points"`, 물체·양턱의 세 점을 요구한다. 자동 관측 결과와 같은
계약인 것처럼 섞지 않는다. 출처 선언이 분석가의 물리 계측 정확도 증명은 아니다.

카메라 이동·관절 고정 상태·물체·부하·점의 의미가 바뀌면 구도와 관측 계약을
다시 선언한다. 장치 ID나 영상 크기가 같다는 사실만으로 같은 구도로 간주하지 않는다.
서로 다른 관측 계약과 관절은 별도 모델로 분리한다. 같은 동작의 두 카메라는 같은
분할 안에서 각각 사용할 수 있지만 train/eval 사이의 원본·동작·세션·프레임 중복은 거절한다.

## 학습·평가 산출물

`output/latest.json`은 해시가 붙은 보고서를 가리킨다. `artifacts/`에는 단계별
불변 결과를, `models/`에는 canonical JSON 해시로 고정한 모델을 저장한다.
동일 입력과 처리 버전의 재실행은 같은 결과를 만들고 원본·기존 모델을 덮어쓰지 않는다.
수집·관측·학습 코드 및 주요 라이브러리 버전도 보고서에 포함한다.

명령당 두 전이는 학습의 최소 지원 수이며 실물 성능 보증이 아니다. 평가에는
상태 ridge·명령 평균·무변화의 분모를 표시한다. 이전 모델과의 비교는 같은 평가
자료에서 두 모델이 모두 지원하는 사례에 한정한다. 개선·동률·회귀·비교 불가를 구분한다.

기존 예측기의 입력 범위 거절은 유지한다. `allowOodDiagnostics: true`는 범위 밖의
오프라인 예측 오차를 별도로 보여줄 뿐, 제어 지원 범위나 평가 합격을 넓히지 않는다.
사후 재가공의 평가는 `developmentReplay`이며 `evaluationPassed`를 만들지 않는다.
새로운 고정 후 평가에는 실행 전에 고정된 관측·모델·입력·예측의 시간 증거가 별도로 필요하다.

## M1 shaft 전용 수집·고정 후 평가

물체 검출의 실패가 유효한 턱축 관측까지 막지 않도록, 물체 좌표를 요구하지 않는
별도 경로를 제공한다. [관측 연결 모듈](../learning/m1_shaft_observations.py)은
원본 감사와 `RegisteredGripperCrosssection.observe_shaft`의 전후 관측을 연결해
`motionbrain.m1_shaft_transition.v1` 전이를 만든다. 입력·정답은 각각
`beforeShaftPx`, `afterShaftPx`이며, 세션·명령·프레임·원본 해시를 함께 보존한다.
관측 계약은 `registered-gripper-shaft-only-v1:`이고 `stateFields`는
`["shaftSeparationPx"]`다. 물체는 `not_required / not_observed`로 표시하며
누락된 XY에 0을 채우지 않는다. 턱축 자체의 가림·잔차·품질 실패는 계속 거절한다.

[1차원 학습 모듈](../learning/m1_shaft_learning.py)의 `fit`, `predict`, `evaluate`는
현재 고정 구도의 M1 명령과 단면 간격만 다룬다. 명령별 최소 두 전이와 ridge
`alpha=1`을 사용하고, 학습한 shaft 입력의 최소·최대 범위를 유지한다. 물체 상대
XY는 예측하거나 지원 범위를 판단하는 데 사용하지 않는다. 기존 3차원 모델과는
별도 모델이며, 완전한 기존 crosssection 전이를 받더라도 새 scalar 계약과 혼합하지
않는다. 명령 평균·무변화와 같은 평가 사례에서 비교하며, 학습·평가의 그룹·원본
해시·세션·동작·프레임 중복을 거절한다. 단독 `evaluate` 호출은 촬영 시점을 증명하지
않으므로 개발 재생으로 남는다.

[공통 실행기](../tools/run_m1_shaft_campaign.py)는 비공개 계획의 준비 동작·학습·모델
고정·별도 평가 순서를 단발 요청으로 진행한다. 계획에는 명시적 seed 원본과 해시,
관측 spec, 실행 코드 해시, 관절·구도 조건, 명령 목록, 동작 수 제한과
`freezeAfterStep`을 지정한다. 준비 동작은 학습 전이를 만들지 않는다. 고정 시
`runtime/frozen-model.json`을 canonical JSON으로 저장하며, 이후 평가의 관측·예측을
명령 전에 기록한다. 실패·불명·범위 밖 입력은 자동 재시도하지 않는다.

현재 현장 조건과 해당 계획의 실행 범위가 확인된 경우에만 서버를 실행한다.
계획은 새 실험 폴더에 두며, 기존 `runtime/`을 덮어써 재시작하지 않는다.

```bash
python3 tools/run_m1_shaft_campaign.py --config .local/pipeline/m1-shaft/config.json
```

기본 인증은 비공개 TTY 입력이다. macOS에서는 사용자가 키체인에 비공개로 등록한
일반 암호 항목을 `--keychain-service '<service>' --keychain-account '<account>'`로
명시할 수 있다. [인증 모듈](../learning/macos_credentials.py)은 지정한 항목만 읽고
인증값을 명령 인수·계획·로그에 넣지 않는다. 실행기는 localhost에만 바인딩하며
시작만으로 모터 명령을 실행하지 않는다. 상태 조회 후, 응답의 계획 해시와 정확한
다음 번호로 각 단계를 명시적으로 요청한다.

```bash
curl --fail-with-body http://127.0.0.1:8769/
curl --fail-with-body -H 'Content-Type: application/json' \
  --data '{"index":1,"planSha256":"<확인한 계획 SHA256>"}' \
  http://127.0.0.1:8769/step
```

[전용 Rerun 표시기](../tools/visualization/m1_shaft_campaign.py)는 저장된 실행 폴더만
읽는다. 원본 JPEG·프레임·픽셀 해시와 유효한 양쪽 shaft 점을 검증하며, 개발 seed,
준비 동작, 미시작·반려와 평가 분모를 구분한다.

```bash
python3 tools/visualization/m1_shaft_campaign.py \
  --runtime .local/pipeline/m1-shaft/runtime --output .local/pipeline/m1-shaft/review.rrd
```

기본은 새 로컬 RRD 저장이며, `--connect rerun+http://127.0.0.1:9876/proxy`를
명시한 경우에만 로컬 Rerun에 연결한다. 승인된 계획 해시, 명령 전 예측 시각,
원본과 콜백의 프레임·측정값, 고정 모델 해시와 재계산 예측을 확인한 사례만
`prospectiveVerified`에 센다. 이 검증은 모델이 명령을 선택했다는 뜻이 아니다.
실행 당시 코드의 해시는 당시 기록으로 표시하며 현재 파일로 과거 실행을 다시
판정하지 않는다. 한 번의 고정 후 평가가 반복 운영·관측 일반화·파지·들기를
보증하지 않으며, 다른 구도나 관절에는 새 계약과 검증이 필요하다.

## 실제 턱끝과 물체의 기하 검토

[기하 검토 모듈](../learning/grasp_geometry.py)의 `evaluate_geometry(review)`는
원본 프레임 ID·픽셀 SHA256에 결합한 수동 주석을 받는다. 물체 몸통 영역과
실제로 보이는 두 턱끝의 좌표·가시성·선언한 픽셀 불확실성을 기록하며,
shaft 점이나 추정한 턱끝을 실제 턱끝으로 받아들이지 않는다.

출력의 거리·중점·몸통 높이 겹침은 영상 투영 진단이다. 물체 중심이 두 점 사이에
있다는 이유로 앞뒤 관계·접촉·충돌 여유·파지를 판정하지 않는다. 같은 프레임에
결합한 현장 확인이 있을 때만 `closeEntry`를 `onsite_confirmed`로 표시하며,
이 값 자체는 구동 권한이나 성공 판정이 아니다. 신선도·현재 구도·실행 범위·
원본 검증과 출력 보호는 실행 경로에서 별도로 유지한다.

현장 감시 아래 별도로 허용한 접촉 탐색은
[단발 실행기](../tools/run_contact_probe.py)로 진행할 수 있다.
비공개 계획에서 M1 닫기 40%·350ms와 최대 6회를 고정하고 `--step`으로
한 번만 요청한다. 다음 단계는 이전 출력·OFF 완료와 원본 이미지 해시에 결합한
새 수동 영상 검토를 요구하며, 실패·불명 결과를 자동 재전송하지 않는다.

```bash
python3 tools/run_contact_probe.py --config .local/pipeline/contact/config.json \
  --step 1 --keychain-service '<service>' --keychain-account '<account>'
```

관측이나 모델의 범위 이탈은 `null`과 반려 사유로 남긴다. 명시적으로 허용한
수동 접촉 탐색의 원본 수집과 모델 선택을 구분하며, 깊이·접촉·파지 성공을
자동으로 만들어 내지 않는다. 실행기는 M4를 구동하거나 일반 수집의 방향 제한을
변경하지 않는다. 접촉 뒤 들기에는 별도 지원 경로와 현재 접촉·지지 확인이 필요하다.

### 접촉 미확정 상태의 M4 반응 시험

별도 접촉 미확정 범위는 확인된 파지를 전제로 하는 들기와 구분한다.
고정된 단발 요청, 현재 탁자 지지·이동 여유, 관측 변화·정지·OFF·공유 원장 검사를
유지하며 접촉을 `unknown`으로 기록한다. 요청 형식과 관측·실행 제한은
[M4 위 방향 전용 경로](m4-up-commissioning.md#접촉-미확정-응답-진단)를 따른다.
절차 완료와 실제 들어 유지하는 결과는 별도 판정이다.

## 다섯 도구의 역할

```bash
python3 tools/visualization/pipeline.py --latest .local/pipeline/output/latest.json \
  --output .local/pipeline/replay.rrd
```

Rerun은 검증된 전후 원본과 유효점·부분 관측·실패 사유·기준선 비교를 표시한다.
기본은 로컬 파일 저장이며, 기존 로컬 서버 연결은 `--connect`로 명시한다.
재생 영상은 실시간 영상으로 표시하지 않고 다른 카메라의 점을 겹치지 않는다.

수집 중에는 수집기가 이미 저장한 외부 영상 파일을 다음 표시기로 따라볼 수 있다.
카메라나 제어기에 추가 요청하지 않으며, 이 표시기의 실행은 모터를 움직이지 않는다.

```bash
python3 tools/visualization/collection.py \
  --sessions .local/pipeline/sessions --output .local/pipeline/file-viewer \
  --connect rerun+http://127.0.0.1:9876/proxy --seconds 0
```

`--sessions`는 읽을 수집 폴더를 명시하며 임의 실험 경로를 검색하지 않는다.
`--output`에는 새 비공개 출력 폴더를 사용한다. `--connect`를 생략하면 로컬 RRD에만
저장한다. `--seconds 0`은 Ctrl-C까지 표시하며, 양수는 지정한 초 뒤 표시기를 종료한다.
파일 갱신 중·지연·누락·검증 오류·수집 종료 후 마지막 기록을 구분한다. 이 화면은
저장 파일을 따라 표시하므로 실제 카메라 노출과 동기화된 실시간 추종이 아니다.
이 표시기는 외부 영상을 표시하고, 집게 카메라의 전후 관측은 사후 파이프라인에서
확인한다. 센서가 있으면 마지막 파일에 기록된 값으로 표시하며 현재 센서 직접 조회로
해석하지 않는다.

W&B에는 별도 평가·모델 버전 비교처럼 후속 판단에 필요한 결과만 선별해 게시한다.
이 파이프라인은 원본이나 준비 진단을 자동으로 웹에 올리지 않는다.
MuJoCo·Gazebo·RViz는 마지막 실측 기반의 정적 근사 자세·좌표·기구 구조 확인에
사용한다. 과거 자료 재생으로 현재 자세를 덮어쓰거나 미보정 동역학·접촉을 실측
학습 정답으로 사용하지 않는다.

## 검증 범위

모듈 테스트는 원본 변조, 부분 자료, 중복 요청, 중단, OFF 불명, 호출 중 명령 변경,
학습·평가 누출과 잘못된 카메라 overlay를 검사한다. 통합 테스트는 합성 native
패킷과 가짜 장치를 쓰며, 실제 원본의 재생 검증과 구분한다. 소프트웨어 검사와
과거 원본 재생만으로 새 카메라 구도의 자동 관측이나 실물 파지·들기를 검증했다고
해석하지 않는다.

등록한 조건의 실제 수집·고정 후 평가와 별도 수동 접촉 시험의 조건·분모·결과는
[2026-11-09 실측 기록](evidence/2026-11-09-shaft-evaluation-and-contact-probe.md)에 둔다.
사용법·처리 계약과 날짜별 실험 결과를 분리하며, 과거의 관측 통과를 새 자세의
관측 성공이나 제어 적격으로 간주하지 않는다.
