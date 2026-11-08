# 고정 모델의 순차 접근·닫기 연결

[상대 상태 모델](relative-action-selection.md) · [수집 방법](manual-data-collection.md)

닫기 이후 들기는 [M4 위 방향 전용 경로](m4-up-commissioning.md)의 별도 고정 모델과 연결한다. 수동 식별·고정 평가·모델 선택을 구분하며, 현재 실물 결과는 [2026-11-08 연결 실험 기록](evidence/2026-11-08-close-lift-integration.md)을 따른다.

`learning/model_grasp_trial.py`는 저장된 물체·양턱 상대 상태 모델로 위치 조정 후보를 선택하고, 실제 새 관측으로 목표 도달을 확인한 뒤 닫기 후보 하나를 선택하는 제한 시험 코어다. 기존 `relative_action_learning.select()`와 `SequencePort`를 재사용한다. 생성자·모델 로드는 장치에 접근하지 않는다. 기본 실행 CLI나 장치 검색·자동 학습은 제공하지 않는다.

코어의 소프트웨어 검증은 합성 관측과 모의 컨트롤러로 수행했다. 관측 어댑터의 실제 원본·모델 계약 진단과 [별도 수동 수집의 구동 전 예측 확인](relative-action-selection.md#2026-11-07-자세를-분리한-학습과-구동-전-예측-확인)은 수행했지만, 이 코어가 선택한 명령으로 실물 순차 제어를 실행하거나 집기에 성공한 결과는 없다. 현재는 분석가 점 검토와, 별도로 등록한 고정 구도의 패치·원호 특징 관측을 구분한다. 범용 물체·턱 인식이나 파지 상태 인식의 완성은 아니다. 분석가 기준점·실제 구도와 현장 확인에 의존하므로 `observationAssisted=true`, `fullyAutonomous=false`를 유지한다. 방향·출력·기간의 선택은 고정 모델의 후보 비교 결과만 사용하며, 사람이 대신 고른 명령이나 규칙 기반 대체 명령을 끼워 넣지 않는다.

## 준비와 모델 재사용

1. 시작 자세는 ㄱ자처럼 반복하기 쉬운 자세를 후보로 삼되 현재 영상에서 물체·양턱·이동 여유를 확인한다. 물체와 초기 자세를 사람이 배치했다면 개입으로 기록한다. 영상만으로 가장 성공하기 쉬운 자세를 보증하지 않는다.
2. `FrozenModelBank`에 `{id, path, sha256}` 목록을 넣어 상대 상태 모델 파일을 고정한다. 팔 표식 네 상태 모델을 물체·양턱 세 상태 모델로 바꾸지 않는다.
3. 위치 조정 단계 0~3개와 마지막 닫기 단계 1개를 지정한다. 현재 위치에서 파지에 진입할 수 있음을 확인한 경우에는 닫기 단계 하나만 지정할 수 있다. 필요한 관절만 포함하며 여러 관절을 동시에 구동하지 않는다. 각 단계는 `modelId`, 영상상 `target`, 좌표별 `tolerancePx`, 명령 후보와 횟수 한도를 가진다. 목표와 허용오차는 명시한 시험 조건이며 학습된 정확도나 물리 안전 한계가 아니다.
4. 각 단계의 실제 관측이 해당 모델의 **전체** 관측 계약과 같아야 한다. 카메라·구도·물체·표식·자세 조건을 모델에 맞추려고 덮어쓰지 않는다. 기존 관절별 모델은 `poseRegime`이 서로 다르고 현재 자세의 적용 가능성도 미확인이다. 파일이 정상 로드된다는 사실만으로 여러 모델을 연속 사용하도록 허용하지 않는다.

부족한 조건은 새 자세·관측 계약에서 별도 자료로 보강해야 한다. 한 명령 전이 2개는 기존 모델의 최소 지원 조건일 뿐 충분한 학습량이 아니다. M2의 상세 원인 분석이나 전 관절 수집을 이 연결의 선행 조건으로 두지 않는다.

## 실행 범위와 흐름

`ModelGraspTrial(scope, bank, observer, port, store, evidence_root=..., clock=...)`에 이미 확인한 관측 어댑터와 기존 `SequencePort`, 새 `ArtifactStore`를 주입한다. `run(confirmed_scope_sha256=plan_digest(scope))`는 정확한 범위 해시가 맞는 경우 한 번만 실행한다. 해시 일치 자체가 사용자 승인이나 현재 장치 조건을 대신하지 않는다.

`scope`의 스키마는 `motionbrain.model_grasp_trial.v1`이다. 필수 항목은 다음과 같다.

| 항목 | 의미 |
| --- | --- |
| `trialId`, `hostClockId`, `controllerBootId` | 새 시험 식별자, 동일 호스트 시계, 현장에서 확인한 제어기 부팅 ID |
| `modelHashes`, `stages` | 고정 모델 목록과 단계별 목표·후보·한도 |
| `maxCommands`, `maxRequestedMs`, `maxSeconds` | 총 명령 시도 수·요청 기간 합계·호스트 전체 기한 |
| `maxObservationAgeSeconds`, `maxDecisionAgeSeconds` | 관측과 전송 판단의 나이 상한 |
| `postOffSeconds`, `retentionSeconds` | OFF 후 최소 관측 대기 3초와 별도 보유 확인 기간 |

코드는 총 12명령·요청 기간 합계 12초·전체 600초보다 큰 범위를 받지 않으며, 실제 시험에는 필요한 더 작은 범위를 정해야 한다. 이 소프트웨어 상한은 이동 여유나 실물 안전의 보증이 아니다. 마지막 닫기는 최대 한 명령으로 제한한다. 일반 실행기의 어깨 위 방향 제외와 기존 관절별 출력·기간 제한은 유지한다. M4 위 방향은 별도 단발 코어에서만 해당 모델·관측·각도 조건을 검사한다.

닫기 단독 시험도 같은 고정 모델의 후보와 정지 비교, 입력 범위·관측 계약·전송 직전 검사, 현장 깊이·이동 여유 근거와 별도 보유 검증을 거친다. 입력이 범위 밖이면 무구동 보류하며, 이미 닫힌 것으로 보이는 영상만으로 파지를 인정하지 않는다. 닫기 단독 경로는 합성 관측으로 소프트웨어 검증했으며 실물 집기 성능을 검증한 결과가 아니다.

각 위치 조정 단계는 실제 관측→모델 후보와 정지 비교→한 명령→OFF→새 관측 순서다. 예측한 다음 상태로 현재 상태를 대체하거나 다음 단계에 진입하지 않는다. 정지가 선택돼도 아직 실제 목표 범위에 도달하지 않았다면 보류한다. 지원 명령이 없거나 관측 계약·학습 상태 범위가 맞지 않으면 기록을 남기고 종료한다.

RUN 전송 직전에는 원래 선택을 다시 검사한다. `SequencePort.execute(..., before_run=guard)`는 RUN 의도를 내구 저장한 뒤 실제 요청 직전에 검사를 한 번 호출한다. 모델 선택이 바뀌거나 관측·저장 판단이 오래됐거나 입력이 전송 0.1초 전에 저장되지 않았으면 전송하지 않는다. guard 내부에서 HTTP 조회나 새 촬영을 하지 않는다. 요청 기한과 전체 시험 기한을 검사하며, 실패·인터럽트·응답 유실 뒤에는 새 명령 ID나 대체 명령으로 재시도하지 않는다.

## 관측 어댑터 계약

`learning/grasp_observation.py`는 집게 위 카메라 원본을 연결하는 `GripperObservationSource`와 분석가 검토를 받는 `ReviewedGraspObserver`를 제공한다. 기존 `gripper_camera_dataset.read_frame()` 검증을 재사용하며, 현재 실물 구도의 점 정의·정착·깊이·이동 여유는 별도 확인이 필요하다. 외부 카메라 원본을 이용하는 경우 기존 `ActionRecorder`의 원본·시계 검증을 유지하는 별도 어댑터가 필요하다. 저장 점을 자동 추정하거나 최신 시각을 붙여 새 관측처럼 취급하지 않는다.

`GripperObservationSource(url, session_id, expected_boot_id, scene_context, store, ...)`는 생성·시작만으로 촬영하지 않는다. 명시적으로 `capture(deadline=...)`를 호출할 때만 `/capture?fresh=1` GET을 한 번 보낸다. 리다이렉트·프록시·자동 재시도 없이 최대 6초·2MB의 응답을 받고 JPEG·메타데이터·시각·실제 부팅 ID·원본 해시를 검사한다. 오류 응답과 실패 원본도 보존하며 실패한 소스로 다시 촬영하지 않는다.

`scene_context`는 실제 확인한 `physicalView`, `object`, `poseRegime`, `landmarks`이며 요청한 단계나 모델에서 복사하지 않는다. 카메라 부팅 ID·영상 크기는 캡처 원본에서 파생한다. 카메라를 재부팅했거나 실제 구도·자세가 바뀌어 기존 모델과 계약이 다르면 자동 적용하지 않는다.

`ReviewedGraspObserver(source, reviewer, retention_reviewer=..., ...)`는 `reviewer(record, deadline=..., tick=...)`에서 명시적으로 받은 프레임별 주석과 정착 확인만 사용한다. 보유 검토 콜백도 `(observations, deadline=..., tick=...)`를 받는다. 콜백은 즉시 반환하거나 대기 중 주기적으로 `tick()`을 호출하고 기한을 지켜야 한다. 콜백 전후 기한·출력과 원본 바이트를 다시 검사한다. 오래된 주석의 시각을 갱신해 새 영상으로 올리지 않으며, 실패 후 새 촬영으로 자동 복구하지 않는다. 닫기·보유 확인값을 어댑터가 생성하지 않는다.

- `start(deadline=...)`: 해당 시험의 제한 촬영 준비.
- `observe(stage, after_off=..., deadline=..., tick=...)`: 새 원본과 분석가 검토를 연결한 관측 반환. 대기 중 `tick()`을 정기적으로 호출해 전체 기한과 예상하지 못한 출력을 감시한다.
- `latest(stage)`, `check()`: 이미 검증·준비된 관측과 정착 상태의 비차단 검사. RUN 직전 검사에서는 파일 주석 대기·장치 조회·중첩 HTTP 기한을 시작하지 않는다.
- `verify_grasp(stage, after_off=..., deadline=..., tick=...)`: 별도 보유 확인 결과 `held`, `failed`, `unknown` 반환. 대기 중 출력 감시를 유지한다.
- `close()`: 원본 촬영과 시계 기록을 마무리하고 `status=completed`를 반환. 미완결 촬영은 성공으로 올리지 않는다.

관측은 `frame={frameId,pixelSha256,width,height}`, `annotation`, 실제 `context`, `hostClockId`, `sourceEarliest`, `sourceLatest`, `receivedAt`, `persistedAt`, `stable`을 가진다. `annotation`은 `analyst_points` 또는 아래 별도 등록의 `registered_template_points`·`registered_scene_template_points` 계약을 따른다. 등록한 관측은 `annotation.landmarkContract`와 실제 `context.landmarks`가 정확히 같아야 하며 수동 점 계약과 자동 혼합하지 않는다. 시각은 동일한 Python monotonic 영역의 원래 캡처/요청 근거이며 새 시험 시작 이후여야 한다. 집게 카메라는 정확한 노출 시각을 발명하지 않고 요청 시작~수신 범위를 사용한다. OFF 이후 상태는 최소 3초 뒤의 새 프레임이어야 한다. 원본 바이트·메타데이터·실제 구도·정착 검증은 관측 어댑터의 책임이며 플래그만으로 인증되지 않는다.

## 등록 특징과 원호를 사용하는 새 관측 계약

`learning/registered_gripper_landmarks.py`는 분석가가 기준 이미지에서 지정한 물체·양턱 점에, 새 원본의 등록 특징 이동을 더한다. `register(reference_jpeg, points, parameters)`로 기준 이미지와 점·영역·탐색·품질 조건을 고정하고 `RegisteredGripperLandmarks.observe(jpeg, frame_id)`로 새 원본을 측정한다. 이 모듈에는 장치 접근·모터 실행·학습 기능이 없다.

기본 방식은 물체와 양턱의 패치를 NCC로 대응한다. 현재 원통 구도에 한해 `default_parameters(object_arc=True)`로 별도 원호 방식을 등록할 수 있다. `learning/object_arc.py`는 고정한 중앙 영역의 위·아래 원호를 함께 사용해 중심 이동을 계산한다. 물체 점은 **분석가 기준점 + 기준 이미지 대비 원호 중심 이동**이며, 원 적합의 절대 중심을 보정된 물리 기준점으로 취급하지 않는다. 양턱은 기존 패치 대응을 계속 사용한다.

등록은 기준 원본 SHA-256, 점 정의와 파라미터 SHA-256을 포함한다. 새 주석은 `frameId`, 복원 BGRA의 `pixelSha256`, `method=registered_template_points`, `registrationSha256`, `parametersSha256`, `referenceImageSha256`, `landmarkContract`를 갖는다. 모델의 `context.landmarks`는 `registered-template-points-v1:<등록 SHA-256>`에 묶인다. 원호 방식을 추가하면 등록·파라미터 해시가 달라지므로 기존 패치 계약이나 수동 주석 모델에 자료가 자동 합쳐지지 않는다.

추출기는 NCC·후보 유일성·탐색 경계·턱의 기하 관계를 검사하고, 원호 방식은 윤곽 수·대비·잔차·반지름·중심 범위를 함께 검사한다. 실패한 대응은 채택된 주석을 내지 않는다. 사후 결과를 보고 기준을 낮추거나 다른 최고 후보를 골라 모델 입력 범위에 맞추지 않는다. 실패 원본을 다른 관측 방식으로 연구할 때도 원래 실패와 새 계약의 재감사 결과를 별도로 보존한다.

빠른 특징 계산과 입력 신선도는 각각 확인해야 한다. `ReviewedGraspObserver`는 기본 2초의 관측 나이 안에서 콜백과 원본 재검증을 마쳐야 하며, 새 GET이나 영상 처리는 RUN 직전 `latest()`·`check()` 안에서 수행하지 않는다. 과거 주석 복사·시각 갱신이나 기한 확대를 관측 실패의 우회책으로 사용하지 않는다.

등록 특징의 일치는 깊이·접촉·이동 여유·정착·정확도를 인증하지 않는다. `stable`과 닫기 진입의 현장 확인을 NCC 점수만으로 만들지 않으며 외부 카메라·시간순 관측·현장 근거를 별도로 연결한다. 현재 계약은 특정 원통·고정 구도의 개발 범위다. [최신 실물 기록](evidence/2026-11-08-close-lift-integration.md)에서는 새 계약으로 4건을 학습·고정했으나 고정 후 평가와 모델 선택 닫기는 아직 0회다.

새 외부 구도에는 `learning/registered_scene_landmarks.py`의 별도 등록을 사용할 수 있다. 분석가가 물체·양턱 점의 실제 정체성과 패치·검색 영역을 동작 전에 지정하고, 기준 원본과 장면 문맥을 함께 고정한다. 주석은 `registered_scene_template_points`, 모델의 특징 계약은 `registered-scene-template-points-v1:<등록 SHA-256>`이다. 기존 모델의 문맥이나 입력 범위를 새 영상으로 바꾸지 않는다. 평행이동만 관측하므로 회전·가림·유일성·품질 실패 시 중단하며, 접촉·깊이·제어 적격을 부여하지 않는다.

관측과 모델 선택을 연결하는 실행기는 `ManualCollection.pulse(..., before_run=guard)`로 기존 전송기의 RUN 직전 가드를 사용할 수 있다. 가드는 캐시된 관측과 사전 선택을 비차단으로 확인해야 하며, 실패하면 재시도하지 않는다. 기본 수동 수집 경로와 출력·정지 검사는 유지한다.

## 닫기와 집기 확인

닫기 진입에는 현재 프레임에 묶인 `closeEntryEvidence`가 필요하다. 필드는 `frameId`, `pixelSha256`, `method=onsite_observation`, `objectBetweenJawsVerified`, `depthVerified`, `clearanceVerified`, `evidenceFile`, `evidenceSha256`다. 세 확인값은 모두 `true`여야 하며, 물체 중심이나 턱 간격만으로 만들지 않는다.

`held` 결과에는 `method=analyst_retention_review`, `contactVerified`, `retentionVerified`, `supportDistinguished`, 두 개 이상의 시간순 `observations`, `evidenceFile`, `evidenceSha256`가 필요하다. 처음과 마지막 관측의 간격이 지정된 보유 확인 기간 이상이어야 하며 마지막 관측은 최신이어야 한다. 바닥에 놓인 물체와 실제 집게에 유지되는 물체를 구분하지 못하면 `unknown`이다. 닫기를 추가 반복해 목표 턱 간격을 억지로 맞추지 않는다.

두 검토 기록은 비공개 `evidence_root` 아래의 실제 JSON 파일로 보존한다. 파일은 `evidenceFile`, `evidenceSha256`을 제외한 제출 내용과 같아야 하며 읽은 바이트의 SHA-256을 검사한다. 상대 경로 이탈·심볼릭 링크는 거절한다. 해시는 기록 변경 검사용이며 사람의 판단이나 물리 상태의 진실성을 인증하지 않는다. 보유 확인을 받아도 관측 보조를 받은 결과이며 학습 인식만의 자율 집기나 들기 성공으로 표현하지 않는다.

모델·관측·선택·전송 직전 판단·실제 명령 영수증·미실행/실행 불명·마지막 OFF를 기존 증거 저장소에 남긴다. 종료 직전까지 예상하지 못한 출력 이력을 검사하며 촬영 마무리나 OFF 확인이 실패하면 집기 성공을 보고하지 않는다. 원본과 별도로 학습을 자동 시작하거나 중단된 시험을 재개하지 않는다.

## 무출력 원본·모델 진단

`tools/check_grasp_observation.py`는 카메라 원본과 기존 모델의 계약을 확인한다. 제어기 주소나 모터 실행 옵션은 없다. `--capture`는 카메라 영상 한 장만 읽으며, `--source-binding`은 이미 저장한 원본을 다시 검사한다. 두 경로 모두 `controlEligible=false`이며 실물 실행 준비가 완료됐다는 뜻이 아니다.

설정 JSON은 `schemaVersion=motionbrain.grasp_observation_check.v1`, `sessionId`, `cameraUrl`, `expectedCameraBootId`, `sceneContext`, `modelBindings`, `captureTimeoutSeconds`를 가진다. `modelBindings`의 각 항목은 `{id,path,sha256}`이며 실제 모델 파일을 해시로 고정한다. 장치별 주소와 설정·원본·주석·진단 출력은 `.local/` 아래에 둔다.

```sh
python3 tools/check_grasp_observation.py \
  --config .local/grasp-observation/config.json --capture \
  --output-dir .local/grasp-observation/source-check-01

python3 tools/check_grasp_observation.py \
  --config .local/grasp-observation/config.json \
  --source-binding .local/grasp-observation/source-check-01/capture-0001/source-binding.json \
  --source-binding-sha256 '<원래 source-binding.json의 SHA-256>' \
  --annotation .local/grasp-observation/points.json \
  --annotation-sha256 '<원래 주석 파일의 SHA-256>' \
  --output-dir .local/grasp-observation/point-check-01
```

주석은 기존 `analyst_points` 형식이다. 주석을 제공하지 않거나 승인하지 않은 경우 상태를 생성하지 않는다. 전체 관측 계약이 일치하고 점 주석도 승인된 경우에만 지원 명령별 저장 상태 예측을 계산한다. 계약이 다르면 변경된 필드만 보고하며 모델이나 원본을 바꾸지 않는다. 재생 중 원래 시각은 그대로 보존한다. 오래된 저장 영상의 예측을 최신 실물 판단이나 새 독립 평가로 세지 않는다.

```sh
python3 -m unittest tests.test_model_grasp_trial tests.test_supervised_transport \
  tests.test_relative_model_bundle tests.test_relative_action_learning \
  tests.test_grasp_observation tests.test_check_grasp_observation
```

## 저장 영상의 관측 방식 진단

`tools/audit_grasp_observations.py`는 저장된 집게 카메라 영상에서 점의 소수점 표현과 정수 반올림이 입력 범위 판정에 미치는 영향을 비교한다. 장치 조회·촬영·제어·전이모델 학습은 하지 않는다. 원래 고정 모델과 분석가 주석을 바꾸거나 범위 거절을 해제하지 않으며, 모든 결과는 `trainingEligible=false`, `controlEligible=false`다.

진단기는 고정 구도의 원통 윗면 오른쪽 윤곽을 원으로 근사하고, 좌우 ROI의 어두운 연결 성분 끝부분을 턱 점으로 측정한다. 원통·조명·구도가 다른 영상의 범용 검출기가 아니다. ROI 경계에 닿는 성분이나 원 근사 잔차 등 품질 플래그가 있는 측정은 범위 계산에서 제외하되 원본·플래그·분모를 남긴다. 검출 성공이 물체 중심이나 접촉 위치의 정확도를 입증하지 않는다.

설정은 `motionbrain.grasp_observation_study.v1`이며 `.local/` 안에 다음 항목을 둔다.

- `frozenModel={path,sha256}`, `manifestPath`, `manifestSha256`: 기존 고정 모델과 같은 학습 세션을 가리키며 평가 그룹은 비워 둔다. 원본 수집 감사와 프레임 해시·부팅 ID 검사를 다시 거친다.
- `sceneContext`, `expectedCameraBootId`: 실제 관측 계약과 예상 부팅 ID. 다른 구도로 계약을 바꾸지 않는다.
- `parameters`: `imageShape`, `grayThreshold`, `objectRightArcRows`, `objectRightArcColumns`, `jawRois`, `jawDistalRows`. 원 윤곽의 행·열 끝값은 포함하며 턱 ROI는 `[x0,y0,x1,y1]`의 뒤 끝값을 제외한다. 결과를 보고 범위를 통과하도록 조정하지 않는다. 기존 진단에서 가져온 경우 `parameterOrigin={path,sha256}`로 원래 보고서의 `parameters`와 동일한지 검사한다(`imageShape` 제외).
- `rejectedQuery={source,annotationPath,annotationSha256}`: 학습 프레임과 겹치지 않는 거절 입력 한 장과 원래 분석가 주석. `source={root,label,sessionId}`이며 `label=before`만 허용한다.
- 선택 항목 `stationarySources`: 같은 `source` 형식의 과거 정지 영상 목록. 시각을 갱신하거나 새 동기 관측으로 취급하지 않는다.

```sh
python3 tools/audit_grasp_observations.py \
  --config .local/grasp-observation/study.json \
  --config-sha256 '<고정한 설정 파일의 SHA-256>' \
  --output-dir .local/grasp-observation/study-01

python3 -m unittest tests.test_grasp_observation_diagnostic
```

`inputCoverage`는 품질 검사를 통과한 동작 전 입력만으로 명령별 최소·최대를 계산한다. `pairedInputCoverage`는 동작 전후 모두 통과한 쌍만 사용하며, 두 결과 모두 실패 입력의 분모를 보존한다. 원본 감사 실패로 명령을 신뢰할 수 없으면 `unknownCommand` 그룹에 남긴다. 동작 후 정답으로 입력 범위를 넓히지 않는다. `originalAnalystModelQueryCoverage`는 기존 모델의 원래 판정과 분리한다.

소수점과 정수 표현의 판정 차이, 정지 영상의 반복 변동, 원래 분석가 점과의 차이는 관측 계약을 검토할 근거다. 어느 것도 독립적인 정확도·예측 성능·물리 안전의 측정값이 아니다. 새 관측 방식을 채택하려면 점 정의와 실패 처리를 먼저 고정하고 별도 모델·보류 평가에서 검증해야 한다.
## 닫기에서 들기로 넘기는 별도 연결

`learning/close_lift_trial.py`의 `CloseLiftTrial`은 새 `motionbrain.model_close_contact_trial.v1` 닫기 코어와 별도 짧은 M4 모델을 연결한다. 닫기 코어는 최대1명령 뒤 정상 OFF를 `closed_off`로 반환하며, 이 결과를 파지·공중 보유 성공으로 표시하지 않는다. 기존 파지 코어의 보유 검증 경로는 그대로 유지한다.

연결은 정상 닫기 종료 뒤 최소3초가 지난 새 양 카메라 관측2쌍을 요구한다. 두 쌍은 최소1초 떨어져 있어야 하며, 원본·시각·카메라 계약·탁자 지지·양턱 접촉·깊이·이동 여유·같은 물체 확인을 검증한다. 기존 M4 입력 범위를 유지하고, 원본에서 측정한 집게 위 상대 이동은 가로·세로 각각4px 이하여야 한다. 접촉은 `table_supported_contact_ready`이며 공중 보유와 구분한다.

새 모델의 선택으로 들기 최대1회를 실행하며 중간 출력 이력 변경, 원본 변경·노화, 실패·불명확한 명령은 후속 단계를 막는다. RUN 직전에는 검증한 원본 바이트를 재검사하고 기존 M4 사전 검사를 유지한다. 들기 실행 완료 뒤에도 별도 영상의 물체-탁자 공극·보유 판독 전에는 `liftVerified`를 참으로 만들지 않는다. 위치 조정·복귀·재학습·자동 재시도는 이 연결기의 기능에 포함하지 않는다. 이 연결 코어의 검증은 모의 환경에서 수행했다. 2026-11-08 실물 묶음은 M1 관측·입력 범위에서 중단돼 연결 실행은 0회이며, 준비 명령의 정상 종료를 연결 성공으로 세지 않는다.
