# 고정 모델의 순차 접근·닫기 연결

[상대 상태 모델](relative-action-selection.md) · [수집 방법](manual-data-collection.md)

`learning/model_grasp_trial.py`는 저장된 물체·양턱 상대 상태 모델로 위치 조정 후보를 선택하고, 실제 새 관측으로 목표 도달을 확인한 뒤 닫기 후보 하나를 선택하는 제한 시험 코어다. 기존 `relative_action_learning.select()`와 `SequencePort`를 재사용한다. 생성자·모델 로드는 장치에 접근하지 않는다. 기본 실행 CLI나 장치 검색·자동 학습은 제공하지 않는다.

소프트웨어 검증은 합성 관측과 모의 컨트롤러로 수행했다. 이 경로로 새 실물 명령을 실행하거나 집기에 성공한 결과는 없다. 자동 물체·턱 인식도 추가하지 않았다. 현재 주석 계약은 분석가가 영상의 점을 검토하는 관측 보조를 전제로 하며 `observationAssisted=true`, `fullyAutonomous=false`를 유지한다. 방향·출력·기간의 선택은 고정 모델의 후보 비교 결과만 사용하며, 사람이 대신 고른 명령이나 규칙 기반 대체 명령을 끼워 넣지 않는다.

## 준비와 모델 재사용

1. 시작 자세는 ㄱ자처럼 반복하기 쉬운 자세를 후보로 삼되 현재 영상에서 물체·양턱·이동 여유를 확인한다. 물체와 초기 자세를 사람이 배치했다면 개입으로 기록한다. 영상만으로 가장 성공하기 쉬운 자세를 보증하지 않는다.
2. `FrozenModelBank`에 `{id, path, sha256}` 목록을 넣어 상대 상태 모델 파일을 고정한다. 팔 표식 네 상태 모델을 물체·양턱 세 상태 모델로 바꾸지 않는다.
3. 위치 조정 단계 1~3개와 마지막 닫기 단계 1개를 지정한다. 필요한 관절만 포함하며 여러 관절을 동시에 구동하지 않는다. 각 단계는 `modelId`, 영상상 `target`, 좌표별 `tolerancePx`, 명령 후보와 횟수 한도를 가진다. 목표와 허용오차는 명시한 시험 조건이며 학습된 정확도나 물리 안전 한계가 아니다.
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

코드는 총 12명령·요청 기간 합계 12초·전체 600초보다 큰 범위를 받지 않으며, 실제 시험에는 필요한 더 작은 범위를 정해야 한다. 이 소프트웨어 상한은 이동 여유나 실물 안전의 보증이 아니다. 마지막 닫기는 최대 한 명령으로 제한한다. 어깨 위 방향과 기존 관절별 출력·기간 제한은 유지한다.

각 위치 조정 단계는 실제 관측→모델 후보와 정지 비교→한 명령→OFF→새 관측 순서다. 예측한 다음 상태로 현재 상태를 대체하거나 다음 단계에 진입하지 않는다. 정지가 선택돼도 아직 실제 목표 범위에 도달하지 않았다면 보류한다. 지원 명령이 없거나 관측 계약·학습 상태 범위가 맞지 않으면 기록을 남기고 종료한다.

RUN 전송 직전에는 원래 선택을 다시 검사한다. `SequencePort.execute(..., before_run=guard)`는 RUN 의도를 내구 저장한 뒤 실제 요청 직전에 검사를 한 번 호출한다. 모델 선택이 바뀌거나 관측·저장 판단이 오래됐거나 입력이 전송 0.1초 전에 저장되지 않았으면 전송하지 않는다. guard 내부에서 HTTP 조회나 새 촬영을 하지 않는다. 요청 기한과 전체 시험 기한을 검사하며, 실패·인터럽트·응답 유실 뒤에는 새 명령 ID나 대체 명령으로 재시도하지 않는다.

## 관측 어댑터 계약

현재 실물 구도에 맞는 관측 어댑터는 현장 연결 전에 검증해야 한다. 기존 `ActionRecorder`의 원본·시계 검증과 집게 카메라의 `gripper_camera_dataset.read_frame()` 검증을 재사용할 수 있다. 저장 점을 자동 추정하거나 최신 시각을 붙여 새 관측처럼 취급하지 않는다.

- `start(deadline=...)`: 해당 시험의 제한 촬영 준비.
- `observe(stage, after_off=..., deadline=..., tick=...)`: 새 원본과 분석가 검토를 연결한 관측 반환. 대기 중 `tick()`을 정기적으로 호출해 전체 기한과 예상하지 못한 출력을 감시한다.
- `latest(stage)`, `check()`: 이미 검증·준비된 관측과 정착 상태의 비차단 검사. RUN 직전 검사에서는 파일 주석 대기·장치 조회·중첩 HTTP 기한을 시작하지 않는다.
- `verify_grasp(stage, after_off=..., deadline=..., tick=...)`: 별도 보유 확인 결과 `held`, `failed`, `unknown` 반환. 대기 중 출력 감시를 유지한다.
- `close()`: 원본 촬영과 시계 기록을 마무리하고 `status=completed`를 반환. 미완결 촬영은 성공으로 올리지 않는다.

관측은 `frame={frameId,pixelSha256,width,height}`, `annotation`, 실제 `context`, `hostClockId`, `sourceEarliest`, `sourceLatest`, `receivedAt`, `persistedAt`, `stable`을 가진다. `annotation`은 기존 `analyst_points` 계약을 그대로 따른다. 시각은 동일한 Python monotonic 영역의 원래 캡처/요청 근거이며 새 시험 시작 이후여야 한다. 집게 카메라는 정확한 노출 시각을 발명하지 않고 요청 시작~수신 범위를 사용한다. OFF 이후 상태는 최소 3초 뒤의 새 프레임이어야 한다. 원본 바이트·메타데이터·실제 구도·정착 검증은 관측 어댑터의 책임이며 플래그만으로 인증되지 않는다.

## 닫기와 집기 확인

닫기 진입에는 현재 프레임에 묶인 `closeEntryEvidence`가 필요하다. 필드는 `frameId`, `pixelSha256`, `method=onsite_observation`, `objectBetweenJawsVerified`, `depthVerified`, `clearanceVerified`, `evidenceFile`, `evidenceSha256`다. 세 확인값은 모두 `true`여야 하며, 물체 중심이나 턱 간격만으로 만들지 않는다.

`held` 결과에는 `method=analyst_retention_review`, `contactVerified`, `retentionVerified`, `supportDistinguished`, 두 개 이상의 시간순 `observations`, `evidenceFile`, `evidenceSha256`가 필요하다. 처음과 마지막 관측의 간격이 지정된 보유 확인 기간 이상이어야 하며 마지막 관측은 최신이어야 한다. 바닥에 놓인 물체와 실제 집게에 유지되는 물체를 구분하지 못하면 `unknown`이다. 닫기를 추가 반복해 목표 턱 간격을 억지로 맞추지 않는다.

두 검토 기록은 비공개 `evidence_root` 아래의 실제 JSON 파일로 보존한다. 파일은 `evidenceFile`, `evidenceSha256`을 제외한 제출 내용과 같아야 하며 읽은 바이트의 SHA-256을 검사한다. 상대 경로 이탈·심볼릭 링크는 거절한다. 해시는 기록 변경 검사용이며 사람의 판단이나 물리 상태의 진실성을 인증하지 않는다. 보유 확인을 받아도 관측 보조를 받은 결과이며 학습 인식만의 자율 집기나 들기 성공으로 표현하지 않는다.

모델·관측·선택·전송 직전 판단·실제 명령 영수증·미실행/실행 불명·마지막 OFF를 기존 증거 저장소에 남긴다. 종료 직전까지 예상하지 못한 출력 이력을 검사하며 촬영 마무리나 OFF 확인이 실패하면 집기 성공을 보고하지 않는다. 원본과 별도로 학습을 자동 시작하거나 중단된 시험을 재개하지 않는다.

```sh
python3 -m unittest tests.test_model_grasp_trial tests.test_supervised_transport \
  tests.test_relative_model_bundle tests.test_relative_action_learning
```
