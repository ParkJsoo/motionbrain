# 배치 모델 후보 비교와 단계 실행

`learning.batch_model_trial`은 `batch_learning` 모델의 영상 상태와 예정 명령으로 후보 결과를 예측한다. 후보의 목표 오차를 정지 후보와 비교하며, 정지보다 나은 지원 범위 안의 후보만 요청한다. 수동 수집 조건 선택인 `batch_campaign`과 구분한다. 목표와 허용오차는 모델의 `stateFields` 순서·단위를 그대로 사용한다.

`BatchModelSequence`는 위치 접근(`position`) → 닫기(`close`) → 들기(`lift`) 순서를 관리한다. 위치 단계는 새 실제 영상이 허용오차 안에 들어와야 진행한다. 닫기·들기는 실행 원본, OFF 확인, 새로운 후관측과 기존 접촉·들림 검토가 있어야 진행한다. 예측값으로 접촉이나 들림을 대신하지 않는다. 실패·불명확한 실행은 요청을 소비하고 멈추며, 새 ID 재전송이나 출력 증량을 하지 않는다.

## 기존 실행기 연결

`learning.batch_model_runtime.ModelDecisionRuntime`은 `BatchRuntime`의 관측·실행·원본 감사·접촉 검토를 재사용한다. M4 들기는 기존 `BatchM4Route`를 통과한다. 실행 직전 실제 수집 영상으로 같은 모델·후보를 다시 비교하고 선택이 바뀌면 출력하지 않는다. 외부 영상과 집게 영상의 유효기간은 각각 기존 계약을 적용한다. 수집 완료·원본 감사가 오래 걸리면 기존 정답 프레임을 현재 영상으로 바꾸어 부르지 않고 새 관측을 촬영한다.

`BatchModelRunner` 생성은 파일과 고정 계약만 검사한다. `run` 호출은 실제 장치 접근·촬영·명령을 포함하므로 해당 실행 범위의 승인이 있어야 한다. 아래 변수는 이미 검증한 비공개 파일에서 읽은 값이며 모델이나 조건을 즉석 생성하는 예제가 아니다.

```python
from learning.batch_model_runtime import ModelDecisionRuntime, BatchModelRunner

# repository, private_root: 저장소 및 .local 아래 새 실행 경로
# plans: 단계 ID별 기존 evaluation 배치 계획
# credentials: 기존 비공개 인증 공급자 (값을 로그/인수에 쓰지 않는다)
runtimes = {
    stage_id: ModelDecisionRuntime(plan, repository,
                                  private_root / stage_id, credentials)
    for stage_id, plan in plans.items()
}
runner = BatchModelRunner(
    trial_id, stages, runtimes, conditions, models,
    private_root / "sequence", max_seconds=120,
)
# runner.scope와 runner.scope_sha256으로 구체적인 고정 범위를 검토한다.
# 아래 줄은 그 범위가 승인된 실제 실행에서만 호출한다.
report = runner.run(confirmed_scope_sha256=approved_scope_sha256)
```

각 `stages` 항목은 `id`, `kind`, `modelId`, `commands`, `target`, `tolerance`, `maxCommands`를 가진다. `models`는 모델 ID별 실제 `batch_learning` 모델이다. `conditions`는 단계 ID별 기존 계획에 포함된 evaluation 조건 목록이며, 명령 순서가 해당 단계의 `commands`와 같아야 한다. 위치 단계 예산은 1~4회, 접촉·들기 단계는 1회다. 전체 계획의 시간·시도·명령 제한도 유지한다. 들기는 `m4_up` 경로만 허용한다. 같은 실행 디렉터리는 중단 후에도 재실행되지 않는다.

모델 실행도 수동 배치와 동일한 제어기별 dispatch 잠금을 사용한다. 실제 배치 실행 진입점은 저장소의 `.local/batch-dispatch-budgets/`에 계획 해시별 예약을 남겨 조건·전체·준비 명령 예산을 함께 제한한다. 예약 이후 실패나 미시작이어도 예산을 돌려주지 않는다. 새 실행 폴더나 trial ID로 같은 계획의 명령 예산을 초기화할 수 없다. 이 원장은 도입 이후 예약을 기록하며 과거 실물 시도를 자동 복원하지 않는다. 새 계획의 허용은 기존 실패 명령 재전송 허용이 아니며 기존 실행 원장 확인이 필요하다.

## M4 위쪽 비접촉 위치 단계

M4 위쪽 명령을 모두 들기로 해석하던 제약을 분리했다. `position` 단계는 `m4_up` 경로의 `noncontact` binding을 사용할 수 있다. 이때 별도 `motionbrain.m4_up_noncontact.v1` scope를 발급하며 `context.object="none"`, `context.supportMode="empty_no_object"`, `load="empty"`가 필요하다. 기존 접촉·들기 scope를 비접촉으로 재해석하지 않는다. 실제 외부 관측 계약도 `object="none"`, `load="empty"`여야 한다.

이 경로는 기존 고정 단발 16%·200 ms, 시작각 범위·±0.5° 허용, 관측 변화 상한 1°, 제어기 증거 0.25초 신선도, 공유 원장·OFF 검사를 유지한다. 공급된 이중 카메라 검토에서 작업자 입회·이동 여유를 확인하고, 턱 접촉과 물체의 테이블 지지가 모두 없음을 명시해야 한다. 사후 접촉이 보고되면 비접촉 원본으로 인정하지 않는다. 유지 파지 촬영은 이 scope에 넣을 수 없고, 결과를 파지·들기 성공으로 표시하지 않는다.

모델은 기존 M4 외부 영상 상태로 위쪽 후보와 정지를 비교한다. 실제 경로에 따라 관측 계약을 선택하며, 하나의 단계에서 generic과 M4 관측을 혼합하지 않는다. `lift` 단계는 계속 닫기 이후의 `grasped` M4 경로만 사용한다. 물체가 있는 장면의 M4 접근, 임의 출력·기간, 실제 학습 성능까지 허용하거나 검증한 변경은 아니다. 구현 해시가 바뀐 기존 scope는 실물 실행에 그대로 재사용할 수 없으며 새 실행 범위를 명시적으로 검토해야 한다. 비접촉 경로 추가 직전의 알려진 구현 해시는 오프라인 완료 원본 감사에만 허용한다. 원본 scope를 고치지 않고 기존 결과 해시·안전 계약을 다시 검증하며, 이전 구현 해시로 새 비접촉 scope를 만들 수 없다.

## 단계 사이의 사전 검토

앞 단계 출력 이후에는 이전 `outputSequence`에 대한 접촉 검토를 재사용할 수 없다. 동적 검토를 사용할 generic 실행 계획의 `runtime.beforeReviewer`는 다음 형식이다.

```json
{"kind":"file","protocolVersion":1,"maxWaitSeconds":3}
```

실행기는 현재 수집 중인 외부·집게 원본으로 기존 호환 스냅샷을 만들고 세션 디렉터리에 `batch-before-review-request.json`을 기록한다. 요청에는 현재 출력 sequence, 조건·명령, 세션·관측 문맥, 원본 경로와 프레임 해시가 있다. 검토자는 그 원본을 실제로 검토한 뒤 같은 디렉터리에 다음 응답을 제한시간 안에 제공한다.

```json
{"requestSha256":"요청의 batch_observation.digest 값", "review": {"stage":"before"}}
```

위 `review`는 형식 축약이며, 실제로는 기존 `batch_interaction.verify_review`가 요구하는 전체 원본 검토 envelope여야 한다. `stage`, `conditionId`, 현재 `outputSequence`, 원본 파일 해시·프레임 결합·관측 문맥을 모두 검증한다. 요청에 없는 프레임이나 잘못된 해시, 누락·지연 응답은 실행을 막는다. 검토 대기 중에도 기존 세션 tick·마감 검사를 유지한다. `declaration` 검사는 계획 준비용이며 실행 증거로 쓸 수 없다. 사후 검토는 기존 `afterReviewer` 파일 프로토콜을 그대로 사용한다.

## 원본 검토 응답 제출

`learning.batch_review_response`는 사람이 실제로 판독해 작성한 기존 검토 envelope를 파일 응답으로 제출한다. 영상 판독이나 접촉·파지 판단을 생성하는 도구가 아니다. 요청·조건·명령·출력 순서·세션·원본 해시와 프레임 연결을 검사하고 기존 `verify_review`로 인과관계와 유효기간을 검증한다. 판정 불가(`unknown`)와 실패(`failed`)를 그대로 보존한다. 기존 응답은 덮어쓰지 않으며 제출 근거에 사람 검토 여부를 기록한다.

```sh
python3 -m learning.batch_review_response \
  --repository /absolute/path/to/motionbrain \
  --plan /absolute/path/to/motionbrain/.local/plan.json \
  --request /absolute/path/to/motionbrain/.local/session/batch-before-review-request.json \
  --review /absolute/path/to/motionbrain/.local/session/operator-review.json \
  --operator-reviewed
```

사후 검토는 실제 `batch-after-review-request.json`과 사후 envelope를 지정한다. 상대 원본 경로가 실행기의 envelope 복사 이후에도 같도록 검토 파일은 요청과 같은 디렉터리에 둔다. 닫기·들기 단계는 계획에 `afterReviewer`가 없으면 생성 시점에 차단한다. 설정이 있다는 사실은 응답하는 검토자가 실행 중임을 보장하지 않는다.

**사람이 처음 영상을 확인하는 대화형 경로는 아직 연결되지 않았다.** 현행 요청 응답 제한은 최대 3초이며, 이 제출기는 그 제한이나 영상 유효기간을 늘리지 않는다. 늦은 검토·이전 출력의 검토는 실패로 남긴다. 준비된 판독자가 없는 상황에서 이 CLI만 추가해 접근→닫기→들기가 실사용 가능하다고 볼 수 없다. 대화형 검토에는 사전 장면 확인 후 새 스냅샷을 요청하고 그 실제 프레임을 다시 검토하는 연결이 필요하다. 이전 프레임의 사람 판단을 새 프레임에 복사하거나 정적 검토를 최신 검토로 취급해서는 안 된다.

## 검증 범위와 남은 실물 조건

합성 모델·원본 fixture로 후보/정지 비교, 단계 전이, 실제 원본 검토 연결, 시간 지연, 실패·중단·정리 실패, 중복 실행 방지를 검증한다. 3단계 통합 fixture는 장치 backend·모델 로딩·정적 계약 검사를 주입하고, 기존 원본 contact/retention 검증 결과로 접근→닫기→들기를 연결한다. retention 원본 변조 때는 완료와 재실행을 막는다. 정적 실행 계약과 M4 장치 어댑터는 각각 기존 회귀로 검증한다. 이는 로봇의 집기 성공을 의미하지 않는다. 새 등록 외부 영상과 집게 양턱 계약은 정지 영상만 검증했으므로 기존 stationary 실행 차단을 유지한다. 이 계약으로 실제 제어하려면 작은 움직임에서 추적·시점 일관성을 검증해야 한다. 한 건의 양턱 전이는 모델 선택 성능을 평가하기에 부족하다. 접촉·깊이·파지력·실제 들림 검증과 관절·자세별 학습 자료는 별도로 확보해야 한다.

관련 회귀는 `python3 -m unittest tests.test_batch_model_trial tests.test_batch_model_runtime tests.test_batch_interaction tests.test_batch_m4 -q`로 실행한다.
