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

## 검증 범위와 남은 실물 조건

합성 모델·원본 fixture로 후보/정지 비교, 단계 전이, 실제 원본 검토 연결, 시간 지연, 실패·중단·정리 실패, 중복 실행 방지를 검증한다. 3단계 통합 fixture는 장치 backend·모델 로딩·정적 계약 검사를 주입하고, 기존 원본 contact/retention 검증 결과로 접근→닫기→들기를 연결한다. retention 원본 변조 때는 완료와 재실행을 막는다. 정적 실행 계약과 M4 장치 어댑터는 각각 기존 회귀로 검증한다. 이는 로봇의 집기 성공을 의미하지 않는다. 새 등록 외부 영상과 집게 양턱 계약은 정지 영상만 검증했으므로 기존 stationary 실행 차단을 유지한다. 이 계약으로 실제 제어하려면 작은 움직임에서 추적·시점 일관성을 검증해야 한다. 한 건의 양턱 전이는 모델 선택 성능을 평가하기에 부족하다. 접촉·깊이·파지력·실제 들림 검증과 관절·자세별 학습 자료는 별도로 확보해야 한다.

관련 회귀는 `python3 -m unittest tests.test_batch_model_trial tests.test_batch_model_runtime tests.test_batch_interaction tests.test_batch_m4 -q`로 실행한다.
