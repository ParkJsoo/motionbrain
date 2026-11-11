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

## FileReviewer의 실제 판독 화면

기존 M4 `FileReviewer` 요청을 사람이 판독할 때는 별도 로컬 화면을 미리 열 수 있다. 서버 시작은 원본 파일만 읽으며 촬영·장치 연결·모터 명령을 하지 않는다. scope는 승인 대상인 현재 M4 파일을, 요청 디렉터리는 해당 실행의 `FileReviewer`가 사용하는 정확한 출력 디렉터리를 지정한다. 다른 실행이나 과거 원본을 검색하지 않는다.

```sh
python3 tools/file_review_server.py \
  --scope .local/current-m4-scope.json \
  --request-root .local/current-run/m4-scopes/current-request \
  --port 8766
```

브라우저에서 `http://127.0.0.1:8766`을 열고 요청을 기다린다. 화면은 검증한 외부·집게 원본 두 장과 예정 관절·방향·출력·기간, 물체·자세를 표시한다. 두 영상을 직접 확인하고 영상 안정 여부와 **현장 사람이 확인한** 작업자 입회·이동 여유·접촉·테이블 지지를 각각 명시한다. 기본값은 모두 미선택이다. 현장 확인을 영상만으로 추정해 긍정으로 입력하지 않는다. 불명확하면 `판정 불가로 제출`하여 모든 확인을 미확인으로 보존한다. 이 응답은 파지·들기 성공 판정이 아니다.

동작 전 제출 기한은 요청 발견이나 페이지 로드가 아닌 원본 `sourceEarliest` 이후 9초다. 사후도 원본 신선도 10초 이내로 제한하며 실제 소비자의 전체 마감도 적용된다. 원본 픽셀·해시와 요청 nonce를 결합하고, 제출 직전 원본 변경을 다시 검사한다. 늦거나 변경된 요청, 이전 응답 덮어쓰기는 거부한다. 사전 화면 준비만으로 현장 판단을 대신하지 않으며 이전 판단을 새 프레임에 자동 복사하지 않는다. 시간 초과 뒤 동작을 자동 재실행하지 않는다.

이 화면은 `before/after-review-request.json` → `before/after-review-decision.json`인 기존 **M4 FileReviewer 프로토콜 전용**이다. 배치의 최대 3초 `batch-before/after-review-response.json` producer가 완성됐다는 뜻이 아니다. 접촉 확정에는 별도의 두 시점 원본·고정 계측 등록·깊이와 동일 물체 현장 검토가, 실제 파지·들림에는 독립 유지 판독이 계속 필요하다. 화면의 현장 접촉 선택만으로 이 검증을 통과시키지 않는다. 서버는 loopback만 열고 기존 로컬 콘솔과 같은 Host·Origin·요청 토큰 검사를 사용한다. 사용 후 서버 프로세스를 종료한다.

모터 없이 실제 원본 연결만 확인할 때는 서버에 `--read-only-review`를 붙인다. 화면은 **읽기 전용 판독 시험 / 참조 메타데이터(실행 예정 아님)**으로 표시하고 현장 긍정 선택을 비활성화한다. 서버도 `unknown` 외 응답을 거부한다. 별도 승인된 촬영 담당자는 기존 M4 설정 파일을 `python3 tools/m4_up_trial.py review-check --config .local/read-only-config.json --output-dir .local/new-read-only-capture`에 제공할 수 있다. 설정은 기존 `{scope, camera}` 형식이며 scope는 이 시험의 원본 결합용 메타데이터다. 모터 실행 승인이 아니다. 출력 디렉터리는 새 경로여야 한다.

`review-check`는 기존 모든 POST 금지 `ReadOnlyPort`를 그대로 사용하고, 현재 이중 카메라 한 쌍만 촬영해 `FileReviewer`의 명시한 판정 불가를 소비한다. 영상과 출력 이력이 바뀌거나 판단이 긍정이면 실패로 남긴다. 종료 시 촬영 정리와 출력 순서 불변·OFF를 다시 확인한다. 각 요청의 `*-review-timing.json`에는 원본 시각·게시 시각·기존 마감·게시 당시 남은 시간을 기록하여 촬영 준비 지연과 화면 전달 지연을 구분한다. 실패 결과는 기존 오류 종류와 함께 제한된 코드 목록에 속하는 `failureDetails`만 기록하며 임의 예외 문자열·인증정보를 출력하지 않는다. UI를 먼저 열고 담당자가 준비를 확인한 뒤 촬영해야 하며, 시간 초과를 새 프레임의 긍정 판단으로 대체하지 않는다.

합성 원본으로 HTTP 화면·두 원본 전달·명시한 판정 불가 제출·실제 `FileReviewer` 소비를 연결하고, 지연·원본 변조·다른 요청·중복 제출·교차 출처 요청을 검증한다. 실제 브라우저에서는 명령과 원본 표시, 미선택 기본값, 만료 후 제출 차단을 확인했다. 실제 카메라를 연결한 읽기 전용 시험에서는 두 원본 표시까지 확인했지만, 원격 브라우저 도구의 확인·제출 지연 때문에 현행 기한 안의 응답 소비는 완료하지 못했다. 합성 원본의 소비 검증을 실제 판독 운영 성공으로 해석하지 않는다. 기한을 늘리거나 현재 영상을 읽었다고 자동 확인하지 않으며, 이 검증은 실물 접촉 성공이 아니다.

## 검증 범위와 남은 실물 조건

합성 모델·원본 fixture로 후보/정지 비교, 단계 전이, 실제 원본 검토 연결, 시간 지연, 실패·중단·정리 실패, 중복 실행 방지를 검증한다. 3단계 통합 fixture는 장치 backend·모델 로딩·정적 계약 검사를 주입하고, 기존 원본 contact/retention 검증 결과로 접근→닫기→들기를 연결한다. retention 원본 변조 때는 완료와 재실행을 막는다. 정적 실행 계약과 M4 장치 어댑터는 각각 기존 회귀로 검증한다. 이는 로봇의 집기 성공을 의미하지 않는다. 새 등록 외부 영상과 집게 양턱 계약은 정지 영상만 검증했으므로 기존 stationary 실행 차단을 유지한다. 이 계약으로 실제 제어하려면 작은 움직임에서 추적·시점 일관성을 검증해야 한다. 한 건의 양턱 전이는 모델 선택 성능을 평가하기에 부족하다. 접촉·깊이·파지력·실제 들림 검증과 관절·자세별 학습 자료는 별도로 확보해야 한다.

관련 회귀는 `python3 -m unittest tests.test_batch_model_trial tests.test_batch_model_runtime tests.test_batch_interaction tests.test_batch_m4 -q`로 실행한다.

## 수집 영역과 모델 지원 범위의 분리

`collectionAdmission`은 관측·안전 조건을 고정한 **비접촉 M1 수동 수집 전용** 모드다. 기존 `motionAdmission`과 동시에 사용하지 않는다. 모델의 15차원 학습 최소·최댓값, `predict()`와 후보 선택 지원 범위를 변경하거나 좌표를 보정하지 않는다. `ModelDecisionRuntime`과 `BatchModelRunner`는 이 모드를 생성 시 거부한다. 접촉·파지·M4 경로에도 연결할 수 없다.

새 독립 자료를 촬영하기 **전** `motion_validation_scope.v1`에 `collectionDomain`을 선언하고 기존 `install_validation_scope()`로 원본 세션과 명령 intent에 봉인한다. 선언에는 양턱의 네 좌표 범위, 방향별 시작·종료 개구 범위, 원래 검색 영역 안의 여유(최소 2 px), 최대 개구 변화(원래 등록 개구 범위의 폭 이하)를 포함한다. 현재 구현의 명령은 M1 열기·닫기 각각 40%·350 ms로 한정된다. 숫자는 관측 허용 오차를 모델 범위에 더하는 방식으로 만들지 않는다.

`learning.batch_collection_domain.build_collection_admission(repository, manifest)`은 기존 원본 감사와 독립 수동 라벨 검증을 재사용한다. 사전에 선언한 동일 관측기·자세·빈 하중, 명령별 독립 표본 최소 2개, 수동 개구 변화가 라벨 허용 오차의 두 배를 초과하는 조건을 모두 확인해야 한다. 과거 자료에 선언을 사후 첨부해 발급할 수 없다. 합격 파일은 유한 수집 영역의 자격 근거이며 모델 제어 승인이나 파지 성공이 아니다.

실행에는 별도의 `manualCollectionAuthorization`이 필요하다. 동일 캠페인·부팅·명령·예산에 대한 사용자의 현장 참석·여유·다른 관절 자세 유지 확인 출처와 실제 원본 영상의 배치 검토 출처를 각각 해시로 연결한다. 일반적인 진행 승인만으로 현장 확인 값을 생성하지 않는다. 현재 프레임의 접촉 판독을 했다고 표시하지 않으며 접촉·파지·들기 값은 거짓/미판독으로 유지한다. 출처가 없으면 실행을 거부한다. 기존 공유 잠금·물리 명령 예산·OFF·실패 재전송 금지 조건은 그대로 적용한다.

유효하게 관측한 무변화·역방향·예상 종료 범위 밖의 반응은 학습 정답과 평가 오차에서 삭제하지 않는다. 전이와 중단 사유를 저장하고 캠페인을 영속 정지한다. 원본 해시·계약 변경이나 관측 불명확은 정상 반응으로 취급하지 않는다.

## 사전 고정 명령의 새 자료 평가

공식 학습 CLI의 기존 `frozenModels`만 지정하면 여전히 개발 재생이다. 실제 사전 예측 검증을 하려면 별도로 `learning.batch_fixed_evaluation.freeze_model()`이 만든 고정 파일의 경로·해시를 runtime 및 평가 manifest의 `fixedCommandEvaluation`에 지정한다. 같은 manifest에는 학습 자료를 넣지 않고 정확히 하나의 고정 모델을 사용한다.

이 모드는 수집 영역에서 사전 승인한 명령을 평가한다. 모델이 명령을 선택한 실물 제어 시험이 아니다. 실제 두 카메라의 실행 전 원본이 모델 고정 이후 촬영·저장되었는지 확인한 뒤, 동일 입력에 대한 상태 모델·명령 평균·무변화 예측을 계산해 원본 명령 intent보다 먼저 봉인한다. 기존 지원 범위 밖의 상태에서도 **평가 전용** 예측은 가능하지만 `predict()`의 제어 지원 범위는 유지한다.

```sh
python3 tools/train_batch_campaign.py --manifest .local/future-evaluation/manifest.json --output .local/future-evaluation/report
```

manifest의 평가 항목은 기존 일반 `batch_visual_transition.v1` runtime 전이 또는 `sourceKind=registered_jaws`의 양턱 5D 전이와 원본 경로·해시를 사용한다. 양턱 전이는 실제 집게 before를 첫 입력으로, 외부 카메라의 원본 pre-window 프레임을 두 번째 증거로 봉인한다. 외부 좌표를 모델 입력으로 합치지 않으며, 집게 request→command 10초 한도와 두 원본의 모델 고정 이후 촬영·명령 이전 영속 저장을 재검사한다. CLI는 양 카메라 원본을 다시 관측하고, 입력 픽셀·시간·원본 intent의 사전 봉인·모델 해시·학습 자료와의 원본/촬영 그룹 분리를 확인한다. 모델 지원 범위 안팎을 나누되 세 예측의 평가는 같은 정답 묶음을 쓴다. 제외 수와 사유를 보존하며 범위 밖 성능이 좋아도 제어 적격으로 승격하지 않는다.

이 연결의 소프트웨어 시험과 실물 영역 확장·자동 수집·고정 후 평가의 성공은 별개다. 독립 확대 표본과 현장 확인 출처가 없는 영역은 실행할 수 없으며, 실패로 중단한 기존 실물 계획을 이 모드로 다시 활성화하지 않는다.

실물 운영 전에는 등록 원본과 일치하는 카메라 해상도·구도를 확인한다. VGA 등록을 사용한다면 해당 수집 동안 VGA가 필요하며, 기존 원본 metadata의 shape 검사를 우회하지 않는다. 수집 전 모델·관측기·수집 영역·현장 확인·예산 파일을 고정하고, 현재 OFF·부팅·원장·두 카메라 신선도를 검사한다. 종료 후에는 다섯 모터 OFF를 확인하고 사용한 카메라 설정을 QVGA로 복구해 확인한다. 실패로 끝난 범위의 재실행, 초기 자세·물체·해상도·관측기 또는 명령 범위 변경에는 새로운 구체적 실행 범위와 필요한 현장 확인이 필요하다. 소프트웨어 경로가 준비되어도 실제 영역 자격 발급·고정 후 새 자료 평가·자동 수집 실행은 각각 별도로 검증해야 한다.
