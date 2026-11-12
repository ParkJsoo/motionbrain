# 집게 영상 자동 관측기의 오프라인 시제품

[실험 파이프라인](experiment-pipeline.md) · [수동 수집](manual-data-collection.md)

목적은 영상 상태를 매번 사람이 표시하는 부담을 줄이는 것이다. 이 시제품은
저장된 집게 영상에서 보이는 색 영역과 양쪽 턱끝을 관측하고, 좌표 주석 및
고정 좌표 기준과 비교한다. 실시간 등록 관측기·수집 실행기에는 연결하지 않는다.
`trainingEligible`, `controlEligible`, `liveAdmissionVerified`는 항상 `false`다.

## 관측하는 항목

- 분홍색으로 보이는 영역의 경계상자와 그 상자의 중심. 가려진 물체의 전체
  윤곽·실제 중심·파지점은 복원하지 않는다.
- 실제로 보이는 손가락 끝의 안쪽 모서리. 서로 다른 기준 영상의 패치를
  비교하되, 모호한 일치·기준 간 위치 불일치·끝점 주변 무늬의 소실·색 영역에
  의한 가림은 점을 반환하지 않는다. 한쪽만 보이면 반대편을 대칭으로 채우지 않는다.
- 물체와 두 턱끝이 모두 관측될 때만 영상상 턱끝 중점·수평 간격·물체 중심의
  상대 좌표를 계산한다. 간격은 물리 개구폭이 아니며 깊이·접촉·힘·미끄럼 부재를
  판정하지 않는다.

색 영역은 최소 면적 이상의 연결 영역이 하나일 때만 사용한다. 여러 후보 중
가장 큰 것을 임의로 고르지 않는다. 너무 큰 영역과 관측 경계 가까이 있는 영역도
보류한다. 현재 정책은 320×240 영상, 지정 색상 및 탐색 구역을 사용하는 개발 가정이다.
새 자세·배경·촬영 구도에 자동 전이하거나 재생 중 임계값을 완화하지 않는다.

집게 카메라와 붙잡힌 물체가 함께 움직이면 물체의 상대 좌표가 거의 변하지 않을
수 있다. 이 좌표만으로 들림량을 측정할 수 없으며, 외부 영상의 별도 관측과
시각 대응이 필요하다.

## 저장된 영상 재생

```sh
python3 tools/replay_development_gripper_observer.py \
  --manifest .local/development-gripper/manifest.json \
  --output .local/development-gripper/replay-01
```

출력은 새 폴더여야 한다. 원본·주석·재생 결과는 `.local/` 안에 둔다.
명령은 파일만 읽으며 카메라 연결·촬영·ARM·모터 요청·행동 모델 학습을 하지 않는다.

manifest의 최상위 필드는 다음과 같다.

| 필드 | 내용 |
| --- | --- |
| `schemaVersion` | `motionbrain.development_gripper_replay.v1` |
| `sourceRoot` | 저장소 `.local/` 아래 원본 폴더의 절대 경로 |
| `context` | `viewId`, `poseRegime`, `object` 문자열 |
| `references` | 최대 8개 기준 영상의 `id`, 상대 `path`, `sha256`, `points`, `rois` |
| `frames` | 재생할 영상의 고유 `frameId`, 상대 `path`, `sha256` |
| `annotations` | 비교 주석 파일의 상대 `path`, `sha256` |

기준 영상의 `points`와 `rois`는 `jawLeft`, `jawRight`를 각각 명시한다.
점은 모서리 주변 9×9 영상이 패치 안에 들어가도록 지정한다. 서로 다른 개구
상태의 기준 패치는 개발자가 선택한 것이며, 이를 자동 학습이나 새 자료 평가로
세지 않는다. 기준 영상 자체의 재생도 별도로 센다.

주석 파일의 `frames` 항목은 `imageBinding`의 `path`, `sha256`,
`decodedRgbSha256`과 `visiblePinkRegionBoxXYXY`,
`jawLeftDistalInnerCorner.xy`, `jawRightDistalInnerCorner.xy`를 포함한다.
경계상자의 오른쪽·아래쪽은 제외 좌표다. 가려진 점은 `null`로 둔다.
색상 초안이 보조한 주석과 독립 판독 주석의 출처는 `provenance`에 구분해 남긴다.

기준·재생 영상과 주석의 파일 해시 및 RGB 픽셀 해시를 검사한다. 기준 영상을
재생 목록에서 제외해도 고정 좌표 기준에 사용한 주석의 연결을 검사한다.
`report.json`에는 다음을 함께 기록한다.

- 전체·완전 상태·부분 관측·경계 보류·기준 영상 자체 재생 수
- 주석 비교의 기대 좌표 수, 실제 반환 수, 가려진 점을 잘못 반환한 수
- 반환된 좌표에 대한 유클리드 거리 차이와 고정 좌표 기준의 같은 지표
- 기준 영상을 제외한 비교. 이 부분도 같은 시연의 개발자료이며 독립 평가가 아니다.

반환하지 않은 점을 오차 0으로 세지 않는다. 좌표 차이가 작아도 선택된 영상과
수동 주석에 대한 일치도일 뿐이며, 물리 정확도·새 구도 일반화·실물 제어 성능은
검증하지 않는다. 명령만 쓰는 기준과의 비교는 관측기의 좌표 비교가 아니라
뒤의 행동 결과 학습 단계에서 수행한다.

## 행동 결과 학습을 시작하는 조건

모든 관절의 관측기 완성을 첫 학습의 선행 조건으로 두지 않는다. 우선 관측이
가능하고 기존 실행 범위 안에 있는 한 관절·한 자세·하중 조건부터 연결한다.

1. 관측 항목·구도·가림 처리·시각 계약을 고정하고 별도의 새 영상에서 검증한다.
   개발 재생 통과만으로 실행 관측 적격을 만들지 않는다.
2. 기존 수집 경로가 실행 전 영상과 예정 관절·방향·출력·기간을 기록하고,
   실제 출력·OFF·사후 영상·실패·사람 개입을 별도 근거로 보존하도록 연결한다.
   예정 기간이 없는 수동 유지 조작의 실제 ON 시간을 예정 기간으로 역산하지 않는다.
3. 기존 학습기가 지원하는 관측 계약에 연결된 유효 전이부터 개발 학습한다.
   현재 `relative_action_learning.fit`은 같은 명령의 유효 전이가 최소 2개여야
   그 명령의 예측기를 만든다. 두 후보 명령을 비교하려면 각각 최소 2개가 필요하다.
   이 최소치는 상태 변화 학습·정확도·일반화에 충분한 자료량이라는 뜻이 아니다.
4. 상태를 쓰는 모델을 명령 평균·무변화 기준과 비교하고 모델을 고정한 뒤,
   고정 후 새 수집 자료를 별도로 평가한다. 실제 후보 선택 제어와 집기·들기
   성공은 그 뒤의 별도 실물 검증이다.

개발용 관측 결과를 원본 수집 증거에 연결하는 읽기 전용 어댑터는 아래와 같다.
계약 및 수집 시각·예정 명령 근거가 부족하면 주석과 관측 결과는 개발자료로
유지한다. 실물 구동이나 행동 학습 적격을 만드는 어댑터는 제공하지 않는다.

## fresh 원본과 단발 수집 증거 연결

`learning/development_gripper_source.py`는 기존 `read_frame`과
`bind_gripper_pair`를 재사용한다. 파일 해시뿐 아니라 카메라 부팅 ID,
`fresh=1` 요청·`after_request` 응답, 카메라 프레임 순서, 호스트 시계 식별자와
요청·수신·저장 순서를 검사한다. 저장 시각을 카메라의 정확한 노출 시각으로
바꾸지 않는다.

관측기 바인딩은 `motionbrain.development_gripper_source_binding.v1`이며
`context`, `references`, `configurationSha256`을 포함한다. 기준 영상 항목은
재생 manifest와 같은 `id`, 상대 `path`, `sha256`, `points`, `rois`다.
설정 해시는 기준 영상·정책·문맥에 연결되며, 바인딩 파일 자체도 별도의 SHA256으로
검사한다. 현재 구도 선언은 설정과 일치해야 하지만 선언이 실제 구도의 독립
판독을 대신하지는 않는다.

다음 명령은 저장된 `esp32-frame.jpg/json`만 다시 검사한다. 기본값으로 촬영하지 않는다.

```sh
python3 tools/check_development_gripper_source.py \
  --config .local/development-gripper/source-check.json \
  --frame-root .local/development-gripper/saved-capture \
  --output .local/development-gripper/source-check-01
```

검사 설정에는 `schemaVersion=motionbrain.development_gripper_source_check.v1`,
`.local/` 아래 `assetRoot`, `observerBindingPath`, `observerBindingSha256`,
`cameraBootId`, `sessionId`, `actualScene`, `maxAgeSeconds`, `cameraUrl`을 명시한다.
`actualScene`은 `physicalView`, `object`, `poseRegime`, `landmarks`이며
`landmarks`는 `development-gripper-visible-v1:` 뒤에 관측기 설정 해시를 붙인다.

현장 조건을 확인하고 새 정지 영상의 읽기 전용 촬영이 허용된 경우에만
`--frame-root` 대신 **`--capture`**를 사용한다. 기존 `GripperObservationSource`로
최대 한 번 GET 촬영하고 원본과 metadata를 보존한 뒤 닫는다. 재시도·프로필 변경·
ARM·모터 명령은 없다. 정적 설정 오류는 촬영 전에 거절한다.

저장 재생의 신선도는 당시의 저장 시각 기준일 뿐 현재 신선도가 아니며,
`savedReceiptReplayOnly=true`, `currentFreshnessVerified=false`로 구분한다.
현재 읽기 전용 촬영의 신선도 검사가 통과해도 움직임 추적 적격과는 다르다.

기존 실험 파이프라인의 새 관측 `kind=development_gripper`에는 `context`,
`captureGroup`, `gripperPairRoot`, `observerBindingPath`, `observerBindingSha256`,
`cameraBootId`를 지정한다. 기존 수집기의 사전 저장 명령과 OFF 시각,
카메라 전후 창을 그대로 검사한다. pair 검사 전후 네 원본 파일을 고정하고
관측 때 읽은 전체 프레임·metadata까지 대조하므로 같은 프레임 ID의 시각/영상
변경으로 사후 프레임이 사전 입력이 되는 것을 거절한다.

기본 결과는 `development_only` 또는 `partial_development`로 저장하고 학습 전이는
생성하지 않는다. 새 상태 필드는 `objectVisibleMinusCornerMidXpx`,
`objectVisibleMinusCornerMidYpx`, `visibleCornerHorizontalSpanPx`다.
마지막 항목은 기존 `jawOpeningPx`의 유클리드 거리와 다르다. 기본 `fit`과 실물용
모델 로더는 이 개발 namespace를 거절한다.
기존 수집기의 예정 명령·실제 출력 기록, 원본 종료와 중복 실행 차단은 유지한다.

### 명시적 개발 학습 연결

오프라인 연결 검증에는 관측 설정에 `learningUse: "development_replay"`를 추가한다.
양쪽 영상의 물체와 두 끝점이 모두 관측되고 원본·시간·부팅 검사가 통과해야
전이를 만든다. 입력 영상 요청부터 RUN 응답까지 5초를 넘으면 거절한다.
잘림·가림·일부 좌표만 있는 결과는 학습 전이를 만들지 않는다.
구도·자세 문맥 선언이 같다는 검사는 실제 자세의 동일성이나 충돌 여유를 증명하지 않는다.

기존 실험 파이프라인의 `split`에도 다음 두 값을 명시해야 학습 자료로 사용한다.

```json
{
  "trainGroups": ["development-train"],
  "evaluationGroups": [],
  "allowRecoveredDevelopmentTraining": true,
  "allowDevelopmentGripperTraining": true
}
```

자동 관측 전이는 `developmentObservationOnly=true`, `recoveryDevelopmentOnly=true`로
표시된다. 학습기는 같은 예정 명령의 서로 다른 전이가 최소 2개 있어야 해당 명령의
예측기를 만든다. 한 행은 학습 목록과 표본 부족량으로 남으며, 지원되는 예측기가
없으면 `trainingPerformed=false`, 모델 없음으로 보고한다. 프레임·원본·실행·촬영 묶음의
학습/평가 중복 검사는 유지한다. 자료가 충분하면 기존 영상 상태 모델·명령 평균·무변화
기준선을 비교하고 모델 파일을 해시로 고정한다. 매 표본의 수동 좌표 입력은 필요하지 않다.

이 경로의 모델과 평가는 **개발용 오프라인 재생**이다. 개발 모델에는
`developmentObservationOnly=true`를 기록하며, 기본 `load_frozen_model`은 빈 모델까지
거절한다. 평가 묶음이 있어도 촬영 시점이 모델 고정 이후였다는 증거를 새로 만들지 않는다.
관측 정확도·실물 관측 적격·모델 선택 집기·들기 성능을 검증한 것으로 해석하지 않는다.

### M2 조회 기한 모드와 자료 분리

`ManualCollection.pulse(..., monitor_policy="wrist-deadline-550-v1")`는 M2의
550ms 명령에만 명시적으로 적용한다. 실행 중 조회들은 명령의 남은 절대 기한을
공유하며, 원본 요청 저장 뒤와 소켓 호출 직전에도 그 기한을 검사한다.
최소 한 번의 실행 중 안전 상태 검사가 필요하다. 이후 마지막 0.2초에는 새 조회를
시작하지 않고 로컬 카메라 검사 뒤 기존 STOP으로 종료한다. 조회 시간 초과는 실패로
보존하고 정지하며 재시도하지 않는다. 기본 모드·M4·짧은 명령에는 적용하지 않는다.

이 모드도 RUN 응답 이후를 기준으로 하므로 통신·장치 처리·감속을 포함한 실제 PWM
출력 기간이 정확히 550ms라고 보장하지 않는다. 명령 intent와 receipt의 정책이
일치해야 하며, 감사된 행에서 모델 문맥의 `executionTimingPolicy`로 전달한다.
기존 모드 자료와 모델은 별도 파티션으로 유지하고, 새 조건에서 학습·고정·평가한다.
조회 예산 구현판은 intent·receipt의 `monitorReadBudgetRevision`에 함께 남긴다.
짧은 개별 조회 제한에서 공유 기한으로 바뀐 경우에도 정지 기한·출력 명령은 같지만,
이전 정상 자료를 재사용할 때는 이 차이와 공통 실행 조건의 적합성을 명시한다.
초기 구현의 호출당 0.2초 제한은 명령 마감이 남았는데도 조회를 거절할 수 있었다.
`shared-command-deadline-v2` 구현판은 이 개별 제한을 없애고 동일한 550ms 절대
마감을 유지한다. 이전 원본의 정책·구현판을 소급 변경하지 않으며, 더 엄격한 제한을
통과한 기존 정상 자료를 쓸 때도 구현판별 표본 수와 중단 이력을 보존한다.

### 고정 모델의 후보 비교

`relative_action_learning.select`의 `predictor`는 기본 `stateRidge` 또는
`commandMean`을 받는다. 후자는 현재 영상 상태에 학습한 명령별 평균 변화를 더한다.
두 방식 모두 같은 명령 지원·입력 상태 범위 검사를 거치며, 지원되지 않는 후보는
거절 이유를 남긴다. 정지도 후보에 포함하고 동률이면 정지한다.
`minimum_improvement_px=4`이면 정지보다 예측 오차가 **4px 초과** 줄어드는 경우에만
움직임 후보를 선택한다. 이는 공학적으로 지정한 여유이며 관측 정확도나 신뢰구간이 아니다.
원래 예측 오차 순위와 최종 선택을 별도 필드로 남기므로 여유 조건에 의해 정지를
선택하면 `selected`가 `candidates[0]`과 다를 수 있다.

기존 재생 CLI의 `mode="offline_candidate_queries"`는 학습·평가 원본을 받지 않고
해시로 고정한 모델만 읽는다. `queries`에는 `state`, `target`, `commands`, 모델과
동일한 `context`, `targetKind="synthetic_image_state"`를 넣는다. 선택적으로
`predictor="commandMean"`, `minimumImprovementPx=4`를 지정한다.

```sh
python3 tools/replay_relative_actions.py \
  --manifest .local/candidate-query/manifest.json \
  --frozen-model .local/candidate-query/frozen-model.json \
  --model-sha256 "${MODEL_SHA256}" \
  --allow-development-model \
  --output-dir .local/candidate-query/replay-01
```

개발 모델 허용은 이 질의 전용 모드에만 적용하며 기본 실물 모델 로더의 차단은
유지한다. 도구는 모델을 다시 학습하지 않고, 질의 상태의 원본을 재감사하지도 않는다.
다른 단계에서 감사한 상태를 입력해도 합성 목표를 실제 집기 목표로 바꾸지는 않는다.
출력은 `evaluated=0`, `independentEvaluation=false`, `controlEligible=false`이며,
선택하지 않은 행동의 실제 결과나 성공률을 관측했다는 뜻이 아니다.

## 소프트웨어 검사

```sh
python3 -m unittest discover -s tests -p 'test_development_gripper_observer.py'
python3 -m unittest discover -s tests -p 'test_development_gripper_source.py'
python3 -m unittest discover -s tests -p 'test_development_gripper_learning.py'
```

합성 영상의 변화·중복 후보·가림·잘림·문맥과 해시 변경, 기준 영상이 없는
재생의 주석 연결을 검사한다. 실제 새 영상·카메라 노출 시각·실물 정확도의
검증을 대신하지 않는다.
