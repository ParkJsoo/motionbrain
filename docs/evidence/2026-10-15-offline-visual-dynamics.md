# Mac CPU 오프라인 dynamics 기반 — 2026-10-15

## 범위와 사전 고정한 모델

검증된 action-transition manifest를 로드하고 무변화 예측과 작은 선형 ridge 모델을 같은 고정 분할에서 비교한다. NumPy CPU만 사용한다. **ridge alpha=1.0**을 구현 전에 고정했고 탐색·test 기반 모델 선택은 없다. MLP·VLA·클라우드 학습은 범위 밖이다. 기존 규칙 policy replay는 행동 제안 평가이므로 dynamics 모델의 학습/평가 코드로 재사용하지 않았다.

특징 버전 `motionbrain.planned_visual_m4.v1`:

- 입력16개: before 박스의 정규화 중심x/y·너비/높이, AS5600 M4 각도의 sin/cos, 요청 축 one-hot5개, shoulder 명령 여부, 요청 PWM/255·기간초·목표각과 before M4의 최단 각도차/180·percent/100.
- 목표5개: after−before 중심x/y·너비/높이(각 영상 크기로 정규화), M4의 최단 부호 각도변화[-180,180). 작은 제한 행동을 위한 표현으로 180도 이상 회전/다회전 복원은 지원하지 않는다. 다른 관절의 기본값·추정값은 실측 목표로 쓰지 않는다.
- actual output trace·실제 elapsed·종료 상태·targetReached·after 관측은 입력에 들어가지 않는다. 출력 trace는 기존 manifest/schema의 실제 행동 증거 검증에 사용한다. 학습은 실행 전에 알 수 있는 후보 명령으로 결과를 예측한다.
- exact live before/after 검출·유효 박스·동일 영상 크기/label/classId·실측 M4를 요구한다. missing 박스를 화면 중앙/0으로 채우지 않는다. label/classId 일치는 물체 instance 추적 증거가 아니므로 고정 한 물체 수집 조건에서 출발해야 한다.

## 데이터와 재현 계약

`learning/dataset.py`의 `validate_manifest`를 재사용하여 저장된 플래그를 신뢰하지 않고 원본/hash·journal·출처·행동 연결·분할을 재검증한다. `splits.py`의 session/episode/frame/image/command 누수 검사를 유지한다. 원본 provenance가 합성/불명이거나 train/validation/test가 비면 production 학습을 거부한다. 적격 행에서 새 특징 조건을 만족하지 못해도 조용히 행을 버리지 않고 해당 sample ID와 함께 전체 실행을 거부한다.

정규화 mean/std와 모델은 **train만으로 fit**한다. 상수 열 scale은1.0, intercept는 train 목표 평균이다. 학습 CLI는 test 점수를 계산하지 않는다. 평가 CLI가 고정 artifact와 같은 manifest/분할/코드/runtime에서 train·validation·test의 무변화/선형 MAE·RMSE와 sample IDs·분모를 기록한다. 단위가 다른 시각값과 각도를 합친 종합 점수는 만들지 않는다. 빈 평가 matrix의 결과는 unavailable/n0이다.

artifact에 feature/target 열과 단위, seed, manifest/split/train-matrix/source hash, Python/NumPy 버전, 정규화 통계, 계수, alpha, fit 행 수, 전체/적격/제외/분할 행 수를 저장한다. 다른 manifest·source·runtime 또는 합성 smoke artifact를 production 평가에 넣으면 거부한다. SHA-256은 일관성 확인이며 악의적인 원본 전체 재작성의 인증 수단이 아니다.

## CLI

저장소 루트에서, 새 output 디렉터리를 지정한다. 기존 출력은 덮어쓰지 않는다.

```sh
python3 tools/train_visual_dynamics.py --manifest <manifest.json> --output-dir <new-training-dir>
python3 tools/evaluate_visual_dynamics.py --manifest <manifest.json> --model <new-training-dir>/model.json --output-dir <new-evaluation-dir>
python3 tools/smoke_visual_dynamics.py --output-dir <new-smoke-dir>
```

production 실패는 `refusal.json` 또는 `unavailable.json`과 종료코드2로 남는다. 합성을 허용하는 production flag는 없다. smoke는 고정 seed로 만든 **수학적 fixture 120개**이며 센서/카메라/물리 dynamics를 모사·입증하지 않는다. 별도 schema `motionbrain.synthetic_dynamics_smoke.v1`, `trainingEligible=false`, `controlEligible=false`를 사용한다. smoke의 `measured` 필드는 변환 함수의 계약 테스트 값이며 실측 주장으로 사용할 수 없다.

## 검증과 결과

실행 후 실제 측정치·거부 결과·테스트·독립 리뷰를 아래에 기록한다. 실물 학습과 물리 조작 성능 개선은 아직 없다.

최종 합성 smoke는 **120행 → train94/validation11/test15**, Mac CPU wall0.17초(user0.07/sys0.02초)였다. 합성 test의 중심x MAE는 무변화0.001875/선형0.0000195592, 가상 M4 변화 MAE는1.2/0.0125179였다. 이는 의도적으로 선형 관계를 만든 fixture에서 수치 경로가 작동한 결과이며 **실물 예측 개선이나 조작 개선이 아니다**. 소수 단일 실행의 시간은 성능 benchmark로 일반화하지 않는다. 계수·정규화·전체 지표·source hash는 [합성 전용 artifact](visual-dynamics-2026-10-15/synthetic-smoke.json)에 저장했다.

기존 실제 로컬 inventory manifest를 원본부터 다시 검증한 결과 적격0/제외11, train/validation/test 모두0이다. production 학습 CLI는 종료코드2·[refusal.json](visual-dynamics-2026-10-15/production-refusal.json)을 남기고 **model.json을 생성하지 않았다**. 합성 artifact를 production 평가 CLI에 넣는 시도도 trained receipt가 없어 [unavailable](visual-dynamics-2026-10-15/production-evaluation-unavailable.json)로 거부했다. 이는 평가 성공이 아니다.

최종 통합 `python3 -m unittest discover -s tests -q`는 **589개 통과(20.753초)**다. 신규 B 테스트19개는 특징·M4 wrap·after/실제출력 입력 누수 방지·train 전용 정규화/fit·고정 재현성·빈 분모·NaN/형태 오류·합성 차단·원본/hash·manifest/split 변조·모델 receipt·CLI 실패 기록을 검증했다. 독립 리뷰에서 malformed JSON이 오류 결과 파일 없이 종료되던 P2를 수정했고 최종 재리뷰에서 미해결 P0/P1/P2 없음. 수치 routing 단위 테스트는 loader를 mock한 합성 배열을 사용하며 실물 corpus가 통과한 것으로 해석하지 않는다. 원본 실물 적격 데이터의 성공 로딩→학습→평가 경로는 데이터 부재로 미검증이다.

[검증·실행환경 메타데이터](visual-dynamics-2026-10-15/validation.json). A의 실패 때문에 B 구현을 보류하지 않았으며, 기존 데이터 검증이나 기본 비활성 수집 adapter의 제한을 풀지 않았다.

## 다음에 필요한 최소 작업

1. A 관문 미통과 결과를 바탕으로 수집 방식을 선택한다. 요청 실패를 명시하는 정지/작업자 승인 단일 단계 흐름부터 검토하며, 같은 관측 미세 최적화를 자동 재개하지 않는다.
2. host journal의 command identity와 실제 실행 결과 조회·driver trace를 `CaptureRunner`용 실물 adapter에 연결한다. exact before/after 원본·M4·정착/순서 증거를 현재 schema에 맞춰 보존하고 flag만 풀어 실물 동작을 허용하지 않는다.
3. 별도 승인 후 M4 영점/기구 범위와 한 축 최소 제한 출력·취소·off·잔여 이동/정착을 검증한다. 장치 구동·업로드·서비스 재시작은 이번 실행에 포함하지 않았다.
4. 별도 session으로 적격 실물 행동 데이터를 수집해 고정 manifest를 만들고 위 CLI로 무변화/선형 모델을 비교한다. 이후 실제 제한 조작에서 정책 비교를 해야 취업 포트폴리오의 학습 효과를 주장할 수 있다. 예측 MAE 개선과 조작 성공 개선은 별도 관문이다.
