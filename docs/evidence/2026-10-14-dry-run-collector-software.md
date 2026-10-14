# 행동 전후 수집 runner dry-run — 2026-10-14

기준 커밋: `ca48f04` (`feat: preserve source observations and validate action transitions`).
관측·schema 변경은 독립 리뷰에서 조기 완료 pulse/종료 상태 모순을 수정하고
291개 테스트를 통과한 뒤 이 커밋으로 저장했다. 이 문서는 그 이후 S4b의
장치 없는 수집·저장·감사 구현을 기록한다.

로봇팔·Pi·카메라 접속, firmware 업로드, 모터 구동은 수행하지 않았다.
새 CLI는 합성 adapter만 만들며 장치 주소·실행 허용 옵션이 없다.
추가 센서·GPU·클라우드·ROS 설치는 dry-run의 선행조건이 아니다.

## 구현과 완료 기준

| 파일 | 구현과 검증 |
|---|---|
| `learning/runner.py` | before → exact 이미지 → 실행 직전 검사 → durable intent → 단일 가상 실행 → 결과/off 확인 → 가상 정착 대기 → after → 검증·종료 기록 |
| `learning/artifacts.py` | 원본 bytes 저장, SHA-256/크기/readback 확인, 원자적 생성·fsync, 덮어쓰기·경로 탈출·symlink 차단, 이벤트 읽기와 무결성 감사 |
| `learning/dry_run.py` | 가상 시간, 원본 2×2 JPEG, 합성 카메라/M4/MCU adapter, 실패 주입 10종 |
| `tools/capture_action_transitions.py` | 명시적 `--dry-run`, 읽기 전용 `--audit-run`, 종료 코드, SIGINT/일반 SIGTERM 중단 처리 |
| `learning/schema.py` | collection 또는 관측/action의 synthetic 표시가 있으면 `synthetic_dry_run` 학습 제외 |
| `tests/test_action_transition_capture.py` | 17개 통합 테스트: 정상·누락·중단·저장 실패·ID 혼선·실행 전 stale·감사·CLI |
| `tests/test_transition_artifacts.py` | 13개 실제 임시 디스크 테스트: 원본 보존·오염·삭제·경로·원자적 쓰기 실패 |
| `tests/test_dry_run_adapter.py` | 9개 가상 시계/원본 JPEG/실패 adapter 시험 |
| `tests/test_action_transition_schema.py` | 합성 데이터가 wrapper 없이도 실물 학습 적격으로 섞이지 않는 회귀 시험 추가 |

기존 policy episode 도구와 호스트 명령 journal을 다시 만들거나 교체하지 않았다.
현재 runner는 단일 step 수집 구조이며 CLI의 여러 step도 가상 adapter에서만 실행된다.

## 재현

저장소 루트에서 Python 표준 라이브러리만으로 실행한다.

```sh
python3 tools/capture_action_transitions.py --dry-run --steps 2
python3 tools/capture_action_transitions.py --dry-run --scenario after_image_missing
python3 tools/capture_action_transitions.py --dry-run --scenario unknown_result
python3 tools/capture_action_transitions.py --dry-run --scenario interrupt_during_action
python3 -m unittest discover -s tests -q
```

기본 저장 위치는 `datasets/action_transitions/dry_run/<새 UUID>/`이다.
`--output-dir`로 부모 경로를 바꿀 수 있다. 기존 run 디렉터리를 재사용하거나 덮어쓰지 않는다.
stdout의 `runDir`를 다음 감사 명령에 전달한다.

```sh
python3 tools/capture_action_transitions.py --audit-run datasets/action_transitions/dry_run/RUN_ID
```

dry-run 종료 코드: 정상 수집 `0`, 데이터 누락/불명/제외/저장 실패 `2`, 중단 `130`.
첫 이상이 발생하면 이후 step은 실행하지 않는다. 감사 종료 코드 `0`은 **저장 기록의
무결성이 맞는다**는 뜻이다. 정상적으로 기록된 실패/불명/중단 run도 감사에는 통과할 수
있으며, 이는 실험 성공이나 학습 적격 판정이 아니다.

## 저장 계약

```text
session.json                     session·정책·버전과 snapshot 참조
snapshots/config.json            설정 원본, SHA-256으로 metadata에 연결
snapshots/calibration.json       합성 교정 표시, 실물 교정 아님
events/00000001.json ...         순서 있는 불변 이벤트 파일
raw/<transitionId>/before.json   producer가 반환한 원본 관측
raw/<transitionId>/action.json   반환된 요청·결과·출력 trace 원본
raw/<transitionId>/after.json    producer가 반환한 원본 관측
images/<transitionId>/before.jpg 원본 JPEG bytes
images/<transitionId>/after.jpg  원본 JPEG bytes
attempts/<transitionId>.json     부분/완전 transition, 검증·제외·누락·오류
run.json                        세션 종료 표식과 attempt 목록
```

누락된 항목의 파일은 만들어 채우지 않는다. `missing`, `errors`, `executionState`로 남긴다.
원본 관측 JSON에 로컬 이미지 경로나 성공 기본값을 덧씌우지 않고, 별도 transition 복사본에
저장 경로를 붙인다. timestamp/sequence/command ID도 수집기가 성공에 맞게 고치지 않는다.

이미지는 `sourceFrameId`로만 exact 조회한다. 캐시 퇴거/누락이면 최신 이미지로 대체하지 않는다.
bytes를 재인코딩하지 않고 저장한 뒤 다시 읽어 SHA-256과 크기를 확인한다. producer의
`imageSha256`와 다르면 받은 원본을 보존하며 descriptor의 `expectedSha256`, `sha256`,
`hashMatches=false`와 오류를 남긴다. before hash 불일치는 행동 호출 전에 중단한다.
같은 JPEG hash를 가진 서로 다른 frame ID는 가능하며 합치지 않는다.

각 파일은 임시 파일의 fsync 후 hardlink로 최초 생성한다. 기존 파일에 추가 덮어쓰기를 하지
않으며 이벤트도 한 줄 append 대신 별도 순번 JSON 파일로 저장한다. 디렉터리 fsync와
readback을 수행한다. 이는 소프트웨어 수준의 불변 기록 규칙이며 외부 삭제를 막는 WORM
저장소나 암호학적 서명이 아니다. 데이터와 manifest를 함께 악의적으로 수정하는 위협까지
보증하지 않는다. 저장소 전체 손실 시 새 성공/미실행 증거를 만들어내지 않는다.

## 중단·불명·복구 의미

- `attempt_started`와 before 원본/이미지를 먼저 기록한다. 기록 실패·stale·held·신원 미상은
  실행을 차단한다. 설정·이미지 저장 무결성도 실행 전에 확인한다.
- `action_intent`를 디스크에 확정한 뒤에만 adapter를 한 번 호출한다. intent 저장과
  checkpoint 동안 지연될 수 있어 바로 호출하기 전에 before 신선도를 다시 검사한다.
- intent 이후 terminal 결과를 확정하지 못하면 `outcome_unknown`이다. 보수적으로 intent
  뒤 호출 직전 중단도 이 상태로 남긴다. 미실행이 확실한 앞 단계 실패는 `not_sent`다.
- terminal 결과는 원래 request, MCU record의 boot/command ID와 행동 내용이 모두 일치하고
  outputOff가 확인됐을 때만 `known_terminal`로 기록한다. 다른 명령의 결과는 증거가 아니다.
- 결과 불명·중단·수집 실패 뒤 재호출하지 않는다. 이 runner에는 resume/retry API가 없다.
- 일반 KeyboardInterrupt/SIGTERM은 가능한 부분 기록과 종료 요약을 남긴다. SIGKILL,
  프로세스 crash, 디스크 실패는 요약까지 남기지 못할 수 있다. read-only audit는 남은 intent,
  결과와 artifact를 검사하고 `attempt_incomplete`/`session_terminal_missing` 등을 보고한다.
- audit도 이미지·원본·요약을 재검증한다. 중간 이벤트가 없거나 깨지면 연속 prefix만 읽고
  결과를 보수적으로 해석한다. terminal 표식 없는 이벤트 prefix로 완료를 선언하지 않는다.
  누락된 기록을 이유로 행동을 재생하거나 장치 상태를 변경하지 않는다.

## 소프트웨어 증거

- 전체 회귀: **331 tests, OK** (9.468초). 변경 파일 py_compile와 `git diff --check` 통과.
- 커밋 전 추가 독립 리뷰에서 현 dry-run 범위의 차단 결함을 발견하지 못했다.
  전체 **331 tests, OK** (10.321초)를 다시 확인했다. 후속 manifest에서는 감사 통과와
  학습 적격을 분리하고 중첩 합성 표식·raw/summary 연결을 추가 검사하기로 했다.
- CLI 실제 파일 저장: 아래 10개 시나리오 모두 기대 종료 코드·호출 횟수 확인.
- 정상 시나리오에서 2회 수집, JPEG 원본·hash·관측/행동 연결·config snapshot 확인.
- 외부 네트워크 socket을 금지한 통합 테스트에서도 정상 수집 통과.
- 독립 리뷰에서 발견한 다른 명령 결과의 terminal 오인과 intent 저장 중 신선도 만료를
  수정하고 재현 회귀 시험을 추가했다. 수정 후 재리뷰에서 남은 차단 문제를 발견하지 못했다.

| 시나리오 | 가상 execute 호출 | 결과 |
|---|---:|---|
| complete (`--steps 2`) | 2 | completed, 코드 0 |
| before_missing | 0 | aborted/not_sent, 코드 2 |
| before_hash_mismatch | 0 | 원본 보존, aborted/not_sent, 코드 2 |
| stale_before | 0 | aborted/not_sent, 코드 2 |
| after_missing | 1 | after 관측·이미지 누락, 코드 2 |
| after_image_missing | 1 | after metadata 보존·이미지 누락, 코드 2 |
| after_hash_mismatch | 1 | 불일치 bytes와 두 hash 보존, 코드 2 |
| unknown_result | 1 | outcome_unknown, 코드 2 |
| dropped_outputs | 1 | output_history_dropped로 제외, 코드 2 |
| interrupt_during_action | 1 | interrupted/outcome_unknown, 코드 130 |

실행 원본은 ignored `datasets/` 아래에 있고 실물 데이터와 섞지 않는다. 이번 로컬 CLI
실행 목록은 `datasets/action_transitions/dry_run/smoke-630b281ff7b34c66be78350047426e8a.json`에 있다.
이 파일은 커밋된 evidence fixture가 아니며 위 명령으로 새로 재현할 수 있다.

## 중간 리뷰 후 실행 전 시계 오차 검사 수정

`2bda97e`까지의 전체 리뷰에서 실행 직전 검사와 schema의 source-age 계산 차이를
확인했다. sourceAgeMs 약 20ms, 교정 오차 1ms, 허용 한계 20.5ms인 경우 기존 runner는
가상 행동을 1회 호출한 뒤 schema에서 제외했다.

실행 전 두 검사 지점 모두 `sourceAgeMs + 교정 오차(ms) + 추론 이후 경과시간(ms)`에
동일한 허용 한계를 적용한다. 교정 여부가 true가 아니거나 오차가 누락·잘못된 타입·
음수·비유한이면 호출 전에 중단한다. 원본 관측을 수정하거나 오차를 0으로 대체하지 않는다.
M4 age 계산과 저장된 intent 이후의 보수적 `outcome_unknown` 구분은 유지한다.

추가한 4개 회귀 테스트의 핵심 조건:

| 조건 | 가상 execute 호출 / 결과 |
|---|---|
| 약 20ms 관측 + 1ms 오차 > 20.5ms 한계 | 0회, aborted/not_sent, 원본 JPEG·관측 보존 |
| 약 20ms 관측 + 1ms 오차 + intent 지연 2ms > 22.5ms 한계 | 0회, aborted/outcome_unknown, intent 보존 |
| 오차 0 또는 1ms를 포함해 정확히 한계인 관측 | 1회, completed; 합성 학습 제외 유지 |
| null·음수·bool·문자열 오차 또는 미교정 표시 | 0회, aborted/not_sent |

앞의 두 테스트는 수정 전 코드에서 모두 executeCalls=1로 실패하는 것을 확인했다.
수정 후에는 실행 0회 및 감사의 not_sent/outcome_unknown 구분까지 통과했다.
전체 suite **372개 통과(10.949초)**, 독립 재리뷰에서 남은 차단 문제 없음.
이 검증은 가상 시계·출력과 실제 임시 저장 파일을 사용하며 실물 구동 시험이 아니다.

## 남은 작업과 실물 관문

이번 완료는 **장치 없는 데이터 계약·수집 실패 처리**다. 합성 adapter의 clock 교정·M4 실측·
driver write 필드는 형태를 시험하기 위한 값이며 실제 측정 증거가 아니다. 모든 runner 결과는
`mode=dry_run`, `evidence=synthetic`, `trainingEligible=false`다. 학습 개선·물리 정지·집기
성공을 입증하지 않는다. 기존 ROS 전체 빌드와 새 firmware 실물 검증도 여전히 남아 있다.

다음 소프트웨어 단계는 저장 artifact로부터 데이터셋 manifest를 만들고 hash/episode 단위
분할 검사를 구현하는 것이다. 실물 adapter는 기존 journal, MCU capability/boot/결과 조회,
원본 perception API와 명시적으로 연결해야 한다. 이 dry-run adapter를 physical로 표시만
바꾸어 사용하지 않는다. collector checkpoint/cancel은 현재 물리 stop 호출이 아니다.

실물 수집 전에는 [MCU 실물 관문](2026-10-14-bounded-actions-software.md)과
[카메라·시계·관측 관문](2026-10-14-source-observations-software.md)을 충족해야 한다.
준비 상태는 모터 전원 분리, 보드/firmware/포트 식별, STM32 heartbeat,
M4 AS5600 freshness, 안정된 카메라 시야다. 먼저 무구동 상태에서 exact 이미지·hash와
boot/ID/결과 조회를 확인한 후, 교정된 한 축의 작은 행동으로 output-off→정착→새 촬영
연결을 검증한다. MCU/host 시간 구간의 매핑·오차는 실측 근거가 필요하다.
