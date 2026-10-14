# 데이터셋 manifest·분할 검증 — 2026-10-14

> 현재 상태 (2026-10-14): 이 단계는 `2bda97e`에 커밋되었다. 최신 코드 수정은 `73132e9`이며, 수정 후 전체 **372개 테스트 통과(10.949초)**와 독립 재리뷰를 기록했다. 아래 단계별 테스트 수는 당시 결과로 보존한다. 완료 범위·미완료 구현·다음 학습 단계는 [종합 계획서](../ai-manipulation-plan-2026-10-14.md), 마지막 회귀는 [시계 오차 수정 기록](2026-10-14-dry-run-collector-software.md#중간-리뷰-후-실행-전-시계-오차-검사-수정)을 기준으로 한다. 오늘 새 실물 검증이나 실제 데이터 학습은 하지 않았다.

기준 커밋: `9ad4935` (`feat: collect immutable dry-run action transition evidence`).
이전 dry-run runner 변경은 독립 리뷰와 전체 331개 테스트를 통과한 뒤 커밋했다.
이 문서는 이후 S5a의 장치 없는 데이터셋 준비 구현과 검증을 기록한다.

로봇팔·Pi·카메라 접속, 업로드, 모터 구동, 실제 데이터 수집, 모델 학습은 수행하지 않았다.
Mac의 Python 표준 라이브러리와 로컬 파일만 사용했다. 합성 데이터를 실물로 바꾸거나
기존 vision 데이터셋을 행동 데이터로 변환하지 않았다.

## 구현과 완료 기준

| 파일 | 구현과 검증 |
|---|---|
| `learning/dataset.py` | 명시한 run 목록으로 모든 시도 inventory 생성, 원본 파일/hash·journal·snapshot·raw/summary 연결 재검증, 적격 여부 재계산, manifest 재검증 |
| `learning/provenance.py` | 중첩 합성 표시·합성 참조/버전/adapter 감지, 물리 수집/교정의 명시적 선언 요구, 알려진 fixture 이미지 hash 집합 |
| `learning/splits.py` | 세션/episode/이미지 hash/원본 frame/명령/배치 그룹의 연결 성분 분할, 독립 교차 분할 검사 |
| `learning/artifacts.py` | 기존 안전한 원본 읽기를 재사용하는 파일 descriptor 생성 함수 |
| `tools/build_action_dataset.py` | 새 디렉터리에 manifest·validation 저장, 읽기 전용 재검증, 학습 준비 gate |
| `tests/test_action_dataset.py` | 실제 임시 파일 통합 15개: 합성 제외, 변조·누락, 원본 연결, 중복, 중단, 수동 manifest 조작, 순서·잘못된 참조, CLI |
| `tests/test_dataset_splits.py` | 구조 fixture 13개: 재현성, 세션/episode·전후 hash·frame·배치 연결, 중복·분할 누수, 빈 집합 |
| `tests/test_dataset_provenance.py` | 계약 fixture 9개: 명시적 출처, 중첩 합성 오염, 교정 bool, 알려진 이미지 hash |

## 파일과 적격 판정

출력은 `manifest.json`, `validation.json` 두 파일이다. 기존 디렉터리를 덮어쓰지 않는다.
manifest는 원본 run의 절대 경로, session/run descriptor, 구조화된 journal의 SHA-256,
각 시도의 summary descriptor·이미지 SHA-256/크기·원본 frame ID·action ID,
재계산한 검증 결과·제외 사유·분할 정책/seed/배정을 포함한다. 원본 이미지는 복사하거나
재인코딩하지 않으며 원본 run 디렉터리와 함께 보관해야 한다.

`validate_manifest`와 CLI `--validate`는 원본 파일을 다시 읽는다. 이전
`trainingEligible`, cached `validation`, manifest의 `eligible`/`readyForTraining`를
그대로 신뢰하지 않는다. 입력 파일과 manifest가 달라지면 재검증에 실패한다.

다음 조건을 모두 만족하는 시도만 분할 후보가 된다.

- session/run/attempt 종료와 인덱스·ID가 일치하고 원본 파일 hash/크기가 유효하다.
- config/calibration snapshot hash가 metadata와 연결되고 세션의 정책/metadata가 일치한다. config에 qualityPolicy가 있으면 세션과 동일한 객체여야 하며, physical config에는 이 선언이 필수다.
- raw 관측과 행동이 transition 요약과 일치한다. 관측의 로컬 이미지 descriptor 부착만 허용한다.
- before 원본/이미지 → 실행 의도 → 행동 결과 → after 원본/이미지 순서가 맞는다.
- producer 이미지 hash·저장 hash·expected hash가 일치하며 두 원본 이미지가 존재한다.
- transition schema와 신선도·시계·출력·종료 검증을 통과하고 완료/known_terminal이며 수집 오류·누락이 없다.
- session/run/summary/transition collection은 mode/evidence `physical`, config는 mode `physical`, calibration은 `physicalCalibration=true`를 명시한다. 미상 출처를 실제 데이터로 추정하지 않는다.
- 어떤 중첩 기록에도 합성 표시가 없고, 알려진 dry-run fixture 또는 이번 입력의 합성 데이터에 등장한 이미지 hash가 아니다.

`taskSuccess=false` 자체는 데이터 품질 제외 조건이 아니다. 성공 여부와 무관하게 위 조건을
적용한다. 실패·중단·누락·불명 시도도 manifest 행으로 보존하며 분할 배정만 하지 않는다.
원본 무결성이 깨진 run은 그 run의 모든 시도를 제외한다. 중복 source/session/sample/action
식별자와 입력 오류는 전체 manifest의 valid/ready 판정을 실패시킨다.

## 분할과 누수 차단

기본 분할은 train/validation/test 목표 비율 80/10/10이다. 세션을 통째로 유지하므로
같은 `(sessionId, episodeId)`의 인접 행동도 쪼개지지 않는다. 세션을 가로질러 동일 JPEG
SHA-256, sourceFrameId, `(bootId, commandId)`, 선택적 `metadata.splitGroupId`가 연결되면
전이적으로 같은 그룹에 배치한다. 배치를 반복한 세션들은 수집자가 공통 splitGroupId를
기록해야 한다. 중복 실행 명령은 같은 분할 안에서도 유효한 별도 샘플로 인정하지 않는다.

그룹 sample ID 목록과 seed의 SHA-256으로 배정한다. 같은 입력/seed이면 입력 순서나
이미지 경로 이름에 영향받지 않는다. 작은 집합이나 큰 연결 그룹 때문에 분할이 비면
그대로 남기며 억지로 그룹을 쪼개지 않는다. 비율은 보장값이 아니다.
독립 validator가 assignment 누락/추가·잘못된 label과 모든 그룹의 교차 분할을 검사한다.

새 데이터를 추가하면 연결 그룹과 배정이 바뀔 수 있다. 실험 시작 전에 명시적 입력 목록과
manifest를 고정하고 별도 버전으로 관리한다. 모델/전처리/정규화는 아직 구현하지 않았으며,
후속 단계에서 이 manifest의 train에만 통계를 fit하고 validation/test로 fit하지 않는
계약과 테스트가 필요하다. test 결과를 보고 seed를 고르는 운영은 이 코드만으로 막지 못한다.

## 재현과 결과

저장소 루트에서 실행한다. `RUN_ID`, `MANIFEST_ID`는 실제 저장 경로로 대체한다.

```sh
python3 tools/build_action_dataset.py --runs datasets/action_transitions/dry_run/RUN_ID --output-dir datasets/action_transitions/manifests/MANIFEST_ID
python3 tools/build_action_dataset.py --validate datasets/action_transitions/manifests/MANIFEST_ID/manifest.json
python3 tools/build_action_dataset.py --validate datasets/action_transitions/manifests/MANIFEST_ID/manifest.json --require-ready
python3 -m unittest tests.test_action_dataset tests.test_dataset_splits tests.test_dataset_provenance -q
python3 -m unittest discover -s tests -q
```

| 검증 | 결과 |
|---|---|
| 이번 추가 테스트 | 37개 통과 |
| manifest 커밋 당시 전체 회귀 | 368개 통과, 8.945초 |
| 기존 dry-run 10개 세션 / 11개 시도 | inputIssues 0, eligible 0, excluded 11 |
| 실제 저장 manifest 생성·읽기 전용 재검증 | valid=true, 각각 종료 코드 0 |
| 같은 manifest의 `--require-ready` | readyForTraining=false, 종료 코드 2 |
| train / validation / test | 각각 0 / 0 / 0 |

로컬 smoke artifact: `datasets/action_transitions/manifests/f2c3c4b1781a41c3867ac17fc4f0df3d/manifest.json`.
원본과 결과는 `.gitignore`의 datasets 아래 있으며 Git에는 코드·테스트·검증 문서를 남긴다.

기본 종료 코드 0은 **일관된 inventory**를 의미하며 학습 준비 완료가 아니다. 학습 호출자는
반드시 `--require-ready` 또는 검증 결과의 `readyForTraining=true`를 요구해야 한다.
이 gate는 유효한 실제 출처 후보와 비어 있지 않은 세 분할만 확인하며, 데이터량 충분성이나
분포 대표성·물리 성능을 판정하지 않는다. 현재 모든 원본은 합성이므로 적격 0이 정상이다.

## 코드 리뷰

독립 리뷰에서 잘못된 artifactRef/hash 타입이 dict/set lookup에서 예외를 발생시키는
문제를 확인하고, 타입 검증과 회귀 시험으로 수정했다. 원본/저장/expected 이미지 hash
일치, 이벤트 순서, transition 적격 판정, 중첩 합성 표식과 manifest 정책 객체 공유도
점검·보강했다. 커밋 전 재리뷰에서는 config snapshot과 session의 qualityPolicy 불일치가
검증을 통과하는 문제를 추가로 발견해 무결성 오류로 처리하고 회귀 시험을 추가했다.
수정 재현과 독립 재리뷰에서 남은 확정 차단 문제는 발견하지 못했다.
manifest 커밋 당시 전체 368개 회귀 통과는 소프트웨어 범위의 결과다. 후속 runner 시계 오차 수정 후 최신 전체 결과는 372개 통과이며, 실제 학습 적격 데이터는 여전히 0개다.

## 한계와 다음 실물 조건

- hash는 정확히 같은 bytes와 변경을 검출한다. 재인코딩·유사 장면·라벨/특징의 간접 누수까지 검출하는 지각적 중복 검사는 아니다.
- 출처 선언 검사는 하드웨어 인증이 아니다. 파일과 모든 참조를 일관되게 위조하는 행위나 표시 없는 미지의 합성 이미지를 물리 데이터와 완전히 구분하지 못한다.
- 테스트의 물리 선언 예제는 메모리 내 계약 fixture이며 실제 수집 증거가 아니다. 실물 적격 데이터가 들어간 전체 파이프라인 검증은 아직 하지 않았다.
- 원본 디렉터리는 검증·학습 중 변경하지 않아야 한다. 이번 단계는 파일 잠금 또는 훈련 직전 byte 소비까지 연결한 loader를 제공하지 않는다. 후속 loader도 사용할 bytes를 다시 hash 검증해야 한다.
- 추가 센서나 대형 VLA/GPU 학습은 필요 조건이 아니다. 다음 소프트웨어 작업은 이 gate를 사용하는 특징 추출/기준선과 train-only 전처리 시험이다.
- 실제 데이터는 기존 [실물 세션 준비 절차](../ai-manipulation-plan-2026-10-14.md#7-다음-실물-세션-준비와-실험)에 따라 장치/firmware 식별, 안전 heartbeat, M4 freshness, 원본 카메라 시계와 MCU 출력 종료의 교정, 최소 한 축 행동과 정착 시험을 먼저 수행한 뒤 수집해야 한다. 이번 CLI에는 구동 기능이 없다.

학습에 의한 조작 성능 개선은 아직 주장하지 않는다. 이후 별도 실물 세션에서 기준선과
학습 정책의 접근·집기·배치 결과를 같은 조건으로 비교하고 실패/불명을 포함해 보고한다.
