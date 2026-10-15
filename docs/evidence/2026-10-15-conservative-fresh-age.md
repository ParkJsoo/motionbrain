# 시계 속도·2µs 여유 없는 fresh 영상 나이 상한 — 2026-10-15

**선택형 `send_start` 상한을 구현하고 전체 538개 테스트, 원본 재생, Pi 비구동 비교를 완료했다.** 새 프레임의 나이를 `소비 시각 − 첫 GET 송신 시각`으로 제한한다. 100ms 소비·250ms 저장 기준은 유지했다. Pi 실제 소비 통과는 기존 방식 21/24개, 보수적 방식 5/24개였다. 모터 명령은 0회이며 제어·학습 적격으로 승격하지 않았다.

## 계산 계약과 구현

정확한 `?fresh=1` 응답은 같은 CAM boot에서 `capturedUs > requestedUs`를 보장한다. 따라서 **Pi 첫 GET 송신 ≤ CAM 요청 처리 < first-DMA 촬영 시점 ≤ 관측 소비**라는 사건 순서가 성립한다. Pi의 동일 monotonic clock에서 `소비 − 첫 송신`은 first-DMA 이후 경과 시간의 보수적 상한이다.

- `learning/source_age.py`: `fresh_age_mode="send_start"`를 추가했다. CAM 차분 시간을 Pi 시간으로 변환하지 않고, ±1000ppm·2µs 여유·CAM 200ms 차분 필터도 사용하지 않는다. 기존 `clock_margin`이 기본값이며 비교용 경로로 유지된다.
- receipt의 원본 hash/frame/boot/host identity, exact fresh 증거, 인증된 **첫 socket send 직전** trace와 순서를 확인한다. 증거가 없거나 잘못되면 거부하며 기존 상한으로 fallback하지 않는다. 이 의미 표시는 로컬 transport의 출처 계약이지 암호학적 인증은 아니다.
- `learning/fast_observation.py`, `learning/observation_consumer.py`: 계산 완료와 실제 shadow 소비 시점에 같은 선택 모드로 100ms를 재검사한다. consumer는 measurement에 저장된 상한을 신뢰하지 않고 receipt에서 다시 만든다. 모드 불일치는 중단한다.
- `learning/fast_probe.py`, `learning/capture_timing.py`, CLI: session·측정·소비·저장·진단에 모드를 기록한다. `send_start`에는 `--fresh-capture --connection-timing`이 필요하며 잘못된 설정은 I/O 전에 거부한다. CLI에서 connection timing은 `--capture-timing`도 필요하다.
- 저장 barrier 250ms, 저장 실패/지연 뒤 추가 촬영 중단, 원본/receipt 보존 규칙은 유지한다. shadow 소비는 메모리상 관측 사용 기록이며 모터나 실제 정책 실행이 아니다.

**제거 범위는 현재 fresh 프레임의 영상 나이 계산이다.** 기준 이미지(anchor)의 15초 만료 상한에는 기존 `CAM 차분 / 0.999 + 현재 프레임 상한 + 0.002ms` 가정이 남아 있다. 전체 관측 경로가 시계 가정 없이 검증됐다는 뜻이 아니다. same-boot 단조 timestamp·first-DMA 사건 의미도 여전히 전제이며, 광학 노출 시각을 교정한 결과가 아니다.

## 동일 원본·기록 시각 재생

원본 JPEG·receipt·terminal report·실행 소스 hash를 검증한 뒤, 동일 ROI 계산을 원래 측정/소비/저장 monotonic 시각으로 재생했다. ROI score·peak margin·scale은 원래 결과와 1e-5 이내로 같았고, 새 상한은 기존보다 작아지지 않았다. 새 방식의 소비 통과 집합은 기존 방식의 부분집합이다. **실행 시간을 고정한 판정 비교이며 처리 속도 benchmark가 아니다.**

| 자료 | 표본 | 기존 소비 통과 | 보수적 소비 통과 | 기존/보수적 소비 나이 중앙값 |
|---|---:|---:|---:|---:|
| 이전 shadow 시험 fresh 원본 (`77fc791` 소스) | 24 | 21 | 7 | 87.33 / 111.96ms |
| 이번 Pi 시험 전체 원본 | 48 | 40 | 12 | 94.04 / 110.46ms |

이번 원본 48개를 두 계산식으로 다시 평가한 40/48·12/48은 아래 실측 각 24개와 분모가 다르다. 원본을 중복 수집했거나 실물 통과 횟수를 늘린 것이 아니다.

## Pi 실제 비구동 비교

Pi `/tmp/motionbrain-conservative-fresh-20261015`에서 **기존 → 보수적 → 보수적 → 기존**, 각 12개를 수집했다. 모든 run이 fresh 캡처이며, 기존/보수적의 차이는 나이 계산식이다. 실패/저장 지연/결과 불명 시 다음 run을 시작하지 않는 비교기이며 이번에는 4개 모두 완료했다. 운영 서비스·펌웨어는 변경하지 않았다.

| 실제 선택 방식 | 계산 통과 | 소비 통과 | 소비 나이 최소/중앙값/최대 | 저장 barrier 중앙값/최대 |
|---|---:|---:|---:|---:|
| 기존 `clock_margin` | 23/24 | **21/24** | 79.20 / 93.78 / 105.71ms | 75.01 / 97.64ms |
| 보수적 `send_start` | 6/24 | **5/24** | 91.36 / 111.65 / 166.06ms | 73.86 / 91.19ms |

계산 직후에는 통과했지만 소비 때 100ms를 넘은 기존 2개·보수적 1개도 거부했다. 촬영 전 대기를 포함하는 보수적 상한 때문에 통과율이 낮아졌으며, 이는 ROI 정확도 저하나 시계 오차 가정 위반을 검출했다는 의미가 아니다. 서로 다른 실제 프레임/네트워크 조건의 작은 표본이므로 일반 성공률로 해석하지 않는다.

48개 저장 barrier 모두 250ms 안에 완료했다. 원본·측정 저장 시점의 영상 나이는 48개 모두 100ms를 초과했으므로, 소비 시점 통과를 저장 완료 시점 통과로 표현하지 않는다. 이번에는 저장 장애를 실물 주입하지 않았으며 해당 중단 규칙은 아래 가상 시간 회귀와 [직전 Pi 주입 시험](2026-10-15-pi-storage-faults.md)으로 구분한다.

원본 JPEG **52개**(anchor 4개 포함), artifact **484개**, event **104개**를 검증했다. frame 유일성·원본 hash 일치·fresh 계약·소스 hash·pending/final 소비 행·저장 barrier·다음 capture 순서·종료 보고를 재검증했다. 전후/각 run 사이 컨트롤러 IDLE·모터 비활성·출력 0·출력 로그 및 기존 disarm 명령 불변, CAM boot/failure/recovery 불변이다. Pi 3개 서비스 active/NRestarts=0, throttled=0x0, 운영 checkout `559db54` 유지다.

## 테스트·재현·리뷰

- 전체 `python3 -m unittest discover -s tests -q`: **538개 통과, 22.357초**. 새 production 회귀 10개 포함이다.
- 신규 10개: first-send 경계, rate 상수/큰 CAM 차분에 비의존, fresh/trace/identity 오류 거부, 100ms 경계, 위조된 상한 재계산, 모드 불일치, 모든 기록 단계 모드 일치, 잘못된 설정 시 요청 0회, 저장 지연/실패 뒤 추가 촬영 0회, 늦은 소비 0회.
- 별도 비교 runner 4개 통과: 정상 순서, 실패/halt, terminal 누락, timeout 후 다음 run·재시도 없음. 전체 538개에 이 4개는 포함하지 않는다.
- 구현 및 실험 검증기·결과를 독립 리뷰했다. 최종 리뷰와 소스 hash는 [validation.json](conservative-fresh-2026-10-15/validation.json)에 기록한다.

검증기 실행 (저장소 루트에서):

```sh
python3 docs/evidence/conservative-fresh-2026-10-15/analyze.py . /tmp/motionbrain-conservative-fresh-20261015
python3 docs/evidence/conservative-fresh-2026-10-15/replay.py . /tmp/motionbrain-conservative-fresh-20261015
python3 docs/evidence/conservative-fresh-2026-10-15/replay.py . /tmp/motionbrain-consumption-live-20261015 77fc791
```

[실측 집계](conservative-fresh-2026-10-15/live.json), [이전 원본 재생](conservative-fresh-2026-10-15/historical-replay.json), [이번 원본 재생](conservative-fresh-2026-10-15/live-replay.json), [시험 순서](conservative-fresh-2026-10-15/comparison-runs.json), [최종 상태](conservative-fresh-2026-10-15/final-status.json). 전체 원본과 receipt는 Mac `datasets/fast_probes/<runId>` 및 Pi 임시 시험 디렉터리에 보존했다. Git에 원본 이미지를 넣지 않았다.

## 다음 작업과 남은 실물 관문

1. **anchor 만료 판정도 CAM 속도/2µs 가정 없이 만들 수 있는 경로를 구현·비구동 검증한다.** 기준 이미지의 exact fresh 획득 증거를 확보하고 이후 Pi 단조 시각만으로 만료를 판단해야 한다. 기존 검출 결과와 새 anchor 원본의 동일성/대상 연결을 유지하며 단순히 수신 시각을 촬영 시각으로 치환하지 않는다.
2. 현재 보수적 소비는 5/24라 안정적으로 사용할 준비가 됐다고 볼 수 없다. 100ms를 유지하며 거부 시 동작 0회·유한 대기/중단 규칙을 정하고, 기존 원본으로 줄일 수 있는 전송/계산 비용을 우선 검토한다. 무한 재촬영으로 좋은 표본만 골라 성공률을 높이지 않는다.
3. 비구동 관측 계약이 정리된 뒤 M4 영점·기구 안전 범위·출력 종료/정착을 실제로 확인하고 제한 동작 수집으로 연결한다. 배터리 연결·모터 출력 시험은 별도 단계다. 실제 학습 적격 행동 데이터 수집, 기준선/작은 모델 학습과 조작 성능 비교는 아직 남아 있다.
