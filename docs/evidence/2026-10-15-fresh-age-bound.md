# 촬영 전 대기를 분리한 나이 상한 — 2026-10-15

**100ms를 유지하며 새 캡처의 촬영 시각 하한을 계산했다.** 이전 원본 재생에서 fresh 측정 수락 0→14/24, 새 비구동 실측에서 같은 fresh 프레임의 기존/새 계산 수락은 0→17/24다. **실제 처리 속도가 빨라진 결과가 아니라, 중복 포함하던 촬영 전 대기를 제외한 상한 계산 개선이다.** 원본·측정값 저장 완료 시점에는 여전히 통과 0이다.

## 계산과 제한

`S`는 실제 첫 GET socket send 직전의 host 시각, `R`과 `C`는 같은 CAM boot의 요청 처리 시작·first-DMA 시각이다. 직접 GET의 인과관계는 `S ≤ host(R)`이며 fresh 증거가 `C > R`를 보장한다. CAM clock 속도가 host 대비 최대 `1+ρ`라는 가정에서:

```text
deltaLowerMs = max(0, (C-R)/1000/(1+ρ) - ε)
captureHostLower = S + deltaLowerMs/1000
ageUpperMs(T) = (T - captureHostLower)*1000
```

`C-R`는 먼저 정수 µs로 뺀다. 카메라 보고 나이를 새 식에 다시 더하지 않는다. 기존 송신 시작 필드와 full-request 비교 상한은 보존하며 `sourceCaptureLowerBoundMonotonic`을 별도 기록한다. 측정·최종화·영속 저장·HTTP 진단이 공용 `source_age_upper_ms()`를 사용한다.

- `ρ=0.001`(1000ppm), `ε=0.002ms`는 **명시적인 실험 가정/여유**다. 이번 원본에서 추정했거나 장비로 교정한 오차 상한이 아니다. `freshClockErrorCalibrated=false`, `controlEligible=false`, `trainingEligible=false`를 유지한다. 실제 장비가 이 가정을 충족하는지와 광학 timestamp 검증은 미완료다.
- 새 식은 exact fresh query, mode/요청 timestamp/반환 수, 동일 frame/boot/hash, 순서가 유효한 host trace, 실제 send-start 의미가 모두 확인될 때만 적용한다. first-send 증거가 없으면 기존 full-request 상한을 유지한다. 기본 캡처 계산은 바꾸지 않는다.
- 요청→촬영이 200ms 선택 예산 이상이거나, 요청→CAM 나이 표본 시각 구간이 오차 여유를 적용해도 GET 송신→헤더 수신 구간보다 길면 거부한다. 불가능한 증거를 잘라서 수락하지 않는다. 음수·NaN·무한·bool 오차 설정도 거부한다.
- `100ms` 경계, 계산/최종화 시간, 저장 I/O 이후 경과를 모두 포함한다. 100ms 통과는 해당 측정 시점의 shadow ROI 정책 수락이며 미래 소비 시점이나 실제 모터 구동 적격성을 의미하지 않는다.

## 소프트웨어 검증

전체 **506개 테스트 통과(21.419초)**. 신규 10개 항목은 비대칭 네트워크/clock rate, 긴 CAM uptime 정수 차이, 잘못된 fresh query, 불가능한 시간 구간, 200ms 경계, 잘못된 오차 설정, fallback, 진단식 일치, 계산/최종화/저장 이후 검사 등을 다룬다.

오차 여유가 없으면 99.99ms지만 속도·timestamp 여유를 포함하면 100ms를 초과하는 사례에서 **수락 0·좌표 없음·모터 명령 0**을 회귀 검증했다. 실제 물리 모터 테스트는 아니다. 독립 리뷰 에이전트가 관련 32개 테스트를 별도 실행했고 구현에서 P0/P1/P2를 발견하지 않았다.

## 기존 원본 재생 — 기록 시각 고정

[이전 비교](2026-10-15-fresh-capture-live.md)의 4개 run, 비교 48장+anchor 4장, 434 artifact, 104 event를 hash/종료 report로 검증했다. 실제 JPEG를 다시 계산하면서 당시 측정 시작·완료 시각을 가상 clock으로 제공했다. score/peak margin/scale 일치(플랫폼 부동소수 허용차 1e-5), 기본 캡처 수락/나이 일치, anchor 만료 거부 유지를 검사했다. Mac의 재계산 소요 시간을 Pi 속도 개선으로 해석하지 않는다.

| 기존 fresh 24장 | 이전 식 | 새 식 |
|---|---:|---:|
| 측정 수락 | 0/24 | 14/24 |
| 측정 나이 상한 중앙값 (유효 23장) | 150.87ms | 98.24ms |
| 저장 후 나이 상한 중앙값 (유효 23장) | 189.60ms | 138.11ms |
| 저장 후 100ms 통과 | 0/24 | 0/24 |

기존 anchor_expired 1장은 그대로 거부하고 나이 통계에서 제외했다. 기본 24장은 수락 3/24와 나이 계산 모두 동일하다. [재생 코드·결과](fresh-age-2026-10-15/replay.json).

## 새 비구동 실측

배터리함 분리·USB/Pi 유지 상태에서 Pi 임시 `/tmp/motionbrain-fresh-age-20261015`의 소스를 사용했다. 펌웨어 업로드·서비스 재시작 없이 기존 CAM boot `5bfe5283286bc8599cae44c1612b6869`, QVGA/quality 15를 유지했다. 기본→fresh→fresh→기본 순서, 각 12장, 매 run 새 semantic anchor, 3 ROI worker/병렬 영속 저장 조건이다.

| 실측 결과 | 기본 24장 | fresh 24장 |
|---|---:|---:|
| 수집 완료 | 24/24 | 24/24 |
| 측정 직후 100ms 및 ROI 수락 | 5/24 | 17/24 |
| 측정 나이 상한 중앙값 | 116.05ms | 93.31ms |
| 같은 프레임에 이전 send-start 식 적용 | 5/24, 116.05ms | 0/24, 150.15ms |
| 저장 직후 나이 상한 중앙값 | 154.73ms | 133.36ms |
| 저장 직후 100ms 통과 | 0/24 | 0/24 |

fresh 나이 상한 범위는 측정 82.94–119.45ms, 저장 117.93–323.16ms다. fresh 거부 7장은 모두 나이 초과였으며 이번 실행에는 anchor 만료가 없었다. fresh 24장 모두 exact 요청 이후 원본 증거가 유효했고 후보 2개씩 반환했다. 카메라 실패·복구·fresh 거부 누계 증가는 0, boot/전 구간 출력 로그 동일, 모터 명령 0이다. 기본 1장/fresh 2장은 운영 클라이언트의 캡처가 끼어 서버 시간 연결 통계에서 제외됐지만 원본 수집 실패는 아니다.

run 순서: `92a6e7dd9191474c9b763c35da86e976`(기본 2/12), `3a8618c39d0b4793bec97acad1bc7875`(fresh 9/12), `904e00772a10418cb1822fea2c629317`(fresh 8/12), `197e7b1f79a34fc4a13753f9cfb64c6c`(기본 3/12). 새 실측도 52 JPEG·434 artifact·104 event·실행 소스 hash·공용식의 측정/저장 값 일치를 검증했다. 원본은 Git 제외 `datasets/fast_probes/<run ID>` 및 Pi 임시 runs에 보존했다.

종료 시 메인 IDLE, motorEnabled=false, 5개 출력 0, 기존 web disarm ID 9 유지. Pi perception ok/fresh, 연속 오류 0, 세 서비스 active/NRestarts=0, throttled=0x0다. 생산 서비스에는 새 Python 코드를 배포하지 않았다.

최종 독립 리뷰에서 두 분석기를 재실행해 저장된 JSON과 완전히 일치함을 확인했다. 코드·원본/실측 무결성·소스 hash·수치·문서의 제한을 검토하여 P0/P1/P2 발견 없음.

## 다음 작업

**측정 결과를 실제로 소비하는 시점의 100ms 재검사와 영속 기록 완료를 구분하는 경로를 설계·테스트한다.** 이번 결과는 측정 17/24 통과라도 기록 완료를 기다리면 모두 기한을 넘긴다는 것을 보여 준다. 먼저 단일 프레임의 소비 시점·기록 완료 시점·실패 시 중단 규칙을 명시하고, 지연/중단/기록 실패를 가상 시간으로 검증해야 한다. 기록 보존·무결성이나 100ms를 완화하지 않는다. 모터 실행, clock/M4 실물 교정, 장시간 안정성, 실제 조작 데이터 수집·학습 효과 검증은 여전히 별도 단계다.

[전체 테스트](fresh-age-2026-10-15/tests.log), [재생 코드](fresh-age-2026-10-15/replay.py), [실측 프레임별 결과](fresh-age-2026-10-15/live.json), [실측 검증 코드](fresh-age-2026-10-15/analyze_live.py), [실험 코드](fresh-age-2026-10-15/compare.py), [소스·장치 검증](fresh-age-2026-10-15/validation.json), [시작 상태](fresh-age-2026-10-15/before.json), [종료 상태](fresh-age-2026-10-15/final.json).
