# Pi 실제 영상 shadow 소비 비구동 비교 — 2026-10-15

**실제 영상 48개에서 소비 직전 100ms 재검사를 수행했다. fresh 캡처는 21/24(87.5%), 기본 캡처는 8/24(33.3%)가 모의 소비됐고, 측정 때 수락됐어도 소비 때 100ms를 넘은 2개는 차단됐다.** 전 프레임의 저장 장벽은 250ms 이내에 완료됐다. 모터 명령은 0회이며, 이 수치는 객체 검출 정확도·조작 성공률·학습 효과가 아니다.

## 실험 범위와 조건

- 대상 소스: `77fc79102447802cceab0ffac724382c6d580083`. 기존 `feat/fast-observation`의 shadow 구현을 그대로 사용했다. production 코드·펌웨어 수정 없음.
- Pi 임시 경로 `/tmp/motionbrain-consumption-live-20261015`에서 실행했다. 운영 checkout은 `559db54` 그대로이며 서비스 재시작·펌웨어 업로드 없음.
- 사용자 확인 배터리함 분리 상태. 고정된 기존 카메라·컵 장면으로 fresh → 기본 → 기본 → fresh 각 12개를 수집했다. 배터리 물리 분리는 소프트웨어로 판정할 수 없으며, 전후 상태에서 `IDLE`, `motorEnabled=false`, 5개 출력 속도 0을 별도로 확인했다.
- `--shadow-consumption --max-persistence-ms 250 --capture-timing --connection-timing`, 기존 ROI·화질·영상 크기·3-worker 계산 설정 유지. 관측 소비 기준 **100ms**와 별도 저장 기한 **250ms**를 유지했다.
- 첫 run 실패·소비 중단·저장 지연 시 전체 남은 run을 건너뛰는 runner를 사용했다. 자동 재시도 없음. 종료 확인용 GET 외에 실패 뒤 새 capture는 시작하지 않는다.
- fresh 촬영 하한 및 기본 캡처 소비 나이의 CAM/host 속도 `0.999~1.001`, timestamp 여유 0.002ms는 여전히 **물리 교정되지 않은 실험 가정**이다. 센서 노출 시각 대신 첫 DMA timestamp를 사용한다. 이번 통과율은 이 조건하의 나이 상한 판정이다.

## 실측 결과

| 지표 | fresh 캡처 | 기본 캡처 |
|---|---:|---:|
| 표본 수 | 24 | 24 |
| 측정 시 수락 | 22/24 | 9/24 |
| 소비 직전 재검사 통과 | **21/24** | **8/24** |
| 측정 수락 후 소비에서 추가 차단 | 1 | 1 |
| 소비 시 영상 나이 상한 중앙값 | 87.33ms | 107.08ms |
| 측정 완료→소비 검사 중앙값 | 1.02ms | 0.91ms |
| 소비 검사→전체 프레임 저장 장벽 중앙값 | 76.89ms | 75.27ms |
| 전체 프레임 저장 장벽 최대 | 88.51ms | 84.83ms |
| 저장 상태 `completed / late / unconfirmed` | 24 / 0 / 0 | 24 / 0 / 0 |
| 이미지·측정 저장 시에도 100ms 이내 | 0/24 | 0/24 |

소비는 저장 전에 수행되므로, 저장 시 영상이 100ms를 넘는 것과 소비 당시 신선도 통과는 양립한다. 마지막 행은 기존 persistence checkpoint(이미지·receipt·measurement 완료)이며, 전체 프레임 저장 장벽은 consumption JSON과 measured event까지 저장한 뒤다. 저장 기한은 영상 나이가 아닌 **소비 검사부터 기록 장벽 완료까지의 경과**다.

실제로 추가 차단된 원본:

| 캡처 | 원본 frame timestamp (동일 CAM boot) | 측정 나이 | 소비 나이 | 결과 |
|---|---:|---:|---:|---|
| 기본 | 3771091436 | 99.9831ms | 100.8387ms | 소비 0, 기록 완료 |
| fresh | 3778150951 | 99.9300ms | 100.9300ms | 소비 0, 기록 완료 |

두 표본 모두 `source_age_at_consumption_exceeds_policy`로 모의 대상 박스 사용을 거부했다. 계산 결과를 저장 없이 곧바로 썼다는 추정이 아니라 실제 소비 시각의 재검사가 작동한 사례다. 두 모드의 24개씩 작은 고정 장면 비교이므로 모든 환경의 안정적 통과율을 보장하지 않는다. 기본 캡처 run 간에도 7/12와 1/12 차이가 있었다.

| 순서 | 모드 | run ID | 측정 수락 | 소비 통과 | 상태 |
|---|---|---|---:|---:|---|
| 1 | fresh | `638de6b0e34a4c809bf52fd34ba9dbe9` | 12/12 | 12/12 | completed |
| 2 | 기본 | `9f852bc31df245e38c265c336b727fb5` | 7/12 | 7/12 | completed |
| 3 | 기본 | `eb2e2ae450b24e71aa425ed3a58b3313` | 2/12 | 1/12 | completed |
| 4 | fresh | `a47fa8801b034cffae428d0bc83aea7e` | 10/12 | 9/12 | completed |

## 증거·테스트·리뷰

- 원본 52 JPEG(각 run 기준 영상 1개 포함), artifact **482개**, event **104개**를 검증했다. terminal event가 가리키는 report, 파일 hash, frame/host/boot, fresh proof, 실행 소스 hash를 확인했다.
- 48개 원본·receipt·측정 결과를 실제 기록된 소비/저장 monotonic 시각으로 재생해 pending sidecar와 terminal의 최종 소비 기록이 정확히 일치함을 검사했다. 추가로 시간 차이를 직접 계산하고, 다음 capture가 직전 저장 장벽 이후 시작했는지 확인했다. 모든 관측은 `controlEligible=false`, `trainingEligible=false`다.
- 모터 출력 기록·마지막 명령(id 9, 기존 disarm) 불변. CAM boot `5bfe5283286bc8599cae44c1612b6869` 불변, capture failure/recovery 증가 없음. Pi 서비스 3개 active, NRestarts=0, throttled=0x0.
- 소비/저장 가상 시간 회귀 **22개 재통과**. 비교 runner 테스트 **4개 통과**: 정상 4회, failed/halted/비정상 종료 즉시 중단, terminal 출력 누락·timeout 시 재시도 없음. 모든 테스트는 장치 요청 없이 수행했다.
- production 코드가 바뀌지 않아 앞선 **전체 528개 통과 결과**를 재사용했다. 이번에 전체 528개를 다시 실행했다고 집계하지 않는다.
- 독립 리뷰에서 분석 스크립트의 descriptor 키(`path`→`artifactRef`)와 수락 measurement의 선택적 `reason` 접근 2건을 수정했다. 정상 실제 원본 전체 재생으로 두 경로를 회귀 검증했다. 최종 리뷰 결과는 검증 메타데이터에 기록한다.

**이번 실제 장치에서 저장 실패·250ms 초과는 발생하지 않았다.** 따라서 실물 지연 중단을 관측했다고 주장하지 않는다. 해당 중단 규칙은 [가상 시간 검증](2026-10-15-observation-consumption.md)과 runner 중단 테스트로 확인했다. blocking fsync 취소·프로세스 crash 시 소비와 저장의 원자성은 보장하지 않는다.

원본은 로컬 `datasets/fast_probes/<run ID>` 및 Pi 임시 경로의 `runs/<run ID>`에 보존했고 Git에 넣지 않았다. 공유 기록은 [집계와 표본별 시간](consumption-live-2026-10-15/live.json), [전후 상태](consumption-live-2026-10-15/comparison-before.json), [최종 상태](consumption-live-2026-10-15/final-status.json), [분석기](consumption-live-2026-10-15/analyze.py), [runner](consumption-live-2026-10-15/compare.py), [검증 메타데이터](consumption-live-2026-10-15/validation.json)다.

## 다음 단계

1. 배터리 분리·비구동 상태로 Pi의 **시험용 저장 경로에만** 의도적인 지연/실패를 주입해, 실제 영상 소비 후 `late/unconfirmed`, 다음 capture 0회, 원본 보존/terminal 기록을 확인한다. 운영 서비스나 실제 디스크를 손상시키는 방식은 사용하지 않는다. 100ms/250ms 기준은 유지한다.
2. CAM/host 시간 오차의 물리 근거와 실제 동작 전 안전 조건을 확인한 뒤 제한 동작 commissioning을 준비한다. 이번 shadow 성공만으로 실제 모터 출력이나 학습용 행동 데이터를 허용하지 않는다.
3. 제한 동작의 실제 출력·행동 전후 원본 수집을 검증하고 데이터셋/오프라인 학습·평가로 연결한다. 물체 집기·배치와 학습에 따른 성능 개선은 아직 남아 있다.

## 후속 저장 장애 주입 결과

[Pi 저장 장애 주입](2026-10-15-pi-storage-faults.md)에서 300ms 지연·이미지 실패·receipt 실패 세 run 모두 추가 capture 0회를 확인했다. receipt 실패는 95.61ms에 실제 shadow 소비한 뒤 `unconfirmed`로 중단됐고 원본을 보존했다. 나머지 두 case는 신선도 거부 후 저장 장애였다. 시계 오차 가정 점검과 제한 동작 commissioning은 남아 있다.
