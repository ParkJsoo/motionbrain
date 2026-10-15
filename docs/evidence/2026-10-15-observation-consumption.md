# 관측 소비 재검사와 저장 완료 분리 — 2026-10-15

**관측 소비 직전 100ms 재검사와 이후 저장 완료를 분리하는 선택형 shadow 경로를 구현했다.** 외부 callback·모터 명령 인터페이스 없이 메모리 안에서 대상 박스를 선택하는 모의 소비만 한다. 이번 작업의 장치 요청·펌웨어 업로드·서비스 재시작·모터 명령은 모두 0회다.

## 흐름과 상태

`tools/probe_fast_observation.py --shadow-consumption`으로 켠다. 기존 기본 probe 동작은 유지한다. `--max-persistence-ms`는 이 모드의 별도 저장 기한이며 기본 250ms, 허용 범위는 유한한 `(0, 30000]`이다. **관측 신선도 100ms를 250ms로 완화하는 옵션이 아니다.**

1. 기존 ROI 측정을 마친 뒤 원본 bytes hash, measurement/frame/hash/host, HTTP receipt와 source metadata를 재검증하고 나이 상한을 다시 구성한다. measurement에 저장된 촬영 하한을 무조건 신뢰하지 않는다.
2. 검증에 걸린 시간까지 포함해 현재 monotonic 시각을 읽는다. 측정 수락 여부·관측 나이 100ms·기준 영상 유효기간을 검사하고, 통과할 때만 모의 대상 박스를 소비 기록에 남긴다. 거부 시 박스는 남기지 않는다. `controlEligible=false`, `trainingEligible=false`를 유지한다.
3. 현재 프레임은 `pending`이다. 이미지/receipt 쓰기, 측정 JSON, 기존 persistence JSON, consumption JSON, measured event의 fsync가 끝나기 전에는 다음 촬영·소비를 진행하지 않는다. 여러 프레임 queue나 백그라운드 선행 촬영을 추가하지 않았다.
4. 같은 frame/host의 완료 확인을 한 번만 처리한다. 소비 검사 시각부터 마지막 measured event 저장 완료까지의 경과가 저장 기한 이내면 `completed`, 초과면 `late`이며 run을 실패로 끝낸다. late는 쓰기가 끝났지만 기한을 넘긴 상태다. 기한 초과·저장 실패·중단·시계 오류 이후 재개/재시도/다음 GET은 없다.

소비 기록의 `consumedMonotonic`은 소비 여부를 판단한 검사 시각이다. 실제 모의 소비 여부는 `consumed`로 구분한다. 늦은 관측은 `consumed=false`지만 해당 원본의 기록을 정상 완료하면 다음 새 관측은 검사할 수 있다. 신선한 관측을 소비한 뒤 원본이 오래된 상태로 저장됐다고 과거 소비를 취소하거나 저장 시점에도 신선했다고 바꾸지 않는다.

소비 단계에서는 CAM 속도가 host의 `0.999~1.001` 범위라는 **미교정 실험 가정**을 사용한다. fresh 촬영 하한은 기존 식을 유지하며, 기본/fallback 캡처의 보고 나이는 `reportedAge/0.999+0.002ms`로 보수적으로 변환한 뒤 host 경과를 더한다. 기본 probe의 측정 상한은 바꾸지 않는다. 소비 시점 기준 영상 나이는 `frame−anchor CAM 시각 차이/0.999 + 현재 프레임 나이 상한 + 0.002ms`로 검사해 촬영→측정→소비 전체 시간을 포함한다. 적용 가정·보정값·미교정 여부를 소비 기록에 남긴다.

저장 실패/KeyboardInterrupt 시 이미 시작한 이미지 worker를 join하고 남길 수 있는 원본을 보존한다. 전체 프레임 기록 장벽을 확인하지 못했으면 `unconfirmed`다. 복구 가능한 저장 오류는 terminal report에 이미 수행된 모의 소비와 미확정 저장 상태를 별도로 남긴다. **소비 이후 저장 실패 시 `consumedObservations=1`이고 기존 완료 measurement 집계인 `acceptedMeasurements=0`일 수 있다.** 이는 소비가 실제 모터 동작을 의미해서가 아니라 소비와 저장 완료를 분리했기 때문이다.

파일별 `consumption/*.json`은 저장 중의 pending 스냅샷이다. 종료 report와 그 terminal event·artifact 무결성을 함께 확인해야 최종 완료 여부를 판단할 수 있다. 종료 report를 쓸 수 없는 영구 디스크 장애나 프로세스 종료에서는 **소비 여부와 전체 저장 완료 모두 추정하지 않는다**. 소비/기록의 crash-atomic 보장이나 exactly-once 물리 실행 보장을 추가한 것이 아니다.

저장 기한은 blocking fsync를 강제 취소하는 hard timeout이 아니다. 쓰기가 반환된 뒤 늦음을 판정하고 중단하며, 무기한 block 중에는 다음 관측이 진행되지 않는다. 저장 기한 범위는 현재 프레임의 core evidence 완료까지이며 이후 선택형 HTTP 진단·최종 report 저장 시간은 별도다. 이 단계에서 실패해도 run은 중단한다.

## 가상 시간 검증

전체 **528개 소프트웨어 테스트 통과(20.057초)**.

신규 22개 테스트는 소비 전 스케줄링 지연, 정확한 frame/host 완료, 중복 소비/완료, forged bound·원본/receipt 불일치, NaN/역행 시계, 측정 거부 유지, 저장 단계별 실패·지연·중단·영구 장애를 검사한다.

독립 리뷰에서 기준 영상 만료 검사에 촬영→측정 시간이 빠지는 P2를 발견해 위 식으로 수정했다. 생산 ROI tracker로 `frame−anchor=14990ms`인 관측이 측정 때 수락돼도 소비 시점 15초 초과로 거부됨을 재현했다. 속도 여유가 없으면 15초 이내지만 여유를 포함하면 초과하는 경계와 기본 캡처 100ms 경계도 회귀 검증한다.

fresh 상한에 clock-rate/timestamp 여유를 포함하면 소비 시점에 100ms를 넘는 사례에서 측정은 수락됐어도 **소비 0·모터 명령 0**임을 확인했다. 이러한 오차 여유는 [앞선 계산 문서](2026-10-15-fresh-age-bound.md)의 실험 가정이며 물리 교정값이 아니다.

| 가상 시나리오 | 소비 결과 | 저장/중단 결과 |
|---|---|---|
| 정상 | 2/2, 소비 나이 약 21ms | 저장 완료 후 다음 촬영 |
| 측정 후 소비까지 100ms 추가 | 0/2, 소비 나이 약 121ms | 거부 원본 저장 후 다음 새 관측 |
| 이미지 저장 120ms | 2/2, 소비 나이 약 21ms | 저장 때는 낡아도 250ms 기록 기한 이내 완료 |
| 이미지 저장 251ms | 첫 1개 모의 소비 | `late`, 두 번째 촬영 0회, 원본 보존 |
| 이미지 저장 실패 | 첫 1개 모의 소비 | `unconfirmed`, 두 번째 촬영 0회 |
| receipt 저장 중 interrupt | 첫 1개 모의 소비 | worker join·원본 보존, `unconfirmed`, 두 번째 촬영 0회 |

위 대표 시나리오는 실제 production probe에 합성 JPEG/FakeDevice/VirtualClock을 연결해 생성했다. 각 저장 성공 artifact의 hash와 terminal을 검사했다. 테스트는 receipt·이미지·measurement·consumption sidecar·measured event 각각의 실패와 sidecar/event 단계 지연도 확인한다. 영구 디스크 장애는 terminal이 없고 다음 촬영도 없는 상태로 별도 검증한다. 합성 fixture를 실제 학습 데이터나 실물 성공률로 사용하지 않는다.

최종 독립 리뷰에서 신규 22개 테스트와 대표 시나리오 6개를 재실행했고 저장 증거·검증 소스 hash·전체 테스트 로그를 확인했다. 발견했던 P2 수정 후 미해결 P0/P1/P2 없음.

## 해석과 후속 검증

이 단계는 가상 시나리오 검증이다. [후속 비구동 실측](2026-10-15-shadow-consumption-live.md)에서 소비 나이와 저장 장벽을 측정했다. 실제 모터 실행·행동 학습 적격성은 검증하지 않았다.

[테스트 로그](consumption-2026-10-15/tests.log), [가상 시나리오 코드](consumption-2026-10-15/virtual_scenarios.py), [시나리오 결과](consumption-2026-10-15/virtual-scenarios.json), [검증 소스 hash](consumption-2026-10-15/validation.json).

## 후속 실물 비구동 확인

[Pi 실제 영상 shadow 소비 비교](2026-10-15-shadow-consumption-live.md)에서 48개 관측을 검사했다. fresh 소비 21/24, 기본 8/24이며 측정 통과 후 100ms 초과한 2개를 소비 시 차단했다. 저장 장벽은 모두 250ms 이내다. 이 정상 비교 당시에는 실제 저장 실패·지연 중단이 발생하지 않았다. 이후 [Pi 시험 경로 장애 주입](2026-10-15-pi-storage-faults.md)으로 세 중단 사례를 확인했으며, 물리 시계 교정은 남아 있다.
