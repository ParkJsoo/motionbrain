# Pi 실제 영상 저장 장애 주입 — 2026-10-15

**Pi의 시험용 저장 인스턴스에 지연·실패를 주입한 3개 run 모두 첫 capture 뒤 중단됐고, 각 run의 추가 capture는 0회였다.** 관측 소비 100ms와 저장 장벽 250ms 기준은 유지했다. receipt 실패 시험에서는 실제 영상이 95.61ms에 shadow 소비된 뒤 저장 미확정 상태로 중단됐으며, 이미지 worker가 종료될 때까지 기다려 원본을 보존했다. 모터 명령은 0회다.

## 시험 범위

- 실행 production 소스 `b70bf1c821d421658cdd8983a725b2d6b6555856`, 소비 구현은 `77fc791`과 동일. production 코드·기준·펌웨어 변경 없음.
- Pi `/tmp/motionbrain-storage-fault-20261015`의 별도 checkout에서 실제 카메라 fresh GET과 실제 파일 저장을 사용했다. `FaultStore`는 해당 시험 객체의 첫 이미지/receipt 쓰기에만 적용하며 anchor·terminal·운영 서비스 저장에는 영향을 주지 않는다.
- 이미지 쓰기 전 실제 sleep 300ms, 이미지 쓰기 전 `OSError`, capture receipt 쓰기 전 `OSError`를 각각 한 번 주입했다. 실제 디스크를 채우거나 파일 시스템·전원·장치를 손상시키는 방식은 사용하지 않았다. 이는 **실제 영상/실제 시간/실제 저장을 이용한 소프트웨어 장애 주입**이며 실제 저장장치 고장이나 전원 차단 시험이 아니다.
- 각 case는 새로운 저장소·transport로 3개 관측을 요청한 독립 시험이다. 실패한 run을 재개하거나 재시도하지 않았다. 정확한 예상 중단과 전후 상태 불변을 검증한 경우에만 다음 계획된 case를 시작했다. 예상 밖 결과면 전체 시험을 중단한다.
- 사용자 확인 배터리함 분리 상태를 유지했다. 소프트웨어로 물리 분리를 검증했다고 주장하지 않는다. 시험 전후 `IDLE`, `motorEnabled=false`, 모든 모터 출력 0을 별도로 확인했다.

## 실제 결과

| 주입 | 소비 검사 시 나이 | shadow 소비 | 저장 결과 | 원본 이미지 | capture receipt | 추가 capture |
|---|---:|---|---|---|---|---:|
| 이미지 저장 300ms 지연 | 110.46ms | 거부 | **381.30ms**, `late` | 보존 | 보존 | **0** |
| 이미지 저장 실패 | 109.00ms | 거부 | `unconfirmed` | **누락** | 보존 | **0** |
| receipt 저장 실패 | 95.61ms | **수행** | `unconfirmed` | **보존** | **누락** | **0** |

지연 sleep 실측은 300.17ms다. 381.30ms는 소비 검사부터 이미지·receipt·measurement·consumption·measured event의 저장 장벽 완료까지 전체 시간이며, sleep만의 시간이 아니다. `late`는 늦게나마 기록을 마쳤다는 의미이며 250ms에 쓰기를 강제 취소했다는 의미가 아니다.

앞의 두 관측은 소비 검사 시점의 나이가 100ms를 넘었고, 소비 기록의 거부 사유는 `measurement_rejected`다. 이미지 실패 case에는 전체 measurement 파일도 남지 않아 그 측정 단계의 세부 거부 원인은 확정하지 않는다. **이 두 case를 성공 소비 후 저장 실패한 실측으로 집계하지 않는다.** 소비 후 지연/이미지 실패 경로 자체는 기존 가상 시간 회귀 및 이번 harness fixture에서 검증했다. 실제 성공 소비 후 실패를 확인한 것은 세 번째 receipt case다.

모든 run은 예상대로 `status=failed`, `consumptionHalted=true`로 끝났다. 첫 case의 `completedSamples=1`은 늦게 기록을 마친 프레임 수이며 정상 run 성공을 뜻하지 않는다. 두 실패 case는 `completedSamples=0`이다. receipt 실패 case는 `consumedObservations=1`과 `completedSamples=0`을 함께 기록해 소비와 저장을 구분한다. 세 case 모두 요청했던 나머지 2개 촬영을 수행하지 않았다.

## 추가 촬영·원본·종료 기록 검증

- `AuditedTransport`가 probe에서 시도한 모든 GET을 저장 오류와 독립적인 메모리 원장에 기록했다. 각 run `/capture`는 1회뿐이며 주입 시작 이후에는 0회다. 세 run의 마지막 상태 확인은 GET `/actions`, `/action_outputs`, `/status`이며, 새 capture를 보내지 않는다.
- 이 **0회는 해당 probe run의 요청 범위**다. 기존 Pi perception 서비스는 변경 없이 별도로 계속 촬영한다. 카메라 전체 capture counter가 멈췄다는 주장이나 장치 전체 네트워크 packet audit가 아니다.
- 종료 후 0.5초 대기 시 `capture-image` worker가 남아 있지 않음을 확인했다. receipt 실패에서 worker가 기록한 원본이 report artifact 목록에 포함됐으며 readback SHA-256이 HTTP receipt의 원본 hash와 일치했다.
- 세 report의 terminal event, 연속 event, **artifact 52개·event 10개·보존 JPEG 5장**(anchor 3장 포함)을 검증했다. 이미지 실패 case의 현재 JPEG 1장은 의도대로 누락됐고, receipt 실패 case의 core receipt 1개도 누락 상태를 유지했다. 누락을 채우거나 저장 완료로 바꾸지 않았다.
- capture receipt 실패 시 별도 시험 원장에 HTTP metadata가 남는다. 이 원장은 검증용 관찰 기록이며 core receipt를 복구한 것으로 취급하지 않는다. 따라서 `unconfirmed`는 그대로다.
- 오프라인 분석에서 실행 production/실험 소스 hash, fresh proof, frame/boot/host/hash와 소비 시 나이 상한을 원장으로 재계산했다. 실제 이미지가 누락된 case의 원본 bytes를 검증했다고 주장하지 않는다.
- 컨트롤러 출력 로그·기존 마지막 disarm 명령·CAM boot 불변. capture failure/recovery 증가 없음. Pi 서비스 3개 active, NRestarts=0, throttled=0x0. 운영 checkout `559db54` 유지. 서비스 재시작·펌웨어 업로드·모터 구동 없음.

## 테스트와 코드리뷰

- 실제 Pi 실행에 사용한 harness의 **오프라인 테스트 7개 통과**: 세 주입 경로, 다른 store/anchor 비영향, 추가 capture/누락 주입 검출, 원본 손상 검출, 예상 밖 결과 시 다음 case 차단.
- 기존 소비/저장 가상 시간 회귀 **22개 재통과**. 이번 재실행 합계 **29개**다.
- production 변경이 없어 이전 **전체 528개 통과 결과를 재사용**했다. 이번에 528개를 다시 실행했다고 집계하지 않는다.
- 사전 독립 리뷰에서 차단할 P0/P1/P2 없음. 최종 실측 증거 재리뷰 결과는 [검증 메타데이터](storage-fault-live-2026-10-15/validation.json)에 기록한다.

[실측 집계](storage-fault-live-2026-10-15/live.json), [GET 요청 원장](storage-fault-live-2026-10-15/request-ledgers.json), [주입 스크립트](storage-fault-live-2026-10-15/fault_probe.py), [오프라인 분석기](storage-fault-live-2026-10-15/analyze.py), [최종 장치 상태](storage-fault-live-2026-10-15/final-status.json).

원본·전체 HTTP 원장은 로컬 `datasets/storage_fault_probes/2026-10-15`와 Pi 임시 checkout에 보존했다. Git에는 원본 JPEG·전체 private raw 데이터를 넣지 않았다. 이 데이터는 의도적인 장애 실험이며 `trainingEligible=false`, `controlEligible=false`다.

## 남은 한계와 다음 작업

저장 실패 시 중단과 정상 shadow 소비 경로의 비구동 검증은 완료했다. 다만 CAM/host 속도 0.999~1.001 및 timestamp 여유 0.002ms는 아직 미교정 실험 가정이다. 영구 디스크 장애로 terminal 자체를 쓰지 못하는 경우, 프로세스 강제 종료, 전원 차단, crash-atomic 소비/기록은 이번 실측 범위에 포함하지 않는다.

**다음은 기존 시간 교정 probe를 확장해 CAM–Pi 시계 오차 가정을 비구동 실측으로 점검하는 단계**다. 첫 DMA timestamp와 광학 노출 시각 차이, 실측 구간 밖 drift 한계는 별도로 남긴다. 그 결과와 기존 M4 영점·안전 범위를 확인한 뒤 제한 동작 commissioning을 준비한다. 실제 모터 구동·배터리 재연결은 별도 지시와 준비 상태 확인 후 진행한다. 이후 실제 행동 전후 데이터 수집·학습·조작 성능 개선 검증이 남아 있다.
