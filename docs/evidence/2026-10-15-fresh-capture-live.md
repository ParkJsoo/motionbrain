# CAM 요청 이후 프레임 — 업로드·비구동 비교 (2026-10-15)

후속: [촬영 전 대기 분리 계산·원본 재생·새 실측](2026-10-15-fresh-age-bound.md)을 완료했다. 아래 결과는 이전 상한식으로 진행한 비교 기록이다.

**CAM 업로드와 48장 비교를 완료했다. 새 캡처의 요청 이후 프레임 계약은 24/24 유효했지만, 현재 계산 방식에서 100ms 통과율은 개선되지 않았다.** 기준은 100ms 그대로이며 모터 명령은 0회다. 자동 제어·학습 데이터로 승인한 결과가 아니다.

## 업로드와 연결 상태

- 사용자 확인: 배터리함→XL4015 분리, USB/Pi 유지. 메인 ESP32는 전후 IDLE, motorEnabled=false, M1~M5 출력 0. 마지막 명령은 기존 web disarm ID 9 그대로다.
- CAM `/dev/cu.usbserial-2120`, MAC `30:76:f5:e5:ea:60` 재확인. 전체 4MB flash를 로컬 `/tmp/motionbrain-fresh-live-20261015/cam-before.bin`에 백업했다. 최초 460800 baud 읽기는 직렬 패킷 오류로 실패했고, 115200 재시도는 377.2초에 성공했다. 백업은 설정을 포함할 수 있어 Git에 넣지 않았다. 복구가 필요하면 CAM 포트/MAC을 재확인한 뒤 이 이미지를 flash 0번지에 쓸 수 있다. 이번에는 복구 쓰기를 하지 않았다.
- `b811074`의 검증된 CAM binary SHA-256 `e124ba563cfbca1baafc2d6cce0e34de65784d49cee7489f3d57cf47e5f37e33` 업로드 및 esptool hash 검증 성공(61.98초). 메인 ESP32·STM32 firmware, Pi 운영 소스·서비스는 변경하지 않았다.
- 새 CAM boot `5bfe5283286bc8599cae44c1612b6869`가 비교 종료까지 유지됐다. QVGA·quality 15 유지, 비교 중 captureFailures/recoveries/freshRejected 모두 0. `/`, `/capture`, `/capture?t=commissioning` HTTP 200, 기본 JPEG 원본 metadata 유효. Pi perception은 업로드 후와 종료 시 ok/fresh, 연속 오류 0. 세 서비스 active/NRestarts=0, throttled=0x0. 이는 API 회귀 확인이며 브라우저 육안 검증이나 장시간 안정성 인증은 아니다.

## 같은 조건의 순서 교차 비교

Pi 운영 서비스를 유지하고 `/tmp/motionbrain-fresh-live-20261015`에 복사한 `b811074` 소스를 사용했다. 정지 컵, 각 run 새 semantic anchor, 3개 ROI worker, 병렬 영속 저장, capture/connection timing을 동일하게 적용했다. 순서는 기본→fresh→fresh→기본, 각 12장이다. 서로 다른 실제 프레임이므로 동일 영상 정확도 비교로 해석하지 않는다.

| 항목 | 기본 24장 | fresh 24장 |
|---|---:|---:|
| 측정 직후 100ms 및 ROI 정책 수락 | 3/24 | 0/24 |
| 저장 직후 100ms 수락 | 0/24 | 0/24 (1장 나이 산출 불가) |
| 카메라 보고 나이 중앙값 | 59.78ms | 36.87ms |
| HTTP 전체 시간 중앙값 | 43.78ms | 103.67ms |
| 측정 직후 나이 상한 중앙값 | 121.31ms | 150.80ms |
| 저장 직후 나이 상한 중앙값 | 157.34ms (24장) | 189.60ms (23장) |
| 서버 획득/선택 시간 중앙값 | 1ms (24장 연결 유효) | 56ms (22장 연결 유효) |

기본은 21장 나이 초과. fresh는 23장 나이 초과, 1장 `anchor_expired`다. 만료된 1장은 ROI 계산을 조기 종료해 저장 시점 나이 상한이 없으므로 23장 통계와 구분했다. 정책을 완화하거나 해당 장면을 재시도하여 표본에서 제거하지 않았다. fresh의 서버 시간 2장은 다른 클라이언트 캡처가 끼어 상태 연결을 거부했으며 통계에서 제외했다.

fresh 24장 모두 같은 boot에서 `capturedUs > requestedUs`, 원본 ID/hash/응답 헤더 증거가 일치했다. 이전 후보 반환 수는 1개 1회·2개 23회(총 47개), fresh 오류·fallback 없음. 이 결과는 SDK first-DMA 시각 기준이며 광학 노출 시각 교정은 아니다.

| 순서 | run ID | 모드 | 측정 수락 |
|---|---|---|---:|
| 1 | `1bdd4cb50cf5474ab1a123e822a4fe63` | 기본 | 2/12 |
| 2 | `675bb1c66e724f638aa6657133b691b4` | fresh | 0/12 |
| 3 | `cc0f7996d2f8476cbbe1c6bf70eb76e5` | fresh | 0/12 |
| 4 | `8db28e2255d543108735b768eee9afb2` | 기본 | 1/12 |

각 run completed, 전 구간 출력 로그 동일, motorCommandsSent=0. 52 JPEG(48 비교+4 anchor), 434 artifact, 104 event의 hash·종료 report·소스 hash·fresh 증거를 로컬에서 재검증했다. 원본은 Git 제외 `datasets/fast_probes/<run ID>` 및 Pi 임시 runs에 보존했다. 검증 스크립트는 4개 run 전체 순서/개수와 전후 상태도 검사한다.

## 해석·다음 작업

새 캡처는 요청 뒤 생성된 원본을 받는 데 성공했지만 다음 프레임을 기다리는 시간이 늘었다. 기존 계산 `카메라 보고 나이 + GET 전송 시작 이후 경과`는 이 대기까지 포함한다. 따라서 이번 숫자만으로 실제 영상이 더 오래됐다고 단정하거나, 새 기능이 100ms를 해결했다고 주장할 수 없다. 저장 완료 후에도 통과 0이므로 모터 검증 단계로 넘어갈 근거는 아직 없다.

다음은 **requestedUs/capturedUs의 같은 CAM clock 증거와 실제 GET 전송 시작 시각으로 촬영 전 대기를 분리한 보수적 나이 상한을 유도·회귀 테스트**하는 작업이다. 100ms는 유지하고 잘못된 timestamp·boot·시계 오차에서는 거부해야 한다. 먼저 이번 원본으로 같은 프레임의 구/신 상한을 비교하고, 검토 후 비구동 실험을 반복한다. 15초 anchor 만료도 run 길이·갱신 조건과 함께 다루되 값을 무조건 늘리지 않는다. 실제 소비 시점 재검사·clock/M4 교정·실제 제한 동작 및 학습 효과 검증은 별도 미완료다.

생산 코드 변경은 없으며 기존 전체 **496개 소프트웨어 테스트·CAM 빌드 통과** 결과를 재사용했다. 이번 실물 검증은 업로드·GET 계약·시간·저장 무결성·출력 불변 범위다. 무전원 상태에서 실제 모터 기구 동작을 검증한 것으로 취급하지 않는다.

독립 리뷰에서 분석기를 재실행해 저장된 분석 JSON과 완전 일치함을 확인했고, 4개 run·원본/종료 증거·업로드/백업 hash·통계와 문서의 한계를 검토했다. P0/P1/P2 발견 없음.

[프레임별 분석](fresh-capture-live-2026-10-15/analysis.json), [분석 코드](fresh-capture-live-2026-10-15/analyze.py), [실험 코드](fresh-capture-live-2026-10-15/compare.py), [업로드 로그](fresh-capture-live-2026-10-15/upload.log), [장치·binary 검증](fresh-capture-live-2026-10-15/validation.json), [업로드 전 상태](fresh-capture-live-2026-10-15/before.json), [최종 상태](fresh-capture-live-2026-10-15/final.json), [기본 URL 검증](fresh-capture-live-2026-10-15/basic-checks.json).
