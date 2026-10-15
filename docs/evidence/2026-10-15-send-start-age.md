# GET 전송 시작을 이용한 영상 나이 상한 — 2026-10-15

**100ms 정책을 유지하면서, 실제 GET의 첫 전송 직전 시각으로 보수적 상한을 좁혔다.** 같은 실제 프레임 12개에서 기존 상한 대비 5.91~16.58ms(개별 감소량 중앙값 10.56ms)를 제외할 수 있었다. 측정 완료 시 100ms 이내인 결과는 기존식 3/12에서 새 식 6/12로 바뀌었고 나머지는 계속 거부했다. 카메라가 더 빨라졌거나 실제 영상 나이가 줄었다는 결과는 아니다.

전체 소프트웨어 **471개 테스트 통과(18.765초)**. 실제 원본·측정 저장 후에는 **12개 모두 100ms 초과**다. 모터 명령 0회, `controlEligible=false`, `trainingEligible=false`, 물리 시간 교정 미완료를 유지한다.

## 근거와 적용 범위

동일 Pi monotonic clock에서 요청 시작을 `R`, 첫 socket send 호출 직전을 `S`, 서버가 해당 GET에 응답하면서 source age를 샘플하는 사건을 `E`, 응답 헤더 파싱 완료를 `H`, 측정/소비 시각을 `T`라 한다. 직접 연결·GET·프록시/redirect/HTTP 재시도 없는 현재 경로와 현 펌웨어에서는 다음 순서를 보장한다.

```text
R <= S <= E <= H <= T
기존 상한 = reportedSourceAgeMs + (T - R) * 1000
새 상한   = reportedSourceAgeMs + (T - S) * 1000
```

`S`는 wire 송출 완료나 촬영 시각이 아니다. 첫 `HTTPConnection.send`에서 연결 후 실제 `socket.sendall` 이전에 찍는 로컬 시각이다. 서버가 이미 처리한 뒤 클라이언트의 send 호출이 반환될 수도 있으므로 **전송 완료 시각은 사용하지 않는다**. RTT 절반이나 왕복 대칭도 가정하지 않는다. 요청 준비·주소 해석·TCP 연결은 첫 요청 bytes보다 앞이어서 제외할 수 있지만, 요청 전송·서버 대기·헤더/본문 수신·hash·ROI 계산과 후속 경과시간은 모두 포함한다.

기존과 같이 요청 내 카메라/Pi clock rate가 같다는 가정이 남는다. 광학 촬영 시각, 노출/readout, buffer residence, 장기 drift, M4와의 시간 정합성을 검증한 것은 아니다. 이 상한은 실험용 관측 판정에만 적용하며 물리 제어의 신선도 보증으로 승격하지 않는다.

## 구현과 실패 처리

- `learning/http_timing.py`: 기존 선택형 계측에 첫 send의 의미 버전 tag를 기록한다. process-wide patch, 추가 socket/요청 또는 retry는 없다.
- `learning/source_age.py`: receipt의 GET `/capture` 성공·오류 없음, 동일 host clock, request/receive 시각, 원본 hash·boot/frame/capture 시각·reported age·timestamp 의미를 측정 입력과 대조한다. trace 전체의 유한성·순서·addressAttempts를 확인한다. 잘못되거나 섞인 기록은 거부한다. trace 또는 알려진 의미 tag가 없으면 기존 전체 요청 상한으로 복귀한다. tag는 로컬 생산자의 기록 형식이며 인증 증명은 아니다.
- `learning/fast_observation.py`: 선택된 기준을 `sourceAgeBoundBasis`/`sourceAgeBoundStartedMonotonic`에 명시한다. `requestStartedMonotonic`은 원래 의미를 보존한다. `reportedAgePlusElapsedMs`는 선택된 상한, `fullRequestAgePlusElapsedMs`는 기존식 비교값이다. 최종화까지 시간에 포함하고, 마지막 시계가 앞선 검사보다 역행하거나 NaN이면 수락을 취소하고 좌표를 제거한다.
- `learning/fast_probe.py`: exact GET receipt를 연결하고 저장 완료 나이에도 같은 기준을 적용한다. 원본/receipt/측정 기록을 모두 유지한다. 저장 시점이 잘못됐거나 사용할 기준이 없으면 저장 후 freshness는 통과하지 않는다. 계산 시점의 수락을 나중 소비 시점의 승인으로 간주하지 않는다.
- `learning/capture_timing.py`: 기존 전체 요청 기반 age/offset interval은 보존하고, `causalAgeAtReceiptUpperMs`와 `causalBoundReductionMs`를 추가한다. 과거 계측 파일이나 timestamp를 고치지 않는다.

`--connection-timing`이 없는 기본 probe와 기존 직접 측정 호출은 원래 전체 요청 상한을 유지한다. 펌웨어·Pi 운영 서비스·운영 설정 변경은 없다.

## 소프트웨어 검증

새 테스트 10개와 기존 회귀를 포함해 [전체 471개](send-start-age-2026-10-15/tests.log)를 통과했다. 가상 시간/socket 시험은 서버 age 사건이 가능한 구간 어디에 있어도 새 상한이 실제 나이보다 작아지지 않음을 검사한다. 서버 처리 후 send 완료가 관측되는 반례, 미지원 의미 fallback, trace 순서/NaN/누락, 다른 host/frame/hash/age/실패 응답, 계산·최종화·저장 지연, 최종 시계 역행도 포함했다. 새 상한이 100ms를 넘는 probe에서 수락 0·모터 명령 0·출력 로그 불변을 확인했다. 실제 출력 구동 시험은 아니다.

독립 리뷰 에이전트가 관련 54개 테스트를 별도 실행했다. 최종 코드·실측 통계·원본/종료/실행 소스 hash·문서까지 재검토하여 P0/P1/P2 문제 없음을 확인했다.

## 같은 프레임의 실제 GET 비교

배터리함 분리, USB/Pi 유지, 컵 정지 상태에서 Pi `/tmp/motionbrain-send-age-20261015`의 별도 코드로 실행했다. 운영 코드 `559db54`와 서비스는 그대로다.

```sh
/home/motionbrain/.cache/motionbrain/opencv-venv/bin/python \
  /tmp/motionbrain-send-age-20261015/tools/probe_fast_observation.py \
  --controller-url http://192.168.219.113 --camera-url http://192.168.219.116 \
  --samples 12 --capture-timing --connection-timing \
  --output-dir /tmp/motionbrain-send-age-20261015/runs
```

실행 ID `95c43314228b470c9770fc1106ac348b`. 별도 시간에 촬영한 이전 실험과 통과율을 비교하지 않고 **이번 같은 GET·원본·동일 계산 완료 시각**에 두 식을 적용했다.

| 항목 | 중앙값 | 최소~최대 |
|---|---:|---:|
| 기존 전체 요청 기반 측정 완료 상한 | 111.87ms | 98.09~139.42ms |
| 첫 GET 전송 시작 기반 측정 완료 상한 | 100.03ms | 82.86~129.83ms |
| 프레임별 상한 감소량 | 10.56ms | 5.91~16.58ms |
| ROI 계산 시간 | 23.80ms | 18.57~28.98ms |
| 새 식의 원본·측정 저장 후 상한 | 142.89ms | 126.87~171.09ms |

중앙값끼리의 차이는 개별 차이의 중앙값과 다를 수 있다. anchor age는 7.60~13.43초로 전부 15초 이내다. 계산 완료 수락 6/12은 이 정지 장면의 실험적 ROI 판정이며, 움직이는 팔의 제어 성공률이나 파지 성능이 아니다. 저장 이후에는 0/12이므로 이 결과를 바로 물리 행동에 사용하지 않는다.

원본 JPEG 13개·artifact 108개·event 26개의 hash/종료 report와 현재 실행 소스 hash를 검증했다. Git 제외 원본은 `datasets/fast_probes/95c43314228b470c9770fc1106ac348b`에 보존했다. 시작/종료 출력 로그 동일, IDLE·모터 출력 0·motorEnabled=false, CAM boot 유지·captureFailures=0, Pi 서비스 3개 active·NRestarts=0을 확인했다.

[결과](send-start-age-2026-10-15/live-results.json), [저장 시점 기록](send-start-age-2026-10-15/persistence.json), [요청별 시간 기록](send-start-age-2026-10-15/timing.json), [환경·코드 hash](send-start-age-2026-10-15/live-session.json), [집계·무결성 검증](send-start-age-2026-10-15/validation.json), [시작 상태](send-start-age-2026-10-15/before-status.json), [종료 상태](send-start-age-2026-10-15/final-status.json).

## 해석과 후속 검증

계산 완료 수락은 소비 시점 수락이 아니다. [저장 비용 비교](2026-10-15-capture-persistence.md)와 [소비 시점 재검사 계약](2026-10-15-observation-consumption.md)에서 이를 별도로 다뤘다. 실제 행동 전후 수집과 물리 조작 성능은 미검증이다.
