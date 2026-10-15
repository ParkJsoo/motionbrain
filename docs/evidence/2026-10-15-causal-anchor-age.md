# Anchor 만료의 CAM 시계 가정 제거 — 2026-10-15

**선택형 `--causal-anchor` 경로에서 현재 fresh 영상과 기준 이미지(anchor)의 나이를 모두 Pi 단조 시각의 경과 시간으로 제한한다.** CAM 속도·offset·2µs 여유를 만료 판정에 사용하지 않는다. 전체 **553개 테스트**, 별도 시험기 3개 테스트와 독립 리뷰를 통과했다. Pi 정상 표본 12개 중 1개가 100ms 소비 기준을 통과했고, 별도 만료 시험은 **영상 77.88ms·anchor 상한 15.30초에서 소비 0회, 저장 후 추가 촬영 0회**였다. 모터 명령은 전 과정 0회다.

## 인과 근거와 변경 범위

1. 기존 `?fresh=1` GET으로 checkpoint 원본과 exact receipt를 한 번 저장한다. 해당 first-DMA 촬영은 첫 GET 송신 이후다.
2. 기존 검출 서비스의 메타데이터를 **최대 8회, 오래된 유효 프레임이면 1초 간격**으로 조회한다. 같은 CAM boot에서 checkpoint보다 뒤에 촬영된 frame ID가 나타나면 기존 adapter로 검출 원본 JPEG·관측을 정확히 가져온다. 대기 중 추가 camera capture는 요청하지 않는다. HTTP/identity 오류는 즉시 중단하며 횟수 소진도 중단한다.
3. 최종 원본도 `anchor.capturedUs > checkpoint.capturedUs`, 같은 boot·frame/hash·live detection을 재검사한다. 새 영상에 과거 bounding box를 복사하거나 수신 시각을 촬영 시각으로 바꾸지 않는다. 최종 exact 조회가 다른 원본을 반환해도 순서·동일성을 다시 검사한다.
4. `anchor 상한(ms) = (현재 Pi monotonic − checkpoint 첫 GET 송신 monotonic) × 1000`. CAM에서는 촬영의 선후만 판단하며 시간 길이를 변환하지 않는다. 새 검출 대기·원본 조회·저장·ROI 계산 시간도 모두 포함돼 실제 anchor 나이보다 보수적이다.

`learning/anchor_age.py`가 증거 검증과 상한 계산을 맡는다. `fast_observation.py`는 anchor 생성·측정 시작/종료에서 15초 만료를 검사하고, `observation_consumer.py`는 고정한 원본 identity와 checkpoint 근거로 소비 시점에 다시 계산한다. measurement에 적힌 하한이나 CAM 차분을 조작해도 소비 gate를 바꿀 수 없다. anchor 만료 시 현재 프레임을 저장한 뒤 consumer가 중단되어 다음 capture를 시작하지 않는다. consumer 없는 진단 모드는 거부 결과를 계속 기록할 수 있으므로 실측은 `--shadow-consumption`을 명시했다.

CLI 조합은 `--fresh-capture --capture-timing --connection-timing --fresh-age-mode send_start --causal-anchor --shadow-consumption --max-persistence-ms 250`이다. 현재 영상 **100ms**, anchor **15000ms**, 현재 프레임 소비 후 저장 barrier **250ms**는 그대로다. 250ms는 anchor를 기다리는 전체 시간 제한이 아니며 저장 syscall을 강제로 취소하는 deadline도 아니다. anchor 대기에는 조회 횟수·HTTP timeout·최종 15초 만료 검사가 적용된다.

기존 비교 경로/기본값은 호환성을 위해 남겼다. `--causal-anchor`를 켠 경로의 만료 계산에서 가정을 제거했으며, 옛 경로까지 물리적으로 교정했다는 뜻이 아니다. 원본 관측에 있던 producer의 시계 추정 필드와 `anchorAgeMs` CAM 차분은 진단 자료로 보존하지만 새 gate에 쓰지 않는다. 같은 boot의 단조 timestamp·first-DMA 의미는 여전히 전제이며 **광학 노출 시각 교정이나 실제 제어 안전성 검증 완료는 아니다**.

## 첫 실측 거부와 보완

초기 구현은 checkpoint 이후 5초 고정 대기 뒤 exact anchor를 한 번 조회했다. 실제 서비스가 checkpoint `8958799980µs`보다 오래된 원본 `8958331653µs`를 반환하여 **`anchor_not_after_checkpoint`로 거부**했다. checkpoint 촬영 1회·현재 관측 촬영 0회·소비 0회였고 예정된 만료 시험도 실행하지 않았다. 이는 잘못된 원본을 수락한 문제가 아니라 고정 대기로 새 검출 도착을 보장할 수 없다는 실측 결과다.

이를 위의 유한 메타데이터 조회로 보완했다. 실패 run을 재시도 루프에서 숨기지 않았으며 별도 v2 디렉터리에서 수정 코드를 검증했다. 첫 시도의 원본 2개·artifact 17개·event 2개와 당시 소스 archive도 보존·검증했다. [거부 결과](causal-anchor-2026-10-15/rejected.json) 참조.

## 최종 Pi 비구동 결과

Pi `/tmp/motionbrain-causal-anchor-v2-20261015`에서 정상 12개 표본 run 이후 별도 의도적 만료 run을 수행했다. 두 run 모두 메타데이터 6회 조회 후 anchor를 확보했다. 정상 run이 실패하면 만료 run도 시작하지 않는 시험기다.

| 항목 | 정상 run | 의도적 만료 run |
|---|---:|---:|
| 요청한 현재 표본 / 실제 촬영 | 12 / 12 | 3 / **1** |
| 별도 checkpoint 촬영 | 1 | 1 |
| anchor 확보 시 나이 상한 | 5.37초 | 5.36초 |
| 계산 통과 / 소비 통과 | 1 / **1** | 0 / **0** |
| 현재 영상 소비 나이 최소·중앙값·최대 | 99.63 / 114.44 / 129.66ms | **77.88ms** |
| anchor 소비 나이 상한 범위 | 5.66~12.87초 | **15.30초** |
| 저장 barrier 중앙값 / 최대 | 74.98 / **237.66ms** | **78.80ms** |
| 결과 | 12개 처리 완료, 11개 신선도 거부 | anchor 만료 거부·기록 후 halt |

만료 run은 tracker 생성 후 첫 현재 촬영 전에 checkpoint 첫 송신+15.05초까지 약 9.63초 대기하도록 시험기에만 주입했다. 새 영상 자체는 100ms 안이었으므로 anchor 만료에 의한 거부를 분리해 확인했다. `run.json`의 상태는 의도적으로 **failed**이고 소비 행의 구체적 이유는 `anchor_expired_at_consumption`이다. 공통 exception 문구에 `persistence_deadline_or_consumption_evidence`가 포함되지만 이번 저장은 78.80ms로 완료됐으며 저장 지연 실패가 아니다.

만료 거부 뒤 남은 현재 촬영 2회는 실행하지 않았고 추가 촬영은 0회였다. 이는 시험 수집기의 촬영 횟수이며 계속 동작 중인 perception 서비스의 배경 촬영까지 0이라는 뜻은 아니다. 새 코드에서는 저장 장애를 실물 주입하지 않았으며 저장 실패/250ms 초과 후 중단은 가상 회귀로 재검증했다.

최종 현재 영상 13개와 checkpoint/anchor 각 2개, 총 JPEG **17개**·artifact **161개**·event **42개**를 검증했다. 첫 거부 시도는 이 분모에 포함하지 않는다. 소비 시점 1/12는 작은 표본의 실측 결과이며 이전 5/24와 조건을 통제한 속도 비교가 아니다. 저장 최대 237.66ms도 여유가 크지 않으므로 안정적인 제어 준비 완료로 주장하지 않는다.

컨트롤러 IDLE·motorEnabled=false·5개 출력 0, 출력 기록과 기존 disarm 명령 불변. CAM boot/failure/recovery 불변. Pi 운영 checkout `559db54`, 3개 서비스 active/NRestarts=0, throttled=0x0 유지. 펌웨어 업로드·운영 서비스 재시작·모터 명령은 없었다.

## 소프트웨어 테스트와 증거

- 전체 `python3 -m unittest discover -s tests -q`: **553개 통과, 22.211초**. 최종 유한 대기 보완 후 실행했다.
- 그 안의 신규 **15개**: rate 상수/큰 CAM 차분 비의존, 오래된 anchor/boot/held/trace/hash/host 거부, 수신 시각 reset 금지, 15초 경계·계산 종료 만료, 소비 시 고정 근거 재검사, identity/모드 교체, 현재 프레임 역순, 유한 대기 성공/소진/HTTP 오류, checkpoint 저장 실패, 현재 저장 실패/지연 후 추가 촬영 0, 획득 중 만료, 잘못된 설정 I/O 0.
- 별도 시험기 **3개**: 대기 주입 1회, 정상 경로에 주입 없음, 정상 시험 실패 시 만료 시험 생략. 전체 553개와 별도로 집계했다.
- 원본 검증기는 소스 hash·모든 artifact·exact anchor/이미지·metadata poll 순서·pending/final 소비 행·barrier 뒤 다음 capture·terminal report를 대조했다. 원래 기록 시각으로 ROI/소비를 재생했고 판정과 값이 일치했다. 이것은 처리 속도 benchmark가 아니다.
- 구현·실험 스크립트·최종 자료의 독립 리뷰 상태와 hash는 [validation.json](causal-anchor-2026-10-15/validation.json)에 기록한다.

[최종 실측 집계](causal-anchor-2026-10-15/live.json), [시험기](causal-anchor-2026-10-15/collect.py), [원본 검증기](causal-anchor-2026-10-15/verify.py), [첫 거부 검증기](causal-anchor-2026-10-15/verify_rejected.py), [장치 최종 상태](causal-anchor-2026-10-15/final-status.json). 전체 원본은 Mac `datasets/anchor_probes/2026-10-15/{final,fixed-wait-rejected}`와 Pi 시험 디렉터리에 보존했으며 Git에는 이미지를 넣지 않았다.

## 해석과 후속 검증

낮은 소비 통과율은 자동 제어 준비의 근거가 아니다. [후속 유한 관측 평가](2026-10-15-bounded-observation.md)는 고정 횟수·시간·저장 기준에서 미통과로 종료했다. 실제 행동 수집·조작 개선은 입증하지 않았다.
