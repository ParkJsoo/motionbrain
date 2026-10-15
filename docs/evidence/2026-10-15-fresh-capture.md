# 선택형 요청 이후 프레임 캡처 — 2026-10-15

후속 상태: CAM 업로드·48장 비구동 비교를 완료했다. [실물 비교 결과](2026-10-15-fresh-capture-live.md)를 참고한다. 아래 업로드 미실시 표기는 이 구현 작업 당시의 기록이다.

**기존 `/capture` 기본 동작을 유지하며 `/capture?fresh=1` 선택형 경로를 구현했다.** 요청 처리 시작 시각보다 first-DMA timestamp가 뒤인 프레임만 응답하고, 오래된 프레임으로 대체하지 않는다. 전체 소프트웨어 **496개 테스트 통과(19.748초)**, ESP32-CAM compile/link 성공이다.

**업로드·장치 요청·재부팅·모터 구동은 이번 작업에서 수행하지 않았다.** 실제 대기 시간, 성공률, 영상 나이 또는 100ms 통과율 개선은 아직 미검증이다. 이 문서의 결과는 소프트웨어 검증과 빌드 결과다.

## 계약과 범위

- `/capture` 및 기존 화면의 `/capture?t=...`는 기존 단일 `esp_camera_fb_get()` 경로를 유지한다. fresh 옵션의 허용값은 정확히 `1` 하나이며 빈 값/0/중복 fresh는 400으로 거부한다. 해상도·JPEG quality·fb_count·카메라 드라이버를 바꾸지 않는다.
- fresh 모드는 같은 카메라 boot clock의 `requestedUs=esp_timer_get_time()`를 저장한 뒤 프레임을 획득한다. `capturedUs > requestedUs`와 기존 source metadata 유효성을 모두 만족해야 선택한다. 같은 시각도 거부하며 SDK의 first-DMA 시각을 요청 시각으로 덮어쓰지 않는다. 광학 노출 종료나 실제 대상 위치를 증명하는 계약은 아니다.
- 선택 과정에서 최대 4개의 버퍼를 가져오며, 각 획득 전후에 200ms 선택 예산과 연결 상태를 검사한다. 오래된 버퍼는 SDK에 반환한 뒤 다음 후보를 확인한다. 성공 버퍼는 기존 HTTP 쓰기 경로가 소유하고 완료/실패 뒤 반환한다. 선택 과정에서 거부한 버퍼 수는 응답·상태에 명시한다. 이 버퍼들은 HTTP 관측으로 채택되지 않은 후보이며 학습 데이터로 저장됐다고 주장하지 않는다.
- SDK의 blocking `esp_camera_fb_get()`를 중간 취소할 수 없으므로 **200ms는 HTTP 반환 시간의 강제 상한이 아니다**. 호출이 오래 걸리면 반환 뒤 예산을 검사하여 버퍼를 돌려주고 503으로 실패한다. 연결 종료도 blocking 호출이 반환한 뒤 확인할 수 있다. 강제 watchdog 재시작이나 드라이버 교체를 추가하지 않았다.
- null/잘못된 timestamp/시계 역행/연결 종료/예산·후보 수 초과는 실패다. fresh 선택 실패는 카메라 재설정이나 기본 캡처 재요청으로 숨기지 않는다. 기존 기본 캡처의 null/slow recovery 동작은 유지한다.

성공 시 기존 원본 ID·first-DMA timestamp·source-age 헤더와 함께 아래 증거를 보낸다.

```text
X-MotionBrain-Capture-Mode: after_request
X-MotionBrain-Capture-Requested-Us: <같은 camera boot clock의 요청 처리 시작 시각>
X-MotionBrain-Discarded-Frames: <선택에서 반환한 버퍼 수 0..3>
```

`/status`에 `freshCaptureSupported`, `freshCaptureBudgetMs`, `freshCaptureMaxFrames`, 요청/거부/버퍼 반환 누계를 추가했다. fresh 모드의 `lastCaptureMs`는 **여러 fb_get·return·yield를 포함하는 선택 전체 시간**이다. 호스트 timing 진단도 이 경우 `serverFreshFrameSelectionMs`로 기록하며, 기본 모드의 `serverFrameAcquireWaitMs`와 구분한다. 다른 클라이언트가 끼면 기존 카운터 검증에 따라 서버 시간 연결을 거부한다.

## 호스트 검증

`ReadOnlyHTTP`는 camera `/capture`에 정확한 `{"fresh":"1"}` query만 추가 허용한다. 기존 GET whitelist·redirect/proxy/retry 차단은 유지한다. `tools/probe_fast_observation.py --fresh-capture`는 기본 비활성이다.

옵션을 켜면 먼저 capability를 읽어 미지원 장치는 촬영 전 거부한다. 성공 응답에서도 exact query·mode·requested timestamp·discard count와 원본 metadata를 검사한다. 구형 펌웨어가 옵션을 무시한 응답이나 요청 이전/동시 프레임은 실패로 기록한다. 수신한 원본과 receipt는 기존 종료/중단 경로로 보존하며 자동 fallback은 없다. 다른 boot의 프레임은 이전 ROI anchor로 수락하지 않는다.

**기존 100ms 기준과 나이 상한 계산식은 그대로다.** fresh 증거가 나이 검사를 면제하지 않는다. 요청 시작/전송 시작부터 응답까지의 전체 경과를 여전히 포함하므로 새 프레임을 기다리는 시간도 보수적으로 상한에 포함된다. 이 계약만으로 지연 상한이 반드시 줄어들지는 않는다. 같은 clock rate 가정, 광학/clock/M4 교정, 실제 소비 시점 재검사와 물리 제어 적격성은 별도 미완료다.

## 변경 파일과 검증

- `firmware/esp32cam/src/fresh_capture.h`: 생산 코드와 native 테스트가 공유하는 선택/소유권/시간 한도 로직, query mode 및 증거 헤더.
- `firmware/esp32cam/src/main.cpp`: 선택형 분기·상태·응답 연결. 기존 단일 캡처 기본 경로 보존.
- `learning/fresh_capture.py`, `source_age.py`: fresh 증거 검증과 기존 exact receipt/원본 연결.
- `learning/device_adapter.py`, `fast_probe.py`, `capture_timing.py`, `tools/probe_fast_observation.py`: query·capability·옵션·선택 시간 진단.
- native C++ 및 Python 테스트: query 모드, old→new 선택, 같은 시각/반복 후보 한도, null, invalid/future timestamp, 예산 이전/정확한 경계/4초 늦은 반환, 시계 역행, 연결 종료, boot/header 계약 등 **native 9개 시나리오**를 실제 생산 helper로 실행한다. C++ 헤더를 Python으로 roundtrip한다.
- fake probe/로컬 HTTP: 미지원 firmware, 옵션 무시, 원본 보존, boot 변경, 503 무재시도, 엄격한 query whitelist, 기본 동작 유지, fresh 증거가 있어도 100ms 초과면 수락 0·명령 0을 검사한다. 이전 481개에 Python 테스트 항목 15개를 추가해 **전체 496개**다. native 시나리오를 Python 테스트 수에 중복 합산하지 않았다.

[전체 테스트 로그](fresh-capture-2026-10-15/tests.log), [빌드 로그](fresh-capture-2026-10-15/build.log), [검증 소스·펌웨어 binary hash](fresh-capture-2026-10-15/validation.json). CAM 빌드는 RAM 51,844/327,680 bytes, flash 906,493/3,145,728 bytes로 성공했다. binary 자체는 Git에 포함하지 않았다. 독립 리뷰 에이전트가 fresh 관련 13개·HTTP 전송 38개, 총 51개 테스트를 별도 실행했다. 최종 코드·빌드/테스트 로그·소스/binary hash·문서를 검토하여 P0/P1/P2 문제 없음.

## 해석과 후속 검증

[후속 CAM 업로드·48장 비교](2026-10-15-fresh-capture-live.md)에서 요청 이후 캡처 계약을 검증했다. 새 캡처만으로 100ms 소비 기준이나 실제 제어 준비를 충족한 것은 아니다.
