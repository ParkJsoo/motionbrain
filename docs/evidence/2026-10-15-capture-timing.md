# 카메라 원본 나이·HTTP 구간 진단 — 2026-10-15

**읽기 전용 16개 표본에서 `fb_get` 대기는 0~1ms였지만, HTTP 헤더를 만들 때 프레임은 첫 DMA 시각에서 이미 37~69ms 지난 상태였다. 요청부터 응답 수신 완료까지는 35~59ms였다. 촬영·버퍼 체류와 HTTP 구간을 더 좁혔으며, 센서 노출/읽기와 버퍼 체류를 각각 확정한 것은 아니다.**

전체 소프트웨어 테스트 **443개 통과(15.494초)**, 독립 코드리뷰에서 P0/P1/P2 차단 문제 없음. 실물은 정지 장면·GET 전용이고 모터 명령 0회다. 기존 100ms 정책, 제어·학습 적격성, 운영 서비스·펌웨어·카메라 설정은 변경하지 않았다.

## 무엇을 추가했는가

- `learning/device_adapter.py`: 기존 요청 시작·receipt 완료 시각 사이에 `headersAvailableMonotonic`, `bodyReadCompletedMonotonic`을 추가했다. 중간 시각이 역행해도 요청을 거부한다. 기존 `receivedMonotonic` 의미와 신선도 판정은 유지한다.
- `learning/capture_timing.py`: 원본 ID·timestamp 의미·동일 host clock을 검사하고, 요청 구간과 진단용 age/offset 범위를 계산한다. 미확인 노출/읽기 시간과 버퍼 체류 시간은 null로 남긴다. 여러 host/boot, 중복·역행 프레임, 잘못된 표본을 합치지 않는다.
- 기존 `tools/probe_fast_observation.py`에 `--capture-timing` 옵션을 추가했다. 별도 수집 runner를 만들지 않고 원본·hash·중단/저장·출력 로그 검증을 재사용한다. 옵션을 켜면 각 capture 앞뒤에 `/status` GET을 추가하고 `timing/*.json`을 저장한다.

`headersAvailableMonotonic`은 `urllib.open()`이 HTTP 헤더를 파싱해 반환한 것을 관측한 시각이다. 첫 바이트 도착 시각이나 순수 네트워크 전달 지연이 아니다. 본문 구간에는 이미 버퍼링된 바이트의 읽기와 클라이언트 스케줄링도 포함된다.

## 펌웨어에서 확인한 시간 의미

`firmware/esp32cam/src/main.cpp::handleCapture()`는 `esp_camera_fb_get()` 소요 시간을 `lastCaptureMs`에 기록한다. 이후 프레임 timestamp와 현재 `esp_timer_get_time()`으로 source age를 계산하고, HTTP 헤더를 쓰고, 본문 쓰기 시간을 `lastWriteMs`로 기록한다. `source_frame_metadata.h`와 설치된 `esp_camera.h`의 timestamp 계약은 첫 DMA buffer 시각이다. 광학 노출 시각으로 검증된 값이 아니다.

`lastCaptureMs`는 **프레임 취득 대기**이며 센서가 프레임 전체를 생성하는 시간이 아니다. `lastWriteMs`는 서버의 본문 socket write 구간이며 클라이언트 전달 완료를 보장하지 않는다. 여러 구간이 겹치므로 표의 서버 시간을 클라이언트 시간에 다시 더하지 않는다.

상태의 마지막 값은 다른 클라이언트가 덮어쓸 수 있다. 따라서 앞뒤 boot가 해당 원본과 같고, `captureRequests`·`captures`가 각각 정확히 1 증가하고, failure/recovery 증가가 없고, byte 수와 duration 형식이 맞을 때만 서버 통계를 해당 프레임에 연결한다. 조건이 맞지 않으면 원본·클라이언트 시간은 남기되 서버 통계 귀속은 거부한다.

## 실제 결과

Pi의 별도 `/tmp/motionbrain-capture-timing-20261015` 코드로 실행했다. QVGA 320×240/JPEG quality 15, PSRAM 사용, 기존 perception/ROS/dashboard 실행 상태에서 정지한 컵을 관찰했다. 설정을 바꾸거나 다른 서비스의 capture를 중단하지 않았다. 표본 16개 모두 상태 카운터 귀속 조건을 통과했으나 이것이 모든 시점에 동시 요청이 없다는 증거는 아니다.

| 구간 | 중앙값 | 범위 |
|---|---:|---:|
| CAM이 헤더 생성 전에 보고한 원본 나이 | 51.22ms | 37.13~68.71ms |
| 요청 시작~헤더 사용 가능 | 28.07ms | 19.25~41.29ms |
| 헤더 사용 가능~본문 읽기 완료 | 11.42ms | 8.29~21.21ms |
| 본문 읽기 완료~receipt 완료(hash 등) | 0.14ms | 0.13~0.24ms |
| 요청 전체 | 39.67ms | 34.63~59.37ms |
| 해당 서버 `fb_get` 대기 | 0ms | 0~1ms |
| 해당 서버 본문 write | 16ms | 10~29ms |
| ROI 계산·검증 | 36.48ms | 31.19~45.16ms |
| 보수적 계산 완료 나이 | 126.30ms | 113.16~161.61ms |

**16/16은 `source_age_exceeds_policy`로 거부했다.** 이전 실험의 중앙값 138ms와 이번 126ms 차이를 이번 진단 기능의 성능 개선으로 해석하지 않는다. 서로 다른 시점의 프레임이며 status GET이 추가된 측정 조건이다.

추론 가능한 것은 다음까지다. 이 표본에서는 긴 `fb_get` 대기가 원인이 아니다. 응답 헤더 생성 전에 이미 존재하는 원본 나이와 HTTP 구간이 계산 비용 외에 상당 부분을 차지한다. 프레임 생성 과정의 시간과 생성된 프레임의 큐 체류 시간은 현재 API로 분리되지 않는다. 전송 구간 역시 DNS/TCP 연결·서버 처리·Wi-Fi·클라이언트 스케줄링으로 더 나눠 측정한 값은 아니다.

## 시계 구간과 한계

CAM source age를 계산한 이벤트는 요청 시작 이후, 클라이언트가 헤더를 사용할 수 있기 이전에 있다. 같은 시계 속도를 가정하면 수신 완료 시 원본 나이는 다음 진단 구간에 있다.

```text
하한 = CAM 보고 나이 + (receipt 완료 - 헤더 사용 가능)
상한 = CAM 보고 나이 + (receipt 완료 - 요청 시작)
```

이는 대칭 네트워크나 RTT/2를 가정하지 않는다. 하한을 제어 판정에 쓰거나 두 값의 중간을 실제 나이로 단정하지 않는다. 표본별 하한의 중앙값은 63.42ms, 상한의 중앙값은 92.44ms였다. 개별 구간은 원본 JSON에 보존했으며 중앙값 둘을 특정 프레임의 구간으로 사용하면 안 된다. ROI 계산·이후 저장/소비 지연은 이 수신 완료 값에 아직 포함되지 않는다.

source timestamp + 보고 age로 CAM 이벤트 시각을 구하고, 요청 시작~헤더 사용 가능 구간으로 constant-offset 후보를 계산했다. 16개 구간의 교집합 폭은 **17.42ms**였다. 해당 짧은 관찰에서 같은 offset 가정과 모순이 없었다는 의미일 뿐, 물리 시계 교정·미래 drift 한계·M4 시간 연결을 입증하지 않는다. 이 추정은 제어 코드에 적용하지 않았으며, 샘플 간 source timestamp 차이를 센서 FPS로 해석하지 않는다.

## 검증과 원본

추가 14개 테스트는 헤더/본문 시각 순서, 요청 초과·역행, 진단 구간 계산, header 대소문자, 다른 capture·실패·recovery에 의한 상태 귀속 거부, host/boot/중복 프레임 혼합 차단, 기존 probe 연결을 검증한다. 별도 리뷰 에이전트가 timing 10개와 fast probe 32개를 직접 실행하고 펌웨어 시간 의미를 대조했다. 전체 443개는 주 작업에서 실행했다.

원본 JPEG 17개, artifact 140개, event 34개를 Mac으로 복사해 hash·종료 레코드·실행 소스 hash를 확인했다. 출력 로그는 시작/종료 동일하다. 종료 장치 IDLE·모터 출력 0, CAM captureFailures=0, Pi 서비스 3개 active·NRestarts=0을 확인했다. 학습 적격 실제 행동 데이터는 여전히 0개다.

```sh
/home/motionbrain/.cache/motionbrain/opencv-venv/bin/python \
  tools/probe_fast_observation.py \
  --controller-url http://192.168.219.113 \
  --camera-url http://192.168.219.116 \
  --samples 16 --capture-timing --output-dir /tmp/new-capture-timing-runs
```

- [실행 결과·요약](capture-timing-2026-10-15/live-results.json), [프레임별 시간 구간](capture-timing-2026-10-15/timing-samples.json), [카메라 상태 표본](capture-timing-2026-10-15/camera-status-sample.json).
- [실행 코드·환경](capture-timing-2026-10-15/session.json), [원본 검증·통계](capture-timing-2026-10-15/validation.json), [전체 테스트](capture-timing-2026-10-15/tests.log), [종료 상태](capture-timing-2026-10-15/final-status.json).
- Git 제외 원본: `datasets/fast_probes/b513aab86fed421f9fc96c23c939115a`. Pi에도 `/tmp/motionbrain-capture-timing-20261015/runs/` 아래 같은 ID로 보존했다.

## 다음 단계

1. 장치 변경 없이 먼저 HTTP 연결·응답 처리 비용을 추가 분리하고, 같은 원본·동일 요청 조건에서 비교할 수 있는 방법을 마련한다. 시간 구간 추정은 광학 timestamp·drift 검증 전까지 진단용으로만 유지한다.
2. 프레임 생성과 큐 체류를 정확히 나누려면 JPEG 준비/큐 진입·프레임 취득·age 샘플 시각을 **해당 frame ID와 함께** 내보내는 계측이 필요하다. 드라이버에서 확보 가능한 시각과 구현 범위를 먼저 확인해야 하며, 지금 계측이나 펌웨어 업로드를 수행한 것은 아니다. 이 단계가 필요해지면 배터리함 분리·USB 안정 연결, 비구동 업로드 승인과 비교 실험 내용을 구체적으로 제시한다.
3. 별도로 컵 수동 이동·가림·유사 물체에서 실물 측정 정확도/거부를 검증한다. 카메라 장착 위치는 유지하고 모터는 구동하지 않는다. 이후 실제 소비 시점의 신선도·M4 시각·제한 명령 실행 검증을 갖춰 행동 수집으로 진행한다.
