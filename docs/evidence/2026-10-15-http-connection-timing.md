# HTTP 연결 구간과 카메라 내부 계측 가능성 — 2026-10-15

**정지 장면 16개에서 HTTP 전체 중앙값은 42.41ms, 그중 TCP 연결 7.75ms, 요청 전송 완료~헤더 파싱 완료 17.61ms였다. 주소 해석은 0.09ms다. 프레임 나이와 ROI 계산을 합치면 114~159ms로 여전히 100ms 정책을 초과한다.** 네트워크 연결만 개선하면 해결된다는 결론은 아니다.

최종 코드의 전체 소프트웨어 테스트 **452개 통과(17.148초)**. 독립 리뷰에서 HTTP 계측과 opener 재사용 수정의 P0/P1/P2 없음. 실물은 GET 전용·정지 장면이고 모터 명령 0회다. 펌웨어·카메라 설정·운영 서비스·신선도 기준을 바꾸지 않았다.

## 구현과 계약

- `learning/http_timing.py`: 기존 urllib/http.client 경로에 요청별 주소 해석 시작/완료, 연결 완료, 요청 전송 시작/완료를 기록한다. 전역 socket 함수를 바꾸지 않는다. DNS 후보 주소의 연결 실패 후 다음 주소 시도는 표준 연결 동작을 유지하며, HTTP 요청 자체를 재시도하지 않는다. 실패한 소켓은 닫는다.
- `ReadOnlyHTTP(connection_timing=True)`: HTTP 카메라 GET에만 적용한다. HTTPS 카메라 계측은 거부한다. 기존 경로 허용 목록·프록시/리다이렉트 차단·본문 크기 제한·hash·요청 시간 초과 검사를 유지한다. opener는 초기화 때 한 번 만들고 trace는 각 Request에 따로 전달한다.
- 기존 CLI의 `--connection-timing`은 `--capture-timing`과 함께 사용한다. 기존 원본 저장·중단 기록·출력 로그 확인을 재사용한다. 운영 수집기와 실제 제어 경로에 연결하지 않았다.
- 상세 시각의 누락·역행·비유한 값·요청 범위 이탈을 거부하고, 합이 요청 시작~헤더 사용 가능 시간과 일치하는 구간을 계산한다. **기존 age 상한이나 offset 구간을 좁히지 않는다.** 전송 완료 시각은 서버의 요청 수신 시각을 보장하지 않으므로 source age의 새 기준점으로 쓰지 않는다.

주소 해석 값은 `getaddrinfo()` 전체 시간이다. 이번에는 숫자 IP로 요청했으므로 DNS 서버 응답 시간으로 해석하지 않는다. TCP 구간에는 socket 준비·주소 후보 시도·스케줄링이 포함된다. 요청 전송 완료~헤더 사용 가능 구간에는 서버 처리·네트워크·클라이언트 스케줄링·헤더 파싱이 섞여 있으며 순수 Wi-Fi 지연이나 TTFB가 아니다. 본문 읽기 구간에도 선행 버퍼링이 포함될 수 있다. 서버 write 시간은 클라이언트 시간과 겹치므로 더하지 않는다.

## 실측과 계측 자체 오류 수정

Pi 별도 `/tmp/motionbrain-http-timing-20261015`에서 QVGA 320×240/JPEG quality 15, 기존 서비스 유지 조건으로 실행했다. 기존 운영 저장소는 `559db54` 그대로다.

첫 개발 표본 `0fb9b5a3c9b64c12a550c46d30d6193e`에서 요청마다 `build_opener()`를 호출한 탓에 클라이언트 준비 중앙값이 85.00ms였다. 기본 HTTPS context 생성도 이 경로에 포함된다. 이 표본은 **계측 결함을 찾은 기록이며 기존 장치 성능의 기준선에서 제외**한다. opener를 초기화 때 재사용하도록 고쳤고, GET 중 opener 재생성이 없으며 두 요청의 trace가 섞이지 않는 회귀 테스트를 추가했다. 초기 결과 JSON과 Git 제외 원본을 보존했다. 최종 코드 hash 일치 검증은 아래 최종 표본에만 적용한다.

최종 표본 `2e38fa6f40c94aeaac37866d078c5d59`의 결과:

| 구간 | 중앙값 | 최소~최대 |
|---|---:|---:|
| 클라이언트 요청 준비 | 0.51ms | 0.34~0.84ms |
| 주소 해석 | 0.09ms | 0.06~0.16ms |
| TCP 연결 | 7.75ms | 5.15~13.32ms |
| 연결 완료~전송 시작 | 0.04ms | 0.03~0.10ms |
| 요청 전송 | 0.11ms | 0.08~0.22ms |
| 요청 전송 완료~헤더 사용 가능 | 17.61ms | 10.93~30.14ms |
| 헤더 사용 가능~본문 읽기 완료 | 15.51ms | 8.92~27.80ms |
| 본문 읽기 완료~receipt 완료 | 0.17ms | 0.12~0.26ms |
| HTTP 전체 | 42.41ms | 36.40~54.94ms |
| CAM 보고 원본 나이 | 54.39ms | 37.35~66.26ms |
| ROI 계산 | 37.48ms | 32.55~46.21ms |
| 보수적 계산 완료 나이 | 134.71ms | 114.03~159.33ms |

개별 표본 구간의 합은 일치하지만 각 행의 중앙값끼리 더한 값이 전체 중앙값과 같을 필요는 없다. 16/16은 `source_age_exceeds_policy`로 거부했고 anchor age는 모두 15초 이내였다. 서버 통계 귀속 조건도 16/16 통과, `fb_get` 대기 0~1ms·본문 write 12~22ms였다. constant-offset 후보 교집합 16.28ms는 물리 교정이나 미래 drift 한계가 아니다. 이전 실험과 표본이 다르므로 작은 중앙값 차이를 성능 개선/악화로 단정하지 않는다.

## 카메라 내부 계측 가능성

설치된 SDK `framework-arduinoespressif32` 버전 `3.20017.241212+sha.dcc1105b`의 공개 헤더와 ESP32 정적 라이브러리 심볼을 확인했다. `camera_fb_t`는 buf/len/width/height/format/timestamp만 제공하며 timestamp 계약은 첫 DMA buffer다. `esp_camera_fb_get/return`, `cam_take/give`는 있지만 공개 헤더에 프레임 완성·큐 진입 timestamp 또는 해당 콜백 계약은 없다. 이미지 변환의 callback을 프레임 준비 이벤트로 취급하지 않는다. 설치 패키지에는 이 경로의 C 구현 대신 `libesp32-camera.a`가 들어 있다. 정확한 드라이버 구현의 내부 이벤트 위치는 이번에 검증하지 않았다.

1. **애플리케이션 펌웨어만 계측:** 동일 boot/source frame ID에 `fb_get` 전/후와 age 샘플 시각을 microsecond 단위로 붙일 수 있다. 현재 millisecond 대기 시간과 mutable `/status` 연결보다 직접적인 증거가 된다. 그러나 프레임 완성 전 시간과 완성 후 큐 체류는 여전히 분리되지 않는다.
2. **정확한 큐 체류 분리:** 사용 중 SDK와 일치하는 드라이버 소스/빌드를 확보하고, 프레임 완성·enqueue·dequeue 이벤트를 해당 frame slot의 generation 및 원본 ID와 연결해야 한다. 버퍼 재사용·drop·recovery 시 오귀속 방지와 소프트웨어 테스트가 필요하다. 단순 `fb_get` 후 timestamp 추가로 해결됐다고 주장할 수 없다. 노출의 광학적 시작/종료는 이 계측과도 별도다.

현재 목표를 위해 SDK/드라이버 교체를 선행조건으로 삼지 않는다. 이번에는 펌웨어를 수정하거나 빌드/업로드하지 않았다. 향후 앱 계측을 진행한다면 변경 코드·비구동 빌드/회귀·리뷰를 먼저 준비하고 업로드 승인을 받아야 한다. 장비 조건은 배터리함→XL4015 분리, 카메라 USB 안정 연결, Pi/STM32/메인 ESP32 연결 유지다. 업로드 뒤 새 boot ID로 최소 16개 GET 표본의 ID/hash/시간 순서·출력 로그 불변을 비교한다. 이것만으로 실제 동작 성공이나 시계 교정 완료를 선언하지 않는다.

## 검증·증거

추가 9개 테스트는 옵션 범위·HTTPS 거부·같은 응답 bytes/hash 보존·리다이렉트/제어 경로 차단·resolver/connect 실패·소켓 정리·상세 시각 오류·opener 재사용·요청별 trace 분리를 검증한다. 기존 테스트와 함께 452개 통과했다. 로컬 HTTP 서버 검증은 실물 구동 결과가 아니다.

최종 실물 원본 JPEG 17개, artifact 140개, event 34개를 Mac으로 복사해 hash·종료 report·실행 소스 hash를 재검증했다. 시작/종료 출력 로그 동일, 마지막 IDLE·모터 출력 0·motorEnabled=false, CAM boot 유지·captureFailures=0, 서비스 3개 active·NRestarts=0을 확인했다. 실제 행동 학습 데이터는 여전히 0개다.

- [최종 실행 결과](http-connection-timing-2026-10-15/live-results.json), [프레임별 시간](http-connection-timing-2026-10-15/timing-samples.json), [원본 검증](http-connection-timing-2026-10-15/validation.json).
- [소스·환경](http-connection-timing-2026-10-15/session.json), [SDK 계약·hash](http-connection-timing-2026-10-15/driver-contract.json), [테스트](http-connection-timing-2026-10-15/tests.log), [종료 상태](http-connection-timing-2026-10-15/final-status.json).
- [제외한 첫 계측 결과](http-connection-timing-2026-10-15/discarded-first-instrumentation.json).
- Git 제외 원본: `datasets/fast_probes/2e38fa6f40c94aeaac37866d078c5d59`, 초기 표본도 같은 상위 폴더에 보존했다. Pi `/tmp/motionbrain-http-timing-20261015/runs/`에도 보존했다.

```sh
/home/motionbrain/.cache/motionbrain/opencv-venv/bin/python \
  tools/probe_fast_observation.py \
  --controller-url http://192.168.219.113 --camera-url http://192.168.219.116 \
  --samples 16 --capture-timing --connection-timing --output-dir /tmp/new-http-timing-runs
```

## 바로 다음 작업

1. **비구동 실제 장면 검증:** 카메라 고정을 유지하고 컵을 사용자가 조금 이동·가렸다가 복귀시켜 원본과 수동 정답을 기록한다. 기하 일치/모호성/가림 거부와 신선도 실패를 분리해 평가한다. 느린 프레임을 제어 허용으로 바꾸지 않는다. 준비할 것은 흰 컵과 가림용 종이, 배터리함 분리·USB/Pi 유지다.
2. **남은 지연 개선:** 위 실제 자료에서 정확도와 거부 동작을 유지하는 ROI 계산 최적화를 검증한다. 전체 후보/경쟁 후보 검사를 생략해 지연만 낮추지 않는다. TCP 재사용은 현재 CAM의 `Connection: close`와 맞지 않으므로 클라이언트 옵션만 바꿔 해결됐다고 주장하지 않는다.
3. 실제 소비 시점 신선도·시계/M4 시간 범위·제한 명령 실물 검증을 갖춘 뒤 행동 수집과 학습 비교로 간다. 독립적으로 가능한 Mac 오프라인 학습/평가 코드 준비는 병행할 수 있다.
