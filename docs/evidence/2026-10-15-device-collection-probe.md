# 실제 수집 경로 비구동 검증 — 2026-10-15

**실제 장치 adapter의 읽기 경로와 원본 저장·감사를 구현하고 Pi에서 검증했다. 행동 송신 adapter 및 학습 적격 실물 transition 수집은 아직 완료하지 않았다.** 시간 표본은 일관됐지만 물리 시간 교정 완료로 표시하지 않는다.

## 조건과 시야

- 사용자가 배터리함→XL4015를 분리했다고 확인했다. USB·Pi 연결은 유지했다. 모터 명령, ARM, 펌웨어 업로드, 서비스 재시작은 수행하지 않았다.
- CAM `192.168.219.116`, 메인 ESP32 `192.168.219.113`, Pi perception `127.0.0.1:8766`을 조회했다. Pi 운영 저장소는 `559db54`이며 이번 코드는 운영 배포 대신 `/tmp/motionbrain-device-probe.pG9lsA/final-code`에서 실행했다. Mac 기준 HEAD는 `1050c40`이다.
- 초기 320×240 실제 프레임에는 흰 컵과 왼쪽의 일부 잘린 AirPods Max가 보였다. 사용자가 왼쪽 물체는 집게가 아닌 헤드폰이라고 확인했다. 초기 검출기는 이를 낮은 신뢰도의 vase로도 검출했다. 최종 저장 프레임에서는 헤드폰이 보이지 않고 컵이 보인다.
- 후속 사용자 사진으로 집게 근처에 카메라를 고정해 앞을 보는 장착임을 확인했다. 집게 자체가 영상에 안 보이는 것은 자연스럽고, 이를 시야 결함이나 카메라 재배치의 조건으로 삼지 않는다. 현재 장착을 유지하며 실제 집기 자세에서 컵의 영상 위치·크기를 교정한다. 화면 중앙을 집기 위치로 가정하지 않으며, 초기 들림·배치 성공은 작업자/별도 실험 영상으로 판정한다. 지정 배치 영역·초기 자세와 카메라 고정의 반복성은 실물 프로토콜에서 확정한다.

## 구현 범위

| 파일 | 내용 |
|---|---|
| `learning/device_adapter.py` | GET endpoint 허용 목록, redirect/자격 증명 URL/제어 query 거부, 요청·응답 시간 기록, exact observation/JPEG/hash, MCU capability·boot·출력 조회, command ID 결과 조회 |
| `learning/device_probe.py` | 기존 `ArtifactStore` 재사용, 불변 원본·이미지·오류 기록, 시계 진단, 시작/종료 출력 비교, 별도 probe audit |
| `tools/probe_action_collection.py` | Pi에서 실행하는 비구동 CLI, SIGTERM/중단 기록, 오프라인 audit |
| `tests/test_device_collection_probe.py` | 가상 장치·실제 로컬 HTTP 서버로 실패 경계 및 데이터 혼입 방지 회귀 |

- adapter의 `execute()`는 네트워크 요청 전에 항상 거부한다. 토큰이나 물리 실행 옵션은 없다. 기존 `CaptureRunner`는 계속 dry-run만 허용한다.
- 기존 원본 frame parser, 명령 요청/결과 일치 검사, 이미지 저장·hash·event 기록을 재사용한다. 존재하지 않는 명령/다른 boot/유실 응답은 `outcome_unknown`이며 재전송하지 않는다. 결과 조회가 terminal이라고 해도 물리 도달이나 파지 성공을 선언하지 않는다.
- 원본 관측의 Pi `hostClockId`와 monotonic 값을 보존한다. 클라이언트의 시각으로 바꿔 쓰지 않는다. exact observation 요청과 이미지 조회에 소요된 **클라이언트 내부 경과시간**을 추가해 조회 종료 시 관측 나이 상한을 계산한다.
- 나중에 읽은 M4 값을 오래된 영상의 동시 측정값으로 결합하지 않는다. 현재 상태 조회의 `ageMs`와 별도 값으로 남긴다. 이 probe는 transition을 만들지 않는다.
- 데이터는 `device_read_only` / `device_observation`과 별도 schema를 사용한다. 합성 자료로 표시하지도, 물리 행동 학습 자료로 승격하지도 않는다. 기존 manifest에 넣으면 session schema/경계 검증이 실패하며 학습 적격 0개다.
- HTTP body는 2 MiB 상한이며 자동 retry는 없다. socket timeout은 블로킹 호출 단위다. 응답 완료 후 총 요청 시간이 예산을 넘었으면 거부하지만 **강제적인 전체 wall-clock deadline을 보장하는 제어 transport는 아니다.** 활성 제어 경로의 polling 분리·deadline 구현을 대체하지 않는다.
- 기존 호스트 journal의 예약/송신 경로 및 unknown reconciliation **상태 변경**은 연결하지 않았다. 결과 조회 adapter만 준비됐다. 기존 `/shoulder`를 새 `/actions`로 몰래 전환하지 않는다.

## 소프트웨어 검증

최종 전체 회귀 **398개 통과, 12.168초** (`python3 -m unittest discover -s tests`). 기존 372개에 새 회귀 26개가 추가됐다. 실물 시험 횟수와 구분한다.

검증한 실패 경계: 누락 exact image에서 latest로 대체하지 않음, 이미지 hash 불일치 원본 보존, 관측 identity 불일치, controller/camera/perception 재부팅, 출력 변화, 현재 nonzero 출력, timeout/중단/저장 실패, 이미지 변조·terminal 누락 감사, 명령 결과 불명·내용 충돌·off 미확인, redirect 제어 endpoint 이동 차단, oversized body, 요청 예산 초과, 서로 다른 clock domain 보존, 시계 오차 포함 신선도 초과, M4 display-ready와 수집 freshness 구분, 학습 manifest 혼입 거부.

자체 코드리뷰에서 producer 시각 보존 외에도 exact 관측/JPEG 조회 지연을 나이 상한에 추가하고, 시계 교정 오차와 M4 20ms 정책을 별도로 검사하도록 보완했다. 이 최종 코드의 소스 SHA-256을 Pi 임시 실행 파일과 대조해 일치 확인했다.

## 실물 비구동 결과

최종 run: `dfe00922ec62447f9f245863b262ec92`. 앞선 초기 버전 probe도 12회 통과했지만 아래 수치는 **최종 버전 한 run** 기준이다.

| 항목 | 최종 결과 |
|---|---|
| 실제 HTTP GET | 66회; 부적격 명령 ID 조회의 예상 404 포함 |
| 완성된 관측 표본 | 12/12 |
| 저장한 JPEG | perception exact 이미지 12개 + 직접 camera 이미지 12개; 각각 hash 확인 |
| 카메라 시계 표본 | 12개, 서로 다른 원본 frame ID 12개 |
| 시계 표본 관찰 구간 | 약 10.91초 |
| camera capture 왕복/수신 시간 | 33.07~50.54ms, 중앙값 38.58ms |
| 기존 객체 인식 지연 | 2,736.70~2,780.31ms |
| exact 관측의 producer 나이 | 3,435.89~7,282.31ms |
| 이미지 조회 종료까지 반영한 나이 상한 | 3,514.84~7,361.36ms |
| M4 status sample age | 10~25ms; 12개 중 4개가 정책 20ms 초과 |
| 출력 기록 / capability | 시작·종료 동일; `enabled=false`, `active=false`, recordCount=0 |
| 모터 명령 | **0회** |
| Mac 복사 후 artifact audit | 통과 |
| 학습 적격 / 실물 행동 수집 준비 | **false / false** |

출력 로그에는 사용자가 이전에 웹으로 조작한 M3 이력이 있다. 이를 이번 시험의 구동으로 해석하거나 과거 로그가 모두 0이라고 주장하지 않는다. 이번 구간에서는 로그가 증가/변경되지 않았고, 현재 M1~M5 speed=0·disabled, 제어 상태 IDLE을 확인했다. 마지막 명령 ID 9의 기존 web disarm도 유지됐다.

최종 점검에서 STM32 IMU·M4 AS5600 정상, Pi 서비스 3개 active·NRestarts=0, throttled=0x0, CAM captureFailures=0, perception fresh/ok·consecutiveErrors=0이다. 이 상태는 실제 행동 허용이나 수집 적격 판정과 다르다.

## 시간 교정 판정

각 직접 `/capture`에서 `capturedUs + sourceAgeUs`로 **응답 metadata 작성 시점의 카메라 시간**을 얻고, 요청 시작~본문 수신 완료의 Pi monotonic 구간과 대응시켰다. 같은 속도로 흐르는 시계를 가정한 offset interval 12개의 교집합은 약 `[6729.074811, 6729.104816]`초, 폭 **30.01ms**였다.

이는 **이번 짧은 표본의 일정 offset 가정과 모순이 없었다는 증거**다. 30ms를 영구 교정 오차나 물리 촬영 오차로 사용할 수 없다. first DMA buffer와 실제 광학 장면 시점의 대응, 장기 clock drift, controller 출력 시계와의 연결은 미검증이다. boot/hostClockId가 바뀌면 이 구간도 재사용하지 않는다. `physicalCalibration=false`, `captureHostBoundsCalibrated=false`, `usableForControl=false`를 유지했다.

현재 가장 큰 지연은 직접 카메라 HTTP 수신보다 Pi 객체 인식 및 주기적 캐시 관측에 있다. 기존 서비스의 display/검출 freshness가 true여도 수집 정책의 영상 100ms·M4 20ms 한계를 통과하지 못한다. 기준을 낮추거나 타임스탬프를 새로 찍어 통과시키지 않았다.

## 다음 순서

1. 현재 집게 근처의 전방 카메라 장착을 유지한다. 실제 집기 자세에서 컵의 영상 위치·크기를 교정하고 배치 영역과 외부 성공 판정 방법을 정한다. 집게를 영상에 넣는 것은 선행조건이 아니다.
2. 동작 후 정지 관찰 루프에 맞춰 빠른 관측 경로를 준비한다. 기존 약 2.7초 객체 인식을 그대로 행동 직전 관측으로 쓰지 않는다. 기존 검출/ROI 특징 경로 재사용 또는 가벼운 처리의 실제 지연부터 측정하며 모델·GPU를 먼저 키우지 않는다.
3. 실제 촬영/시간 근거와 controller/M4 sample의 시간 구간을 연결한다. freshness 정책은 측정한 동작·관측 특성에 근거해 결정하며 자동 완화하지 않는다.
4. host journal ↔ `/actions` 실행·결과·출력 연결과 결과 불명 reconciliation을 완성한다. 활성 UART seq/session, 제어 polling 분리·deadline 관문도 남아 있다. 새 API 기본 비활성 유지.
5. 위 준비 후 사용자에게 한 축 구동 범위·시작 자세를 안내하고 모터 전원을 다시 연결한다. 실제 최소 동작/정지/중복/단절을 검증한 뒤 행동 전후 학습 데이터를 수집한다.

## 보관과 재현

- [최종 요약·소스 hash](device-collection-probe-2026-10-15/summary.json), [시계 표본](device-collection-probe-2026-10-15/clock-samples.json), [최종 장치 상태](device-collection-probe-2026-10-15/final-status.json).
- 원본 JSON·JPEG·event 전체: 로컬 `datasets/device_probes/dfe00922ec62447f9f245863b262ec92/`. Git 제외 폴더이며 이미지 원본은 저장소에 추가하지 않는다. Pi 임시본은 영구 보존 위치가 아니다.
- 리뷰 후 커밋 대상 코드다. 운영 서비스/펌웨어 변경은 없다.

Pi에서 실제 주소를 확인한 뒤 비구동 실행:

```sh
python3 tools/probe_action_collection.py \
  --controller-url http://192.168.219.113 \
  --camera-url http://192.168.219.116 \
  --perception-url http://127.0.0.1:8766 \
  --samples 12 --interval 0.5
```

Mac에서 저장 결과만 검증:

```sh
python3 tools/probe_action_collection.py --audit-run \
  datasets/device_probes/dfe00922ec62447f9f245863b262ec92
python3 -m unittest discover -s tests
```

## 커밋 전 리뷰 후속

명령 결과 조회의 timeout/서버 오류/예상 밖 응답이 예상한 `404 not_found`와 같은 `outcome_unknown`으로 합쳐져 probe가 완료 처리될 수 있는 P2 문제를 수정했다. 실행 결과 불명은 유지하면서 `lookupStatus`로 조회 검증 상태를 분리한다. 정상적인 `404` + `code=not_found` + `state=outcome_unknown`만 이 부정 조회 시험의 통과 근거로 삼는다. 실패 시 중단하며 재전송은 없다.

추가 회귀에서 timeout·다른 404·500·예상 밖 200 기록을 주입해 모두 중단 및 모터 명령 0회를 확인했다. 최종 전체 **399개 통과, 13.754초**. 수정본의 Pi GET 재검증은 2/2 표본·원본 JPEG 4개·출력 로그 동일·모터 명령 0회이며, 위 12회 원본 시험과 분리한다. [리뷰 검증 기록](device-collection-probe-2026-10-15/review-validation.json)을 참고한다. 남은 확정 차단 결함은 발견하지 못했다.
