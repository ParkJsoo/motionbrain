# 원본 프레임·신선도·행동 전후 schema — 2026-10-14

> 현재 상태 (2026-10-14): 이 단계는 `ca48f04`에 커밋되었다. 최신 코드 수정은 `73132e9`이며, 수정 후 전체 **372개 테스트 통과(10.949초)**와 독립 재리뷰를 기록했다. 아래 단계별 테스트 수는 당시 결과로 보존한다. 완료 범위·미완료 구현·다음 학습 단계는 [종합 계획서](../ai-manipulation-plan-2026-10-14.md), 마지막 회귀는 [시계 오차 수정 기록](2026-10-14-dry-run-collector-software.md#중간-리뷰-후-실행-전-시계-오차-검사-수정)을 기준으로 한다. 오늘 새 실물 검증이나 실제 데이터 학습은 하지 않았다.

MCU 기반 커밋: `1bcd59e` (`feat: add bounded MCU actions and applied output records`).
MCU 변경은 독립 리뷰에서 확정된 P1/P2 문제 없이 커밋했다. 이 문서는 그 이후의
S3a/S3b/S4a 소프트웨어 작업을 기록한다. 로봇팔·Pi·카메라에 연결하거나 업로드하지 않았다.

## 구현 파일과 완료 범위

| 파일 | 변경·완료 기준 |
|---|---|
| `firmware/esp32cam/src/main.cpp`, `source_frame_metadata.h` | boot마다 random 128-bit ID, 드라이버 원본 timestamp 기반 frame ID·source age HTTP 헤더. 같은 버퍼 재응답은 같은 ID. 잘못된 timestamp는 신원 미상으로 표시 |
| `motionbrain_ros_bridge/observation.py` | 헤더 strict 파싱, monotonic 기반 age와 control eligibility 계산 |
| `tools/motionbrain_perception_service.py` | 원본 JPEG·SHA-256·추론·시각을 연결한 캐시와 exact 조회. 중복 추론/신선도 갱신 방지, 충돌·역순·퇴역 boot 거부, held source 분리 |
| `CameraDetection.msg`, `motionbrain_status_node.py` | 원본 ID·시각·age·held·eligibility typed 전파, perception HTTP 소요시간 가산 |
| `control_guard_node.cpp`, `source_observation_freshness.hpp` | steady clock 기반 신선도, 같은 ID 재발행/old boot로 freshness 회복 차단, stale motion suggestion 제거 |
| `tools/motionbrain_dashboard.py` | 영상 기반 정책 후보와 `/api/align_nudge`의 원본 관측 적격성 검사. 수동 light 기능은 유지 |
| `learning/schema.py` | 버전 있는 transition 검증, 원본 보존, schema 오류와 학습 제외 사유 분리 |
| `tests/test_camera_source_native.py`, `tests/native/test_camera_source.cpp` | 생산 C++ 헤더 생성·buffer ID·overflow 경계와 생산 Python 파서 roundtrip |
| `tests/test_observation_contract.py`, `tests/test_perception_service.py` | JPEG/검출 결합·exact HTTP·캐시·시계·held·replay 시험 |
| `tests/test_source_observation_bridge.py`, `tests/test_source_observation_freshness_native.py`, native C++ fixture | typed 전달 및 가상 시간 guard 경계 시험 |
| `tests/test_dashboard_alignment.py`, `tests/test_dashboard_security.py` | stale/headerless/held 정책·정렬 차단, HTTP/상태조회 지연, 기존 light 승인 회귀 |
| `tests/test_action_transition_schema.py` | 합성 transition과 실제 perception payload 형식 연결·제외 사유 시험 |

위 ROS 파일은 `ros2_ws/src/`의 해당 패키지 아래에 있다. 실제 장치 데이터 자동 수집,
host journal→새 `/actions` 연결, 학습·평가 실행은 이번 범위에 포함하지 않았다.

## 카메라와 perception API

`GET /capture`의 JPEG와 함께 아래 헤더를 보낸다.

```text
X-MotionBrain-Camera-Boot-Id: <32 lowercase hex>
X-MotionBrain-Frame-Id: <bootId>:<capturedUs decimal>
X-MotionBrain-Captured-Us: <positive camera boot microseconds>
X-MotionBrain-Source-Age-Us: <header creation time minus capture timestamp>
X-MotionBrain-Clock-Domain: camera_boot_us
X-MotionBrain-Timestamp-Semantics: first_dma_buffer
```

설치된 Arduino ESP32 SDK `esp_camera.h`의 `camera_fb_t.timestamp`는 frame의 첫 DMA buffer 시각이다.
촬영 완료·노출 종료·HTTP 요청 시각으로 바꾸어 부르지 않는다. 원본 timestamp와 같은 boot ID로
frame ID를 구성하며 이미지 hash는 별도 무결성 값이다. 내용이 같아도 새로운 촬영은 다른 ID다.

Perception 조회:

- `GET /api/detection`: 현재 표시/판단 후보와 `sourceFrameId`, `observationId`, `imageSha256`.
- `GET /api/observation?frameId=<sourceFrameId>`: `{ok: true, observation: {...}}`.
- `GET /api/vision_frame?frameId=<sourceFrameId>`: 해당 ID의 정확한 JPEG.
- metadata와 JPEG는 같은 ID로 요청한다. ID가 퇴거됐으면 404, 빈 ID는 400이다.
  ID 없는 기존 vision_frame API는 최신 표시용으로 유지된다.

기본 캐시는 32개이며 JPEG 하나의 상한은 2 MiB다. 동일 ID는 JPEG와 추론을 교체하지 않고
원래 수신시각으로 계속 나이를 계산한다. 동일 ID에 다른 bytes가 오면 기존 evidence의
metadata validity를 내려 충돌을 표시한다. A→B→A의 latest 재생은 제어 부적격이며 저장된
A의 역사적 원본은 유지된다. 새 boot를 관찰하면 이전 boot는 제어용으로 복구하지 않는다.
boot 이력은 최대 128개이고 자동 eviction하지 않는다. 캐시는 영속 저장소가 아니므로 향후
collector가 원본 JPEG와 관측을 퇴거 전에 디스크에 저장해야 한다.

held 좌표의 `detectionSourceFrameId`는 원래 검출 프레임을 가리킨다. 현재 JPEG의 ID와 다르면
현재 이미지에서 검출한 것으로 취급하지 않으며 `controlEligible=false`다. 헤더 없는 구형
firmware도 표시할 수 있지만 원본 시각 미상으로 제어 부적격이다.

source age는 카메라 보고 나이 + 전체 HTTP 소요시간 + 수신 이후 monotonic 경과시간이다.
추론 지연도 포함한다. wall clock은 표시 metadata이며 신선도 갱신 근거가 아니다.
ROS bridge는 추가 HTTP 소요시간을 더하고, guard는 공유 ROS clock의 publication/receipt
차이를 한 번 더한 뒤 steady clock으로 나이를 증가시킨다. 동일 ID의 낮아진 age를 받아도
기존 age 하한은 줄이지 않는다. cleanup도 replay 이력을 지우지 않으며 guard boot 이력은
32개 상한이다. 프로세스 재시작에 걸친 replay 이력 영속성은 아직 제공하지 않는다.

`require_detection=false`의 status-only ready 의미는 유지한다. 다만 stale/display-only
관측으로는 suggestedAction을 만들지 않는다. 메시지 필드가 추가됐으므로 배포 때 전체 ROS
workspace를 함께 빌드해야 하며 기존 generated message와 혼합하지 않는다.
대시보드의 정책 확인 API는 기존 light 전용이다. 모터를 움직이는 영상 기반
`/api/align_nudge`도 원본 시각·source freshness·held·control eligibility를 검사한다.
대시보드는 detection HTTP 왕복과 상태 확인에 걸린 시간을 더해 실행 직전에 재검사하며,
2,000 ms의 소프트웨어 상한을 적용한다. 서비스의 더 짧은 stale 판정도 존중한다.
이 상한은 기존 perception 기본값에 맞춘 것으로 실제 팔의 안전 지연 교정값이 아니다.

## Transition v1 계약

진입점은 `learning.schema.validate_transition(record)`이며 Python 표준 라이브러리만 사용한다.
입력을 변경하지 않고 `valid`, `schemaErrors`, `trainingEligible`, `exclusionReasons`를 반환한다.
완전한 실행 가능한 합성 예시는 `tests/test_action_transition_schema.py::make_transition`에 있다.

| 경로 | 필수 의미 |
|---|---|
| `schemaVersion` | `motionbrain.action_transition.v1` |
| `sessionId`, `episodeId`, `transitionId` | 실험·episode·전이 식별자 |
| `metadata` | robot/config/calibration/firmware/host/detector 식별·버전. config/calibration은 SHA-256 |
| `qualityPolicy` | `maxSourceAgeMs`, `maxM4AgeMs`, `minimumSettlingMs`; 수집 전에 정할 품질 한계 |
| `before`, `after` | exact perception 관측, `image.artifactRef/sha256`, M4 AS5600 실측·sample age·원본 근거·host 시각 구간 |
| `action.request/record` | 정규화 요청과 MCU 수락 기록. command/boot/kind/motor/PWM/duration/target/percent 일치 |
| `action.outputLog` | 같은 boot/ordinal의 `driver_write` trace. sequence 연속·drop 없음·종료 zero write |
| `action.outputSequenceBounds` | 해당 행동의 첫/끝 sequence 및 이를 확보한 근거. 단순 필터링으로 누락을 감추지 않음 |
| `action.startHostBounds/outputOffHostBounds` | 동일 host clock으로 매핑된 시작/종료 구간, method와 evidenceRef |
| `outcome` | `taskSuccess: true/false/null`; 실제 판정이면 labelSource·evidenceRef |

현재 카메라 나이로 만든 `captureHostBounds`는 host와 카메라의 시계 속도가 같다는 가정에
기초한다. 따라서 서비스는 `captureHostBoundsCalibrated=false`,
`captureHostBoundsUncertaintySeconds=null`, `captureHostBoundsEvidenceRef=null`로 보낸다.
이 데이터는 보존 가능하지만 strict 학습에서는 `source_clock_bounds_unverified`로 제외한다.
보정된 외부 근거와 유한한 오차 상한을 제공한 경우에만 구간을 오차만큼 확장하여 검사한다.
실험에서 확인하지 않은 오차 값을 채워 적격으로 바꾸지 않는다.

before의 가장 늦은 촬영이 행동의 가장 이른 시작보다 앞서고, after의 가장 이른 촬영이
가장 늦은 output-off + 정착 대기보다 뒤에 있어야 한다. MCU millis와 카메라 microseconds를
직접 차감하지 않는다. M4도 sample 시각 구간·age·전후 순서를 검사한다. 정착 대기시간은
정책 조건이며 물리적으로 정착했다는 측정 증거의 대체물이 아니다.
기재된 source age가 capture→inference 구간보다 짧으면 제외하며 보정 오차를 포함한
age에 품질 한계를 적용한다. 이 정책은 관측 snapshot의 신선도이고, 실행 직전 재검사는
현재 collector/runner가 별도로 수행하며, `73132e9`에서 교정 오차와 실행 직전 경과시간을 함께 검사하도록 보강했다.

결과 불명·실행 중·출력 종료 미확인·stale·held·같은 before/after·frame 연결 오류·출력 누락은
학습에서 제외한다. 타당한 출력/관측이 있는 취소·실패·timeout은 동역학 학습 후보가 될 수 있다.
`targetReached`, `outputOff`, `driver_write`를 taskSuccess의 판정 출처로 허용하지 않는다.
검증기는 기록된 주장 간의 일관성을 검사한다. JPEG bytes를 읽어 hash를 검증하거나
물리 정지·집기 성공을 측정하지 않는다. artifact 저장·hash 검증은 후속 dry-run collector와 manifest에 구현했다. 실제 장치 adapter는 남아 있다.

## 검증과 한계

재현 명령 (장치/네트워크 불필요):

```sh
python3 -m unittest discover -s tests -q
pio run -d firmware/esp32cam
git diff --check
```

- 전체 Python/native suite: **288 tests, OK** (최종 실행 7.869초).
- 후속 커밋 전 독립 리뷰에서 조기 완료 pulse와 종료 상태·사유 모순이 학습 적격으로
  통과하는 결함을 수정했다. 실제 MCU 종료 규칙과 일치시키고 회귀 시험을 추가한 뒤
  **291 tests, OK** (8.960초)를 확인했다. 관측/카메라/ROS/대시보드 리뷰에서
  추가 확정 P1/P2 문제는 발견하지 못했다.
- 변경 Python 파일의 `py_compile` 및 `git diff --check` 통과.
- ESP32-CAM firmware compile 성공: RAM 51,836 bytes (15.8%), flash 903,401 bytes (28.7%). 업로드 없음.
- producer 독립 리뷰: SDK timestamp 의미, 정수 범위, header/parser 일치, framebuffer 수명 확인.
- ROS bridge는 fake adapter로, guard는 실제 production C++ helper를 native compile하여 시험했다.
  ROS Jazzy 전체 빌드와 lifecycle runtime 시험은 이 Mac 환경에서 실행하지 않았다.
- 합성 시각·출력·JPEG로 계약을 검증했으며 실제 데이터 수집 또는 조작 성능 개선 증거가 아니다.

## 다음 장치 세션과 구현 순서

1. 장치 없이 구현한 S4b dry-run 수집·원본 저장/hash·중단/누락 기록과 S5a manifest를 바탕으로
   S5b loader·특징/기준선·train-only 전처리·평가 코드를 준비한다. 실제 수집 adapter는 별도 구현한다. 기존 policy episode 로그를 전이로 재해석하지 않는다.
2. 장비 연결 가능 시: 모터 전원을 분리하고 ESP32-CAM 보드·현재 firmware·포트를 확인한 뒤
   camera firmware를 업로드한다. `/capture` 반복에서 ID/age, 재부팅 boot 변경, Wi-Fi 지연,
   오래된 buffer와 같은 ID의 bytes 일치, exact API의 JPEG hash를 확인한다.
3. Pi에서 ROS 전체 빌드 후 원본 필드 전파, 지연·재발행·held 때 guard 차단을 확인한다.
   먼저 모터 전원 분리 상태에서 수행한다.
4. MCU/카메라/host clock mapping의 불확실성과 M4 sample age를 기록할 방법을 교정한다.
   원시 로그를 남기고 timing 구간이 겹치는 전이는 제외한다. 교정값은 실제 관측 근거와 함께 버전 관리한다.
5. 이후 한 축의 제한된 작은 동작으로 output-off·영상 정착·새 촬영·M4 sample 연결을 검증한다.
   STM32 heartbeat, M4 AS5600 freshness, 팔 고정 자세, 즉시 전원 차단 수단과 카메라 시야가 필요하다.
   기존 [MCU 실물 관문](2026-10-14-bounded-actions-software.md)을 먼저 충족해야 한다.

추가 센서나 클라우드 GPU는 이 단계의 선행조건이 아니다.
