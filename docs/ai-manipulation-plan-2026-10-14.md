# MotionBrain 시각 기반 조작 학습 실행 계획

작성: 2026-10-14 KST. 목표: 2026년 11월 AI 분야 지원에 사용할, **학습이 제한된 실제 조작 성능을 개선했다는 재현 가능한 증거**. 일정은 작업 가능 시간과 실물 수집 결과에 따른 목표이며 완료 약속이 아니다.

## 1. 현재 상태와 인계 대조

- 시작 HEAD: `99ea0d6` (`Update portfolio with physical M4 ros2_control evidence`), 로컬 `main...origin/main`, tracked/untracked 변경 없음. 원격 fetch 및 현재 Pi/보드 상태 확인은 하지 않았다.
- 저장소와 상위 경로에서 `AGENTS.md` 없음. `.codex/START_HERE.md`, `.codex/README.md`, `.claude/CLAUDE.md` 참고. 로컬 메모의 HEAD `be5c521`와 7월 장치 연결 상태는 과거 기록이며 현재 사실로 사용하지 않는다. `.codex/`는 Git 제외이므로 이 계획은 `docs/`에 보관한다.
- 외부 인계 `MotionBrain_AI_개발인계_2026-10-14.txt` 314행과 `MotionBrain_개발·개선계획_2026-09-23.md` 전체를 읽었다. 인계 이후 코드 변화는 없었다. 명령 경계, 펌웨어, 관찰/학습 전문 에이전트가 나눠 확인하고 교차 검토했다.
- 같은 proposal 재접수 뒤 confirm하면 POST가 2회 발생하는 결함과 POST 응답 유실을 `forwarded=false`로 표시하는 결함을 fake callback으로 재현했다. C++ ID는 프로세스 내 순번뿐이고 ROS `created_at`은 Python core로 전달되지 않았다.
- 카메라는 이중 버퍼/`CAMERA_GRAB_LATEST`를 사용할 수 있다. `/capture`를 행동 종료 후 요청했다고 종료 후 **촬영**을 보장하지 않는다. 인식 완료 `ts`와 재발행 stamp는 원본 획득시각이 아니다.
- 텔레옵은 Dispatcher 밖에서 출력한다. 시퀀스는 일반 stop 뒤 곧바로 다음 단계를 시작하므로 ramp-down 중 축 출력이 겹칠 수 있다. 기존 시퀀스를 그대로 학습용 단일 행동 실행기로 쓰지 않는다.
- 환경 차이: 현재 Python 3.12.11에서 기존 185개 테스트가 수집되었고 OpenCV import 오류는 없다. 최초 sandbox 실행은 176개 통과, localhost bind 제한 오류 9개였다. 최종 검증은 별도 증거 문서에 기록한다.

재사용: 기존 M4 AS5600 목표각 제어/보정/정착/오류 판정, SafetyGate, 수동 입력, 객체·색상 검출, typed ROS/diagnostics/lifecycle, fake endpoint, CI/systemd, URDF/FK/IK. 기존 `policy_proposal`은 규칙 정책이며 `cup_grasp_plan`은 dry-run, 조명 mission 완료는 조작 성공이 아니다. M4 외 JointState 기본값·추정값은 실측 정답으로 쓰지 않는다.

9월 계획에서 M0-01~05와 선택 경로의 시간/출력 검증을 이어받는다. 전체 MCU 정비 완료를 AI 착수 조건으로 삼지 않는다. 비활성 진동 정책, RTOS/DMA 전환, 두 번째 축 센서, 전 축 ros2_control/MoveIt, 대형 VLA 학습은 선행조건에서 제외한다. 기존 카메라·M4·실제 실행 이력, Mac CPU와 필요할 때만 소액 클라우드로 시작한다.

### 이번 소프트웨어 후속 진행

- S1 host journal/ROS adapter/producer ID는 독립 리뷰 후 `f581253`으로 커밋했다.
- S2의 MCU boot/action registry, 단일 제한 행동, 실제 드라이버 출력 기록, 수동/teleop 소유권,
  신규 API 숫자 검사와 공통 M4 finite 검사를 구현했다. 기본 실행 비활성 상태다.
- 새 API는 기존 `/shoulder` 호스트 실행과 별개이며 새 runner의 capability/boot 기반 연결은 남아 있다.
- S2 MCU 변경은 독립 코드리뷰와 native 회귀 후 `1bcd59e`로 커밋했다.
- S3a/S3b/S4a의 원본 프레임 식별, exact JPEG/관측 조회, ROS 신선도 전달과
  행동 전후 schema 검증을 구현했다. 카메라 시계 오차의 실물 근거가 없는 데이터는 학습에서 제외한다.
- 관측/schema는 독립 리뷰 수정과 291개 테스트 후 `ca48f04`로 커밋했다.
- S4b의 dry-run runner·원본 이미지/hash 저장·중단/누락 감사는 구현했다.
  [331개 소프트웨어 테스트와 CLI 10종 결과](evidence/2026-10-14-dry-run-collector-software.md)를 남겼다.
  독립 리뷰·전체 회귀 후 `9ad4935`로 커밋했다.
- S5a manifest·그룹 분할·누수/합성 혼입 검증은 장치 없이 구현했다.
  [데이터셋 검증 기록](evidence/2026-10-14-dataset-manifest-software.md)에 정책과 제한을 남겼다.
  현재 11개 합성 시도는 모두 제외되며 실제 학습·평가 데이터와 모델은 아직 없다.
- 활성 UART seq/session 검증, 기존 모든 숫자 입력 경로 통일, S4b의 실제 장치 adapter와
  시계·출력·관측 연결 검증은 남아 있다.
- 현재 장치와 Raspberry Pi는 연결할 수 없다. 소프트웨어 235 tests와 ESP32 빌드가 통과했으며,
  [구현·검증·실물 관문 기록](evidence/2026-10-14-bounded-actions-software.md)에 분리해 남겼다.
- 후속 관측/schema 결과와 파일별 완료 범위는
  [원본 관측·transition 소프트웨어 검증](evidence/2026-10-14-source-observations-software.md)에 기록했다.

## 2. 공통 계약과 보장 범위

### 명령과 결과

- ID 계층: host producer session + sequence → action ID, 실험 session/episode/transition ID, MCU boot ID, camera boot/frame ID를 별도로 기록한다. producer session은 MCU boot나 실험 session의 대체물이 아니다.
- 같은 action ID/같은 내용 재전달은 최초 기록·TTL·소비 상태 유지, 다른 내용은 충돌 거부. 전송 전에 영속 예약하고 자동 재전송하지 않는다.
- `not_sent`: 이 호스트가 해당 명령을 전송하지 않았음을 아는 경우. `accepted`: MCU 수락 응답. `running`: 장치 실행 확인. `output_off`: 드라이버 출력 0 확인. `canceled`/`failed`: 근거 있는 종료. `outcome_unknown`: 통신/기록/재시작 때문에 결과 확인 불가.
- `target_reached`, `visually_settled`, `task_success`는 별도 근거 필드다. HTTP 성공/close 명령/PWM 종료만으로 도달·파지·작업 성공을 선언하지 않는다.
- 초기 호스트 journal은 최대 10,000개 기록, 자동 삭제 없음. 가득 차면 새 접수를 거부한다. 오래된 ID tombstone을 지워 replay를 허용하는 TTL eviction은 하지 않는다. 장치 boot/session 계약과 감사 보존 절차가 마련되기 전 DB 삭제/새 경로로 우회하지 않는다.
- 호스트 journal만으로 MCU의 물리 exactly-once, 모든 클라이언트 간 배타성, 지정 timeout을 보장하지 않는다. 다른 journal·직접 HTTP 경로는 보호 범위 밖이다. MCU command ID/deadline/result endpoint 통합이 별도 필수다.

### 관찰과 데이터

- 원본 JPEG, camera boot/frame ID, detection source frame ID, capture clock domain/time/age/불확실성, host 수신·추론 시각을 불변 관측으로 묶는다. held 좌표는 원래 detection ID를 유지하며 새 프레임 검출로 취급하지 않는다.
- 다른 clock domain의 시각을 직접 차감하지 않는다. MCU 출력 종료와 카메라 획득의 관계는 동기화 오차 또는 source-age/왕복 지연으로 계산한 시간 구간으로 검증한다. 겹치면 after 적합 판정 보류. host 수신 시각만으로 after 판정 금지.
- hash는 이미지 무결성/누수 확인용이다. 동일 hash가 새 촬영일 수도 있으므로 hash만으로 촬영 ID나 신선도를 증명하지 않는다.
- 모든 실패·불명·held·stale·로그 손실을 원시 데이터에 남긴다. `task_outcome`과 데이터 품질은 분리한다. 유효한 실제 출력과 전후 관찰이 있는 목표 미도달·집기 실패는 dynamics 학습 후보이며 성공 사례만 골라 학습하지 않는다. 결과 불명·잘못 연결된 프레임·held·stale·로그 손실은 기본 `training_eligible=false`와 제외 사유를 기록하고, 모델/태스크별 적합 규칙을 버전 관리한다. 실패 시도를 평가 분모에서 삭제하지 않는다.

## 3. 파일별 실행 순서와 완료 기준

각 단계는 작은 PR/변경 묶음 단위다. S3·S4의 오프라인 작업은 S2 장치 검증과 병행할 수 있다.

| 순서 | 수정·추가 파일 | 구체 작업 | 소프트웨어 완료 기준 | 별도 실물 관문 |
|---|---|---|---|---|
| S1a 이번 구현 | `ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/m4_write_executor_core.py`, 신규 `m4_command_journal.py`, `tests/test_m4_write_executor_core.py` | SQLite 기록, 동일 ID 재접수/충돌, monotonic TTL, 원자 예약, 응답 유실 분리, 재시작 복구, 결과 조회, 기록 상한 | fake clock/POST로 재전달·동시 confirm·만료·충돌·fetch 실패·POST 유실·기록 오류·재시작 시험; ID당 POST 최대 1회 | 아직 MCU의 실제 중복 억제 증거 아님 |
| S1b 이번 구현 | `m4_write_executor_node.py`, `motionbrain_hardware_interface.hpp/.cpp`, 신규 표준 C++ ID helper 및 테스트 | 영속 journal 기본값, read-only 결과 서비스, 원 created_at 보존, 랜덤 producer session ID | Python adapter fake ROS 시험, native C++ ID 생성/실패 시험, 기존 호스트 회귀 | ROS Jazzy 전체 빌드/launch와 재시작 시험 필요 |
| S2a 다음 구현 | 신규 `src/control/numeric_validation.h`, `src/control/shoulder_angle_controller.cpp`, `src/input/serial_command.cpp`, `src/network/web_server.cpp` | 전체 숫자 소비/finite/overflow/범위 검사, 공통 start 경계 재검사 | NaN/Inf/부분 숫자/과대값 거부 및 fake 출력 0회; ESP32 compile | 모터 전원 분리 상태에서 입력 거부 확인 |
| S2b | `src/control/command.h`, 신규 `action_registry.h/.cpp`, `src/network/web_server.*` | boot/action ID, 정규화 요청, 같은 ID 재조회/충돌, 제한 결과 ring/상한 정책, capability version | boot mismatch 거부, 결과 유실은 unknown, 만료 ID 재실행 금지 | 응답 유실/재부팅/중복 수신 후 드라이버 로그 확인 |
| S2c | 신규 `src/control/bounded_action_executor.h/.cpp`, `dispatcher.*`, `safety_gate.*`, `src/main.cpp` | 한 축·한 행동, busy, MCU deadline/취소, M4 기존 제어기 위임, 종료 후 output_off 대기 | 가짜 clock·출력으로 timeout/cancel/safety/시간 wrap, 만료 뒤 재출력 없음 | 한 축 최소 행동과 종료 지연 측정 |
| S2d | 신규 `src/motor/applied_output_log.h/.cpp`, `motor_driver.*`, `teleop_adapter.*`, `motion_sequence.*`, `dispatcher.*`, `web_server.*` | 공통 드라이버 write/강제 off 감사, 제어권 lease, 수동 개입 취소, 이전 sequence/M4/teleop cache 정리 | ramp·방향·off 기록, overflow/drop 감지, override 후 이전 출력 재개 없음 | HTTP 부하/단절/deadman 중 off/잔여 이동 확인 |
| S2e | `src/bridge/stm32_bridge.*`, `src/input/teleop_adapter.*`, `src/main.cpp` | 활성 UART seq/session 검증, reject freshness 갱신 금지, loop 간격/deadline overrun 계측 | 중복·역순·재부팅·wrap 입력, stale 반복으로 재무장 불가 | STM32 heartbeat 유지 및 stale 차단 시험 |
| S3a | `firmware/esp32cam/src/main.cpp`, `tools/motionbrain_perception_service.py`, `tests/test_perception_service.py` | camera boot/source timestamp 의미 확인, 원본 frame identity, 불변 JPEG+검출 캐시/ID 조회 | A metadata에 B JPEG 혼합 불가, 만료 ID 404/410, held 별도 source, wall clock 점프 시험 | 같은 버퍼 반복/카메라 재시작/획득 지연 관찰 |
| S3b | `ros2_ws/src/motionbrain_msgs/msg/CameraDetection.msg`, `motionbrain_status_node.py`, `ros2_ws/src/motionbrain_control/src/control_guard_node.cpp`, 기존 guard 테스트 | source ID/held/validity/age 전파, 재발행으로 freshness 회복 금지 | 오래된 ID 연속 수신해도 stale, metadata 미상은 제어 거부 | 느린 2초 polling에서 stop-observe 루프 확인 |
| S4a | 신규 `learning/schema.py`, `tests/test_action_transition_schema.py` | observation/action/transition validator, 버전·품질·판정 출처 | 잘못 연결된 frame·동일 before/after·unknown·stale M4·NaN·missing metadata 격리 | 합성 fixture 성공은 데이터 수집 성공 아님 |
| S4b | 신규 `learning/runner.py` 단일-step 골격, `tools/capture_action_transitions.py`, `tests/test_action_transition_capture.py`; 기존 두 policy episode 도구는 유지 | 공통 action/observation adapter로 runner의 before/실제 행동/after 연결, append-only 원본 로그, 이미지 hash와 설정 snapshot | 취소/관측 소실/기록 손실까지 기록, 종료 후 획득 증거 없으면 부적합 | 한 물체/정해진 자세에서 작은 교정 데이터 수집 |
| S5a 구현 | 신규 `learning/dataset.py`, `learning/splits.py`, `learning/provenance.py`, `tools/build_action_dataset.py`, 대응 테스트 | 원본 무결성·raw/요약 연결·출처 재검증, 모든 시도 inventory, 세션/episode/hash/frame/행동/배치 그룹 분할 | 수동 manifest 변조·누락·합성 혼입·교차 분할 거부, 합성 11개 적격 0 | 실제 수집 adapter와 교정 근거 연결 후 별도 세션 실물 데이터 적용 |
| S5b | 신규 `features.py`, `models.py`, `tools/train_visual_dynamics.py`, `evaluate_visual_dynamics.py`, `evaluate_robot_trials.py` | 무변화·선형·작은 MLP 비교, 그룹 분할, artifact 저장/재현 | Mac CPU 합성 smoke, train-only 정규화, episode/배치/frame 누수 검사 | 별도 세션 실제 접근·집기·배치 trials |
| S6 | S4b의 공유 `learning/runner.py` 확장, 신규 `tools/run_manipulation_trial.py`; `tools/motionbrain_dashboard.py`와 ROS adapter | 관찰→선택→제한 실행→off→정착→재관찰 단일 루프, shadow/작업자 한 단계/감독 반복 모드 | restart 기본 비구동, goal 소실·범위 밖·unknown 중단, 기존 confirm 자동 위조 없음 | 접근부터 검증 후 집기·들기·이동·놓기 순차 확장 |
| S7 | `docs/evidence/`, `PORTFOLIO.md`, `README.md`, `claim-to-evidence-matrix.md` | 조건/버전/원시 로그/영상/실패 표 연결 | 코드·합성·저장된 실물·새 실물 증거 분리 | 학습 개선이 입증된 범위만 발표 |

S2c의 종료 정책은 API에 명시한다. 초기 후보는 기존 hardStop을 재사용한 deadline 시 출력 차단이며, ramp 종료가 필요하면 최대 drain deadline과 output_off 확인까지 lease를 유지한다. 관절별 최소 PWM/시간/정착값은 실측 전 확정하지 않는다. applied_pwm는 `ledcWrite`에 전달한 명령값(`evidence=driver_write`)이며 핀 파형이나 실측 RPM이 아니다.

`motionbrain_status_node.py`의 제어 상태 polling은 camera/routine/events와 분리하고 요청 deadline/queue 상한을 S3~S6 통합 전에 둔다. 카메라 지연 때문에 guard/명령 판단이 오래된 로봇 상태를 쓰지 않는 장애 주입 테스트가 완료 기준이다. 전체 ROS 운영 도구 재패키징은 선행조건이 아니다.

## 4. S1 운영 계약

- ROS node의 기본 journal: `$XDG_STATE_HOME/motionbrain/m4_commands.sqlite3`; 미설정·빈 값·상대 XDG 경로이면 `~/.local/state/motionbrain/m4_commands.sqlite3`. 명시 journal 경로는 절대 경로(`~` 확장 허용)만 받는다. `command_journal_path`, `command_journal_max_records` 파라미터 지원. node는 `:memory:`를 거부한다. core의 메모리 모드는 단위 테스트용이다.
- 같은 파일은 POSIX flock으로 단일 executor만 소유한다. 다른 계정/DB 경로로 여러 physical executor를 띄우면 보호되지 않는다. 배포에서 하나의 장치와 영속 경로를 고정한다. 파일 접근 실패/손상/가득 참은 자동 초기화나 fallback 없이 오류로 남긴다.
- pending/validating 중 호스트 재시작은 `canceled`; dispatch 예약 뒤 재시작은 `outcome_unknown`. 이미 MCU 수락 응답을 저장했으면 `accepted` 기록을 유지하지만 현재 모터 상태를 의미하지 않는다.
- 결과 불명 기록이 있으면 새 confirm도 차단한다. 이번 슬라이스에는 unknown 해제/재시도 API가 없다. MCU 결과 조회/boot 계약을 구현하고 작업자 확인에 근거한 reconciliation을 추가해야 한다. 기록 삭제로 우회하지 않는다.
- `/motionbrain/m4_write_result`는 기존 `motionbrain_msgs/srv/M4WriteConfirm` wire shape를 재사용하는 **읽기 전용** 서비스다. 명령 ID만 받아 journal을 읽는다. `/motionbrain/m4_write_confirm`은 명시적 작업자 승인 경로로 유지한다.
- `raw_json`의 `executionState`, `dispatchAttempted`, `acknowledged`, `ledgerState`, `proposalMetadata`를 판정에 사용한다. `success=true`는 MCU 수락 응답, `acknowledged=true`는 유효한 수락/거부 응답, `forwarded=true`는 전송 시도 또는 crash 전에 시도했을 가능성이다. bool 하나로 물리 실행 여부를 판단하지 않는다. 결과 조회의 미존재는 `result_not_found/outcome_unknown`이며 미실행 증거가 아니다.
- 원 `created_at`은 ROS clock metadata로 보존만 한다. TTL은 최초 호스트 수신 monotonic 기준이다. 아직 처음 보는 오래된 proposal의 source-age를 보장하지 못하며 새 producer/실험 session handshake는 S2에 남아 있다.
- 현재 `/shoulder`는 command ID와 요청별 timeout을 지원하지 않는다. 무시되는 query 필드를 붙여 해결됐다고 주장하지 않는다. `timeoutMs`는 호스트 검증/fingerprint에만 반영되는 기존 제한이며 MCU 동작 deadline은 S2에서 연결한다. S1만으로 학습 행동 실험을 시작하지 않는다.

## 5. 행동 데이터와 평가 설계

transition 최소 필드: schema/session/episode/transition/action ID; robot/config/calibration/firmware/host/detector/model 버전·hash; before/after frame ID·JPEG·hash·검출 source·clock/age/불확실성·held/validity; M4 실측·sample age; 요청 행동; 실제 signed PWM trace·시작/종료/취소 사유·elapsed·drop count; output_off/정착 근거; 단계별 작업 결과(success/failure/unknown)·판정자/영상; training_eligible·exclusion reasons.

예측 입력은 최근 유효 영상 특징(정규화 중심·크기), M4 실측/age, 과거 실제 행동, **이번에 실행 전에 아는 후보 명령/설정**, 경과시간이다. 이번 행동 실행 후 알게 되는 실제 PWM trace를 사전 예측 입력에 넣지 않는다. 다음 영상 특징 변화가 우선 정답이며 전체 팔 각도/깊이를 안다고 주장하지 않는다.

1. 기존 480장 vision 데이터는 검출/시야 진단용이며 action transition으로 변환하지 않는다. 새 전후 행동 데이터를 수집한다.
2. Mac CPU에서 무변화 예측, 선형 회귀, 작은 PyTorch MLP 순서. 추가 모델/유료 GPU는 실패 분석에서 필요성이 확인될 때만 고려한다.
3. session/episode/시작 배치 그룹으로 train/validation/test manifest를 먼저 고정한다. 인접 프레임 무작위 분할 금지, 동일 raw frame hash 교차 분할 검사, 전처리 통계는 train만 사용한다.
4. 무변화/선형/MLP의 예측 MAE와 축별/행동별 오차를 별도 평가한다. 예측 개선은 물리 제어 개선과 구분한다.
5. 물리 정책 비교: 고정 행동 정책, 단순 선형/영상 오차 보정, 학습 모델을 같은 행동 집합·시작 배치·최대 step/time·중단 규칙으로 비교한다. 순서를 섞고 별도 세션에서 시행한다.
6. 시범 목표: 정책당 10개 시작 배치 × 3회 = 30 trials. 가용 시간/교정 결과에 따라 수집 전 프로토콜에서 확정한다. 적은 반복이면 그 한계를 명시하고 우월성을 단정하지 않는다.
7. 접근 오차·도달률, 집기/들기/배치 성공률, 전체 성공률, 시간, action 수, 개입 수, 실패/unknown 이유와 전체 분모를 보고한다. 이항 성공률 구간과 배치 단위 비교도 함께 제시한다. 학습 개선이 없으면 실패 원인·ablation 결과를 정직하게 남긴다.
8. held bbox/화면 중앙/close/PWM off는 파지 성공이 아니다. 물체 들림과 지정 영역에 내려놓은 결과를 영상 또는 작업자 판정으로 남긴다. 현재 카메라 시야에서 확인이 안 되면 기존 카메라 위치/방향 조정 및 판정가능 범위부터 정한다.

## 6. 일정과 범위 축소 기준

- 10/14~10/18 목표: S1 완료, S2 순수 코어/관찰 schema와 synthetic 시험, 장비 준비 목록 확정.
- 10/19~10/30 교육 기간: 가능한 시간에 S2/S3 통합과 첫 작은 행동 데이터 수집. 오프라인 S4/S5 병행. 교육 완료를 구현 시작 조건으로 삼지 않는다.
- 10월 말~11월 초 목표: 접근 성능 기준선과 학습 비교를 먼저 확보하고, 관측 가능한 집기·들기·배치를 연결한다.
- 11월 지원: 동작 영상, 설정/분할 manifest, 재현 명령, 비교표, 실패 사례, 판단 가능한 증거를 묶는다. 전체 작업이 미완료면 검증한 접근 개선과 남은 단계를 명확히 표시한다.
- 축소 순서: 객체 종류·배치 다양성·UI 장식·자연어 인터페이스부터 줄인다. 정확한 action/frame 연결, 결과 불명 처리, 비교 평가를 생략하지 않는다. 접근도 개선되지 않으면 단순 객체 인식을 조작 학습 성공으로 대체하지 않는다.

## 7. 다음 실물 세션 준비와 실험

현재 장치 상태는 미확인이다. 이 계획 자체는 업로드/모터 구동 승인이 아니다. 해당 단계에서 구체 범위와 준비 상태를 사용자에게 확인한다.

필요 상태: ESP32/ESP32-CAM 포트와 현재 firmware 식별, STM32 안전 heartbeat 활성/imuOk, M4 AS5600 connected/fresh/ready, 모터 전원 분리와 즉시 차단 가능, 케이블 간섭 없는 고정 시작 자세, 카메라 장착·집게/물체/배치 영역 시야. 추가 센서 구매는 요구하지 않는다.

1. 모터 전원 분리 상태: 대상 보드만 업로드, 부팅/재부팅 기본 비구동, ID 중복·충돌·boot mismatch·불명 결과 조회·UART stale 확인.
2. 저위험 고정 자세: 한 축/한 방향씩 최소 실제 움직임 PWM·시간과 ramp를 교정. M4는 과거 검증 자세·하중·범위부터 회귀한다. 임시 122.08~301.02°를 전 자세 충돌 안전 범위로 쓰지 않는다.
3. 종료 계측: 요청 종료, 드라이버 PWM off 로그, 영상 정착, M4 잔여각을 구분. 기존 장비로 핀 파형을 측정하지 못하면 소프트웨어 로그 한계를 명시한다.
4. 통신 부하/호스트 단절/수동 개입/deadman/heartbeat stale를 각각 시험해 deadline 초과와 취소 후 재개 여부 확인. 실제 off 지연 상한은 이 교정 결과로 실행 프로필에 고정한다.
5. 카메라 버퍼·재시작·반복 프레임으로 after 연결 검증 후 작은 전후 행동 데이터 수집. 객체 한 개·작업 영역 한 개·정해진 시작 자세로 제한한다.

소프트웨어 pass는 물리 정지·무충돌·파지 성공 보장이 아니다. 과거 M4 실물 결과는 기존 evidence를 인용하며 새 실험 결과와 섞지 않는다.
