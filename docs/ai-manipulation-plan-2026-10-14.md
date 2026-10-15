# MotionBrain 시각 기반 조작 학습 실행 계획

작성: 2026-10-14 KST. 목표: 2026년 11월 AI 분야 지원에 사용할, **학습이 제한된 실제 조작 성능을 개선했다는 재현 가능한 증거**. 일정은 작업 가능 시간과 실물 수집 결과에 따른 목표이며 완료 약속이 아니다.

## 1. 현재 상태와 인계 대조

소프트웨어 구현 정리: 2026-10-14 KST, 구현 기준 커밋 `73132e9`. **데이터 수집·검증 기반의 소프트웨어 구현은 아래 범위까지 완료했지만, 실제 학습 준비가 모두 끝난 것은 아니다.** 10월 14일에는 장치를 연결할 수 없어 새 실물 검증을 수행하지 않았다.

2026-10-15 배포 후속: Pi에 `559db54` 배포·ROS 전체 빌드, Mac/Pi 각각 372개 회귀와 별도 ROS 72개 테스트, ESP32·CAM 업로드를 완료했다. 초기 저전압 재부팅으로 한때 보류했으나 사용자의 추가 재연결 후 정상 부팅·STM32·AS5600 복귀, 기본 비활성 API 거부·출력 0, CAM 원본/hash·stale 차단과 ROS 읽기 전용 결과 조회를 확인했다. 63.5초/13회 관찰에서 메인 ESP32 boot ID와 출력 0을 유지했다. 모터 구동·활성 동작 중복 억제·물리 정지·시간 교정은 미검증이며 전원 문제의 원인 분리는 남아 있다. 상세 성공·실패 범위와 다음 순서는 [10월 15일 배포 기록](evidence/2026-10-15-deployment-no-drive.md)을 우선 참고한다. 아래 10월 14일 단계별 기록은 당시 결과다.

10월 15일 추가 상태: 이후 CAM에서 간헐적인 Wi-Fi 송신 드라이버 경로 panic 및 지속 무응답을 재현했다. 주소·포트 변경 문제는 아니며, 단기 통과와 별개로 카메라 안정화가 실제 수집 전 과제로 남는다. [원인 진단·실물 로그·이전 코드 비교](evidence/2026-10-15-camera-connectivity.md) 참조.

10월 15일 장시간 후속: 카메라 위치 고정 후 변경 전후 각각 10분, 현재 버전 복원·Pi 재개 후 5분 동안 시험 요청 1,300건을 모두 처리했다. 원본 hash·누락·stale 차단도 재확인했다. 최초 부팅 watchdog 1회는 별도 기록했으며 최근 변경과의 인과 및 기존 간헐 장애의 근본 원인은 미확정이다. 제품 펌웨어 소스는 변경하지 않았다. [장시간 비교·복원 결과](evidence/2026-10-15-camera-long-comparison.md) 참조.

10월 15일 watchdog 후속: 30회 반복 부팅에서 watchdog은 재현되지 않아 근본 원인은 미확정이다. 별도로 확인한 CAM 호스트명 설정 순서와 750ms/750초 timeout 단위 오류를 수정·업로드했다. 수정 진단에서 실제 이름·소켓 설정값을 확인했고 최종 제품 5회 부팅, 2분 영상 통합 104/104 요청, 원본 계약 및 전체 소프트웨어 372개 회귀를 통과했다. 모터 명령은 보내지 않았다. 수정 전 이미지 요청 2건 실패와 미검증 전원 조건은 [watchdog 후속 기록](evidence/2026-10-15-camera-boot-watchdog.md)에 별도 보존했다. 사용자 요청으로 연결 문제 조사는 종료했으며 연결 원인으로 확정한 것은 아니다. 커밋 전 코드리뷰와 372개 회귀 재실행(8.727초), CAM 빌드도 통과했다.

최신 비구동 수집 후속(10월 15일): 실제 장치 adapter의 **읽기 경로**를 추가했다. Pi에서 GET 66회·관측 12개·원본 JPEG 24개 저장/hash 검증, 시작/종료 출력 로그 동일, 모터 명령 0회를 확인했다. 비구동 구현 당시 398개, 커밋 전 명령 조회 실패 판정 수정 후 **399개 통과(13.754초)**다. 카메라 시간 표본은 단기 offset 가정과 일치하지만 물리 교정 완료는 아니며, 영상 나이 3.5~7.4초와 일부 M4 age>20ms가 수집 정책을 초과했다. 실제 명령 송신·host journal 연결·학습용 행동 수집은 여전히 미완료다. [구현·실물 결과·다음 순서](evidence/2026-10-15-device-collection-probe.md)를 우선 참고한다. 아래 단계별 372개 기록은 이전 기준이다.

후속 착수: 집게 근처의 전방 카메라 장착을 사용자 사진으로 확인했으며 집게가 영상에 보이는 것을 요구하지 않는다. 기존 객체 모델의 Pi 설정 비교에서 640/1스레드 중앙값 2.87초, 640/2스레드 1.99초, 320 입력은 현재 모델 shape 오류였다. 운영 설정은 유지했고 다음 구현을 대상 선택과 최신 ROI 특징 측정의 분리로 좁혔다. [측정과 다음 구현 계약](evidence/2026-10-15-fast-observation-baseline.md) 참조.

최신 ROI 구현: 선택 기준 exact frame/JPEG/hash와 현재 측정을 분리한 고정 템플릿 경로·읽기 전용 probe를 추가했다. **424개 테스트 통과(12.319초)**. Pi 최종 정지 장면 12개에서 영역 일치는 확인했지만 원본 나이·전송·기록·계산 포함 146~191ms여서 전부 100ms 기준으로 거부했다. 원본 13개와 artifact 60개를 보존·재검증했고 출력 로그 동일·모터 명령 0회다. 다음은 비구동 지연 개선, 컵 이동·가림/모호성의 실물 검증, 시계/M4 구간 교정 순서다. 제어 연결·실제 행동 학습 데이터는 아직 없다. [구현·검증·완료 기준](evidence/2026-10-15-fast-roi-observation.md) 참조.

최신 지연 개선: ROI 초기 구현 `d081281`을 독립 리뷰 후 `feat/fast-observation`에 커밋·푸시했다. 후속은 동일한 전체 장면 후보 검사에서 좌표 배열 생성을 제거하고, 원본·receipt 기록을 계산 뒤로 옮겨 계산 시점과 저장 시점의 나이를 분리했다. **429개 테스트 통과(13.686초)·독립 리뷰 완료**. Pi 비교에서 계산 완료 나이 중앙값 170.66→138.11ms, 최적화 12개 모두 100ms 기준으로 거부했다. 원본·측정 저장 후 나이 중앙값은 181.15ms다. 모터 명령 0회이며 main 병합은 별도 승인 대상이다. 다음은 카메라 촬영·전송·시계 구간 분리 측정과 실제 컵 이동·가림 검증이다. [전후 비교·남은 한계](evidence/2026-10-15-fast-roi-latency.md) 참조.

### 작업 시작 당시 인계 대조 (과거 기준)

- 시작 HEAD: `99ea0d6` (`Update portfolio with physical M4 ros2_control evidence`), 로컬 `main...origin/main`, tracked/untracked 변경 없음. 원격 fetch 및 현재 Pi/보드 상태 확인은 하지 않았다.
- 저장소와 상위 경로에서 `AGENTS.md` 없음. `.codex/START_HERE.md`, `.codex/README.md`, `.claude/CLAUDE.md` 참고. 로컬 메모의 HEAD `be5c521`와 7월 장치 연결 상태는 과거 기록이며 현재 사실로 사용하지 않는다. `.codex/`는 Git 제외이므로 이 계획은 `docs/`에 보관한다.
- 외부 인계 `MotionBrain_AI_개발인계_2026-10-14.txt` 314행과 `MotionBrain_개발·개선계획_2026-09-23.md` 전체를 읽었다. 작업 시작 당시 인계 이후 코드 변화는 없었다. 명령 경계, 펌웨어, 관찰/학습 전문 에이전트가 나눠 확인하고 교차 검토했다.
- 같은 proposal 재접수 뒤 confirm하면 POST가 2회 발생하는 결함과 POST 응답 유실을 `forwarded=false`로 표시하는 결함을 fake callback으로 재현했다. C++ ID는 프로세스 내 순번뿐이고 ROS `created_at`은 Python core로 전달되지 않았다.
- 카메라는 이중 버퍼/`CAMERA_GRAB_LATEST`를 사용할 수 있다. `/capture`를 행동 종료 후 요청했다고 종료 후 **촬영**을 보장하지 않는다. 인식 완료 `ts`와 재발행 stamp는 원본 획득시각이 아니다.
- 텔레옵은 Dispatcher 밖에서 출력한다. 시퀀스는 일반 stop 뒤 곧바로 다음 단계를 시작하므로 ramp-down 중 축 출력이 겹칠 수 있다. 기존 시퀀스를 그대로 학습용 단일 행동 실행기로 쓰지 않는다.
- 환경 차이: 현재 Python 3.12.11에서 기존 185개 테스트가 수집되었고 OpenCV import 오류는 없다. 최초 sandbox 실행은 176개 통과, localhost bind 제한 오류 9개였다. 최종 검증은 별도 증거 문서에 기록한다.

재사용: 기존 M4 AS5600 목표각 제어/보정/정착/오류 판정, SafetyGate, 수동 입력, 객체·색상 검출, typed ROS/diagnostics/lifecycle, fake endpoint, CI/systemd, URDF/FK/IK. 기존 `policy_proposal`은 규칙 정책이며 `cup_grasp_plan`은 dry-run, 조명 mission 완료는 조작 성공이 아니다. M4 외 JointState 기본값·추정값은 실측 정답으로 쓰지 않는다.

9월 계획에서 M0-01~05와 선택 경로의 시간/출력 검증을 이어받는다. 전체 MCU 정비 완료를 AI 착수 조건으로 삼지 않는다. 비활성 진동 정책, RTOS/DMA 전환, 두 번째 축 센서, 전 축 ros2_control/MoveIt, 대형 VLA 학습은 선행조건에서 제외한다. 기존 카메라·M4·실제 실행 이력, Mac CPU와 필요할 때만 소액 클라우드로 시작한다.

### 완료한 구현과 최신 검증

| 단계 | 완료한 소프트웨어 범위 | 커밋 / 상세 기록 |
|---|---|---|
| S1 | 영속 명령 journal, ID 중복·충돌 차단, 재시작·결과 불명 처리, ROS adapter/producer ID | `f581253` · [호스트](evidence/2026-10-14-command-journal-software.md) |
| S2 일부 | MCU boot/command ID, 제한 실행·deadline·취소, 드라이버 PWM write 기록, 제어권 관리, 신규 API 숫자 검사 | `1bcd59e` · [MCU](evidence/2026-10-14-bounded-actions-software.md) |
| S3a/S3b/S4a | 원본 프레임 ID·exact JPEG 조회, 신선도 전달, 행동 전후 schema | `ca48f04` · [관측](evidence/2026-10-14-source-observations-software.md) |
| S4b dry-run | 원본 이미지/hash·설정 저장, 실행 의도·중단·누락·불명 기록, 읽기 전용 감사 | `9ad4935` · [수집 runner](evidence/2026-10-14-dry-run-collector-software.md) |
| S5a | manifest, 원본 무결성·출처 재검증, 세션/episode 및 연결 그룹 분할, 누수·합성 혼입 차단 | `2bda97e` · [데이터셋](evidence/2026-10-14-dataset-manifest-software.md) |
| 리뷰 후 수정 | 실행 직전 source age에 시계 교정 오차와 경과시간 포함, 한계 초과 시 실행 0회 회귀 | `73132e9` · [수정·회귀 결과](evidence/2026-10-14-dry-run-collector-software.md#중간-리뷰-후-실행-전-시계-오차-검사-수정) |

- 최신 전체 회귀: **372개 통과, 10.949초** (`python3 -m unittest discover -s tests -q`, 마지막 코드 수정 후 실행). 독립 재리뷰에서 남은 확정 차단 문제는 발견하지 못했다. 이번 문서 최신화에서는 코드 테스트를 재실행하지 않았다.
- ESP32와 ESP32-CAM은 각 구현 단계에서 compile/link 성공. 업로드·보드 실행·모터 구동은 하지 않았다. native 가상 시간/출력·fake ROS·로컬 파일 시험은 ROS Jazzy 전체 빌드/lifecycle 시험이나 물리 정지·파지 성공의 증거가 아니다.
- 저장된 dry-run은 **10개 세션, 11개 합성 시도**이며 모두 학습·평가 제외다. manifest `valid=true`, `readyForTraining=false`, 적격 0개다. 실제 행동 데이터·학습 모델·조작 성능 개선 결과는 아직 없다.
- 단계별 증거 문서의 217/235/291/331/368개 결과는 해당 단계 당시 기록이다. 최신 전체 기준은 위 372개다.

### 남은 구현과 바로 다음 단계

**장치 없이 다음에 할 일은 S5b 학습·평가 코드 준비다.** 아래 파일은 예정이며 아직 구현되지 않았다. 기존 manifest를 재사용하고 추가 센서·대형 VLA·유료 GPU를 선행조건으로 삼지 않는다.

| 순서 | 예정 파일 / 작업 | 완료 기준 |
|---|---|---|
| 1 | `learning/features.py`, `learning/dataset.py` 확장: manifest 기반 loader, 입력·정답 계약 | 소비할 원본 bytes hash 재확인, 부적격/합성 데이터의 실제 학습 진입 거부, 현재 행동의 after·실제 출력 trace를 사전 예측 입력에 넣지 않음 |
| 2 | `learning/models.py`, `tools/train_visual_dynamics.py`: 무변화·선형 기준선부터, 필요 시 작은 MLP | 정규화는 train만 fit, validation/test 오염 회귀, seed·분할·설정·모델 저장과 재현, Mac CPU에서 분리된 합성 smoke |
| 3 | `tools/evaluate_visual_dynamics.py`, `tools/evaluate_robot_trials.py`: 예측 및 실물 trial 평가 | 예측 MAE와 조작 성공률 구분, 실패/unknown 포함 분모, 모델 비교표; 합성 점수를 실물 성과로 발표하지 않음 |

실제 수집 adapter 중 capability/boot/결과·출력 및 exact perception 읽기·저장은 비구동 probe로 구현했다. 남은 연결은 host journal ↔ MCU 명령 송신과 transition 수집, 결과 불명 reconciliation, 활성 UART seq/session 검증, 기존 숫자 입력 경로 통일, 제어 상태 polling 분리·deadline/queue 상한이다. 연결된 장치에서는 먼저 시야·빠른 관측 경로·시계/M4 시간 구간을 정리한다. 전체 펌웨어 재설계는 범위에 넣지 않는다. 새 `/actions`는 기본 비활성이며 기존 host `/shoulder`가 자동 전환되지는 않는다.

**실제 데이터로 학습하기 전**에는 아래 §7의 장치 준비·시계/출력 교정·한 축 수집 검증을 통과해야 한다. 실제 데이터로 기준 모델과 학습 모델을 평가한 뒤, 같은 조건의 접근·집기·배치 trial에서 개선 여부를 확인한다. 소프트웨어 기반 완료와 실물 학습·성능 입증 완료를 구분한다.

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

아래의 구현 완료는 각 단계 당시 소프트웨어 범위다. 후속 비구동 실물 결과는 위 10월 15일 기록을 우선한다. S2a는 신규 API와 공통 M4 finite 검사만 완료했으며 기존 모든 파서 통일은 미완료다. S4b의 실제 장치 adapter는 읽기 경로만 구현했고 명령 송신·물리 transition 수집은 미완료다.

| 순서 | 수정·추가 파일 | 구체 작업 | 소프트웨어 완료 기준 | 별도 실물 관문 |
|---|---|---|---|---|
| S1a 구현 완료 | `ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/m4_write_executor_core.py`, 신규 `m4_command_journal.py`, `tests/test_m4_write_executor_core.py` | SQLite 기록, 동일 ID 재접수/충돌, monotonic TTL, 원자 예약, 응답 유실 분리, 재시작 복구, 결과 조회, 기록 상한 | fake clock/POST로 재전달·동시 confirm·만료·충돌·fetch 실패·POST 유실·기록 오류·재시작 시험; ID당 POST 최대 1회 | 아직 MCU의 실제 중복 억제 증거 아님 |
| S1b 구현 완료 | `m4_write_executor_node.py`, `motionbrain_hardware_interface.hpp/.cpp`, 신규 표준 C++ ID helper 및 테스트 | 영속 journal 기본값, read-only 결과 서비스, 원 created_at 보존, 랜덤 producer session ID | Python adapter fake ROS 시험, native C++ ID 생성/실패 시험, 기존 호스트 회귀 | ROS Jazzy 전체 빌드/launch와 재시작 시험 필요 |
| S2a 일부 구현 | 신규 `src/control/numeric_validation.h`, `src/control/shoulder_angle_controller.cpp`, `src/input/serial_command.cpp`, `src/network/web_server.cpp` | 전체 숫자 소비/finite/overflow/범위 검사, 공통 start 경계 재검사 | NaN/Inf/부분 숫자/과대값 거부 및 fake 출력 0회; ESP32 compile | 모터 전원 분리 상태에서 입력 거부 확인 |
| S2b 구현 완료 | `src/control/command.h`, `src/control/action_registry.h`, `src/network/web_server.*` | boot/action ID, 정규화 요청, 같은 ID 재조회/충돌, 32건 무퇴거 기록/상한 정책, capability version | boot mismatch 거부, 결과 유실은 unknown, 만료 ID 재실행 금지 | 응답 유실/재부팅/중복 수신 후 드라이버 로그 확인 |
| S2c 구현 완료 | 신규 `src/control/bounded_action_executor.h/.cpp`, `dispatcher.*`, `safety_gate.*`, `src/main.cpp` | 한 축·한 행동, busy, MCU deadline/취소, M4 기존 제어기 위임, 종료 후 output_off 대기 | 가짜 clock·출력으로 timeout/cancel/safety/시간 wrap, 만료 뒤 재출력 없음 | 한 축 최소 행동과 종료 지연 측정 |
| S2d 구현 완료 | `src/motor/applied_output_log.h`, `motor_driver.*`, `teleop_adapter.*`, `motion_sequence.*`, `dispatcher.*`, `web_server.*` | 공통 드라이버 write/강제 off 감사, 제어권 lease, 수동 개입 취소, 이전 sequence/M4/teleop cache 정리 | ramp·방향·off 기록, overflow/drop 감지, override 후 이전 출력 재개 없음 | HTTP 부하/단절/deadman 중 off/잔여 이동 확인 |
| S2e 미구현 | `src/bridge/stm32_bridge.*`, `src/input/teleop_adapter.*`, `src/main.cpp` | 활성 UART seq/session 검증, reject freshness 갱신 금지, loop 간격/deadline overrun 계측 | 중복·역순·재부팅·wrap 입력, stale 반복으로 재무장 불가 | STM32 heartbeat 유지 및 stale 차단 시험 |
| S3a 구현 완료 | `firmware/esp32cam/src/main.cpp`, `tools/motionbrain_perception_service.py`, `tests/test_perception_service.py` | camera boot/source timestamp 의미 확인, 원본 frame identity, 불변 JPEG+검출 캐시/ID 조회 | A metadata에 B JPEG 혼합 불가, 만료 ID 404/410, held 별도 source, wall clock 점프 시험 | 같은 버퍼 반복/카메라 재시작/획득 지연 관찰 |
| S3b 구현 완료 | `ros2_ws/src/motionbrain_msgs/msg/CameraDetection.msg`, `motionbrain_status_node.py`, `ros2_ws/src/motionbrain_control/src/control_guard_node.cpp`, 기존 guard 테스트 | source ID/held/validity/age 전파, 재발행으로 freshness 회복 금지 | 오래된 ID 연속 수신해도 stale, metadata 미상은 제어 거부 | 느린 2초 polling에서 stop-observe 루프 확인 |
| S4a 구현 완료 | 신규 `learning/schema.py`, `tests/test_action_transition_schema.py` | observation/action/transition validator, 버전·품질·판정 출처 | 잘못 연결된 frame·동일 before/after·unknown·stale M4·NaN·missing metadata 격리 | 합성 fixture 성공은 데이터 수집 성공 아님 |
| S4b dry-run 완료 | 신규 `learning/runner.py` 단일-step 골격, `tools/capture_action_transitions.py`, `tests/test_action_transition_capture.py`; 기존 두 policy episode 도구는 유지 | 공통 action/observation adapter로 runner의 before/실제 행동/after 연결, append-only 원본 로그, 이미지 hash와 설정 snapshot | 취소/관측 소실/기록 손실까지 기록, 종료 후 획득 증거 없으면 부적합 | 한 물체/정해진 자세에서 작은 교정 데이터 수집 |
| S5a 구현 완료 | 신규 `learning/dataset.py`, `learning/splits.py`, `learning/provenance.py`, `tools/build_action_dataset.py`, 대응 테스트 | 원본 무결성·raw/요약 연결·출처 재검증, 모든 시도 inventory, 세션/episode/hash/frame/행동/배치 그룹 분할 | 수동 manifest 변조·누락·합성 혼입·교차 분할 거부, 합성 11개 적격 0 | 실제 수집 adapter와 교정 근거 연결 후 별도 세션 실물 데이터 적용 |
| S5b 다음 구현 | 예정 `learning/features.py`, `learning/models.py`, `tools/train_visual_dynamics.py`, `tools/evaluate_visual_dynamics.py`, `tools/evaluate_robot_trials.py` | 무변화·선형·작은 MLP 비교, 그룹 분할, artifact 저장/재현 | Mac CPU 합성 smoke, train-only 정규화, episode/배치/frame 누수 검사 | 별도 세션 실제 접근·집기·배치 trials |
| S6 미구현 | S4b의 공유 `learning/runner.py` 확장, 신규 `tools/run_manipulation_trial.py`; `tools/motionbrain_dashboard.py`와 ROS adapter | 관찰→선택→제한 실행→off→정착→재관찰 단일 루프, shadow/작업자 한 단계/감독 반복 모드 | restart 기본 비구동, goal 소실·범위 밖·unknown 중단, 기존 confirm 자동 위조 없음 | 접근부터 검증 후 집기·들기·이동·놓기 순차 확장 |
| S7 소프트웨어 기록 / 실물 비교 미완료 | `docs/evidence/`, `PORTFOLIO.md`, `README.md`, `claim-to-evidence-matrix.md` | 조건/버전/원시 로그/영상/실패 표 연결 | 코드·합성·저장된 실물·새 실물 증거 분리 | 학습 개선이 입증된 범위만 발표 |

S2c는 deadline/취소 시 기존 hardStopAll을 호출하고 output_off를 확인하는 방식으로 구현했다. 출력 종료를 확인하지 못하면 unknown과 제어권을 유지하며 새 구동을 차단한다. cooperative loop의 blocking 중에는 즉시 정지를 보장하지 못한다. 관절별 최소 PWM/시간/정착값은 실측 전 확정하지 않는다. applied_pwm는 `ledcWrite`에 전달한 명령값(`evidence=driver_write`)이며 핀 파형이나 실측 RPM이 아니다.

`motionbrain_status_node.py`의 제어 상태 polling은 camera/routine/events와 분리하고 요청 deadline/queue 상한을 S3~S6 통합 전에 둔다. 카메라 지연 때문에 guard/명령 판단이 오래된 로봇 상태를 쓰지 않는 장애 주입 테스트가 완료 기준이다. 전체 ROS 운영 도구 재패키징은 선행조건이 아니다.

## 4. S1 운영 계약

- ROS node의 기본 journal: `$XDG_STATE_HOME/motionbrain/m4_commands.sqlite3`; 미설정·빈 값·상대 XDG 경로이면 `~/.local/state/motionbrain/m4_commands.sqlite3`. 명시 journal 경로는 절대 경로(`~` 확장 허용)만 받는다. `command_journal_path`, `command_journal_max_records` 파라미터 지원. node는 `:memory:`를 거부한다. core의 메모리 모드는 단위 테스트용이다.
- 같은 파일은 POSIX flock으로 단일 executor만 소유한다. 다른 계정/DB 경로로 여러 physical executor를 띄우면 보호되지 않는다. 배포에서 하나의 장치와 영속 경로를 고정한다. 파일 접근 실패/손상/가득 참은 자동 초기화나 fallback 없이 오류로 남긴다.
- pending/validating 중 호스트 재시작은 `canceled`; dispatch 예약 뒤 재시작은 `outcome_unknown`. 이미 MCU 수락 응답을 저장했으면 `accepted` 기록을 유지하지만 현재 모터 상태를 의미하지 않는다.
- 결과 불명 기록이 있으면 새 confirm도 차단한다. 이번 슬라이스에는 unknown 해제/재시도 API가 없다. MCU 결과 조회/boot 계약을 구현하고 작업자 확인에 근거한 reconciliation을 추가해야 한다. 기록 삭제로 우회하지 않는다.
- `/motionbrain/m4_write_result`는 기존 `motionbrain_msgs/srv/M4WriteConfirm` wire shape를 재사용하는 **읽기 전용** 서비스다. 명령 ID만 받아 journal을 읽는다. `/motionbrain/m4_write_confirm`은 명시적 작업자 승인 경로로 유지한다.
- `raw_json`의 `executionState`, `dispatchAttempted`, `acknowledged`, `ledgerState`, `proposalMetadata`를 판정에 사용한다. `success=true`는 MCU 수락 응답, `acknowledged=true`는 유효한 수락/거부 응답, `forwarded=true`는 전송 시도 또는 crash 전에 시도했을 가능성이다. bool 하나로 물리 실행 여부를 판단하지 않는다. 결과 조회의 미존재는 `result_not_found/outcome_unknown`이며 미실행 증거가 아니다.
- 원 `created_at`은 ROS clock metadata로 보존만 한다. TTL은 최초 호스트 수신 monotonic 기준이다. 아직 처음 보는 오래된 proposal의 source-age를 보장하지 못하며 새 producer/실험 session handshake는 S2에 남아 있다.
- 현재 `/shoulder`는 command ID와 요청별 timeout을 지원하지 않는다. 무시되는 query 필드를 붙여 해결됐다고 주장하지 않는다. `timeoutMs`는 호스트 검증/fingerprint에만 반영되는 기존 제한이며 MCU 동작 deadline은 새 `/actions`에 구현했으나 이 호스트 경로와의 연결은 남아 있다. S1만으로 학습 행동 실험을 시작하지 않는다.

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

- 10/14 완료: 위 S1~S5a의 명시된 소프트웨어 범위와 리뷰 수정. 10/18까지의 다음 목표: S5b loader·특징/기준선·평가 준비, 실제 adapter 계약 구체화. 장비 연결 가능 시점은 미정이다.
- 10/19~10/30 교육 기간: 가능한 시간에 S2/S3 통합과 첫 작은 행동 데이터 수집. S5b 오프라인 준비와 실제 adapter 구현 병행. 교육 완료를 구현 시작 조건으로 삼지 않는다.
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
