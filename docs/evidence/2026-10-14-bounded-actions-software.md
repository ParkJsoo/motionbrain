# MCU 명령 식별·제한 실행·출력 기록 — 소프트웨어 검증

> 현재 상태 (2026-10-14): 이 단계는 `1bcd59e`에 커밋되었다. 최신 코드 수정은 `73132e9`이며, 수정 후 전체 **372개 테스트 통과(10.949초)**와 독립 재리뷰를 기록했다. 아래 단계별 테스트 수는 당시 결과로 보존한다. 완료 범위·미완료 구현·다음 학습 단계는 [종합 계획서](../ai-manipulation-plan-2026-10-14.md), 마지막 회귀는 [시계 오차 수정 기록](2026-10-14-dry-run-collector-software.md#중간-리뷰-후-실행-전-시계-오차-검사-수정)을 기준으로 한다. 오늘 새 실물 검증이나 실제 데이터 학습은 하지 않았다.

작성: 2026-10-14 KST. 기반 커밋: `f581253` (이전 호스트 명령 journal 변경).
사용자는 외부에 있어 로봇팔과 Raspberry Pi 연결이 불가능하다. 이번 작업은 로컬 구현·빌드·가상 시간/출력 검증이며 장치 접속, 배포, 업로드, 보드 재시작, 실제 구동은 하지 않았다.

## 기존 변경 리뷰와 이번 범위

이전 host journal/ROS adapter/C++ producer ID를 독립 에이전트가 재검토했다.
수정이 필요한 확정 결함이 없었고 관련 35개 테스트도 직접 재실행해 통과했다.
기존 변경은 별도 로컬 커밋 `f581253`으로 보존한 뒤 MCU 구현을 시작했다.

이번에는 기존 수동·시퀀스·M4 구현을 유지하고 **추가 bounded action API**를 연결했다.
기존 호스트 executor의 `/shoulder` 요청이 자동으로 이 API로 전환되지는 않는다.
새 경로를 실제 수집 runner에 연결할 때 capability/boot ID 확인이 필요하다.

| 파일 | 구현 |
|---|---|
| `src/control/action_registry.h` | boot ID + command ID, 고정 32건 무퇴거 기록, 요청 원문·실행 상태 |
| `src/control/bounded_action_executor.h/.cpp` | 가상 출력 포트, 단일 행동, 중복/충돌/busy, deadline·취소·안전 차단, 결과 불명 latch |
| `src/control/bounded_action_service.h/.cpp` | 기존 SafetyGate/M4/드라이버 adapter, 수동 제어권 이전, 결과/출력 JSON |
| `src/motor/applied_output_log.h`, `motor_driver.*` | 실제 `ledcWrite` 지점/강제 OFF/부팅 0 기록, ordinal·시각·signed PWM·유실 집계 |
| `src/control/numeric_validation.h`, `web_server.*` | 전체 숫자 소비/finite/범위 검사, `/actions`·취소·조회·출력 로그 |
| `dispatcher.cpp`, `teleop_adapter.*`, `main.cpp` | 기존 명령/텔레옵 개입 시 취소, unknown일 때 새 구동 차단, 안전 정지는 허용, 출력 전 deadline 확인 |
| `shoulder_angle_controller.cpp` | 공통 M4 시작 경계에 finite 검사 추가 |
| `tests/native/`, 관련 Python harness | production core·driver·adapter를 native 컴파일하고 가상 시간/출력 실행 |

## API 계약

새 실행은 기본 **비활성** (`MOTIONBRAIN_ENABLE_BOUNDED_ACTIONS=0`)이다.
실험용 빌드의 `-DMOTIONBRAIN_ENABLE_BOUNDED_ACTIONS=1`은 소프트웨어 활성화 수단일 뿐,
물리 교정/검증을 완료했다는 뜻이 아니다. 기존 기능은 그대로 동작한다.

| 메서드/경로 | 의미 |
|---|---|
| GET `/actions` | schema version, enabled, boot ID, active, 기록 수/상한, clock domain |
| POST `/actions` | 기존 명령 인증 필요. bootId/commandId/kind/motor/durationMs + 행동 값 접수 |
| GET `/actions?bootId=…&commandId=…` | 읽기 전용 원요청/결과 조회. 미존재는 404 + outcome_unknown, 다른 부트는 409 |
| POST `/actions/cancel` | 기존 인증 + bootId/commandId. 해당 활성 행동만 취소; terminal은 결과 재조회 |
| GET `/action_outputs` | 부트 ID, driver_write 근거, 순번·시각·ordinal·motor·pwm·원인, dropped |

- `kind=pulse`: M1/M2/M3/M5, signed `pwm` -255..255(0 제외), `durationMs` 1..10000.
  값은 defaultSpeed의 percent가 아닌 드라이버 목표 PWM이다. 기존 ramp는 유지된다.
- `kind=shoulder`: M4만, `targetDegrees` 기존 조건부 soft limits,
  `percent` 75..100, `durationMs`는 외부 deadline. 기존 폐루프·센서·정착·보정을 재사용한다.
- 이 범위는 파서/메모리/시간 연산을 위한 **소프트웨어 상한**이다. 모든 축에서 안전하거나
  유효한 물리 행동 범위로 교정된 값이 아니다. 특히 100ms나 최대 PWM을 표준 행동으로 정하지 않았다.
- command ID 최대 128자, boot ID 최대 32자, ASCII 영숫자·`-_.:`만 허용.
  긴 ID를 잘라서 접수하지 않는다. 같은 boot/ID와 같은 내용은 원기록을 반환하며 시작/TTL을 갱신하지 않는다.
  다른 내용은 conflict, 새 ID 경쟁은 busy, 32건 이후는 registry_full이며 기존 ID 조회/중복응답은 유지한다.
- boot ID는 Wi-Fi 초기화 이후 ESP32 RNG에서 생성한 128비트 식별자다. registry는 RAM에만 있다.
  이전 boot를 넣은 요청은 재부팅 뒤 거부되며, 결과를 확인할 수 없으면 unknown이다.
  호스트가 예전 명령을 새 boot ID로 다시 작성해 보내는 것을 이 장치가 막아주지는 않는다.
- POST `accepted=true`는 ID가 접수되었다는 뜻이다. 시작 실패도 예약된 ID의 failed 기록으로 남을 수 있으므로
  `record.state`, `record.reason`, `outputOff`를 확인해야 한다. taskSuccess/visuallySettled는 null이다.

## 실행과 결과 의미

1. 최초 실행 전 기존 시퀀스·base/M4 제어기·teleop/deadman·모터 current/target이 모두 idle인지 확인한다.
2. ID를 먼저 예약한 뒤 action ordinal을 드라이버에 연결하고 이전 웹 lease와 teleop 출력 캐시를 정리한다.
3. MCU loop에서 deadline과 SafetyGate를 검사한다. 센서 I2C 갱신 직후/M4 제어 직전,
   모터 ramp 직전과 HTTP 처리 뒤에도 검사한다.
4. 종료 때 기존 M4 제어기를 취소하고 hardStopAll로 목표/현재 출력을 지운다.
   stop 처리 뒤 실제 MCU 단조시각으로 elapsed/overrun을 기록한다.
5. 정상 기간 종료는 completed+outputOff, M4 도달은 별도 targetReached 근거가 있다.
   늦은 tick/정지 처리로 deadline을 넘기면 timed_out/deadline_exceeded로 남긴다.
   단 1ms 초과도 기록하므로 이 판정은 물리 품질 허용오차를 확정한 것이 아니다.
6. OFF가 확인되지 않으면 outcome_unknown과 소유권을 유지한다. 후속 tick은 stop만 재시도한다.
   나중에 OFF를 확인해도 결과를 성공으로 바꾸거나 새 구동을 허용하지 않는다.
7. 작업자 명령/teleop 개입은 operator_override로 취소한다. unknown은 새 구동/ARM을 막지만
   기존 STOP/DISARM/전체 정지 경로를 막지 않는다. 조명·dry-run은 계속 사용할 수 있다.

단일 cooperative loop이므로 I2C/HTTP/로그 등의 blocking 동안 즉시 deadline을 강제할 수 없다.
이번 코드는 hard real-time 정지 보장이 아니다. MCU hang/전원/핀 상태도 별도 실물 검증 대상이다.
unsigned 시간 차이는 millis wrap을 지원하지만 loop가 2^32ms 이상 멈춘 경우를 구별하지 못한다.

## 출력 기록과 데이터 품질

- 초기 5개 PWM=0, ramp 단계·방향 변화, hardStop/emergencyStop/state 차단을 공통 드라이버 지점에서 기록한다.
- signed PWM은 소프트웨어가 `ledcWrite`로 적용한 값이다. 실제 핀 파형·RPM·관절 속도·기계 정착이 아니다.
- `actionOrdinal=0`은 bounded 행동 밖의 출력, 양수는 같은 boot의 action record ordinal이다.
  boot ID 없이 ordinal만 세션 간 연결하면 안 된다.
- 정상 write는 동일 PWM/ordinal 반복을 생략하며, 강제 OFF는 항상 기록한다.
  128건 ring이 넘치면 오래된 출력 로그가 사라지고 dropped가 증가한다. action 기록은 지우지 않는다.
- 수집기는 마지막 sequence와 dropped를 추적해야 한다. 전후 행동 구간이 ring 밖으로 사라졌으면
  완전한 trace라고 판단하지 않는다. 한 행동이 128개 이상의 ramp/보정 기록을 만들 수도 있다.
- 후속 커밋에서 원본 frame ID/신선도·행동 전후 dry-run 수집을 구현했다. 실제 adapter와 카메라 시계 교정은 남아 있으며 출력 로그만으로 학습 샘플이 완성되지 않는다.

## 소프트웨어 시험

실제 C++ 구현을 C++11로 컴파일하고 가짜 millis/LEDC/GPIO/제어기 포트에 연결했다.
문자열이 소스에 존재하는지만 검사하는 시험으로 대체하지 않았다.

- 부팅 불일치, 같은 ID 재접수/충돌, 새 ID busy, 32건 상한/무퇴거, 잘못된 숫자/ID.
- 정확 deadline/늦은 loop/stop 작업 지연, millis wrap, 취소/안전 차단, 부분 시작 실패.
- 출력 미확인 latch, stop만 재시도, unknown 이후 OFF가 확인돼도 새 구동 금지.
- M4 제어 결과의 reached/failed/deadline 분리, 실패 뒤 중복 ID 재실행 없음.
- 생산 motor driver의 실제 ramp write와 가짜 LEDC 기록 대조, 강제 정지/부팅 OFF,
  disarm 중 target 제거와 재무장 후 잔류 출력 없음, ring overflow 집계.
- adapter의 기본 비활성/명시 opt-in, M4 재사용, 소유권·수동 개입·안전 정지 허용.

재현(장치 접속 없는 명령):

```sh
python3 -m unittest discover -s tests -p 'test_bounded*_native.py'
python3 -m unittest discover -s tests -p 'test_motor_output_native.py'
python3 -m unittest discover -s tests
pio run
git diff --check
```

최종 검증 결과:

| 검사 | 결과 |
|---|---|
| 기존 전체 suite + 새 native 시험 | **235개 통과**, 7.450초, skip 없음 |
| 새 core/숫자 파서 native | 10개 그룹 통과 |
| 생산 motor driver native | 3개 그룹 통과 |
| 생산 service+core+driver adapter native | 5개 그룹 통과; 기본0/opt-in1 양쪽 컴파일 |
| ESP32 `pio run` | **성공**, 기본 비활성 firmware compile/link |
| ESP32 정적 RAM / flash | 65,048 / 1,103,701 bytes (19.9% / 84.2%) |
| `git diff --check` | 통과 |

최초 PlatformIO 실행은 sandbox의 cache lock 쓰기 제한으로 실패했고, 로컬 빌드 권한으로 재실행했다.
초기 통합 컴파일에서 기존 MotionSequence API 이름을 바로잡은 뒤 최종 빌드가 통과했다.
Arduino 주변 객체를 대체한 native 시험과 ESP32 컴파일 결과를 실제 보드 실행으로 표현하지 않는다.

독립 통합 리뷰에서 발견한 unknown 상태의 긴급 정지 차단과 I2C 읽기 후 deadline 재확인 누락을 수정했다.
생산 adapter 시험에서 긴급 정지는 허용하고 재구동은 차단하는 회귀를 검증했고, 후속 독립 리뷰에서
이 두 항목과 요청 kind 기록 수정 확인을 받았다.

## 남은 실물 검증과 후속 구현

실물 연결이 가능해지면 먼저 ESP32·CAM 포트/firmware, 모터 전원 분리·독립 차단,
STM32 안전 heartbeat/imuOk, M4 AS5600 상태, 현재 자세/카메라 시야를 확인한다.

1. 모터 전원 분리 후에만 대상 firmware 업로드·부팅/재부팅 기본 비구동·boot mismatch/API 인증을 확인한다.
2. 명시적으로 허가한 고정 자세/한 축에서 PWM/ramp/지속시간의 최소 유효 행동을 교정한다.
3. HTTP 부하/I2C 지연/호스트 단절 중 실제 off 지연을 측정한다. software trace와 영상 정착을 구분한다.
4. 수동 개입·deadman·heartbeat stale·상태 전환에서 취소 후 과거 동작이 재개되지 않는지 확인한다.
5. M4는 기존 검증 자세/부하/범위부터 재검증한다. 넓은 임시 soft limits를 전 자세 충돌 안전 범위로 쓰지 않는다.
6. 결과 불명/registry_full을 풀기 위해 자동 재부팅하지 않는다. 출력/자세/기록 보존을 확인한 새 작업자 세션 절차가 필요하다.

원본 관측 ID/age, transition validator와 dry-run 수집기·manifest는 후속 커밋에서 완료했다.
남은 구현: 활성 UART seq/session 중복·역순과 freshness, host의 새 capability/boot 기반 실제 수집 adapter.
장치 없이 진행할 다음 학습 단계는 manifest loader·특징/기준선·train-only 전처리·평가 코드다.
기존 모든 시리얼/웹 숫자 파서의 통일, 장치 HTTP runtime 시험, 물리 결과 reconciliation도 남아 있다.
