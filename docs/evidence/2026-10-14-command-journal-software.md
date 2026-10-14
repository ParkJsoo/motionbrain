# 2026-10-14 M4 명령 기록 소프트웨어 검증

> 현재 상태 (2026-10-14): 이 단계는 `f581253`에 커밋되었다. 최신 코드 수정은 `73132e9`이며, 수정 후 전체 **372개 테스트 통과(10.949초)**와 독립 재리뷰를 기록했다. 아래 단계별 테스트 수는 당시 결과로 보존한다. 완료 범위·미완료 구현·다음 학습 단계는 [종합 계획서](../ai-manipulation-plan-2026-10-14.md), 마지막 회귀는 [시계 오차 수정 기록](2026-10-14-dry-run-collector-software.md#중간-리뷰-후-실행-전-시계-오차-검사-수정)을 기준으로 한다. 오늘 새 실물 검증이나 실제 데이터 학습은 하지 않았다.

## 범위와 출발점

- 기준 HEAD: `99ea0d6`. 이 문서는 그 위에서 구현하여 `f581253`으로 커밋한 변경의 검증 기록이다.
- 실행 환경: macOS, Python 3.12.11, 로컬 clang++/선택된 Xcode SDK.
- fake 상태/POST callback, 가상 monotonic clock, 임시 SQLite 파일, fake ROS Node,
  native C++ 실행과 기존 localhost fake HTTP 서버만 사용했다.
- 실제 Pi/ESP32/STM32/카메라 접속, firmware 업로드, 모터 구동은 하지 않았다.
- ROS Jazzy 전체 colcon 빌드/실제 ROS service 및 lifecycle 통합 검증은 하지 않았다.
  C++ ID helper의 native compile/run과 Python adapter 시험은 이를 대체하지 않는다.

## 기존 결함과 변경

기존 `accept → confirm → 같은 ID accept → confirm`은 fake POST가 2회였다.
POST callback이 수신을 기록한 뒤 TimeoutError를 내면 `forwarded=false`였다.
기존 연속 confirm만 거부하는 시험은 이 두 경계를 검증하지 못했다.

이번 변경:

- SQLite journal에 명령 의미, 최초 TTL, 실행 예약 및 결과를 보존한다.
- 같은 ID·같은 내용은 원래 결과 유지, 목표/timeout 변경은 충돌 거부한다.
- TTL은 최초 수신 monotonic 기준이며 재전달·느린 상태 조회로 연장되지 않는다.
- status fetch 실패는 `not_sent`, POST 진입 뒤 응답 유실/잘못된 응답은
  `outcome_unknown`이다. 저장 오류 후 재확인·조회에서도 unknown을 유지한다.
- 전송 직전 예약을 영속화한다. pending/validating 재시작은 canceled,
  dispatch 예약 후 재시작은 unknown이다. 과거 승인 자동 실행은 없다.
- 결과 불명은 새로운 confirm도 차단한다. 결과 조회는 HTTP 호출 없이 journal만 읽는다.
- 기록 상한에서는 자동 삭제 없이 신규 접수를 거부한다. 동일 파일 동시 owner는 거부한다.
- C++ producer ID는 랜덤 session 128비트와 순번이다. entropy/초기화 실패 시
  configure 실패 또는 publish 없이 ERROR로 종료한다. 생성시각은 ROS metadata로 보존한다.

## 수행한 검사

| 검사 | 결과 | 의미 |
|---|---|---|
| 변경 전 sandbox 기존 suite | 185개 중 176 통과, 9 오류 | 오류는 localhost bind 권한; OpenCV import 오류 없음 |
| M4 host/adapter 관련 suite | 45개 통과 | core 25개, node adapter 8개, 기존 contract/shadow 회귀 포함 |
| native C++ helper | 2개 통과 | 실제 compile/run, 서로 다른 프로세스/인스턴스 ID, 증가/실패 주입 |
| 최종 전체 host suite | 217개 통과 | localhost bind 허용 환경, 2026-10-14 실행 |
| diff 공백/패치 검사 | 통과 | `git diff --check` |

재현 명령(저장소 루트):

```sh
python3 -m unittest discover -s tests -p 'test_m4_write*.py'
python3 -m unittest discover -s tests -p 'test_proposal_command_ids_native.py'
python3 -m unittest discover -s tests
git diff --check
```

전체 suite에는 OpenCV/numpy와 localhost bind 권한이 필요하다. native 시험은 C++17
컴파일러가 없으면 명시적으로 skip하므로 결과에서 skip 여부를 확인해야 한다.
이번 환경에서는 native 시험도 실행했다. Mac SDK 경로는 테스트 명령에만 지정하며
시스템 SDK 설정을 바꾸지 않았다.

## 중요한 시험 시나리오

- 접수 전/소비 후/만료 후 같은 ID 재전달, 다른 payload 충돌, 최초 TTL 경계.
- 상태 조회가 TTL을 초과하거나 실패하면 POST 0회.
- 같은 ID 동시 confirm과 다른 ID 실행 경쟁, 기록 잠금/용량 상한.
- POST 수신 후 응답 유실, malformed 응답, 거부 응답과 수락 응답 구분.
- pending/validating/dispatching/accepted 기록의 close/reopen 복구.
- journal write/read 실패 전후, failure latch 후 같은 ID 재confirm/조회 의미 유지.
- node의 영속 기본 경로, 메모리/상대 경로 거부, 빈 XDG fallback, 초기화 오류 시 잠금·ROS context 해제, readonly 결과 서비스, 원 생성시각 전달.
- 수락 응답을 목표 도달/물체 조작 성공으로 변환하지 않음.

재시작 시험은 저장된 phase를 만들어 journal을 닫고 다시 여는 호스트 복구 시험이다.
실제 전원 차단, 파일시스템/스토리지 손실, ROS 프로세스 강제 종료 실험은 수행하지 않았다.

## 보장하지 않은 것과 다음 관문

- 호스트의 동일 journal 경로에 기록된 ID에 대한 POST 최대 1회 시도이며, MCU의
  물리 exactly-once 실행 보장이 아니다. 다른 journal·host·직접 HTTP는 범위 밖이다.
- journal을 삭제/교체/과거 백업으로 되돌리면 replay 방지 근거를 잃는다.
  unknown/capacity를 풀기 위한 DB 삭제를 운영 절차로 사용하지 않는다.
- `success`는 MCU 수락 응답일 뿐이다. `acknowledged`는 유효 응답(거부 포함),
  `forwarded`/`dispatchAttempted`는 시도 또는 crash 전 시도 가능성이다.
  기록 부재/읽기 실패의 unknown에서 false는 물리 미실행 증명이 아니다.
- 기존 호스트가 쓰는 `/shoulder`에는 ID별 결과 조회·dedup·요청 timeout 적용이 없다. 후속 MCU `/actions`에는 구현했지만 host 연결은 미완료다. 기존 `timeoutMs`는
  호스트 검증/명령 동일성에만 반영된다. 결과 불명 해제 API도 아직 없다.
- 최초 수신 이전에 지연된 proposal의 source-age/producer session 허가는 후속 작업이다.
  원 ROS 시각을 host monotonic과 직접 비교하지 않는다.
- 이 S1 커밋의 범위 밖이었던 bounded MCU executor/출력 로그/frame identity/transition dry-run 수집기는 후속 커밋에서 구현했다. [파일별 후속 계획](../ai-manipulation-plan-2026-10-14.md)에
  순서·완료 기준·실물 준비와 교정 실험을 기록했다.

현재 다음 학습 작업은 manifest 기반 loader·특징/기준선·train-only 전처리·평가 코드다.
실제 host→MCU 수집 adapter와 남은 입력 경계 검사는 별도로 필요하다. 실물 단계에서는
모터 전원 분리 부팅 확인 → 한 축 최소 행동/종료 측정 → 관측 연결 검증 순서로 진행한다.
