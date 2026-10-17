# M4 실물 ROS2 단일 목표 검증 — 2026-07-13

[English](2026-07-13-m4-physical-ros2-control.en.md) · [인터페이스](../../ros2_ws/src/motionbrain_hardware_interface/README.md)

M4 어깨 한 축에서 작업자 확인을 거치는 실물 쓰기 경로를 검증했다.

```text
ForwardCommandController → m4_proposal → 타입이 정의된 제안
  → 20초 이내 1회 확인 → 현재 상태 재검사 → 인증된 /shoulder
  → ESP32 Dispatcher / SafetyGate → AS5600 목표각 제어
```

20초는 제안 확인 유효기간이며 구동 시간이 아니다. 제안만으로 명령을 보내지 않는다. 실행기는 ARMED·센서 최신성·목표 범위·제안 나이를 재검사하며 같은 제안은 한 번만 소비한다.

| 시험 | 결과 |
| --- | --- |
| 실물 단일 목표 | 시작248.20°, 목표250.00°, 종료249.96°, 오차−0.04°, `TARGET_REACHED`, 보정0 |
| 다른 축 | M1/M2/M3/M5 명령0 유지 |
| 중복 확인 | `proposal_already_consumed`, `forwarded=false` |
| IDLE 요청 | `state_not_armed`, `forwarded=false` |
| 실행기 재시작 | 실행기 하나와 확인 서비스 복구. 제안 제어기는 별도 실행 |

실행기 직접 경로의 보조 시험은 목표248.00°에 248.20°(+0.20°, 보정2)로 도달했다. 당시 소프트웨어 검증은 로컬 Python185/185, Pi ROS2 72개·실패0이었다.

`ForwardCommandController` 단일 목표 경로의 결과이며 궤적 추종·전체 관절 실물 쓰기·무인 실행의 근거는 아니다. 230–245° 반복 시험과 별개의 한 점 검증이다. 122.08–301.02°는 당시 자세 조건부 소프트 설정이며 전 구간 검증값이 아니다.
