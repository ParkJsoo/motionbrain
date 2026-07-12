# M4 physical `ros2_control` one-shot 검증

2026-07-13 기준으로 M4 어깨 한 축에 한해 operator-confirmed physical write 경로를 실물 검증했다.

## 계약

```text
ForwardCommandController -> m4_proposal hardware write
  -> typed non-forwarded proposal -> 20초 one-shot M4WriteConfirm
  -> live status 재검증 -> authenticated /shoulder
  -> ESP32 Dispatcher / SafetyGate -> AS5600 closed loop
```

proposal은 모터 명령을 자동 전달하지 않는다. executor는 ARMED 상태, 센서 freshness,
목표 범위, proposal age를 다시 확인하며 같은 proposal은 한 번만 소비한다. direct
`http`/`physical` transport와 full-arm write는 비활성화 상태다.

## 실물 결과

| 항목 | 결과 |
| --- | --- |
| 전체 physical write | 시작 248.20°, 목표 250.00°, 종료 249.96°, 오차 -0.04°, `TARGET_REACHED`, correction 0 |
| 비-M4 출력 | M1/M2/M3/M5 command 0 유지 |
| replay | `proposal_already_consumed`, `forwarded=false` |
| IDLE smoke | `state_not_armed`, `forwarded=false` |
| systemd 재시작 | executor 정확히 1개와 `/motionbrain/m4_write_confirm` 자동 복구; proposal controller는 별도 명시 launch |
| 회귀 테스트 | 로컬 Python 185/185, Pi ROS2 72 tests/0 failures |

executor-direct 보조 검증에서는 249.96°에서 248.00° 목표로 이동해 248.20°
(+0.20°, correction 2)에 도달했다.

## 주장 경계

이는 M4 single-target hardware write 증거다. 보간 command가 여러 proposal을 만드는
것을 피하려고 `ForwardCommandController`를 사용했으므로 physical trajectory tracking,
full-arm actuation, unattended execution을 뜻하지 않는다. 230-245°는 과거 matrix 검증
범위다. write guard는 live soft min/max를 사용하지만 122.08-301.02°는 자세 조건부
임시 범위이며 전 구간이 동등하게 검증됐다는 뜻이 아니다.
