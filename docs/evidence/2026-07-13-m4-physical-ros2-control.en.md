# M4 physical `ros2_control` one-shot evidence

As of 2026-07-13, the operator-confirmed physical-write path was validated on the M4 shoulder axis only.

## Contract

```text
ForwardCommandController -> m4_proposal hardware write
  -> typed non-forwarded proposal -> 20-second one-shot M4WriteConfirm
  -> live status revalidation -> authenticated /shoulder
  -> ESP32 Dispatcher / SafetyGate -> AS5600 closed loop
```

A proposal does not forward a motor command. The executor revalidates ARMED state,
sensor freshness, target range, and proposal age; each proposal can be consumed once.
Direct `http`/`physical` transports and full-arm writes remain disabled.

## Physical results

| Check | Result |
| --- | --- |
| Full physical write | 248.20 deg start, 250.00 deg target, 249.96 deg final, -0.04 deg error, `TARGET_REACHED`, correction 0 |
| Non-M4 outputs | M1/M2/M3/M5 commands remained zero |
| Replay | `proposal_already_consumed`, `forwarded=false` |
| IDLE smoke | `state_not_armed`, `forwarded=false` |
| systemd restart | Exactly one executor and `/motionbrain/m4_write_confirm` restored; proposal controller requires an explicit launch |
| Regression tests | Local Python 185/185; Pi ROS2 72 tests/0 failures |

An executor-direct supporting check moved from 249.96 deg toward 248.00 deg and ended
at 248.20 deg (+0.20 deg, correction 2).

## Claim boundary

This is evidence of a real M4 single-target hardware-write transaction.
`ForwardCommandController` deliberately avoids the stream of proposals that interpolated
trajectory commands would create. It is not physical trajectory tracking, full-arm
actuation, or unattended execution. The historical 230-245 deg interval remains the
strongest matrix-validated range. The write guard uses live soft limits, but the provisional
122.08-301.02 deg interval is posture-conditioned and not equivalently validated end to end.
