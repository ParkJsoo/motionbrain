# M4 Physical ROS2 Single-Target Validation — 2026-07-13

[Korean](2026-07-13-m4-physical-ros2-control.md) · [Interface](../../ros2_ws/src/motionbrain_hardware_interface/README.en.md)

An operator-confirmed physical write path was validated on the M4 shoulder only.

```text
ForwardCommandController → m4_proposal → typed proposal
  → one confirmation within 20 seconds → live-state recheck → authenticated /shoulder
  → ESP32 Dispatcher / SafetyGate → AS5600 target control
```

Twenty seconds is the proposal-confirmation window, not motion duration. Proposals alone send no command. The executor rechecks ARMED state, sensor freshness, target bounds and proposal age, consuming each proposal only once.

| Test | Result |
| --- | --- |
| Physical single target | Start248.20°, target250.00°, final249.96°, error−0.04°, `TARGET_REACHED`, correction0 |
| Other axes | M1/M2/M3/M5 commands remained0 |
| Duplicate confirmation | `proposal_already_consumed`, `forwarded=false` |
| IDLE request | `state_not_armed`, `forwarded=false` |
| Executor restart | One executor and confirmation service recovered; proposal controller launched separately |

An auxiliary executor-direct test reached 248.20° for a 248.00° target (+0.20°, two corrections). Software validation at that time passed local Python185/185 and Pi ROS2 72 tests with zero failures.

This is a `ForwardCommandController` single-target result, not trajectory tracking, whole-arm physical writes or unattended execution. It is a separate point test from the 230–245° repetition matrix. The 122.08–301.02° soft setting was pose-conditional at that time, not a validated whole range.
