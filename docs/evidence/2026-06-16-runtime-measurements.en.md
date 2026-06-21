# 2026-06-16 Runtime Measurement Evidence

[README](../../README.en.md) | [Robotics system readiness](../../ROBOTICS_SYSTEM_READINESS.en.md)

Read-only runtime measurements captured on the Raspberry Pi host. No physical motion,
motor command, or routine execution command was sent; ROS2 service/action calls used
`action: status` only.

## Environment

| Item | Value |
| --- | --- |
| Capture time | `2026-06-16T23:39:11+09:00` |
| Host | `motionbrain-pi` |
| Kernel | `Linux motionbrain-pi 6.8.0-1057-raspi #61-Ubuntu SMP PREEMPT_DYNAMIC Tue May 26 22:12:44 UTC 2026 aarch64 aarch64 aarch64 GNU/Linux` |
| Git | `37be6c5 Clarify runtime probe timeout evidence` |
| Worktree | `## main...origin/main` |
| Groups | `motionbrain adm dialout cdrom sudo audio video plugdev games users netdev render input gpio spi i2c` |
| Pi temperature | `temp=49.1'C` |
| Pi throttling | `throttled=0x50005` |

## Instrument Inventory

| Item | Observed |
| --- | --- |
| USB devices | `Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub; Bus 001 Device 002: ID 2109:3431 VIA Labs, Inc. Hub; Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub; Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub` |
| USB serial devices | `none detected` |
| I2C devices | `/dev/i2c-1, /dev/i2c-20, /dev/i2c-21` |
| GPIO chips | `/dev/gpiochip0, /dev/gpiochip1` |
| `sigrok-cli` | `not installed` |
| `pulseview` | `not installed` |
| `i2cdetect` | `not installed` |
| `gpioinfo` | `not installed` |
| `gpiomon` | `not installed` |
| `pigpiod` | `not installed` |
| `pigs` | `not installed` |
| `vcgencmd` | `/usr/bin/vcgencmd` |

No USB oscilloscope, logic analyzer, USB serial adapter, or multimeter interface
was visible to the Pi during this capture. Physical PWM/UART/I2C waveform and
motor-voltage measurements therefore remain equipment-gated, not software-gated.

## HTTP Endpoint Latency

Each endpoint was sampled 2 times from the Pi with a 0.7 s request timeout.

| Endpoint | URL | OK/fail | median ms | p95 ms | min ms | max ms | median bytes | status |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Pi dashboard /api/status | `dashboard discovered endpoint` | 2/0 | 318.5 | 517.5 | 119.4 | 517.5 | 2533.0 | `200` |
| Pi dashboard /api/config | `dashboard discovered endpoint` | 2/0 | 11.2 | 12.1 | 10.3 | 12.1 | 332.0 | `200` |
| Pi perception /health | `perception discovered endpoint` | 2/0 | 9.5 | 9.7 | 9.3 | 9.7 | 717.0 | `200` |
| Pi perception /api/detection | `perception discovered endpoint` | 2/0 | 7.0 | 7.8 | 6.3 | 7.8 | 753.0 | `200` |

Direct ESP32-CAM `/status` discovery did not return during this capture; camera evidence is represented through dashboard/perception endpoints.

## ROS2 Topic Sample Latency

`ros2 topic echo --once` was sampled 1 times per topic with a 1.0 s per-sample timeout.

| Topic | OK/fail | median ms | p95 ms | min ms | max ms | result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `/motionbrain/status_typed` | 0/1 | n/a | n/a | n/a | n/a | `` |
| `/camera/detection_typed` | 0/1 | n/a | n/a | n/a | n/a | `` |
| `/joint_states` | 0/1 | n/a | n/a | n/a | n/a | `` |
| `/motionbrain/control_guard_typed` | 0/1 | n/a | n/a | n/a | n/a | `` |
| `/motionbrain/mission_state_typed` | 0/1 | n/a | n/a | n/a | n/a | `` |

All topic probes hit the bounded timeout in this capture; no ROS2 message
sample latency value was captured.

## ROS2 Status Round Trip

| Check | rc | elapsed ms | success=true |
| --- | ---: | ---: | --- |
| routine service status | 124 | 5131.9 | `false` |
| guarded routine action status | 124 | 5045.8 | `false` |

Both ROS2 status probes hit the bounded timeout in this capture.

## Physical Measurement Status

| Signal | Status | Reason |
| --- | --- | --- |
| ESP32 PWM frequency/duty | not captured | no visible oscilloscope/logic analyzer/sigrok device on Pi |
| STM32-to-ESP32 UART timing | not captured | no USB serial adapter or logic analyzer visible on Pi |
| MPU-6050 I2C waveform | not captured | Pi I2C bus is available, but the STM32 sensor bus is not proven wired to Pi and should not be probed blindly |
| Deadman release-to-stop latency | not captured | needs synchronized physical input/video or logic capture |
| Motor voltage drop under bounded pulse | not captured | needs a meter/scope connected across motor supply during a safe bounded pulse |

## Correct Claim

```text
Captured read-only runtime measurements for MotionBrain on the live Raspberry Pi host:
HTTP endpoint latency, bounded ROS2 topic/status CLI probes, Pi health, and
hardware-instrument inventory. In this capture, ROS2 CLI probes timed out
before returning message/status data. Physical waveform and voltage measurements
still require external instruments.
```
