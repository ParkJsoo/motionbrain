# 2026-06-17 Runtime Measurement Evidence

[README](../../README.en.md) | [Robotics system readiness](../../ROBOTICS_SYSTEM_READINESS.en.md)

Read-only runtime measurements captured on the Raspberry Pi host. No physical
motion, motor command, or routine execution command was sent; ROS2
service/action calls used `action: status` only.

## Environment

| Item | Value |
| --- | --- |
| Capture time | `2026-06-17T00:32:15+09:00` |
| Host | `motionbrain-pi` |
| Kernel | `Linux motionbrain-pi 6.8.0-1057-raspi #61-Ubuntu SMP PREEMPT_DYNAMIC Tue May 26 22:12:44 UTC 2026 aarch64 aarch64 aarch64 GNU/Linux` |
| Git | `18c5c5b Restore embedded bench check evidence` |
| Worktree | `## main...origin/main` |
| Groups | `motionbrain adm dialout cdrom sudo audio video plugdev games users netdev render input gpio spi i2c` |
| Pi temperature | `temp=42.8'C` |
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

No USB oscilloscope, logic analyzer, USB serial adapter, or multimeter
interface was visible to the Pi during this capture. Physical PWM/UART/I2C
waveform and motor-voltage measurements therefore remain equipment-gated, not
software-gated.

## HTTP Endpoint Latency

Each endpoint was sampled 2 times from the Pi with a 0.8 s request timeout.

| Endpoint | URL | OK/fail | median ms | p95 ms | min ms | max ms | median bytes | status |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| ESP32 controller `/status` | `controller discovered endpoint` | 2/0 | 391.5 | 432.2 | 350.9 | 432.2 | 2538.0 | `200` |
| ESP32 controller `/routine` | `controller discovered endpoint` | 2/0 | 97.0 | 145.2 | 48.8 | 145.2 | 2932.0 | `200` |
| ESP32-CAM `/status` | `camera discovered endpoint` | 2/0 | 193.5 | 248.5 | 138.5 | 248.5 | 495.0 | `200` |
| Pi dashboard `/api/status` | `dashboard discovered endpoint` | 2/0 | 153.3 | 190.2 | 116.3 | 190.2 | 2533.0 | `200` |
| Pi dashboard `/api/config` | `dashboard discovered endpoint` | 2/0 | 4.9 | 5.1 | 4.7 | 5.1 | 332.0 | `200` |
| Pi perception `/health` | `perception discovered endpoint` | 2/0 | 15.7 | 22.5 | 8.8 | 22.5 | 715.5 | `200` |
| Pi perception `/api/detection` | `perception discovered endpoint` | 2/0 | 14.2 | 15.0 | 13.4 | 15.0 | 751.0 | `200` |

Direct ESP32-CAM `/status` discovery returned a reachable endpoint.

## ROS2 Graph And Topic Acquisition

Before this capture, a 4 s per-topic probe timed out even though graph
discovery showed publishers for the sampled topics. A 15 s bounded acquisition
was therefore used for CLI-based evidence.

`ros2 topic echo --once` was sampled 1 time per topic with a 15.0 s per-sample
timeout.

| Topic | OK/fail | median ms | p95 ms | min ms | max ms | result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `/motionbrain/status_typed` | 1/0 | 11171.2 | 11171.2 | 11171.2 | 11171.2 | `` |
| `/camera/detection_typed` | 1/0 | 11431.2 | 11431.2 | 11431.2 | 11431.2 | `` |
| `/joint_states` | 1/0 | 10226.3 | 10226.3 | 10226.3 | 10226.3 | `` |
| `/motionbrain/control_guard_typed` | 1/0 | 9902.9 | 9902.9 | 9902.9 | 9902.9 | `` |
| `/motionbrain/mission_state_typed` | 1/0 | 11271.9 | 11271.9 | 11271.9 | 11271.9 | `` |

All configured topic probes returned one sample within the bounded timeout.
The latency values include ROS2 CLI startup, environment setup, DDS discovery,
subscription matching, and first-sample wait; they are not publisher periods.
A separate `ros2 topic hz /joint_states --window 3` check reported approximately
4.9 to 5.0 Hz.

## ROS2 Status Round Trip

| Check | rc | elapsed ms | success=true |
| --- | ---: | ---: | --- |
| routine service status | 0 | 9919.7 | `true` |
| guarded routine action status | 0 | 9712.6 | `true` |

Both ROS2 status probes completed successfully within the bounded timeout.

## Physical Measurement Status

| Signal | Status | Reason |
| --- | --- | --- |
| ESP32 PWM frequency/duty | not captured | no visible oscilloscope/logic analyzer/sigrok device on Pi |
| STM32-to-ESP32 UART timing | not captured | no USB serial adapter or logic analyzer visible to Pi |
| MPU-6050 I2C waveform | not captured | Pi I2C bus is available, but the STM32 sensor bus is not proven wired to Pi and should not be probed blindly |
| Deadman release-to-stop latency | not captured | needs synchronized physical input/video or logic capture |
| Motor voltage drop under bounded pulse | not captured | needs a meter/scope connected across motor supply during a safe bounded pulse |

## Correct Claim

```text
Captured read-only runtime measurements for MotionBrain on the live Raspberry Pi
host: HTTP endpoint latency, bounded ROS2 topic acquisition, ROS2 routine status
service/action round trips, Pi health, and hardware-instrument inventory. Topic
latencies include ROS2 CLI startup and DDS discovery overhead. Physical waveform
and voltage measurements still require external instruments.
```
