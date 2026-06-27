# M4 Shoulder AS5600 Closed-Loop Bench Evidence — 2026-06-28

[한국어](2026-06-28-m4-shoulder-closed-loop.md)

## Scope

This bench run validates one absolute-position feedback loop on the M4 shoulder.
It does not claim position feedback on the other four joints or full-arm closed-loop control.

## Hardware path

- Sensor: AS5600 magnetic angle sensor, address `0x36`
- Joint: M4 shoulder
- ESP32 temporary I2C bus: SDA `GPIO0`, SCL `GPIO15`
- Analog OUT remained connected to VP/GPIO36 for comparison
- Magnet and sensor were mounted so they rotate relative to each other with the shoulder joint

The VP analog path saturated at `raw=4095`, approximately `3145mV`, and was not used for control.
I2C exposed the actual failure mode during initial alignment: `MD=NO`, `ML=YES`, magnitude near zero.
Moving and centering the magnet produced `MD=YES` and a continuously changing 12-bit angle.

## Mounted open-loop characterization

With the sensor taped in its trial mount:

| Action | Start | End | Delta | Magnet status |
| --- | ---: | ---: | ---: | --- |
| M4 up, 100%, 1s | 233.79 deg | 241.96 deg | +8.17 deg | `MD=YES ML=NO MH=NO` |
| M4 down, 100%, 1s | 241.96 deg | 232.73 deg | -9.23 deg | `MD=YES ML=NO MH=NO` |

Observed magnitude remained approximately `1917-2055` during this run.

## Controller safety envelope

The first implementation deliberately limits absolute targets to the already exercised range:

- temporary soft limits: `230-245 deg`
- target tolerance: `0.35 deg`
- sensor freshness limit: `150ms`
- command timeout: `5s`
- no-progress stop
- immediate M4 output cutoff on stale/disconnected I2C, missing magnet, weak/strong magnet, safety block, state change, timeout, or soft-limit violation

Manual M4 motor/joint commands, shoulder teleoperation input, and sequence commands cancel active shoulder angle control. A new shoulder target is rejected while a sequence is running.

## Coast/backlash characterization

Immediate output cutoff still allowed repeatable direction-dependent mechanical settling:

| Direction | Controller cutoff | Settled position | Additional travel |
| --- | ---: | ---: | ---: |
| Up toward 238 deg | 237.66 deg | 238.54-238.62 deg | +0.88 to +0.96 deg |
| Down toward 234 deg | 234.23-234.32 deg | 232.73 deg | -1.50 to -1.59 deg |

The controller therefore applies provisional stop leads of `0.90 deg` upward and `1.50 deg` downward,
then waits `600ms` and records the settled sensor position.

## Compensated closed-loop result

| Command | Start | Motor cutoff | Settled result | Final error |
| --- | ---: | ---: | ---: | ---: |
| `shoulder angle 238 100` | 232.73 deg | 236.87 deg | 238.10 deg | -0.10 deg |
| `shoulder angle 234 100` | 238.10 deg | 235.72 deg | 233.96 deg | +0.04 deg |

Both runs ended with `TARGET_REACHED`; I2C stayed fresh and magnet status stayed valid.

## Remaining limits

- The taped mount is suitable for bring-up, not long-term repeatability or vibration testing.
- GPIO0/GPIO15 are temporary pins; GPIO0 is a boot strap and requires a permanent pin-allocation decision.
- Only a narrow, supervised shoulder range has been calibrated.
- Directional stop-lead values need repeated trials across load and battery-voltage conditions.
- The analog OUT/VP path is still saturated and is intentionally excluded from control.

## Reproduction commands

```text
arm
shoulder status
shoulder angle 238 100
shoulder status
shoulder angle 234 100
shoulder status
disarm
```
