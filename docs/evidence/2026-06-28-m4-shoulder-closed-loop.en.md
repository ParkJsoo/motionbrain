# M4 Shoulder AS5600 Closed-Loop Bench Evidence — 2026-06-28

[한국어](2026-06-28-m4-shoulder-closed-loop.md)

## Scope

This bench run validates one absolute-position feedback loop on the M4 shoulder.
It does not claim position feedback on the other four joints or full-arm closed-loop control.

## Hardware path

- Sensor: AS5600 magnetic angle sensor, address `0x36`
- Joint: M4 shoulder
- ESP32 supported I2C allocation: SDA `GPIO0`, SCL `GPIO15` ([pin map](../../PIN_MAP.en.md))
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

## Final deployment regression after remount

After mechanically securing the sensor and magnet, magnetic quality improved to `AGC=102-104`,
magnitude approximately `2107-2136`, `MD=YES`, `ML=NO`, and `MH=NO`. The new
mount orientation made raw `258.93 deg` correspond to the established shoulder
coordinate `234.58 deg`, so a provisional `-24.35 deg` mount offset was added.
Diagnostics retain both raw and calibrated angles.

Target regression after redeployment:

| Command | Start | Settled result | Final error |
| --- | ---: | ---: | ---: |
| `shoulder angle 238 100` | 234.58 deg | 238.09 deg | -0.09 deg |
| `shoulder angle 234 100` | 238.09 deg | 234.14 deg | -0.14 deg |

Conflict-path regression:

- A new absolute target was rejected while an M4 1% sequence was running with
  `Stop active sequence before shoulder angle control`.
- Shoulder teleoperation input cancelled an active absolute target as
  `OVERRIDDEN` and hard-stopped M4 within 53 ms.
- The first teleoperation timing attempt exposed that manual input arriving
  after target completion could continue beyond the closed-loop trial range.
  M4 reached 256.99 deg, was stopped/disarmed immediately, and was returned
  under supervision to 235.89 deg.
- The resulting fix makes direct M4, sequence, and teleoperation paths share
  AS5600 readiness and direction-aware coast margins. Manual boundaries are
  244.10 deg upward and 231.50 deg downward; when outside the range, only
  motion back toward the safe range is allowed.

Final state: `IDLE`, M1-M5 speed 0, M4 235.89 deg, sensor `ready=YES`,
`ML=NO`, and no latched fault.

## Remaining limits

- The sensor and magnet are secured, but long-term repeatability and vibration conditions remain unvalidated.
- GPIO0/GPIO15 is the supported allocation under the current pin budget but
  requires boot-strap discipline. GPIO0 must not be held LOW at reset, and boot
  and upload regression must be repeated after wiring changes.
- The `-24.35 deg` mount offset is valid only for the current trial mount and must be recalibrated after remounting.
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
