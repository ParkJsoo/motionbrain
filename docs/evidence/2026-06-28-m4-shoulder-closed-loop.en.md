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
- final target tolerance: `+/-0.50 deg` (the initial coast-stop window remains `0.35 deg`)
- sensor freshness limit: `150ms`
- command timeout: `7s`
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

## Fixed-mount repeatability and settled-error validation

Repeated targets after mechanical fastening exposed an important controller
defect. The first short 235.81-to-234 deg move stopped at 235.37 deg, but the
controller returned `TARGET_REACHED` without checking the settled error again.
Before the fix, three 238-to-234 deg cycles produced upward errors of
`+0.26/+0.17/-0.27 deg` and downward errors of `-0.75/-0.58/-0.14 deg`.
Magnitude stayed at `2095-2106` and maximum sensor age was `37ms`, ruling out a
sensor-health failure.

The revised controller now:

- rechecks final error after settling instead of returning unconditional success;
- applies bounded correction pulses: 75%/500ms upward and 35%/250ms downward,
  at most four attempts;
- stops as `TARGET_MISSED` if the result remains outside `+/-0.50 deg`;
- exposes correction state, attempts, tolerance, and failure reason through HTTP, dashboard, and ROS2.

Wide-range regression under the current load and battery state:

| Speed | Target | Start | Stable after 1s | Final error | Corrections |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 100% | 236 deg | 243.10 deg | 235.72 deg | +0.28 deg | 0 |
| 100% | 243 deg | 235.72 deg | 243.36 deg | -0.36 deg | 1 |
| 75% | 240 deg | 243.36 deg | 240.20 deg | -0.20 deg | 1 |
| 75% | 232 deg | 240.20 deg | 232.20 deg | -0.20 deg | 1 |
| 100% | 236 deg | 232.20 deg | 235.98 deg | +0.02 deg | 0 |

All 5 runs passed. Mean absolute error was `0.212 deg`, maximum absolute error
was `0.36 deg`, travel covered `3.36-8.20 deg`, magnitude stayed at
`2091-2105`, and maximum sensor age was `41ms`. No additional drift was
observed one second after command completion.

The final three 238-to-234 deg cycles also passed 6/6, with `0.213 deg` mean
absolute error, `0.44 deg` maximum absolute error, and at most four correction
attempts. Final state was `IDLE`, M1-M5 speed 0, M4 234.05 deg, sensor ready,
AGC 100, and magnitude 2100.

## Independent repeatability rerun

The complete 11-run matrix was repeated with the same fixed mount and current
no-added-load setup. Preflight initially found a STM32 MPU-6050 boot probe
failure: `IMU_FAULT`, `imuStatus=2`, I2C timeout `0x20`, and SDA LOW. SafetyGate
blocked motion, and no motion command was sent while the block was active.
After a STM32 reset, the MPU-6050 recovered at address `0x68` with error 0,
SCL/SDA HIGH, and `blockReason=NONE`; validation started only after those checks.

| Group | Speed | Target | Start | Stable after 1s | Final error | Corrections |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Wide | 100% | 236 deg | 233.96 deg | 235.72 deg | +0.28 deg | 2 |
| Wide | 100% | 243 deg | 235.72 deg | 243.19 deg | -0.19 deg | 1 |
| Wide | 75% | 240 deg | 243.19 deg | 240.29 deg | -0.29 deg | 0 |
| Wide | 75% | 232 deg | 240.29 deg | 232.29 deg | -0.29 deg | 2 |
| Wide | 100% | 236 deg | 232.29 deg | 235.89 deg | +0.11 deg | 0 |
| Repeat | 100% | 238 deg | 235.98 deg | 237.83 deg | +0.17 deg | 0 |
| Repeat | 100% | 234 deg | 237.83 deg | 234.05 deg | -0.05 deg | 0 |
| Repeat | 100% | 238 deg | 234.05 deg | 238.18 deg | -0.18 deg | 0 |
| Repeat | 100% | 234 deg | 238.18 deg | 234.31 deg | -0.31 deg | 0 |
| Repeat | 100% | 238 deg | 234.31 deg | 238.09 deg | -0.09 deg | 0 |
| Repeat | 100% | 234 deg | 238.09 deg | 234.14 deg | -0.14 deg | 0 |

The five wide-range runs had `0.232 deg` mean absolute error and `0.29 deg`
maximum error. The six repeated-cycle runs had `0.157 deg` mean absolute error
and `0.31 deg` maximum error. All 11 runs passed, with `0.191 deg` overall mean
absolute error, `0.31 deg` maximum absolute error, and at most two correction
attempts. Across 184 in-motion samples there were no non-M4 motor activations,
safety blocks, IMU failures, or AS5600 readiness failures. AS5600 magnitude was
`2093-2107`, and maximum sensor age was `37ms`.

Final state was `IDLE`, M1-M5 stopped, M4 234.14 deg, AS5600 ready, MPU-6050
healthy, and no latched fault. The two complete post-fix matrices on the same
fixed mount therefore passed 22/22 in total. Neither run used a known added
load or separately instrumented battery voltage.

## 23.10 g load validation and correction update

With three 500 KRW coins held directly by the gripper (`23.10 g`), a 75%
238-to-234 deg retention smoke passed 2/2 with final errors of `+0.35/-0.40 deg`.
The first short upward move in the full matrix then commanded 236 deg from
234.40 deg but settled at 235.10 deg. It ended as `TARGET_MISSED` with
`+0.90 deg` error after four corrections, and no later matrix step was run.

Loaded coast after the initial cutoff was only 0.08 deg, which did not match
the 0.90 deg upward stop lead characterized without a known load. Each 250 ms
correction also restarted the shared motor ramp from zero; PWM speed reached
only 40-50 before cutoff, and the later pulses did not overcome static friction.
AS5600 and STM32 safety inputs stayed healthy, and no other motor moved. Battery
voltage was not instrumented, so load and supply-voltage effects cannot be
fully separated.

Failure cleanup left the system `IDLE`, M1-M5 stopped, M4 235.10 deg, sensors
healthy, and no latched fault. This does not prove that the arm cannot carry
23.10 g; it shows that the unloaded short-move coast compensation and ramped
correction pulse are not valid for this loaded case.

The soft limits, `+/-0.50 deg` tolerance, sensor target cutoff, 7 s timeout,
and four-attempt cap were preserved. Only the upward correction pulse maximum
was increased from 250 to 500 ms, giving MotorControl's 50 ms/10-count PWM ramp
time to cross loaded static friction. After the change, a 75% 238-to-234 deg
smoke passed 2/2 before the complete matrix was retried.

| Group | Result | Mean absolute error | Maximum absolute error | Maximum corrections |
| --- | ---: | ---: | ---: | ---: |
| 232-243 deg / 75-100% | 5/5 | 0.328 deg | 0.47 deg | 3 |
| Repeated 238-to-234 deg | 6/6 | 0.245 deg | 0.49 deg | 1 |
| Overall | 11/11 | 0.283 deg | 0.49 deg | 3 |

The first short 236 deg target changed from `235.10 deg/TARGET_MISSED` before
the fix to `236.42 deg/TARGET_REACHED` afterward. Across 180 motion samples
there were no safety blocks, IMU/AS5600 failures, or non-M4 motor activations.
AS5600 magnitude was `2093-2111`, and maximum age was `36ms`. Final state was
`IDLE`, M1-M5 stopped, M4 234.14 deg, sensors healthy, and no latched fault.

## Boundary success margin and final regressions

The following no-added-load regression exposed another boundary defect on its
ninth target. Firmware returned `TARGET_REACHED` at 234.49 deg, inside the
`+/-0.50 deg` acceptance band, but one AS5600 step of variation produced
234.58 deg one second later. Instead of loosening external acceptance to
0.60 deg, the external `+/-0.50 deg` contract was preserved and internal
success was tightened to `+/-0.40 deg`. HTTP status exposes these separately as
`targetToleranceDeg=0.50` and `settledSuccessToleranceDeg=0.40`.

Final complete regressions without and with the 23.10 g load:

| Condition | Result | Mean absolute error | Maximum absolute error | Maximum corrections | Motion samples |
| --- | ---: | ---: | ---: | ---: | ---: |
| No added load | 11/11 | 0.132 deg | 0.36 deg | 2 | 214 |
| 23.10 g | 11/11 | 0.155 deg | 0.31 deg | 2 | 201 |

Both runs had zero safety blocks, IMU/AS5600 failures, or non-M4 motor
activations. No-load AS5600 magnitude was `2093-2107` with `36ms` maximum age;
the 23.10 g run had magnitude `2094-2108` and `39ms` maximum age. Final loaded
state was `IDLE`, M1-M5 stopped, M4 234.31 deg, sensors healthy, and no latched
fault.

## Remaining limits

- The sensor and magnet are secured, but long-term repeatability and vibration conditions remain unvalidated.
- GPIO0/GPIO15 is the supported allocation under the current pin budget but
  requires boot-strap discipline. GPIO0 must not be held LOW at reset, and boot
  and upload regression must be repeated after wiring changes.
- The `-24.35 deg` mount offset is valid only for the current fixed mount and must be recalibrated after remounting.
- Only a narrow, supervised shoulder range has been calibrated.
- Directional stop-lead values need repeated trials across load and battery-voltage conditions.
- The two complete repeatability runs used the same current mount and no-added-load
  condition. The final controller was rerun 11/11 both without load and at
  23.10 g, but larger loads and measured battery-voltage cases remain untested.
- The STM32 MPU-6050 boot probe failure recovered after reset. If it recurs,
  power, SDA/SCL wiring, pull-ups, boot order, and I2C bus recovery need a
  separate investigation.
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
