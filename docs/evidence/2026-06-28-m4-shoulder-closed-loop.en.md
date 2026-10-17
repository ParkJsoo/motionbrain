# M4 Shoulder Closed-Loop Validation — 2026-06-28

[Korean](2026-06-28-m4-shoulder-closed-loop.md) · [Portfolio](../../PORTFOLIO.en.md)

Target control was evaluated on the AS5600-equipped M4 shoulder only. These measurements use the fixed mount from that date, not the current pose or the other four joints. The validated range is 230–245°; wider firmware soft limits are not established as safe by this test.

## Setup

- AS5600 I2C address `0x36`, SDA `GPIO0`, SCL `GPIO15`. [Pin map](../../PIN_MAP.en.md)
- Analog OUT saturated at `raw=4095`, about 3145mV, and was excluded from control.
- Initial magnet misalignment produced `MD=NO`, `ML=YES` and near-zero magnitude. Realignment/fixing restored normal magnet status and continuous angles.
- Remounting mapped raw 258.93° to the prior 234.58° coordinate using −24.35°. This offset is not reusable for another mount.
- Acceptance ±0.50°, sensor freshness 150ms, command timeout 7s. Sensor/magnet faults, no progress, safety blocks and limit violations stop output.

## Failures and corrections

| Problem | Evidence | Correction |
| --- | --- | --- |
| Coasting after output-off | Upward +0.88 to +0.96°; downward −1.50 to −1.59° | Direction-dependent stop lead and a 600ms settled check |
| False success despite target miss | A 234° request stopped at 235.37° but returned `TARGET_REACHED` | Recheck settled error, bounded corrections, `TARGET_MISSED` on failure |
| Manual input exceeded the narrow range after target completion | Reached 256.99° in a test | Shared M4 sensor/directional-margin guard for direct, sequence and teleoperation paths |
| Short upward motion failed at 23.10g | 236° request ended at 235.10°, +0.90° error after four corrections | Upward correction pulse increased 250→500ms so the PWM ramp could overcome static friction |
| One sensor step crossed the success boundary | 234.49° success became 234.58° after one second | Retain external ±0.50°, tighten internal settled success to ±0.40° |

Sensors and other-axis outputs were normal during the loaded failure. No-load compensation and pulse limitations were identified, but load and supply effects were not fully isolated because voltage was not measured during that attempt. Failed attempts are not counted in the final success sample.

## Final regression

Each condition used five 232–243° targets at 75/100% plus six 238↔234° moves.

| Condition | Result | Mean absolute error | Maximum error | Maximum corrections | Motion samples |
| --- | ---: | ---: | ---: | ---: | ---: |
| No added load | 11/11 | 0.132° | 0.36° | 2 | 214 |
| 23.10g (three KRW 500 coins) | 11/11 | 0.155° | 0.31° | 2 | 201 |

No safety blocks, IMU/AS5600 faults or non-M4 activation were recorded in either sample. Maximum AS5600 age was 36ms unloaded and 39ms loaded. An earlier 22/22 no-added-load result preceded the final success-window change and is not pooled with this table as same-version repetition.

## Power and boot recovery

Approximate measurements used a basic multimeter and phone video with the same 23.10g load. Battery terminals measured 5.97–5.98V idle and 5.86V minimum during ascent. In a separate ascent, XL4015 output measured 5.98→5.62V. These were neither simultaneous nor MIN/MAX captures; approximate drops of 2.0% and 6.0% do not establish a power root cause.

An STM32 MPU-6050 initialization failure reported `imuError=0x20` and SDA LOW while motion was blocked. Recovery was extended to I2C reinitialization/bus recovery up to four times at boot and every five seconds afterward. Six hardware resets and one power cycle recovered successfully; wiring, pull-ups and intermittent sensor faults remain possible.

## Limits

- One axis, fixed mount, small load and finite samples. Long-term repetition, vibration, different battery voltages and all-pose clearance remain unverified.
- GPIO0/GPIO15 boot conditions and the mounted sensor zero require separate checks.
- Output records and sensor angles do not measure electrical cutoff latency, gripping force or whole-arm trajectory tracking.
- Original measurements/execution records are retained locally; this public note provides conditions and aggregate results.
