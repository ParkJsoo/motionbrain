# 2026-06-16 Embedded Bench Check Evidence

[README](../../README.en.md) | [Robotics system readiness](../../ROBOTICS_SYSTEM_READINESS.en.md)

This note restores public-safe embedded bench evidence from prior repository
history. It documents digital-multimeter-level sanity checks and the claim
boundary around them. It is not oscilloscope, logic-analyzer, or closed-loop
control evidence.

## Source Trace

The recovered source is commit `2438801` (`Document embedded firmware evidence`),
which introduced `docs/EMBEDDED_FIRMWARE_EVIDENCE.md` with a multimeter-based
bench-check section. The original note also mapped the project to a specific
application context; that context is intentionally removed here. This public
note keeps only reusable project evidence and limitations.

## Bench Scope

| Area | Recovered bench evidence | Claim boundary |
| --- | --- | --- |
| Common ground | STM32 GND, ESP32 GND, TB6612FNG GND, and the external supply negative rail were checked as a shared ground path | Continuity sanity check only |
| Short checks | ESP32 `3V3` to GND and TB6612FNG `VM` to GND were checked for obvious shorts before power-up | Does not prove load behavior |
| Logic rails | ESP32 and STM32 `3V3` rails were checked for normal DC level after power-up | No ripple, noise, or transient claim |
| Motor-driver rails | TB6612FNG `VCC` was checked against the logic rail, and `VM` was checked against the intended external supply or XL4015 output | No motor transient sag claim |
| Active-low buttons | Button wiring to STM32 GPIO and GND was checked; unpressed/pressed states were checked as HIGH/LOW DC levels | No debounce or timing claim |
| Output sanity | Conservative nudge or light-toggle commands changed the related output rail before/after command | No PWM duty/frequency or waveform-integrity claim |

## Not Supported By This Evidence

- UART bit timing or edge integrity
- PWM duty cycle, PWM frequency, or motor-drive waveform quality
- I2C rise time, bus capacitance margin, or signal integrity
- Motor supply transient sag under load
- Encoder-grade joint feedback or closed-loop motion control
- Production safety-channel validation

## Correct Claim

```text
Recovered historical bench evidence supports multimeter-level power, ground,
button, motor-driver rail, and output sanity checks on the embedded hardware.
It does not support waveform timing, PWM duty/frequency, transient motor-voltage,
or closed-loop joint-control claims.
```
