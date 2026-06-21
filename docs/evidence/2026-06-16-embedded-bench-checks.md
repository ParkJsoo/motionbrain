# 2026-06-16 Embedded Bench Check 증거

[README](../../README.md) | [로보틱스 시스템 준비도](../../ROBOTICS_SYSTEM_READINESS.md) | [English](2026-06-16-embedded-bench-checks.en.md)

이 문서는 과거 repository history에 있던 embedded bench evidence를 공개 문서로
다시 정리한 것이다. 범위는 digital multimeter 수준의 sanity check이며,
oscilloscope, logic analyzer, closed-loop control 증거는 아니다.

## 출처

복구한 원천은 commit `2438801` (`Document embedded firmware evidence`)이다.
이 commit은 `docs/EMBEDDED_FIRMWARE_EVIDENCE.md`에 multimeter 기반 bench check
섹션을 추가했다. 원문에는 특정 지원 맥락도 섞여 있었기 때문에, 이 공개 문서에는
재사용 가능한 project evidence와 한계만 남겼다.

## Bench 범위

| 영역 | 복구된 bench evidence | 주장 경계 |
| --- | --- | --- |
| 공통 GND | STM32 GND, ESP32 GND, TB6612FNG GND, 외부 전원 음극 rail을 shared ground path로 확인 | continuity sanity check만 의미함 |
| Short check | 전원 인가 전 ESP32 `3V3`-GND, TB6612FNG `VM`-GND obvious short 확인 | load behavior를 증명하지 않음 |
| Logic rail | 전원 인가 후 ESP32/STM32 `3V3` rail의 DC level 확인 | ripple, noise, transient claim 없음 |
| Motor-driver rail | TB6612FNG `VCC`를 logic rail 기준으로, `VM`을 외부 전원 또는 XL4015 출력 기준으로 확인 | motor transient sag claim 없음 |
| Active-low button | STM32 GPIO와 GND로 연결된 button wiring을 확인하고 unpressed/pressed HIGH/LOW DC level 확인 | debounce나 timing claim 없음 |
| Output sanity | 보수적인 nudge 또는 light-toggle command 전후로 관련 output rail 변화 확인 | PWM duty/frequency 또는 waveform integrity claim 없음 |

## 이 증거로 주장하면 안 되는 것

- UART bit timing 또는 edge integrity
- PWM duty cycle, PWM frequency, motor-drive waveform quality
- I2C rise time, bus capacitance margin, signal integrity
- 부하 상태 motor supply transient sag
- Encoder-grade joint feedback 또는 closed-loop motion control
- Production safety-channel validation

## 올바른 주장

```text
과거 bench evidence는 embedded hardware의 power, ground, button,
motor-driver rail, output sanity를 multimeter 수준에서 확인했다는 증거다.
이 증거는 waveform timing, PWM duty/frequency, transient motor voltage,
closed-loop joint control 주장을 뒷받침하지 않는다.
```
