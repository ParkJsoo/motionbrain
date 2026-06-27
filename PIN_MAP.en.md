# MotionBrain ESP32 Pin Map

[한국어](PIN_MAP.md)

This document records the active ESP32 five-axis controller pin allocation and
the M4 AS5600 policy. Build flags override the source defaults when present.

## Active allocation

| GPIO | Direction/function | Connection | Note |
| ---: | --- | --- | --- |
| 0 | I2C SDA | M4 AS5600 SDA | boot strap; must remain HIGH at reset |
| 1, 3 | UART0 | USB serial/programming | do not reassign |
| 4, 13, 14 | output | M5 direction pair and PWM | TB6612FNG #3 motor A |
| 5 | output | search light | boot strap |
| 6-11 | internal flash | ESP32 module flash | do not use |
| 15 | I2C SCL | M4 AS5600 SCL | boot strap; unused TB6612 B constant only |
| 16, 17, 18 | output | M1 direction pair and PWM | TB6612FNG #1 motor A |
| 19, 21, 22 | output | M2 direction pair and PWM | TB6612FNG #1 motor B |
| 23, 25, 26 | output | M3 direction pair and PWM | TB6612FNG #2 motor A |
| 27, 32, 33 | output | M4 direction pair and PWM | TB6612FNG #2 motor B |
| 34 | input only | STM32 teleop RX | UART receive only |
| 35 | input only | STM32 sensor bridge RX | UART receive only |
| 36 / VP | ADC input only | AS5600 analog OUT | saturated diagnostic path; I2C controls motion |

GPIO2 and GPIO12 are unassigned but are boot straps. GPIO34-39 are input-only,
and every remaining non-strapping output-capable pin is occupied by motor
control. There is no safe two-pin I2C relocation that preserves the current
wiring and feature set.

## M4 AS5600 policy

- The supported allocation is address `0x36`, SDA `GPIO0`, SCL `GPIO15`, polled every 20 ms.
- GPIO0 LOW at reset selects the serial bootloader. The sensor and wiring must
  not hold it LOW; keep the normal I2C pull-up and do not add a large capacitor.
- GPIO15 LOW at reset can suppress ROM boot logs. Keep its I2C pull-up and ensure
  that no external device holds it LOW during boot.
- The unused TB6612FNG #3 motor-B constant names GPIO15, but firmware never calls
  `pinMode` or `digitalWrite` for that channel. The channel must remain unused.
- The GPIO36 AS5600 analog path is diagnostic-only. The optional base-yaw
  reference also defaults to GPIO36 but is disabled and must not be enabled at
  the same time.
- A future board can remap `MOTIONBRAIN_SHOULDER_I2C_SDA_PIN`,
  `MOTIONBRAIN_SHOULDER_I2C_SCL_PIN`, and `MOTIONBRAIN_SHOULDER_ADC_PIN`; boot
  and regression validation must then be repeated.

The electrical allocation is now the supported prototype allocation, and the
sensor and magnet are mechanically secured. The `-24.35 deg` offset,
`230-245 deg` calibrated range, and directional stop leads remain specific to
this mount. Recalibrate after moving either the sensor or the magnet.

## References

- [Espressif boot mode selection](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html): GPIO0/GPIO15 boot behavior
- [ESP-IDF GPIO API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html): GPIO34-39 input-only restriction
- [ESP32 hardware design guidelines](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/schematic-checklist.html): strap pull-ups and GPIO0 capacitor guidance
