# MotionBrain ESP32 핀 맵

[English](PIN_MAP.en.md)

이 문서는 현재 ESP32 5축 제어기 펌웨어의 실제 핀 점유와 M4 AS5600 배치
정책을 기록한다. 소스의 기본값과 다르게 빌드했다면 빌드 플래그가 우선한다.

## 현재 배치

| GPIO | 방향/기능 | 연결 대상 | 비고 |
| ---: | --- | --- | --- |
| 0 | I2C SDA | M4 AS5600 SDA | 부트 스트랩 핀, reset 시 HIGH 유지 필요 |
| 1, 3 | UART0 | USB serial/programming | 재배치 금지 |
| 4, 13, 14 | 출력 | M5 방향 2개, PWM | TB6612FNG #3 motor A |
| 5 | 출력 | 서치라이트 | 부트 스트랩 핀 |
| 6-11 | 내부 flash | ESP32 module flash | 사용 금지 |
| 15 | I2C SCL | M4 AS5600 SCL | 부트 스트랩 핀, 미사용 TB6612 B 상수만 존재 |
| 16, 17, 18 | 출력 | M1 방향 2개, PWM | TB6612FNG #1 motor A |
| 19, 21, 22 | 출력 | M2 방향 2개, PWM | TB6612FNG #1 motor B |
| 23, 25, 26 | 출력 | M3 방향 2개, PWM | TB6612FNG #2 motor A |
| 27, 32, 33 | 출력 | M4 방향 2개, PWM | TB6612FNG #2 motor B |
| 34 | 입력 전용 | STM32 teleop RX | UART 수신 전용 |
| 35 | 입력 전용 | STM32 sensor bridge RX | UART 수신 전용 |
| 36 / VP | 입력 전용 ADC | AS5600 analog OUT | 포화되어 진단 전용, 제어에는 I2C 사용 |

GPIO2와 GPIO12는 현재 미배정이지만 부트 스트랩 핀이다. GPIO34-39는 입력
전용이고, 나머지 비스트랩 출력 가능 핀은 모터 제어에 모두 사용 중이다.
따라서 기존 배선을 유지하면서 M4 I2C를 옮길 수 있는 안전한 2핀 조합은 없다.

## M4 AS5600 정책

- 현재 지원 배치는 주소 `0x36`, SDA `GPIO0`, SCL `GPIO15`, 20ms polling이다.
- GPIO0은 reset 시 LOW이면 serial bootloader로 진입하므로 센서나 배선이
  GPIO0을 LOW로 강제하면 안 된다. 정상 I2C pull-up으로 HIGH를 유지하고,
  GPIO0에 큰 커패시터를 추가하지 않는다.
- GPIO15는 reset 시 LOW이면 ROM boot log가 억제될 수 있다. I2C pull-up을
  유지하고, 부트 중 외부 장치가 이 선을 LOW로 잡지 않게 한다.
- `GPIO15`로 정의된 미사용 TB6612FNG #3 motor B 핀은 코드에서
  `pinMode`/`digitalWrite`하지 않는다. 해당 채널은 앞으로도 사용 금지다.
- AS5600 analog OUT의 GPIO36 경로는 진단 비교용이다. optional base-yaw
  reference도 GPIO36을 기본값으로 갖지만 현재 disabled이며 동시에 활성화하면
  안 된다.
- 다른 보드로 이전할 때는 `MOTIONBRAIN_SHOULDER_I2C_SDA_PIN`,
  `MOTIONBRAIN_SHOULDER_I2C_SCL_PIN`, `MOTIONBRAIN_SHOULDER_ADC_PIN` 빌드
  매크로로 재배치하고 부트/회귀 검증을 다시 수행한다.

전기 핀 배치는 현재 프로토타입의 지원 배치로 확정했고, 센서와 자석의 기구
고정도 완료했다. 다만 `-24.35°` 오프셋, `230-245°` 보정 범위와 방향별 정지
선행값은 현재 장착 조건에 종속된다. 센서 또는 자석을 다시 장착하면 오프셋과
안전 범위를 재보정한다.

## 근거

- [Espressif boot mode selection](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html): GPIO0/GPIO15 부트 동작
- [ESP-IDF GPIO API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html): GPIO34-39 입력 전용 제한
- [ESP32 hardware design guidelines](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/schematic-checklist.html): 스트랩 핀 pull-up과 GPIO0 커패시터 조건
