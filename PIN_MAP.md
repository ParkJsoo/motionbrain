# MotionBrain ESP32 핀 맵

[English](PIN_MAP.en.md)

이 문서는 현재 ESP32 5축 제어기 펌웨어의 실제 핀 점유와 M4 AS5600 배치
정책을 기록한다. 소스의 기본값과 다르게 빌드했다면 빌드 플래그가 우선한다.

## 현재 배치

| GPIO | 방향/기능 | 연결 대상 | 비고 |
| ---: | --- | --- | --- |
| 0 | I2C SDA | M4 AS5600 SDA | 부트 스트랩 핀, 리셋 시 HIGH 유지 필요 |
| 1, 3 | UART0 | USB 시리얼·프로그램 업로드 | 재배치 금지 |
| 4, 13, 14 | 출력 | M5 방향 2개, PWM | TB6612FNG #3 모터 A |
| 5 | 출력 | 서치라이트 | 부트 스트랩 핀 |
| 6-11 | 내부 플래시 | ESP32 모듈 플래시 | 사용 금지 |
| 15 | I2C SCL | M4 AS5600 SCL | 부트 스트랩 핀, 미사용 TB6612 B 상수만 존재 |
| 16, 17, 18 | 출력 | M1 방향 2개, PWM | TB6612FNG #1 모터 A |
| 19, 21, 22 | 출력 | M2 방향 2개, PWM | TB6612FNG #1 모터 B |
| 23, 25, 26 | 출력 | M3 방향 2개, PWM | TB6612FNG #2 모터 A |
| 27, 32, 33 | 출력 | M4 방향 2개, PWM | TB6612FNG #2 모터 B |
| 34 | 입력 전용 | STM32 원격조작 RX | UART 수신 전용 |
| 35 | 입력 전용 | STM32 센서 브리지 RX | UART 수신 전용 |
| 36 / VP | 입력 전용 ADC | AS5600 아날로그 OUT | 포화되어 진단 전용, 제어에는 I2C 사용 |

GPIO2와 GPIO12는 현재 미배정이지만 부트 스트랩 핀이다. GPIO34-39는 입력
전용이고, 나머지 비스트랩 출력 가능 핀은 모터 제어에 모두 사용 중이다.
따라서 기존 배선을 유지하면서 M4 I2C를 옮길 수 있는 안전한 2핀 조합은 없다.

## M4 AS5600 정책

- 현재 지원 배치는 주소 `0x36`, SDA `GPIO0`, SCL `GPIO15`, 20ms 주기 폴링이다.
- GPIO0은 리셋 시 LOW이면 시리얼 부트로더로 진입하므로 센서나 배선이
  GPIO0을 LOW로 강제하면 안 된다. 정상 I2C 풀업으로 HIGH를 유지하고,
  GPIO0에 큰 커패시터를 추가하지 않는다.
- GPIO15는 리셋 시 LOW이면 ROM 부팅 로그가 억제될 수 있다. I2C 풀업을
  유지하고, 부트 중 외부 장치가 이 선을 LOW로 잡지 않게 한다.
- `GPIO15`로 정의된 미사용 TB6612FNG #3 모터 B 핀은 코드에서
  `pinMode`/`digitalWrite`하지 않는다. 해당 채널은 앞으로도 사용 금지다.
- AS5600 아날로그 OUT의 GPIO36 경로는 진단 비교용이다. 선택 기능인 베이스 회전 기준 센서도 GPIO36을 기본값으로 갖지만 현재 비활성화 상태이며 동시에 활성화하면
  안 된다.
- 다른 보드로 이전할 때는 `MOTIONBRAIN_SHOULDER_I2C_SDA_PIN`,
  `MOTIONBRAIN_SHOULDER_I2C_SCL_PIN`, `MOTIONBRAIN_SHOULDER_ADC_PIN` 빌드
  매크로로 재배치하고 부트/회귀 검증을 다시 수행한다.

보정값은 적용하는 좌표계를 구분한다.

- ESP32 센서의 장착 오프셋은 [`MOUNT_OFFSET_DEGREES = -24.35°`](src/peripheral/shoulder_angle_sensor.h)다. AS5600 원시각을 로봇팔의 센서 좌표로 보정한다.
- ROS2 M4 제안·실행 경로의 기본 영점은 [`222.80°`, 방향 부호 `+1`](ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/m4_write_contract.py)이다. 센서 좌표와 ROS 관절 좌표를 변환하는 값이며 펌웨어 장착 오프셋과 별개다.
- 읽기 전용 [`m4_state` URDF](ros2_ws/src/motionbrain_hardware_interface/urdf/motionbrain_m4_measured.urdf)는 영점 `0.0°`·보정 비활성으로 시작한다. 현재 장착에서 영점·방향을 확인한 뒤 설정한다.

`230–245°`는 2026-06-28 실측 행렬의 검증 범위이고, `122.08–301.02°`는
당시 팔꿈치·손목·집게 자세에서 확인한 잠정 소프트 범위다. 현재 자세의 안전
범위로 자동 적용할 수 없다. 센서·자석·관절 자세·하중·간섭 조건이 바뀌면
영점과 사용 범위를 다시 검증한다. [실측 조건과 결과](docs/evidence/2026-06-28-m4-shoulder-closed-loop.md)

## 근거

- [Espressif 부팅 모드 선택](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html): GPIO0/GPIO15 부트 동작
- [ESP-IDF GPIO API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html): GPIO34-39 입력 전용 제한
- [ESP32 하드웨어 설계 지침](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/schematic-checklist.html): 스트랩 핀 풀업과 GPIO0 커패시터 조건
