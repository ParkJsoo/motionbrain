# Embedded Firmware Evidence

이 문서는 MotionBrain을 `Robot Embedded System Firmware Engineer` 관점에서 설명할 때 쓸 수 있는 근거를 정리한다. 없는 항목을 확장해 보이기보다, 현재 repo와 실물 bench에서 확인한 MCU 펌웨어/인터페이스/통합 evidence와 남은 gap을 분리한다.

## 공고 요구사항 매핑

| 요구사항 | 현재 evidence | 비고 |
| --- | --- | --- |
| 로봇 내 시스템 펌웨어 개발 | ESP32 motion controller, STM32 handheld teleop/safety firmware, Raspberry Pi ROS2 bridge가 실제 command path로 연결됨 | public media 캡처만 남음 |
| 마이크로컨트롤러 기반 실시간 디지털 시스템 | STM32 `TIM3` 200Hz IMU sampling tick, 25Hz UART teleop frame, HC-SR04 echo EXTI capture, ESP32 1kHz 8-bit PWM motor output | RTOS 없이 bare-metal/HAL loop + interrupt 구조 |
| C/C++ 임베디드 개발 | ESP32 Arduino C++ modules, STM32Cube HAL C firmware | `src/`, `firmware/stm32/` |
| RTOS | 없음 | 보유 역량으로 주장하지 않음 |
| SPI/I2C/USB/CAN/Ethernet | I2C2 MPU-6050, UART teleop/sensor bridge, Wi-Fi HTTP/serial control | CAN/Ethernet/SPI/USB device firmware evidence는 현재 project에 없음 |
| HAL porting | STM32Cube HAL 기반 GPIO/I2C/UART/TIM/EXTI bring-up, I2C2 bus recovery, MPU probe/init/read | HAL 자체 이식이나 RTOS porting은 없음 |
| 모듈 간 인터페이스 상세 정의/구현 | STM32->ESP32 newline JSON teleop protocol, ESP32 serial/HTTP command boundary, ROS2 typed bridge | `docs/TELEOP_BRINGUP.md`, `MESSAGE_INTERFACE.md` |
| System Integration 및 튜닝 | deadman, frame timeout, safety gate, active-low button mapping, teleop output cap, token-gated ROS2 command path | bench 검증 완료, 공개 사진/영상 남음 |
| 기본 연구 장비 활용 | 멀티미터 기반 전원/GND/버튼/출력 sanity check 기준 정리 | 오실로스코프 사용 claim 없음 |

## STM32 펌웨어 근거

- `firmware/stm32/MotionBrainSensor/Core/Src/main.c`
  - `APP_MODE_TELEOP_REMOTE`: handheld remote mode.
  - `MPU_SAMPLE_RATE_HZ=200`, `TELEOP_TX_RATE_HZ=25`, `HCSR04_TRIGGER_INTERVAL_MS=100`.
  - `SendTeleopPacket()`에서 USART2 JSON frame을 만들고 `HAL_UART_Transmit(&huart2, ...)`로 송신.
  - `HAL_GPIO_EXTI_Callback()`에서 HC-SR04 echo pulse width capture.
  - `HAL_TIM_PeriodElapsedCallback()`에서 TIM3 sampling due counter 증가.
  - `RecoverI2c2Bus()`에서 I2C2 SDA stuck 상태 복구 시도.
- `firmware/stm32/MotionBrainSensor/Core/Src/i2c.c`
  - `I2C2` 100kHz, `PB10/D15=SCL`, `PC12/D14=SDA`, open-drain pull-up.
- `firmware/stm32/MotionBrainSensor/Core/Src/usart.c`
  - `USART2` 115200 8N1, `PD5/D1=TX`, `PD6/D0=RX`.
- `firmware/stm32/MotionBrainSensor/Core/Src/tim.c`
  - `TIM3` prescaler `83`, period `4999`; 84MHz timer clock 기준 200Hz tick.
- `firmware/stm32/MotionBrainSensor/Core/Src/mpu6050.c`
  - `HAL_I2C_IsDeviceReady`, `HAL_I2C_Mem_Read`, `HAL_I2C_Mem_Write` 기반 MPU probe/init/read.

## ESP32 펌웨어 근거

- `src/input/teleop_adapter.h/.cpp`
  - `Serial1 RX=GPIO34`, 115200 8N1, RX-only.
  - newline JSON parser, 512-byte line buffer, 1024-byte RX buffer.
  - frame timeout `200ms`, recommended frame period `40ms`.
  - malformed frame drop, `{` resync, overflow drop.
  - `DEADMAN_RELEASE`, `FRAME_TIMEOUT`, `NOT_ARMED`, `SAFETY_BLOCK` stop reason.
  - continuous teleop output cap `35%`, gripper button output `50%`, 5% quantization.
- `src/motor/motor_driver.*`
  - TB6612FNG 3개로 5 motor output.
  - PWM 1kHz, 8-bit, channel 0~4.
  - emergency stop에서 PWM 0과 direction pin LOW 적용.
- `src/main.cpp`
  - `Stm32Bridge`, `TeleopAdapter`, `SafetyMonitor`, `Dispatcher`, `WebServer`, `MotionSequence` 통합 loop.
  - STM32 sensor bridge 또는 teleop embedded safety snapshot 중 active safety source 선택.

## UART Interface Contract

`docs/TELEOP_BRINGUP.md`가 source of truth다. 요약하면 다음과 같다.

- physical: `STM32 PD5 / D1 / USART2_TX -> ESP32 GPIO34 / Serial1 RX`, common GND.
- link: 115200 baud, 8N1, newline-delimited JSON, STM32 TX only.
- rate: STM32 약 25Hz 송신, ESP32 200ms freshness timeout.
- control fields: `type`, `ts_ms`, `seq`, `session`, `deadman`, `reach`, `lift`, `twist`, `grip_open`, `grip_close`, `led_toggle_seq`.
- embedded safety fields: `imu_ok`, `range_ok`, `roll`, `pitch`, `gyro_x`, `gyro_y`, `gyro_z`, `vibe`, `dist_cm`, `imu_status`, `imu_addr`, `imu_error`, `i2c_scl`, `i2c_sda`.

## 멀티미터 기반 Bench Check

오실로스코프가 없으므로 waveform 품질, PWM duty/frequency, UART edge integrity를 검증했다고 쓰지 않는다. 대신 멀티미터로 확인 가능한 항목과 한계를 명시한다.

### 전원 OFF

- STM32 GND, ESP32 GND, TB6612FNG GND, 외부 전원 음극이 공통 GND인지 continuity로 확인.
- ESP32 3V3 rail과 GND 사이 short가 없는지 확인.
- TB6612FNG `VM`과 GND 사이 short가 없는지 확인.
- 버튼 한쪽이 목표 STM32 GPIO로, 다른 쪽이 GND rail로 연결되는지 continuity로 확인.
- 버튼을 누르지 않았을 때 해당 GPIO와 GND가 short 상태로 고정되지 않는지 확인.

### 전원 ON

- ESP32 3V3 rail이 약 3.3V인지 확인.
- TB6612FNG `VCC`가 ESP32 3V3 logic rail에 물려 있는지 확인.
- TB6612FNG `VM`이 외부 전원/XL4015에서 의도한 전압으로 들어오는지 확인.
- STM32 3V3 rail이 정상인지 확인.
- active-low 버튼은 미입력 HIGH, 눌림 LOW로 변하는지 DC voltage로 확인.
- conservative nudge 또는 light toggle 시 관련 출력 rail이 명령 전/후로 변하는지 sanity check.

### 한계

- 멀티미터만으로 UART bit timing, PWM duty/frequency, I2C rise time, motor transient sag, noise margin은 확인할 수 없다.
- 이 항목들은 오실로스코프 또는 logic analyzer가 생겼을 때 별도 evidence로 추가한다.
- 현재 문서/이력서에서는 "멀티미터 기반 전원/접지/버튼/출력 sanity 검증"까지만 claim한다.

## 지원서 문구 후보

아래 정도가 과장 없이 맞다.

> STM32Cube HAL 기반으로 I2C MPU-6050, HC-SR04 EXTI capture, USART2 teleop JSON 송신, TIM3 기반 sampling tick을 구현했고, ESP32 쪽에서는 UART frame freshness/deadman/safety gate를 통합해 실제 모터 출력까지 end-to-end 검증했습니다. 하드웨어 검증은 멀티미터로 전원 rail, 공통 GND, active-low 버튼, 출력 voltage sanity를 확인했으며, 오실로스코프 기반 waveform 검증은 수행하지 않았습니다.

## 남은 Gap

- RTOS task scheduling, priority inversion, queue/timer primitive evidence 없음.
- CAN/Ethernet physical interface evidence 없음.
- 오실로스코프/logic analyzer 캡처 evidence 없음.
- 물리 guarded routine 실행 전 하드웨어 피드백 gap은
  `docs/HARDWARE_FEEDBACK_GAP.md`에 정의한다. 현재 첫 closure target은
  `base_yaw_reference`이며, 이 증거가 생기기 전까지 routine executor는
  disabled 상태를 유지한다.
- `base_yaw_reference`의 첫 실제 배선 후보는 ESP32 `GPIO36` active-low
  index/reference 입력이다. GPIO36은 입력 전용이고 내부 pull-up이 없으므로
  외부 10k pull-up과 Hall/index switch 또는 normally-open reference switch가
  필요하다. 기본 firmware는 이 입력을 disabled 상태로 두며
  `not_installed`를 보고한다.
- 제품형 safety architecture 수준의 독립 safety MCU/channel은 아직 bench MVP 이후 과제다.
