# MotionBrain 핀 맵 (Pin Map)

ESP32와 TB6612FNG 모터 드라이버 간의 핀 연결 정보

---

## 전체 핀 맵 개요

### 모터 구성

- **M1**: 그리퍼 (Gripper) - TB6612FNG #1 모터 A
- **M2**: 손목 관절 (Wrist Tilt) - TB6612FNG #1 모터 B
- **M3**: 팔꿈치 관절 (Elbow Joint) - TB6612FNG #2 모터 A
- **M4**: 어깨 관절 (Shoulder Joint) - TB6612FNG #2 모터 B
- **M5**: 베이스 회전 (Base Rotation) - TB6612FNG #3 모터 A

---

## TB6612FNG #1 (M1, M2)

### 핀 연결표

| TB6612FNG 핀 | ESP32 GPIO  | 기능         | PWM 채널 | 방향   |
| ------------ | ----------- | ------------ | -------- | ------ |
| **VCC**      | ESP32 5V    | 로직 전원    | -        | -      |
| **GND**      | ESP32 GND   | 공통 GND     | -        | -      |
| **VM**       | 외부 전원 + | 모터 전원    | -        | -      |
| **AIN1**     | GPIO 16     | M1 방향 1    | -        | OUTPUT |
| **AIN2**     | GPIO 17     | M1 방향 2    | -        | OUTPUT |
| **PWMA**     | GPIO 18     | M1 PWM       | 채널 0   | PWM    |
| **A01**      | M1 모터 +   | 모터 출력    | -        | -      |
| **A02**      | M1 모터 -   | 모터 출력    | -        | -      |
| **BIN1**     | GPIO 19     | M2 방향 1    | -        | OUTPUT |
| **BIN2**     | GPIO 21     | M2 방향 2    | -        | OUTPUT |
| **PWMB**     | GPIO 22     | M2 PWM       | 채널 1   | PWM    |
| **B01**      | M2 모터 +   | 모터 출력    | -        | -      |
| **B02**      | M2 모터 -   | 모터 출력    | -        | -      |

### 코드 상수 정의

```cpp
// TB6612FNG #1
static const uint8_t PIN_AIN1_1 = 16;     // 모터 A 방향 1
static const uint8_t PIN_AIN2_1 = 17;     // 모터 A 방향 2
static const uint8_t PIN_PWMA_1 = 18;     // 모터 A PWM
static const uint8_t PIN_BIN1_1 = 19;     // 모터 B 방향 1
static const uint8_t PIN_BIN2_1 = 21;     // 모터 B 방향 2
static const uint8_t PIN_PWMB_1 = 22;     // 모터 B PWM
```

---

## TB6612FNG #2 (M3, M4)

### 핀 연결표

| TB6612FNG 핀 | ESP32 GPIO  | 기능         | PWM 채널 | 방향   |
| ------------ | ----------- | ------------ | -------- | ------ |
| **VCC**      | ESP32 5V    | 로직 전원    | -        | -      |
| **GND**      | ESP32 GND   | 공통 GND     | -        | -      |
| **VM**       | 외부 전원 + | 모터 전원    | -        | -      |
| **AIN1**     | GPIO 23     | M3 방향 1    | -        | OUTPUT |
| **AIN2**     | GPIO 25     | M3 방향 2    | -        | OUTPUT |
| **PWMA**     | GPIO 26     | M3 PWM       | 채널 2   | PWM    |
| **A01**      | M3 모터 +   | 모터 출력    | -        | -      |
| **A02**      | M3 모터 -   | 모터 출력    | -        | -      |
| **BIN1**     | GPIO 27     | M4 방향 1    | -        | OUTPUT |
| **BIN2**     | GPIO 32     | M4 방향 2    | -        | OUTPUT |
| **PWMB**     | GPIO 33     | M4 PWM       | 채널 3   | PWM    |
| **B01**      | M4 모터 +   | 모터 출력    | -        | -      |
| **B02**      | M4 모터 -   | 모터 출력    | -        | -      |

### 코드 상수 정의

```cpp
// TB6612FNG #2
static const uint8_t PIN_AIN1_2 = 23;     // 모터 A 방향 1
static const uint8_t PIN_AIN2_2 = 25;     // 모터 A 방향 2
static const uint8_t PIN_PWMA_2 = 26;     // 모터 A PWM
static const uint8_t PIN_BIN1_2 = 27;     // 모터 B 방향 1
static const uint8_t PIN_BIN2_2 = 32;     // 모터 B 방향 2
static const uint8_t PIN_PWMB_2 = 33;     // 모터 B PWM
```

---

## TB6612FNG #3 (M5)

### 핀 연결표

| TB6612FNG 핀 | ESP32 GPIO  | 기능         | PWM 채널 | 방향   | 비고                         |
| ------------ | ----------- | ------------ | -------- | ------ | ---------------------------- |
| **VCC**      | ESP32 5V    | 로직 전원    | -        | -      | -                            |
| **GND**      | ESP32 GND   | 공통 GND     | -        | -      | -                            |
| **VM**       | 외부 전원 + | 모터 전원    | -        | -      | -                            |
| **AIN1**     | GPIO 4      | M5 방향 1    | -        | OUTPUT | GPIO12에서 변경 (strapping pin 회피) |
| **AIN2**     | GPIO 13     | M5 방향 2    | -        | OUTPUT | -                            |
| **PWMA**     | GPIO 14     | M5 PWM       | 채널 4   | PWM    | -                            |
| **A01**      | M5 모터 +   | 모터 출력    | -        | -      | -                            |
| **A02**      | M5 모터 -   | 모터 출력    | -        | -      | -                            |
| **BIN1**     | GPIO 15     | 미사용       | -        | -      | Strapping pin (부팅 시 HIGH 필요) |
| **BIN2**     | N/C (미연결) | 미사용      | -        | -      | **절대 연결 금지** — GPIO0는 부팅 모드 핀, PIN_UNUSED(0xFF) 처리 |
| **PWMB**     | N/C (미연결) | 미사용      | -        | -      | **절대 연결 금지** — GPIO35는 INPUT ONLY, PIN_UNUSED(0xFF) 처리 |

### 코드 상수 정의

```cpp
// TB6612FNG #3
static const uint8_t PIN_AIN1_3 = 4;      // 모터 A 방향 1 (M5) — GPIO12에서 변경
static const uint8_t PIN_AIN2_3 = 13;     // 모터 A 방향 2 (M5)
static const uint8_t PIN_PWMA_3 = 14;     // 모터 A PWM (M5)
static const uint8_t PIN_BIN1_3 = 15;          // 모터 B 방향 1 (미사용)
static const uint8_t PIN_BIN2_3 = PIN_UNUSED;  // GPIO0 부트핀 회피
static const uint8_t PIN_PWMB_3 = PIN_UNUSED;  // GPIO35 입력전용 회피
```

---

## 전원 연결

### 공통 연결 (모든 드라이버 공통)

```
ESP32 5V ──┬── TB6612FNG #1 VCC
           ├── TB6612FNG #2 VCC
           └── TB6612FNG #3 VCC

ESP32 GND ──┬── TB6612FNG #1 GND
            ├── TB6612FNG #2 GND
            └── TB6612FNG #3 GND
```

### 모터 전원 (외부 전원 공급 장치)

```
외부 전원 + ──┬── TB6612FNG #1 VM
              ├── TB6612FNG #2 VM
              └── TB6612FNG #3 VM

외부 전원 - ──┴── 공통 GND
```

**주의사항**:

- 모터 전원은 반드시 외부 전원 공급 장치 사용
- ESP32 5V 핀은 로직 전원용 (모터 전원 불가)
- 모든 GND는 반드시 공통 연결

---

## PWM 채널 할당

| 모터        | PWM 채널 | ESP32 GPIO | PWM 주파수 | PWM 해상도    |
| ----------- | -------- | ---------- | ---------- | ------------- |
| M1 (그리퍼) | 채널 0   | GPIO 18    | 1kHz       | 8-bit (0-255) |
| M2 (손목)   | 채널 1   | GPIO 22    | 1kHz       | 8-bit (0-255) |
| M3 (팔꿈치) | 채널 2   | GPIO 26    | 1kHz       | 8-bit (0-255) |
| M4 (어깨)   | 채널 3   | GPIO 33    | 1kHz       | 8-bit (0-255) |
| M5 (베이스) | 채널 4   | GPIO 14    | 1kHz       | 8-bit (0-255) |

---

## ESP32 GPIO 핀 상태

### ✅ 정상 사용 가능 핀

- GPIO 4, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33: 모두 정상 사용 중
- GPIO 13, 14: 정상 사용 가능

### ⚠️ 주의 필요 핀

- **GPIO 4**: M5 방향 1 핀 (GPIO12 strapping pin 회피로 변경됨)
- **GPIO 15**: 미연결/미사용 (코드 상수 PIN_BIN1_3만 정의, 실제 동작에 영향 없음)

### ❌ 사용 금지 핀

- **GPIO 0**: 부팅 모드 핀 — 코드 상수로만 정의, `pinMode`/`digitalWrite` 호출 제거됨 (실제 미사용)
- **GPIO 35**: INPUT ONLY 핀 — 코드 상수로만 정의, 실제 미사용

---

## Wired Handheld Teleop UART

유선 handheld remote v1은 모터 제어 핀과 별도로 teleop frame 수신용 UART 한 줄을 사용한다.

### ESP32 수신 핀

| 용도 | ESP32 GPIO | 시리얼 포트 | 방향 | 비고 |
| ---- | ---------- | ----------- | ---- | ---- |
| teleop RX | GPIO 34 | `Serial1 RX` | INPUT | 입력 전용 핀, TX 없이 RX-only 사용 |

### STM32 송신 핀

| 용도 | STM32 핀 | 보드 표기 | 주변장치 | 방향 |
| ---- | -------- | --------- | -------- | ---- |
| teleop TX | PD5 | Arduino D1 | `USART2_TX` | OUTPUT |

### 배선

```text
STM32 PD5 / D1 / USART2_TX  ->  ESP32 GPIO34 / Serial1 RX
STM32 GND                   ->  ESP32 GND
```

주의:

- teleop v1은 단방향 UART만 사용한다.
- `GPIO34`는 입력 전용이므로 teleop RX에 적합하다.
- 기존 STM32 sensor bridge가 쓰던 `Serial2 RX=GPIO35`와 별도 채널이다.
- 센서 허브와 teleop remote를 동시에 별도 STM32 보드로 운용하는 구성을 우선 가정한다.

---

## STM32 Handheld Teleop Provisional Buttons

현재 `MotionBrainSensor`의 `APP_MODE_TELEOP_REMOTE`는 아래 임시 버튼 핀 매핑을 사용한다.

| 기능 | STM32 핀 | 비고 |
| ---- | -------- | ---- |
| `deadman` | `PA0` | hold-to-enable |
| `LED toggle` | `PA1` | rising edge counter |
| `grip open` | `PA4` | active-low pull-up 기준 |
| `grip close` | `PB0` | active-low pull-up 기준 |

주의:

- 현재 코드는 내부 pull-up + active-low 버튼을 가정한다.
- 실제 handheld 하우징과 버튼 배치가 확정되면 STM32 `main.c` 상단 매크로를 기준으로 핀만 교체하면 된다.

---

## 모터 방향 제어 로직

### 정방향 (Forward)

```
AIN1 = HIGH, AIN2 = LOW  (또는 BIN1 = HIGH, BIN2 = LOW)
PWMA (또는 PWMB) = 속도 값 (0-255)
```

### 역방향 (Reverse)

```
AIN1 = LOW, AIN2 = HIGH  (또는 BIN1 = LOW, BIN2 = HIGH)
PWMA (또는 PWMB) = 속도 값 (0-255)
```

### 정지 (Stop)

```
AIN1 = LOW, AIN2 = LOW  (또는 BIN1 = LOW, BIN2 = LOW)
PWMA (또는 PWMB) = 0
```

**참고**: STBY 핀은 소프트웨어에서 제어하지 않습니다. 모터 제어는 PWM과 방향 핀만으로 수행됩니다.
TB6612FNG가 동작하려면 STBY 핀이 HIGH여야 합니다. 브레이크아웃 보드를 사용하는 경우 보드 내장 풀업 저항으로 자동 처리됩니다. 직접 IC를 배선하는 경우 STBY 핀을 VCC(3.3V 또는 5V)에 직접 연결하세요.

---

## 하드웨어 연결 체크리스트

### 전원 연결

- [ ] ESP32 5V → TB6612FNG #1, #2, #3 VCC
- [ ] ESP32 GND → TB6612FNG #1, #2, #3 GND (공통)
- [ ] 외부 전원 + → TB6612FNG #1, #2, #3 VM
- [ ] 외부 전원 - → 공통 GND

### 제어 핀 연결

- [ ] GPIO 16 → TB6612FNG #1 AIN1
- [ ] GPIO 17 → TB6612FNG #1 AIN2
- [ ] GPIO 18 → TB6612FNG #1 PWMA
- [ ] GPIO 19 → TB6612FNG #1 BIN1
- [ ] GPIO 21 → TB6612FNG #1 BIN2
- [ ] GPIO 22 → TB6612FNG #1 PWMB

- [ ] GPIO 23 → TB6612FNG #2 AIN1
- [ ] GPIO 25 → TB6612FNG #2 AIN2
- [ ] GPIO 26 → TB6612FNG #2 PWMA
- [ ] GPIO 27 → TB6612FNG #2 BIN1
- [ ] GPIO 32 → TB6612FNG #2 BIN2
- [ ] GPIO 33 → TB6612FNG #2 PWMB

- [ ] GPIO 4 → TB6612FNG #3 AIN1 (GPIO12에서 변경)
- [ ] GPIO 13 → TB6612FNG #3 AIN2
- [ ] GPIO 14 → TB6612FNG #3 PWMA

### 모터 연결

- [ ] M1 모터 + → TB6612FNG #1 A01
- [ ] M1 모터 - → TB6612FNG #1 A02
- [ ] M2 모터 + → TB6612FNG #1 B01
- [ ] M2 모터 - → TB6612FNG #1 B02
- [ ] M3 모터 + → TB6612FNG #2 A01
- [ ] M3 모터 - → TB6612FNG #2 A02
- [ ] M4 모터 + → TB6612FNG #2 B01
- [ ] M4 모터 - → TB6612FNG #2 B02
- [ ] M5 모터 + → TB6612FNG #3 A01
- [ ] M5 모터 - → TB6612FNG #3 A02

---

## 기존 로봇팔 하드웨어 참고

배터리·모터 연결부 커버 안에 있는 **기존 로봇팔 전원/분배 보드** 구조를 정리한 내용입니다. MotionBrain(ESP32 + TB6612FNG) 연동 시 참고용입니다.

### 기존 보드 구성 (PCB)

| 표기 | 의미 | 비고 |
|------|------|------|
| **M1+** | 모터 1 연결 (2핀) | TB6612FNG #1 A01/A02에 대응 |
| **M2+** | 모터 2 연결 (2핀) | TB6612FNG #1 B01/B02에 대응 |
| **M3+** | 모터 3 연결 (2핀) | TB6612FNG #2 A01/A02에 대응 |
| **M4+** | 모터 4 연결 (2핀) | TB6612FNG #2 B01/B02에 대응 |
| **M5** | 모터 5 연결 | TB6612FNG #3 A01/A02에 대응 |
| **GND** | 공통 GND | ESP32·TB6612FNG·외부전원 GND와 공통 연결 필요 |
| **LED+** | LED 표시등 | 선택 사용 |
| **8핀 커넥터** | BAT, BK, R 등 | 기존 컨트롤러/배터리 입력용 (우리 구동계에서는 미사용) |

- M1+~M4+는 각각 **2핀 헤더 한 쌍**(모터 +/-)으로 보면 됨.
- 기존 보드는 **배터리(BAT) → 이 보드 → M1~M5 모터**로 전원/신호를 나누는 역할.

### 우리 프로젝트와의 대응

- **전원 경로**: 기존에는 배터리 → 기존 보드 → 모터. MotionBrain에서는 **D형 건전지 4개 → XL4015 → TB6612FNG VM** 으로 모터 전원을 공급하고, **기존 보드는 전원/제어 경로에서 제외**하는 구성을 권장.
- **모터 연결**: TB6612FNG 출력을 **로봇팔의 모터 선에 직접** 연결하거나, 기존 M1~M5 커넥터 위치에 새로 선을 댈 때 아래 매핑 사용.

| 로봇팔 보드 표기 | MotionBrain (TB6612FNG) |
|-----------------|--------------------------|
| M1+ (2핀)       | TB6612FNG #1 **A01**, **A02** |
| M2+ (2핀)       | TB6612FNG #1 **B01**, **B02** |
| M3+ (2핀)       | TB6612FNG #2 **A01**, **A02** |
| M4+ (2핀)       | TB6612FNG #2 **B01**, **B02** |
| M5              | TB6612FNG #3 **A01**, **A02** |

- A01/A02(또는 B01/B02) 중 한쪽이 +, 한쪽이 - 역할을 하므로, 모터가 반대로 돌면 두 선을 서로 바꿔 연결하면 됨.
- **GND**: 기존 보드의 GND는 ESP32, XL4015 출력 -, TB6612FNG GND, 배터리 - 와 **반드시 한 곳에서 공통**으로 연결.

### 연동 시 유의사항

1. **기존 보드 사용 여부**: MotionBrain로 완전 대체 시 기존 보드에 배터리/컨트롤러 입력을 넣지 않고, TB6612FNG 출력만 로봇팔 모터(M1~M5)에 연결하는 방식이 안전함.
2. **케이블 재사용**: 로봇팔에서 나온 M1~M5 케이블을 그대로 쓸 경우, 반대쪽 끝을 TB6612FNG의 A01/A02, B01/B02에 맞춰 연결하고, 극성은 테스트 후 반대면 스왑.
3. **전원 분리**: 모터 전원(건전지 → XL4015 → VM)과 로직 전원(ESP32 5V)은 구분하고, GND만 공통으로 연결.

---

## 참고 자료

- **ESP32 핀맵**: [ESP32 DevKitC Pinout](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html)
- **TB6612FNG 데이터시트**: [Pololu TB6612FNG](https://www.pololu.com/product/713)
- **코드 참조**: `src/motor/motor_driver.h`

---

**작성일**: 2024
**버전**: 1.0
