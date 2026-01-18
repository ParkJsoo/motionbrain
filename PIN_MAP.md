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
| **AIN1**     | GPIO 12     | M5 방향 1    | -        | OUTPUT | 부팅 시 주의                 |
| **AIN2**     | GPIO 13     | M5 방향 2    | -        | OUTPUT | -                            |
| **PWMA**     | GPIO 14     | M5 PWM       | 채널 4   | PWM    | -                            |
| **A01**      | M5 모터 +   | 모터 출력    | -        | -      | -                            |
| **A02**      | M5 모터 -   | 모터 출력    | -        | -      | -                            |
| **BIN1**     | GPIO 15     | 미사용       | -        | -      | Strapping pin (부팅 시 HIGH 필요) |
| **BIN2**     | GPIO 0      | 미사용       | -        | -      | **사용 금지** (부팅 모드 핀) |
| **PWMB**     | GPIO 35     | 미사용       | -        | -      | **사용 불가** (INPUT ONLY)   |

### 코드 상수 정의

```cpp
// TB6612FNG #3
static const uint8_t PIN_AIN1_3 = 12;     // 모터 A 방향 1 (M5)
static const uint8_t PIN_AIN2_3 = 13;     // 모터 A 방향 2 (M5)
static const uint8_t PIN_PWMA_3 = 14;     // 모터 A PWM (M5)
static const uint8_t PIN_BIN1_3 = 15;     // 모터 B 방향 1 (미사용)
static const uint8_t PIN_BIN2_3 = 0;      // 모터 B 방향 2 (미사용)
static const uint8_t PIN_PWMB_3 = 35;     // 모터 B PWM (미사용)
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

- GPIO 4, 5, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33: 모두 정상
- GPIO 13, 14: 정상 사용 가능

### ⚠️ 주의 필요 핀

- **GPIO 12**: 부팅 시 플래시 전압에 민감 (현재 사용 중, 문제 없으면 계속 사용)
- **GPIO 15**: Strapping pin (부팅 시 HIGH 필요, 현재 미사용)

### ❌ 사용 금지 핀

- **GPIO 0**: 부팅 모드 핀 (현재 코드에서 BIN2_3에 할당되어 있으나 실제 사용 안 함)
- **GPIO 35**: INPUT ONLY 핀 (현재 코드에서 PWMB_3에 할당되어 있으나 실제 사용 안 함)

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

**참고**: STBY 핀은 사용하지 않습니다. 모터 제어는 PWM과 방향 핀만으로 수행됩니다.

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

- [ ] GPIO 12 → TB6612FNG #3 AIN1
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

## 참고 자료

- **ESP32 핀맵**: [ESP32 DevKitC Pinout](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html)
- **TB6612FNG 데이터시트**: [Pololu TB6612FNG](https://www.pololu.com/product/713)
- **코드 참조**: `src/motor/motor_driver.h`

---

**작성일**: 2024
**버전**: 1.0
