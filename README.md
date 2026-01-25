# MotionBrain

ESP32 기반 지능형 모션 제어 시스템 (로봇팔용)

## 📋 프로젝트 개요

MotionBrain은 ESP32를 사용한 안전하고 확장 가능한 모션 제어 플랫폼입니다. 로봇팔 제어를 위한 5개 모터를 독립적으로 제어할 수 있으며, 웹 UI와 시리얼 명령을 통해 제어할 수 있습니다.

### 핵심 목표

> **입력 → 판단 → 상태 → 움직임 → 피드백**

ESP32는 실시간성, 안전성, 물리 제어를 책임지는 **모션 제어 브레인** 역할을 수행합니다.

## ✨ 주요 기능

### 현재 구현된 기능 (Phase 1.5 완료)

- ✅ **시스템 상태 머신**: BOOT → IDLE → ARMED → FAULT 상태 관리
- ✅ **안전 규칙 강제**: ARMED 상태에서만 모터 제어 허용
- ✅ **웹 대시보드**: 실시간 상태 모니터링 및 제어
- ✅ **조이스틱 모드**: 5개 모터 독립 제어 (모바일 터치 지원)
- ✅ **버튼 모드**: 각 모터별 Forward/Reverse/Stop 버튼
- ✅ **키보드 단축키**: PC에서 키보드로 모터 제어
- ✅ **시리얼 명령**: USB 시리얼을 통한 명령 제어
- ✅ **Wi-Fi AP**: 무선 네트워크를 통한 웹 접근
- ✅ **모바일 최적화**: 반응형 웹 UI, 터치 이벤트 지원
- ✅ **여러 조이스틱 동시 조작**: 각 조이스틱이 독립적으로 동작

### 모터 구성

- **M1**: 그리퍼 (Gripper) - 물체 잡기/놓기
- **M2**: 손목 관절 (Wrist Tilt) - 손목 상하 움직임
- **M3**: 팔꿈치 관절 (Elbow Joint) - 팔꿈치 상하 움직임
- **M4**: 어깨 관절 (Shoulder Joint) - 어깨 상하 움직임
- **M5**: 베이스 회전 (Base Rotation) - 로봇팔 전체 좌우 회전

## 🔧 하드웨어 구성

### 필수 하드웨어

- **ESP32 개발 보드** (ESP32-DevKitC 등)
- **TB6612FNG 모터 드라이버** × 3개
  - TB6612FNG #1: M1 (모터 A), M2 (모터 B)
  - TB6612FNG #2: M3 (모터 A), M4 (모터 B)
  - TB6612FNG #3: M5 (모터 A), 미사용 (모터 B)
- **DC 모터** × 5개

### 핀 연결

상세한 핀 연결 정보는 다음 문서를 참조하세요:

- **핀 맵**: `PIN_MAP.md` - 전체 핀 연결표 및 하드웨어 연결 가이드
- **코드 정의**: `src/motor/motor_driver.h` - 소스 코드 상수 정의

## 🚀 설치 및 사용법

### 개발 환경 설정

1. **PlatformIO 설치**

   ```bash
   # PlatformIO CLI 설치 (선택사항)
   pip install platformio
   ```

2. **프로젝트 클론**

   ```bash
   git clone <repository-url>
   cd motionbrain
   ```

3. **의존성 설치 및 빌드**

   ```bash
   pio run
   ```

4. **ESP32에 업로드**

   ```bash
   pio run -t upload
   ```

5. **시리얼 모니터**
   ```bash
   pio device monitor
   ```

### 사용 방법

#### 1. Wi-Fi 연결

1. ESP32 전원을 켭니다
2. Wi-Fi 네트워크 목록에서 `MotionBrain-AP`를 찾습니다
3. 비밀번호 없이 연결합니다 (또는 설정된 비밀번호 입력)
4. 브라우저에서 `http://192.168.4.1` 접속

#### 2. 웹 대시보드 사용

**시스템 제어:**

- **ARM**: 시스템을 ARMED 상태로 전환 (모터 제어 가능)
- **DISARM**: 시스템을 IDLE 상태로 전환 (모터 제어 불가)
- **STOP**: 긴급 정지 (모든 모터 즉시 정지)

**모터 제어:**

- **버튼 모드**: 각 모터별 Forward/Reverse 버튼으로 제어
- **조이스틱 모드**: 조이스틱을 드래그하여 연속 제어
- **속도 조절**: 버튼 모드에서 슬라이더로 속도 조절 (0-100%)

**키보드 단축키 (버튼 모드):**

- `Q/A`: M1 Forward/Reverse
- `W/S`: M2 Forward/Reverse
- `E/D`: M3 Forward/Reverse
- `R/F`: M4 Forward/Reverse
- `T/G`: M5 Forward/Reverse

#### 3. 시리얼 명령 사용

USB 시리얼 연결 후 다음 명령 사용:

```
help              # 도움말 표시
status            # 시스템 상태 확인
arm               # 시스템 ARMED 상태로 전환
disarm            # 시스템 IDLE 상태로 전환
stop              # 긴급 정지
motor forward 1   # M1 정방향 (기본 속도)
motor forward 1 50 # M1 정방향 (50% 속도)
motor reverse 2   # M2 역방향
motor stop 3      # M3 정지
motor status      # 모든 모터 상태 확인
motor default 150 # 기본 속도 설정 (1-255)
```

## 📁 프로젝트 구조

```
motionbrain/
├── platformio.ini          # PlatformIO 설정
├── README.md                # 이 파일
├── 로드맵.md                 # 프로젝트 로드맵
├── PHASE1-5_STEPS.md        # Phase 1-5 단계별 가이드
├── PIN_MAP.md               # 핀 맵 및 하드웨어 연결 가이드
└── src/
    ├── main.cpp             # 진입점
    ├── system/              # 시스템 상태 관리
    │   ├── system_init.h
    │   └── system_init.cpp
    ├── motor/               # 모터 제어
    │   ├── motor_driver.h
    │   └── motor_driver.cpp
    ├── input/               # 입력 처리
    │   ├── serial_command.h
    │   └── serial_command.cpp
    ├── network/             # 네트워크
    │   ├── wifi_ap.h
    │   ├── wifi_ap.cpp
    │   ├── web_server.h
    │   └── web_server.cpp
    └── debug/               # 디버그 로그
        ├── debug_log.h
        └── debug_log.cpp
```

## 🔒 안전 기능

### 안전 규칙

1. **부팅 직후 항상 SAFE**: 시스템은 IDLE 상태로 시작
2. **명시적 ARM 필요**: ARMED 상태에서만 모터 제어 가능
3. **타임아웃 자동 안전 전환**: 30초 동안 명령이 없으면 자동으로 IDLE 상태로 전환
4. **긴급 정지**: 언제든지 STOP 명령으로 모든 모터 즉시 정지
5. **물리 차단**: PWM=0 및 방향 핀=LOW로 설정하여 하드웨어 레벨에서 차단

### 상태 머신

- **BOOT**: 시스템 초기화 중
- **IDLE**: 안전 상태 (모터 제어 불가)
- **ARMED**: 구동 가능 상태 (모터 제어 가능)
- **FAULT**: 오류 상태 (모든 제어 차단)

## 📊 현재 상태

### 완료된 Phase

- ✅ **Phase 0**: 환경 정리 및 리스크 제거
- ✅ **Phase 1-1**: 프로젝트 구조 고정
- ✅ **Phase 1-2**: 시스템 상태 머신 구현
- ✅ **Phase 1-3**: 안전 규칙 코드화
- ✅ **Phase 1-4**: 시리얼 명령 기반 제어
- ✅ **Phase 1.5-1**: Wi-Fi AP 입력 채널
- ✅ **Phase 1.5-2**: 웹 UI (대시보드 + 명령)
- ✅ **Phase 1.5-3**: 키보드 / 모바일 입력

### 현재 진행 중

- 🔄 **Phase 1-5**: TB6612FNG 연동 (물리 제어 시작)
  - ✅ Step 1-1: 드라이버 전원 연결 및 측정 완료
  - 🔄 Step 1-2: 펌웨어 업로드 및 GPIO 핀 검증 (진행 중)
  - 📋 Step 2: PWM 핀 전압 측정 (선택사항)
  - 📋 Step 3: PWM 신호 출력 테스트
  - 📋 Step 4: 모터 1개 연결 및 낮은 PWM 테스트
  - 📋 Step 5: 듀얼 모터 확장 테스트
  - 📋 Step 6: 모든 모터 연결 및 통합 테스트

### 다음 단계

- 📋 **Phase 2**: 모션 추상화 계층
- 📋 **Phase 3**: 입력 · 판단 계층 분리 확장
- 📋 **Phase 4**: AI 연계 (장기 목표)

**상세 가이드**:

- **Phase 1-5 단계별 가이드**: `PHASE1-5_STEPS.md` - TB6612FNG 연동 상세 가이드
- **핀 맵**: `PIN_MAP.md` - 전체 핀 연결표 및 하드웨어 연결 가이드
- **로드맵**: `로드맵.md` - 전체 프로젝트 로드맵

## 🛠️ 기술 스택

- **하드웨어**: ESP32 (Arduino Framework)
- **개발 환경**: PlatformIO
- **프로그래밍 언어**: C++
- **네트워크**: Wi-Fi AP 모드
- **웹 기술**: HTML5, CSS3, JavaScript (Vanilla)
- **모터 드라이버**: TB6612FNG

## 📝 로그 시스템

시스템은 상세한 로그를 시리얼 포트로 출력합니다:

- **INFO**: 일반 정보
- **DEBUG**: 디버깅 정보
- **WARN**: 경고
- **ERROR**: 오류
- **STATE**: 상태 전환
- **MOTOR**: 모터 제어 명령
- **SAFETY**: 안전 관련 이벤트
- **COMMAND**: 명령 실행

## 🔧 설정

### Wi-Fi AP 설정

`src/main.cpp` 파일에서 Wi-Fi AP 설정을 변경할 수 있습니다:

```cpp
const char* apSSID = "MotionBrain-AP";
const char* apPassword = nullptr;  // 공개 AP
// const char* apPassword = "your-password";  // 비밀번호 설정
```

### 안전 타임아웃 설정

기본 타임아웃은 30초입니다. `src/main.cpp`에서 변경 가능:

```cpp
systemState.setTimeout(30000);  // 30초 (밀리초 단위)
```

## 🐛 문제 해결

### 웹 대시보드에 접속할 수 없음

1. Wi-Fi 네트워크 목록에서 `MotionBrain-AP` 확인
2. ESP32가 정상적으로 부팅되었는지 시리얼 모니터 확인
3. 브라우저에서 `http://192.168.4.1` 직접 입력

### 모터가 동작하지 않음

1. 시스템이 ARMED 상태인지 확인 (웹 대시보드 또는 시리얼 명령)
2. 시리얼 모니터에서 오류 메시지 확인
3. 모터 드라이버 연결 확인

### 조이스틱이 동작하지 않음

1. 조이스틱 모드로 전환했는지 확인
2. 시스템이 ARMED 상태인지 확인
3. 브라우저 콘솔에서 JavaScript 오류 확인

## 📄 라이선스

이 프로젝트는 개인 연구/학습/제작 프로젝트입니다.

## 👤 작성자

개인 연구 프로젝트

## 📚 참고 자료

- [ESP32 Arduino Framework](https://docs.espressif.com/projects/arduino-esp32/)
- [PlatformIO Documentation](https://docs.platformio.org/)
- [TB6612FNG Datasheet](https://www.pololu.com/product/713)

---

## 📚 문서

- **README.md**: 프로젝트 개요 및 사용법
- **로드맵.md**: 전체 프로젝트 로드맵 및 Phase 정의
- **PHASE1-5_STEPS.md**: Phase 1-5 (TB6612FNG 연동) 단계별 상세 가이드
- **PIN_MAP.md**: ESP32 ↔ TB6612FNG 핀 연결표 및 하드웨어 연결 가이드

---

**현재 버전**: Phase 1-5 진행 중 (Step 1-2)
