#include "motor_driver.h"
#include "debug/debug_log.h"
#include "system/system_init.h"  // SystemState enum 사용

// 전역 객체 참조 (main.cpp에 선언됨)
extern SystemStateManager systemState;

// STBY 핀 배열 정의
const uint8_t MotorControl::STBY_PINS[NUM_DRIVERS] = {
  PIN_STBY_1,  // 드라이버 #1
  PIN_STBY_2,  // 드라이버 #2
  PIN_STBY_3   // 드라이버 #3
};

/**
 * MotorControl 생성자
 */
MotorControl::MotorControl() {
  // 모든 모터 상태 초기화
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    enabled_[i] = false;
    currentSpeed_[i] = 0;
    targetSpeed_[i] = 0;
    lastUpdateTime_[i] = 0;
  }
  
  // 드라이버별 오류 상태 초기화
  for (uint8_t i = 0; i < NUM_DRIVERS; i++) {
    driverError_[i] = false;
  }
  
  // 기본 속도 초기화 (기본값: 100)
  defaultSpeed_ = 100;
}

/**
 * 모터 제어 모듈 초기화
 */
bool MotorControl::init() {
  DebugLog::info("=== Motor Control Initialization (Phase 1-5: TB6612FNG) ===");
  DebugLog::info("Number of motors: %d", NUM_MOTORS);
  DebugLog::info("Number of drivers: %d", NUM_DRIVERS);
  
  // 1. 핀 모드 설정
  // TB6612FNG #1
  pinMode(PIN_STBY_1, OUTPUT);
  pinMode(PIN_AIN1_1, OUTPUT);
  pinMode(PIN_AIN2_1, OUTPUT);
  pinMode(PIN_BIN1_1, OUTPUT);
  pinMode(PIN_BIN2_1, OUTPUT);
  
  // TB6612FNG #2
  pinMode(PIN_STBY_2, OUTPUT);
  pinMode(PIN_AIN1_2, OUTPUT);
  pinMode(PIN_AIN2_2, OUTPUT);
  pinMode(PIN_BIN1_2, OUTPUT);
  pinMode(PIN_BIN2_2, OUTPUT);
  
  // TB6612FNG #3
  pinMode(PIN_STBY_3, OUTPUT);
  pinMode(PIN_AIN1_3, OUTPUT);
  pinMode(PIN_AIN2_3, OUTPUT);
  pinMode(PIN_BIN1_3, OUTPUT);
  pinMode(PIN_BIN2_3, OUTPUT);
  
  DebugLog::debug("GPIO pins configured");
  
  // 2. PWM 채널 설정 (5개 모터만 사용)
  // ESP32의 ledcSetup은 항상 성공하므로 별도 체크 불필요
  // 하지만 로그는 남기고 진행
  ledcSetup(PWM_CHANNEL_M1, PWM_FREQUENCY, PWM_RESOLUTION);  // M1: 그리퍼
  ledcSetup(PWM_CHANNEL_M2, PWM_FREQUENCY, PWM_RESOLUTION);  // M2: 손목
  ledcSetup(PWM_CHANNEL_M3, PWM_FREQUENCY, PWM_RESOLUTION);  // M3: 팔꿈치
  ledcSetup(PWM_CHANNEL_M4, PWM_FREQUENCY, PWM_RESOLUTION);  // M4: 어깨
  ledcSetup(PWM_CHANNEL_M5, PWM_FREQUENCY, PWM_RESOLUTION);  // M5: 베이스
  
  // PWM 핀 연결
  // ESP32의 ledcAttachPin은 항상 성공하므로 별도 체크 불필요
  ledcAttachPin(PIN_PWMA_1, PWM_CHANNEL_M1);  // M1: 그리퍼
  ledcAttachPin(PIN_PWMB_1, PWM_CHANNEL_M2);  // M2: 손목
  ledcAttachPin(PIN_PWMA_2, PWM_CHANNEL_M3);  // M3: 팔꿈치
  ledcAttachPin(PIN_PWMB_2, PWM_CHANNEL_M4);  // M4: 어깨
  ledcAttachPin(PIN_PWMA_3, PWM_CHANNEL_M5);  // M5: 베이스
  
  DebugLog::debug("PWM channels configured (freq: %d Hz, resolution: %d-bit)", 
                  PWM_FREQUENCY, PWM_RESOLUTION);
  
  // 3. 안전: 모든 STBY 핀을 LOW로 설정 (차단)
  setSTBYAll(false);
  
  // 4. 모든 PWM 출력을 0으로 설정 (5개 모터만)
  ledcWrite(PWM_CHANNEL_M1, 0);  // M1: 그리퍼
  ledcWrite(PWM_CHANNEL_M2, 0);  // M2: 손목
  ledcWrite(PWM_CHANNEL_M3, 0);  // M3: 팔꿈치
  ledcWrite(PWM_CHANNEL_M4, 0);  // M4: 어깨
  ledcWrite(PWM_CHANNEL_M5, 0);  // M5: 베이스
  
  // 5. 모든 방향 핀을 LOW로 설정
  digitalWrite(PIN_AIN1_1, LOW);
  digitalWrite(PIN_AIN2_1, LOW);
  digitalWrite(PIN_BIN1_1, LOW);
  digitalWrite(PIN_BIN2_1, LOW);
  digitalWrite(PIN_AIN1_2, LOW);
  digitalWrite(PIN_AIN2_2, LOW);
  digitalWrite(PIN_BIN1_2, LOW);
  digitalWrite(PIN_BIN2_2, LOW);
  digitalWrite(PIN_AIN1_3, LOW);
  digitalWrite(PIN_AIN2_3, LOW);
  digitalWrite(PIN_BIN1_3, LOW);
  digitalWrite(PIN_BIN2_3, LOW);
  
  DebugLog::motor("INIT", "TB6612FNG initialized - STBY=LOW (safe), all motors stopped");
  DebugLog::info("Motor control ready (Phase 1-5: Step 1 - Power connection only)");
  
  return true;
}

/**
 * 기본 속도 설정
 */
bool MotorControl::setDefaultSpeed(uint8_t speed) {
  // 속도 범위 검증 (1-255만 유효, 0은 모터가 움직이지 않으므로 기본 속도로는 무의미)
  if (speed == 0) {
    DebugLog::error("Default speed cannot be 0 - motors will not move. Valid range: 1-255");
    return false;
  }
  if (speed > 255) {
    DebugLog::error("Default speed out of range: %d. Valid range: 1-255", speed);
    return false;
  }
  
  defaultSpeed_ = speed;
  DebugLog::info("Default speed set to: %d", speed);
  
  return true;
}

/**
 * 기본 속도 가져오기
 */
uint8_t MotorControl::getDefaultSpeed() const {
  return defaultSpeed_;
}

/**
 * 모터 정방향 구동 (기본 속도 사용)
 */
bool MotorControl::forward(uint8_t motorId) {
  return forward(motorId, 100);  // 100% = 기본 속도
}

/**
 * 모터 정방향 구동 (속도 비율 지정)
 */
bool MotorControl::forward(uint8_t motorId, uint8_t percent) {
  // 모터 ID 유효성 검사
  if (!isValidMotorId(motorId)) {
    DebugLog::error("Invalid motor ID: %d (valid range: 1-%d for M1~M5)", motorId, NUM_MOTORS);
    return false;
  }
  
  // 안전 검사: ARMED 상태에서만 허용
  if (!checkSafety("forward")) {
    return false;
  }
  
  // 속도 비율 범위 제한
  if (percent > 100) percent = 100;
  
  // 속도 비율을 실제 속도로 변환
  uint8_t speed = percentToSpeed(percent);
  
  // 목표 속도 설정
  uint8_t index = motorIdToIndex(motorId);
  targetSpeed_[index] = speed;
  enabled_[index] = true;
  
  // 타임아웃 타이머 리셋 (모터 명령 수신으로 간주)
  systemState.resetTimeout();
  
  // 로그는 serial_command에서 출력하므로 여기서는 제거
  // DebugLog::debug("Motor M%d: forward at %d%% speed (%d)", motorId, percent, speed);
  
  return true;
}

/**
 * 모터 역방향 구동 (기본 속도 사용)
 */
bool MotorControl::reverse(uint8_t motorId) {
  return reverse(motorId, 100);  // 100% = 기본 속도
}

/**
 * 모터 역방향 구동 (속도 비율 지정)
 */
bool MotorControl::reverse(uint8_t motorId, uint8_t percent) {
  // 모터 ID 유효성 검사
  if (!isValidMotorId(motorId)) {
    DebugLog::error("Invalid motor ID: %d (valid range: 1-%d for M1~M5)", motorId, NUM_MOTORS);
    return false;
  }
  
  // 안전 검사: ARMED 상태에서만 허용
  if (!checkSafety("reverse")) {
    return false;
  }
  
  // 속도 비율 범위 제한
  if (percent > 100) percent = 100;
  
  // 속도 비율을 실제 속도로 변환 (음수로)
  uint8_t speed = percentToSpeed(percent);
  int16_t reverseSpeed = -(int16_t)speed;
  
  // 목표 속도 설정
  uint8_t index = motorIdToIndex(motorId);
  targetSpeed_[index] = reverseSpeed;
  enabled_[index] = true;
  
  // 타임아웃 타이머 리셋 (모터 명령 수신으로 간주)
  systemState.resetTimeout();
  
  // 로그는 serial_command에서 출력하므로 여기서는 제거
  // DebugLog::debug("Motor M%d: reverse at %d%% speed (%d)", motorId, percent, reverseSpeed);
  
  return true;
}

/**
 * 개별 모터 속도 설정
 */
bool MotorControl::setSpeed(uint8_t motorId, int16_t speed) {
  // 모터 ID 유효성 검사
  if (!isValidMotorId(motorId)) {
    DebugLog::error("Invalid motor ID: %d (valid range: 1-%d for M1~M5)", motorId, NUM_MOTORS);
    return false;
  }
  
  // 안전 검사: ARMED 상태에서만 허용
  if (!checkSafety("setSpeed")) {
    return false;
  }
  
  // 속도 범위 제한
  if (speed < -255) speed = -255;
  if (speed > 255) speed = 255;
  
  // 목표 속도 설정 (점진적 변경을 위해)
  uint8_t index = motorIdToIndex(motorId);
  targetSpeed_[index] = speed;
  
  // 목표 속도가 0이 아니면 활성화 상태로 표시
  if (speed != 0) {
    enabled_[index] = true;
  }
  
  // 타임아웃 타이머 리셋 (모터 명령 수신으로 간주)
  systemState.resetTimeout();
  
  DebugLog::debug("Motor M%d: target speed set to %d (will ramp gradually)", motorId, speed);
  
  return true;
}

/**
 * 모든 모터 속도 설정 (동일한 속도)
 */
bool MotorControl::setSpeedAll(int16_t speed) {
  // 안전 검사: ARMED 상태에서만 허용
  if (!checkSafety("setSpeedAll")) {
    return false;
  }
  
  // 속도 범위 제한
  if (speed < -255) speed = -255;
  if (speed > 255) speed = 255;
  
  // 모든 모터의 목표 속도 설정
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    targetSpeed_[i] = speed;
    if (speed != 0) {
      enabled_[i] = true;
    }
  }
  
  // 타임아웃 타이머 리셋 (모터 명령 수신으로 간주)
  systemState.resetTimeout();
  
  DebugLog::debug("All motors: target speed set to %d (will ramp gradually)", speed);
  
  return true;
}

/**
 * 개별 모터 정지 (점진적 정지)
 */
bool MotorControl::stop(uint8_t motorId) {
  // 모터 ID 유효성 검사
  if (!isValidMotorId(motorId)) {
    DebugLog::error("Invalid motor ID: %d (valid range: 1-%d for M1~M5)", motorId, NUM_MOTORS);
    return false;
  }
  
  // 안전 검사: ARMED 상태에서만 허용
  if (!checkSafety("stop")) {
    return false;
  }
  
  // 목표 속도를 0으로 설정 (점진적 정지)
  uint8_t index = motorIdToIndex(motorId);
  targetSpeed_[index] = 0;
  
  // 타임아웃 타이머 리셋 (모터 명령 수신으로 간주)
  systemState.resetTimeout();
  
  DebugLog::debug("Motor M%d: target speed set to 0 (will ramp down gradually)", motorId);
  
  return true;
}

/**
 * 모든 모터 정지 (점진적 정지)
 */
bool MotorControl::stopAll() {
  // 안전 검사: ARMED 상태에서만 허용
  if (!checkSafety("stopAll")) {
    return false;
  }
  
  // 모든 모터의 목표 속도를 0으로 설정
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    targetSpeed_[i] = 0;
  }
  
  // 타임아웃 타이머 리셋 (모터 명령 수신으로 간주)
  systemState.resetTimeout();
  
  DebugLog::debug("All motors: target speed set to 0 (will ramp down gradually)");
  
  return true;
}

/**
 * 비상 정지 (상태 무시하고 즉시 정지)
 */
void MotorControl::emergencyStop() {
  // 모든 모터 즉시 정지
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    uint8_t motorId = indexToMotorId(i);
    uint8_t index = motorIdToIndex(motorId);
    
    // 목표 속도와 현재 속도를 모두 0으로 설정 (즉시 정지)
    targetSpeed_[index] = 0;
    currentSpeed_[index] = 0;
    enabled_[index] = false;
    lastUpdateTime_[index] = 0;
    
    // PWM 출력 0
    uint8_t pwmChannel = index;  // 배열 인덱스 = PWM 채널 번호
    ledcWrite(pwmChannel, 0);
    
    // 방향 핀 LOW
    uint8_t driverId = getDriverId(motorId);
    uint8_t motorIndex = getMotorIndexInDriver(motorId);
    
    if (driverId == 0) {
      if (motorIndex == 0) {
        digitalWrite(PIN_AIN1_1, LOW);
        digitalWrite(PIN_AIN2_1, LOW);
      } else {
        digitalWrite(PIN_BIN1_1, LOW);
        digitalWrite(PIN_BIN2_1, LOW);
      }
    } else if (driverId == 1) {
      if (motorIndex == 0) {
        digitalWrite(PIN_AIN1_2, LOW);
        digitalWrite(PIN_AIN2_2, LOW);
      } else {
        digitalWrite(PIN_BIN1_2, LOW);
        digitalWrite(PIN_BIN2_2, LOW);
      }
    } else {
      if (motorIndex == 0) {
        digitalWrite(PIN_AIN1_3, LOW);
        digitalWrite(PIN_AIN2_3, LOW);
      } else {
        digitalWrite(PIN_BIN1_3, LOW);
        digitalWrite(PIN_BIN2_3, LOW);
      }
    }
  }
  
  // 모든 STBY 핀을 LOW로 설정 (물리 차단)
  setSTBYAll(false);
  
  DebugLog::safety("EMERGENCY_STOP", "All motors emergency stopped - STBY=LOW (physical block)");
  DebugLog::motor("emergencyStop", "FORCED STOP - all %d motors", NUM_MOTORS);
}

/**
 * 모터 활성화 여부 확인
 */
bool MotorControl::isEnabled(uint8_t motorId) const {
  if (!isValidMotorId(motorId)) {
    return false;
  }
  
  // ARMED 상태가 아니면 항상 false
  SystemState currentState = getCurrentState();
  if (currentState != SystemState::ARMED) {
    return false;
  }
  
  uint8_t index = motorIdToIndex(motorId);
  return enabled_[index] && currentSpeed_[index] != 0;
}

/**
 * 모든 모터 활성화 여부 확인 (하나라도 활성화되어 있으면 true)
 */
bool MotorControl::isEnabled() const {
  // ARMED 상태가 아니면 항상 false
  SystemState currentState = getCurrentState();
  if (currentState != SystemState::ARMED) {
    return false;
  }
  
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (enabled_[i] && currentSpeed_[i] != 0) {
      return true;
    }
  }
  return false;
}

/**
 * 현재 모터 속도 가져오기
 */
int16_t MotorControl::getSpeed(uint8_t motorId) const {
  if (!isValidMotorId(motorId)) {
    return 0;
  }
  uint8_t index = motorIdToIndex(motorId);
  return currentSpeed_[index];
}

/**
 * 모터 제어 업데이트 (주기적으로 호출)
 * 점진적 속도 변경 처리
 */
void MotorControl::update() {
  // 시스템 상태 확인: ARMED 상태가 아니면 모든 모터 정지
  SystemState currentState = getCurrentState();
  if (currentState != SystemState::ARMED) {
    // ARMED 상태가 아니면 모든 모터의 목표 속도를 0으로 설정
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (targetSpeed_[i] != 0) {
        targetSpeed_[i] = 0;
      }
    }
  }
  
  uint32_t currentTime = millis();
  
  // 모든 모터에 대해 점진적 속도 변경 처리
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    // 현재 속도와 목표 속도가 같으면 스킵
    if (currentSpeed_[i] == targetSpeed_[i]) {
      continue;
    }
    
    // 마지막 업데이트 시간 확인 (간격 체크)
    uint32_t elapsed = currentTime - lastUpdateTime_[i];
    if (elapsed < RAMP_INTERVAL_MS) {
      continue;  // 아직 업데이트 시간이 안 됨
    }
    
    // ARMED 상태가 아니면 즉시 정지
    if (currentState != SystemState::ARMED) {
      currentSpeed_[i] = 0;
      enabled_[i] = false;
      targetSpeed_[i] = 0;
      
      // PWM 출력 0
      uint8_t pwmChannel = i;
      ledcWrite(pwmChannel, 0);
      
      // 드라이버별 STBY 핀 제어
      uint8_t motorId = indexToMotorId(i);
      uint8_t driverId = getDriverId(motorId);
      setSTBY(driverId, false);
      
      continue;
    }
    
    // 점진적 속도 변경
    int16_t diff = targetSpeed_[i] - currentSpeed_[i];
    int16_t step = (diff > 0) ? RAMP_STEP_SIZE : -RAMP_STEP_SIZE;
    
    // 차이가 단계 크기보다 작으면 목표 속도로 바로 설정
    if (abs(diff) <= RAMP_STEP_SIZE) {
      currentSpeed_[i] = targetSpeed_[i];
    } else {
      currentSpeed_[i] += step;
    }
    
    // 목표 속도가 0이고 현재 속도도 0에 가까우면 비활성화
    if (targetSpeed_[i] == 0 && abs(currentSpeed_[i]) < RAMP_STEP_SIZE) {
      currentSpeed_[i] = 0;
      enabled_[i] = false;
    }
    
    // 실제 PWM 출력 업데이트
    uint8_t motorId = indexToMotorId(i);
    setMotorSpeedInternal(motorId, currentSpeed_[i]);
    
    // 마지막 업데이트 시간 갱신
    lastUpdateTime_[i] = currentTime;
  }
}

/**
 * 안전 검사 (private)
 * 
 * 시스템 상태가 ARMED일 때만 모터 제어 허용
 * 그 외 상태(BOOT, IDLE, FAULT)에서는 모든 명령 거부
 * 
 * @param action 동작 이름 (로그 출력용)
 * @return 안전 여부 (ARMED 상태면 true, 그 외 false)
 */
bool MotorControl::checkSafety(const char* action) {
  // 현재 시스템 상태 확인
  SystemState currentState = systemState.getState();
  
  // ARMED 상태에서만 모터 제어 허용
  if (currentState == SystemState::ARMED) {
    return true;  // 안전 - 제어 허용
  }
  
  // 그 외 상태에서는 제어 거부
  const char* stateString = systemState.getStateString();
  DebugLog::safety("MOTOR_BLOCKED", "Motor control blocked - system state is not ARMED");
  DebugLog::warn("Motor action '%s' blocked - current state: %s", action, stateString);
  DebugLog::motor(action, "BLOCKED - unsafe state: %s", stateString);
  
  return false;  // 불안전 - 제어 거부
}

/**
 * 모터 ID 유효성 검사 (1-based: 1 ~ 5)
 */
bool MotorControl::isValidMotorId(uint8_t motorId) const {
  return motorId >= 1 && motorId <= NUM_MOTORS;
}

/**
 * 모터 번호를 배열 인덱스로 변환 (1-based → 0-based)
 */
uint8_t MotorControl::motorIdToIndex(uint8_t motorId) const {
  // M1(1) -> 0, M2(2) -> 1, ..., M5(5) -> 4
  return motorId - 1;
}

/**
 * 배열 인덱스를 모터 번호로 변환 (0-based → 1-based)
 */
uint8_t MotorControl::indexToMotorId(uint8_t index) const {
  // 0 -> M1(1), 1 -> M2(2), ..., 4 -> M5(5)
  return index + 1;
}

/**
 * 개별 모터 제어 (내부 헬퍼)
 */
bool MotorControl::setMotorSpeedInternal(uint8_t motorId, int16_t speed) {
  if (!isValidMotorId(motorId)) {
    return false;
  }
  
  // 속도 범위 제한
  if (speed < -255) speed = -255;
  if (speed > 255) speed = 255;
  
  // 모터 번호를 배열 인덱스로 변환
  uint8_t index = motorIdToIndex(motorId);
  
  // 드라이버 번호 및 드라이버 내 모터 인덱스 가져오기
  uint8_t driverId = getDriverId(motorId);
  uint8_t motorIndex = getMotorIndexInDriver(motorId);
  uint8_t pwmChannel = index;  // 배열 인덱스 = PWM 채널 번호
  
  // 드라이버 오류 상태 확인
  if (driverError_[driverId]) {
    DebugLog::warn("Motor M%d: driver #%d has error - motor control disabled", motorId, driverId);
    return false;
  }
  
  // 방향 결정
  bool forward = (speed > 0);
  bool reverse = (speed < 0);
  uint8_t pwmValue = (speed < 0) ? -speed : speed;  // 절댓값
  
  // 드라이버별 핀 설정
  if (driverId == 0) {
    // TB6612FNG #1
    if (motorIndex == 0) {
      // 모터 A (M1: 그리퍼)
      digitalWrite(PIN_AIN1_1, forward ? HIGH : LOW);
      digitalWrite(PIN_AIN2_1, reverse ? HIGH : LOW);
      ledcWrite(PWM_CHANNEL_M1, pwmValue);
    } else {
      // 모터 B (M2: 손목)
      digitalWrite(PIN_BIN1_1, forward ? HIGH : LOW);
      digitalWrite(PIN_BIN2_1, reverse ? HIGH : LOW);
      ledcWrite(PWM_CHANNEL_M2, pwmValue);
    }
  } else if (driverId == 1) {
    // TB6612FNG #2
    if (motorIndex == 0) {
      // 모터 A (M3: 팔꿈치)
      digitalWrite(PIN_AIN1_2, forward ? HIGH : LOW);
      digitalWrite(PIN_AIN2_2, reverse ? HIGH : LOW);
      ledcWrite(PWM_CHANNEL_M3, pwmValue);
    } else {
      // 모터 B (M4: 어깨)
      digitalWrite(PIN_BIN1_2, forward ? HIGH : LOW);
      digitalWrite(PIN_BIN2_2, reverse ? HIGH : LOW);
      ledcWrite(PWM_CHANNEL_M4, pwmValue);
    }
  } else {
    // TB6612FNG #3
    if (motorIndex == 0) {
      // 모터 A (M5: 베이스)
      digitalWrite(PIN_AIN1_3, forward ? HIGH : LOW);
      digitalWrite(PIN_AIN2_3, reverse ? HIGH : LOW);
      ledcWrite(PWM_CHANNEL_M5, pwmValue);
    } else {
      // 모터 B는 미사용
      // 이 코드는 실행되지 않아야 함 (motorId 1~5만 유효)
      DebugLog::error("Invalid motor configuration: driver #%d motor B (M6 not used)", driverId);
      return false;
    }
  }
  
  // STBY 핀 제어: ARMED 상태에서만 HIGH
  SystemState currentState = getCurrentState();
  if (currentState == SystemState::ARMED) {
    setSTBY(driverId, true);
  } else {
    setSTBY(driverId, false);
  }
  
  // 상태 업데이트
  currentSpeed_[index] = speed;
  enabled_[index] = (speed != 0);
  
  // 로그 출력
  if (speed == 0) {
    DebugLog::motor("setSpeed", "M%d: stopped", motorId);
  } else if (speed > 0) {
    DebugLog::motor("setSpeed", "M%d: forward - speed: %d", motorId, speed);
  } else {
    DebugLog::motor("setSpeed", "M%d: reverse - speed: %d", motorId, -speed);
  }
  
  return true;
}

/**
 * STBY 핀 제어
 */
void MotorControl::setSTBY(uint8_t driverId, bool enable) {
  if (driverId >= NUM_DRIVERS) {
    return;
  }
  
  digitalWrite(STBY_PINS[driverId], enable ? HIGH : LOW);
  
  if (enable) {
    DebugLog::debug("STBY driver #%d: HIGH (enabled)", driverId);
  } else {
    DebugLog::debug("STBY driver #%d: LOW (disabled)", driverId);
  }
}

/**
 * 모든 STBY 핀 제어
 */
void MotorControl::setSTBYAll(bool enable) {
  for (uint8_t i = 0; i < NUM_DRIVERS; i++) {
    setSTBY(i, enable);
  }
}

/**
 * 모터 번호로 드라이버 번호 가져오기
 */
uint8_t MotorControl::getDriverId(uint8_t motorId) const {
  // M1(1), M2(2) -> 드라이버 0
  // M3(3), M4(4) -> 드라이버 1
  // M5(5) -> 드라이버 2
  uint8_t index = motorIdToIndex(motorId);
  return index / 2;
}

/**
 * 모터 번호로 드라이버 내 모터 인덱스 가져오기 (A=0, B=1)
 */
uint8_t MotorControl::getMotorIndexInDriver(uint8_t motorId) const {
  // M1(1), M3(3), M5(5) = 모터 A (0)
  // M2(2), M4(4) = 모터 B (1)
  uint8_t index = motorIdToIndex(motorId);
  return index % 2;
}

/**
 * 현재 시스템 상태 가져오기
 */
SystemState MotorControl::getCurrentState() const {
  // 전역 객체 systemState 참조
  return systemState.getState();
}

/**
 * 속도 비율을 실제 속도로 변환
 */
uint8_t MotorControl::percentToSpeed(uint8_t percent) const {
  // 속도 비율 범위 제한
  if (percent > 100) percent = 100;
  
  // 기본 속도의 퍼센트 계산
  // 예: 기본 속도 100, 비율 50% -> 50
  // 예: 기본 속도 200, 비율 50% -> 100
  uint16_t speed = (uint16_t)defaultSpeed_ * percent / 100;
  
  // 최대값 제한
  if (speed > 255) speed = 255;
  
  return (uint8_t)speed;
}
