#include "system_init.h"
#include "debug/debug_log.h"

/**
 * SystemStateManager 생성자
 * 
 * 객체가 생성될 때 자동으로 호출됨
 * - BOOT 상태로 시작
 * - 타임아웃 기본값: 5000ms (5초)
 */
SystemStateManager::SystemStateManager() 
  : currentState_(SystemState::BOOT)  // 부팅 상태로 시작
  , lastCommandTime_(0)                // 아직 명령 없음
  , timeoutMs_(5000)                   // 기본 타임아웃: 5초
{
  // 생성자 본문
  // DebugLog는 아직 초기화되지 않았을 수 있으므로
  // 여기서는 로그 출력 안 함 (나중에 setup()에서 로그 출력)
}

/**
 * 현재 시스템 상태를 반환
 * 
 * @return 현재 상태 (SystemState enum)
 */
SystemState SystemStateManager::getState() const {
  return currentState_;
}

/**
 * 현재 상태를 문자열로 반환 (로그 출력용)
 * 
 * @return 상태 문자열 ("BOOT", "IDLE", "ARMED", "FAULT")
 */
const char* SystemStateManager::getStateString() const {
  switch (currentState_) {
    case SystemState::BOOT:
      return "BOOT";
    
    case SystemState::IDLE:
      return "IDLE";
    
    case SystemState::ARMED:
      return "ARMED";
    
    case SystemState::FAULT:
      return "FAULT";
    
    default:
      return "UNKNOWN";  // 예상치 못한 상태
  }
}

// ===== 상태 전환 메서드 =====

/**
 * 상태 전환 (일반적인 방법)
 * 
 * @param newState 전환할 상태
 * @return 전환 성공 여부 (false면 잘못된 전환)
 */
bool SystemStateManager::transitionTo(SystemState newState) {
  // 1. 이미 같은 상태면 전환 불필요
  if (currentState_ == newState) {
    DebugLog::debug("State transition skipped: already in %s", getStateString());
    return true;  // 성공 (이미 해당 상태)
  }
  
  // 2. 상태 전환 가능 여부 검증
  if (!isValidTransition(currentState_, newState)) {
    // 잘못된 전환 시도
    DebugLog::warn(
      "Invalid state transition: %s -> %s",
      getStateString(),
      stateToString(newState));
    return false;  // 실패
  }
  
  // 3. 상태 전환 실행
  SystemState oldState = currentState_;  // 로그용으로 이전 상태 저장
  currentState_ = newState;
  
  // 4. 타임아웃 타이머 리셋 (상태 전환 시 명령 수신으로 간주)
  resetTimeout();
  
  // 5. 상태 변화 로그 출력
  DebugLog::stateChange(stateToString(oldState), getStateString(), "State transition");
  
  return true;  // 전환 성공
}

/**
 * 시스템 무장 (IDLE → ARMED)
 * 모터 제어가 가능한 상태로 전환
 * 
 * @return 전환 성공 여부
 */
bool SystemStateManager::arm() {
  // IDLE 상태에서만 ARMED로 전환 가능
  if (currentState_ != SystemState::IDLE) {
    DebugLog::warn("arm() failed: current state is %s (must be IDLE)", getStateString());
    return false;  // IDLE 상태가 아니면 실패
  }
  
  // transitionTo()가 isValidTransition()으로 검증하므로
  // 여기서는 단순히 호출만 하면 됨
  bool result = transitionTo(SystemState::ARMED);
  if (result) {
    DebugLog::command("arm", true, "System armed - motor control enabled");
  }
  return result;
}

/**
 * 시스템 해제 (ARMED → IDLE)
 * 모터 제어를 불가능한 상태로 전환
 * 
 * @return 전환 성공 여부
 */
bool SystemStateManager::disarm() {
  // ARMED 상태에서만 IDLE로 전환 가능
  if (currentState_ != SystemState::ARMED) {
    DebugLog::warn("disarm() failed: current state is %s (must be ARMED)", getStateString());
    return false;  // ARMED 상태가 아니면 실패
  }
  
  // transitionTo()가 isValidTransition()으로 검증하므로
  // 여기서는 단순히 호출만 하면 됨
  bool result = transitionTo(SystemState::IDLE);
  if (result) {
    DebugLog::command("disarm", true, "System disarmed - motor control disabled");
  }
  return result;
}

/**
 * 비상 정지 (모든 상태 → IDLE)
 * 어떤 상태에서든 안전 상태로 즉시 전환
 * 
 * @return 전환 성공 여부
 */
bool SystemStateManager::enterSafe() {
  // 이미 IDLE 상태면 전환 불필요
  if (currentState_ == SystemState::IDLE) {
    return true;  // 이미 안전 상태
  }
  
  // 어떤 상태에서든 IDLE로 전환 시도
  // isValidTransition()에서 다음 전환이 허용됨:
  // - BOOT → IDLE ✓
  // - ARMED → IDLE ✓
  // - FAULT → IDLE ✓
  // 따라서 transitionTo()를 호출하면 됨
  bool result = transitionTo(SystemState::IDLE);
  if (result) {
    DebugLog::safety("ENTER_SAFE", "Emergency safe mode activated");
  }
  return result;
}

// ===== 타이머 관리 메서드 =====

/**
 * 상태 머신 업데이트 (주기적으로 호출)
 * 타임아웃 체크 및 자동 SAFE 복귀 처리
 * loop()에서 주기적으로 호출해야 함
 */
void SystemStateManager::update() {
  // SAFE(IDLE)나 FAULT 상태에서는 타임아웃 체크 불필요
  if (currentState_ == SystemState::IDLE || currentState_ == SystemState::FAULT) {
    return;  // 타임아웃 체크 안 함
  }
  
  // 현재 시간 확인
  uint32_t currentTime = millis();
  
  // 마지막 명령 시간으로부터 경과 시간 계산
  // millis()는 약 49일 후 오버플로우되므로, unsigned 뺄셈 사용
  uint32_t elapsed = currentTime - lastCommandTime_;
  
  // 타임아웃 체크
  if (elapsed >= timeoutMs_) {
    // 타임아웃 발생! 자동으로 안전 상태로 복귀
    DebugLog::safety(
      "AUTO_SAFE_TIMEOUT",
      "No command received - auto returning to IDLE"
    );
    enterSafe();  // IDLE 상태로 전환
  }
}

/**
 * 타임아웃 타이머 리셋
 * 명령을 받았을 때 호출하여 타이머를 초기화
 */
void SystemStateManager::resetTimeout() {
  lastCommandTime_ = millis();  // 현재 시간으로 리셋
}

/**
 * 타임아웃 시간 설정
 * 
 * @param timeoutMs 타임아웃 시간 (밀리초)
 */
void SystemStateManager::setTimeout(uint32_t timeoutMs) {
  timeoutMs_ = timeoutMs;  // 타임아웃 시간 설정
}

// ===== Private 메서드 =====

/**
 * 상태를 문자열로 변환 (private - 내부 헬퍼 함수)
 * 
 * @param state 변환할 상태
 * @return 상태 문자열
 */
const char* SystemStateManager::stateToString(SystemState state) {
  switch (state) {
    case SystemState::BOOT:  return "BOOT";
    case SystemState::IDLE:  return "IDLE";
    case SystemState::ARMED: return "ARMED";
    case SystemState::FAULT: return "FAULT";
    default:                 return "UNKNOWN";
  }
}

/**
 * 상태 전환 검증 (private - 내부에서만 사용)
 * from 상태에서 to 상태로 전환 가능한지 확인
 * 
 * @param from 현재 상태
 * @param to 전환할 상태
 * @return 전환 가능 여부
 */
bool SystemStateManager::isValidTransition(SystemState from, SystemState to) const {
  // 로드맵의 상태 전환 규칙에 따라 검증
  
  // 1. FAULT 상태는 모든 상태에서 가능 (오류 발생 시)
  if (to == SystemState::FAULT) {
    return true;  // 어떤 상태에서든 FAULT로 전환 가능
  }
  
  // 2. FAULT에서 나가는 것은 IDLE로만 가능
  if (from == SystemState::FAULT) {
    return (to == SystemState::IDLE);
  }
  
  // 3. 각 상태별 허용된 전환 확인
  switch (from) {
    case SystemState::BOOT:
      // BOOT → IDLE만 허용
      return (to == SystemState::IDLE);
    
    case SystemState::IDLE:
      // IDLE → ARMED만 허용 (arm 명령)
      return (to == SystemState::ARMED);
    
    case SystemState::ARMED:
      // ARMED → IDLE만 허용 (disarm/stop/timeout)
      return (to == SystemState::IDLE);
    
    case SystemState::FAULT:
      // 이미 위에서 처리됨 (여기 도달 안 함)
      return false;
    
    default:
      // 예상치 못한 상태
      return false;
  }
}

