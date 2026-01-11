#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include <Arduino.h>

/**
 * MotionBrain System State
 * 
 * 시스템의 안전 상태를 나타내는 enum
 * - BOOT: 부팅 중
 * - IDLE: 기본값, SAFE 상태 (구동 불가)
 * - ARMED: 구동 가능 (모터 제어 허용)
 * - FAULT: 강제 차단 (오류 상태)
 */
enum class SystemState {
  BOOT,   // 부팅 중
  IDLE,   // 기본값, SAFE 상태 (구동 불가)
  ARMED,  // 구동 가능 (모터 제어 허용)
  FAULT   // 강제 차단 (오류 상태)
};

/**
 * SystemStateManager
 * 
 * 시스템 상태를 관리하는 클래스
 * - 현재 상태 저장
 * - 상태 전환 제어
 * - 안전 타이머 관리
 */
class SystemStateManager {
private:
  // 현재 시스템 상태
  SystemState currentState_;
  
  // 마지막 명령을 받은 시간 (타임아웃 체크용)
  // uint32_t: 0 ~ 4,294,967,295 (약 49일)
  uint32_t lastCommandTime_;
  
  // 자동 SAFE 복귀 타임아웃 시간 (밀리초)
  // 기본값: 5000ms (5초)
  uint32_t timeoutMs_;
  
  // 상태 전환 검증 (private - 내부에서만 사용)
  // from 상태에서 to 상태로 전환 가능한지 확인
  bool isValidTransition(SystemState from, SystemState to) const;
  
  // 상태를 문자열로 변환 (private - 내부 헬퍼 함수)
  // SystemState enum을 문자열로 변환
  static const char* stateToString(SystemState state);

public:
  // 생성자 (객체 생성 시 호출)
  SystemStateManager();
  
  // ===== 상태 조회 메서드 =====
  
  /**
   * 현재 시스템 상태를 반환
   * @return 현재 상태 (SystemState enum)
   */
  SystemState getState() const;
  
  /**
   * 현재 상태를 문자열로 반환 (로그 출력용)
   * @return 상태 문자열 ("BOOT", "IDLE", "ARMED", "FAULT")
   */
  const char* getStateString() const;
  
  // ===== 상태 전환 메서드 =====
  
  /**
   * 상태 전환 (일반적인 방법)
   * @param newState 전환할 상태
   * @return 전환 성공 여부 (false면 잘못된 전환)
   */
  bool transitionTo(SystemState newState);
  
  /**
   * 시스템 무장 (IDLE → ARMED)
   * 모터 제어가 가능한 상태로 전환
   * @return 전환 성공 여부
   */
  bool arm();
  
  /**
   * 시스템 해제 (ARMED → IDLE)
   * 모터 제어를 불가능한 상태로 전환
   * @return 전환 성공 여부
   */
  bool disarm();
  
  /**
   * 비상 정지 (모든 상태 → IDLE)
   * 어떤 상태에서든 안전 상태로 즉시 전환
   * @return 전환 성공 여부
   */
  bool enterSafe();
  
  // ===== 타이머 관리 메서드 =====
  
  /**
   * 상태 머신 업데이트 (주기적으로 호출)
   * 타임아웃 체크 및 자동 SAFE 복귀 처리
   * loop()에서 주기적으로 호출해야 함
   */
  void update();
  
  /**
   * 타임아웃 타이머 리셋
   * 명령을 받았을 때 호출하여 타이머를 초기화
   */
  void resetTimeout();
  
  /**
   * 타임아웃 시간 설정
   * @param timeoutMs 타임아웃 시간 (밀리초)
   */
  void setTimeout(uint32_t timeoutMs);
};

#endif // SYSTEM_INIT_H

