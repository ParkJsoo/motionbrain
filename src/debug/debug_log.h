#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <Arduino.h>
#include <stdint.h>

/**
 * MotionBrain Debug Log System
 * 
 * 모든 상태 변화와 명령을 시리얼 로그로 출력
 * 로그 없는 동작은 버그로 간주
 */

// 로그 레벨
enum class LogLevel {
  DEBUG = 0,  // 디버그 정보
  INFO = 1,   // 일반 정보
  WARN = 2,   // 경고
  ERROR = 3   // 오류
};

/**
 * DebugLog 클래스
 * 
 * 정적(static) 메서드만 제공
 * 객체 생성 없이 사용: DebugLog::info("메시지");
 */
class DebugLog {
public:
  /**
   * 로그 시스템 초기화
   * @param baudRate 시리얼 통신 속도 (기본값: 115200)
   */
  static void init(uint32_t baudRate = 115200);
  
  // ===== 기본 로그 함수 =====
  
  /**
   * 기본 로그 함수
   * @param level 로그 레벨
   * @param format 포맷 문자열 (printf 스타일)
   * @param ... 가변 인자
   */
  static void log(LogLevel level, const char* format, ...);
  
  /**
   * DEBUG 레벨 로그
   */
  static void debug(const char* format, ...);
  
  /**
   * INFO 레벨 로그
   */
  static void info(const char* format, ...);
  
  /**
   * WARN 레벨 로그
   */
  static void warn(const char* format, ...);
  
  /**
   * ERROR 레벨 로그
   */
  static void error(const char* format, ...);
  
  // ===== 특수 로그 함수 =====
  
  /**
   * 상태 변화 로그
   * @param from 이전 상태 (문자열)
   * @param to 새 상태 (문자열)
   * @param reason 전환 이유 (선택적)
   */
  static void stateChange(const char* from, const char* to, const char* reason = nullptr);
  
  /**
   * 명령 실행 로그
   * @param command 명령 이름
   * @param success 성공 여부
   * @param message 추가 메시지 (선택적)
   */
  static void command(const char* command, bool success, const char* message = nullptr);
  
  /**
   * 안전 관련 로그
   * @param event 이벤트 이름
   * @param details 상세 정보 (선택적)
   */
  static void safety(const char* event, const char* details = nullptr);
  
  /**
   * 모터 제어 로그 (printf 스타일)
   * @param action 동작 이름
   * @param format 포맷 문자열 (없으면 nullptr)
   * @param ... 가변 인자
   */
  static void motor(const char* action, const char* format = nullptr, ...);
  
  /**
   * 모터 제어 로그 (동작 이름만)
   * @param action 동작 이름
   */
  static void motor(const char* action);

private:
  /**
   * 로그 레벨을 문자열로 변환
   */
  static const char* levelToString(LogLevel level);
  
  /**
   * 초기화 여부
   */
  static bool initialized_;
};

#endif // DEBUG_LOG_H

