#include "debug_log.h"
#include <stdarg.h>

// 정적 멤버 변수 초기화
bool DebugLog::initialized_ = false;

/**
 * 로그 시스템 초기화
 */
void DebugLog::init(uint32_t baudRate) {
  if (initialized_) {
    return;  // 이미 초기화됨
  }
  
  Serial.begin(baudRate);
  delay(500);  // 시리얼 포트 안정화 대기
  
  initialized_ = true;
  
  Serial.println("\n=== MotionBrain Debug Log System ===");
  Serial.println("Log system initialized");
}

/**
 * 로그 레벨을 문자열로 변환
 */
const char* DebugLog::levelToString(LogLevel level) {
  switch (level) {
    case LogLevel::DEBUG: return "DEBUG";
    case LogLevel::INFO:  return "INFO";
    case LogLevel::WARN:  return "WARN";
    case LogLevel::ERROR: return "ERROR";
    default:              return "UNKNOWN";
  }
}

/**
 * 기본 로그 함수
 */
void DebugLog::log(LogLevel level, const char* format, ...) {
  if (!initialized_) {
    init();  // 자동 초기화
  }
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  Serial.printf("[%s] %s\n", levelToString(level), buffer);
}

/**
 * DEBUG 레벨 로그
 */
void DebugLog::debug(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  log(LogLevel::DEBUG, buffer);
}

/**
 * INFO 레벨 로그
 */
void DebugLog::info(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  log(LogLevel::INFO, buffer);
}

/**
 * WARN 레벨 로그
 */
void DebugLog::warn(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  log(LogLevel::WARN, buffer);
}

/**
 * ERROR 레벨 로그
 */
void DebugLog::error(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  log(LogLevel::ERROR, buffer);
}

/**
 * 상태 변화 로그
 */
void DebugLog::stateChange(const char* from, const char* to, const char* reason) {
  if (!initialized_) {
    init();
  }
  
  if (reason) {
    Serial.printf("[STATE] %s -> %s [%s]\n", from, to, reason);
  } else {
    Serial.printf("[STATE] %s -> %s\n", from, to);
  }
}

/**
 * 명령 실행 로그
 */
void DebugLog::command(const char* command, bool success, const char* message) {
  if (!initialized_) {
    init();
  }
  
  if (success) {
    if (message) {
      Serial.printf("[CMD] %s: OK - %s\n", command, message);
    } else {
      Serial.printf("[CMD] %s: OK\n", command);
    }
  } else {
    if (message) {
      Serial.printf("[CMD] %s: FAILED - %s\n", command, message);
    } else {
      Serial.printf("[CMD] %s: FAILED\n", command);
    }
  }
}

/**
 * 안전 관련 로그
 */
void DebugLog::safety(const char* event, const char* details) {
  if (!initialized_) {
    init();
  }
  
  if (details) {
    Serial.printf("[SAFETY] %s: %s\n", event, details);
  } else {
    Serial.printf("[SAFETY] %s\n", event);
  }
}

/**
 * 모터 제어 로그 (동작 이름만)
 */
void DebugLog::motor(const char* action) {
  if (!initialized_) {
    init();
  }
  
  Serial.printf("[MOTOR] %s\n", action);
}

/**
 * 모터 제어 로그 (printf 스타일)
 */
void DebugLog::motor(const char* action, const char* format, ...) {
  if (!initialized_) {
    init();
  }
  
  if (format == nullptr) {
    // format이 없으면 동작 이름만 출력
    Serial.printf("[MOTOR] %s\n", action);
    return;
  }
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  Serial.printf("[MOTOR] %s: %s\n", action, buffer);
}

