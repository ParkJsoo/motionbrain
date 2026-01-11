#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H

#include <Arduino.h>

// 전방 선언 (순환 참조 방지)
class SystemStateManager;
class MotorControl;

/**
 * MotionBrain Serial Command Module
 * 
 * Phase 1-4: 시리얼 명령 기반 제어
 * - 시리얼 입력을 읽어서 명령어로 파싱
 * - 명령어를 처리하여 시스템 제어
 * 
 * 목표:
 * - 네트워크 없이도 명령 흐름 완성
 * - 시리얼로 상태 전환 가능
 * - 잘못된 명령 무시 + 로그 출력
 */

/**
 * SerialCommand 클래스
 * 
 * 시리얼 입력을 처리하는 클래스
 * - 시리얼 버퍼에서 명령어 읽기
 * - 명령어 파싱 및 처리
 */
class SerialCommand {
public:
  /**
   * 생성자
   */
  SerialCommand();

  /**
   * 초기화
   * 시리얼 통신 준비
   * @param systemState SystemStateManager 참조 (명령어 처리용)
   * @param motorControl MotorControl 참조 (명령어 처리용)
   */
  void init(SystemStateManager* systemState, MotorControl* motorControl);

  /**
   * 업데이트 (주기적으로 호출)
   * 시리얼 입력을 확인하고 명령어 처리
   * loop()에서 주기적으로 호출해야 함
   */
  void update();

  /**
   * 명령어가 수신되었는지 확인
   * @return 명령어 수신 여부
   */
  bool hasCommand() const;

  /**
   * 수신된 명령어 가져오기
   * @return 명령어 문자열 (없으면 nullptr)
   */
  const char* getCommand() const;

  /**
   * 명령어 파싱 (명령어와 인자 분리)
   * 명령어 처리 전에 호출하여 명령어와 인자를 분리
   * @param command 전체 명령어 문자열
   * @param cmdName 파싱된 명령어 이름 (출력)
   * @param args 파싱된 인자 (출력, 없으면 nullptr)
   * @return 파싱 성공 여부
   */
  static bool parseCommand(const char* command, char* cmdName, char* args);

  /**
   * 명령어 처리 완료 후 호출
   * 다음 명령어를 받을 수 있도록 플래그 리셋
   */
  void clearCommand();

private:
  // 시리얼 입력 버퍼
  static const size_t BUFFER_SIZE = 64;  // 최대 명령어 길이
  static const size_t CMD_NAME_SIZE = 32; // 최대 명령어 이름 길이
  static const size_t ARGS_SIZE = 32;     // 최대 인자 길이
  char commandBuffer_[BUFFER_SIZE];      // 명령어 버퍼
  bool commandReady_;                    // 명령어 수신 완료 플래그
  size_t bufferIndex_;                   // 현재 버퍼 인덱스

  /**
   * 시리얼 입력 처리 (private)
   * 한 문자씩 읽어서 버퍼에 저장
   */
  void processSerialInput();

  /**
   * 명령어 처리 (private)
   * 파싱된 명령어를 적절한 처리 함수로 라우팅
   * @param cmdName 명령어 이름
   * @param args 명령어 인자
   */
  void processCommand(const char* cmdName, const char* args);

  // ===== 명령어 처리 함수들 (private) =====

  /**
   * help 명령어 처리
   * 사용 가능한 명령어 목록 출력
   */
  void handleHelp();

  /**
   * status 명령어 처리
   * 현재 시스템 상태 출력
   */
  void handleStatus();

  /**
   * arm 명령어 처리
   * 시스템 무장 (IDLE → ARMED)
   */
  void handleArm();

  /**
   * disarm 명령어 처리
   * 시스템 해제 (ARMED → IDLE)
   */
  void handleDisarm();

  /**
   * stop 명령어 처리
   * 비상 정지 (모든 상태 → IDLE)
   */
  void handleStop();
  
  /**
   * motor 명령어 처리
   * 모터 제어 명령 (forward, reverse, stop, status, default)
   * @param args 명령어 인자 (예: "forward 1 50", "stop 2", "status" 등)
   */
  void handleMotor(const char* args);

  // 외부 객체 참조 (명령어 처리용)
  SystemStateManager* systemState_;
  MotorControl* motorControl_;
};

#endif // SERIAL_COMMAND_H

