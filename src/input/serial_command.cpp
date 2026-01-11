#include "serial_command.h"
#include "debug/debug_log.h"
#include "system/system_init.h"
#include "motor/motor_driver.h"

/**
 * SerialCommand 생성자
 */
SerialCommand::SerialCommand()
  : commandReady_(false)
  , bufferIndex_(0)
  , systemState_(nullptr)
  , motorControl_(nullptr)
{
  // 버퍼 초기화
  commandBuffer_[0] = '\0';
}

/**
 * 초기화
 * 시리얼 통신 준비
 */
void SerialCommand::init(SystemStateManager* systemState, MotorControl* motorControl) {
  // 외부 객체 참조 저장
  systemState_ = systemState;
  motorControl_ = motorControl;
  
  // 시리얼 통신은 이미 DebugLog::init()에서 초기화됨
  // 여기서는 로그만 출력
  DebugLog::info("Serial command module initialized");
  DebugLog::info("Type 'help' for available commands");
}

/**
 * 업데이트 (주기적으로 호출)
 * 시리얼 입력을 확인하고 명령어 처리
 */
void SerialCommand::update() {
  // 시리얼 입력 처리
  processSerialInput();
  
  // 명령어가 완성되면 처리
  if (hasCommand()) {
    const char* fullCommand = getCommand();
    
    // 명령어 파싱
    char cmdName[CMD_NAME_SIZE];
    char args[ARGS_SIZE];
    
    if (parseCommand(fullCommand, cmdName, args)) {
      // 명령어 처리
      processCommand(cmdName, args);
    } else {
      // 파싱 실패
      DebugLog::warn("Failed to parse command: %s", fullCommand);
    }
    
    // 명령어 처리 완료
    clearCommand();
  }
}

/**
 * 명령어가 수신되었는지 확인
 */
bool SerialCommand::hasCommand() const {
  return commandReady_;
}

/**
 * 수신된 명령어 가져오기
 */
const char* SerialCommand::getCommand() const {
  if (commandReady_) {
    return commandBuffer_;
  }
  return nullptr;
}

/**
 * 명령어 파싱 (명령어와 인자 분리)
 * 
 * 예시:
 *   "help" → cmdName: "help", args: nullptr
 *   "arm" → cmdName: "arm", args: nullptr
 *   "setSpeed 100" → cmdName: "setSpeed", args: "100"
 *   "test motor 1" → cmdName: "test", args: "motor 1"
 * 
 * @param command 전체 명령어 문자열
 * @param cmdName 파싱된 명령어 이름 (출력 버퍼, 최소 CMD_NAME_SIZE 크기)
 * @param args 파싱된 인자 (출력 버퍼, 최소 ARGS_SIZE 크기, 없으면 빈 문자열)
 * @return 파싱 성공 여부
 */
bool SerialCommand::parseCommand(const char* command, char* cmdName, char* args) {
  if (command == nullptr || cmdName == nullptr || args == nullptr) {
    return false;
  }

  // 공백 제거 (앞쪽)
  while (*command == ' ' || *command == '\t') {
    command++;
  }

  // 빈 명령어 체크
  if (*command == '\0') {
    return false;
  }

  // 명령어 이름 추출 (첫 번째 공백까지)
  size_t i = 0;
  while (*command != '\0' && *command != ' ' && *command != '\t' && i < CMD_NAME_SIZE - 1) {
    cmdName[i] = *command;
    command++;
    i++;
  }
  cmdName[i] = '\0';  // 문자열 종료

  // 명령어 이름이 비어있으면 실패
  if (i == 0) {
    return false;
  }

  // 인자 추출 (나머지 부분)
  // 공백 건너뛰기
  while (*command == ' ' || *command == '\t') {
    command++;
  }

  // 인자가 있으면 복사
  if (*command != '\0') {
    i = 0;
    while (*command != '\0' && i < ARGS_SIZE - 1) {
      args[i] = *command;
      command++;
      i++;
    }
    args[i] = '\0';  // 문자열 종료
  } else {
    // 인자가 없으면 빈 문자열
    args[0] = '\0';
  }

  return true;
}

/**
 * 명령어 처리 완료 후 호출
 * 다음 명령어를 받을 수 있도록 플래그 리셋
 */
void SerialCommand::clearCommand() {
  commandReady_ = false;
  commandBuffer_[0] = '\0';
  bufferIndex_ = 0;
}

/**
 * 시리얼 입력 처리 (private)
 * 한 문자씩 읽어서 버퍼에 저장
 */
void SerialCommand::processSerialInput() {
  // 시리얼 데이터가 있는지 확인
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // 줄바꿈 또는 캐리지 리턴이면 명령어 완성
    if (c == '\n' || c == '\r') {
      if (bufferIndex_ > 0) {
        // 명령어 완성
        commandBuffer_[bufferIndex_] = '\0';  // 문자열 종료
        commandReady_ = true;
        bufferIndex_ = 0;  // 다음 명령어를 위해 인덱스 리셋
        
        // 디버그 로그 (나중에 제거 가능)
        DebugLog::debug("Command received: %s", commandBuffer_);
      }
    }
    // 일반 문자면 버퍼에 추가
    else if (bufferIndex_ < BUFFER_SIZE - 1) {
      commandBuffer_[bufferIndex_] = c;
      bufferIndex_++;
    }
    // 버퍼 오버플로우 방지
    else {
      DebugLog::warn("Command buffer overflow - command too long");
      // 버퍼 초기화
      bufferIndex_ = 0;
      commandBuffer_[0] = '\0';
    }
  }
}

/**
 * 명령어 처리 (private)
 * 파싱된 명령어를 적절한 처리 함수로 라우팅
 */
void SerialCommand::processCommand(const char* cmdName, const char* args) {
  if (cmdName == nullptr) {
    return;
  }

  // 명령어 이름 비교 (대소문자 구분 없이)
  // strcmp를 사용하여 문자열 비교
  
  if (strcmp(cmdName, "help") == 0) {
    handleHelp();
  }
  else if (strcmp(cmdName, "status") == 0) {
    handleStatus();
  }
  else if (strcmp(cmdName, "arm") == 0) {
    handleArm();
  }
  else if (strcmp(cmdName, "disarm") == 0) {
    handleDisarm();
  }
  else if (strcmp(cmdName, "stop") == 0) {
    handleStop();
  }
  else if (strcmp(cmdName, "motor") == 0) {
    handleMotor(args);
  }
  else {
    // 알 수 없는 명령어
    DebugLog::warn("Unknown command: %s", cmdName);
    DebugLog::info("Type 'help' for available commands");
  }
}

/**
 * help 명령어 처리
 */
void SerialCommand::handleHelp() {
  DebugLog::info("=== Available Commands ===");
  DebugLog::info("  help      - Show this help message");
  DebugLog::info("  status    - Show current system status");
  DebugLog::info("  arm       - Arm the system (IDLE -> ARMED)");
  DebugLog::info("  disarm    - Disarm the system (ARMED -> IDLE)");
  DebugLog::info("  stop      - Emergency stop (any state -> IDLE)");
  DebugLog::info("");
  DebugLog::info("=== Motor Control Commands ===");
  DebugLog::info("  motor forward <id> [percent]  - Motor forward (default: 100%%)");
  DebugLog::info("  motor reverse <id> [percent]  - Motor reverse (default: 100%%)");
  DebugLog::info("  motor stop <id>               - Stop specific motor");
  DebugLog::info("  motor status                 - Show all motor status");
  DebugLog::info("  motor default <speed>        - Set default speed (0-255)");
  DebugLog::info("");
  DebugLog::info("Examples:");
  DebugLog::info("  motor forward 1        - M1 forward at default speed");
  DebugLog::info("  motor forward 1 50     - M1 forward at 50%% speed");
  DebugLog::info("  motor reverse 5        - M5 reverse at default speed");
  DebugLog::info("  motor stop 2           - Stop M2");
  DebugLog::info("  motor default 150      - Set default speed to 150");
}

/**
 * status 명령어 처리
 */
void SerialCommand::handleStatus() {
  if (systemState_ == nullptr) {
    DebugLog::error("SystemStateManager not initialized");
    return;
  }
  
  const char* stateString = systemState_->getStateString();
  DebugLog::info("=== System Status ===");
  DebugLog::info("Current state: %s", stateString);
  
  // 모터 상태 표시
  if (motorControl_ != nullptr) {
    DebugLog::info("Motor enabled: %s", motorControl_->isEnabled() ? "YES" : "NO");
    DebugLog::info("Default speed: %d", motorControl_->getDefaultSpeed());
    
    // 각 모터 상태 표시
    DebugLog::info("=== Motor Status ===");
    const char* motorNames[] = {"Gripper", "Wrist", "Elbow", "Shoulder", "Base"};
    for (uint8_t i = 1; i <= 5; i++) {
      int16_t speed = motorControl_->getSpeed(i);
      bool enabled = motorControl_->isEnabled(i);
      DebugLog::info("  M%d (%s): speed=%d, enabled=%s", 
                     i, motorNames[i-1], speed, enabled ? "YES" : "NO");
    }
  }
}

/**
 * arm 명령어 처리
 */
void SerialCommand::handleArm() {
  if (systemState_ == nullptr) {
    DebugLog::error("SystemStateManager not initialized");
    return;
  }
  
  bool result = systemState_->arm();
  if (result) {
    DebugLog::info("System armed successfully");
  } else {
    DebugLog::warn("Failed to arm system - check current state");
  }
}

/**
 * disarm 명령어 처리
 */
void SerialCommand::handleDisarm() {
  if (systemState_ == nullptr) {
    DebugLog::error("SystemStateManager not initialized");
    return;
  }
  
  bool result = systemState_->disarm();
  if (result) {
    DebugLog::info("System disarmed successfully");
  } else {
    DebugLog::warn("Failed to disarm system - check current state");
  }
}

/**
 * stop 명령어 처리
 */
void SerialCommand::handleStop() {
  if (systemState_ == nullptr) {
    DebugLog::error("SystemStateManager not initialized");
    return;
  }
  
  // 비상 정지
  systemState_->enterSafe();
  
  // 모터도 비상 정지
  if (motorControl_ != nullptr) {
    motorControl_->emergencyStop();
  }
  
  DebugLog::info("Emergency stop activated");
}

/**
 * motor 명령어 처리
 */
void SerialCommand::handleMotor(const char* args) {
  if (motorControl_ == nullptr) {
    DebugLog::error("MotorControl not initialized");
    return;
  }
  
  if (args == nullptr || strlen(args) == 0) {
    DebugLog::warn("Motor command requires arguments");
    DebugLog::info("Usage: motor <action> [args]");
    DebugLog::info("  motor forward <id> [percent]");
    DebugLog::info("  motor reverse <id> [percent]");
    DebugLog::info("  motor stop <id>");
    DebugLog::info("  motor status");
    DebugLog::info("  motor default <speed>");
    return;
  }
  
  // 인자 파싱: "forward 1 50" -> action="forward", rest="1 50"
  char action[CMD_NAME_SIZE];
  char rest[ARGS_SIZE];
  
  // 첫 번째 단어 추출 (action)
  size_t i = 0;
  while (args[i] != '\0' && args[i] != ' ' && args[i] != '\t' && i < CMD_NAME_SIZE - 1) {
    action[i] = args[i];
    i++;
  }
  action[i] = '\0';
  
  // 나머지 부분 추출
  while (args[i] == ' ' || args[i] == '\t') {
    i++;
  }
  if (args[i] != '\0') {
    size_t j = 0;
    while (args[i] != '\0' && j < ARGS_SIZE - 1) {
      rest[j] = args[i];
      i++;
      j++;
    }
    rest[j] = '\0';
  } else {
    rest[0] = '\0';
  }
  
  // action에 따라 처리
  if (strcmp(action, "forward") == 0) {
    // motor forward <id> [percent]
    int motorId = 0;
    int percent = 100;  // 기본값
    
    // motorId 파싱
    if (sscanf(rest, "%d", &motorId) < 1) {
      DebugLog::error("Invalid motor ID");
      return;
    }
    
    // percent 파싱 (옵셔널)
    char* percentStr = strchr(rest, ' ');
    if (percentStr != nullptr) {
      percentStr++;  // 공백 건너뛰기
      if (sscanf(percentStr, "%d", &percent) < 1) {
        percent = 100;  // 파싱 실패 시 기본값
      }
    }
    
    if (motorId < 1 || motorId > 5) {
      DebugLog::error("Invalid motor ID: %d (valid range: 1-5)", motorId);
      return;
    }
    
    if (percent < 0 || percent > 100) {
      DebugLog::error("Invalid percent: %d (valid range: 0-100)", percent);
      return;
    }
    
    bool result = motorControl_->forward(motorId, percent);
    if (result) {
      DebugLog::info("Motor M%d: forward at %d%% speed", motorId, percent);
    } else {
      DebugLog::warn("Failed to set motor M%d forward", motorId);
    }
  }
  else if (strcmp(action, "reverse") == 0) {
    // motor reverse <id> [percent]
    int motorId = 0;
    int percent = 100;  // 기본값
    
    // motorId 파싱
    if (sscanf(rest, "%d", &motorId) < 1) {
      DebugLog::error("Invalid motor ID");
      return;
    }
    
    // percent 파싱 (옵셔널)
    char* percentStr = strchr(rest, ' ');
    if (percentStr != nullptr) {
      percentStr++;  // 공백 건너뛰기
      if (sscanf(percentStr, "%d", &percent) < 1) {
        percent = 100;  // 파싱 실패 시 기본값
      }
    }
    
    if (motorId < 1 || motorId > 5) {
      DebugLog::error("Invalid motor ID: %d (valid range: 1-5)", motorId);
      return;
    }
    
    if (percent < 0 || percent > 100) {
      DebugLog::error("Invalid percent: %d (valid range: 0-100)", percent);
      return;
    }
    
    bool result = motorControl_->reverse(motorId, percent);
    if (result) {
      DebugLog::info("Motor M%d: reverse at %d%% speed", motorId, percent);
    } else {
      DebugLog::warn("Failed to set motor M%d reverse", motorId);
    }
  }
  else if (strcmp(action, "stop") == 0) {
    // motor stop <id>
    int motorId = 0;
    
    if (sscanf(rest, "%d", &motorId) < 1) {
      DebugLog::error("Invalid motor ID");
      return;
    }
    
    if (motorId < 1 || motorId > 5) {
      DebugLog::error("Invalid motor ID: %d (valid range: 1-5)", motorId);
      return;
    }
    
    bool result = motorControl_->stop(motorId);
    if (result) {
      DebugLog::info("Motor M%d: stopped", motorId);
    } else {
      DebugLog::warn("Failed to stop motor M%d", motorId);
    }
  }
  else if (strcmp(action, "status") == 0) {
    // motor status
    DebugLog::info("=== Motor Status ===");
    DebugLog::info("Default speed: %d", motorControl_->getDefaultSpeed());
    
    const char* motorNames[] = {"Gripper", "Wrist", "Elbow", "Shoulder", "Base"};
    for (uint8_t i = 1; i <= 5; i++) {
      int16_t speed = motorControl_->getSpeed(i);
      bool enabled = motorControl_->isEnabled(i);
      const char* direction = (speed > 0) ? "forward" : (speed < 0) ? "reverse" : "stopped";
      DebugLog::info("  M%d (%s): speed=%d (%s), enabled=%s", i, motorNames[i-1], speed, direction, enabled ? "YES" : "NO");
    }
  }
  else if (strcmp(action, "default") == 0) {
    // motor default <speed>
    int speed = 0;
    
    if (sscanf(rest, "%d", &speed) < 1) {
      DebugLog::error("Invalid speed value");
      return;
    }
    
    if (speed < 0 || speed > 255) {
      DebugLog::error("Invalid speed: %d (valid range: 0-255)", speed);
      return;
    }
    
    bool result = motorControl_->setDefaultSpeed(speed);
    if (result) {
      DebugLog::info("Default speed set to: %d", speed);
    } else {
      DebugLog::warn("Failed to set default speed");
    }
  }
  else {
    DebugLog::warn("Unknown motor action: %s", action);
    DebugLog::info("Available actions: forward, reverse, stop, status, default");
  }
}
