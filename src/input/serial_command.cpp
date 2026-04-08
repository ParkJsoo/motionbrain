#include "serial_command.h"
#include "debug/debug_log.h"
#include "system/system_init.h"
#include "motor/motor_driver.h"
#include "motion/robot_arm.h"
#include "motion/motion_sequence.h"
#include "peripheral/search_light.h"

/**
 * SerialCommand 생성자
 */
SerialCommand::SerialCommand()
  : commandReady_(false)
  , bufferIndex_(0)
  , overflowDropping_(false)
  , systemState_(nullptr)
  , motorControl_(nullptr)
  , robotArm_(nullptr)
  , motionSequence_(nullptr)
  , searchLight_(nullptr)
{
  // 버퍼 초기화
  commandBuffer_[0] = '\0';
}

/**
 * 초기화
 * 시리얼 통신 준비
 */
void SerialCommand::init(SystemStateManager* systemState, MotorControl* motorControl, RobotArm* robotArm, MotionSequence* motionSequence, SearchLight* searchLight) {
  // 외부 객체 참조 저장
  systemState_    = systemState;
  motorControl_   = motorControl;
  robotArm_       = robotArm;
  motionSequence_ = motionSequence;
  searchLight_    = searchLight;
  
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
  // 이전 명령어 처리 완료 전까지 새 입력 무시 (\r\n 연속 수신 시 버퍼 오염 방지)
  if (commandReady_) return;

  // 시리얼 데이터가 있는지 확인
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // 줄바꿈 또는 캐리지 리턴이면 명령어 완성 (또는 오버플로우 드롭 종료)
    if (c == '\n' || c == '\r') {
      if (overflowDropping_) {
        // 오버플로우 후 나머지 문자 버리기 완료 — 다음 명령어 대기
        overflowDropping_ = false;
        bufferIndex_ = 0;
        commandBuffer_[0] = '\0';
      } else if (bufferIndex_ > 0) {
        // 명령어 완성
        commandBuffer_[bufferIndex_] = '\0';  // 문자열 종료
        commandReady_ = true;
        bufferIndex_ = 0;  // 다음 명령어를 위해 인덱스 리셋

        // 디버그 로그 (나중에 제거 가능)
        DebugLog::debug("Command received: %s", commandBuffer_);
      }
    }
    // 오버플로우 드롭 중이면 '\n' 전까지 모두 버림
    else if (overflowDropping_) {
      // no-op: discard
    }
    // 일반 문자면 버퍼에 추가
    else if (bufferIndex_ < BUFFER_SIZE - 1) {
      commandBuffer_[bufferIndex_] = c;
      bufferIndex_++;
    }
    // 버퍼 오버플로우 — '\n'까지 나머지 문자 버림
    else {
      DebugLog::warn("Command buffer overflow - command too long, discarding until newline");
      overflowDropping_ = true;
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
  
  if (strcasecmp(cmdName, "help") == 0) {
    handleHelp();
  }
  else if (strcasecmp(cmdName, "status") == 0) {
    handleStatus();
  }
  else if (strcasecmp(cmdName, "arm") == 0) {
    handleArm();
  }
  else if (strcasecmp(cmdName, "disarm") == 0) {
    handleDisarm();
  }
  else if (strcasecmp(cmdName, "stop") == 0) {
    handleStop();
  }
  else if (strcasecmp(cmdName, "motor") == 0) {
    handleMotor(args);
  }
  else if (strcasecmp(cmdName, "joint") == 0) {
    handleJoint(args);
  }
  else if (strcasecmp(cmdName, "sequence") == 0) {
    handleSequence(args);
  }
  else if (strcasecmp(cmdName, "light") == 0) {
    handleLight(args);
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
  DebugLog::info("  stop      - Emergency stop / FAULT 상태 복구");
  DebugLog::info("");
  DebugLog::info("=== Motor Control Commands ===");
  DebugLog::info("  motor forward <id> [percent]  - Motor forward (default: 100%%)");
  DebugLog::info("  motor reverse <id> [percent]  - Motor reverse (default: 100%%)");
  DebugLog::info("  motor stop <id>               - Stop specific motor");
  DebugLog::info("  motor stop all               - Stop all motors (stay ARMED)");
  DebugLog::info("  motor status                 - Show all motor status");
  DebugLog::info("  motor default <speed>        - Set default speed (0-255)");
  DebugLog::info("");
  DebugLog::info("Examples:");
  DebugLog::info("  motor forward 1        - M1 forward at default speed");
  DebugLog::info("  motor forward 1 50     - M1 forward at 50%% speed");
  DebugLog::info("  motor reverse 5        - M5 reverse at default speed");
  DebugLog::info("  motor stop 2           - Stop M2");
  DebugLog::info("  motor default 150      - Set default speed to 150");
  DebugLog::info("");
  DebugLog::info("=== Joint Control Commands (Phase 2-A) ===");
  DebugLog::info("  joint gripper open [%%]  - Open gripper (M1)");
  DebugLog::info("  joint gripper close [%%] - Close gripper (M1)");
  DebugLog::info("  joint gripper stop      - Stop gripper");
  DebugLog::info("  joint wrist up [%%]      - Wrist up (M2)");
  DebugLog::info("  joint wrist down [%%]    - Wrist down (M2)");
  DebugLog::info("  joint wrist stop        - Stop wrist");
  DebugLog::info("  joint elbow up [%%]      - Elbow up (M3)");
  DebugLog::info("  joint elbow down [%%]    - Elbow down (M3)");
  DebugLog::info("  joint elbow stop        - Stop elbow");
  DebugLog::info("  joint shoulder up [%%]   - Shoulder up (M4)");
  DebugLog::info("  joint shoulder down [%%] - Shoulder down (M4)");
  DebugLog::info("  joint shoulder stop     - Stop shoulder");
  DebugLog::info("  joint base left [%%]     - Base rotate left (M5)");
  DebugLog::info("  joint base right [%%]    - Base rotate right (M5)");
  DebugLog::info("  joint base stop         - Stop base");
  DebugLog::info("  joint stop              - Stop all joints (stay ARMED)");
  DebugLog::info("");
  DebugLog::info("=== Sequence Commands (Phase 2-B) ===");
  DebugLog::info("  sequence add <joint> <dir> <speed%%> <ms>  - Add command to queue");
  DebugLog::info("  sequence run             - Start executing sequence");
  DebugLog::info("  sequence stop            - Stop running sequence");
  DebugLog::info("  sequence clear           - Clear all commands");
  DebugLog::info("  sequence status          - Show sequence state");
  DebugLog::info("");
  DebugLog::info("Examples:");
  DebugLog::info("  sequence add shoulder up 50 2000  - Shoulder up 50%% for 2 sec");
  DebugLog::info("  sequence add gripper open 80 1000 - Gripper open 80%% for 1 sec");
  DebugLog::info("  sequence run");
  DebugLog::info("");
  DebugLog::info("=== Search Light Commands ===");
  DebugLog::info("  light on      - Turn on search light");
  DebugLog::info("  light off     - Turn off search light");
  DebugLog::info("  light toggle  - Toggle search light");
  DebugLog::info("  light status  - Show light state");
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
    for (uint8_t i = 1; i <= MotorControl::NUM_MOTORS; i++) {
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
  if (!systemState_->enterSafe()) {
    DebugLog::warn("STOP: enterSafe() failed - state may already be safe");
  }
  
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
    DebugLog::info("  motor stop all");
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
  if (strcasecmp(action, "forward") == 0) {
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
    
    if (motorId < 1 || motorId > MotorControl::NUM_MOTORS) {
      DebugLog::error("Invalid motor ID: %d (valid range: 1-5)", motorId);
      return;
    }

    if (percent < 0 || percent > 100) {
      DebugLog::error("Invalid percent: %d (valid range: 0-100)", percent);
      return;
    }
    if (percent == 0) {
      DebugLog::error("Use 'motor stop %d' for 0%% speed", motorId);
      return;
    }

    bool result = motorControl_->forward(motorId, percent);
    if (result) {
      DebugLog::info("Motor M%d: forward at %d%% speed", motorId, percent);
    } else {
      DebugLog::warn("Failed to set motor M%d forward", motorId);
    }
  }
  else if (strcasecmp(action, "reverse") == 0) {
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
    
    if (motorId < 1 || motorId > MotorControl::NUM_MOTORS) {
      DebugLog::error("Invalid motor ID: %d (valid range: 1-5)", motorId);
      return;
    }

    if (percent < 0 || percent > 100) {
      DebugLog::error("Invalid percent: %d (valid range: 0-100)", percent);
      return;
    }
    if (percent == 0) {
      DebugLog::error("Use 'motor stop %d' for 0%% speed", motorId);
      return;
    }

    bool result = motorControl_->reverse(motorId, percent);
    if (result) {
      DebugLog::info("Motor M%d: reverse at %d%% speed", motorId, percent);
    } else {
      DebugLog::warn("Failed to set motor M%d reverse", motorId);
    }
  }
  else if (strcasecmp(action, "stop") == 0) {
    // motor stop all  — 모든 모터 정지 (ARMED 상태 유지)
    if (strcasecmp(rest, "all") == 0) {
      motorControl_->stopAll();
      DebugLog::info("All motors stopped (system remains ARMED)");
      return;
    }

    // motor stop <id>
    int motorId = 0;

    if (sscanf(rest, "%d", &motorId) < 1) {
      DebugLog::error("Invalid motor ID");
      return;
    }

    if (motorId < 1 || motorId > MotorControl::NUM_MOTORS) {
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
  else if (strcasecmp(action, "status") == 0) {
    // motor status
    DebugLog::info("=== Motor Status ===");
    DebugLog::info("Default speed: %d", motorControl_->getDefaultSpeed());
    
    const char* motorNames[] = {"Gripper", "Wrist", "Elbow", "Shoulder", "Base"};
    for (uint8_t i = 1; i <= MotorControl::NUM_MOTORS; i++) {
      int16_t speed = motorControl_->getSpeed(i);
      bool enabled = motorControl_->isEnabled(i);
      const char* direction = (speed > 0) ? "forward" : (speed < 0) ? "reverse" : "stopped";
      DebugLog::info("  M%d (%s): speed=%d (%s), enabled=%s", i, motorNames[i-1], speed, direction, enabled ? "YES" : "NO");
    }
  }
  else if (strcasecmp(action, "default") == 0) {
    // motor default <speed>
    int speed = 0;
    
    if (sscanf(rest, "%d", &speed) < 1) {
      DebugLog::error("Invalid speed value");
      return;
    }
    
    if (speed < 1 || speed > 255) {
      DebugLog::error("Invalid speed: %d (Speed must be 1-255 (0 = no movement, rejected))", speed);
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

/**
 * joint 명령어 처리 (Phase 2-A)
 * 형식: joint <joint_name> <action> [percent]
 *       joint stop
 */
void SerialCommand::handleJoint(const char* args) {
  if (robotArm_ == nullptr) {
    DebugLog::error("RobotArm not initialized");
    return;
  }

  if (args == nullptr || strlen(args) == 0) {
    DebugLog::warn("Usage: joint <joint> <action> [percent]");
    DebugLog::info("  Joints: gripper, wrist, elbow, shoulder, base");
    DebugLog::info("  Actions: open/close (gripper), up/down (wrist/elbow/shoulder),");
    DebugLog::info("           left/right (base), stop");
    DebugLog::info("  Or: joint stop  (stop all joints)");
    return;
  }

  // "stop" 단독 처리
  if (strcasecmp(args, "stop") == 0) {
    robotArm_->stopAll();
    DebugLog::info("All joints stopped (system remains ARMED)");
    return;
  }

  // joint 이름 파싱
  char jointName[CMD_NAME_SIZE];
  char rest[ARGS_SIZE];
  size_t i = 0;
  while (args[i] != '\0' && args[i] != ' ' && args[i] != '\t' && i < CMD_NAME_SIZE - 1) {
    jointName[i] = args[i];
    i++;
  }
  jointName[i] = '\0';

  // 나머지 파싱 (action [percent])
  while (args[i] == ' ' || args[i] == '\t') i++;
  size_t j = 0;
  while (args[i] != '\0' && j < ARGS_SIZE - 1) {
    rest[j++] = args[i++];
  }
  rest[j] = '\0';

  if (rest[0] == '\0') {
    DebugLog::warn("joint %s: action required (e.g. open, up, left, stop)", jointName);
    return;
  }

  // action 파싱
  char action[CMD_NAME_SIZE];
  i = 0;
  while (rest[i] != '\0' && rest[i] != ' ' && rest[i] != '\t' && i < CMD_NAME_SIZE - 1) {
    action[i] = rest[i];
    i++;
  }
  action[i] = '\0';

  // percent 파싱 (옵셔널)
  uint8_t percent = RobotArm::DEFAULT_SPEED;
  while (rest[i] == ' ' || rest[i] == '\t') i++;
  if (rest[i] != '\0') {
    int p = 0;
    if (sscanf(&rest[i], "%d", &p) == 1 && p >= 0 && p <= 100) {
      percent = (uint8_t)p;
    } else {
      DebugLog::warn("Invalid percent value — using default (%d%%)", percent);
    }
  }

  // percent=0은 동작 명령에 사용 불가 — stop 명령 사용
  if (percent == 0 && strcasecmp(action, "stop") != 0) {
    DebugLog::warn("joint %s %s: Use 'stop' for 0%% speed", jointName, action);
    return;
  }

  // 관절별 처리
  bool result = false;

  if (strcasecmp(jointName, "gripper") == 0) {
    if      (strcasecmp(action, "open")  == 0) result = robotArm_->gripperOpen(percent);
    else if (strcasecmp(action, "close") == 0) result = robotArm_->gripperClose(percent);
    else if (strcasecmp(action, "stop")  == 0) result = robotArm_->gripperStop();
    else { DebugLog::warn("gripper: unknown action '%s' (open/close/stop)", action); return; }
  }
  else if (strcasecmp(jointName, "wrist") == 0) {
    if      (strcasecmp(action, "up")   == 0) result = robotArm_->wristUp(percent);
    else if (strcasecmp(action, "down") == 0) result = robotArm_->wristDown(percent);
    else if (strcasecmp(action, "stop") == 0) result = robotArm_->wristStop();
    else { DebugLog::warn("wrist: unknown action '%s' (up/down/stop)", action); return; }
  }
  else if (strcasecmp(jointName, "elbow") == 0) {
    if      (strcasecmp(action, "up")   == 0) result = robotArm_->elbowUp(percent);
    else if (strcasecmp(action, "down") == 0) result = robotArm_->elbowDown(percent);
    else if (strcasecmp(action, "stop") == 0) result = robotArm_->elbowStop();
    else { DebugLog::warn("elbow: unknown action '%s' (up/down/stop)", action); return; }
  }
  else if (strcasecmp(jointName, "shoulder") == 0) {
    if      (strcasecmp(action, "up")   == 0) result = robotArm_->shoulderUp(percent);
    else if (strcasecmp(action, "down") == 0) result = robotArm_->shoulderDown(percent);
    else if (strcasecmp(action, "stop") == 0) result = robotArm_->shoulderStop();
    else { DebugLog::warn("shoulder: unknown action '%s' (up/down/stop)", action); return; }
  }
  else if (strcasecmp(jointName, "base") == 0) {
    if      (strcasecmp(action, "left")  == 0) result = robotArm_->baseLeft(percent);
    else if (strcasecmp(action, "right") == 0) result = robotArm_->baseRight(percent);
    else if (strcasecmp(action, "stop")  == 0) result = robotArm_->baseStop();
    else { DebugLog::warn("base: unknown action '%s' (left/right/stop)", action); return; }
  }
  else {
    DebugLog::warn("Unknown joint: '%s' (gripper/wrist/elbow/shoulder/base)", jointName);
    return;
  }

  if (!result) {
    DebugLog::warn("joint %s %s: failed (system ARMED?)", jointName, action);
  }
}

/**
 * sequence 명령어 처리 (Phase 2-B)
 * 형식: sequence <action> [joint direction speed durationMs]
 *   add <joint> <direction> <speed> <durationMs>
 *   run | stop | clear | status
 */
void SerialCommand::handleSequence(const char* args) {
  if (motionSequence_ == nullptr) {
    DebugLog::error("MotionSequence not initialized");
    return;
  }

  if (args == nullptr || strlen(args) == 0) {
    DebugLog::warn("Usage: sequence <add|run|stop|clear|status>");
    return;
  }

  // action 추출
  char action[CMD_NAME_SIZE];
  size_t i = 0;
  while (args[i] != '\0' && args[i] != ' ' && args[i] != '\t' && i < CMD_NAME_SIZE - 1) {
    action[i] = args[i];
    i++;
  }
  action[i] = '\0';

  // 나머지 인자
  while (args[i] == ' ' || args[i] == '\t') i++;
  const char* rest = &args[i];

  if (strcasecmp(action, "status") == 0) {
    DebugLog::info("=== Sequence Status ===");
    DebugLog::info("  State   : %s", MotionSequence::stateToString(motionSequence_->getState()));
    DebugLog::info("  Step    : %d / %d", motionSequence_->getCurrentIndex() + 1,
                   motionSequence_->getTotalCount());
    DebugLog::info("  Remaining: %lums", motionSequence_->getRemainingMs());
    return;
  }

  if (strcasecmp(action, "run") == 0) {
    if (motionSequence_->run()) {
      DebugLog::info("Sequence started (%d commands)", motionSequence_->getTotalCount());
    } else {
      DebugLog::warn("Sequence run failed — check state and ARMED status");
    }
    return;
  }

  if (strcasecmp(action, "stop") == 0) {
    motionSequence_->stop();
    DebugLog::info("Sequence stopped");
    return;
  }

  if (strcasecmp(action, "clear") == 0) {
    motionSequence_->clear();
    DebugLog::info("Sequence cleared");
    return;
  }

  if (strcasecmp(action, "add") == 0) {
    // "add <joint> <direction> <speed> <durationMs>"
    // rest 예: "shoulder up 50 2000"
    char jointStr[CMD_NAME_SIZE];
    char dirStr[CMD_NAME_SIZE];
    int  speed    = 0;
    long duration = 0;

    int parsed = sscanf(rest, "%31s %31s %d %ld", jointStr, dirStr, &speed, &duration);
    if (parsed < 4) {
      DebugLog::warn("sequence add: needs <joint> <direction> <speed%%> <durationMs>");
      DebugLog::info("  Example: sequence add shoulder up 50 2000");
      return;
    }

    MotionJoint     joint;
    MotionDirection direction;

    if (!MotionSequence::parseJoint(jointStr, joint)) {
      DebugLog::warn("sequence add: unknown joint '%s'", jointStr);
      return;
    }
    if (!MotionSequence::parseDirection(joint, dirStr, direction)) {
      DebugLog::warn("sequence add: invalid direction '%s' for joint '%s'", dirStr, jointStr);
      return;
    }
    if (speed < 1 || speed > 100) {
      DebugLog::warn("sequence add: speed must be 1-100 (got %d)", speed);
      return;
    }
    if (duration <= 0) {
      DebugLog::warn("sequence add: durationMs must be > 0 (got %ld)", duration);
      return;
    }

    if (systemState_ != nullptr) {
      systemState_->resetTimeout();
    }
    if (motionSequence_->addCommand(joint, direction, (uint8_t)speed, (uint32_t)duration)) {
      DebugLog::info("sequence add: [%d/%d] %s %s %d%% %ldms",
                     motionSequence_->getTotalCount(), MotionSequence::MAX_COMMANDS,
                     jointStr, dirStr, speed, duration);
    } else {
      DebugLog::warn("sequence add: failed (queue full or invalid params)");
    }
    return;
  }

  DebugLog::warn("sequence: unknown action '%s' (add/run/stop/clear/status)", action);
}

/**
 * light 명령어 처리
 * 형식: light <on|off|toggle|status>
 */
void SerialCommand::handleLight(const char* args) {
  if (searchLight_ == nullptr) {
    DebugLog::error("SearchLight not initialized");
    return;
  }

  if (args == nullptr || strlen(args) == 0 || strcasecmp(args, "status") == 0) {
    DebugLog::info("SearchLight: %s", searchLight_->isOn() ? "ON" : "OFF");
    return;
  }

  if (strcasecmp(args, "on") == 0) {
    searchLight_->on();
  } else if (strcasecmp(args, "off") == 0) {
    searchLight_->off();
  } else if (strcasecmp(args, "toggle") == 0) {
    searchLight_->toggle();
    DebugLog::info("SearchLight: %s", searchLight_->isOn() ? "ON" : "OFF");
  } else {
    DebugLog::warn("light: unknown action '%s' (on/off/toggle/status)", args);
  }
}
