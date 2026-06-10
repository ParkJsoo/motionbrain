#include "serial_command.h"
#include "debug/debug_log.h"
#include "bridge/stm32_bridge.h"
#include "control/angle_controller.h"
#include "control/command.h"
#include "control/command_bus.h"
#include "control/dispatcher.h"
#include "control/guarded_routine.h"
#include "control/guarded_routine_executor.h"
#include "input/teleop_adapter.h"
#include "safety/safety_monitor.h"
#include "system/system_init.h"
#include "motor/motor_driver.h"
#include "motion/robot_arm.h"
#include "motion/motion_sequence.h"
#include "network/wifi_provisioning.h"
#include "peripheral/search_light.h"

extern Stm32Bridge stm32Bridge;
extern SafetyMonitor safetyMonitor;
extern AngleController angleController;
extern TeleopAdapter teleopAdapter;

namespace {

constexpr size_t COMMAND_TOKEN_MAX_LEN = 64;

bool isSensitiveSerialCommand(const char* command) {
  if (command == nullptr) {
    return false;
  }

  while (*command == ' ' || *command == '\t') {
    command++;
  }
  return strncasecmp(command, "wifi token", 10) == 0;
}

} // namespace

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
  , commandBus_(nullptr)
  , dispatcher_(nullptr)
{
  // 버퍼 초기화
  commandBuffer_[0] = '\0';
}

/**
 * 초기화
 * 시리얼 통신 준비
 */
void SerialCommand::init(SystemStateManager* systemState, MotorControl* motorControl,
                         RobotArm* robotArm, MotionSequence* motionSequence,
                         SearchLight* searchLight, CommandBus* commandBus,
                         Dispatcher* dispatcher) {
  // 외부 객체 참조 저장
  systemState_    = systemState;
  motorControl_   = motorControl;
  robotArm_       = robotArm;
  motionSequence_ = motionSequence;
  searchLight_    = searchLight;
  commandBus_     = commandBus;
  dispatcher_     = dispatcher;
  
  // 시리얼 통신은 이미 DebugLog::init()에서 초기화됨
  // 여기서는 로그만 출력
  DebugLog::info("Serial command module initialized");
  DebugLog::info("Type 'help' for available commands");
}

bool SerialCommand::submitCommand(const Command& command, CommandResult& result) {
  if (commandBus_ == nullptr || dispatcher_ == nullptr) {
    result.success = false;
    strlcpy(result.message, "Command path not initialized", sizeof(result.message));
    DebugLog::error("%s", result.message);
    return false;
  }

  Command queued = command;
  queued.id = commandBus_->allocateId();
  queued.createdAtMs = millis();
  return dispatcher_->execute(queued, result);
}

void SerialCommand::logCommandResult(const CommandResult& result) {
  if (result.success) {
    DebugLog::info("%s", result.message);
  } else {
    DebugLog::warn("%s", result.message);
  }
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

        if (isSensitiveSerialCommand(commandBuffer_)) {
          DebugLog::debug("Command received: wifi token [redacted]");
        } else {
          DebugLog::debug("Command received: %s", commandBuffer_);
        }
        return;
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
  else if (strcasecmp(cmdName, "base") == 0) {
    handleBase(args);
  }
  else if (strcasecmp(cmdName, "sequence") == 0) {
    handleSequence(args);
  }
  else if (strcasecmp(cmdName, "light") == 0) {
    handleLight(args);
  }
  else if (strcasecmp(cmdName, "routine") == 0) {
    handleRoutine(args);
  }
  else if (strcasecmp(cmdName, "sensor") == 0) {
    handleSensor(args);
  }
  else if (strcasecmp(cmdName, "wifi") == 0) {
    handleWifi(args);
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
  DebugLog::info("  sensor    - Show sensor status / simulation control");
  DebugLog::info("  wifi      - Show/update Wi-Fi provisioning state");
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
  DebugLog::info("  base angle <dir> <deg> [%%] - Base relative angle (left/right, 3-180 deg)");
  DebugLog::info("  base stop               - Stop base / cancel angle control");
  DebugLog::info("");
  DebugLog::info("=== Sequence Commands (Phase 2-B) ===");
  DebugLog::info("  sequence add <joint> <dir> <speed%%> <ms>  - Add command to queue");
  DebugLog::info("  sequence add base <dir> <speed%%> angle=<deg> - Add base angle step");
  DebugLog::info("  sequence run             - Start executing sequence");
  DebugLog::info("  sequence stop            - Stop running sequence");
  DebugLog::info("  sequence clear           - Clear all commands");
  DebugLog::info("  sequence status          - Show sequence state");
  DebugLog::info("");
  DebugLog::info("Examples:");
  DebugLog::info("  sequence add shoulder up 50 2000  - Shoulder up 50%% for 2 sec");
  DebugLog::info("  sequence add gripper open 80 1000 - Gripper open 80%% for 1 sec");
  DebugLog::info("  sequence add base left 40 angle=45 - Base relative left 45 deg");
  DebugLog::info("  sequence run");
  DebugLog::info("");
  DebugLog::info("=== Guarded Routine Commands ===");
  DebugLog::info("  routine list                 - Show available routine plans");
  DebugLog::info("  routine dry-run <name>       - Build a dry-run plan and log event");
  DebugLog::info("  routine run <name> confirm=<code> - Validate confirmation/preflight, then reject");
  DebugLog::info("  routine status              - Show routine executor scaffold state");
  DebugLog::info("  routine abort               - Abort active routine executor scaffold");
  DebugLog::info("  example: routine run inspect confirm=confirm-inspect");
  DebugLog::info("  routines: inspect, open_gripper_check, stow, center_target_dry_run, soft_home_reference");
  DebugLog::info("");
  DebugLog::info("=== Search Light Commands ===");
  DebugLog::info("  light on      - Turn on search light");
  DebugLog::info("  light off     - Turn off search light");
  DebugLog::info("  light toggle  - Toggle search light");
  DebugLog::info("  light status  - Show light state");
  DebugLog::info("");
  DebugLog::info("=== Sensor Simulation Commands ===");
  DebugLog::info("  sensor status                 - Show sensor + simulation state");
  DebugLog::info("  sensor sim healthy [dist]     - Continuous healthy packets");
  DebugLog::info("  sensor sim obstacle [dist]    - Continuous obstacle packets");
  DebugLog::info("  sensor sim vibration [vibe]   - Continuous vibration fault packets");
  DebugLog::info("  sensor sim imu_fault          - Continuous imu fault packets");
  DebugLog::info("  sensor sim range_fault        - Continuous range fault packets");
  DebugLog::info("  sensor sim rotate <dir> <dps> - Continuous gyro packets for base angle");
  DebugLog::info("  sensor sim stale              - Emit once, then freeze for stale");
  DebugLog::info("  sensor sim off                - Disable simulation and clear snapshot");
  DebugLog::info("");
  DebugLog::info("=== Wi-Fi Provisioning Commands ===");
  DebugLog::info("  wifi status                  - Show Wi-Fi/token provisioning state");
  DebugLog::info("  wifi token <value>           - Update command token and reboot");
  DebugLog::info("");
  DebugLog::info("=== Teleop Bring-Up Note ===");
  DebugLog::info("  Single-STM32 remote bench: teleop frames can carry embedded safety telemetry");
  DebugLog::info("  Then watch 'status' for teleop connected/deadman/reach/lift/twist");
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

  const bool useStm32Sensor = stm32Bridge.isSimulationEnabled() || stm32Bridge.isConnected() ||
                              !teleopAdapter.hasEmbeddedSafetySnapshot();
  const SensorSnapshot& snapshot = useStm32Sensor
                                 ? stm32Bridge.getSnapshot()
                                 : teleopAdapter.getEmbeddedSafetySnapshot();
  const uint32_t sensorAgeMs = useStm32Sensor
                             ? stm32Bridge.getLastPacketAgeMs()
                             : teleopAdapter.getEmbeddedSafetyAgeMs();
  const uint32_t sensorPackets = useStm32Sensor
                               ? stm32Bridge.getPacketsReceived()
                               : teleopAdapter.getEmbeddedSafetyPacketsReceived();
  const uint32_t sensorParseErrors = useStm32Sensor ? stm32Bridge.getParseErrors() : teleopAdapter.getParseErrors();
  const bool sensorConnected = useStm32Sensor
                             ? stm32Bridge.isConnected()
                             : (teleopAdapter.getEmbeddedSafetyAgeMs() <= SafetyMonitor::SENSOR_STALE_MS);
  DebugLog::info("=== Sensor Status ===");
  DebugLog::info("Source: %s", useStm32Sensor ? "stm32_bridge" : "teleop_embedded");
  DebugLog::info("Connected: %s", sensorConnected ? "YES" : "NO");
  DebugLog::info("Simulation: %s", stm32Bridge.getSimulationModeString());
  DebugLog::info("Packets : %lu", sensorPackets);
  DebugLog::info("Parse errors: %lu", sensorParseErrors);
  DebugLog::info("Last update age: %lums", sensorAgeMs);
  DebugLog::info("Source timestamp: %lums", snapshot.sourceTimestampMs);
  DebugLog::info("IMU OK / Range OK: %s / %s", snapshot.imuOk ? "YES" : "NO", snapshot.rangeOk ? "YES" : "NO");
  DebugLog::info("Gyro xyz: %.2f / %.2f / %.2f dps", snapshot.gyroX, snapshot.gyroY, snapshot.gyroZ);
  DebugLog::info("Roll / Pitch: %.2f / %.2f deg", snapshot.roll, snapshot.pitch);
  DebugLog::info("Distance: %.1f cm", snapshot.distanceCm);
  DebugLog::info("Vibration: %.2f", snapshot.vibe);
  DebugLog::info("Obstacle / Vibration safety: %s / %s",
                 snapshot.obstacleSafetyEnabled ? "ON" : "OFF",
                 snapshot.vibrationSafetyEnabled ? "ON" : "OFF");
  DebugLog::info("IMU diag: status=%lu addr=0x%02lX err=0x%08lX",
                 snapshot.imuStatus,
                 snapshot.imuAddress,
                 snapshot.imuError);
  DebugLog::info("I2C pins: SCL=%s SDA=%s",
                 snapshot.i2cSclHigh ? "HIGH" : "LOW",
                 snapshot.i2cSdaHigh ? "HIGH" : "LOW");
  DebugLog::info("Motion blocked: %s", safetyMonitor.isMotionBlocked() ? "YES" : "NO");
  DebugLog::info("Block reason: %s", safetyMonitor.getBlockReasonString());
  DebugLog::info("Fault latched: %s", safetyMonitor.hasLatchedFault() ? "YES" : "NO");
  DebugLog::info("Fault reason: %s", safetyMonitor.getLatchedFaultReasonString());
  DebugLog::info("=== Teleop Status ===");
  DebugLog::info("Connected: %s", teleopAdapter.isConnected() ? "YES" : "NO");
  DebugLog::info("Deadman / Active: %s / %s",
                 teleopAdapter.isDeadmanHeld() ? "YES" : "NO",
                 teleopAdapter.isControlActive() ? "YES" : "NO");
  DebugLog::info("Frame age: %lums", teleopAdapter.getLastFrameAgeMs());
  DebugLog::info("Packets / Parse errors: %lu / %lu",
                 teleopAdapter.getPacketsReceived(),
                 teleopAdapter.getParseErrors());
  DebugLog::info("Session / Sequence: %lu / %lu",
                 teleopAdapter.getLastSession(),
                 teleopAdapter.getLastSequence());
  DebugLog::info("Reach / Lift / Twist: %.2f / %.2f / %.2f",
                 teleopAdapter.getLastReach(),
                 teleopAdapter.getLastLift(),
                 teleopAdapter.getLastTwist());
  DebugLog::info("Grip open / close: %s / %s",
                 teleopAdapter.getLastGripOpen() ? "YES" : "NO",
                 teleopAdapter.getLastGripClose() ? "YES" : "NO");
  DebugLog::info("LED toggle seq: %lu", teleopAdapter.getLastLedToggleSeq());
  DebugLog::info("Last stop reason: %s", teleopAdapter.getLastStopReasonString());
  DebugLog::info("=== Base Angle Control ===");
  DebugLog::info("Active: %s", angleController.isActive() ? "YES" : "NO");
  DebugLog::info("Direction: %s", angleController.getDirectionString());
  DebugLog::info("Target / Current / Remaining: %.1f / %.1f / %.1f deg",
                 angleController.getTargetDegrees(),
                 angleController.getAccumulatedDegrees(),
                 angleController.getRemainingDegrees());
  DebugLog::info("Speed: %u%%", angleController.getPercent());
  DebugLog::info("Elapsed / Timeout: %lums / %lums",
                 angleController.getElapsedMs(),
                 angleController.getTimeoutMs());
  DebugLog::info("Samples / Last rate: %lu / %.2f dps",
                 angleController.getProcessedSamples(),
                 angleController.getLastRateDegreesPerSecond());
  DebugLog::info("Last stop reason: %s", angleController.getLastStopReasonString());
}

void SerialCommand::handleSensor(const char* args) {
  if (args == nullptr || strlen(args) == 0 || strcasecmp(args, "status") == 0) {
    handleStatus();
    return;
  }

  char action[CMD_NAME_SIZE];
  size_t i = 0;
  while (args[i] != '\0' && args[i] != ' ' && args[i] != '\t' && i < CMD_NAME_SIZE - 1) {
    action[i] = args[i];
    i++;
  }
  action[i] = '\0';

  while (args[i] == ' ' || args[i] == '\t') i++;
  const char* rest = &args[i];

  if (strcasecmp(action, "sim") != 0) {
    DebugLog::warn("sensor: unknown action '%s' (status/sim)", action);
    return;
  }

  if (rest == nullptr || rest[0] == '\0') {
    DebugLog::info("sensor sim: mode=%s", stm32Bridge.getSimulationModeString());
    return;
  }

  char mode[CMD_NAME_SIZE];
  i = 0;
  while (rest[i] != '\0' && rest[i] != ' ' && rest[i] != '\t' && i < CMD_NAME_SIZE - 1) {
    mode[i] = rest[i];
    i++;
  }
  mode[i] = '\0';

  while (rest[i] == ' ' || rest[i] == '\t') i++;
  const char* simArgs = &rest[i];

  if (strcasecmp(mode, "off") == 0) {
    stm32Bridge.clearSimulation(true);
    return;
  }

  SensorSnapshot snapshot;
  snapshot.connected = true;
  snapshot.imuOk = true;
  snapshot.rangeOk = true;
  snapshot.distanceCm = 50.0f;
  snapshot.vibe = 0.0f;

  if (strcasecmp(mode, "healthy") == 0) {
    float dist = 50.0f;
    if (simArgs[0] != '\0') {
      dist = strtof(simArgs, nullptr);
    }
    snapshot.distanceCm = dist > 0.0f ? dist : 50.0f;
    stm32Bridge.setSimulatedSnapshot(snapshot);
    return;
  }

  if (strcasecmp(mode, "obstacle") == 0) {
    float dist = 10.0f;
    if (simArgs[0] != '\0') {
      dist = strtof(simArgs, nullptr);
    }
    snapshot.distanceCm = dist > 0.0f ? dist : 10.0f;
    stm32Bridge.setSimulatedSnapshot(snapshot);
    return;
  }

  if (strcasecmp(mode, "vibration") == 0) {
    float vibe = 9.0f;
    if (simArgs[0] != '\0') {
      vibe = strtof(simArgs, nullptr);
    }
    snapshot.vibe = vibe > 0.0f ? vibe : 9.0f;
    stm32Bridge.setSimulatedSnapshot(snapshot);
    return;
  }

  if (strcasecmp(mode, "imu_fault") == 0) {
    snapshot.imuOk = false;
    stm32Bridge.setSimulatedSnapshot(snapshot);
    return;
  }

  if (strcasecmp(mode, "range_fault") == 0) {
    snapshot.rangeOk = false;
    stm32Bridge.setSimulatedSnapshot(snapshot);
    return;
  }

  if (strcasecmp(mode, "stale") == 0) {
    stm32Bridge.setSimulatedSnapshot(snapshot);
    stm32Bridge.update();
    stm32Bridge.freezeSimulation();
    return;
  }

  if (strcasecmp(mode, "rotate") == 0) {
    char directionStr[CMD_NAME_SIZE];
    float degreesPerSecond = 0.0f;
    int parsed = sscanf(simArgs, "%31s %f", directionStr, &degreesPerSecond);
    if (parsed < 2) {
      DebugLog::warn("sensor sim rotate: needs <left|right> <dps>");
      return;
    }

    if (degreesPerSecond <= 0.0f) {
      DebugLog::warn("sensor sim rotate: dps must be > 0");
      return;
    }

    if (strcasecmp(directionStr, "left") == 0) {
      snapshot.gyroZ = AngleController::GYRO_Z_LEFT_IS_POSITIVE
        ? degreesPerSecond
        : -degreesPerSecond;
    } else if (strcasecmp(directionStr, "right") == 0) {
      snapshot.gyroZ = AngleController::GYRO_Z_LEFT_IS_POSITIVE
        ? -degreesPerSecond
        : degreesPerSecond;
    } else {
      DebugLog::warn("sensor sim rotate: direction must be left or right");
      return;
    }

    stm32Bridge.setSimulatedSnapshot(snapshot);
    return;
  }

  DebugLog::warn("sensor sim: unknown mode '%s'", mode);
}

void SerialCommand::handleWifi(const char* args) {
  if (args == nullptr || args[0] == '\0') {
    DebugLog::info("Wi-Fi commands:");
    DebugLog::info("  wifi status");
    DebugLog::info("  wifi token <value>");
    return;
  }

  while (*args == ' ' || *args == '\t') {
    args++;
  }

  char action[16];
  size_t i = 0;
  while (args[i] != '\0' && args[i] != ' ' && args[i] != '\t' && i < sizeof(action) - 1) {
    action[i] = args[i];
    i++;
  }
  action[i] = '\0';

  const char* rest = args + i;
  while (*rest == ' ' || *rest == '\t') {
    rest++;
  }

  if (strcasecmp(action, "status") == 0) {
    WifiProvisioningConfig config;
    if (!WifiProvisioning::load(config)) {
      DebugLog::warn("Wi-Fi provisioning: no stored home Wi-Fi config");
      return;
    }
    DebugLog::info("Wi-Fi provisioning: SSID configured");
    DebugLog::info("Wi-Fi provisioning: command token %s",
                   config.commandToken[0] != '\0' ? "configured" : "missing");
    return;
  }

  if (strcasecmp(action, "token") == 0) {
    String token(rest);
    token.trim();
    if (token.length() == 0) {
      DebugLog::warn("Usage: wifi token <value>");
      return;
    }
    if (token.length() > COMMAND_TOKEN_MAX_LEN) {
      DebugLog::warn("Wi-Fi provisioning: command token too long");
      return;
    }
    if (!WifiProvisioning::saveCommandToken(token.c_str())) {
      DebugLog::warn("Wi-Fi provisioning: command token update failed");
      return;
    }
    DebugLog::info("Wi-Fi provisioning: command token updated; restarting controller");
    delay(200);
    ESP.restart();
    return;
  }

  DebugLog::warn("Unknown wifi command: %s", action);
}

/**
 * arm 명령어 처리
 */
void SerialCommand::handleArm() {
  if (systemState_ == nullptr) {
    DebugLog::error("SystemStateManager not initialized");
    return;
  }

  Command command;
  command.source = CommandSource::SERIAL_INPUT;
  command.type = CommandType::ARM;

  CommandResult result;
  submitCommand(command, result);
  logCommandResult(result);
}

/**
 * disarm 명령어 처리
 */
void SerialCommand::handleDisarm() {
  if (systemState_ == nullptr) {
    DebugLog::error("SystemStateManager not initialized");
    return;
  }

  Command command;
  command.source = CommandSource::SERIAL_INPUT;
  command.type = CommandType::DISARM;

  CommandResult result;
  submitCommand(command, result);
  logCommandResult(result);
}

/**
 * stop 명령어 처리
 */
void SerialCommand::handleStop() {
  if (systemState_ == nullptr) {
    DebugLog::error("SystemStateManager not initialized");
    return;
  }

  Command command;
  command.source = CommandSource::SERIAL_INPUT;
  command.type = CommandType::STOP;

  CommandResult result;
  submitCommand(command, result);
  logCommandResult(result);
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

    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::MOTOR_RUN;
    command.motorId = (uint8_t)motorId;
    command.forward = true;
    command.percent = (uint8_t)percent;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
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

    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::MOTOR_RUN;
    command.motorId = (uint8_t)motorId;
    command.forward = false;
    command.percent = (uint8_t)percent;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
  }
  else if (strcasecmp(action, "stop") == 0) {
    // motor stop all  — 모든 모터 정지 (ARMED 상태 유지)
    if (strcasecmp(rest, "all") == 0) {
      Command command;
      command.source = CommandSource::SERIAL_INPUT;
      command.type = CommandType::MOTOR_STOP_ALL;

      CommandResult result;
      submitCommand(command, result);
      logCommandResult(result);
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

    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::MOTOR_STOP;
    command.motorId = (uint8_t)motorId;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
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
    
    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::MOTOR_SET_DEFAULT_SPEED;
    command.speed = (uint8_t)speed;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
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
    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::JOINT_STOP_ALL;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
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

  Command command;
  command.source = CommandSource::SERIAL_INPUT;
  if (strcasecmp(jointName, "gripper") == 0) {
    command.joint = MotionJoint::GRIPPER;
    if      (strcasecmp(action, "open")  == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::OPEN; }
    else if (strcasecmp(action, "close") == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::CLOSE; }
    else if (strcasecmp(action, "stop")  == 0) { command.type = CommandType::JOINT_STOP; }
    else { DebugLog::warn("gripper: unknown action '%s' (open/close/stop)", action); return; }
  }
  else if (strcasecmp(jointName, "wrist") == 0) {
    command.joint = MotionJoint::WRIST;
    if      (strcasecmp(action, "up")   == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::UP; }
    else if (strcasecmp(action, "down") == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::DOWN; }
    else if (strcasecmp(action, "stop") == 0) { command.type = CommandType::JOINT_STOP; }
    else { DebugLog::warn("wrist: unknown action '%s' (up/down/stop)", action); return; }
  }
  else if (strcasecmp(jointName, "elbow") == 0) {
    command.joint = MotionJoint::ELBOW;
    if      (strcasecmp(action, "up")   == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::UP; }
    else if (strcasecmp(action, "down") == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::DOWN; }
    else if (strcasecmp(action, "stop") == 0) { command.type = CommandType::JOINT_STOP; }
    else { DebugLog::warn("elbow: unknown action '%s' (up/down/stop)", action); return; }
  }
  else if (strcasecmp(jointName, "shoulder") == 0) {
    command.joint = MotionJoint::SHOULDER;
    if      (strcasecmp(action, "up")   == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::UP; }
    else if (strcasecmp(action, "down") == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::DOWN; }
    else if (strcasecmp(action, "stop") == 0) { command.type = CommandType::JOINT_STOP; }
    else { DebugLog::warn("shoulder: unknown action '%s' (up/down/stop)", action); return; }
  }
  else if (strcasecmp(jointName, "base") == 0) {
    command.joint = MotionJoint::BASE;
    if      (strcasecmp(action, "left")  == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::LEFT; }
    else if (strcasecmp(action, "right") == 0) { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::RIGHT; }
    else if (strcasecmp(action, "stop")  == 0) { command.type = CommandType::JOINT_STOP; }
    else { DebugLog::warn("base: unknown action '%s' (left/right/stop)", action); return; }
  }
  else {
    DebugLog::warn("Unknown joint: '%s' (gripper/wrist/elbow/shoulder/base)", jointName);
    return;
  }

  command.percent = percent;

  CommandResult result;
  submitCommand(command, result);
  logCommandResult(result);
}

void SerialCommand::handleBase(const char* args) {
  if (args == nullptr || strlen(args) == 0) {
    DebugLog::warn("Usage: base <angle|stop> ...");
    DebugLog::info("  base angle <left|right> <degrees> [percent]");
    DebugLog::info("  base stop");
    return;
  }

  char action[CMD_NAME_SIZE];
  char rest[ARGS_SIZE];
  size_t i = 0;
  while (args[i] != '\0' && args[i] != ' ' && args[i] != '\t' && i < CMD_NAME_SIZE - 1) {
    action[i] = args[i];
    i++;
  }
  action[i] = '\0';

  while (args[i] == ' ' || args[i] == '\t') i++;
  size_t j = 0;
  while (args[i] != '\0' && j < ARGS_SIZE - 1) {
    rest[j++] = args[i++];
  }
  rest[j] = '\0';

  if (strcasecmp(action, "stop") == 0) {
    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::JOINT_STOP;
    command.joint = MotionJoint::BASE;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
    return;
  }

  if (strcasecmp(action, "angle") != 0) {
    DebugLog::warn("base: unknown action '%s' (angle/stop)", action);
    return;
  }

  char directionStr[CMD_NAME_SIZE];
  float degrees = 0.0f;
  int percent = AngleController::DEFAULT_SPEED;

  int parsed = sscanf(rest, "%31s %f %d", directionStr, &degrees, &percent);
  if (parsed < 2) {
    DebugLog::warn("base angle: needs <left|right> <degrees> [percent]");
    DebugLog::info("  Example: base angle left 45 40");
    return;
  }

  MotionDirection direction;
  if (strcasecmp(directionStr, "left") == 0) {
    direction = MotionDirection::LEFT;
  } else if (strcasecmp(directionStr, "right") == 0) {
    direction = MotionDirection::RIGHT;
  } else {
    DebugLog::warn("base angle: direction must be left or right");
    return;
  }

  if (degrees < AngleController::MIN_TARGET_DEGREES ||
      degrees > AngleController::MAX_TARGET_DEGREES) {
    DebugLog::warn("base angle: degrees must be %.0f-%.0f",
                   AngleController::MIN_TARGET_DEGREES,
                   AngleController::MAX_TARGET_DEGREES);
    return;
  }

  if (percent < 1 || percent > 100) {
    DebugLog::warn("base angle: percent must be 1-100");
    return;
  }

  Command command;
  command.source = CommandSource::SERIAL_INPUT;
  command.type = CommandType::BASE_ANGLE_RUN;
  command.joint = MotionJoint::BASE;
  command.direction = direction;
  command.percent = static_cast<uint8_t>(percent);
  command.targetDegrees = degrees;

  CommandResult result;
  submitCommand(command, result);
  logCommandResult(result);
}

/**
 * sequence 명령어 처리 (Phase 2-B)
 * 형식: sequence <action> [joint direction speed durationMs|angle=<deg>]
 *   add <joint> <direction> <speed> <durationMs>
 *   add base <left|right> <speed> angle=<deg>
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
    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::SEQUENCE_RUN;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
    return;
  }

  if (strcasecmp(action, "stop") == 0) {
    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::SEQUENCE_STOP;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
    return;
  }

  if (strcasecmp(action, "clear") == 0) {
    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::SEQUENCE_CLEAR;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
    return;
  }

  if (strcasecmp(action, "add") == 0) {
    // "add <joint> <direction> <speed> <durationMs|angle=deg>"
    char jointStr[CMD_NAME_SIZE];
    char dirStr[CMD_NAME_SIZE];
    int  speed    = 0;
    char valueStr[ARGS_SIZE];

    int parsed = sscanf(rest, "%31s %31s %d %47s", jointStr, dirStr, &speed, valueStr);
    if (parsed < 4) {
      DebugLog::warn("sequence add: needs <joint> <direction> <speed%%> <durationMs|angle=deg>");
      DebugLog::info("  Example: sequence add shoulder up 50 2000");
      DebugLog::info("  Example: sequence add base left 40 angle=45");
      return;
    }

    MotionJoint     joint;
    MotionDirection direction;
    uint32_t        durationMs = 0;
    float           targetDegrees = 0.0f;

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

    if (joint == MotionJoint::BASE && strncasecmp(valueStr, "angle=", 6) == 0) {
      char* endPtr = nullptr;
      targetDegrees = strtof(valueStr + 6, &endPtr);
      if (endPtr == valueStr + 6 || targetDegrees < AngleController::MIN_TARGET_DEGREES ||
          targetDegrees > AngleController::MAX_TARGET_DEGREES) {
        DebugLog::warn("sequence add: base angle must be %.0f-%.0f deg",
                       AngleController::MIN_TARGET_DEGREES,
                       AngleController::MAX_TARGET_DEGREES);
        return;
      }
    } else {
      char* endPtr = nullptr;
      long duration = strtol(valueStr, &endPtr, 10);
      if (endPtr == valueStr || duration <= 0) {
        DebugLog::warn("sequence add: durationMs must be > 0 (got %s)", valueStr);
        return;
      }
      durationMs = (uint32_t)duration;
    }

    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::SEQUENCE_ADD;
    command.joint = joint;
    command.direction = direction;
    command.percent = (uint8_t)speed;
    command.durationMs = durationMs;
    command.targetDegrees = targetDegrees;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);
    return;
  }

  DebugLog::warn("sequence: unknown action '%s' (add/run/stop/clear/status)", action);
}

void SerialCommand::handleRoutine(const char* args) {
  if (args == nullptr || strlen(args) == 0 || strcasecmp(args, "list") == 0) {
    DebugLog::info("=== Guarded Routines ===");
    for (uint8_t i = 0; i < GuardedRoutine::routineCount(); ++i) {
      DebugLog::info("  %s", GuardedRoutine::routineNameAt(i));
    }
    DebugLog::info("Use: routine dry-run <name>");
    DebugLog::info("Use: routine status | routine abort");
    return;
  }

  if (strcasecmp(args, "status") == 0) {
    const GuardedRoutineExecutorStatus status = GuardedRoutineExecutor::status();
    DebugLog::info("=== Guarded Routine Executor ===");
    DebugLog::info("State: %s", GuardedRoutineExecutor::stateToString(status.state));
    DebugLog::info("Routine: %s", status.routineName[0] != '\0' ? status.routineName : "(none)");
    DebugLog::info("Steps: current=%u total=%u motion=%u",
                   status.currentStep, status.totalSteps, status.motionStepCount);
    DebugLog::info("Timing: elapsed=%lums remaining=%lums",
                   status.elapsedMs, status.remainingMs);
    DebugLog::info("Last result: %s",
                   GuardedRoutineExecutor::resultToString(status.lastResult));
    DebugLog::info("Last detail: %s", status.lastDetail);
    return;
  }

  char action[CMD_NAME_SIZE];
  size_t i = 0;
  while (args[i] != '\0' && args[i] != ' ' && args[i] != '\t' && i < CMD_NAME_SIZE - 1) {
    action[i] = args[i];
    i++;
  }
  action[i] = '\0';

  while (args[i] == ' ' || args[i] == '\t') i++;
  const char* routineArgs = &args[i];

  if (strcasecmp(action, "abort") == 0 || strcasecmp(action, "cancel") == 0) {
    Command command;
    command.source = CommandSource::SERIAL_INPUT;
    command.type = CommandType::ROUTINE_ABORT;

    CommandResult result;
    submitCommand(command, result);
    logCommandResult(result);

    const GuardedRoutineExecutorReport report = GuardedRoutineExecutor::lastReport();
    DebugLog::info("Executor: state=%s result=%s detail=%s",
                   GuardedRoutineExecutor::stateToString(report.state),
                   GuardedRoutineExecutor::resultToString(report.result),
                   report.detail);
    DebugLog::info("Step journal: count=%u truncated=%s",
                   report.stepJournalCount,
                   report.stepJournalTruncated ? "YES" : "NO");
    return;
  }

  if (routineArgs[0] == '\0') {
    DebugLog::warn("routine: missing name");
    DebugLog::info("Use: routine dry-run <name>");
    return;
  }

  char routineName[CMD_NAME_SIZE] = {0};
  size_t nameIndex = 0;
  while (routineArgs[nameIndex] != '\0' &&
         routineArgs[nameIndex] != ' ' &&
         routineArgs[nameIndex] != '\t' &&
         nameIndex < sizeof(routineName) - 1) {
    routineName[nameIndex] = routineArgs[nameIndex];
    nameIndex++;
  }
  routineName[nameIndex] = '\0';

  while (routineArgs[nameIndex] == ' ' || routineArgs[nameIndex] == '\t') {
    nameIndex++;
  }
  const char* optionArgs = &routineArgs[nameIndex];

  Command command;
  command.source = CommandSource::SERIAL_INPUT;
  strlcpy(command.routineName, routineName, sizeof(command.routineName));

  while (optionArgs[0] != '\0') {
    char option[48] = {0};
    size_t optionIndex = 0;
    while (optionArgs[optionIndex] != '\0' &&
           optionArgs[optionIndex] != ' ' &&
           optionArgs[optionIndex] != '\t' &&
           optionIndex < sizeof(option) - 1) {
      option[optionIndex] = optionArgs[optionIndex];
      optionIndex++;
    }
    option[optionIndex] = '\0';

    if (strncasecmp(option, "confirm=", 8) == 0) {
      strlcpy(command.routineConfirmCode, option + 8, sizeof(command.routineConfirmCode));
    } else if (strncasecmp(option, "confirmCode=", 12) == 0) {
      strlcpy(command.routineConfirmCode, option + 12, sizeof(command.routineConfirmCode));
    }

    while (optionArgs[optionIndex] == ' ' || optionArgs[optionIndex] == '\t') {
      optionIndex++;
    }
    optionArgs += optionIndex;
  }

  if (strcasecmp(action, "dry-run") == 0 || strcasecmp(action, "dry_run") == 0 ||
      strcasecmp(action, "plan") == 0) {
    command.type = CommandType::ROUTINE_DRY_RUN;
  } else if (strcasecmp(action, "run") == 0 || strcasecmp(action, "execute") == 0) {
    command.type = CommandType::ROUTINE_RUN;
  } else {
    DebugLog::warn("routine: unknown action '%s' (list/status/dry-run/run/abort)", action);
    return;
  }

  GuardedRoutinePlan plan;
  const bool hasPlan = GuardedRoutine::getPlan(command.routineName, plan);

  CommandResult result;
  submitCommand(command, result);
  logCommandResult(result);

  if (!hasPlan) {
    return;
  }

  DebugLog::info("Routine: %s", plan.name);
  DebugLog::info("Summary: %s", plan.summary);
  DebugLog::info("Dry-run only: YES");
  DebugLog::info("Confirm code: %s", plan.confirmationCode);
  const GuardedRoutineExecutorReport report = GuardedRoutineExecutor::lastReport();
  DebugLog::info("Executor: state=%s result=%s started=%s",
                 GuardedRoutineExecutor::stateToString(report.state),
                 GuardedRoutineExecutor::resultToString(report.result),
                 report.sequenceStarted ? "YES" : "NO");
  DebugLog::info("Prepared sequence: result=%s ready=%s applied=%s steps=%u motion=%u",
                 GuardedRoutineExecutor::prepareResultToString(report.prepareResult),
                 report.prepareReady ? "YES" : "NO",
                 report.preparedSequenceApplied ? "YES" : "NO",
                 report.preparedStepCount,
                 report.preparedMotionCount);
  DebugLog::info("Step journal: count=%u truncated=%s",
                 report.stepJournalCount,
                 report.stepJournalTruncated ? "YES" : "NO");
  for (uint8_t journalIndex = 0; journalIndex < report.stepJournalCount; ++journalIndex) {
    const GuardedRoutineStepJournalEntry& entry = report.stepJournal[journalIndex];
    DebugLog::info("  journal %u. %s %s - %s",
                   entry.index,
                   entry.stepId,
                   GuardedRoutineExecutor::stepResultToString(entry.result),
                   entry.detail);
  }
  DebugLog::info("Steps: %u", plan.stepCount);
  for (uint8_t stepIndex = 0; stepIndex < plan.stepCount; ++stepIndex) {
    const GuardedRoutineStep& step = plan.steps[stepIndex];
    if (step.kind == GuardedRoutineStepKind::MOTION) {
      DebugLog::info("  %u. [%s] %s - %s %s %u%% %lums",
                     stepIndex + 1,
                     GuardedRoutine::stepKindToString(step.kind),
                     step.label,
                     GuardedRoutine::jointToString(step.joint),
                     GuardedRoutine::directionToString(step.direction),
                     step.percent,
                     step.durationMs);
    } else {
      DebugLog::info("  %u. [%s] %s - %s",
                     stepIndex + 1,
                     GuardedRoutine::stepKindToString(step.kind),
                     step.label,
                     step.detail);
    }
  }
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

  Command command;
  command.source = CommandSource::SERIAL_INPUT;

  if (strcasecmp(args, "on") == 0) {
    command.type = CommandType::LIGHT_ON;
  } else if (strcasecmp(args, "off") == 0) {
    command.type = CommandType::LIGHT_OFF;
  } else if (strcasecmp(args, "toggle") == 0) {
    command.type = CommandType::LIGHT_TOGGLE;
  } else {
    DebugLog::warn("light: unknown action '%s' (on/off/toggle/status)", args);
    return;
  }

  CommandResult result;
  submitCommand(command, result);
  logCommandResult(result);
}
