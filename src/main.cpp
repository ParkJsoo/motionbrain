#include <Arduino.h>
#include "system/system_init.h"
#include "motor/motor_driver.h"        // MotorControl 사용
#include "motion/robot_arm.h"          // RobotArm 사용 (Phase 2-A)
#include "motion/motion_sequence.h"    // MotionSequence 사용 (Phase 2-B)
#include "input/serial_command.h"      // SerialCommand 사용
#include "input/teleop_adapter.h"      // TeleopAdapter 사용
#include "network/wifi_ap.h"           // WiFiAP 사용
#include "network/web_server.h"        // MotionBrainWebServer 사용
#include "peripheral/search_light.h"   // SearchLight 사용
#include "bridge/stm32_bridge.h"
#include "control/angle_controller.h"
#include "control/command_bus.h"
#include "control/event_log.h"
#include "control/dispatcher.h"
#include "control/safety_gate.h"
#include "safety/safety_monitor.h"
#include "debug/debug_log.h"

// 전역 객체 생성
SystemStateManager systemState;
MotorControl motorControl;        // MotorControl 객체 생성
RobotArm robotArm(&motorControl); // RobotArm 객체 생성 (Phase 2-A)
MotionSequence motionSequence;    // MotionSequence 객체 생성 (Phase 2-B)
SerialCommand serialCommand;      // SerialCommand 객체 생성
TeleopAdapter teleopAdapter;      // 유선 handheld teleop adapter
WiFiAP wifiAP;                    // WiFiAP 객체 생성
MotionBrainWebServer webServer;   // MotionBrainWebServer 객체 생성
SearchLight searchLight;          // SearchLight 객체 생성
Stm32Bridge stm32Bridge;          // STM32 센서 브리지
SafetyMonitor safetyMonitor;      // 센서 기반 safety 모니터
AngleController angleController;  // base 상대각 폐루프 제어
EventLog eventLog;                // 최근 시스템 이벤트 로그
CommandBus commandBus;            // 공통 명령 버스
Dispatcher dispatcher;            // 공통 명령 디스패처
SafetyGate safetyGate;            // 공통 safety 정책 게이트

/**
 * setup() - ESP32 부팅 시 한 번만 실행
 * 
 * 초기화 작업을 수행합니다.
 */
void setup() {
  // 1. 로그 시스템 초기화 (가장 먼저)
  DebugLog::init(115200);
  DebugLog::info("=== MotionBrain System Boot ===");
  
  // 2. SystemStateManager 객체는 이미 생성됨 (전역 변수)
  // 생성자는 자동으로 호출됨 (BOOT 상태로 시작)
  DebugLog::stateChange("BOOT", systemState.getStateString(), "System initialized");
  
  // 3. 현재 상태 확인 및 로그 출력
  SystemState currentState = systemState.getState();
  const char* stateString = systemState.getStateString();
  
  DebugLog::debug("Current state (enum): %d", (int)currentState);
  DebugLog::info("Current state (string): %s", stateString);
  
  // 4. 타임아웃 설정 (기본값은 생성자에서 5000ms로 설정됨)
  systemState.setTimeout(30000);
  DebugLog::debug("Safety timeout: 30000ms");
  
  // 5. 모터 제어 모듈 초기화 (stub 모드)
  if (!motorControl.init()) {
    DebugLog::error("Motor control initialization failed");
    // 오류 발생 시 시스템을 FAULT 상태로 전환
    systemState.transitionTo(SystemState::FAULT);
    return;  // 더 이상 진행하지 않음
  }
  
  // 6. BOOT → IDLE 자동 전환 (시스템 초기화 완료)
  if (systemState.getState() == SystemState::BOOT) {
    systemState.transitionTo(SystemState::IDLE);
  }
  
  // 7. 서치라이트 초기화
  searchLight.init();

  // 8. STM32 센서 브리지 및 safety 모니터 초기화
  stm32Bridge.init();
  safetyMonitor.init(&systemState, &motorControl, &motionSequence);
  angleController.init(&systemState, &robotArm, &safetyMonitor);

  // 9. 모션 시퀀스 초기화 (base angle step 지원)
  motionSequence.init(&robotArm, &systemState, &angleController);

  safetyGate.init(&systemState, &safetyMonitor);
  dispatcher.init(&systemState, &motorControl, &robotArm, &motionSequence, &searchLight,
                  &safetyGate, &angleController);

  // 10. 시리얼 명령 모듈 초기화
  serialCommand.init(&systemState, &motorControl, &robotArm, &motionSequence, &searchLight,
                     &commandBus, &dispatcher);

  // 10-B. 유선 handheld teleop adapter 초기화
  teleopAdapter.init(&systemState, &motorControl, &motionSequence, &safetyMonitor,
                     &angleController, &commandBus, &dispatcher);
  
  // 11. Wi-Fi AP 초기화
  const char* apSSID = "MotionBrain-AP";
  const char* apPassword = "motionbrain";  // WPA2 보호 — 배포 전 반드시 변경

  if (!wifiAP.init(apSSID, apPassword)) {
    DebugLog::error("Wi-Fi AP initialization failed");
    // Wi-Fi AP 실패는 치명적 오류는 아니므로 계속 진행
  } else {
    DebugLog::info("Wi-Fi AP: Ready for connections");
    DebugLog::info("Connect to SSID: %s", apSSID);
    DebugLog::info("AP IP: %s", wifiAP.getIP().toString().c_str());
  }

  // 12. 웹 서버 초기화 (Wi-Fi AP 이후에 초기화)
  if (!webServer.init(&systemState, &motorControl, &robotArm, &motionSequence, &searchLight,
                      &commandBus, &dispatcher)) {
    DebugLog::error("Web server initialization failed");
    // 웹 서버 실패는 치명적 오류는 아니므로 계속 진행
  } else {
    DebugLog::info("Web Server: Ready");
    DebugLog::info("Access dashboard at: http://%s", wifiAP.getIP().toString().c_str());
  }
  
  DebugLog::info("Boot complete - system is in %s state", systemState.getStateString());
  DebugLog::info("=== MotionBrain Core Ready: 5-axis control + web UI ===");
  eventLog.push("system", "BOOT_COMPLETE", EventSeverity::INFO, systemState.getStateString());
}

/**
 * loop() - setup() 후 무한 반복 실행
 * 
 * Phase 1.5-2: Wi-Fi AP + Web UI
 * - 상태 머신 업데이트 (타임아웃 체크)
 * - 시리얼 명령 처리
 * - Wi-Fi AP 클라이언트 접속 관리
 * - 웹 서버 HTTP 요청 처리
 */
void loop() {
  // 상태 머신 업데이트 (타임아웃 체크)
  systemState.update();

  // STM32 센서 브리지 업데이트
  stm32Bridge.update();

  // 센서 기반 safety 평가
  safetyMonitor.update(stm32Bridge.getSnapshot());

  // base 상대각 폐루프 업데이트
  angleController.update(stm32Bridge.getSnapshot());

  // 공통 명령 처리
  dispatcher.dispatchPending(commandBus);

  // 유선 handheld teleop 처리
  teleopAdapter.update();
  
  // 모터 제어 업데이트 (점진적 속도 변경 처리)
  motorControl.update();

  // 모션 시퀀스 업데이트 (Phase 2-B)
  motionSequence.update();
  
  // 시리얼 명령 처리
  serialCommand.update();
  
  // Wi-Fi AP 업데이트 (클라이언트 접속 상태 확인)
  wifiAP.update();
  
  // 웹 서버 업데이트 (HTTP 요청 처리)
  webServer.update();
  
  // 최소 딜레이 (CPU 부하 완화 + 웹 서버 응답성 개선)
  delay(1);
}
