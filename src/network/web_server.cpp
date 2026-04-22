#include "web_server.h"
#include "bridge/stm32_bridge.h"
#include "control/angle_controller.h"
#include "control/command.h"
#include "control/command_bus.h"
#include "control/event_log.h"
#include "control/dispatcher.h"
#include "safety/safety_monitor.h"
#include "input/teleop_adapter.h"
#include "system/system_init.h"       // SystemStateManager 사용
#include "motor/motor_driver.h"        // MotorControl 사용
#include "motion/robot_arm.h"          // RobotArm 사용
#include "motion/motion_sequence.h"    // MotionSequence 사용 (Phase 2-B)
#include "peripheral/search_light.h"   // SearchLight 사용
#include "debug/debug_log.h"

extern Stm32Bridge stm32Bridge;
extern SafetyMonitor safetyMonitor;
extern AngleController angleController;
extern EventLog eventLog;
extern TeleopAdapter teleopAdapter;

namespace {

constexpr const char* MESSAGE_SCHEMA_VERSION = "phase3.v1";

String jsonEscape(const String& raw) {
  String escaped;
  escaped.reserve(raw.length() + 8);

  for (size_t i = 0; i < raw.length(); ++i) {
    char c = raw[i];
    switch (c) {
      case '\\': escaped += "\\\\"; break;
      case '"':  escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default:   escaped += c; break;
    }
  }

  return escaped;
}

} // namespace

/**
 * MotionBrainWebServer 생성자
 */
MotionBrainWebServer::MotionBrainWebServer()
  : active_(false)
  , port_(80)
  , systemState_(nullptr)
  , motorControl_(nullptr)
  , robotArm_(nullptr)
  , motionSequence_(nullptr)
  , searchLight_(nullptr)
  , commandBus_(nullptr)
  , dispatcher_(nullptr)
{
  // 생성자에서는 초기화만 수행
  // 실제 서버 시작은 init()에서 수행
}

/**
 * 웹 서버 초기화
 */
bool MotionBrainWebServer::init(SystemStateManager* systemState, MotorControl* motorControl,
                                RobotArm* robotArm, MotionSequence* motionSequence,
                                SearchLight* searchLight, CommandBus* commandBus,
                                Dispatcher* dispatcher, uint16_t port) {
  systemState_    = systemState;
  motorControl_   = motorControl;
  robotArm_       = robotArm;
  motionSequence_ = motionSequence;
  searchLight_    = searchLight;
  commandBus_     = commandBus;
  dispatcher_     = dispatcher;
  port_           = port;

  DebugLog::info("=== Web Server Initialization ===");
  DebugLog::info("Port: %d", port_);

  // HTTP 라우트 등록 (begin() 이전에 먼저 등록해야 함)
  // 람다 함수를 사용하여 클래스 메서드 호출
  server_.on("/", HTTP_GET, [this]() { this->handleRoot(); });
  server_.on("/status", HTTP_GET, [this]() { this->handleStatus(); });
  server_.on("/events", HTTP_GET, [this]() { this->handleEvents(); });
  server_.on("/command", HTTP_POST, [this]() { this->handleCommand(); });
  server_.on("/motor", HTTP_POST, [this]() { this->handleMotor(); });
  server_.on("/joint", HTTP_POST, [this]() { this->handleJoint(); });
  server_.on("/base", HTTP_POST, [this]() { this->handleBase(); });
  server_.on("/sequence", HTTP_POST, [this]() { this->handleSequence(); });
  server_.on("/sequence", HTTP_GET,  [this]() { this->handleSequenceStatus(); });
  server_.on("/light",    HTTP_POST, [this]() { this->handleLight(); });
  server_.on("/favicon.ico", HTTP_GET, [this]() { this->handleFavicon(); });
  server_.on("/apple-touch-icon.png", HTTP_GET, [this]() { this->handleAppleTouchIcon(); });
  server_.on("/apple-touch-icon-precomposed.png", HTTP_GET, [this]() { this->handleAppleTouchIcon(); });
  server_.on("/apple-touch-icon-120x120.png", HTTP_GET, [this]() { this->handleAppleTouchIcon(); });
  server_.on("/apple-touch-icon-120x120-precomposed.png", HTTP_GET, [this]() { this->handleAppleTouchIcon(); });
  server_.onNotFound([this]() { this->handleNotFound(); });

  // CSRF 방지: X-MotionBrain 헤더 수집
  const char* csrfHeader[] = {"X-MotionBrain"};
  server_.collectHeaders(csrfHeader, 1);

  // ESP32 WebServer 시작
  server_.begin();
  
  DebugLog::info("Web Server: Routes registered");
  DebugLog::debug("  GET  /         -> Dashboard");
  DebugLog::debug("  GET  /status   -> JSON status");
  DebugLog::debug("  GET  /events   -> JSON recent events");
  DebugLog::debug("  POST /command  -> Execute command");
  DebugLog::debug("  POST /motor    -> Motor control");
  DebugLog::debug("  POST /joint     -> Joint control");
  DebugLog::debug("  POST /base      -> Base angle control");
  DebugLog::debug("  POST /sequence  -> Sequence control");
  DebugLog::debug("  GET  /sequence  -> Sequence status");
  DebugLog::debug("  POST /light     -> Search light control");

  active_ = true;

  DebugLog::info("Web Server: Started successfully");
  DebugLog::info("Access dashboard at: http://192.168.4.1");

  return true;
}

bool MotionBrainWebServer::submitCommand(const Command& command, CommandResult& result) {
  if (commandBus_ == nullptr || dispatcher_ == nullptr) {
    result.success = false;
    strlcpy(result.message, "Command path not initialized", sizeof(result.message));
    return false;
  }

  Command queued = command;
  queued.id = commandBus_->allocateId();
  queued.createdAtMs = millis();
  return dispatcher_->execute(queued, result);
}

void MotionBrainWebServer::appendStateSummaryJson(String& json) const {
  json += "\"state\":\"";
  json += systemState_ != nullptr ? systemState_->getStateString() : "UNKNOWN";
  json += "\",\"sensorBlocked\":";
  json += safetyMonitor.isMotionBlocked() ? "true" : "false";
  json += ",\"blockReason\":\"";
  json += safetyMonitor.getBlockReasonString();
  json += "\",\"faultLatched\":";
  json += safetyMonitor.hasLatchedFault() ? "true" : "false";
  json += ",\"faultReason\":\"";
  json += safetyMonitor.getLatchedFaultReasonString();
  json += "\",\"baseAngleActive\":";
  json += angleController.isActive() ? "true" : "false";
  json += ",\"baseAngleReason\":\"";
  json += angleController.getLastStopReasonString();
  json += "\"";
}

void MotionBrainWebServer::sendErrorJson(int statusCode, const char* error, const String& details) {
  String json = "{\"schemaVersion\":\"";
  json += MESSAGE_SCHEMA_VERSION;
  json += "\",\"messageType\":\"error\",\"success\":false,\"error\":\"";
  json += jsonEscape(error != nullptr ? error : "Unknown error");
  json += "\"";

  if (details.length() > 0) {
    json += ",\"details\":\"";
    json += jsonEscape(details);
    json += "\"";
  }

  json += ",";
  appendStateSummaryJson(json);
  json += "}";
  server_.send(statusCode, "application/json", json);
}

void MotionBrainWebServer::sendCommandResult(const CommandResult& result, const String& extraJson) {
  String json = "{\"schemaVersion\":\"";
  json += MESSAGE_SCHEMA_VERSION;
  json += "\",\"messageType\":\"command_result\",\"success\":";
  json += result.success ? "true" : "false";
  json += ",\"commandId\":";
  json += String(result.commandId);
  json += ",\"message\":\"";
  json += jsonEscape(result.message);
  json += "\",";
  appendStateSummaryJson(json);
  if (extraJson.length() > 0) {
    json += ",";
    json += extraJson;
  }
  json += "}";
  server_.send(200, "application/json", json);
}

/**
 * 웹 서버 업데이트
 * HTTP 요청 처리
 */
void MotionBrainWebServer::update() {
  if (!active_) {
    return;
  }

  // ESP32 WebServer의 handleClient() 호출
  // 이 메서드는 수신된 HTTP 요청을 처리합니다
  server_.handleClient();
}

/**
 * 웹 서버 활성화 여부 확인
 */
bool MotionBrainWebServer::isActive() const {
  return active_;
}

/**
 * GET / 처리
 * HTML 대시보드 페이지 반환
 * Step 3: 개선된 UI/UX
 */
void MotionBrainWebServer::handleRoot() {
  DebugLog::debug("Web Server: GET / requested");
  
  // HTML을 생성하면서 동시에 전송 (스트리밍 방식)
  // 문제: 전체 HTML을 먼저 생성하면 메모리 부족 또는 전송 실패 가능
  // 해결: 생성과 동시에 전송하여 메모리 사용 최소화
  server_.setContentLength(CONTENT_LENGTH_UNKNOWN);  // 청크 전송 모드
  server_.send(200, "text/html", "");
  
  // 헤더 부분 즉시 전송
  server_.sendContent("<!DOCTYPE html><html><head>");
  server_.sendContent("<title>MotionBrain Dashboard</title>");
  server_.sendContent("<meta charset=\"UTF-8\">");
  server_.sendContent("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
  server_.sendContent("<link rel=\"icon\" type=\"image/svg+xml\" href=\"/favicon.ico\">");
  server_.sendContent("<link rel=\"apple-touch-icon\" href=\"/apple-touch-icon.png\">");
  server_.sendContent("<style>");
  
  // 스타일 부분 전송 (생성과 동시에 전송)
  server_.sendContent("* { box-sizing: border-box; }");
  server_.sendContent("body { font-family: \"Segoe UI\", Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; }");
  server_.sendContent(".container { max-width: 600px; margin: 0 auto; }");
  server_.sendContent("h1 { color: white; text-align: center; margin-bottom: 30px; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }");
  server_.sendContent(".card { background: white; padding: 20px; border-radius: 10px; margin-bottom: 20px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }");
  server_.sendContent(".card-title { font-size: 14px; color: #666; margin-bottom: 10px; text-transform: uppercase; letter-spacing: 1px; }");
  server_.sendContent(".status-badge { display: inline-block; padding: 8px 16px; border-radius: 20px; font-weight: bold; font-size: 16px; }");
  server_.sendContent(".state-BOOT { background: #ffc107; color: #000; }");
  server_.sendContent(".state-IDLE { background: #9e9e9e; color: #fff; }");
  server_.sendContent(".state-ARMED { background: #4caf50; color: #fff; }");
  server_.sendContent(".state-FAULT { background: #f44336; color: #fff; }");
  server_.sendContent(".state-LOADING { background: #e0e0e0; color: #666; }");
  server_.sendContent(".info-row { display: flex; justify-content: space-between; align-items: center; padding: 10px 0; border-bottom: 1px solid #eee; }");
  server_.sendContent(".info-row:last-child { border-bottom: none; }");
  server_.sendContent(".info-label { color: #666; }");
  server_.sendContent(".info-value { font-weight: bold; color: #333; }");
  server_.sendContent(".button-group { display: flex; gap: 10px; flex-wrap: wrap; }");
  server_.sendContent("button { flex: 1; min-width: 120px; padding: 12px 20px; font-size: 16px; font-weight: bold; border: none; border-radius: 5px; cursor: pointer; transition: all 0.3s; touch-action: none; -webkit-touch-callout: none; -webkit-user-select: none; user-select: none; }");
  server_.sendContent("button:hover { transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.2); }");
  server_.sendContent("button:active { transform: translateY(0); }");
  server_.sendContent("button:disabled { opacity: 0.5; cursor: not-allowed; transform: none; }");
  server_.sendContent(".btn-arm { background: #4caf50; color: white; }");
  server_.sendContent(".btn-light-on { background: #ffc107; color: #333; }");
  server_.sendContent(".btn-light-off { background: #607d8b; color: white; }");
  server_.sendContent(".btn-disarm { background: #f44336; color: white; }");
  server_.sendContent(".btn-stop { background: #ff9800; color: white; }");
  server_.sendContent(".btn-forward { background: #2196f3; color: white; }");
  server_.sendContent(".btn-reverse { background: #9c27b0; color: white; }");
  server_.sendContent(".btn-motor-stop { background: #f44336; color: white; }");
  server_.sendContent(".btn-motor-stop:active { background: #d32f2f; }");
  server_.sendContent(".btn-forward:active { background: #1976d2; }");
  server_.sendContent(".btn-reverse:active { background: #7b1fa2; }");
  server_.sendContent(".btn-pressed { opacity: 0.7; transform: scale(0.95); box-shadow: inset 0 2px 4px rgba(0,0,0,0.3); }");
  server_.sendContent(".mode-selector { display: flex; gap: 10px; margin-bottom: 15px; padding: 10px; background: #f5f5f5; border-radius: 5px; }");
  server_.sendContent(".mode-button { flex: 1; padding: 10px; border: 2px solid #ddd; border-radius: 5px; background: white; cursor: pointer; font-weight: bold; transition: all 0.3s; }");
  server_.sendContent(".mode-button.active { background: #667eea; color: white; border-color: #667eea; }");
  server_.sendContent(".mode-button:hover { border-color: #667eea; }");
  server_.sendContent(".joystick-container { display: none; }");
  server_.sendContent(".joystick-container.active { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; }");
  server_.sendContent(".joystick-container.active > .joystick-motor-card:nth-child(5) { grid-column: 2 / 4; justify-self: center; }");
  server_.sendContent(".button-container { display: none; }");
  server_.sendContent(".button-container.active { display: block; }");
  server_.sendContent(".joystick-row { grid-column: 1 / -1; display: flex; justify-content: center; }");
  server_.sendContent(".joystick-motor-card { min-width: 0; width: 100%; }");
  server_.sendContent("@media (max-width: 600px) { .joystick-container.active { grid-template-columns: repeat(2, 1fr); gap: 10px; } }");
  server_.sendContent(".joystick-motor-card { background: #fff; border-radius: 10px; padding: 10px; margin-bottom: 10px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }");
  server_.sendContent(".joystick-wrapper { display: flex; flex-direction: column; gap: 10px; align-items: center; }");
  server_.sendContent(".joystick-area { position: relative; width: 60px; height: 60px; border-radius: 50%; background: linear-gradient(135deg, #e0e0e0 0%, #f5f5f5 100%); border: 2px solid #ddd; cursor: pointer; touch-action: none; user-select: none; flex-shrink: 0; }");
  server_.sendContent(".joystick-area.vertical-only { cursor: ns-resize; }");
  server_.sendContent(".joystick-area.horizontal-only { cursor: ew-resize; }");
  server_.sendContent(".joystick-area.disabled { opacity: 0.5; cursor: not-allowed; pointer-events: none; background: linear-gradient(135deg, #f0f0f0 0%, #e0e0e0 100%); }");
  server_.sendContent(".joystick-handle { position: absolute; width: 20px; height: 20px; border-radius: 50%; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); border: 2px solid #fff; box-shadow: 0 2px 6px rgba(0,0,0,0.3); top: 50%; left: 50%; transform: translate(-50%, -50%); transition: none; }");
  server_.sendContent(".joystick-handle.active { box-shadow: 0 4px 12px rgba(102, 126, 234, 0.6); }");
  server_.sendContent(".joystick-info { width: 100%; display: flex; flex-direction: column; align-items: center; text-align: center; gap: 4px; }");
  server_.sendContent(".joystick-speed { display: none; }");
  server_.sendContent(".joystick-direction { font-size: 12px; color: #666; margin: 0; line-height: 1.2; text-transform: uppercase; letter-spacing: 0.5px; }");
  server_.sendContent(".joystick-center-line { position: absolute; width: 2px; height: 100%; background: rgba(0,0,0,0.1); left: 50%; top: 0; transform: translateX(-50%); pointer-events: none; }");
  server_.sendContent(".joystick-center-line.horizontal { width: 100%; height: 2px; top: 50%; left: 0; transform: translateY(-50%); }");
  server_.sendContent(".motor-card { background: #fff; border-radius: 10px; padding: 15px; margin-bottom: 15px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }");
  server_.sendContent(".motor-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }");
  server_.sendContent(".motor-name { font-size: 16px; font-weight: bold; color: #333; }");
  server_.sendContent(".motor-role { font-size: 11px; color: #666; margin-top: 2px; }");
  server_.sendContent(".motor-status { font-size: 11px; padding: 3px 8px; border-radius: 12px; background: #e0e0e0; white-space: nowrap; }");
  server_.sendContent(".motor-status.active { background: #4caf50; color: white; }");
  server_.sendContent(".joystick-header-speed { font-size: 16px; font-weight: bold; color: #667eea; min-width: 45px; text-align: right; }");
  server_.sendContent(".motor-controls { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }");
  server_.sendContent(".speed-slider { flex: 1; min-width: 150px; }");
  server_.sendContent(".speed-value { min-width: 50px; text-align: center; font-weight: bold; }");
  server_.sendContent("input[type=\"range\"] { width: 100%; height: 6px; border-radius: 3px; background: #ddd; outline: none; }");
  server_.sendContent("input[type=\"range\"]::-webkit-slider-thumb { appearance: none; width: 18px; height: 18px; border-radius: 50%; background: #667eea; cursor: pointer; }");
  server_.sendContent("input[type=\"range\"]::-moz-range-thumb { width: 18px; height: 18px; border-radius: 50%; background: #667eea; cursor: pointer; border: none; }");
  server_.sendContent(".default-speed { margin-top: 15px; padding-top: 15px; border-top: 1px solid #eee; }");
  server_.sendContent(".default-speed-row { display: flex; gap: 10px; align-items: center; margin-bottom: 5px; }");
  server_.sendContent("input[type=\"number\"] { width: 80px; padding: 8px; border: 1px solid #ddd; border-radius: 4px; }");
  server_.sendContent("input[type=\"number\"]:invalid { border-color: #f44336; }");
  server_.sendContent(".validation-message { font-size: 12px; color: #f44336; min-height: 16px; margin-top: 2px; }");
  server_.sendContent(".validation-message.valid { color: #4caf50; }");
  server_.sendContent(".validation-message.hidden { display: none; }");
  server_.sendContent("input[type='number'].warning { border-color: #ff9800; }");
  server_.sendContent(".message { padding: 10px; border-radius: 5px; margin-top: 10px; display: none; }");
  server_.sendContent(".message.success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }");
  server_.sendContent(".message.error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }");
  server_.sendContent(".loading { display: inline-block; width: 12px; height: 12px; border: 2px solid #f3f3f3; border-top: 2px solid #667eea; border-radius: 50%; animation: spin 1s linear infinite; }");
  server_.sendContent("@keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }");
  server_.sendContent(".last-update { text-align: center; color: rgba(255,255,255,0.8); font-size: 12px; margin-top: 20px; }");
  server_.sendContent(".joint-speed-row { display: flex; gap: 10px; align-items: center; margin-bottom: 15px; padding-bottom: 10px; border-bottom: 1px solid #eee; }");
  server_.sendContent("</style></head><body>");
  
  // HTML 본문 전송
  server_.sendContent("<div class=\"container\">");
  server_.sendContent("<h1>🤖 MotionBrain Control</h1>");
  server_.sendContent("<div class=\"card\">");
  server_.sendContent("<div class=\"card-title\">System Status</div>");
  server_.sendContent("<div class=\"info-row\">");
  server_.sendContent("<span class=\"info-label\">Current State:</span>");
  server_.sendContent("<span class=\"status-badge state-LOADING\" id=\"state-badge\">LOADING</span>");
  server_.sendContent("</div>");
  server_.sendContent("<div class=\"info-row\">");
  server_.sendContent("<span class=\"info-label\">Motor Enabled:</span>");
  server_.sendContent("<span class=\"info-value\" id=\"motor\">-</span>");
  server_.sendContent("</div></div>");
  server_.sendContent("<div class=\"card\">");
  server_.sendContent("<div class=\"card-title\">Commands</div>");
  server_.sendContent("<div class=\"button-group\">");
  server_.sendContent("<button class=\"btn-arm\" id=\"btn-arm\" onclick=\"sendCommand('arm')\">ARM</button>");
  server_.sendContent("<button class=\"btn-disarm\" id=\"btn-disarm\" onclick=\"sendCommand('disarm')\">DISARM</button>");
  server_.sendContent("<button class=\"btn-stop\" id=\"btn-stop\" onclick=\"sendCommand('stop')\">STOP</button>");
  server_.sendContent("</div>");
  server_.sendContent("<div class=\"message\" id=\"message\"></div></div>");
  server_.sendContent("<div class=\"card\">");
  server_.sendContent("<div class=\"card-title\">Search Light</div>");
  server_.sendContent("<div class=\"button-group\">");
  server_.sendContent("<button class=\"btn-light-on\" id=\"btn-light-on\" onclick=\"sendLight('on')\">ON</button>");
  server_.sendContent("<button class=\"btn-light-off\" id=\"btn-light-off\" onclick=\"sendLight('off')\">OFF</button>");
  server_.sendContent("<button class=\"btn-stop\" onclick=\"sendLight('toggle')\">TOGGLE</button>");
  server_.sendContent("</div></div>");
  server_.sendContent("<div class=\"card\">");
  server_.sendContent("<div class=\"card-title\">Motor Control</div>");
  server_.sendContent("<div class=\"mode-selector\">");
  server_.sendContent("<button class=\"mode-button active\" id=\"mode-button\" onclick=\"switchMode('button')\">Button Mode</button>");
  server_.sendContent("<button class=\"mode-button\" id=\"mode-joystick\" onclick=\"switchMode('joystick')\">Joystick Mode</button>");
  server_.sendContent("</div>");
  server_.sendContent("<div class=\"default-speed\">");
  server_.sendContent("<div class=\"default-speed-row\">");
  server_.sendContent("<label>Default Speed:</label>");
  server_.sendContent("<input type=\"number\" id=\"default-speed\" min=\"1\" max=\"255\" step=\"1\" value=\"100\" oninput=\"validateDefaultSpeed()\" onchange=\"validateDefaultSpeed()\">");
  server_.sendContent("<button id=\"btn-set-speed\" onclick=\"setDefaultSpeed()\" style=\"padding: 8px 16px;\">Set</button>");
  server_.sendContent("</div>");
  server_.sendContent("<div class=\"validation-message hidden\" id=\"speed-validation\"></div></div>");
  
  // 모터 목록 (M1~M5) 전송 - 버튼 모드
  const char* motorNames[] = {"Gripper", "Wrist", "Elbow", "Shoulder", "Base"};
  server_.sendContent("<div class=\"button-container active\" id=\"button-container\">");
  for (int i = 1; i <= MotorControl::NUM_MOTORS; i++) {
    String motorCard = "<div class=\"motor-card\"><div class=\"motor-header\"><div><div class=\"motor-name\">M" + String(i) + "</div><div class=\"motor-role\">" + String(motorNames[i-1]) + "</div></div><div class=\"motor-status\" id=\"motor-status-" + String(i) + "\">STOPPED</div></div><div class=\"motor-controls\"><input type=\"range\" id=\"speed-" + String(i) + "\" min=\"0\" max=\"100\" value=\"100\" class=\"speed-slider\" oninput=\"updateSpeedValue(" + String(i) + ")\"><span class=\"speed-value\" id=\"speed-value-" + String(i) + "\">100%</span><button class=\"btn-forward\" id=\"btn-forward-" + String(i) + "\" onmousedown=\"motorStart(" + String(i) + ", 'forward', event)\" onmouseup=\"motorStop(" + String(i) + ", event)\" onmouseleave=\"motorStop(" + String(i) + ", event)\" ontouchstart=\"motorStart(" + String(i) + ", 'forward', event)\" ontouchend=\"motorStop(" + String(i) + ", event)\" ontouchcancel=\"motorStop(" + String(i) + ", event)\">Forward</button><button class=\"btn-reverse\" id=\"btn-reverse-" + String(i) + "\" onmousedown=\"motorStart(" + String(i) + ", 'reverse', event)\" onmouseup=\"motorStop(" + String(i) + ", event)\" onmouseleave=\"motorStop(" + String(i) + ", event)\" ontouchstart=\"motorStart(" + String(i) + ", 'reverse', event)\" ontouchend=\"motorStop(" + String(i) + ", event)\" ontouchcancel=\"motorStop(" + String(i) + ", event)\">Reverse</button><button class=\"btn-motor-stop\" onclick=\"motorStop(" + String(i) + ", event)\">Stop</button></div></div>";
    server_.sendContent(motorCard);
  }
  server_.sendContent("</div>");
  server_.sendContent("<div class=\"joystick-container\" id=\"joystick-container\">");
  for (int i = 1; i <= MotorControl::NUM_MOTORS; i++) {
    String isVertical = (i <= 4) ? "vertical-only" : "horizontal-only";
    String centerLine = (i <= 4) ? "<div class=\"joystick-center-line\"></div>" : "<div class=\"joystick-center-line horizontal\"></div>";
    String joyCard = "<div class=\"joystick-motor-card\"><div class=\"motor-header\"><div><div class=\"motor-name\">M" + String(i) + "</div><div class=\"motor-role\">" + String(motorNames[i-1]) + "</div></div><div class=\"joystick-header-speed\" id=\"joy-speed-" + String(i) + "\">0%</div></div><div class=\"joystick-wrapper\"><div class=\"joystick-area " + isVertical + "\" id=\"joystick-" + String(i) + "\">" + centerLine + "<div class=\"joystick-handle\" id=\"handle-" + String(i) + "\"></div></div><div class=\"joystick-info\"><div class=\"joystick-direction\" id=\"joy-direction-" + String(i) + "\">STOPPED</div></div></div></div>";
    server_.sendContent(joyCard);
  }
  server_.sendContent("</div>");
  
  server_.sendContent("</div>");
  // Joint Control 카드
  server_.sendContent("<div class=\"card\"><div class=\"card-title\">Joint Control</div>");
  server_.sendContent("<div class=\"joint-speed-row\"><label>Speed:</label><input type=\"range\" id=\"joint-speed\" min=\"1\" max=\"100\" value=\"50\" style=\"flex:1;\" oninput=\"updateJointSpeed()\"><span class=\"speed-value\" id=\"joint-speed-value\">50%</span></div>");
  const char* jointNames[]    = {"gripper", "wrist", "elbow", "shoulder", "base"};
  const char* jointLabels[]   = {"Gripper", "Wrist", "Elbow", "Shoulder", "Base"};
  const char* jointPosLabels[] = {"Open", "Up",   "Up",   "Up",   "Left"};
  const char* jointNegLabels[] = {"Close", "Down", "Down", "Down", "Right"};
  const char* jointPosActions[] = {"open", "up",   "up",   "up",   "left"};
  const char* jointNegActions[] = {"close","down", "down", "down", "right"};
  for (int i = 0; i < MotorControl::NUM_MOTORS; i++) {
    String jn = String(jointNames[i]);
    String jl = String(jointLabels[i]);
    String posL = String(jointPosLabels[i]);
    String negL = String(jointNegLabels[i]);
    String posA = String(jointPosActions[i]);
    String negA = String(jointNegActions[i]);
    String card = "<div class=\"motor-card\"><div class=\"motor-header\"><div><div class=\"motor-name\">" + jl + "</div></div><div class=\"motor-status\" id=\"joint-status-" + jn + "\">STOPPED</div></div><div class=\"motor-controls\">";
    card += "<button class=\"btn-forward\" onmousedown=\"jointStart('" + jn + "','" + posA + "',event)\" onmouseup=\"jointStop('" + jn + "',event)\" onmouseleave=\"jointStop('" + jn + "',event)\" ontouchstart=\"jointStart('" + jn + "','" + posA + "',event)\" ontouchend=\"jointStop('" + jn + "',event)\" ontouchcancel=\"jointStop('" + jn + "',event)\">" + posL + "</button>";
    card += "<button class=\"btn-reverse\" onmousedown=\"jointStart('" + jn + "','" + negA + "',event)\" onmouseup=\"jointStop('" + jn + "',event)\" onmouseleave=\"jointStop('" + jn + "',event)\" ontouchstart=\"jointStart('" + jn + "','" + negA + "',event)\" ontouchend=\"jointStop('" + jn + "',event)\" ontouchcancel=\"jointStop('" + jn + "',event)\">" + negL + "</button>";
    card += "<button class=\"btn-motor-stop\" onclick=\"jointStopNow('" + jn + "')\">Stop</button>";
    card += "</div></div>";
    server_.sendContent(card);
  }
  server_.sendContent("</div>");

  server_.sendContent("<div class=\"last-update\">Last update: <span id=\"last-update\">-</span></div></div>");
  
  // JavaScript 전송
  server_.sendContent("<script>");
  server_.sendContent("const stateColors = { \"BOOT\": \"state-BOOT\", \"IDLE\": \"state-IDLE\", \"ARMED\": \"state-ARMED\", \"FAULT\": \"state-FAULT\" };");
  server_.sendContent("function showMessage(text, isError) { const msg = document.getElementById(\"message\"); msg.textContent = text; msg.className = \"message \" + (isError ? \"error\" : \"success\"); msg.style.display = \"block\"; setTimeout(() => { msg.style.display = \"none\"; }, 3000); }");
  server_.sendContent("function sendCommand(cmd) { const btn = document.getElementById(\"btn-\" + cmd); btn.disabled = true; fetch(\"/command?cmd=\" + cmd, { method: \"POST\", headers: {\"X-MotionBrain\": \"1\"} }).then(r => r.json()).then(data => { btn.disabled = false; showMessage(data.message || \"Command sent\", !data.success); updateStatus(); }).catch(err => { btn.disabled = false; showMessage(\"Error: \" + err.message, true); }); }");
  server_.sendContent("function sendLight(action) { fetch(\"/light?action=\" + action, { method: \"POST\", headers: {\"X-MotionBrain\": \"1\"} }).then(r => r.json()).then(data => { updateStatus(); }).catch(() => {}); }");
  server_.sendContent("function updateStatus() { fetch(\"/status\").then(r => { if (!r.ok) { throw new Error(\"HTTP \" + r.status + \": \" + r.statusText); } return r.text(); }).then(text => { try { const data = JSON.parse(text); const state = data.state || \"UNKNOWN\"; const badge = document.getElementById(\"state-badge\"); if (badge) { badge.textContent = state; badge.className = \"status-badge \" + (stateColors[state] || \"state-LOADING\"); } const motorEl = document.getElementById(\"motor\"); if (motorEl) motorEl.textContent = data.motorEnabled ? \"YES\" : \"NO\"; const lastUpdate = document.getElementById(\"last-update\"); if (lastUpdate) lastUpdate.textContent = new Date().toLocaleTimeString(); updateButtons(state); if (data.motors) updateMotorStatus(data); } catch (e) { console.error(\"JSON parse error:\", e, \"Response:\", text); } }).catch(err => { console.error(\"Status update error:\", err); }); }");
  server_.sendContent("function updateButtons(state) { const btnArm = document.getElementById(\"btn-arm\"); const btnDisarm = document.getElementById(\"btn-disarm\"); const btnStop = document.getElementById(\"btn-stop\"); btnArm.disabled = (state === \"ARMED\" || state === \"FAULT\" || state === \"BOOT\"); btnDisarm.disabled = (state !== \"ARMED\"); btnStop.disabled = (state === \"IDLE\"); const isArmed = (state === \"ARMED\"); if (btnStop) { btnStop.textContent = (state === \"FAULT\") ? \"RECOVER\" : \"STOP\"; } for (let i = 1; i <= MOTOR_COUNT; i++) { const joystickArea = document.getElementById(\"joystick-\" + i); if (joystickArea) { if (isArmed) { joystickArea.classList.remove(\"disabled\"); } else { joystickArea.classList.add(\"disabled\"); } } } }");
  server_.sendContent("function updateSpeedValue(motorId) { const slider = document.getElementById(\"speed-\" + motorId); const value = document.getElementById(\"speed-value-\" + motorId); value.textContent = slider.value + \"%\"; }");
  server_.sendContent("function validateDefaultSpeed() { const speedInput = document.getElementById(\"default-speed\"); const btnSet = document.getElementById(\"btn-set-speed\"); const validationMsg = document.getElementById(\"speed-validation\"); const value = speedInput.value.trim(); if (value === \"\") { btnSet.disabled = true; validationMsg.textContent = \"Please enter a speed value (1-255)\"; validationMsg.className = \"validation-message\"; speedInput.style.borderColor = \"#f44336\"; return false; } if (value.indexOf(\".\") !== -1 || value.indexOf(\",\") !== -1) { btnSet.disabled = true; validationMsg.textContent = \"Please enter an integer (no decimals)\"; validationMsg.className = \"validation-message\"; speedInput.style.borderColor = \"#f44336\"; return false; } const speed = parseInt(value); if (isNaN(speed)) { btnSet.disabled = true; validationMsg.textContent = \"Please enter a valid number\"; validationMsg.className = \"validation-message\"; speedInput.style.borderColor = \"#f44336\"; return false; } if (speed < 1 || speed > 255) { btnSet.disabled = true; validationMsg.textContent = \"Speed must be between 1 and 255\"; validationMsg.className = \"validation-message\"; speedInput.style.borderColor = \"#f44336\"; return false; } btnSet.disabled = false; validationMsg.textContent = \"Valid speed value\"; validationMsg.className = \"validation-message valid\"; speedInput.style.borderColor = \"#4caf50\"; return true; }");
  server_.sendContent("function setDefaultSpeed() { if (!validateDefaultSpeed()) { return; } const speedInput = document.getElementById(\"default-speed\"); const btnSet = document.getElementById(\"btn-set-speed\"); const speed = parseInt(speedInput.value); btnSet.disabled = true; fetch(\"/motor?action=default&speed=\" + speed, { method: \"POST\", headers: {\"X-MotionBrain\": \"1\"} }).then(r => r.json()).then(data => { btnSet.disabled = false; showMessage(data.message || \"Default speed set\", !data.success); if (data.success) { const validationMsg = document.getElementById(\"speed-validation\"); validationMsg.textContent = \"Speed set successfully\"; validationMsg.className = \"validation-message valid\"; } }).catch(err => { btnSet.disabled = false; showMessage(\"Error: \" + err, true); }); }");
  server_.sendContent("const MOTOR_COUNT = " + String(MotorControl::NUM_MOTORS) + ";");
  server_.sendContent("let activeMotors = {};");
  server_.sendContent("let joystickActive = {};");
  server_.sendContent("let joystickLastUpdate = {};");
  server_.sendContent("const JOYSTICK_UPDATE_INTERVAL = 100;");
  server_.sendContent("let currentMode = 'button';");
  server_.sendContent("let activeJoysticks = {};");
  server_.sendContent("function switchMode(mode) { currentMode = mode; const btnMode = document.getElementById('mode-button'); const joyMode = document.getElementById('mode-joystick'); const btnContainer = document.getElementById('button-container'); const joyContainer = document.getElementById('joystick-container'); if (mode === 'button') { btnMode.classList.add('active'); joyMode.classList.remove('active'); btnContainer.classList.add('active'); joyContainer.classList.remove('active'); stopAllMotors(); for (let motorId in joystickActive) { const handle = document.getElementById('handle-' + motorId); if (handle) { handle.style.transform = 'translate(-50%, -50%)'; handle.classList.remove('active'); } document.getElementById('joy-speed-' + motorId).textContent = '0%'; document.getElementById('joy-direction-' + motorId).textContent = 'STOPPED'; fetch('/motor?action=stop&id=' + motorId, { method: 'POST', headers: {'X-MotionBrain': '1'} }).catch(() => {}); } joystickActive = {}; for (let motorId in activeJoysticks) { const handle = activeJoysticks[motorId].handle; if (handle) { handle.style.transform = 'translate(-50%, -50%)'; handle.classList.remove('active'); } document.getElementById('joy-speed-' + motorId).textContent = '0%'; document.getElementById('joy-direction-' + motorId).textContent = 'STOPPED'; fetch('/motor?action=stop&id=' + motorId, { method: 'POST', headers: {'X-MotionBrain': '1'} }).catch(() => {}); } activeJoysticks = {}; } else { btnMode.classList.remove('active'); joyMode.classList.add('active'); btnContainer.classList.remove('active'); joyContainer.classList.add('active'); stopAllMotors(); } }");
  server_.sendContent("function motorStart(motorId, direction, e) { if (currentMode !== 'button') return; if (e && e.preventDefault) e.preventDefault(); const speed = document.getElementById('speed-' + motorId).value; const btnId = direction === 'forward' ? 'btn-forward-' + motorId : 'btn-reverse-' + motorId; const btn = document.getElementById(btnId); if (btn) btn.classList.add('btn-pressed'); activeMotors[motorId] = direction; const action = direction === 'forward' ? 'forward' : 'reverse'; fetch('/motor?action=' + action + '&id=' + motorId + '&percent=' + speed, { method: 'POST', headers: {'X-MotionBrain': '1'} }).then(r => r.json()).then(data => { if (!data.success) { showMessage(data.message || 'Motor control failed', true); motorStop(motorId); } }).catch(err => { showMessage('Error: ' + err, true); motorStop(motorId); }); }");
  server_.sendContent("function motorStop(motorId, e) { if (currentMode !== 'button') return; if (e && e.preventDefault) e.preventDefault(); if (activeMotors[motorId]) { const direction = activeMotors[motorId]; const btnId = direction === 'forward' ? 'btn-forward-' + motorId : 'btn-reverse-' + motorId; const btn = document.getElementById(btnId); if (btn) btn.classList.remove('btn-pressed'); delete activeMotors[motorId]; fetch('/motor?action=stop&id=' + motorId, { method: 'POST', headers: {'X-MotionBrain': '1'} }).then(r => r.json()).then(data => { updateStatus(); }).catch(err => { console.error('Stop error:', err); }); } }");
  server_.sendContent("function stopAllMotors() { for (let motorId in activeMotors) { motorStop(parseInt(motorId)); } }");
  // motorForward/motorReverse 제거됨 — motorStart/motorStop으로 대체 (CSRF 헤더 포함)
  server_.sendContent("function updateMotorStatus(data) { if (data.motors) { for (let i = 1; i <= MOTOR_COUNT; i++) { const motor = data.motors[\"M\" + i]; if (motor) { const statusEl = document.getElementById(\"motor-status-\" + i); const joySpeedEl = document.getElementById(\"joy-speed-\" + i); const joyDirectionEl = document.getElementById(\"joy-direction-\" + i); if (motor.enabled) { const statusText = motor.direction.toUpperCase() + \" (\" + motor.speed + \")\"; if (statusEl) { statusEl.textContent = statusText; statusEl.className = \"motor-status active\"; } if (joySpeedEl) joySpeedEl.textContent = Math.abs(motor.speed) + '%'; if (joyDirectionEl) joyDirectionEl.textContent = motor.direction.toUpperCase(); } else { if (statusEl) { statusEl.textContent = \"STOPPED\"; statusEl.className = \"motor-status\"; } if (joySpeedEl) joySpeedEl.textContent = '0%'; if (joyDirectionEl) joyDirectionEl.textContent = 'STOPPED'; } } } } }");
  server_.sendContent("function initJoystick(motorId) { const area = document.getElementById('joystick-' + motorId); const handle = document.getElementById('handle-' + motorId); if (!area || !handle) return; const isVertical = motorId >= 1 && motorId <= 4; const isHorizontal = motorId === 5; let centerX = 0; let centerY = 0; let radius = 0; function updateCenter() { const rect = area.getBoundingClientRect(); centerX = rect.left + rect.width / 2; centerY = rect.top + rect.height / 2; radius = rect.width / 2 - 10; } function updateJoystick(clientX, clientY) { if (area.classList.contains('disabled')) return; const dx = clientX - centerX; const dy = clientY - centerY; let x = 0; let y = 0; let speedPercent = 0; let isForward = false; if (isVertical) { const distance = Math.abs(dy); const limitedDistance = Math.min(distance, radius); y = dy < 0 ? -limitedDistance : limitedDistance; speedPercent = Math.round((limitedDistance / radius) * 100); isForward = dy < 0; } else if (isHorizontal) { const distance = Math.abs(dx); const limitedDistance = Math.min(distance, radius); x = dx < 0 ? -limitedDistance : limitedDistance; speedPercent = Math.round((limitedDistance / radius) * 100); isForward = dx < 0; } handle.style.transform = 'translate(calc(-50% + ' + x + 'px), calc(-50% + ' + y + 'px))'; const direction = isForward ? 'FORWARD' : (speedPercent < 5 ? 'STOPPED' : 'REVERSE'); document.getElementById('joy-speed-' + motorId).textContent = speedPercent + '%'; document.getElementById('joy-direction-' + motorId).textContent = direction; if (speedPercent > 5) { const action = isForward ? 'forward' : 'reverse'; const now = Date.now(); if (!joystickLastUpdate[motorId] || now - joystickLastUpdate[motorId] >= JOYSTICK_UPDATE_INTERVAL) { joystickLastUpdate[motorId] = now; fetch('/motor?action=' + action + '&id=' + motorId + '&percent=' + speedPercent, { method: 'POST', headers: {'X-MotionBrain': '1'} }).then(r => r.json()).then(data => { if (!data.success) { console.error('Joystick control failed:', data); } }).catch(err => { console.error('Joystick error:', err); }); } joystickActive[motorId] = { action: action, percent: speedPercent }; } else { if (joystickActive[motorId]) { fetch('/motor?action=stop&id=' + motorId, { method: 'POST', headers: {'X-MotionBrain': '1'} }).catch(err => console.error('Stop error:', err)); delete joystickActive[motorId]; } } } function getTouchPoint(e, storedTouchId, joystickArea) { if (e.touches && storedTouchId !== null) { for (let i = 0; i < e.touches.length; i++) { if (e.touches[i].identifier === storedTouchId) { return { x: e.touches[i].clientX, y: e.touches[i].clientY }; } } return null; } if (e.clientX !== undefined && e.clientY !== undefined) { const rect = joystickArea.getBoundingClientRect(); if (e.clientX >= rect.left && e.clientX <= rect.right && e.clientY >= rect.top && e.clientY <= rect.bottom) { return { x: e.clientX, y: e.clientY }; } } return null; } function findTouchInArea(e, joystickArea) { if (e.touches && e.touches.length > 0) { const rect = joystickArea.getBoundingClientRect(); const usedTouchIds = new Set(); for (let id in activeJoysticks) { if (activeJoysticks[id] && activeJoysticks[id].touchId !== null) { usedTouchIds.add(activeJoysticks[id].touchId); } } for (let i = 0; i < e.touches.length; i++) { const touch = e.touches[i]; if (touch.clientX >= rect.left && touch.clientX <= rect.right && touch.clientY >= rect.top && touch.clientY <= rect.bottom) { if (!usedTouchIds.has(touch.identifier)) { return touch.identifier; } } } } return null; } function startDrag(e) { if (currentMode !== 'joystick' || area.classList.contains('disabled')) return; e.preventDefault(); updateCenter(); let currentTouchId = null; if (e.touches && e.touches.length > 0) { currentTouchId = findTouchInArea(e, area); if (currentTouchId === null) return; } const joyObj = { area: area, handle: handle, updateCenter: updateCenter, updateJoystick: updateJoystick, motorId: motorId, touchId: currentTouchId }; joyObj.getTouchPoint = function(e) { return getTouchPoint(e, joyObj.touchId, joyObj.area); }; activeJoysticks[motorId] = joyObj; handle.classList.add('active'); const point = getTouchPoint(e, currentTouchId, area); if (point) updateJoystick(point.x, point.y); } area.addEventListener('mousedown', startDrag); area.addEventListener('touchstart', startDrag, { passive: false }); }");
  server_.sendContent("function handleGlobalDrag(e) { let shouldPreventDefault = false; const isMouseEvent = e.type === 'mousemove'; let mouseHandled = false; for (let motorId in activeJoysticks) { const joy = activeJoysticks[motorId]; if (joy && joy.area && !joy.area.classList.contains('disabled')) { if (isMouseEvent && joy.touchId !== null) continue; if (isMouseEvent && mouseHandled) continue; joy.updateCenter(); const point = joy.getTouchPoint(e); if (point) { shouldPreventDefault = true; if (isMouseEvent) mouseHandled = true; joy.updateJoystick(point.x, point.y); } } } if (shouldPreventDefault && e.touches && e.touches.length > 0) { e.preventDefault(); } } function handleGlobalEndDrag(e) { const endedTouchIds = new Set(); if (e.changedTouches) { for (let i = 0; i < e.changedTouches.length; i++) { endedTouchIds.add(e.changedTouches[i].identifier); } } let shouldPreventDefault = false; const isMouseEvent = e.type === 'mouseup'; for (let motorId in activeJoysticks) { const joy = activeJoysticks[motorId]; if (joy && joy.area) { let shouldEnd = false; if (isMouseEvent) { if (joy.touchId === null) { shouldEnd = true; } } else if (e.type === 'touchend' || e.type === 'touchcancel') { if (joy.touchId !== null && endedTouchIds.has(joy.touchId)) { shouldEnd = true; shouldPreventDefault = true; } } if (shouldEnd) { const handle = joy.handle; delete activeJoysticks[motorId]; handle.classList.remove('active'); handle.style.transform = 'translate(-50%, -50%)'; document.getElementById('joy-speed-' + joy.motorId).textContent = '0%'; document.getElementById('joy-direction-' + joy.motorId).textContent = 'STOPPED'; if (joystickActive[joy.motorId]) { fetch('/motor?action=stop&id=' + joy.motorId, { method: 'POST', headers: {'X-MotionBrain': '1'} }).catch(err => console.error('Stop error:', err)); delete joystickActive[joy.motorId]; } } } } if (shouldPreventDefault && e.changedTouches && e.changedTouches.length > 0) { e.preventDefault(); } } document.addEventListener('mousemove', handleGlobalDrag); document.addEventListener('touchmove', handleGlobalDrag, { passive: false }); document.addEventListener('mouseup', handleGlobalEndDrag); document.addEventListener('touchend', handleGlobalEndDrag, { passive: false }); document.addEventListener('touchcancel', handleGlobalEndDrag, { passive: false });");
  server_.sendContent("window.addEventListener(\"load\", function() { validateDefaultSpeed(); for (let i = 1; i <= MOTOR_COUNT; i++) { initJoystick(i); } });");
  server_.sendContent("window.addEventListener(\"beforeunload\", function() { stopAllMotors(); for (let motorId in joystickActive) { fetch('/motor?action=stop&id=' + motorId, { method: 'POST', headers: {'X-MotionBrain': '1'} }).catch(() => {}); } for (let motorId in activeJoysticks) { fetch('/motor?action=stop&id=' + motorId, { method: 'POST', headers: {'X-MotionBrain': '1'} }).catch(() => {}); } });");
  server_.sendContent("document.addEventListener(\"keydown\", function(e) { if (currentMode !== 'button') return; const keyMap = { 'KeyQ': { motor: 1, dir: 'forward' }, 'KeyA': { motor: 1, dir: 'reverse' }, 'KeyW': { motor: 2, dir: 'forward' }, 'KeyS': { motor: 2, dir: 'reverse' }, 'KeyE': { motor: 3, dir: 'forward' }, 'KeyD': { motor: 3, dir: 'reverse' }, 'KeyR': { motor: 4, dir: 'forward' }, 'KeyF': { motor: 4, dir: 'reverse' }, 'KeyT': { motor: 5, dir: 'forward' }, 'KeyG': { motor: 5, dir: 'reverse' } }; const mapping = keyMap[e.code]; if (mapping && !activeMotors[mapping.motor]) { e.preventDefault(); motorStart(mapping.motor, mapping.dir); } });");
  server_.sendContent("document.addEventListener(\"keyup\", function(e) { if (currentMode !== 'button') return; const keyMap = { 'KeyQ': 1, 'KeyA': 1, 'KeyW': 2, 'KeyS': 2, 'KeyE': 3, 'KeyD': 3, 'KeyR': 4, 'KeyF': 4, 'KeyT': 5, 'KeyG': 5 }; const motorId = keyMap[e.code]; if (motorId && activeMotors[motorId]) { e.preventDefault(); motorStop(motorId); } });");
  server_.sendContent("let activeJointButtons = {};");
  server_.sendContent("function updateJointSpeed() { const v = document.getElementById('joint-speed').value; document.getElementById('joint-speed-value').textContent = v + '%'; }");
  server_.sendContent("function jointStart(joint, action, e) { if (e && e.preventDefault) e.preventDefault(); const speed = document.getElementById('joint-speed').value; const btn = e ? e.currentTarget : null; if (btn) btn.classList.add('btn-pressed'); activeJointButtons[joint] = {action: action, btn: btn}; fetch('/joint?joint=' + joint + '&action=' + action + '&percent=' + speed, {method: 'POST', headers: {'X-MotionBrain': '1'}}).then(r => r.json()).then(data => { if (!data.success) { showMessage(data.message || 'Joint control failed', true); jointStopNow(joint); } }).catch(err => { showMessage('Error: ' + err, true); jointStopNow(joint); }); }");
  server_.sendContent("function jointStop(joint, e) { if (e && e.preventDefault) e.preventDefault(); if (activeJointButtons[joint]) { const entry = activeJointButtons[joint]; if (entry.btn) entry.btn.classList.remove('btn-pressed'); delete activeJointButtons[joint]; fetch('/joint?joint=' + joint + '&action=stop', {method: 'POST', headers: {'X-MotionBrain': '1'}}).catch(() => {}); } }");
  server_.sendContent("function jointStopNow(joint) { const entry = activeJointButtons[joint]; if (entry && entry.btn) entry.btn.classList.remove('btn-pressed'); delete activeJointButtons[joint]; fetch('/joint?joint=' + joint + '&action=stop', {method: 'POST', headers: {'X-MotionBrain': '1'}}).catch(() => {}); }");
  server_.sendContent("setInterval(updateStatus, 1000); updateStatus();");
  server_.sendContent("</script></body></html>");
  
  DebugLog::info("Web Server: HTML sent successfully (streaming mode)");
}

/**
 * GET /status 처리
 * JSON 형식으로 현재 상태 반환
 * 주의: CSRF 헤더 불필요 — GET은 읽기 전용이므로 상태 변경 없음
 */
void MotionBrainWebServer::handleStatus() {
  DebugLog::debug("Web Server: GET /status requested");
  
  if (systemState_ == nullptr) {
    sendErrorJson(500, "SystemStateManager not initialized");
    return;
  }
  
  // 현재 상태 조회
  const char* stateString = systemState_->getStateString();
  bool motorEnabled = false;
  
  if (motorControl_ != nullptr) {
    motorEnabled = motorControl_->isEnabled();
  }
  
  // JSON 응답 생성
  String json;
  json.reserve(1200);  // pre-allocate to avoid realloc
  json = "{\"schemaVersion\":\"";
  json += MESSAGE_SCHEMA_VERSION;
  json += "\",\"messageType\":\"status\",\"uptimeMs\":";
  json += String(millis());
  json += ",\"state\":\"";
  json += stateString;
  json += "\",";
  json += "\"motorEnabled\":";
  json += motorEnabled ? "true" : "false";
  
  // 모터 상태 추가
  if (motorControl_ != nullptr) {
    json += ",\"motors\":{";
    const char* motorNames[] = {"Gripper", "Wrist", "Elbow", "Shoulder", "Base"};
    for (uint8_t i = 1; i <= MotorControl::NUM_MOTORS; i++) {
      if (i > 1) json += ",";
      json += "\"M";
      json += String(i);
      json += "\":{";
      json += "\"name\":\"";
      json += motorNames[i-1];
      json += "\",";
      int16_t speed = motorControl_->getSpeed(i);
      bool enabled = motorControl_->isEnabled(i);
      json += "\"speed\":";
      json += String(speed);
      json += ",";
      json += "\"enabled\":";
      json += enabled ? "true" : "false";
      json += ",";
      json += "\"direction\":\"";
      if (speed > 0) {
        json += "forward";
      } else if (speed < 0) {
        json += "reverse";
      } else {
        json += "stopped";
      }
      json += "\"";
      json += "}";
    }
    json += "}";
  }
  
  if (searchLight_ != nullptr) {
    json += ",\"light\":";
    json += searchLight_->isOn() ? "true" : "false";
  }

  const SensorSnapshot& snapshot = stm32Bridge.getSnapshot();
  json += ",\"sensor\":{";
  json += "\"connected\":";
  json += stm32Bridge.isConnected() ? "true" : "false";
  json += ",\"simulated\":";
  json += stm32Bridge.isSimulationEnabled() ? "true" : "false";
  json += ",\"simulationMode\":\"";
  json += stm32Bridge.getSimulationModeString();
  json += "\"";
  json += ",\"lastUpdateMs\":";
  json += String(stm32Bridge.getLastPacketAgeMs());
  json += ",\"packetsReceived\":";
  json += String(stm32Bridge.getPacketsReceived());
  json += ",\"parseErrors\":";
  json += String(stm32Bridge.getParseErrors());
  json += ",\"imuOk\":";
  json += snapshot.imuOk ? "true" : "false";
  json += ",\"rangeOk\":";
  json += snapshot.rangeOk ? "true" : "false";
  json += ",\"sourceTimestampMs\":";
  json += String(snapshot.sourceTimestampMs);
  json += ",\"gyroX\":";
  json += String(snapshot.gyroX, 2);
  json += ",\"gyroY\":";
  json += String(snapshot.gyroY, 2);
  json += ",\"gyroZ\":";
  json += String(snapshot.gyroZ, 2);
  json += ",\"roll\":";
  json += String(snapshot.roll, 2);
  json += ",\"pitch\":";
  json += String(snapshot.pitch, 2);
  json += ",\"distCm\":";
  json += String(snapshot.distanceCm, 1);
  json += ",\"vibe\":";
  json += String(snapshot.vibe, 2);
  json += ",\"blocked\":";
  json += safetyMonitor.isMotionBlocked() ? "true" : "false";
  json += ",\"blockReason\":\"";
  json += safetyMonitor.getBlockReasonString();
  json += "\"";
  json += ",\"faultLatched\":";
  json += safetyMonitor.hasLatchedFault() ? "true" : "false";
  json += ",\"faultReason\":\"";
  json += safetyMonitor.getLatchedFaultReasonString();
  json += "\"}";

  json += ",\"baseAngle\":{";
  json += "\"active\":";
  json += angleController.isActive() ? "true" : "false";
  json += ",\"direction\":\"";
  json += angleController.getDirectionString();
  json += "\"";
  json += ",\"targetDeg\":";
  json += String(angleController.getTargetDegrees(), 1);
  json += ",\"currentDeg\":";
  json += String(angleController.getAccumulatedDegrees(), 1);
  json += ",\"remainingDeg\":";
  json += String(angleController.getRemainingDegrees(), 1);
  json += ",\"percent\":";
  json += String(angleController.getPercent());
  json += ",\"elapsedMs\":";
  json += String(angleController.getElapsedMs());
  json += ",\"timeoutMs\":";
  json += String(angleController.getTimeoutMs());
  json += ",\"processedSamples\":";
  json += String(angleController.getProcessedSamples());
  json += ",\"lastRateDps\":";
  json += String(angleController.getLastRateDegreesPerSecond(), 2);
  json += ",\"lastStopReason\":\"";
  json += angleController.getLastStopReasonString();
  json += "\"";
  json += ",\"lastTransitionMs\":";
  json += String(angleController.getLastTransitionMs());
  json += "}";

  json += ",\"teleop\":{";
  json += "\"connected\":";
  json += teleopAdapter.isConnected() ? "true" : "false";
  json += ",\"deadman\":";
  json += teleopAdapter.isDeadmanHeld() ? "true" : "false";
  json += ",\"controlActive\":";
  json += teleopAdapter.isControlActive() ? "true" : "false";
  json += ",\"lastFrameAgeMs\":";
  json += String(teleopAdapter.getLastFrameAgeMs());
  json += ",\"packetsReceived\":";
  json += String(teleopAdapter.getPacketsReceived());
  json += ",\"parseErrors\":";
  json += String(teleopAdapter.getParseErrors());
  json += ",\"session\":";
  json += String(teleopAdapter.getLastSession());
  json += ",\"seq\":";
  json += String(teleopAdapter.getLastSequence());
  json += ",\"reach\":";
  json += String(teleopAdapter.getLastReach(), 2);
  json += ",\"lift\":";
  json += String(teleopAdapter.getLastLift(), 2);
  json += ",\"twist\":";
  json += String(teleopAdapter.getLastTwist(), 2);
  json += ",\"ledToggleSeq\":";
  json += String(teleopAdapter.getLastLedToggleSeq());
  json += ",\"lastStopReason\":\"";
  json += teleopAdapter.getLastStopReasonString();
  json += "\"}";

  json += "}";

  DebugLog::debug("Web Server: Status response - state: %s, motor: %s",
                  stateString, motorEnabled ? "enabled" : "disabled");

  server_.send(200, "application/json", json);
}

void MotionBrainWebServer::handleEvents() {
  DebugLog::debug("Web Server: GET /events requested");

  uint8_t limit = eventLog.size();
  String limitStr = server_.arg("limit");
  if (limitStr.length() > 0) {
    int parsedLimit = limitStr.toInt();
    if (parsedLimit > 0 && parsedLimit < limit) {
      limit = static_cast<uint8_t>(parsedLimit);
    }
  }

  String json = "{\"schemaVersion\":\"";
  json += MESSAGE_SCHEMA_VERSION;
  json += "\",\"messageType\":\"event_list\",";
  appendStateSummaryJson(json);
  json += ",\"count\":";
  json += String(limit);
  json += ",\"events\":[";

  uint8_t total = eventLog.size();
  uint8_t startIndex = total > limit ? total - limit : 0;
  bool first = true;
  for (uint8_t i = startIndex; i < total; ++i) {
    MotionEvent event;
    if (!eventLog.getOldestFirst(i, event)) {
      continue;
    }
    if (!first) {
      json += ",";
    }
    first = false;
    json += "{\"id\":";
    json += String(event.id);
    json += ",\"tsMs\":";
    json += String(event.tsMs);
    json += ",\"severity\":\"";
    json += EventLog::severityToString(event.severity);
    json += "\",\"category\":\"";
    json += jsonEscape(event.category);
    json += "\",\"code\":\"";
    json += jsonEscape(event.code);
    json += "\",\"detail\":\"";
    json += jsonEscape(event.detail);
    json += "\"}";
  }

  json += "]}";
  server_.send(200, "application/json", json);
}

/**
 * POST /command 처리
 * 명령 실행 (arm, disarm, stop 등)
 * 
 * 파라미터:
 *   - cmd: 명령어 이름 (arm, disarm, stop)
 */
void MotionBrainWebServer::handleCommand() {
  DebugLog::debug("Web Server: POST /command requested");
  
  if (systemState_ == nullptr || motorControl_ == nullptr) {
    sendErrorJson(500, "System not initialized");
    return;
  }

  if (server_.header("X-MotionBrain") != "1") {
    sendErrorJson(403, "Forbidden: missing X-MotionBrain header");
    return;
  }

  // POST 요청에서 'cmd' 파라미터 읽기
  String cmd = server_.arg("cmd");
  
  if (cmd.length() == 0) {
    DebugLog::warn("Web Server: Command parameter missing");
    sendErrorJson(400, "Missing 'cmd' parameter");
    return;
  }
  
  DebugLog::info("Web Server: Command received: %s", cmd.c_str());

  Command command;
  command.source = CommandSource::WEB_INPUT;
  if (cmd == "arm") {
    command.type = CommandType::ARM;
  }
  else if (cmd == "disarm") {
    command.type = CommandType::DISARM;
  }
  else if (cmd == "stop") {
    command.type = CommandType::STOP;
  }
  else {
    DebugLog::warn("Web Server: Unknown command: %s", cmd.c_str());
    sendErrorJson(400, "Unknown command", cmd);
    return;
  }

  CommandResult result;
  submitCommand(command, result);
  sendCommandResult(result, String("\"state\":\"") + systemState_->getStateString() + "\"");
}

/**
 * POST /motor 처리
 * 모터 제어 (forward, reverse, stop, default)
 */
void MotionBrainWebServer::handleMotor() {
  DebugLog::debug("Web Server: POST /motor requested");
  
  if (motorControl_ == nullptr) {
    sendErrorJson(500, "MotorControl not initialized");
    return;
  }

  if (server_.header("X-MotionBrain") != "1") {
    sendErrorJson(403, "Forbidden: missing X-MotionBrain header");
    return;
  }

  // 쿼리 파라미터에서 action 추출
  String action = server_.arg("action");
  String motorIdStr = server_.arg("id");
  String percentStr = server_.arg("percent");
  String speedStr = server_.arg("speed");
  
  Command command;
  command.source = CommandSource::WEB_INPUT;
  
  if (action == "forward") {
    if (motorIdStr.length() == 0) {
      sendErrorJson(400, "Motor ID required");
      return;
    }

    int motorIdInt = motorIdStr.toInt();
    if (motorIdInt < 1 || motorIdInt > MotorControl::NUM_MOTORS) {
      sendErrorJson(400, "Invalid motor ID (1-5)", motorIdStr);
      return;
    }
    uint8_t motorId = (uint8_t)motorIdInt;

    uint8_t percent = 100;
    if (percentStr.length() > 0) {
      int pv = percentStr.toInt();
      if (pv < 0 || pv > 100 || (pv == 0 && percentStr != "0")) {
        sendErrorJson(400, "Invalid percent value (0-100)", percentStr);
        return;
      }
      if (pv == 0) {
        sendErrorJson(400, "Use 'stop' action for 0% speed");
        return;
      }
      percent = (uint8_t)pv;
    }

    command.type = CommandType::MOTOR_RUN;
    command.motorId = motorId;
    command.forward = true;
    command.percent = percent;
  }
  else if (action == "reverse") {
    if (motorIdStr.length() == 0) {
      sendErrorJson(400, "Motor ID required");
      return;
    }

    int motorIdInt = motorIdStr.toInt();
    if (motorIdInt < 1 || motorIdInt > MotorControl::NUM_MOTORS) {
      sendErrorJson(400, "Invalid motor ID (1-5)", motorIdStr);
      return;
    }
    uint8_t motorId = (uint8_t)motorIdInt;

    uint8_t percent = 100;
    if (percentStr.length() > 0) {
      int pv = percentStr.toInt();
      if (pv < 0 || pv > 100 || (pv == 0 && percentStr != "0")) {
        sendErrorJson(400, "Invalid percent value (0-100)", percentStr);
        return;
      }
      if (pv == 0) {
        sendErrorJson(400, "Use 'stop' action for 0% speed");
        return;
      }
      percent = (uint8_t)pv;
    }

    command.type = CommandType::MOTOR_RUN;
    command.motorId = motorId;
    command.forward = false;
    command.percent = percent;
  }
  else if (action == "stop") {
    if (motorIdStr.length() == 0) {
      sendErrorJson(400, "Motor ID required");
      return;
    }

    int motorIdInt = motorIdStr.toInt();
    if (motorIdInt < 1 || motorIdInt > MotorControl::NUM_MOTORS) {
      sendErrorJson(400, "Invalid motor ID (1-5)", motorIdStr);
      return;
    }
    uint8_t motorId = (uint8_t)motorIdInt;

    command.type = CommandType::MOTOR_STOP;
    command.motorId = motorId;
  }
  else if (action == "default") {
    if (speedStr.length() == 0) {
      sendErrorJson(400, "Speed value required");
      return;
    }
    
    int speedInt = speedStr.toInt();
    
    if (speedInt < 1) {
      sendErrorJson(400, "Default speed must be between 1 and 255 (0 means no movement)");
      return;
    }
    if (speedInt > 255) {
      sendErrorJson(400, "Default speed must be between 1 and 255");
      return;
    }
    
    uint8_t speed = (uint8_t)speedInt;
    
    command.type = CommandType::MOTOR_SET_DEFAULT_SPEED;
    command.speed = speed;
  }
  else {
    DebugLog::warn("Web Server: Unknown motor action: %s", action.c_str());
    sendErrorJson(400, "Unknown action", action);
    return;
  }

  CommandResult result;
  submitCommand(command, result);
  sendCommandResult(result);
}

/**
 * POST /joint 처리
 * 관절 제어 (gripper/wrist/elbow/shoulder/base)
 */
void MotionBrainWebServer::handleJoint() {
  DebugLog::debug("Web Server: POST /joint requested");

  if (robotArm_ == nullptr) {
    sendErrorJson(500, "RobotArm not initialized");
    return;
  }

  if (server_.header("X-MotionBrain") != "1") {
    sendErrorJson(403, "Forbidden: missing X-MotionBrain header");
    return;
  }

  String joint = server_.arg("joint");
  String action = server_.arg("action");

  if (joint.length() == 0) {
    sendErrorJson(400, "Missing 'joint' parameter");
    return;
  }
  if (action.length() == 0) {
    sendErrorJson(400, "Missing 'action' parameter");
    return;
  }

  String percentStr = server_.arg("percent");
  uint8_t percent = 50;
  if (percentStr.length() > 0) {
    int pVal = percentStr.toInt();
    if (pVal < 0 || pVal > 100 || (pVal == 0 && percentStr != "0")) {
      sendErrorJson(400, "Invalid 'percent' value (0-100)", percentStr);
      return;
    }
    if (pVal == 0 && action != "stop") {
      sendErrorJson(400, "Use 'stop' action for 0% speed");
      return;
    }
    percent = (uint8_t)pVal;
  }

  Command command;
  command.source = CommandSource::WEB_INPUT;

  if (joint == "gripper") {
    command.joint = MotionJoint::GRIPPER;
    if      (action == "open")  { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::OPEN; }
    else if (action == "close") { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::CLOSE; }
    else if (action == "stop")  { command.type = CommandType::JOINT_STOP; }
    else { sendErrorJson(400, "Unknown action", action); return; }
  }
  else if (joint == "wrist") {
    command.joint = MotionJoint::WRIST;
    if      (action == "up")   { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::UP; }
    else if (action == "down") { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::DOWN; }
    else if (action == "stop") { command.type = CommandType::JOINT_STOP; }
    else { sendErrorJson(400, "Unknown action", action); return; }
  }
  else if (joint == "elbow") {
    command.joint = MotionJoint::ELBOW;
    if      (action == "up")   { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::UP; }
    else if (action == "down") { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::DOWN; }
    else if (action == "stop") { command.type = CommandType::JOINT_STOP; }
    else { sendErrorJson(400, "Unknown action", action); return; }
  }
  else if (joint == "shoulder") {
    command.joint = MotionJoint::SHOULDER;
    if      (action == "up")   { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::UP; }
    else if (action == "down") { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::DOWN; }
    else if (action == "stop") { command.type = CommandType::JOINT_STOP; }
    else { sendErrorJson(400, "Unknown action", action); return; }
  }
  else if (joint == "base") {
    command.joint = MotionJoint::BASE;
    if      (action == "left")  { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::LEFT; }
    else if (action == "right") { command.type = CommandType::JOINT_RUN;  command.direction = MotionDirection::RIGHT; }
    else if (action == "stop")  { command.type = CommandType::JOINT_STOP; }
    else { sendErrorJson(400, "Unknown action", action); return; }
  }
  else if (joint == "all" && action == "stop") {
    command.type = CommandType::JOINT_STOP_ALL;
  }
  else {
    DebugLog::warn("Web Server: Unknown joint: %s", joint.c_str());
    sendErrorJson(400, "Unknown joint", joint);
    return;
  }

  command.percent = percent;

  CommandResult result;
  submitCommand(command, result);
  sendCommandResult(result);
}

void MotionBrainWebServer::handleBase() {
  DebugLog::debug("Web Server: POST /base requested");

  if (server_.header("X-MotionBrain") != "1") {
    sendErrorJson(403, "Forbidden: missing X-MotionBrain header");
    return;
  }

  String action = server_.arg("action");
  if (action.length() == 0) {
    sendErrorJson(400, "Missing 'action' parameter");
    return;
  }

  Command command;
  command.source = CommandSource::WEB_INPUT;

  if (action == "stop") {
    command.type = CommandType::JOINT_STOP;
    command.joint = MotionJoint::BASE;
  }
  else if (action == "angle") {
    String directionStr = server_.arg("direction");
    String degreesStr = server_.arg("degrees");
    String percentStr = server_.arg("percent");

    if (directionStr.length() == 0 || degreesStr.length() == 0) {
      sendErrorJson(400, "Missing direction or degrees");
      return;
    }

    float degrees = degreesStr.toFloat();
    if (degrees <= 0.0f && degreesStr != "0" && degreesStr != "0.0") {
      sendErrorJson(400, "Invalid degrees value", degreesStr);
      return;
    }

    int percent = AngleController::DEFAULT_SPEED;
    if (percentStr.length() > 0) {
      percent = percentStr.toInt();
      if ((percent == 0 && percentStr != "0") || percent < 1 || percent > 100) {
        sendErrorJson(400, "Percent must be 1-100", percentStr);
        return;
      }
    }

    MotionDirection direction;
    if (directionStr == "left") {
      direction = MotionDirection::LEFT;
    } else if (directionStr == "right") {
      direction = MotionDirection::RIGHT;
    } else {
      sendErrorJson(400, "Direction must be left or right", directionStr);
      return;
    }

    if (degrees < AngleController::MIN_TARGET_DEGREES ||
        degrees > AngleController::MAX_TARGET_DEGREES) {
      sendErrorJson(400, "Degrees must be between 3 and 180", degreesStr);
      return;
    }

    command.type = CommandType::BASE_ANGLE_RUN;
    command.joint = MotionJoint::BASE;
    command.direction = direction;
    command.targetDegrees = degrees;
    command.percent = static_cast<uint8_t>(percent);
  }
  else {
    sendErrorJson(400, "Unknown action (angle/stop)", action);
    return;
  }

  CommandResult result;
  submitCommand(command, result);
  sendCommandResult(
    result,
    String("\"baseAngleActive\":") + (angleController.isActive() ? "true" : "false") +
      ",\"baseAngleReason\":\"" + angleController.getLastStopReasonString() + "\"");
}

/**
 * POST /sequence 처리 (Phase 2-B)
 * 시퀀스 제어: action=add|run|stop|clear
 * add 추가 파라미터:
 * - duration 기반: joint, direction, speed, duration
 * - base angle 기반: joint=base, direction, speed, degrees
 */
void MotionBrainWebServer::handleSequence() {
  DebugLog::debug("Web Server: POST /sequence requested");

  if (motionSequence_ == nullptr) {
    sendErrorJson(500, "MotionSequence not initialized");
    return;
  }

  if (server_.header("X-MotionBrain") != "1") {
    sendErrorJson(403, "Forbidden: missing X-MotionBrain header");
    return;
  }

  String action = server_.arg("action");
  if (action.length() == 0) {
    sendErrorJson(400, "Missing 'action' parameter");
    return;
  }

  Command command;
  command.source = CommandSource::WEB_INPUT;

  if (action == "run") {
    command.type = CommandType::SEQUENCE_RUN;
  }
  else if (action == "stop") {
    command.type = CommandType::SEQUENCE_STOP;
  }
  else if (action == "clear") {
    command.type = CommandType::SEQUENCE_CLEAR;
  }
  else if (action == "add") {
    String jointStr = server_.arg("joint");
    String dirStr   = server_.arg("direction");
    int    speed    = server_.arg("speed").toInt();
    String durationStr = server_.arg("duration");
    String degreesStr = server_.arg("degrees");

    if (jointStr.length() == 0 || dirStr.length() == 0) {
      sendErrorJson(400, "Missing joint or direction");
      return;
    }

    MotionJoint     joint;
    MotionDirection direction;

    if (!MotionSequence::parseJoint(jointStr.c_str(), joint)) {
      sendErrorJson(400, "Unknown joint", jointStr);
      return;
    }
    if (!MotionSequence::parseDirection(joint, dirStr.c_str(), direction)) {
      sendErrorJson(400, "Invalid direction for joint", dirStr);
      return;
    }
    if (speed < 1 || speed > 100) {
      sendErrorJson(400, "Speed must be 1-100", String(speed));
      return;
    }

    command.type = CommandType::SEQUENCE_ADD;
    command.joint = joint;
    command.direction = direction;
    command.percent = (uint8_t)speed;

    if (joint == MotionJoint::BASE && degreesStr.length() > 0) {
      if (durationStr.length() > 0) {
        sendErrorJson(400, "Use either duration or degrees", durationStr + "," + degreesStr);
        return;
      }

      float degrees = degreesStr.toFloat();
      if ((degrees <= 0.0f && degreesStr != "0" && degreesStr != "0.0") ||
          degrees < AngleController::MIN_TARGET_DEGREES ||
          degrees > AngleController::MAX_TARGET_DEGREES) {
        sendErrorJson(400, "Degrees must be between 3 and 180", degreesStr);
        return;
      }
      command.targetDegrees = degrees;
      command.durationMs = 0;
    } else {
      long duration = durationStr.toInt();
      if (duration <= 0) {
        sendErrorJson(400, "Duration must be > 0", durationStr);
        return;
      }
      command.durationMs = (uint32_t)duration;
      command.targetDegrees = 0.0f;
    }
  }
  else {
    sendErrorJson(400, "Unknown action", action);
    return;
  }

  CommandResult result;
  submitCommand(command, result);
  sendCommandResult(result,
                    String("\"state\":\"") + MotionSequence::stateToString(motionSequence_->getState()) +
                    "\",\"count\":" + String(motionSequence_->getTotalCount()));
}

/**
 * GET /sequence 처리 (Phase 2-B)
 * 시퀀스 상태 JSON 반환
 */
void MotionBrainWebServer::handleSequenceStatus() {
  DebugLog::debug("Web Server: GET /sequence requested");

  if (motionSequence_ == nullptr) {
    sendErrorJson(500, "MotionSequence not initialized");
    return;
  }

  String json = "{\"schemaVersion\":\"";
  json += MESSAGE_SCHEMA_VERSION;
  json += "\",\"messageType\":\"sequence_status\",";
  appendStateSummaryJson(json);
  json += ",\"sequence\":{";
  json += "\"state\":\"";
  json += MotionSequence::stateToString(motionSequence_->getState());
  json += "\",\"currentStep\":";
  json += motionSequence_->getCurrentIndex() + 1;
  json += ",\"totalCount\":";
  json += motionSequence_->getTotalCount();
  json += ",\"remainingMs\":";
  json += motionSequence_->getRemainingMs();
  json += ",\"full\":";
  json += motionSequence_->isFull() ? "true" : "false";
  json += "}";
  json += "}";

  server_.send(200, "application/json", json);
}

/**
 * 공통 favicon 응답
 */
void MotionBrainWebServer::handleFavicon() {
  String svg = "<svg xmlns='http://www.w3.org/2000/svg' width='16' height='16' viewBox='0 0 16 16'>";
  svg += "<rect width='16' height='16' fill='#667eea'/>";
  svg += "<circle cx='5' cy='5' r='2' fill='white'/>";
  svg += "<circle cx='11' cy='5' r='2' fill='white'/>";
  svg += "<rect x='4' y='8' width='8' height='4' rx='1' fill='white'/>";
  svg += "</svg>";
  server_.send(200, "image/svg+xml", svg);
}

/**
 * 공통 Apple touch icon 응답
 */
void MotionBrainWebServer::handleAppleTouchIcon() {
  String svg = "<svg xmlns='http://www.w3.org/2000/svg' width='180' height='180' viewBox='0 0 180 180'>";
  svg += "<rect width='180' height='180' rx='40' fill='#667eea'/>";
  svg += "<circle cx='60' cy='60' r='20' fill='white'/>";
  svg += "<circle cx='120' cy='60' r='20' fill='white'/>";
  svg += "<rect x='50' y='100' width='80' height='50' rx='10' fill='white'/>";
  svg += "</svg>";
  server_.send(200, "image/svg+xml", svg);
}

/**
 * 404 Not Found 처리
 * 존재하지 않는 경로 접근 시
 * favicon.ico 같은 브라우저 자동 요청은 실제 파일 제공
 */
void MotionBrainWebServer::handleNotFound() {
  String uri = server_.uri();
  
  // favicon.ico - 간단한 SVG 아이콘 제공
  if (uri == "/favicon.ico") {
    handleFavicon();
    return;
  }
  
  // robots.txt - 검색 엔진 크롤러 제어
  if (uri == "/robots.txt") {
    server_.send(200, "text/plain", "User-agent: *\nDisallow: /\n");
    return;
  }
  
  // apple-touch-icon.png - iOS 홈 화면 아이콘 (SVG로 제공)
  if (uri == "/apple-touch-icon.png" ||
      uri == "/apple-touch-icon-precomposed.png" ||
      uri == "/apple-touch-icon-120x120.png" ||
      uri == "/apple-touch-icon-120x120-precomposed.png") {
    handleAppleTouchIcon();
    return;
  }
  
  // 그 외의 404는 로그 남기기
  DebugLog::debug("Web Server: 404 Not Found - %s", uri.c_str());
  server_.send(404, "text/plain", "404: Not Found");
}

/**
 * POST /light 처리
 * 서치라이트 on/off/toggle
 */
void MotionBrainWebServer::handleLight() {
  if (server_.header("X-MotionBrain") != "1") {
    sendErrorJson(403, "Forbidden: missing X-MotionBrain header");
    return;
  }

  if (searchLight_ == nullptr) {
    sendErrorJson(500, "SearchLight not initialized");
    return;
  }

  String action = server_.arg("action");
  Command command;
  command.source = CommandSource::WEB_INPUT;

  if (action == "on") {
    command.type = CommandType::LIGHT_ON;
  } else if (action == "off") {
    command.type = CommandType::LIGHT_OFF;
  } else if (action == "toggle") {
    command.type = CommandType::LIGHT_TOGGLE;
  } else {
    sendErrorJson(400, "Unknown action (on/off/toggle)", action);
    return;
  }

  CommandResult result;
  submitCommand(command, result);
  sendCommandResult(result, String("\"light\":") + (searchLight_->isOn() ? "true" : "false"));
}
