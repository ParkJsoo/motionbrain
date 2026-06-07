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
constexpr uint32_t MANUAL_COMMAND_LEASE_MS = 750;

void sendNoStoreHeaders(::WebServer& server) {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
}

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
  , commandToken_{0}
{
  // 생성자에서는 초기화만 수행
  // 실제 서버 시작은 init()에서 수행
  clearAllManualLeases();
}

/**
 * 웹 서버 초기화
 */
bool MotionBrainWebServer::init(SystemStateManager* systemState, MotorControl* motorControl,
                                RobotArm* robotArm, MotionSequence* motionSequence,
                                SearchLight* searchLight, CommandBus* commandBus,
                                Dispatcher* dispatcher, uint16_t port, const char* commandToken) {
  systemState_    = systemState;
  motorControl_   = motorControl;
  robotArm_       = robotArm;
  motionSequence_ = motionSequence;
  searchLight_    = searchLight;
  commandBus_     = commandBus;
  dispatcher_     = dispatcher;
  port_           = port;
  if (commandToken != nullptr) {
    strlcpy(commandToken_, commandToken, sizeof(commandToken_));
  } else {
    commandToken_[0] = '\0';
  }

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
  const char* authHeaders[] = {"X-MotionBrain", "X-MotionBrain-Token"};
  server_.collectHeaders(authHeaders, 2);

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

bool MotionBrainWebServer::requireCommandAuth() {
  if (server_.header("X-MotionBrain") != "1") {
    sendErrorJson(403, "Forbidden: missing X-MotionBrain header");
    return false;
  }

  if (commandToken_[0] == '\0') {
    sendErrorJson(403, "Forbidden: command token is not provisioned");
    return false;
  }

  if (server_.header("X-MotionBrain-Token") != commandToken_) {
    sendErrorJson(403, "Forbidden: invalid X-MotionBrain-Token");
    return false;
  }

  return true;
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

  expireManualLeases();

  // ESP32 WebServer의 handleClient() 호출
  // 이 메서드는 수신된 HTTP 요청을 처리합니다
  server_.handleClient();

  expireManualLeases();
}

/**
 * 웹 서버 활성화 여부 확인
 */
bool MotionBrainWebServer::isActive() const {
  return active_;
}

void MotionBrainWebServer::expireManualLeases() {
  if (motorControl_ == nullptr) {
    return;
  }

  const uint32_t now = millis();
  for (uint8_t i = 0; i < MANUAL_LEASE_MOTOR_COUNT; i++) {
    if (!manualLeaseActive_[i]) {
      continue;
    }
    if (static_cast<int32_t>(now - manualLeaseUntilMs_[i]) < 0) {
      continue;
    }

    const uint8_t motorId = i + 1;
    manualLeaseActive_[i] = false;
    manualLeaseUntilMs_[i] = 0;
    motorControl_->hardStop(motorId);
    DebugLog::warn("Web Server: manual lease expired for M%d", motorId);
    eventLog.push("web", "MANUAL_LEASE_EXPIRED", EventSeverity::WARN, "manual command refresh missed");
  }
}

void MotionBrainWebServer::extendManualLease(uint8_t motorId) {
  if (motorId < 1 || motorId > MANUAL_LEASE_MOTOR_COUNT) {
    return;
  }
  const uint8_t index = motorId - 1;
  manualLeaseActive_[index] = true;
  manualLeaseUntilMs_[index] = millis() + MANUAL_COMMAND_LEASE_MS;
}

void MotionBrainWebServer::clearManualLease(uint8_t motorId) {
  if (motorId < 1 || motorId > MANUAL_LEASE_MOTOR_COUNT) {
    return;
  }
  const uint8_t index = motorId - 1;
  manualLeaseActive_[index] = false;
  manualLeaseUntilMs_[index] = 0;
}

void MotionBrainWebServer::clearAllManualLeases() {
  for (uint8_t i = 0; i < MANUAL_LEASE_MOTOR_COUNT; i++) {
    manualLeaseActive_[i] = false;
    manualLeaseUntilMs_[i] = 0;
  }
}

uint8_t MotionBrainWebServer::motorIdForJoint(MotionJoint joint) const {
  switch (joint) {
    case MotionJoint::GRIPPER:  return MotorControl::MOTOR_1;
    case MotionJoint::WRIST:    return MotorControl::MOTOR_2;
    case MotionJoint::ELBOW:    return MotorControl::MOTOR_3;
    case MotionJoint::SHOULDER: return MotorControl::MOTOR_4;
    case MotionJoint::BASE:     return MotorControl::MOTOR_5;
    default:                    return 0;
  }
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
  sendNoStoreHeaders(server_);
  server_.setContentLength(CONTENT_LENGTH_UNKNOWN);  // 청크 전송 모드
  server_.send(200, "text/html", "");
  
  // 헤더 부분 즉시 전송
  server_.sendContent("<!DOCTYPE html><html><head>");
  server_.sendContent("<title>MotionBrain Control Console</title>");
  server_.sendContent("<meta charset=\"UTF-8\">");
  server_.sendContent("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
  server_.sendContent("<link rel=\"icon\" type=\"image/svg+xml\" href=\"/favicon.ico\">");
  server_.sendContent("<link rel=\"apple-touch-icon\" href=\"/apple-touch-icon.png\">");
  server_.sendContent("<style>");
  
  // 스타일 부분 전송 (생성과 동시에 전송)
  server_.sendContent("* { box-sizing: border-box; }");
  server_.sendContent(":root { --bg: #080b10; --panel: #111821; --panel-2: #151e29; --panel-3: #0e141c; --line: #263241; --line-soft: #1b2532; --text: #e6edf3; --muted: #8fa1b4; --faint: #5e7084; --cyan: #38bdf8; --green: #22c55e; --amber: #f59e0b; --red: #ef4444; }");
  server_.sendContent("body { font-family: Inter, -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif; margin: 0; padding: 18px; background: var(--bg); color: var(--text); min-height: 100vh; }");
  server_.sendContent(".container { position: relative; max-width: 1280px; margin: 0 auto; display: grid; grid-template-columns: minmax(250px, 0.85fr) minmax(320px, 1.15fr) minmax(220px, 0.75fr); gap: 14px; align-items: start; }");
  server_.sendContent(".app-header { grid-column: 1 / -1; display: flex; justify-content: space-between; align-items: flex-end; gap: 16px; padding: 10px 2px 8px; border-bottom: 1px solid var(--line-soft); }");
  server_.sendContent(".brand-block { min-width: 0; }");
  server_.sendContent(".brand-kicker { color: var(--cyan); font-size: 11px; font-weight: 800; text-transform: uppercase; letter-spacing: 1.8px; margin-bottom: 6px; }");
  server_.sendContent(".brand-title { margin: 0; color: var(--text); font-size: 34px; line-height: 1.05; font-weight: 800; letter-spacing: 0; }");
  server_.sendContent(".brand-subtitle { margin-top: 6px; color: var(--muted); font-size: 13px; }");
  server_.sendContent(".header-pills { display: flex; gap: 8px; flex-wrap: wrap; justify-content: flex-end; }");
  server_.sendContent(".header-pills span { color: #b8c7d6; background: #101721; border: 1px solid var(--line); border-radius: 999px; padding: 7px 10px; font-size: 11px; font-weight: 700; text-transform: uppercase; }");
  server_.sendContent(".status-strip { grid-column: 1 / -1; display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }");
  server_.sendContent(".metric { display: flex; justify-content: space-between; align-items: center; gap: 12px; min-height: 52px; padding: 12px 14px; background: var(--panel-3); border: 1px solid var(--line-soft); border-radius: 8px; }");
  server_.sendContent(".metric-label { color: var(--muted); font-size: 11px; font-weight: 800; text-transform: uppercase; letter-spacing: 1.1px; }");
  server_.sendContent(".metric-value { color: #cbd5e1; font-size: 13px; font-weight: 900; text-transform: uppercase; letter-spacing: 0.5px; text-align: right; }");
  server_.sendContent(".metric-value.ok { color: #86efac; }");
  server_.sendContent(".metric-value.warn { color: #fcd34d; }");
  server_.sendContent(".metric-value.hot { color: #fca5a5; }");
  server_.sendContent(".card { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 16px; box-shadow: 0 14px 36px rgba(0,0,0,0.28); min-width: 0; }");
  server_.sendContent(".card-vision { grid-column: span 2; padding: 0; overflow: hidden; }");
  server_.sendContent(".card-command { grid-column: span 2; }");
  server_.sendContent(".card-system { border-color: rgba(56,189,248,0.32); }");
  server_.sendContent(".card-motor, .card-joint { grid-column: 1 / -1; }");
  server_.sendContent(".vision-layout { display: grid; grid-template-columns: minmax(0, 1fr) 210px; min-height: 270px; }");
  server_.sendContent(".vision-feed { position: relative; background: #05080d; border-right: 1px solid var(--line-soft); display: flex; align-items: center; justify-content: center; min-width: 0; overflow: hidden; }");
  server_.sendContent(".vision-feed img { display: block; width: 100%; height: 100%; max-height: 330px; object-fit: contain; background: #05080d; }");
  server_.sendContent(".vision-overlay { position: absolute; inset: 0; pointer-events: none; overflow: hidden; }");
  server_.sendContent(".vision-target-box { position: absolute; display: none; border: 2px solid rgba(134,239,172,0.95); border-radius: 4px; box-shadow: 0 0 0 1px rgba(4,120,87,0.7), 0 0 24px rgba(34,197,94,0.34); }");
  server_.sendContent(".vision-target-box.visible { display: block; }");
  server_.sendContent(".vision-target-label { position: absolute; left: -2px; top: -28px; padding: 4px 7px; border-radius: 4px; background: rgba(8,11,16,0.82); color: #bbf7d0; border: 1px solid rgba(134,239,172,0.5); font-size: 10px; font-weight: 900; letter-spacing: 1px; text-transform: uppercase; white-space: nowrap; }");
  server_.sendContent(".vision-target-dot { position: absolute; display: none; width: 10px; height: 10px; border-radius: 50%; border: 2px solid #ecfeff; background: rgba(34,197,94,0.82); box-shadow: 0 0 16px rgba(34,197,94,0.8); transform: translate(-50%, -50%); }");
  server_.sendContent(".vision-target-dot.visible { display: block; }");
  server_.sendContent(".vision-lock-state { position: absolute; left: 12px; top: 12px; padding: 6px 9px; border-radius: 999px; border: 1px solid rgba(148,163,184,0.35); background: rgba(8,11,16,0.74); color: #cbd5e1; font-size: 10px; font-weight: 900; letter-spacing: 1px; text-transform: uppercase; }");
  server_.sendContent(".vision-lock-state.lock { color: #bbf7d0; border-color: rgba(34,197,94,0.58); background: rgba(20,83,45,0.42); } .vision-lock-state.track { color: #fef3c7; border-color: rgba(245,158,11,0.55); background: rgba(120,53,15,0.35); }");
  server_.sendContent(".vision-panel { padding: 16px; display: flex; flex-direction: column; gap: 12px; min-width: 0; }");
  server_.sendContent(".camera-status { align-self: flex-start; border: 1px solid #2b3544; border-radius: 999px; color: #94a3b8; background: #0f1722; padding: 6px 9px; font-size: 10px; font-weight: 900; text-transform: uppercase; }");
  server_.sendContent(".camera-status.ok { color: #86efac; border-color: rgba(34,197,94,0.38); background: rgba(34,197,94,0.12); }");
  server_.sendContent(".camera-status.warn { color: #fcd34d; border-color: rgba(245,158,11,0.45); background: rgba(245,158,11,0.12); }");
  server_.sendContent(".camera-status.hot { color: #fca5a5; border-color: rgba(239,68,68,0.45); background: rgba(239,68,68,0.12); }");
  server_.sendContent(".camera-url-row { display: grid; grid-template-columns: 1fr auto; gap: 8px; }");
  server_.sendContent(".camera-url-row input { min-width: 0; padding: 9px; border: 1px solid #334155; border-radius: 6px; background: #0d141d; color: var(--text); font-weight: 700; }");
  server_.sendContent(".camera-actions { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }");
  server_.sendContent(".card-joint { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; }");
  server_.sendContent(".card-joint .card-title, .card-joint .joint-speed-row { grid-column: 1 / -1; }");
  server_.sendContent(".card-title { display: flex; align-items: center; gap: 8px; color: #c9d6e2; font-size: 12px; margin-bottom: 14px; text-transform: uppercase; letter-spacing: 1.2px; font-weight: 800; }");
  server_.sendContent(".card-title::before { content: ''; width: 7px; height: 7px; border-radius: 50%; background: var(--cyan); box-shadow: 0 0 12px rgba(56,189,248,0.7); }");
  server_.sendContent(".status-badge { display: inline-flex; align-items: center; justify-content: center; min-width: 92px; padding: 7px 12px; border-radius: 999px; font-weight: 800; font-size: 12px; text-transform: uppercase; border: 1px solid transparent; }");
  server_.sendContent(".state-BOOT { background: rgba(245,158,11,0.16); color: #ffd48a; border-color: rgba(245,158,11,0.42); }");
  server_.sendContent(".state-IDLE { background: rgba(148,163,184,0.14); color: #cbd5e1; border-color: rgba(148,163,184,0.28); }");
  server_.sendContent(".state-ARMED { background: rgba(34,197,94,0.15); color: #86efac; border-color: rgba(34,197,94,0.38); }");
  server_.sendContent(".state-FAULT { background: rgba(239,68,68,0.16); color: #fca5a5; border-color: rgba(239,68,68,0.45); }");
  server_.sendContent(".state-LOADING { background: rgba(56,189,248,0.12); color: #bae6fd; border-color: rgba(56,189,248,0.35); }");
  server_.sendContent(".info-row { display: flex; justify-content: space-between; align-items: center; gap: 12px; padding: 11px 0; border-bottom: 1px solid var(--line-soft); }");
  server_.sendContent(".info-row:last-child { border-bottom: none; padding-bottom: 0; }");
  server_.sendContent(".info-label { color: var(--muted); font-size: 13px; }");
  server_.sendContent(".info-value { color: var(--text); font-weight: 800; font-size: 13px; text-align: right; overflow-wrap: anywhere; }");
  server_.sendContent(".button-group { display: grid; grid-template-columns: repeat(auto-fit, minmax(92px, 1fr)); gap: 10px; }");
  server_.sendContent("button { flex: 1; min-width: 78px; min-height: 38px; padding: 10px 12px; font-size: 13px; font-weight: 800; border: 1px solid #334155; border-radius: 6px; background: #1d2836; color: #dbeafe; cursor: pointer; transition: background 0.16s, border-color 0.16s, box-shadow 0.16s, transform 0.16s; touch-action: none; -webkit-touch-callout: none; -webkit-user-select: none; user-select: none; text-transform: uppercase; letter-spacing: 0.4px; }");
  server_.sendContent("button:hover { box-shadow: 0 8px 18px rgba(0,0,0,0.22); transform: translateY(-1px); }");
  server_.sendContent("button:active { transform: translateY(0); box-shadow: none; }");
  server_.sendContent("button:disabled { opacity: 0.48; cursor: not-allowed; transform: none; box-shadow: none; }");
  server_.sendContent(".btn-arm { background: #14532d; color: #dcfce7; border-color: rgba(34,197,94,0.45); }");
  server_.sendContent(".btn-light-on { background: #78350f; color: #fef3c7; border-color: rgba(245,158,11,0.5); }");
  server_.sendContent(".btn-light-off { background: #1d2836; color: #cbd5e1; border-color: #334155; }");
  server_.sendContent(".btn-disarm { background: #3f1d25; color: #fecdd3; border-color: rgba(244,63,94,0.45); }");
  server_.sendContent(".btn-stop { background: #4c1d1d; color: #fee2e2; border-color: rgba(239,68,68,0.55); }");
  server_.sendContent(".btn-motor-stop { background: #182332; color: #aebdd0; border-color: #334155; }");
  server_.sendContent(".btn-light-toggle { background: #332515; color: #fcd34d; border-color: rgba(245,158,11,0.35); }");
  server_.sendContent(".btn-forward { background: #0b3b54; color: #d9f3ff; border-color: rgba(56,189,248,0.45); }");
  server_.sendContent(".btn-reverse { background: #263449; color: #d7e4f2; border-color: #40536d; }");
  server_.sendContent(".btn-motor-stop:active { background: #243246; }");
  server_.sendContent(".btn-light-toggle:active { background: #433018; }");
  server_.sendContent(".btn-forward:active { background: #0e4f70; }");
  server_.sendContent(".btn-reverse:active { background: #30425d; }");
  server_.sendContent(".btn-pressed { opacity: 0.88; transform: scale(0.98); box-shadow: inset 0 0 0 1px rgba(255,255,255,0.12), inset 0 3px 8px rgba(0,0,0,0.35); }");
  server_.sendContent(".mode-selector { display: flex; gap: 6px; margin-bottom: 14px; padding: 5px; background: var(--panel-3); border: 1px solid var(--line-soft); border-radius: 8px; }");
  server_.sendContent(".mode-button { flex: 1; min-width: 0; padding: 9px 10px; border: 1px solid transparent; border-radius: 6px; background: transparent; color: var(--muted); cursor: pointer; font-weight: 800; transition: all 0.16s; }");
  server_.sendContent(".mode-button.active { background: #173348; color: #e0f2fe; border-color: rgba(56,189,248,0.45); box-shadow: inset 0 0 0 1px rgba(56,189,248,0.08); }");
  server_.sendContent(".mode-button:hover { border-color: rgba(56,189,248,0.35); }");
  server_.sendContent(".joystick-container { display: none; }");
  server_.sendContent(".joystick-container.active { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; }");
  server_.sendContent(".button-container { display: none; }");
  server_.sendContent(".button-container.active { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; }");
  server_.sendContent(".joystick-row { grid-column: 1 / -1; display: flex; justify-content: center; }");
  server_.sendContent(".joystick-motor-card { min-width: 0; width: 100%; background: var(--panel-2); border: 1px solid var(--line-soft); border-radius: 8px; padding: 12px; }");
  server_.sendContent(".joystick-wrapper { display: flex; flex-direction: column; gap: 12px; align-items: center; }");
  server_.sendContent(".joystick-area { position: relative; width: 82px; height: 82px; border-radius: 50%; background: #0b1119; border: 1px solid #344255; cursor: pointer; touch-action: none; user-select: none; flex-shrink: 0; box-shadow: inset 0 0 0 10px rgba(255,255,255,0.02), inset 0 10px 24px rgba(0,0,0,0.42); }");
  server_.sendContent(".joystick-area.vertical-only { cursor: ns-resize; }");
  server_.sendContent(".joystick-area.horizontal-only { cursor: ew-resize; }");
  server_.sendContent(".joystick-area.disabled { opacity: 0.45; cursor: not-allowed; pointer-events: none; background: #141923; }");
  server_.sendContent(".joystick-handle { position: absolute; width: 24px; height: 24px; border-radius: 50%; background: #38bdf8; border: 2px solid #0f172a; box-shadow: 0 0 0 4px rgba(56,189,248,0.18), 0 8px 18px rgba(0,0,0,0.38); top: 50%; left: 50%; transform: translate(-50%, -50%); transition: none; }");
  server_.sendContent(".joystick-handle.active { box-shadow: 0 0 0 5px rgba(56,189,248,0.22), 0 0 20px rgba(56,189,248,0.45); }");
  server_.sendContent(".joystick-info { width: 100%; display: flex; flex-direction: column; align-items: center; text-align: center; gap: 4px; }");
  server_.sendContent(".joystick-speed { display: none; }");
  server_.sendContent(".joystick-direction { font-size: 11px; color: var(--muted); margin: 0; line-height: 1.2; text-transform: uppercase; letter-spacing: 0.7px; font-weight: 800; }");
  server_.sendContent(".joystick-center-line { position: absolute; width: 1px; height: 100%; background: rgba(148,163,184,0.25); left: 50%; top: 0; transform: translateX(-50%); pointer-events: none; }");
  server_.sendContent(".joystick-center-line.horizontal { width: 100%; height: 1px; top: 50%; left: 0; transform: translateY(-50%); }");
  server_.sendContent(".motor-card { background: var(--panel-2); border: 1px solid var(--line-soft); border-radius: 8px; padding: 13px; min-width: 0; }");
  server_.sendContent(".motor-header { display: flex; justify-content: space-between; align-items: flex-start; gap: 10px; margin-bottom: 10px; }");
  server_.sendContent(".motor-name { font-size: 15px; font-weight: 800; color: var(--text); }");
  server_.sendContent(".motor-role { font-size: 11px; color: var(--muted); margin-top: 3px; text-transform: uppercase; letter-spacing: 0.8px; }");
  server_.sendContent(".motor-status { font-size: 10px; padding: 5px 8px; border-radius: 999px; background: #0f1722; border: 1px solid #2b3544; color: #94a3b8; white-space: nowrap; font-weight: 800; }");
  server_.sendContent(".motor-status.active { background: rgba(34,197,94,0.14); border-color: rgba(34,197,94,0.42); color: #86efac; }");
  server_.sendContent(".joystick-header-speed { font-size: 16px; font-weight: 800; color: var(--cyan); min-width: 45px; text-align: right; }");
  server_.sendContent(".motor-controls { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }");
  server_.sendContent(".speed-slider { flex: 1 1 130px; min-width: 130px; }");
  server_.sendContent(".speed-value { min-width: 48px; text-align: center; font-weight: 800; color: #cbd5e1; font-size: 12px; }");
  server_.sendContent("input[type=\"range\"] { width: 100%; height: 6px; border-radius: 999px; background: #263241; outline: none; accent-color: var(--cyan); }");
  server_.sendContent("input[type=\"range\"]::-webkit-slider-runnable-track { height: 6px; border-radius: 999px; background: #263241; }");
  server_.sendContent("input[type=\"range\"]::-moz-range-track { height: 6px; border-radius: 999px; background: #263241; }");
  server_.sendContent("input[type=\"range\"]::-webkit-slider-thumb { appearance: none; width: 18px; height: 18px; border-radius: 50%; background: var(--cyan); cursor: pointer; border: 2px solid #0f172a; box-shadow: 0 0 0 4px rgba(56,189,248,0.16); }");
  server_.sendContent("input[type=\"range\"]::-moz-range-thumb { width: 18px; height: 18px; border-radius: 50%; background: var(--cyan); cursor: pointer; border: 2px solid #0f172a; }");
  server_.sendContent(".default-speed { margin-bottom: 14px; padding: 12px; border: 1px solid var(--line-soft); border-radius: 8px; background: var(--panel-3); }");
  server_.sendContent(".default-speed-row { display: flex; gap: 10px; align-items: center; margin-bottom: 6px; flex-wrap: wrap; }");
  server_.sendContent(".default-speed-row button { flex: 0 0 96px; }");
  server_.sendContent("label { color: var(--muted); font-size: 12px; font-weight: 800; text-transform: uppercase; letter-spacing: 0.8px; }");
  server_.sendContent("input[type=\"number\"] { width: 82px; padding: 8px; border: 1px solid #334155; border-radius: 6px; background: #0d141d; color: var(--text); font-weight: 800; }");
  server_.sendContent("input[type=\"number\"]:invalid { border-color: var(--red); }");
  server_.sendContent(".validation-message { font-size: 12px; color: #fca5a5; min-height: 16px; margin-top: 2px; }");
  server_.sendContent(".validation-message.valid { color: #86efac; }");
  server_.sendContent(".validation-message.hidden { display: none; }");
  server_.sendContent("input[type='number'].warning { border-color: var(--amber); }");
  server_.sendContent(".message { padding: 10px 12px; border-radius: 6px; margin-top: 10px; display: none; font-size: 13px; font-weight: 700; }");
  server_.sendContent(".message.success { background: rgba(34,197,94,0.12); color: #bbf7d0; border: 1px solid rgba(34,197,94,0.35); }");
  server_.sendContent(".message.error { background: rgba(239,68,68,0.14); color: #fecaca; border: 1px solid rgba(239,68,68,0.4); }");
  server_.sendContent(".loading { display: inline-block; width: 12px; height: 12px; border: 2px solid #263241; border-top: 2px solid var(--cyan); border-radius: 50%; animation: spin 1s linear infinite; }");
  server_.sendContent("@keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }");
  server_.sendContent(".last-update { grid-column: 1 / -1; text-align: right; color: var(--faint); font-size: 11px; padding: 2px 2px 10px; }");
  server_.sendContent(".joint-speed-row { display: flex; gap: 10px; align-items: center; margin-bottom: 2px; padding: 12px; border: 1px solid var(--line-soft); border-radius: 8px; background: var(--panel-3); }");
  server_.sendContent("@media (max-width: 980px) { .container { grid-template-columns: repeat(2, minmax(0, 1fr)); } .card-vision, .card-command, .card-light { grid-column: 1 / -1; } .status-strip { grid-template-columns: repeat(2, minmax(0, 1fr)); } }");
  server_.sendContent("@media (max-width: 720px) { body { padding: 10px; } .container { grid-template-columns: 1fr; gap: 10px; } .app-header { align-items: flex-start; flex-direction: column; } .header-pills { justify-content: flex-start; } .brand-title { font-size: 28px; } .status-strip { grid-template-columns: 1fr; } .vision-layout { grid-template-columns: 1fr; } .vision-feed { border-right: none; border-bottom: 1px solid var(--line-soft); min-height: 220px; } .card-vision, .card-command, .card-light, .card-motor, .card-joint { grid-column: 1 / -1; } .button-container.active, .joystick-container.active, .card-joint { grid-template-columns: 1fr; } .motor-controls { align-items: stretch; } .motor-controls button { flex: 1 1 30%; } }");
  server_.sendContent("</style></head><body>");
  
  // HTML 본문 전송
  server_.sendContent("<div class=\"container\">");
  server_.sendContent("<header class=\"app-header\"><div class=\"brand-block\"><div class=\"brand-kicker\">Control Console</div><h1 class=\"brand-title\">MotionBrain</h1><div class=\"brand-subtitle\">Local robotics operations for bring-up and demo control</div></div><div class=\"header-pills\"><span>ESP32</span><span>Token Gate</span><span>Local Link</span></div></header>");
  server_.sendContent("<section class=\"status-strip\"><div class=\"metric\"><span class=\"metric-label\">Controller</span><span class=\"metric-value warn\" id=\"link-status\">CHECKING</span></div><div class=\"metric\"><span class=\"metric-label\">Teleop</span><span class=\"metric-value warn\" id=\"teleop-status\">CHECKING</span></div><div class=\"metric\"><span class=\"metric-label\">Sensor</span><span class=\"metric-value warn\" id=\"sensor-status\">CHECKING</span></div><div class=\"metric\"><span class=\"metric-label\">Light</span><span class=\"metric-value\" id=\"light-status\">OFF</span></div></section>");
  server_.sendContent("<div class=\"card card-vision\"><div class=\"vision-layout\"><div class=\"vision-feed\" id=\"vision-feed\"><img id=\"camera-stream\" alt=\"ESP32-CAM live feed\"><div class=\"vision-overlay\" id=\"vision-overlay\"><div class=\"vision-target-box\" id=\"vision-target-box\"><span class=\"vision-target-label\" id=\"vision-target-label\">TARGET</span></div><div class=\"vision-target-dot\" id=\"vision-target-dot\"></div><div class=\"vision-lock-state\" id=\"vision-lock-state\">VISION API</div></div></div><div class=\"vision-panel\"><div class=\"card-title\">Vision Feed</div><div class=\"camera-status\" id=\"camera-status\">CONNECTING</div><div class=\"camera-url-row\"><input id=\"camera-url\" type=\"text\" value=\"http://motionbrain-cam.local\" placeholder=\"ESP32-CAM URL\"><button onclick=\"applyCameraUrl()\">CAM</button></div><div class=\"camera-url-row\"><input id=\"vision-url\" type=\"text\" value=\"http://motionbrain-pi.local:8765\" placeholder=\"Dashboard URL\"><button onclick=\"applyVisionUrl()\">API</button></div><div class=\"camera-actions\"><button onclick=\"startTrackedCamera(true)\">TRACKED</button><button onclick=\"startCameraStream()\">STREAM</button><button onclick=\"snapshotCamera()\">SNAPSHOT</button></div></div></div></div>");
  server_.sendContent("<div class=\"card card-system\">");
  server_.sendContent("<div class=\"card-title\">System</div>");
  server_.sendContent("<div class=\"info-row\">");
  server_.sendContent("<span class=\"info-label\">Current State:</span>");
  server_.sendContent("<span class=\"status-badge state-LOADING\" id=\"state-badge\">LOADING</span>");
  server_.sendContent("</div>");
  server_.sendContent("<div class=\"info-row\">");
  server_.sendContent("<span class=\"info-label\">Motor Output:</span>");
  server_.sendContent("<span class=\"info-value\" id=\"motor\">-</span>");
  server_.sendContent("</div></div>");
  server_.sendContent("<div class=\"card card-command\">");
  server_.sendContent("<div class=\"card-title\">Command Authority</div>");
  server_.sendContent("<div class=\"info-row\"><span class=\"info-label\">Command Token:</span><span class=\"info-value\" id=\"token-status\">-</span></div>");
  server_.sendContent("<div class=\"button-group\">");
  server_.sendContent("<button class=\"btn-arm\" id=\"btn-arm\" onclick=\"sendCommand('arm')\">ARM</button>");
  server_.sendContent("<button class=\"btn-disarm\" id=\"btn-disarm\" onclick=\"sendCommand('disarm')\">DISARM</button>");
  server_.sendContent("<button class=\"btn-stop\" id=\"btn-stop\" onclick=\"sendCommand('stop')\">STOP</button>");
  server_.sendContent("</div>");
  server_.sendContent("<div class=\"message\" id=\"message\"></div></div>");
  server_.sendContent("<div class=\"card card-light\">");
  server_.sendContent("<div class=\"card-title\">Search Light</div>");
  server_.sendContent("<div class=\"button-group\">");
  server_.sendContent("<button class=\"btn-light-on\" id=\"btn-light-on\" onclick=\"sendLight('on')\">ON</button>");
  server_.sendContent("<button class=\"btn-light-off\" id=\"btn-light-off\" onclick=\"sendLight('off')\">OFF</button>");
  server_.sendContent("<button class=\"btn-light-toggle\" onclick=\"sendLight('toggle')\">TOGGLE</button>");
  server_.sendContent("</div></div>");
  server_.sendContent("<div class=\"card card-motor\">");
  server_.sendContent("<div class=\"card-title\">Manual Motors</div>");
  server_.sendContent("<div class=\"mode-selector\">");
  server_.sendContent("<button class=\"mode-button active\" id=\"mode-button\" onclick=\"switchMode('button')\">Buttons</button>");
  server_.sendContent("<button class=\"mode-button\" id=\"mode-joystick\" onclick=\"switchMode('joystick')\">Joystick</button>");
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
  server_.sendContent("<div class=\"card card-joint\"><div class=\"card-title\">Joint Console</div>");
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
  server_.sendContent(String("const COMMAND_TOKEN_CONFIGURED = ") + (commandToken_[0] != '\0' ? "true" : "false") + ";");
  server_.sendContent("const COMMAND_TOKEN_REQUIRED = true;");
  server_.sendContent("let commandToken = '';");
  server_.sendContent("function updateTokenStatus() { const el = document.getElementById('token-status'); if (!el) return; if (!COMMAND_TOKEN_CONFIGURED) { el.textContent = 'not provisioned'; return; } el.textContent = commandToken ? 'entered for this page' : 'required'; }");
  server_.sendContent("function commandHeaders(promptForToken = true) { if (!COMMAND_TOKEN_CONFIGURED) { throw new Error('Command token not provisioned'); } if (!commandToken && promptForToken) { const value = window.prompt('MotionBrain command token'); if (value === null || value.trim() === '') { throw new Error('Command token required'); } commandToken = value.trim(); updateTokenStatus(); } if (!commandToken) return null; return {'X-MotionBrain': '1', 'X-MotionBrain-Token': commandToken}; }");
  server_.sendContent("function commandFetch(url, options = {}, promptForToken = true) { let headers; try { headers = commandHeaders(promptForToken); } catch (err) { return Promise.reject(err); } if (!headers) return Promise.reject(new Error('Command token required')); return fetch(url, Object.assign({}, options, { headers })).then(r => r.json().catch(() => ({ success: false, message: 'HTTP ' + r.status }))).then(data => { const errText = String((data && (data.error || data.message)) || ''); if (errText.indexOf('invalid X-MotionBrain-Token') >= 0) { commandToken = ''; updateTokenStatus(); } return data; }); }");
  server_.sendContent("function showMessage(text, isError) { const msg = document.getElementById(\"message\"); msg.textContent = text; msg.className = \"message \" + (isError ? \"error\" : \"success\"); msg.style.display = \"block\"; setTimeout(() => { msg.style.display = \"none\"; }, 3000); }");
  server_.sendContent("function sendCommand(cmd) { const btn = document.getElementById(\"btn-\" + cmd); btn.disabled = true; commandFetch(\"/command?cmd=\" + cmd, { method: \"POST\" }).then(data => { btn.disabled = false; showMessage(data.message || data.error || \"Command sent\", !data.success); updateStatus(); }).catch(err => { btn.disabled = false; showMessage(\"Error: \" + err.message, true); }); }");
  server_.sendContent("function sendLight(action) { commandFetch(\"/light?action=\" + action, { method: \"POST\" }).then(data => { updateStatus(); if (data && data.success === false) showMessage(data.message || data.error || 'Light command failed', true); }).catch(err => { showMessage('Error: ' + err.message, true); }); }");
  server_.sendContent("function setMetric(id, value, tone) { const el = document.getElementById(id); if (!el) return; el.textContent = value; el.className = 'metric-value' + (tone ? ' ' + tone : ''); }");
  server_.sendContent("const DEFAULT_CAMERA_URL = 'http://motionbrain-cam.local';");
  server_.sendContent("const DEFAULT_VISION_URL = 'http://motionbrain-pi.local:8765';");
  server_.sendContent("const CAMERA_STREAM_RECONNECT_MS = 18000;");
  server_.sendContent("const CAMERA_STREAM_RETRY_MS = 2000;");
  server_.sendContent("const TRACKED_FRAME_MS = 350;");
  server_.sendContent("const VISION_POLL_MS = 350;");
  server_.sendContent("const TRACKED_STARTUP_FAILURE_LIMIT = 8;");
  server_.sendContent("const TRACKED_STREAM_RELEASE_MS = 1600;");
  server_.sendContent("const CAMERA_CONFIG_SYNC_RETRY_MS = 5000;");
  server_.sendContent("let cameraBaseUrl = localStorage.getItem('motionbrainCameraUrl') || DEFAULT_CAMERA_URL;");
  server_.sendContent("let visionBaseUrl = localStorage.getItem('motionbrainVisionUrl') || DEFAULT_VISION_URL;");
  server_.sendContent("let cameraReconnectTimer = null; let cameraRetryTimer = null; let trackedFrameTimer = null; let visionPollTimer = null; let trackedWarmupTimer = null; let cameraConfigSyncTimer = null; let cameraConfigSyncInFlight = false; let trackedFailureCount = 0; let cameraMode = 'stream'; let trackedUserRequested = false; let lastVisionDetection = null;");
  server_.sendContent("function normalizeCameraUrl(value) { let url = (value || '').trim(); if (!url) url = DEFAULT_CAMERA_URL; if (!/^https?:\\/\\//i.test(url)) url = 'http://' + url; return url.replace(/\\/+$/, ''); }");
  server_.sendContent("function normalizeVisionUrl(value) { let url = (value || '').trim(); if (!url) url = DEFAULT_VISION_URL; if (!/^https?:\\/\\//i.test(url)) url = 'http://' + url; url = url.replace(/\\/+$/, ''); if (url === 'http://127.0.0.1:8765' || url === 'http://localhost:8765') url = DEFAULT_VISION_URL; return url; }");
  server_.sendContent("function cameraPath(path) { return cameraBaseUrl + path; }");
  server_.sendContent("function visionPath(path) { return visionBaseUrl + path; }");
  server_.sendContent("function setCameraStatus(text, tone) { const el = document.getElementById('camera-status'); if (!el) return; el.textContent = text; el.className = 'camera-status' + (tone ? ' ' + tone : ''); }");
  server_.sendContent("function isDefaultCameraUrl(value) { const url = normalizeCameraUrl(value || cameraBaseUrl); return url === DEFAULT_CAMERA_URL || url.indexOf('motionbrain-cam.local') >= 0; }");
  server_.sendContent("function setCameraBaseUrl(value) { const next = normalizeCameraUrl(value); if (!next || next === cameraBaseUrl) return false; cameraBaseUrl = next; const input = document.getElementById('camera-url'); if (input) input.value = cameraBaseUrl; localStorage.setItem('motionbrainCameraUrl', cameraBaseUrl); return true; }");
  server_.sendContent("function syncCameraFromVisionConfig(force) { if (cameraConfigSyncInFlight) return Promise.resolve(false); if (!force && !isDefaultCameraUrl(cameraBaseUrl)) return Promise.resolve(false); cameraConfigSyncInFlight = true; return fetch(visionPath('/api/config'), { cache: 'no-store' }).then(r => { if (!r.ok) throw new Error('HTTP ' + r.status); return r.json(); }).then(data => { if (!data || !data.cameraUrl) return false; const changed = setCameraBaseUrl(data.cameraUrl); if (changed) setCameraStatus('CAM SYNC', 'warn'); return changed; }).catch(() => false).finally(() => { cameraConfigSyncInFlight = false; }); }");
  server_.sendContent("function scheduleCameraConfigSync(force) { if (cameraConfigSyncTimer) clearTimeout(cameraConfigSyncTimer); cameraConfigSyncTimer = setTimeout(function() { cameraConfigSyncTimer = null; syncCameraFromVisionConfig(!!force).then(changed => { if (changed && cameraMode === 'stream' && !document.hidden) startCameraStream(); }); }, CAMERA_CONFIG_SYNC_RETRY_MS); }");
  server_.sendContent("function clearCameraTimers() { if (cameraReconnectTimer) { clearTimeout(cameraReconnectTimer); cameraReconnectTimer = null; } if (cameraRetryTimer) { clearTimeout(cameraRetryTimer); cameraRetryTimer = null; } if (trackedFrameTimer) { clearTimeout(trackedFrameTimer); trackedFrameTimer = null; } if (visionPollTimer) { clearTimeout(visionPollTimer); visionPollTimer = null; } if (trackedWarmupTimer) { clearTimeout(trackedWarmupTimer); trackedWarmupTimer = null; } if (cameraConfigSyncTimer) { clearTimeout(cameraConfigSyncTimer); cameraConfigSyncTimer = null; } }");
  server_.sendContent("function stopCameraFeed(text) { clearCameraTimers(); const img = document.getElementById('camera-stream'); if (img) img.removeAttribute('src'); if (text) setCameraStatus(text, ''); }");
  server_.sendContent("function scheduleCameraReconnect() { if (cameraMode !== 'stream' || document.hidden) return; if (cameraReconnectTimer) clearTimeout(cameraReconnectTimer); cameraReconnectTimer = setTimeout(function() { if (cameraMode === 'stream') startCameraStream(); }, CAMERA_STREAM_RECONNECT_MS); }");
  server_.sendContent("function startCameraStream() { trackedUserRequested = false; cameraMode = 'stream'; cameraBaseUrl = normalizeCameraUrl(cameraBaseUrl); const input = document.getElementById('camera-url'); if (input) input.value = cameraBaseUrl; localStorage.setItem('motionbrainCameraUrl', cameraBaseUrl); if (document.hidden) { stopCameraFeed('PAUSED'); return; } const img = document.getElementById('camera-stream'); if (!img) return; clearCameraTimers(); setCameraStatus('RAW STREAM', 'ok'); setVisionOverlay({ detected: false, reason: 'raw_stream' }); img.src = cameraPath('/stream?t=' + Date.now()); scheduleCameraReconnect(); syncCameraFromVisionConfig(false).then(changed => { if (changed && cameraMode === 'stream' && !document.hidden) img.src = cameraPath('/stream?t=' + Date.now()); }); }");
  server_.sendContent("function retryCameraStream() { if (cameraMode !== 'stream' || document.hidden) return; if (cameraRetryTimer) clearTimeout(cameraRetryTimer); cameraRetryTimer = setTimeout(function() { if (cameraMode === 'stream') startCameraStream(); }, CAMERA_STREAM_RETRY_MS); }");
  server_.sendContent("function snapshotCamera() { trackedUserRequested = false; cameraMode = 'snapshot'; cameraBaseUrl = normalizeCameraUrl(cameraBaseUrl); const input = document.getElementById('camera-url'); if (input) input.value = cameraBaseUrl; localStorage.setItem('motionbrainCameraUrl', cameraBaseUrl); const img = document.getElementById('camera-stream'); if (!img) return; clearCameraTimers(); setCameraStatus('SNAPSHOT', 'ok'); setVisionOverlay({ detected: false, reason: 'snapshot' }); img.src = cameraPath('/capture?t=' + Date.now()); }");
  server_.sendContent("function applyCameraUrl() { const input = document.getElementById('camera-url'); cameraBaseUrl = normalizeCameraUrl(input ? input.value : cameraBaseUrl); if (input) input.value = cameraBaseUrl; startCameraStream(); }");
  server_.sendContent("function applyVisionUrl() { const input = document.getElementById('vision-url'); visionBaseUrl = normalizeVisionUrl(input ? input.value : visionBaseUrl); if (input) input.value = visionBaseUrl; localStorage.setItem('motionbrainVisionUrl', visionBaseUrl); startTrackedCamera(true); }");
  server_.sendContent("function scheduleTrackedFrame(delay) { if (cameraMode !== 'tracked' || document.hidden) return; if (trackedFrameTimer) clearTimeout(trackedFrameTimer); trackedFrameTimer = setTimeout(loadTrackedFrame, delay || TRACKED_FRAME_MS); }");
  server_.sendContent("function loadTrackedFrame() { if (cameraMode !== 'tracked' || document.hidden) return; const img = document.getElementById('camera-stream'); if (!img) return; img.src = visionPath('/api/vision_frame?t=' + Date.now()); }");
  server_.sendContent("function beginTrackedCamera() { if (cameraMode !== 'tracked' || document.hidden) return; setCameraStatus('TRACKED', 'ok'); loadTrackedFrame(); pollVisionDetection(); }");
  server_.sendContent("function startTrackedCamera(userRequested) { const wasStream = cameraMode === 'stream'; trackedUserRequested = !!userRequested || trackedUserRequested; cameraMode = 'tracked'; visionBaseUrl = normalizeVisionUrl(visionBaseUrl); const input = document.getElementById('vision-url'); if (input) input.value = visionBaseUrl; localStorage.setItem('motionbrainVisionUrl', visionBaseUrl); if (document.hidden) { stopCameraFeed('PAUSED'); return; } const img = document.getElementById('camera-stream'); clearCameraTimers(); trackedFailureCount = 0; if (wasStream && img) { setCameraStatus('RELEASING', 'warn'); setVisionOverlay({ detected: false, reason: 'releasing' }); trackedWarmupTimer = setTimeout(function() { trackedWarmupTimer = null; beginTrackedCamera(); }, TRACKED_STREAM_RELEASE_MS); img.removeAttribute('src'); return; } beginTrackedCamera(); }");
  server_.sendContent("function initCameraFeed() { const input = document.getElementById('camera-url'); const visionInput = document.getElementById('vision-url'); const img = document.getElementById('camera-stream'); cameraBaseUrl = normalizeCameraUrl(cameraBaseUrl); visionBaseUrl = normalizeVisionUrl(visionBaseUrl); if (input) input.value = cameraBaseUrl; if (visionInput) visionInput.value = visionBaseUrl; if (img) { img.onerror = function() { if (cameraMode === 'tracked') { if (trackedWarmupTimer) return; trackedFailureCount++; const warming = trackedFailureCount <= 2; setCameraStatus(warming ? 'WARMING' : 'VISION API', warming ? 'warn' : 'hot'); setVisionOverlay({ detected: false, reason: warming ? 'warming' : 'vision_api' }); if (!trackedUserRequested && trackedFailureCount >= TRACKED_STARTUP_FAILURE_LIMIT) startCameraStream(); else scheduleTrackedFrame(Math.min(5000, CAMERA_STREAM_RETRY_MS + trackedFailureCount * 400)); } else if (cameraMode === 'stream') { setCameraStatus('CAM CHECK', 'hot'); syncCameraFromVisionConfig(true).then(changed => { if (changed && cameraMode === 'stream' && !document.hidden) startCameraStream(); else retryCameraStream(); }); scheduleCameraConfigSync(true); } else { setCameraStatus('OFFLINE', 'hot'); } }; img.onload = function() { if (cameraMode === 'tracked') { trackedFailureCount = 0; setCameraStatus('TRACKED', 'ok'); scheduleTrackedFrame(TRACKED_FRAME_MS); } else { setCameraStatus(cameraMode === 'snapshot' ? 'SNAPSHOT' : 'RAW STREAM', 'ok'); } }; } document.addEventListener('visibilitychange', function() { if (document.hidden) { stopCameraFeed('PAUSED'); } else if (cameraMode === 'tracked') { startTrackedCamera(trackedUserRequested); } else if (cameraMode === 'stream') { startCameraStream(); } }); startCameraStream(); }");
  server_.sendContent("function visionViewport(host, frameWidth, frameHeight) { const width = Number(frameWidth) || 320; const height = Number(frameHeight) || 240; const rect = host.getBoundingClientRect(); const scale = Math.min(rect.width / width, rect.height / height); const drawWidth = width * scale; const drawHeight = height * scale; return { left: (rect.width - drawWidth) / 2, top: (rect.height - drawHeight) / 2, scale: scale }; }");
  server_.sendContent("function setVisionOverlay(payload) { const host = document.getElementById('vision-feed'); const box = document.getElementById('vision-target-box'); const dot = document.getElementById('vision-target-dot'); const label = document.getElementById('vision-target-label'); const state = document.getElementById('vision-lock-state'); if (!host || !box || !dot || !label || !state) return; const detected = !!(payload && payload.detected); const alignment = (payload && payload.alignment) || 'LOST'; if (!detected) { box.classList.remove('visible'); dot.classList.remove('visible'); const reason = payload && payload.reason; state.textContent = reason === 'raw_stream' ? 'RAW STREAM' : (reason === 'snapshot' ? 'SNAPSHOT' : (reason === 'releasing' ? 'RELEASING' : (reason === 'warming' ? 'WARMING' : (reason ? 'VISION API' : 'SEARCHING')))); state.className = 'vision-lock-state'; return; } const centerX = typeof payload.centerX === 'number' ? payload.centerX : payload.centroidX; const centerY = typeof payload.centerY === 'number' ? payload.centerY : payload.centroidY; if (typeof centerX !== 'number' || typeof centerY !== 'number') return; const vp = visionViewport(host, payload.width, payload.height); const tb = payload.targetBox || {}; const fallback = Math.max(34, Math.min(96, Math.sqrt(Math.max(payload.pixels || 0, 1)) * vp.scale * 1.7)); const x = typeof tb.x === 'number' ? vp.left + tb.x * vp.scale : vp.left + centerX * vp.scale - fallback / 2; const y = typeof tb.y === 'number' ? vp.top + tb.y * vp.scale : vp.top + centerY * vp.scale - fallback / 2; const width = typeof tb.width === 'number' ? Math.max(28, tb.width * vp.scale) : fallback; const height = typeof tb.height === 'number' ? Math.max(28, tb.height * vp.scale) : fallback; box.style.left = x + 'px'; box.style.top = y + 'px'; box.style.width = width + 'px'; box.style.height = height + 'px'; box.classList.add('visible'); dot.style.left = (vp.left + centerX * vp.scale) + 'px'; dot.style.top = (vp.top + centerY * vp.scale) + 'px'; dot.classList.add('visible'); const lockText = alignment === 'CENTER' ? 'LOCK' : 'TRACK ' + alignment; const targetName = payload.label || payload.color || 'target'; label.textContent = lockText + ' ' + String(targetName).toUpperCase(); state.textContent = lockText; state.className = 'vision-lock-state ' + (alignment === 'CENTER' ? 'lock' : 'track'); }");
  server_.sendContent("function scheduleVisionPoll() { if (visionPollTimer) clearTimeout(visionPollTimer); visionPollTimer = setTimeout(pollVisionDetection, VISION_POLL_MS); }");
  server_.sendContent("function pollVisionDetection() { if (cameraMode !== 'tracked') return; if (document.hidden) { scheduleVisionPoll(); return; } fetch(visionPath('/api/detection'), { cache: 'no-store' }).then(r => { if (!r.ok) throw new Error('HTTP ' + r.status); return r.json(); }).then(data => { if (cameraMode !== 'tracked') return; lastVisionDetection = data; setVisionOverlay(data); scheduleVisionPoll(); }).catch(() => { if (cameraMode !== 'tracked') return; setVisionOverlay({ detected: false, reason: 'vision_api' }); scheduleVisionPoll(); }); }");
  server_.sendContent("function initVisionOverlay() { window.addEventListener('resize', function() { if (lastVisionDetection) setVisionOverlay(lastVisionDetection); }); }");
  server_.sendContent("function updateStatus() { fetch(\"/status\").then(r => { if (!r.ok) { throw new Error(\"HTTP \" + r.status + \": \" + r.statusText); } return r.text(); }).then(text => { try { const data = JSON.parse(text); const state = data.state || \"UNKNOWN\"; const badge = document.getElementById(\"state-badge\"); if (badge) { badge.textContent = state; badge.className = \"status-badge \" + (stateColors[state] || \"state-LOADING\"); } const motorEl = document.getElementById(\"motor\"); if (motorEl) motorEl.textContent = data.motorEnabled ? \"ACTIVE\" : \"IDLE\"; setMetric('link-status', 'ONLINE', 'ok'); const teleop = data.teleop || {}; const teleopText = teleop.controlActive ? 'ACTIVE' : (teleop.deadman ? 'READY' : (teleop.connected ? 'STANDBY' : 'OFFLINE')); setMetric('teleop-status', teleopText, teleop.connected ? 'ok' : 'hot'); const sensor = data.sensor || {}; const sensorText = sensor.connected ? (sensor.simulated ? 'SIM' : 'LIVE') : 'STALE'; setMetric('sensor-status', sensorText, sensor.connected ? (sensor.simulated ? 'warn' : 'ok') : 'hot'); setMetric('light-status', data.light ? 'ON' : 'OFF', data.light ? 'warn' : ''); const lastUpdate = document.getElementById(\"last-update\"); if (lastUpdate) lastUpdate.textContent = new Date().toLocaleTimeString(); updateButtons(state); if (data.motors) updateMotorStatus(data); } catch (e) { console.error(\"JSON parse error:\", e, \"Response:\", text); } }).catch(err => { setMetric('link-status', 'OFFLINE', 'hot'); console.error(\"Status update error:\", err); }); }");
  server_.sendContent("function updateButtons(state) { const btnArm = document.getElementById(\"btn-arm\"); const btnDisarm = document.getElementById(\"btn-disarm\"); const btnStop = document.getElementById(\"btn-stop\"); btnArm.disabled = (state === \"ARMED\" || state === \"FAULT\" || state === \"BOOT\"); btnDisarm.disabled = (state !== \"ARMED\"); btnStop.disabled = (state === \"IDLE\"); const isArmed = (state === \"ARMED\"); if (btnStop) { btnStop.textContent = (state === \"FAULT\") ? \"RECOVER\" : \"STOP\"; } for (let i = 1; i <= MOTOR_COUNT; i++) { const joystickArea = document.getElementById(\"joystick-\" + i); if (joystickArea) { if (isArmed) { joystickArea.classList.remove(\"disabled\"); } else { joystickArea.classList.add(\"disabled\"); } } } }");
  server_.sendContent("function updateSpeedValue(motorId) { const slider = document.getElementById(\"speed-\" + motorId); const value = document.getElementById(\"speed-value-\" + motorId); value.textContent = slider.value + \"%\"; }");
  server_.sendContent("function validateDefaultSpeed() { const speedInput = document.getElementById(\"default-speed\"); const btnSet = document.getElementById(\"btn-set-speed\"); const validationMsg = document.getElementById(\"speed-validation\"); const value = speedInput.value.trim(); if (value === \"\") { btnSet.disabled = true; validationMsg.textContent = \"Please enter a speed value (1-255)\"; validationMsg.className = \"validation-message\"; speedInput.style.borderColor = \"#f44336\"; return false; } if (value.indexOf(\".\") !== -1 || value.indexOf(\",\") !== -1) { btnSet.disabled = true; validationMsg.textContent = \"Please enter an integer (no decimals)\"; validationMsg.className = \"validation-message\"; speedInput.style.borderColor = \"#f44336\"; return false; } const speed = parseInt(value); if (isNaN(speed)) { btnSet.disabled = true; validationMsg.textContent = \"Please enter a valid number\"; validationMsg.className = \"validation-message\"; speedInput.style.borderColor = \"#f44336\"; return false; } if (speed < 1 || speed > 255) { btnSet.disabled = true; validationMsg.textContent = \"Speed must be between 1 and 255\"; validationMsg.className = \"validation-message\"; speedInput.style.borderColor = \"#f44336\"; return false; } btnSet.disabled = false; validationMsg.textContent = \"Valid speed value\"; validationMsg.className = \"validation-message valid\"; speedInput.style.borderColor = \"#4caf50\"; return true; }");
  server_.sendContent("function setDefaultSpeed() { if (!validateDefaultSpeed()) { return; } const speedInput = document.getElementById(\"default-speed\"); const btnSet = document.getElementById(\"btn-set-speed\"); const speed = parseInt(speedInput.value); btnSet.disabled = true; commandFetch(\"/motor?action=default&speed=\" + speed, { method: \"POST\" }).then(data => { btnSet.disabled = false; showMessage(data.message || data.error || \"Default speed set\", !data.success); if (data.success) { const validationMsg = document.getElementById(\"speed-validation\"); validationMsg.textContent = \"Speed set successfully\"; validationMsg.className = \"validation-message valid\"; } }).catch(err => { btnSet.disabled = false; showMessage(\"Error: \" + err.message, true); }); }");
  server_.sendContent("const MOTOR_COUNT = " + String(MotorControl::NUM_MOTORS) + ";");
  server_.sendContent("let activeMotors = {};");
  server_.sendContent("let joystickActive = {};");
  server_.sendContent("let joystickLastUpdate = {};");
  server_.sendContent("const JOYSTICK_UPDATE_INTERVAL = 100;");
  server_.sendContent("const MANUAL_LEASE_REFRESH_MS = 250;");
  server_.sendContent("let currentMode = 'button';");
  server_.sendContent("let activeJoysticks = {};");
  server_.sendContent("function switchMode(mode) { currentMode = mode; const btnMode = document.getElementById('mode-button'); const joyMode = document.getElementById('mode-joystick'); const btnContainer = document.getElementById('button-container'); const joyContainer = document.getElementById('joystick-container'); if (mode === 'button') { btnMode.classList.add('active'); joyMode.classList.remove('active'); btnContainer.classList.add('active'); joyContainer.classList.remove('active'); stopAllMotors(); for (let motorId in joystickActive) { const handle = document.getElementById('handle-' + motorId); if (handle) { handle.style.transform = 'translate(-50%, -50%)'; handle.classList.remove('active'); } document.getElementById('joy-speed-' + motorId).textContent = '0%'; document.getElementById('joy-direction-' + motorId).textContent = 'STOPPED'; commandFetch('/motor?action=stop&id=' + motorId, { method: 'POST' }, false).catch(() => {}); } joystickActive = {}; for (let motorId in activeJoysticks) { const handle = activeJoysticks[motorId].handle; if (handle) { handle.style.transform = 'translate(-50%, -50%)'; handle.classList.remove('active'); } document.getElementById('joy-speed-' + motorId).textContent = '0%'; document.getElementById('joy-direction-' + motorId).textContent = 'STOPPED'; commandFetch('/motor?action=stop&id=' + motorId, { method: 'POST' }, false).catch(() => {}); } activeJoysticks = {}; } else { btnMode.classList.remove('active'); joyMode.classList.add('active'); btnContainer.classList.remove('active'); joyContainer.classList.add('active'); stopAllMotors(); } }");
  server_.sendContent("function refreshMotorLease(motorId) { const entry = activeMotors[motorId]; if (!entry) return; const speed = document.getElementById('speed-' + motorId).value; const action = entry.direction === 'forward' ? 'forward' : 'reverse'; commandFetch('/motor?action=' + action + '&id=' + motorId + '&percent=' + speed, { method: 'POST' }, false).then(data => { if (!data.success) { showMessage(data.message || data.error || 'Motor lease refresh failed', true); motorStop(motorId); } }).catch(err => { console.error('Motor lease refresh error:', err); motorStop(motorId); }); }");
  server_.sendContent("function motorStart(motorId, direction, e) { if (currentMode !== 'button') return; if (e && e.preventDefault) e.preventDefault(); if (activeMotors[motorId]) return; const speed = document.getElementById('speed-' + motorId).value; const btnId = direction === 'forward' ? 'btn-forward-' + motorId : 'btn-reverse-' + motorId; const btn = document.getElementById(btnId); if (btn) btn.classList.add('btn-pressed'); activeMotors[motorId] = { direction: direction, timer: null }; const action = direction === 'forward' ? 'forward' : 'reverse'; commandFetch('/motor?action=' + action + '&id=' + motorId + '&percent=' + speed, { method: 'POST' }).then(data => { if (!data.success) { showMessage(data.message || data.error || 'Motor control failed', true); motorStop(motorId); return; } const entry = activeMotors[motorId]; if (entry && !entry.timer) entry.timer = setInterval(() => refreshMotorLease(motorId), MANUAL_LEASE_REFRESH_MS); }).catch(err => { showMessage('Error: ' + err.message, true); motorStop(motorId); }); }");
  server_.sendContent("function motorStop(motorId, e) { if (e && e.preventDefault) e.preventDefault(); const entry = activeMotors[motorId]; if (entry) { const direction = entry.direction; if (entry.timer) clearInterval(entry.timer); const btnId = direction === 'forward' ? 'btn-forward-' + motorId : 'btn-reverse-' + motorId; const btn = document.getElementById(btnId); if (btn) btn.classList.remove('btn-pressed'); delete activeMotors[motorId]; commandFetch('/motor?action=stop&id=' + motorId, { method: 'POST' }, false).then(data => { updateStatus(); }).catch(err => { console.error('Stop error:', err); }); } }");
  server_.sendContent("function stopAllMotors() { for (let motorId in activeMotors) { motorStop(parseInt(motorId)); } }");
  // motorForward/motorReverse 제거됨 — motorStart/motorStop으로 대체 (CSRF 헤더 포함)
  server_.sendContent("function updateMotorStatus(data) { if (data.motors) { for (let i = 1; i <= MOTOR_COUNT; i++) { const motor = data.motors[\"M\" + i]; if (motor) { const statusEl = document.getElementById(\"motor-status-\" + i); const joySpeedEl = document.getElementById(\"joy-speed-\" + i); const joyDirectionEl = document.getElementById(\"joy-direction-\" + i); if (motor.enabled) { const statusText = motor.direction.toUpperCase() + \" (\" + motor.speed + \")\"; if (statusEl) { statusEl.textContent = statusText; statusEl.className = \"motor-status active\"; } if (joySpeedEl) joySpeedEl.textContent = Math.abs(motor.speed) + '%'; if (joyDirectionEl) joyDirectionEl.textContent = motor.direction.toUpperCase(); } else { if (statusEl) { statusEl.textContent = \"STOPPED\"; statusEl.className = \"motor-status\"; } if (joySpeedEl) joySpeedEl.textContent = '0%'; if (joyDirectionEl) joyDirectionEl.textContent = 'STOPPED'; } } } } }");
  server_.sendContent("function initJoystick(motorId) { const area = document.getElementById('joystick-' + motorId); const handle = document.getElementById('handle-' + motorId); if (!area || !handle) return; const isVertical = motorId >= 1 && motorId <= 4; const isHorizontal = motorId === 5; let centerX = 0; let centerY = 0; let radius = 0; function updateCenter() { const rect = area.getBoundingClientRect(); centerX = rect.left + rect.width / 2; centerY = rect.top + rect.height / 2; radius = rect.width / 2 - 10; } function updateJoystick(clientX, clientY) { if (area.classList.contains('disabled')) return; const dx = clientX - centerX; const dy = clientY - centerY; let x = 0; let y = 0; let speedPercent = 0; let isForward = false; if (isVertical) { const distance = Math.abs(dy); const limitedDistance = Math.min(distance, radius); y = dy < 0 ? -limitedDistance : limitedDistance; speedPercent = Math.round((limitedDistance / radius) * 100); isForward = dy < 0; } else if (isHorizontal) { const distance = Math.abs(dx); const limitedDistance = Math.min(distance, radius); x = dx < 0 ? -limitedDistance : limitedDistance; speedPercent = Math.round((limitedDistance / radius) * 100); isForward = dx < 0; } handle.style.transform = 'translate(calc(-50% + ' + x + 'px), calc(-50% + ' + y + 'px))'; const direction = isForward ? 'FORWARD' : (speedPercent < 5 ? 'STOPPED' : 'REVERSE'); document.getElementById('joy-speed-' + motorId).textContent = speedPercent + '%'; document.getElementById('joy-direction-' + motorId).textContent = direction; if (speedPercent > 5) { const action = isForward ? 'forward' : 'reverse'; const now = Date.now(); if (!joystickLastUpdate[motorId] || now - joystickLastUpdate[motorId] >= JOYSTICK_UPDATE_INTERVAL) { joystickLastUpdate[motorId] = now; commandFetch('/motor?action=' + action + '&id=' + motorId + '&percent=' + speedPercent, { method: 'POST' }).then(data => { if (!data.success) { console.error('Joystick control failed:', data); } }).catch(err => { console.error('Joystick error:', err); }); } joystickActive[motorId] = { action: action, percent: speedPercent }; } else { if (joystickActive[motorId]) { commandFetch('/motor?action=stop&id=' + motorId, { method: 'POST' }, false).catch(err => console.error('Stop error:', err)); delete joystickActive[motorId]; } } } function getTouchPoint(e, storedTouchId, joystickArea) { if (e.touches && storedTouchId !== null) { for (let i = 0; i < e.touches.length; i++) { if (e.touches[i].identifier === storedTouchId) { return { x: e.touches[i].clientX, y: e.touches[i].clientY }; } } return null; } if (e.clientX !== undefined && e.clientY !== undefined) { const rect = joystickArea.getBoundingClientRect(); if (e.clientX >= rect.left && e.clientX <= rect.right && e.clientY >= rect.top && e.clientY <= rect.bottom) { return { x: e.clientX, y: e.clientY }; } } return null; } function findTouchInArea(e, joystickArea) { if (e.touches && e.touches.length > 0) { const rect = joystickArea.getBoundingClientRect(); const usedTouchIds = new Set(); for (let id in activeJoysticks) { if (activeJoysticks[id] && activeJoysticks[id].touchId !== null) { usedTouchIds.add(activeJoysticks[id].touchId); } } for (let i = 0; i < e.touches.length; i++) { const touch = e.touches[i]; if (touch.clientX >= rect.left && touch.clientX <= rect.right && touch.clientY >= rect.top && touch.clientY <= rect.bottom) { if (!usedTouchIds.has(touch.identifier)) { return touch.identifier; } } } } return null; } function startDrag(e) { if (currentMode !== 'joystick' || area.classList.contains('disabled')) return; e.preventDefault(); updateCenter(); let currentTouchId = null; if (e.touches && e.touches.length > 0) { currentTouchId = findTouchInArea(e, area); if (currentTouchId === null) return; } const joyObj = { area: area, handle: handle, updateCenter: updateCenter, updateJoystick: updateJoystick, motorId: motorId, touchId: currentTouchId }; joyObj.getTouchPoint = function(e) { return getTouchPoint(e, joyObj.touchId, joyObj.area); }; activeJoysticks[motorId] = joyObj; handle.classList.add('active'); const point = getTouchPoint(e, currentTouchId, area); if (point) updateJoystick(point.x, point.y); } area.addEventListener('mousedown', startDrag); area.addEventListener('touchstart', startDrag, { passive: false }); }");
  server_.sendContent("function refreshJoystickLeases() { if (currentMode !== 'joystick') return; for (let motorId in joystickActive) { const entry = joystickActive[motorId]; if (!entry || !entry.action || !entry.percent) continue; commandFetch('/motor?action=' + entry.action + '&id=' + motorId + '&percent=' + entry.percent, { method: 'POST' }, false).then(data => { if (!data.success) { console.error('Joystick lease refresh failed:', data); commandFetch('/motor?action=stop&id=' + motorId, { method: 'POST' }, false).catch(() => {}); delete joystickActive[motorId]; } }).catch(err => { console.error('Joystick lease refresh error:', err); commandFetch('/motor?action=stop&id=' + motorId, { method: 'POST' }, false).catch(() => {}); delete joystickActive[motorId]; }); } }");
  server_.sendContent("function handleGlobalDrag(e) { let shouldPreventDefault = false; const isMouseEvent = e.type === 'mousemove'; let mouseHandled = false; for (let motorId in activeJoysticks) { const joy = activeJoysticks[motorId]; if (joy && joy.area && !joy.area.classList.contains('disabled')) { if (isMouseEvent && joy.touchId !== null) continue; if (isMouseEvent && mouseHandled) continue; joy.updateCenter(); const point = joy.getTouchPoint(e); if (point) { shouldPreventDefault = true; if (isMouseEvent) mouseHandled = true; joy.updateJoystick(point.x, point.y); } } } if (shouldPreventDefault && e.touches && e.touches.length > 0) { e.preventDefault(); } } function handleGlobalEndDrag(e) { const endedTouchIds = new Set(); if (e.changedTouches) { for (let i = 0; i < e.changedTouches.length; i++) { endedTouchIds.add(e.changedTouches[i].identifier); } } let shouldPreventDefault = false; const isMouseEvent = e.type === 'mouseup'; for (let motorId in activeJoysticks) { const joy = activeJoysticks[motorId]; if (joy && joy.area) { let shouldEnd = false; if (isMouseEvent) { if (joy.touchId === null) { shouldEnd = true; } } else if (e.type === 'touchend' || e.type === 'touchcancel') { if (joy.touchId !== null && endedTouchIds.has(joy.touchId)) { shouldEnd = true; shouldPreventDefault = true; } } if (shouldEnd) { const handle = joy.handle; delete activeJoysticks[motorId]; handle.classList.remove('active'); handle.style.transform = 'translate(-50%, -50%)'; document.getElementById('joy-speed-' + joy.motorId).textContent = '0%'; document.getElementById('joy-direction-' + joy.motorId).textContent = 'STOPPED'; if (joystickActive[joy.motorId]) { commandFetch('/motor?action=stop&id=' + joy.motorId, { method: 'POST' }, false).catch(err => console.error('Stop error:', err)); delete joystickActive[joy.motorId]; } } } } if (shouldPreventDefault && e.changedTouches && e.changedTouches.length > 0) { e.preventDefault(); } } document.addEventListener('mousemove', handleGlobalDrag); document.addEventListener('touchmove', handleGlobalDrag, { passive: false }); document.addEventListener('mouseup', handleGlobalEndDrag); document.addEventListener('touchend', handleGlobalEndDrag, { passive: false }); document.addEventListener('touchcancel', handleGlobalEndDrag, { passive: false });");
  server_.sendContent("window.addEventListener(\"load\", function() { updateTokenStatus(); validateDefaultSpeed(); initCameraFeed(); initVisionOverlay(); for (let i = 1; i <= MOTOR_COUNT; i++) { initJoystick(i); } });");
  server_.sendContent("window.addEventListener(\"beforeunload\", function() { stopAllMotors(); for (let motorId in joystickActive) { commandFetch('/motor?action=stop&id=' + motorId, { method: 'POST' }, false).catch(() => {}); } for (let motorId in activeJoysticks) { commandFetch('/motor?action=stop&id=' + motorId, { method: 'POST' }, false).catch(() => {}); } });");
  server_.sendContent("document.addEventListener(\"keydown\", function(e) { if (currentMode !== 'button') return; const keyMap = { 'KeyQ': { motor: 1, dir: 'forward' }, 'KeyA': { motor: 1, dir: 'reverse' }, 'KeyW': { motor: 2, dir: 'forward' }, 'KeyS': { motor: 2, dir: 'reverse' }, 'KeyE': { motor: 3, dir: 'forward' }, 'KeyD': { motor: 3, dir: 'reverse' }, 'KeyR': { motor: 4, dir: 'forward' }, 'KeyF': { motor: 4, dir: 'reverse' }, 'KeyT': { motor: 5, dir: 'forward' }, 'KeyG': { motor: 5, dir: 'reverse' } }; const mapping = keyMap[e.code]; if (mapping && !activeMotors[mapping.motor]) { e.preventDefault(); motorStart(mapping.motor, mapping.dir); } });");
  server_.sendContent("document.addEventListener(\"keyup\", function(e) { if (currentMode !== 'button') return; const keyMap = { 'KeyQ': 1, 'KeyA': 1, 'KeyW': 2, 'KeyS': 2, 'KeyE': 3, 'KeyD': 3, 'KeyR': 4, 'KeyF': 4, 'KeyT': 5, 'KeyG': 5 }; const motorId = keyMap[e.code]; if (motorId && activeMotors[motorId]) { e.preventDefault(); motorStop(motorId); } });");
  server_.sendContent("let activeJointButtons = {};");
  server_.sendContent("function updateJointSpeed() { const v = document.getElementById('joint-speed').value; document.getElementById('joint-speed-value').textContent = v + '%'; }");
  server_.sendContent("function refreshJointLease(joint) { const entry = activeJointButtons[joint]; if (!entry) return; const speed = document.getElementById('joint-speed').value; commandFetch('/joint?joint=' + joint + '&action=' + entry.action + '&percent=' + speed, {method: 'POST'}, false).then(data => { if (!data.success) { showMessage(data.message || data.error || 'Joint lease refresh failed', true); jointStopNow(joint); } }).catch(err => { console.error('Joint lease refresh error:', err); jointStopNow(joint); }); }");
  server_.sendContent("function jointStart(joint, action, e) { if (e && e.preventDefault) e.preventDefault(); if (activeJointButtons[joint]) return; const speed = document.getElementById('joint-speed').value; const btn = e ? e.currentTarget : null; if (btn) btn.classList.add('btn-pressed'); activeJointButtons[joint] = {action: action, btn: btn, timer: null}; commandFetch('/joint?joint=' + joint + '&action=' + action + '&percent=' + speed, {method: 'POST'}).then(data => { if (!data.success) { showMessage(data.message || data.error || 'Joint control failed', true); jointStopNow(joint); return; } const entry = activeJointButtons[joint]; if (entry && !entry.timer) entry.timer = setInterval(() => refreshJointLease(joint), MANUAL_LEASE_REFRESH_MS); }).catch(err => { showMessage('Error: ' + err.message, true); jointStopNow(joint); }); }");
  server_.sendContent("function jointStop(joint, e) { if (e && e.preventDefault) e.preventDefault(); if (activeJointButtons[joint]) { const entry = activeJointButtons[joint]; if (entry.timer) clearInterval(entry.timer); if (entry.btn) entry.btn.classList.remove('btn-pressed'); delete activeJointButtons[joint]; commandFetch('/joint?joint=' + joint + '&action=stop', {method: 'POST'}, false).catch(() => {}); } }");
  server_.sendContent("function jointStopNow(joint) { const entry = activeJointButtons[joint]; if (entry && entry.timer) clearInterval(entry.timer); if (entry && entry.btn) entry.btn.classList.remove('btn-pressed'); delete activeJointButtons[joint]; commandFetch('/joint?joint=' + joint + '&action=stop', {method: 'POST'}, false).catch(() => {}); }");
  server_.sendContent("setInterval(refreshJoystickLeases, MANUAL_LEASE_REFRESH_MS); setInterval(updateStatus, 1000); updateStatus();");
  server_.sendContent("</script></body></html>");
  
  DebugLog::info("Web Server: HTML sent successfully (streaming mode)");
}

/**
 * GET /status 처리
 * JSON 형식으로 현재 상태 반환
 * 주의: CSRF 헤더 불필요 — GET은 읽기 전용이므로 상태 변경 없음
 */
void MotionBrainWebServer::handleStatus() {
  if (systemState_ == nullptr) {
    sendErrorJson(500, "SystemStateManager not initialized");
    return;
  }
  sendNoStoreHeaders(server_);
  
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
  json += ",\"sensor\":{";
  json += "\"source\":\"";
  json += useStm32Sensor ? "stm32_bridge" : "teleop_embedded";
  json += "\",\"connected\":";
  json += sensorConnected ? "true" : "false";
  json += ",\"simulated\":";
  json += stm32Bridge.isSimulationEnabled() ? "true" : "false";
  json += ",\"simulationMode\":\"";
  json += stm32Bridge.getSimulationModeString();
  json += "\"";
  json += ",\"lastUpdateMs\":";
  json += String(sensorAgeMs);
  json += ",\"packetsReceived\":";
  json += String(sensorPackets);
  json += ",\"parseErrors\":";
  json += String(sensorParseErrors);
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
  json += ",\"obstacleSafetyEnabled\":";
  json += snapshot.obstacleSafetyEnabled ? "true" : "false";
  json += ",\"vibrationSafetyEnabled\":";
  json += snapshot.vibrationSafetyEnabled ? "true" : "false";
  json += ",\"imuStatus\":";
  json += String(snapshot.imuStatus);
  json += ",\"imuAddress\":";
  json += String(snapshot.imuAddress);
  json += ",\"imuError\":";
  json += String(snapshot.imuError);
  json += ",\"i2cSclHigh\":";
  json += snapshot.i2cSclHigh ? "true" : "false";
  json += ",\"i2cSdaHigh\":";
  json += snapshot.i2cSdaHigh ? "true" : "false";
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
  json += ",\"gripOpen\":";
  json += teleopAdapter.getLastGripOpen() ? "true" : "false";
  json += ",\"gripClose\":";
  json += teleopAdapter.getLastGripClose() ? "true" : "false";
  json += ",\"ledToggleSeq\":";
  json += String(teleopAdapter.getLastLedToggleSeq());
  json += ",\"embeddedSafety\":";
  json += teleopAdapter.hasEmbeddedSafetySnapshot() ? "true" : "false";
  json += ",\"embeddedSafetyAgeMs\":";
  json += String(teleopAdapter.getEmbeddedSafetyAgeMs());
  json += ",\"embeddedSafetyPackets\":";
  json += String(teleopAdapter.getEmbeddedSafetyPacketsReceived());
  json += ",\"lastStopReason\":\"";
  json += teleopAdapter.getLastStopReasonString();
  json += "\"}";

  json += "}";

  server_.send(200, "application/json", json);
}

void MotionBrainWebServer::handleEvents() {
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

  if (!requireCommandAuth()) {
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
  if (result.success && (command.type == CommandType::STOP || command.type == CommandType::DISARM)) {
    clearAllManualLeases();
  }
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

  if (!requireCommandAuth()) {
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
  if (result.success) {
    if (action == "forward" || action == "reverse") {
      extendManualLease(command.motorId);
      sendCommandResult(result, String("\"manualLeaseMs\":") + String(MANUAL_COMMAND_LEASE_MS));
      return;
    }
    if (action == "stop") {
      clearManualLease(command.motorId);
    }
  }

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

  if (!requireCommandAuth()) {
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
  if (result.success) {
    if (command.type == CommandType::JOINT_RUN) {
      extendManualLease(motorIdForJoint(command.joint));
      sendCommandResult(result, String("\"manualLeaseMs\":") + String(MANUAL_COMMAND_LEASE_MS));
      return;
    }
    if (command.type == CommandType::JOINT_STOP) {
      clearManualLease(motorIdForJoint(command.joint));
    } else if (command.type == CommandType::JOINT_STOP_ALL) {
      clearAllManualLeases();
    }
  }

  sendCommandResult(result);
}

void MotionBrainWebServer::handleBase() {
  DebugLog::debug("Web Server: POST /base requested");

  if (!requireCommandAuth()) {
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

  if (!requireCommandAuth()) {
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
  if (!requireCommandAuth()) {
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
