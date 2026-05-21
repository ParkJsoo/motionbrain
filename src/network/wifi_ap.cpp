#include "wifi_ap.h"
#include "debug/debug_log.h"
#include <ESPmDNS.h>

/**
 * WiFiAP 생성자
 */
WiFiAP::WiFiAP()
  : active_(false)
  , stationMode_(false)
  , apIP_(192, 168, 4, 1)
  , lastCheckTime_(0)
{
  hostname_[0] = '\0';
  ssid_[0] = '\0';
  password_[0] = '\0';
}

/**
 * 초기화
 * Wi-Fi AP 모드 시작
 */
bool WiFiAP::init(const char* ssid, const char* password, IPAddress ip) {
  if (ssid == nullptr) {
    DebugLog::error("WiFi AP: SSID cannot be nullptr");
    return false;
  }

  strncpy(ssid_, ssid, sizeof(ssid_) - 1);
  ssid_[sizeof(ssid_) - 1] = '\0';
  if (password != nullptr) {
    strncpy(password_, password, sizeof(password_) - 1);
    password_[sizeof(password_) - 1] = '\0';
  } else {
    password_[0] = '\0';
  }
  apIP_ = ip;
  stationMode_ = false;
  hostname_[0] = '\0';

  DebugLog::info("=== Wi-Fi AP Initialization ===");
  DebugLog::info("SSID: %s", ssid);
  DebugLog::info("Password: %s", password ? "***" : "(open)");
  DebugLog::info("IP: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  DebugLog::info("Max clients: %d (Phase 4: host + vision + viewer)", MAX_CLIENTS);

  // Wi-Fi 모드 설정 (AP 모드)
  WiFi.mode(WIFI_AP);

  // IP 주소 설정 (softAP() 이전에 호출해야 적용됨)
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));

  // AP 시작 (최대 연결 수 제한 - 안전 규칙: 단일 클라이언트만 허용)
  // WiFi.softAP(ssid, password, channel, hidden, max_connection)
  bool result = false;
  if (password != nullptr && strlen(password) >= 8) {
    // 비밀번호가 있는 경우 (WPA2: 최소 8자)
    result = WiFi.softAP(ssid, password, 1, false, MAX_CLIENTS);
  } else {
    // 공개 AP (비밀번호 없음 또는 8자 미만 — WPA2 최소 요건 미충족)
    if (password != nullptr && strlen(password) > 0) {
      DebugLog::warn("WiFi AP: Password too short (< 8 chars) — starting as OPEN AP");
    }
    result = WiFi.softAP(ssid, nullptr, 1, false, MAX_CLIENTS);
  }

  if (!result) {
    DebugLog::error("WiFi AP: Failed to start AP");
    active_ = false;
    return false;
  }

  // AP가 softAP() 성공으로 이미 확인됨
  active_ = true;
  DebugLog::info("WiFi AP: Started successfully");
  DebugLog::info("AP IP: %s", WiFi.softAPIP().toString().c_str());
  DebugLog::info("AP MAC: %s", WiFi.softAPmacAddress().c_str());
  DebugLog::info("Max clients: %d (Phase 4: host + vision + viewer)", MAX_CLIENTS);

  lastCheckTime_ = millis();
  return true;
}

bool WiFiAP::initStation(const char* ssid, const char* password, const char* hostname,
                         uint32_t timeoutMs) {
  if (ssid == nullptr || ssid[0] == '\0') {
    DebugLog::error("WiFi STA: SSID is not configured");
    return false;
  }

  strncpy(ssid_, ssid, sizeof(ssid_) - 1);
  ssid_[sizeof(ssid_) - 1] = '\0';
  if (password != nullptr) {
    strncpy(password_, password, sizeof(password_) - 1);
    password_[sizeof(password_) - 1] = '\0';
  } else {
    password_[0] = '\0';
  }

  const char* effectiveHostname =
    (hostname != nullptr && hostname[0] != '\0') ? hostname : "motionbrain";
  strncpy(hostname_, effectiveHostname, sizeof(hostname_) - 1);
  hostname_[sizeof(hostname_) - 1] = '\0';
  stationMode_ = true;
  apIP_ = IPAddress(0, 0, 0, 0);

  DebugLog::info("=== Wi-Fi Station Initialization ===");
  DebugLog::info("SSID: configured");
  DebugLog::info("Password: %s", password_[0] != '\0' ? "***" : "(open)");
  DebugLog::info("Hostname: %s", hostname_);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname_);
  WiFi.begin(ssid_, password_[0] != '\0' ? password_ : nullptr);

  const uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startedAt) < timeoutMs) {
    delay(250);
  }

  if (WiFi.status() != WL_CONNECTED) {
    DebugLog::error("WiFi STA: Failed to connect within %lums", timeoutMs);
    active_ = false;
    return false;
  }

  active_ = true;
  DebugLog::info("WiFi STA: Connected successfully");
  DebugLog::info("STA IP: %s", WiFi.localIP().toString().c_str());
  DebugLog::info("STA MAC: %s", WiFi.macAddress().c_str());
  DebugLog::info("RSSI: %d dBm", WiFi.RSSI());

  if (MDNS.begin(hostname_)) {
    MDNS.addService("http", "tcp", 80);
    DebugLog::info("mDNS: http://%s.local", hostname_);
  } else {
    DebugLog::warn("mDNS: start failed");
  }

  lastCheckTime_ = millis();
  return true;
}

/**
 * 업데이트 (주기적으로 호출)
 * 클라이언트 접속 상태 확인
 */
void WiFiAP::update() {
  if (!active_) {
    return;
  }

  // 5초마다 네트워크 상태 체크
  uint32_t currentTime = millis();
  if (currentTime - lastCheckTime_ >= 5000) {
    if (stationMode_) {
      checkStation();
    } else {
      checkClients();
    }
    lastCheckTime_ = currentTime;
  }
}

/**
 * AP 활성화 여부 확인
 */
bool WiFiAP::isActive() const {
  return active_;
}

/**
 * 연결된 클라이언트 수 확인
 */
uint8_t WiFiAP::getClientCount() const {
  if (!active_) {
    return 0;
  }
  if (stationMode_) {
    return WiFi.status() == WL_CONNECTED ? 1 : 0;
  }
  return WiFi.softAPgetStationNum();
}

/**
 * AP IP 주소 가져오기
 */
IPAddress WiFiAP::getIP() const {
  if (!active_) {
    return IPAddress(0, 0, 0, 0);
  }
  if (stationMode_) {
    return WiFi.localIP();
  }
  return WiFi.softAPIP();
}

/**
 * AP SSID 가져오기
 */
const char* WiFiAP::getSSID() const {
  return ssid_;
}

/**
 * 클라이언트 접속 상태 체크 (private)
 */
void WiFiAP::checkClients() {
  if (!active_) {
    return;
  }

  uint8_t clientCount = WiFi.softAPgetStationNum();

  // 클라이언트 수가 변경되었을 때만 로그 출력
  if (clientCount != lastClientCount_) {
    if (clientCount > lastClientCount_) {
      if (clientCount >= MAX_CLIENTS) {
        DebugLog::info("WiFi AP: Client connected (Total: %d/%d - MAX REACHED)", clientCount, MAX_CLIENTS);
        DebugLog::warn("WiFi AP: Maximum clients reached - new connections will be rejected");
      } else {
        DebugLog::info("WiFi AP: Client connected (Total: %d/%d)", clientCount, MAX_CLIENTS);
      }
    } else if (clientCount < lastClientCount_ && lastClientCount_ != 255) {
      DebugLog::info("WiFi AP: Client disconnected (Total: %d/%d)", clientCount, MAX_CLIENTS);
    }
    lastClientCount_ = clientCount;
  }
}

void WiFiAP::checkStation() {
  if (!stationMode_) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  DebugLog::warn("WiFi STA: disconnected; reconnecting");
  WiFi.disconnect();
  WiFi.begin(ssid_, password_[0] != '\0' ? password_ : nullptr);
}
