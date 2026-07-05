#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include "esp_camera.h"

// AI Thinker ESP32-CAM pin map.
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

namespace {

#ifndef MOTIONBRAIN_CAMERA_HOSTNAME
#define MOTIONBRAIN_CAMERA_HOSTNAME "motionbrain-cam"
#endif

const uint32_t STREAM_FRAME_DELAY_MS = 100;
const uint32_t STREAM_MAX_DURATION_MS = 20000;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;
const uint32_t CAMERA_RECOVERY_SETTLE_MS = 150;
const framesize_t DEFAULT_CAMERA_FRAME_SIZE = FRAMESIZE_QVGA;
const int DEFAULT_CAMERA_JPEG_QUALITY = 15;
const int MIN_CAMERA_JPEG_QUALITY = 4;
const int MAX_CAMERA_JPEG_QUALITY = 30;

struct CameraProfileOption {
  const char* name;
  framesize_t frameSize;
  uint16_t width;
  uint16_t height;
};

const CameraProfileOption CAMERA_PROFILE_OPTIONS[] = {
    {"qvga", FRAMESIZE_QVGA, 320, 240},
    {"vga", FRAMESIZE_VGA, 640, 480},
    {"svga", FRAMESIZE_SVGA, 800, 600},
};

WebServer server(80);

const CameraProfileOption* currentFrameProfile = &CAMERA_PROFILE_OPTIONS[0];
int currentJpegQuality = DEFAULT_CAMERA_JPEG_QUALITY;

struct CameraStats {
  uint32_t captures = 0;
  uint32_t captureFailures = 0;
  uint32_t consecutiveCaptureFailures = 0;
  uint32_t clientWriteFailures = 0;
  uint32_t cameraRecoveries = 0;
  uint32_t lastRecoveryMs = 0;
  uint32_t lastRecoveryDurationMs = 0;
  uint32_t lastCaptureMs = 0;
  uint32_t maxCaptureMs = 0;
  uint32_t lastFrameBytes = 0;
  bool lastRecoveryOk = false;
};

CameraStats cameraStats;
String lastCameraError;

const CameraProfileOption* findCameraProfile(framesize_t frameSize) {
  for (const CameraProfileOption& option : CAMERA_PROFILE_OPTIONS) {
    if (option.frameSize == frameSize) {
      return &option;
    }
  }
  return nullptr;
}

const CameraProfileOption* findCameraProfile(const String& value) {
  String normalized = value;
  normalized.trim();
  normalized.toLowerCase();
  for (const CameraProfileOption& option : CAMERA_PROFILE_OPTIONS) {
    if (normalized == option.name) {
      return &option;
    }
  }
  return nullptr;
}

String allowedCameraProfilesJson() {
  String json = "[";
  for (size_t i = 0; i < sizeof(CAMERA_PROFILE_OPTIONS) / sizeof(CAMERA_PROFILE_OPTIONS[0]); ++i) {
    if (i > 0) {
      json += ",";
    }
    const CameraProfileOption& option = CAMERA_PROFILE_OPTIONS[i];
    json += "{\"name\":\"";
    json += option.name;
    json += "\",\"width\":";
    json += option.width;
    json += ",\"height\":";
    json += option.height;
    json += "}";
  }
  json += "]";
  return json;
}

bool parseQualityArg(const String& value, int& quality) {
  String trimmed = value;
  trimmed.trim();
  if (trimmed.length() == 0) {
    return false;
  }
  char* end = nullptr;
  long parsed = strtol(trimmed.c_str(), &end, 10);
  if (end == trimmed.c_str() || *end != '\0') {
    return false;
  }
  if (parsed < MIN_CAMERA_JPEG_QUALITY || parsed > MAX_CAMERA_JPEG_QUALITY) {
    return false;
  }
  quality = static_cast<int>(parsed);
  return true;
}

bool applyCameraProfile(const CameraProfileOption* profile, int quality, String& error) {
  if (profile == nullptr) {
    error = "unknown_framesize";
    return false;
  }
  if (quality < MIN_CAMERA_JPEG_QUALITY || quality > MAX_CAMERA_JPEG_QUALITY) {
    error = "invalid_quality";
    return false;
  }
  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    error = "camera_sensor_unavailable";
    return false;
  }
  if (sensor->set_framesize(sensor, profile->frameSize) != 0) {
    error = "set_framesize_failed";
    return false;
  }
  if (sensor->set_quality(sensor, quality) != 0) {
    error = "set_quality_failed";
    return false;
  }

  currentFrameProfile = profile;
  currentJpegQuality = quality;
  cameraStats = CameraStats{};
  lastCameraError = "";
  return true;
}

bool configureCamera(const CameraProfileOption* profile, int quality, String& error) {
  if (profile == nullptr) {
    error = "unknown_framesize";
    return false;
  }
  if (quality < MIN_CAMERA_JPEG_QUALITY || quality > MAX_CAMERA_JPEG_QUALITY) {
    error = "invalid_quality";
    return false;
  }

  camera_config_t config{};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = profile->frameSize;
  config.jpeg_quality = quality;
  config.fb_count = psramFound() ? 2 : 1;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    error = "camera_init_failed_0x" + String(static_cast<uint32_t>(err), HEX);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    error = "camera_sensor_unavailable";
    return false;
  }
  if (sensor->set_framesize(sensor, profile->frameSize) != 0) {
    error = "set_framesize_failed";
    return false;
  }
  if (sensor->set_quality(sensor, quality) != 0) {
    error = "set_quality_failed";
    return false;
  }

  currentFrameProfile = profile;
  currentJpegQuality = quality;
  return true;
}

bool recoverCamera(const char* reason) {
  const uint32_t startedAt = millis();
  cameraStats.cameraRecoveries++;
  cameraStats.lastRecoveryMs = startedAt;
  Serial.printf("Camera recovery requested: %s\n", reason);

  esp_camera_deinit();
  delay(CAMERA_RECOVERY_SETTLE_MS);

  String error;
  const bool ok = configureCamera(currentFrameProfile, currentJpegQuality, error);
  cameraStats.lastRecoveryDurationMs = millis() - startedAt;
  cameraStats.lastRecoveryOk = ok;
  if (ok) {
    cameraStats.consecutiveCaptureFailures = 0;
    lastCameraError = "";
    Serial.printf("Camera recovery ok in %lu ms\n",
                  static_cast<unsigned long>(cameraStats.lastRecoveryDurationMs));
  } else {
    lastCameraError = error;
    Serial.printf("Camera recovery failed: %s\n", error.c_str());
  }
  return ok;
}

void appendCameraProfileJson(String& json) {
  json += "\"frameSize\":\"";
  json += currentFrameProfile->name;
  json += "\",\"frameWidth\":";
  json += currentFrameProfile->width;
  json += ",\"frameHeight\":";
  json += currentFrameProfile->height;
  json += ",\"jpegQuality\":";
  json += currentJpegQuality;
}

void sendCameraProfileJson(int statusCode, const char* status, const String& message = "") {
  String json = "{";
  json += "\"status\":\"";
  json += status;
  json += "\",";
  appendCameraProfileJson(json);
  json += ",\"qualityRange\":{\"min\":";
  json += MIN_CAMERA_JPEG_QUALITY;
  json += ",\"max\":";
  json += MAX_CAMERA_JPEG_QUALITY;
  json += "},\"allowedFrameSizes\":";
  json += allowedCameraProfilesJson();
  if (message.length() > 0) {
    json += ",\"message\":\"";
    json += message;
    json += "\"";
  }
  json += "}";
  server.sendHeader("Connection", "close");
  server.send(statusCode, "application/json", json);
  server.client().stop();
}

struct WifiConfig {
  char ssid[33];
  char password[65];
};

void drainLineEndings() {
  delay(2);
  while (Serial.available() > 0) {
    int c = Serial.peek();
    if (c != '\n' && c != '\r') {
      return;
    }
    Serial.read();
  }
}

bool readLine(const char* prompt, char* buffer, size_t bufferSize, bool allowEmpty) {
  while (true) {
    Serial.print(prompt);
    String line;
    while (true) {
      while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
          goto line_complete;
        }
        line += c;
      }
      delay(10);
    }

line_complete:
    drainLineEndings();
    line.trim();
    if (line.length() == 0 && !allowEmpty) {
      Serial.println("Value required.");
      continue;
    }
    if (line.length() >= bufferSize) {
      Serial.printf("Too long. Max %u characters.\n", static_cast<unsigned>(bufferSize - 1));
      continue;
    }
    strlcpy(buffer, line.c_str(), bufferSize);
    return true;
  }
}

bool loadWifiConfig(WifiConfig& config) {
  Preferences prefs;
  if (!prefs.begin("mb_cam_wifi", true)) {
    return false;
  }
  String ssid = prefs.getString("ssid", "");
  String password = prefs.getString("password", "");
  prefs.end();

  ssid.trim();
  if (ssid.length() == 0 || ssid.length() >= sizeof(config.ssid)) {
    return false;
  }
  strlcpy(config.ssid, ssid.c_str(), sizeof(config.ssid));
  strlcpy(config.password, password.c_str(), sizeof(config.password));
  return true;
}

bool saveWifiConfig(const WifiConfig& config) {
  Preferences prefs;
  if (!prefs.begin("mb_cam_wifi", false)) {
    return false;
  }
  bool ok = prefs.putString("ssid", config.ssid) > 0;
  if (config.password[0] != '\0') {
    ok = prefs.putString("password", config.password) > 0 && ok;
  } else {
    prefs.remove("password");
  }
  prefs.end();
  return ok;
}

void clearWifiConfig() {
  Preferences prefs;
  if (prefs.begin("mb_cam_wifi", false)) {
    prefs.clear();
    prefs.end();
  }
  Serial.println("ESP32-CAM Wi-Fi config cleared.");
}

void clearRequestedOnBoot(uint32_t timeoutMs = 3000) {
  Serial.printf("Type CLEAR within %lu ms to erase stored ESP32-CAM Wi-Fi config, or wait to continue.\n",
                static_cast<unsigned long>(timeoutMs));
  const uint32_t startedAt = millis();
  String line;
  while (millis() - startedAt < timeoutMs) {
    while (Serial.available() > 0) {
      char c = static_cast<char>(Serial.read());
      if (c == '\n' || c == '\r') {
        drainLineEndings();
        line.trim();
        if (line == "CLEAR") {
          clearWifiConfig();
        }
        return;
      }
      line += c;
    }
    delay(10);
  }
}

bool promptWifiConfig(WifiConfig& config) {
  if (loadWifiConfig(config)) {
    Serial.println("Loaded stored ESP32-CAM Wi-Fi config.");
    return true;
  }

  Serial.println();
  Serial.println("=== MotionBrain ESP32-CAM Wi-Fi Provisioning ===");
  Serial.println("No stored Wi-Fi config found.");
  Serial.println("Enter values here; they will be saved to ESP32 NVS flash, not to project files.");
  readLine("Wi-Fi SSID: ", config.ssid, sizeof(config.ssid), false);
  readLine("Wi-Fi password: ", config.password, sizeof(config.password), true);
  if (!saveWifiConfig(config)) {
    Serial.println("Failed to save ESP32-CAM Wi-Fi config.");
    return false;
  }
  Serial.println("ESP32-CAM Wi-Fi config saved to NVS.");
  return true;
}

bool initCamera() {
  String error;
  const bool ok = configureCamera(&CAMERA_PROFILE_OPTIONS[0], DEFAULT_CAMERA_JPEG_QUALITY, error);
  if (!ok) {
    Serial.printf("Camera init failed: %s\n", error.c_str());
    lastCameraError = error;
    return false;
  }
  camera_fb_t* warmup = esp_camera_fb_get();
  if (warmup != nullptr) {
    esp_camera_fb_return(warmup);
  } else {
    cameraStats.captureFailures++;
    cameraStats.consecutiveCaptureFailures++;
    lastCameraError = "warmup_capture_failed";
  }
  return true;
}

void handleRoot() {
  server.sendHeader("Connection", "close");
  server.send(
      200,
      "text/html",
      "<!doctype html><html><head><title>MotionBrain ESP32-CAM</title></head>"
      "<body><h1>MotionBrain ESP32-CAM</h1>"
      "<p><a href=\"/capture\">capture</a> | <a href=\"/stream\">stream</a> | <a href=\"/status\">status</a> | <a href=\"/camera\">camera profile</a></p>"
      "<img src=\"/capture\" style=\"max-width:100%;height:auto\">"
      "</body></html>");
  server.client().stop();
}

void handleStatus() {
  String json = "{";
  json += "\"node\":\"esp32cam\",";
  json += "\"hostname\":\"" + String(MOTIONBRAIN_CAMERA_HOSTNAME) + "\",";
  json += "\"wifi\":\"configured\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"mdns\":\"http://" + String(MOTIONBRAIN_CAMERA_HOSTNAME) + ".local\",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"heapFree\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"psram\":" + String(psramFound() ? "true" : "false") + ",";
  appendCameraProfileJson(json);
  json += ",";
  json += "\"captures\":" + String(cameraStats.captures) + ",";
  json += "\"captureFailures\":" + String(cameraStats.captureFailures) + ",";
  json += "\"consecutiveCaptureFailures\":" + String(cameraStats.consecutiveCaptureFailures) + ",";
  json += "\"clientWriteFailures\":" + String(cameraStats.clientWriteFailures) + ",";
  json += "\"cameraRecoveries\":" + String(cameraStats.cameraRecoveries) + ",";
  json += "\"lastRecoveryMs\":" + String(cameraStats.lastRecoveryMs) + ",";
  json += "\"lastRecoveryDurationMs\":" + String(cameraStats.lastRecoveryDurationMs) + ",";
  json += "\"lastRecoveryOk\":" + String(cameraStats.lastRecoveryOk ? "true" : "false") + ",";
  json += "\"lastCaptureMs\":" + String(cameraStats.lastCaptureMs) + ",";
  json += "\"maxCaptureMs\":" + String(cameraStats.maxCaptureMs) + ",";
  json += "\"lastFrameBytes\":" + String(cameraStats.lastFrameBytes) + ",";
  json += "\"lastError\":\"" + lastCameraError + "\"";
  json += "}";
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", json);
  server.client().stop();
}

void handleCameraProfile() {
  const bool wantsChange = server.hasArg("framesize") || server.hasArg("quality");
  if (!wantsChange) {
    sendCameraProfileJson(200, "ok");
    return;
  }
  if (server.method() != HTTP_POST) {
    sendCameraProfileJson(405, "error", "use POST /camera?framesize=qvga|vga|svga&quality=4..30");
    return;
  }

  const CameraProfileOption* nextProfile = currentFrameProfile;
  int nextQuality = currentJpegQuality;
  if (server.hasArg("framesize")) {
    nextProfile = findCameraProfile(server.arg("framesize"));
    if (nextProfile == nullptr) {
      sendCameraProfileJson(400, "error", "unknown_framesize");
      return;
    }
  }
  if (server.hasArg("quality") && !parseQualityArg(server.arg("quality"), nextQuality)) {
    sendCameraProfileJson(400, "error", "invalid_quality");
    return;
  }

  String error;
  if (!applyCameraProfile(nextProfile, nextQuality, error)) {
    sendCameraProfileJson(500, "error", error);
    return;
  }
  sendCameraProfileJson(200, "ok", "camera_profile_updated");
}

void handleCapture() {
  const uint32_t startedAt = millis();
  camera_fb_t* fb = esp_camera_fb_get();
  const uint32_t captureMs = millis() - startedAt;
  cameraStats.lastCaptureMs = captureMs;
  if (captureMs > cameraStats.maxCaptureMs) {
    cameraStats.maxCaptureMs = captureMs;
  }
  if (fb == nullptr) {
    cameraStats.captureFailures++;
    cameraStats.consecutiveCaptureFailures++;
    lastCameraError = "camera_capture_failed";
    recoverCamera("capture_failed");
    server.sendHeader("Connection", "close");
    server.send(503, "text/plain", "camera capture failed");
    server.client().stop();
    return;
  }
  cameraStats.captures++;
  cameraStats.consecutiveCaptureFailures = 0;
  lastCameraError = "";
  cameraStats.lastFrameBytes = fb->len;

  WiFiClient client = server.client();
  client.setTimeout(2000);
  client.setNoDelay(true);
  const bool headerOk = client.connected() &&
                        client.printf("HTTP/1.1 200 OK\r\n"
                                      "Content-Type: image/jpeg\r\n"
                                      "Content-Length: %u\r\n"
                                      "Cache-Control: no-store\r\n"
                                      "Connection: close\r\n\r\n",
                                      static_cast<unsigned>(fb->len)) > 0;
  const size_t written = headerOk && client.connected() ? client.write(fb->buf, fb->len) : 0;
  if (!headerOk || written != fb->len) {
    cameraStats.clientWriteFailures++;
  }
  esp_camera_fb_return(fb);
  client.stop();
}

void handleStream() {
  WiFiClient client = server.client();
  const uint32_t startedAt = millis();
  String response =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-cache\r\n"
      "Connection: close\r\n\r\n";
  client.print(response);

  while (client.connected() && (millis() - startedAt) < STREAM_MAX_DURATION_MS) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb == nullptr) {
      break;
    }

    bool ok = client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len) > 0;
    if (ok) {
      ok = client.write(fb->buf, fb->len) == fb->len;
    }
    if (ok) {
      ok = client.print("\r\n") > 0;
    }
    esp_camera_fb_return(fb);
    if (!ok) {
      break;
    }

    delay(STREAM_FRAME_DELAY_MS);
  }
  client.stop();
}

void connectWifi() {
  WifiConfig config{};
  if (!promptWifiConfig(config)) {
    Serial.println("No Wi-Fi config available; rebooting.");
    delay(5000);
    ESP.restart();
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(MOTIONBRAIN_CAMERA_HOSTNAME);
  WiFi.begin(config.ssid, config.password[0] != '\0' ? config.password : nullptr);
  Serial.print("Connecting to configured Wi-Fi");

  const uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startedAt) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi connect timeout; rebooting after checking credentials/network.");
    delay(5000);
    ESP.restart();
  }
  Serial.printf("ESP32-CAM IP: %s\n", WiFi.localIP().toString().c_str());
  if (MDNS.begin(MOTIONBRAIN_CAMERA_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("ESP32-CAM mDNS: http://%s.local\n", MOTIONBRAIN_CAMERA_HOSTNAME);
  } else {
    Serial.println("ESP32-CAM mDNS start failed");
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();
  Serial.println("MotionBrain ESP32-CAM boot");
  Serial.printf("Camera profile: %s JPEG quality=%d psram=%s\n",
                currentFrameProfile->name,
                currentJpegQuality,
                psramFound() ? "yes" : "no");
  clearRequestedOnBoot();

  if (!initCamera()) {
    Serial.println("Camera unavailable; reboot after checking power and camera ribbon.");
    delay(5000);
    ESP.restart();
  }

  connectWifi();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/camera", HTTP_GET, handleCameraProfile);
  server.on("/camera", HTTP_POST, handleCameraProfile);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/stream", HTTP_GET, handleStream);
  server.begin();
  Serial.println("Camera HTTP server ready");
}

void loop() {
  server.handleClient();
}
