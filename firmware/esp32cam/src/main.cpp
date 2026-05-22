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
const framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_QVGA;
const int CAMERA_JPEG_QUALITY = 15;

WebServer server(80);

struct CameraStats {
  uint32_t captures = 0;
  uint32_t captureFailures = 0;
  uint32_t clientWriteFailures = 0;
  uint32_t lastCaptureMs = 0;
  uint32_t maxCaptureMs = 0;
  uint32_t lastFrameBytes = 0;
};

CameraStats cameraStats;

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
  config.frame_size = CAMERA_FRAME_SIZE;
  config.jpeg_quality = CAMERA_JPEG_QUALITY;
  config.fb_count = psramFound() ? 2 : 1;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    sensor->set_framesize(sensor, CAMERA_FRAME_SIZE);
    sensor->set_quality(sensor, CAMERA_JPEG_QUALITY);
  }

  camera_fb_t* warmup = esp_camera_fb_get();
  if (warmup != nullptr) {
    esp_camera_fb_return(warmup);
  }
  return true;
}

void handleRoot() {
  server.send(
      200,
      "text/html",
      "<!doctype html><html><head><title>MotionBrain ESP32-CAM</title></head>"
      "<body><h1>MotionBrain ESP32-CAM</h1>"
      "<p><a href=\"/capture\">capture</a> | <a href=\"/stream\">stream</a> | <a href=\"/status\">status</a></p>"
      "<img src=\"/capture\" style=\"max-width:100%;height:auto\">"
      "</body></html>");
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
  json += "\"captures\":" + String(cameraStats.captures) + ",";
  json += "\"captureFailures\":" + String(cameraStats.captureFailures) + ",";
  json += "\"clientWriteFailures\":" + String(cameraStats.clientWriteFailures) + ",";
  json += "\"lastCaptureMs\":" + String(cameraStats.lastCaptureMs) + ",";
  json += "\"maxCaptureMs\":" + String(cameraStats.maxCaptureMs) + ",";
  json += "\"lastFrameBytes\":" + String(cameraStats.lastFrameBytes);
  json += "}";
  server.send(200, "application/json", json);
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
    server.send(503, "text/plain", "camera capture failed");
    return;
  }
  cameraStats.captures++;
  cameraStats.lastFrameBytes = fb->len;

  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  client.setTimeout(2000);
  client.setNoDelay(true);
  const size_t written = client.connected() ? client.write(fb->buf, fb->len) : 0;
  if (written != fb->len) {
    cameraStats.clientWriteFailures++;
  }
  esp_camera_fb_return(fb);
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
  Serial.printf("Camera profile: QVGA JPEG quality=%d psram=%s\n", CAMERA_JPEG_QUALITY, psramFound() ? "yes" : "no");
  clearRequestedOnBoot();

  if (!initCamera()) {
    Serial.println("Camera unavailable; reboot after checking power and camera ribbon.");
    delay(5000);
    ESP.restart();
  }

  connectWifi();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/stream", HTTP_GET, handleStream);
  server.begin();
  Serial.println("Camera HTTP server ready");
}

void loop() {
  server.handleClient();
}
