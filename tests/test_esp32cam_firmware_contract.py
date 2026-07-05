import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ESP32CAM_MAIN = REPO_ROOT / "firmware" / "esp32cam" / "src" / "main.cpp"
MOTION_WEB_SERVER = REPO_ROOT / "src" / "network" / "web_server.cpp"


class Esp32CamFirmwareContractTest(unittest.TestCase):
    def test_status_exposes_capture_recovery_diagnostics(self):
        source = ESP32CAM_MAIN.read_text()

        expected_fragments = [
            "uptimeMs",
            "resetReason",
            "rootRequests",
            "statusRequests",
            "cameraProfileRequests",
            "captureRequests",
            "streamRequests",
            "consecutiveCaptureFailures",
            "slowCaptures",
            "slowClientWrites",
            "cameraRecoveries",
            "cameraRecoverySkips",
            "lastRecoveryMs",
            "lastRecoveryDurationMs",
            "lastRecoveryOk",
            "lastWriteMs",
            "maxWriteMs",
            "requestInFlight",
            "requestPath",
            "requestAgeMs",
            "httpStallRestartEnabled",
            "httpStallRestarts",
            "lastHttpStallRestartAgeMs",
            "loopStallRestarts",
            "lastLoopStallRestartAgeMs",
            "loopHeartbeatAgeMs",
            "httpStallRestartMs",
            "loopHeartbeatStallRestartMs",
            "captureSlowRecoveryMs",
            "streamMaxDurationMs",
            "lastError",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, source)

    def test_capture_failure_attempts_camera_recovery(self):
        source = ESP32CAM_MAIN.read_text()

        expected_fragments = [
            "bool recoverCamera(const char* reason)",
            "esp_camera_deinit();",
            "configureCamera(currentFrameProfile, currentJpegQuality, error)",
            'recoverCamera("capture_failed");',
            'lastCameraError = "camera_capture_failed";',
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, source)

    def test_blocked_http_request_restart_is_bounded_opt_in(self):
        source = ESP32CAM_MAIN.read_text()

        expected_fragments = [
            "#define MOTIONBRAIN_ENABLE_HTTP_STALL_RESTART 0",
            "const uint32_t HTTP_REQUEST_STALL_RESTART_MS = 6000;",
            "const uint32_t LOOP_HEARTBEAT_STALL_RESTART_MS = 9000;",
            "void httpSupervisorTask(void*)",
            "#if MOTIONBRAIN_ENABLE_HTTP_STALL_RESTART",
            "HTTP supervisor restart disabled",
            "snapshotHttpRequest(active, startedMs, name)",
            "HTTP request stalled: path=%s age=%lu ms threshold=%lu ms; restarting",
            "HTTP loop heartbeat stalled: age=%lu ms threshold=%lu ms; restarting",
            "httpStallRestartCount++;",
            "lastHttpStallRestartAgeMs = ageMs;",
            "loopStallRestartCount++;",
            "lastLoopStallRestartAgeMs = heartbeatAgeMs;",
            "ESP.restart();",
            "xTaskCreatePinnedToCore(",
            "ScopedHttpRequest request(\"root\");",
            "ScopedHttpRequest request(\"status\");",
            "ScopedHttpRequest request(\"camera\");",
            "ScopedHttpRequest request(\"capture\");",
            "ScopedHttpRequest request(\"stream\");",
            "loopHeartbeatMs = millis();",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, source)

    def test_capture_uses_bounded_chunked_writes_and_stream_is_disabled(self):
        source = ESP32CAM_MAIN.read_text()

        expected_fragments = [
            "const uint32_t CLIENT_IO_TIMEOUT_MS = 750;",
            "const uint32_t CAPTURE_WRITE_DEADLINE_MS = 2000;",
            "const size_t CLIENT_WRITE_CHUNK_BYTES = 1024;",
            "bool writeClientBuffer(",
            "client.setTimeout(CLIENT_IO_TIMEOUT_MS);",
            "writeClientBuffer(client, fb->buf, fb->len, CAPTURE_WRITE_DEADLINE_MS, written)",
            "stream disabled; use /capture",
            "cameraStats.clientWriteFailures++;",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, source)

    def test_stream_endpoint_is_disabled_for_single_threaded_webserver(self):
        source = ESP32CAM_MAIN.read_text()

        self.assertIn("const uint32_t STREAM_MAX_DURATION_MS = 0;", source)
        self.assertIn('server.send(410, "text/plain", "stream disabled; use /capture");', source)
        self.assertNotIn('<a href=\\"/stream\\">stream</a>', source)

    def test_controller_camera_ui_avoids_disabled_stream_endpoint(self):
        source = MOTION_WEB_SERVER.read_text()

        expected_fragments = [
            "cameraMode = 'tracked'",
            "LIVE CAPTURE",
            "cameraMode = 'capture'",
            "img.src = cameraPath('/capture?t=' + Date.now())",
            "startTrackedCamera(false)",
        ]
        for fragment in expected_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, source)

        forbidden_fragments = [
            "RAW STREAM",
            "cameraMode = 'stream'",
            "cameraPath('/stream",
            ">STREAM</button>",
        ]
        for fragment in forbidden_fragments:
            with self.subTest(fragment=fragment):
                self.assertNotIn(fragment, source)


if __name__ == "__main__":
    unittest.main()
