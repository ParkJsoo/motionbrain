import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ESP32CAM_MAIN = REPO_ROOT / "firmware" / "esp32cam" / "src" / "main.cpp"


class Esp32CamFirmwareContractTest(unittest.TestCase):
    def test_status_exposes_capture_recovery_diagnostics(self):
        source = ESP32CAM_MAIN.read_text()

        expected_fragments = [
            "consecutiveCaptureFailures",
            "cameraRecoveries",
            "lastRecoveryMs",
            "lastRecoveryDurationMs",
            "lastRecoveryOk",
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


if __name__ == "__main__":
    unittest.main()
