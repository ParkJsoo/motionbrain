"""Run the production Wi-Fi power diagnostics against a minimal transport double."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class WiFiLatencyNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            raise unittest.SkipTest("native C++ compiler unavailable")
        tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(tmp.cleanup)
        cls.binary = Path(tmp.name) / "wifi_latency"
        flags = []
        if sys.platform == "darwin":
            sdk = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
            flags = ["-isysroot", sdk]
        result = subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *flags,
            "-I", str(ROOT / "tests/native/wifi_stubs"),
            "-I", str(ROOT / "tests/native/stubs"),
            "-I", str(ROOT / "src"),
            str(ROOT / "tests/native/test_wifi_latency.cpp"), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise AssertionError(result.stderr)

    def test_driver_readback_failure_and_already_applied_mode(self):
        subprocess.run([str(self.binary)], check=True, capture_output=True, timeout=10)

    def test_station_init_and_reconnect_apply_preference_before_connect(self):
        source = (ROOT / "src/network/wifi_ap.cpp").read_text()
        init = source.split("bool WiFiAP::initStation", 1)[1].split("void WiFiAP::update", 1)[0]
        reconnect = source.split("void WiFiAP::checkStation", 1)[1]
        for path in (init, reconnect):
            self.assertLess(path.index("WiFiLatency::disableStationSleep()"), path.index("WiFi.begin("))
        self.assertLess(init.index("WiFi.mode(WIFI_STA)"), init.index("WiFiLatency::disableStationSleep()"))
        status = (ROOT / "src/network/web_server.cpp").read_text().split("void MotionBrainWebServer::handleStatus()", 1)[1].split("void MotionBrainWebServer::", 1)[0]
        self.assertIn("WiFiLatency::stationPowerSave()", status)

if __name__ == "__main__": unittest.main()
