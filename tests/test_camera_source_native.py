"""Run the production ESP32-CAM metadata helper without a camera or network."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CameraSourceNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            raise unittest.SkipTest("native C++ compiler unavailable")
        tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(tmp.cleanup)
        cls.binary = Path(tmp.name) / "camera_source"
        flags = []
        if sys.platform == "darwin":
            sdk = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
            flags = ["-isysroot", sdk]
        result = subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *flags,
            "-I", str(ROOT / "firmware/esp32cam/src"),
            str(ROOT / "tests/native/test_camera_source.cpp"), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise AssertionError(result.stderr)

    def test_buffer_identity_capture_age_reboot_and_invalid_timestamp(self):
        subprocess.run([str(self.binary)], check=True, capture_output=True, timeout=10)

    def test_wire_metadata_roundtrips_through_production_python_parser(self):
        bridge = ROOT / "ros2_ws/src/motionbrain_ros_bridge"
        sys.path.insert(0, str(bridge))
        from motionbrain_ros_bridge.observation import parse_capture_headers
        result = subprocess.run([str(self.binary), "headers"], check=True,
                                capture_output=True, text=True, timeout=10)
        headers = dict(line.split(": ", 1) for line in result.stdout.splitlines())
        parsed = parse_capture_headers(headers)
        self.assertEqual(parsed["capturedUs"], 12000345)
        self.assertEqual(parsed["sourceFrameId"], "0123456789abcdef0123456789abcdef:12000345")
        self.assertEqual(parsed["sourceClockDomain"], "camera_boot_us")
        self.assertTrue(parsed["sourceTimeKnown"])


if __name__ == "__main__":
    unittest.main()
