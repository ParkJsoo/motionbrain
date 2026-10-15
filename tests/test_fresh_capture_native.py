"""Run the production ESP32-CAM fresh-frame selector without a camera or network."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FreshCaptureNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            raise unittest.SkipTest("native C++ compiler unavailable")
        tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(tmp.cleanup)
        cls.binary = Path(tmp.name) / "fresh_capture"
        flags = []
        if sys.platform == "darwin":
            sdk = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
            flags = ["-isysroot", sdk]
        result = subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *flags,
            "-I", str(ROOT / "firmware/esp32cam/src"),
            str(ROOT / "tests/native/test_fresh_capture.cpp"), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise AssertionError(result.stderr)

    def test_selection_ownership_and_virtual_time(self):
        for case in ("request_mode", "old_then_new", "equal_repeated_limit", "null", "invalid_timestamp",
                     "deadline", "clock", "disconnect", "boot_and_headers"):
            with self.subTest(case=case):
                subprocess.run([str(self.binary), case], check=True, capture_output=True, timeout=10)

    def test_wire_contract_roundtrip(self):
        from learning.fresh_capture import validate_fresh_capture
        from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers
        raw = subprocess.check_output([str(self.binary), "boot_and_headers"], text=True)
        headers = dict(line.split(": ", 1) for line in raw.splitlines())
        source = parse_capture_headers(headers)
        proof = validate_fresh_capture({"headers":headers, "query":{"fresh":"1"}}, source)
        self.assertEqual(proof["requestedUs"], 1000000)
        self.assertEqual(proof["discardedFrames"], 1)
        self.assertEqual(source["capturedUs"], 1000001)
