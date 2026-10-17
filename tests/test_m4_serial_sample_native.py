"""Run the production read-only serial frame against a minimal transport double."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class M4SerialSampleNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            raise unittest.SkipTest("native C++ compiler unavailable")
        tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(tmp.cleanup)
        cls.binary = Path(tmp.name) / "m4_serial_sample"
        flags = []
        if sys.platform == "darwin":
            sdk = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
            flags = ["-isysroot", sdk]
        result = subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *flags,
            "-I", str(ROOT / "tests/native/network_stubs"),
            "-I", str(ROOT / "tests/native/stubs"),
            "-I", str(ROOT / "src"),
            str(ROOT / "tests/native/test_m4_serial_sample.cpp"), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise AssertionError(result.stderr)

    def test_read_only_framing_and_bounded_response(self):
        subprocess.run([str(self.binary)], check=True, capture_output=True, timeout=10)

    def test_frame_consumption_precedes_legacy_command_termination(self):
        source = (ROOT / "src/input/serial_command.cpp").read_text()
        receiver = source.split("void SerialCommand::processSerialInput()", 1)[1].split("void SerialCommand::sendM4Sample", 1)[0]
        self.assertLess(receiver.index("m4SampleFrame_.consume(c)"), receiver.index("if (c == '\\n'"))
        handler = source.split("void SerialCommand::sendM4Sample", 1)[1].split("void SerialCommand::processCommand", 1)[0]
        for mutation in ("submitCommand", "transitionTo", "setSpeed", "update(", "dispatch"):
            self.assertNotIn(mutation, handler)

if __name__ == "__main__": unittest.main()
