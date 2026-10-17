"""Run the production idle-client adapter against a minimal transport double."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class IdleConnectionWebServerNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            raise unittest.SkipTest("native C++ compiler unavailable")
        tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(tmp.cleanup)
        cls.binary = Path(tmp.name) / "idle_connection_web_server"
        flags = []
        if sys.platform == "darwin":
            sdk = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
            flags = ["-isysroot", sdk]
        result = subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *flags,
            "-I", str(ROOT / "tests/native/network_stubs"),
            "-I", str(ROOT / "tests/native/stubs"),
            "-I", str(ROOT / "src"),
            str(ROOT / "tests/native/test_idle_connection_web_server.cpp"), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise AssertionError(result.stderr)

    def test_empty_clients_do_not_starve_queued_requests(self):
        subprocess.run([str(self.binary)], check=True, capture_output=True, timeout=10)

if __name__ == "__main__": unittest.main()
