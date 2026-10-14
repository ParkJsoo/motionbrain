"""Exercise production motor_driver.cpp with virtual clock and captured GPIO/PWM.

This proves software write attribution; it does not measure voltage or movement.
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class MotorOutputNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next(
            (path for name in ("clang++", "g++", "c++") if (path := shutil.which(name))), None
        )
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; motor driver was not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-motor-output-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.binary = Path(cls.temp.name) / "motor_output"
        flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(
                ["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                check=True, capture_output=True, text=True, timeout=30,
            ).stdout.strip()
            flags = ["-isysroot", sdk]
        result = subprocess.run(
            [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *flags,
             "-I", str(ROOT / "tests/native/stubs"), "-I", str(ROOT / "src"),
             str(ROOT / "tests/native/test_motor_output.cpp"),
             str(ROOT / "src/motor/motor_driver.cpp"), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=60,
        )
        if result.returncode:
            raise AssertionError(f"Production motor driver compilation failed:\n{result.stderr}")

    def run_case(self, name):
        result = subprocess.run([str(self.binary), name], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_ramp_signed_pwm_matches_writes_and_action_ownership(self):
        self.run_case("ramp")

    def test_hard_stop_emergency_disarm_clear_pending_output(self):
        self.run_case("force")

    def test_ring_loss_is_explicit_and_clock_wrap_keeps_ramp_interval(self):
        self.run_case("overflow")


if __name__ == "__main__":
    unittest.main()
