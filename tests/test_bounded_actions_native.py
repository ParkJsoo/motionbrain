"""Run production MCU action logic with virtual time and fake output ports.

This compiles ordinary C++11, without Arduino, a board, firmware upload, or
electrical feedback. Fake output writes are software observations only.
"""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class BoundedActionsNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next(
            (path for name in ("clang++", "g++", "c++") if (path := shutil.which(name))), None
        )
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; MCU action core was not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-bounded-actions-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.binary = Path(cls.temp.name) / "bounded_actions"
        flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(
                ["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                check=True, capture_output=True, text=True, timeout=30,
            ).stdout.strip()
            flags = ["-isysroot", sdk]
        result = subprocess.run(
            [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *flags,
             "-I", str(ROOT / "src"), str(ROOT / "tests/native/test_bounded_actions.cpp"),
             str(ROOT / "src/control/bounded_action_executor.cpp"), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=60,
        )
        if result.returncode:
            raise AssertionError(f"Native MCU core compilation failed:\n{result.stderr}")

    def run_case(self, name):
        result = subprocess.run(
            [str(self.binary), name], capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_boot_identity_duplicate_conflict_and_busy(self):
        self.run_case("identity")

    def test_validation_rejects_before_output(self):
        self.run_case("validation")

    def test_registry_capacity_preserves_tombstones(self):
        self.run_case("capacity")

    def test_deadline_overrun_and_clock_wrap(self):
        self.run_case("deadline")

    def test_stop_failure_latches_unknown_and_retries_off(self):
        self.run_case("unknown")

    def test_m4_completion_failure_and_deadline(self):
        self.run_case("m4")

    def test_start_failure_safety_and_manual_cancel(self):
        self.run_case("cancel")

    def test_output_log_ramp_context_filtering_and_force_off(self):
        self.run_case("output")

    def test_output_ring_reports_dropped_history(self):
        self.run_case("overflow")

    def test_numeric_parser_rejects_malformed_nonfinite_and_overflow(self):
        self.run_case("numeric")


if __name__ == "__main__":
    unittest.main()
