"""Production lease transaction and motor driver; fake time/GPIO, no device IO."""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ManualControlLeaseNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next(
            (
                shutil.which(name)
                for name in ("clang++", "g++", "c++")
                if shutil.which(name)
            ),
            None,
        )
        if not compiler:
            raise unittest.SkipTest("C++ compiler unavailable")
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        cls.binary = Path(cls.temp.name) / "manual_control_lease"
        sdk = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = [
                "-isysroot",
                subprocess.check_output(
                    ["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True
                ).strip(),
            ]
        result = subprocess.run(
            [
                compiler,
                "-std=c++11",
                "-Wall",
                "-Wextra",
                "-Werror",
                *sdk,
                "-I",
                str(ROOT / "tests/native/stubs"),
                "-I",
                str(ROOT / "src"),
                str(ROOT / "src/network/manual_control_lease.cpp"),
                str(ROOT / "src/motor/motor_driver.cpp"),
                str(ROOT / "tests/native/test_manual_control_lease.cpp"),
                "-o",
                str(cls.binary),
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def run_case(self, case):
        result = subprocess.run(
            [str(self.binary), case], capture_output=True, text=True, timeout=10
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def test_user04_exact_timestamps_90ms_pulse_becomes_no_reactivation(self):
        import json

        result = self.run_case("user04")
        self.assertEqual(
            json.loads(result.stdout),
            {"legacyNonzeroWidthMs": 90, "leasedNonzeroWidthMs": 0},
        )

    def test_ttl_checked_inside_handler_before_dispatch_and_clock_wrap(self):
        self.run_case("deadline")
        self.run_case("wrap")

    def test_stop_cancels_delayed_start_replay_and_prior_boot(self):
        self.run_case("start")

    def test_physical_direction_alias_and_monotonic_sequences(self):
        self.run_case("direction")

    def test_serial_stop_disarm_and_foreign_target_invalidate_without_stopping_fresh_owner(
        self,
    ):
        self.run_case("external")

    def test_malformed_legacy_and_failed_start_are_not_replayed(self):
        self.run_case("malformed")


if __name__ == "__main__":
    unittest.main()
