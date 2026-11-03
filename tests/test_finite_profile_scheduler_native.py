"""Deterministic scheduler tickets, core timing, and actual driver-write evidence.

Fake clock and timer transport only; no firmware activation or real-time claims.
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class FiniteProfileSchedulerNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for n in ("clang++", "g++", "c++") if (p := shutil.which(n))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-scheduler-")
        cls.addClassCleanup(cls.temp.cleanup)
        sdk_flags = []
        if sys.platform == "darwin":
            sdk = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
            sdk_flags = ["-isysroot", sdk]
        cls.compile_base = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags,
                            "-I", str(ROOT / "tests/native/stubs"), "-I", str(ROOT / "src")]
        cls.binaries = {}
        for enabled in (False, True):
            binary = Path(cls.temp.name) / ("enabled" if enabled else "disabled")
            flags = ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1", "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER=1", "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER=1"] if enabled else []
            command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags, *flags,
                       "-I", str(ROOT / "tests/native/stubs"), "-I", str(ROOT / "src")]
            command += [str(ROOT / p) for p in (
                "tests/native/test_finite_profile_scheduler.cpp", "src/control/finite_profile_scheduler.cpp",
                "src/control/finite_profile_contract.cpp", "src/control/finite_profile_executor.cpp",
                "src/control/finite_profile_driver_bridge.cpp", "src/motor/motor_driver.cpp")]
            result = subprocess.run([*command, "-o", str(binary)], capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(result.stderr)
            cls.binaries[enabled] = binary

    def run_case(self, case, enabled=True):
        result = subprocess.run([str(self.binaries[enabled]), case], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_scheduler_requires_enabled_driver(self):
        result = subprocess.run([*self.compile_base, "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1",
                                 "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER=1",
                                 "-c", str(ROOT / "src/control/finite_profile_scheduler.cpp"),
                                 "-o", str(Path(self.temp.name) / "invalid.o")],
                                capture_output=True, text=True, timeout=60)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("scheduler requires MOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER", result.stderr)

    def test_default_disabled(self):
        self.run_case("disabled", False)

    def test_schedule_preserves_absolute_times(self):
        self.run_case("nominal")

    def test_early_and_duplicate_delivery(self):
        self.run_case("early")

    def test_late_or_missing_callback_never_catches_up(self):
        self.run_case("delay")

    def test_failure_after_execution_acceptance(self):
        self.run_case("registration")

    def test_registration_failure_after_first_output(self):
        self.run_case("later")

    def test_cancel_then_old_callback(self):
        self.run_case("cancel")

    def test_clock_wrap(self):
        self.run_case("wrap")

    def test_registration_latency_does_not_shift_origin(self):
        self.run_case("registration_delay")

    def test_old_wrapper_ticket_cannot_cancel_new_timer(self):
        self.run_case("recreated")

    def test_cancel_stops_before_slow_timer_cleanup(self):
        self.run_case("cancel_cleanup")

    def test_registration_failure_stops_before_slow_timer_cleanup(self):
        self.run_case("failure_cleanup")

    def test_destruction_stops_before_slow_timer_cleanup(self):
        self.run_case("destructor_cleanup")

    def test_destruction_retains_reserved_before_first_and_quiet_ownership(self):
        self.run_case("destructor_ownership")

    def test_delivery_copies_ticket_before_port_clears_aliased_storage(self):
        self.run_case("ticket_alias")

    def test_early_expiry_wake_returns_expired_when_reregistration_crosses_deadline(self):
        self.run_case("early_expiry")

if __name__ == "__main__":
    unittest.main()
