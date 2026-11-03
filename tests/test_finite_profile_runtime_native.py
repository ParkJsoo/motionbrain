"""Cooperative runtime with real scheduler, service admission and driver; fake clock/IO."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class FiniteProfileRuntimeNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for n in ("clang++", "g++", "c++") if (p := shutil.which(n))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-runtime-")
        cls.addClassCleanup(cls.temp.cleanup)
        sdk = []
        if sys.platform == "darwin":
            path = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
            sdk = ["-isysroot", path]
        cls.command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk]
        for path in ("tests/native/service_stubs", "tests/native/stubs", "src"):
            cls.command += ["-I", str(ROOT/path)]
        cls.binaries = {}
        sources = [str(ROOT/path) for path in (
            "tests/native/test_finite_profile_runtime.cpp", "src/control/finite_profile_runtime.cpp",
            "src/control/finite_profile_scheduler.cpp", "src/control/finite_profile_service_environment.cpp",
            "src/control/finite_profile_driver_bridge.cpp", "src/control/finite_profile_executor.cpp",
            "src/control/finite_profile_contract.cpp", "src/motor/motor_driver.cpp",
            "src/control/bounded_action_service.cpp", "src/control/bounded_action_executor.cpp",
            "src/control/command_bus.cpp", "src/control/finite_profile_legacy_guard.cpp")]
        for enabled in (False, True):
            flags = ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_"+name+"=1"
                     for name in ("CORE", "DRIVER", "SCHEDULER", "RUNTIME")] if enabled else []
            binary = Path(cls.temp.name)/("enabled" if enabled else "disabled")
            result = subprocess.run([*cls.command, *flags, *sources, "-o", str(binary)],
                                    capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(result.stderr)
            cls.binaries[enabled] = binary

    def run_case(self, name, enabled=True):
        result = subprocess.run([str(self.binaries[enabled]), name], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_disabled_has_no_runtime_or_io(self):
        self.run_case("disabled", False)

    def test_init_copies_fixed_profile_once_without_output(self):
        self.run_case("init")

    def test_loop_delivers_single_absolute_event_and_exposes_actual_evidence(self):
        self.run_case("nominal")

    def test_late_first_loop_refuses_nonzero_output(self):
        self.run_case("late")

    def test_delayed_loop_records_actual_late_off_as_failure(self):
        self.run_case("delayed_off")

    def test_external_cancel_prevents_later_loop_reactivation(self):
        self.run_case("cancel")

    def test_reservation_expires_without_execution(self):
        self.run_case("expiry")

    def test_loop_due_comparison_handles_clock_wrap(self):
        self.run_case("wrap")

    def test_real_service_admission_and_safety_are_enforced(self):
        self.run_case("admission")

    def test_allocation_failure_keeps_runtime_unavailable_without_output(self):
        self.run_case("allocation")

    def test_safety_block_at_due_stops_before_next_nonzero_event(self):
        self.run_case("due_safety")

    def test_teleop_ownership_at_due_stops_before_next_nonzero_event(self):
        self.run_case("due_teleop")

    def test_runtime_requires_scheduler(self):
        result = subprocess.run([*self.command, "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME=1",
                                 "-c", str(ROOT/"src/control/finite_profile_runtime.cpp"),
                                 "-o", str(Path(self.temp.name)/"invalid.o")],
                                capture_output=True, text=True, timeout=60)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("runtime requires MOTIONBRAIN_ENABLE_FINITE_PROFILE_SCHEDULER", result.stderr)

if __name__ == "__main__":
    unittest.main()
