"""Maintenance-only polling and schedule-failure state transitions, no hardware."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FiniteProfileMaintenanceNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for name in ("clang++", "g++", "c++")
                         if (p := shutil.which(name))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; core maintenance was not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-maintenance-")
        cls.addClassCleanup(cls.temp.cleanup)
        sdk_flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                                 check=True, capture_output=True, text=True, timeout=30).stdout.strip()
            sdk_flags = ["-isysroot", sdk]
        cls.command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags,
                       "-I", str(ROOT / "src")]
        cls.sources = [str(ROOT / p) for p in (
            "tests/native/test_finite_profile_maintenance.cpp",
            "src/control/finite_profile_contract.cpp", "src/control/finite_profile_executor.cpp")]
        cls.binaries = {}
        for mode, flags in (("default", []), ("core", ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1"])):
            binary = Path(cls.temp.name) / mode
            result = subprocess.run([*cls.command, *flags, *cls.sources, "-o", str(binary)],
                                    capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(f"Maintenance build ({mode}) failed:\n{result.stderr}")
            cls.binaries[mode] = binary

    def run_case(self, name, mode="core"):
        result = subprocess.run([str(self.binaries[mode]), name],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_disabled(self): self.run_case("disabled", "default")
    def test_poll_does_not_dispatch_but_fails_after_lateness_limit(self): self.run_case("deadline")
    def test_poll_safety_loss_before_and_after_first_output(self): self.run_case("safety")
    def test_poll_expires_reserved_token(self): self.run_case("expiry")
    def test_schedule_failure_consumes_reserved_token(self): self.run_case("reserved")
    def test_schedule_failure_after_output_preserves_quiet(self): self.run_case("running")
    def test_wrong_identity_or_old_terminal_cannot_affect_current_owner(self): self.run_case("identity")
    def test_pending_off_schedule_failure_latches_unknown_and_preserves_decision(self): self.run_case("pending_failure")
    def test_poll_can_complete_pending_off_proof_without_output(self): self.run_case("pending_confirmation")
    def test_pending_off_deadline_is_bounded(self): self.run_case("pending_deadline")
    def test_wrap_for_poll_expiry_and_pending_off(self): self.run_case("wrap")


if __name__ == "__main__":
    unittest.main()
