"""Exercise the finite-profile contract/core using fake ports and virtual time.

No device access, firmware upload, or physical timing/safety claims. Compile the
same sources with the feature disabled by default and explicitly enabled.
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FiniteProfileNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for name in ("clang++", "g++", "c++")
                         if (p := shutil.which(name))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; finite profile core was not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-profile-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.binaries = {}
        sdk_flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                                 check=True, capture_output=True, text=True, timeout=30).stdout.strip()
            sdk_flags = ["-isysroot", sdk]
        for enabled in (False, True):
            binary = Path(cls.temp.name) / ("enabled" if enabled else "default")
            flags = ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1"] if enabled else []
            command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags,
                       *flags, "-I", str(ROOT / "src")]
            command += [str(ROOT / p) for p in (
                "tests/native/test_finite_profile.cpp",
                "src/control/finite_profile_contract.cpp",
                "src/control/finite_profile_executor.cpp")]
            result = subprocess.run([*command, "-o", str(binary)],
                                    capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(f"Finite profile core compilation failed:\n{result.stderr}")
            cls.binaries[enabled] = binary

    def run_case(self, name, *, enabled=True):
        result = subprocess.run([str(self.binaries[enabled]), name],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout

    def test_default_disabled_compile_and_link_without_core_types(self):
        self.run_case("disabled", enabled=False)

    def test_contract_validation_and_canonical_vector(self):
        self.run_case("contract")

    def test_prepare_reserves_without_output_and_checks_boot(self):
        self.run_case("prepare")

    def test_duplicate_and_conflicting_profiles_never_repeat_output(self):
        self.run_case("identity")

    def test_cancel_before_first_output_invalidates_pending_callback(self):
        self.run_case("cancel_before_first")

    def test_unknown_cancel_blocks_delayed_initial_prepare(self):
        self.run_case("unknown_cancel")

    def test_expired_token_boundary_and_duplicate_execute(self):
        self.run_case("expiry")

    def test_registry_capacity_preserves_tombstones(self):
        self.run_case("capacity")

    def test_nominal_profile_writes_points_once_and_ends_at_zero(self):
        self.run_case("nominal")

    def test_lateness_boundary_and_actual_write_latency(self):
        self.run_case("latency")

    def test_missed_event_fails_without_catchup(self):
        self.run_case("missed")

    def test_duplicate_and_out_of_order_event_index(self):
        self.run_case("event_index")

    def test_cancel_during_output_prevents_reactivation(self):
        self.run_case("cancel_running")

    def test_late_final_off_is_attempted_and_remains_failure(self):
        self.run_case("late_off")

    def test_safety_changes_before_and_during_execution(self):
        self.run_case("safety")

    def test_stale_quiet_release_cannot_unlock_new_owner(self):
        self.run_case("quiet_release")

    def test_unknown_off_latches_at_deadline_and_never_promotes(self):
        self.run_case("unknown_off")

    def test_reserved_output_change_cannot_reuse_historical_off(self):
        self.run_case("reserved_off_changed")

    def test_backward_proof_clock_cannot_confirm_off_or_retry(self):
        self.run_case("proof_clock")

    def test_clock_wrap_preserves_schedule_and_expiry(self):
        self.run_case("wrap")


if __name__ == "__main__":
    unittest.main()
