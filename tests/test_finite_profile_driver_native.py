"""Production motor driver and offline finite-profile bridge with fake Arduino IO.

Software PWM writes and ownership only; no hardware or real-time guarantees.
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FiniteProfileDriverNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for name in ("clang++", "g++", "c++")
                         if (p := shutil.which(name))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; driver bridge was not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-driver-")
        cls.addClassCleanup(cls.temp.cleanup)
        sdk_flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                                 check=True, capture_output=True, text=True, timeout=30).stdout.strip()
            sdk_flags = ["-isysroot", sdk]
        cls.command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags,
                       "-I", str(ROOT / "tests/native/stubs"), "-I", str(ROOT / "src")]
        cls.sources = [str(ROOT / p) for p in (
            "tests/native/test_finite_profile_driver.cpp",
            "src/motor/motor_driver.cpp",
            "src/control/finite_profile_contract.cpp",
            "src/control/finite_profile_executor.cpp",
            "src/control/finite_profile_driver_bridge.cpp")]
        cls.binaries = {}
        for mode, flags in (
            ("default", []),
            ("core", ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1"]),
            ("driver", ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1",
                        "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER=1"]),
        ):
            binary = Path(cls.temp.name) / mode
            result = subprocess.run([*cls.command, *flags, *cls.sources, "-o", str(binary)],
                                    capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(f"Driver build ({mode}) failed:\n{result.stderr}")
            cls.binaries[mode] = binary

    def run_case(self, name, mode="driver"):
        result = subprocess.run([str(self.binaries[mode]), name],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_disabled_preserves_legacy_ramp(self):
        self.run_case("disabled", "default")

    def test_core_only_preserves_legacy_ramp(self):
        self.run_case("disabled", "core")

    def test_driver_without_core_is_compile_error(self):
        result = subprocess.run([*self.command, "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER=1",
                                 "-c", str(ROOT / "src/motor/motor_driver.cpp"),
                                 "-o", str(Path(self.temp.name) / "invalid.o")],
                                capture_output=True, text=True, timeout=60)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE", result.stderr)

    def test_direct_pwm_has_no_second_ramp(self):
        self.run_case("direct")

    def test_legacy_entry_points_and_ordinal_cannot_interfere(self):
        self.run_case("legacy")

    def test_stale_lease_cannot_write_stop_or_release_new_owner(self):
        self.run_case("stale")

    def test_pending_legacy_output_prevents_lease_acquisition(self):
        self.run_case("ready")

    def test_stop_paths_seal_lease_across_rearm(self):
        self.run_case("stop")

    def test_disarm_fault_and_monitor_trip_seal_active_lease(self):
        self.run_case("safety")

    def test_driver_ring_trace_matches_actual_pwm_writes(self):
        self.run_case("ring")

    def test_bridge_completion_holds_quiet_lease_until_release(self):
        self.run_case("bridge_nominal")

    def test_old_command_callbacks_cannot_disturb_new_driver_owner(self):
        self.run_case("bridge_old")

    def test_external_stop_cancels_core_and_marks_trace_incomplete(self):
        self.run_case("bridge_stop")

    def test_external_stop_before_execute_prevents_old_execution(self):
        self.run_case("bridge_reserved")

    def test_bridge_rejects_busy_controllers_and_foreign_lease(self):
        self.run_case("bridge_admission")

    def test_bridge_destruction_stops_and_keeps_lease_sealed(self):
        self.run_case("bridge_destroyed")

    def test_old_bridge_cannot_stop_replacement_owner_even_on_destruction(self):
        self.run_case("bridge_replaced")

    def test_cancel_records_motor2_off_before_all_motor_confirmation(self):
        self.run_case("bridge_cancel_time")


if __name__ == "__main__":
    unittest.main()
