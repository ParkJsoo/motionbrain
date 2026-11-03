"""Real state/safety/driver transition tests with fake Arduino IO, no hardware."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FiniteProfileSafetyNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for name in ("clang++", "g++", "c++")
                         if (p := shutil.which(name))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; safety boundary was not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-safety-")
        cls.addClassCleanup(cls.temp.cleanup)
        sdk_flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                                 check=True, capture_output=True, text=True, timeout=30).stdout.strip()
            sdk_flags = ["-isysroot", sdk]
        command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags,
                   "-I", str(ROOT / "tests/native/finite_safety_stubs"),
                   "-I", str(ROOT / "src")]
        sources = [str(ROOT / p) for p in (
            "tests/native/test_finite_profile_safety.cpp",
            "src/motor/motor_driver.cpp", "src/system/system_init.cpp",
            "src/safety/safety_monitor.cpp")]
        cls.binaries = {}
        for mode, flags in (
            ("default", []),
            ("core", ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1"]),
            ("driver", ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1",
                        "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER=1"]),
        ):
            binary = Path(cls.temp.name) / mode
            result = subprocess.run([*command, *flags, *sources, "-o", str(binary)],
                                    capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(f"Safety build ({mode}) failed:\n{result.stderr}")
            cls.binaries[mode] = binary

    def run_case(self, name, mode="driver"):
        result = subprocess.run([str(self.binaries[mode]), name],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_disabled_keeps_legacy_stop_timing(self):
        self.run_case("legacy", "default")

    def test_core_only_keeps_legacy_stop_timing(self):
        self.run_case("legacy", "core")

    def test_enabled_unowned_keeps_legacy_stop_timing(self):
        self.run_case("legacy")

    def test_reserved_disarm_rearm_without_tick_revokes_old_lease(self):
        self.run_case("reserved")

    def test_running_disarm_stops_before_state_change(self):
        self.run_case("running")

    def test_fault_recovery_cannot_revive_old_lease(self):
        self.run_case("fault")

    def test_timeout_immediately_stops_owned_output(self):
        self.run_case("timeout")

    def test_timeout_across_clock_wrap(self):
        self.run_case("wrap")

    def test_block_clear_rearm_without_driver_tick(self):
        self.run_case("block_clear")

    def test_vibration_fault_clear_rearm_without_driver_tick(self):
        self.run_case("vibration")

    def test_rejected_and_same_state_transitions_do_not_revoke(self):
        self.run_case("invalid_same")

    def test_unrelated_state_manager_does_not_revoke_global_lease(self):
        self.run_case("foreign")


if __name__ == "__main__":
    unittest.main()
