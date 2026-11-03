"""Actual UART parser/control path with batched fake serial input; no devices."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class FiniteProfileTeleopNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for n in ("clang++", "g++", "c++") if (p := shutil.which(n))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-teleop-")
        cls.addClassCleanup(cls.temp.cleanup)
        sdk_flags = []
        if sys.platform == "darwin":
            sdk = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
            sdk_flags = ["-isysroot", sdk]
        cls.binaries = {}
        for enabled in (False, True):
            binary = Path(cls.temp.name) / ("enabled" if enabled else "disabled")
            flags = ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1", "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER=1"] if enabled else []
            command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags, *flags,
                       "-I", str(ROOT / "tests/native/teleop_stubs"),
                       "-I", str(ROOT / "tests/native/stubs"), "-I", str(ROOT / "src")]
            command += [str(ROOT / p) for p in (
                "tests/native/test_finite_profile_teleop.cpp", "src/input/teleop_adapter.cpp",
                "src/control/finite_profile_legacy_guard.cpp", "src/control/command_bus.cpp", "src/motor/motor_driver.cpp",
                "src/control/finite_profile_contract.cpp", "src/control/finite_profile_executor.cpp",
                "src/control/finite_profile_driver_bridge.cpp")]
            result = subprocess.run([*command, "-o", str(binary)], capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(result.stderr)
            cls.binaries[enabled] = binary

    def run_case(self, case, enabled=True):
        result = subprocess.run([str(self.binaries[enabled]), case], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_disabled_processes_both_frames_without_output(self):
        self.run_case("disabled", False)

    def test_reserved_deadman_edge_survives_same_batch_release(self):
        self.run_case("reserved")

    def test_running_deadman_edge_survives_same_batch_release(self):
        self.run_case("running")

    def test_quiet_owner_remains_sealed_after_both_frames(self):
        self.run_case("quiet")

    def test_denied_control_does_not_pollute_output_cache(self):
        self.run_case("cache")

    def test_uart_budget_bounds_bytes_without_losing_deadman_edge(self):
        self.run_case("budget")

if __name__ == "__main__":
    unittest.main()
