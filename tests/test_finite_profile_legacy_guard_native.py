"""Legacy ownership gates with the real command queue and motor driver; no devices."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class FiniteProfileLegacyGuardNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for n in ("clang++", "g++", "c++") if (p := shutil.which(n))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-guard-")
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
                       "-I", str(ROOT / "tests/native/stubs"), "-I", str(ROOT / "src")]
            command += [str(ROOT / p) for p in (
                "tests/native/test_finite_profile_legacy_guard.cpp", "src/control/finite_profile_legacy_guard.cpp",
                "src/control/command_bus.cpp", "src/motor/motor_driver.cpp")]
            result = subprocess.run([*command, "-o", str(binary)], capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(result.stderr)
            cls.binaries[enabled] = binary

    def run_case(self, case, enabled=True):
        result = subprocess.run([str(self.binaries[enabled]), case], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_disabled_preserves_commands_and_queue(self):
        self.run_case("disabled", False)

    def test_competing_commands_seal_while_readonly_commands_preserve_owner(self):
        self.run_case("movement")

    def test_stop_commands_are_allowed_and_keep_owner_sealed(self):
        self.run_case("stop")

    def test_rejected_queue_intent_cannot_return_to_idle_and_revive_owner(self):
        self.run_case("queue")

    def test_full_queue_cannot_prevent_stop_invalidation(self):
        self.run_case("full")

if __name__ == "__main__":
    unittest.main()
