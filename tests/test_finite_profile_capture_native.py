"""Command-specific actual writes with the real bridge and motor driver."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FiniteProfileCaptureNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for n in ("clang++", "g++", "c++") if (p := shutil.which(n))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; capture integration not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-capture-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.binary = Path(cls.temp.name) / "capture"
        sdk_flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                                 check=True, capture_output=True, text=True, timeout=30).stdout.strip()
            sdk_flags = ["-isysroot", sdk]
        cls.command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags,
            "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1", "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER=1",
            ]
        for p in ("tests/native/stubs", "src"):
            cls.command += ["-I", str(ROOT / p)]
        cls.sources = [str(ROOT / p) for p in (
            "tests/native/test_finite_profile_capture.cpp", "src/control/finite_profile_driver_bridge.cpp",
            "src/control/finite_profile_executor.cpp", "src/control/finite_profile_contract.cpp",
            "src/motor/motor_driver.cpp")]
        result = subprocess.run([*cls.command, *cls.sources, "-o", str(cls.binary)],
                                capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise AssertionError(f"Capture integration compilation failed:\n{result.stderr}")

    def run_case(self, name):
        result = subprocess.run([str(self.binary), name], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_repeated_pwm_and_quiet_writes_survive_ring_overwrite(self):
        self.run_case("repeated")

    def test_external_stop_keeps_capture_completeness_separate(self):
        self.run_case("external")

    def test_overflow_preserves_prefix_and_never_blocks_off(self):
        self.run_case("overflow")

    def test_all_command_slots_retained_without_automatic_eviction(self):
        self.run_case("retention")

    def test_recreated_bridge_cannot_relabel_old_capture(self):
        self.run_case("identity")

    def test_driver_owned_capture_survives_bridge_destruction(self):
        self.run_case("destruction")

    def test_maximum_profile_fits_explicit_memory_budget(self):
        self.run_case("maximum")
