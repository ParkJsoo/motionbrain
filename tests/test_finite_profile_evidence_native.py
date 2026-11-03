"""Raw per-command PWM evidence with the production driver and fake Arduino IO."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FiniteProfileEvidenceNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for name in ("clang++", "g++", "c++")
                         if (p := shutil.which(name))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; raw output evidence was not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-evidence-")
        cls.addClassCleanup(cls.temp.cleanup)
        sdk_flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                                 check=True, capture_output=True, text=True, timeout=30).stdout.strip()
            sdk_flags = ["-isysroot", sdk]
        cls.command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags,
                       "-I", str(ROOT / "tests/native/stubs"), "-I", str(ROOT / "src")]
        cls.sources = [str(ROOT / p) for p in (
            "tests/native/test_finite_profile_evidence.cpp", "src/motor/motor_driver.cpp")]
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
                raise AssertionError(f"Evidence build ({mode}) failed:\n{result.stderr}")
            cls.binaries[mode] = binary

    def run_case(self, name, mode="driver"):
        result = subprocess.run([str(self.binaries[mode]), name],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_disabled_preserves_legacy(self): self.run_case("legacy", "default")
    def test_core_only_preserves_legacy(self): self.run_case("legacy", "core")
    def test_binding_validates_identity_without_output(self): self.run_case("bind")
    def test_same_pwm_writes_are_preserved_without_ring_deduplication(self): self.run_case("repeated")
    def test_quiet_retry_close_and_ring_overwrite(self): self.run_case("quiet")
    def test_overflow_blocks_nonzero_but_never_off(self): self.run_case("overflow")
    def test_capacity_32_has_no_eviction(self): self.run_case("capacity")
    def test_duplicate_identity_cannot_rebind_history(self): self.run_case("duplicate")
    def test_unbound_low_level_lease_remains_supported(self): self.run_case("unbound")
    def test_counters_saturate(self): self.run_case("saturation")
    def test_fixed_memory_budget(self): self.run_case("memory")


if __name__ == "__main__":
    unittest.main()
