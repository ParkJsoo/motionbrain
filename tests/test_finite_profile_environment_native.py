"""Service admission with real environment/bridge/driver and fake peripheral services."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FiniteProfileEnvironmentNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for n in ("clang++", "g++", "c++") if (p := shutil.which(n))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; service environment not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-environment-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.binary = Path(cls.temp.name) / "environment"
        sdk_flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                                 check=True, capture_output=True, text=True, timeout=30).stdout.strip()
            sdk_flags = ["-isysroot", sdk]
        cls.command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags,
            "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE=1", "-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_DRIVER=1",
            "-DMOTIONBRAIN_ENABLE_BOUNDED_ACTIONS=1"]
        for p in ("tests/native/service_stubs", "tests/native/stubs", "src"):
            cls.command += ["-I", str(ROOT / p)]
        cls.sources = [str(ROOT / p) for p in (
            "tests/native/test_finite_profile_environment.cpp", "src/control/finite_profile_service_environment.cpp",
            "src/control/finite_profile_driver_bridge.cpp", "src/control/finite_profile_executor.cpp",
            "src/control/finite_profile_contract.cpp", "src/motor/motor_driver.cpp",
            "src/control/bounded_action_service.cpp", "src/control/bounded_action_executor.cpp",
            "src/control/command_bus.cpp", "src/control/finite_profile_legacy_guard.cpp")]
        result = subprocess.run([*cls.command, *cls.sources, "-o", str(cls.binary)],
                                capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise AssertionError(f"Service environment compilation failed:\n{result.stderr}")

    def run_case(self, name):
        result = subprocess.run([str(self.binary), name], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_uninitialized_dependencies_deny_without_token_or_output(self):
        self.run_case("readiness")

    def test_controllers_queue_manual_lease_and_prepared_work_block_admission(self):
        self.run_case("admission")

    def test_safety_failure_before_execute_cannot_be_rearmed_by_polling(self):
        self.run_case("safety")

    def test_existing_bounded_owner_blocks_finite_prepare(self):
        self.run_case("bounded")

    def test_own_execution_and_quiet_are_not_competing_owners(self):
        self.run_case("quiet")

    def test_missing_failed_or_zero_token_issuer_never_grants(self):
        self.run_case("tokens")

    def test_late_busy_environment_cannot_restore_old_reservation(self):
        self.run_case("late-busy")

    def test_allowed_passive_queue_does_not_invalidate_execution_or_quiet(self):
        self.run_case("passive-queue")

    def test_old_bounded_duplicate_or_conflict_cannot_interrupt_new_finite_owner(self):
        self.run_case("old-bounded")
