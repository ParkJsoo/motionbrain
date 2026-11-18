"""Exercise the whole-arm controller with a simulated clock and output port."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class WholeArmActionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("c++")
        if not compiler:
            raise unittest.SkipTest("C++ compiler unavailable")
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.binary = Path(cls.tmp.name) / "whole_arm_action"
        subprocess.run(
            [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
             "-DMOTIONBRAIN_ENABLE_WHOLE_ARM_POLICY=1", "-I", str(ROOT / "tests/native/stubs"),
             "-I", str(ROOT / "src"), str(ROOT / "src/motor/motor_driver.cpp"),
             str(ROOT / "tests/native/test_whole_arm_action.cpp"),
             str(ROOT / "src/control/whole_arm_action.cpp"), "-o", str(cls.binary)],
            check=True, timeout=60,
        )

        cls.service = Path(cls.tmp.name) / "whole_arm_service"
        native = ROOT / "tests/native"
        include = []
        for path in ("whole_arm_stubs", "service_stubs", "m4_goal_stubs", "stubs"):
            include.extend(["-I", str(native / path)])
        subprocess.run(
            [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
             "-DMOTIONBRAIN_ENABLE_WHOLE_ARM_POLICY=1", *include, "-I", str(ROOT / "src"),
             str(native / "test_whole_arm_service.cpp"), str(ROOT / "src/motor/motor_driver.cpp"),
             str(ROOT / "src/control/whole_arm_service.cpp"), str(ROOT / "src/control/whole_arm_action.cpp"),
             "-o", str(cls.service)], check=True, timeout=60,
        )

    def test_independent_timeout_and_disarm(self):
        for case in ("stalled", "preempt", "first", "race", "correction-once", "correction-stall", "correction-abort",
                     "correction-start-offset", "correction-slow-start"):
            with self.subTest(case=case):
                subprocess.run([str(self.service), case], check=True, timeout=5)

    def test_action_controller(self):
        for case in ("sequence", "deadline", "abort", "sensor", "full", "hard_cap", "wrap",
                     "output", "phase_angle", "motor"):
            with self.subTest(case=case):
                subprocess.run([str(self.binary), case], check=True, timeout=5)
