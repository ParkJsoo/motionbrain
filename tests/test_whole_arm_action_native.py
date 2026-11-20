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
        command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
             "-DMOTIONBRAIN_ENABLE_WHOLE_ARM_POLICY=1", *include, "-I", str(ROOT / "src"),
             str(native / "test_whole_arm_service.cpp"), str(ROOT / "src/motor/motor_driver.cpp"),
             str(ROOT / "src/control/whole_arm_service.cpp"), str(ROOT / "src/control/whole_arm_action.cpp"),
             "-o", str(cls.service)]
        subprocess.run(command, check=True, timeout=60)
        cls.elbow_service = Path(cls.tmp.name) / "elbow_once"
        subprocess.run([command[0], "-DMOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE=1", *command[1:-1], str(cls.elbow_service)],
                       check=True, timeout=60)

        cls.step_service = Path(cls.tmp.name) / "bounded_steps"
        subprocess.run([command[0], "-DMOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE=1",
                        "-DMOTIONBRAIN_ENABLE_BOUNDED_APPROACH=1", *command[1:-1], str(cls.step_service)],
                       check=True, timeout=60)

        cls.pose_service = Path(cls.tmp.name) / "pose_grasp"
        subprocess.run([command[0], "-DMOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE=1",
                        "-DMOTIONBRAIN_ENABLE_BOUNDED_APPROACH=1", "-DMOTIONBRAIN_ENABLE_POSE_GRASP=1",
                        *command[1:-1], str(cls.pose_service)], check=True, timeout=60)
        cls.wave_service = Path(cls.tmp.name) / "pose_elbow_wave"
        subprocess.run([command[0], "-DMOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE=1",
                        "-DMOTIONBRAIN_ENABLE_BOUNDED_APPROACH=1", "-DMOTIONBRAIN_ENABLE_POSE_GRASP=1",
                        "-DMOTIONBRAIN_ENABLE_ELBOW_WAVE=1", *command[1:-1], str(cls.wave_service)],
                       check=True, timeout=60)
        cls.timer_service = Path(cls.tmp.name) / "elbow_timer"
        subprocess.run([command[0], "-DMOTIONBRAIN_ENABLE_ELBOW_DOWN_ONCE=1",
                        "-DMOTIONBRAIN_ENABLE_BOUNDED_APPROACH=1", "-DMOTIONBRAIN_ENABLE_POSE_GRASP=1",
                        "-DMOTIONBRAIN_ENABLE_ELBOW_WAVE=1", "-DMOTIONBRAIN_ENABLE_ELBOW_TIMER=1",
                        str(ROOT / "src/system/system_init.cpp"), *command[1:-1], str(cls.timer_service)], check=True, timeout=60)

    def test_shoulder_timer(self):
        for case in ("300", "420", "650", "recorded", "stale", "missed", "angle", "stop",
                     "stop-after-zero", "handoff-race"):
            with self.subTest(case=case):
                subprocess.run([str(self.timer_service), "shoulder-timer-"+case], check=True, timeout=5)

    def test_timer_wave(self):
        for case in ("timer-normal", "timer-shadow-stall", "timer-no-main",
                     "timer-safety", "timer-late", "timer-stop", "timer-direct-disarm", "timer-shadow-215"):
            with self.subTest(case=case):
                subprocess.run([str(self.timer_service),case],check=True,timeout=5)

    def test_elbow_wave(self):
        for binary in (self.service, self.elbow_service, self.step_service, self.pose_service):
            subprocess.run([str(binary), "wave-disabled"], check=True, timeout=5)
        for case in ("wave-normal", "wave-jitter", "wave-early", "wave-late-first", "wave-late-edge",
                     "wave-late-zero", "wave-stall", "wave-angle", "wave-abort", "pose-chain",
                     "pose-budgets", "pose-long-step", "elbow-normal"):
            with self.subTest(case=case):
                subprocess.run([str(self.wave_service), case], check=True, timeout=5)

    def test_pose_grasp(self):
        for binary in (self.service,self.elbow_service,self.step_service):
            subprocess.run([str(binary),"pose-disabled"],check=True,timeout=5)
        for case in ("pose-chain","pose-stall","pose-abort","pose-expired","pose-angle-change",
                     "pose-close-cap","pose-wrist-stall","pose-budgets","pose-wrist-budget","pose-long-step",
                     "pose-long-stall","pose-long-angle","elbow-normal"):
            with self.subTest(case=case):
                subprocess.run([str(self.pose_service),case],check=True,timeout=5)

    def test_bounded_steps(self):
        for binary in (self.service, self.elbow_service):
            subprocess.run([str(binary), "steps-disabled"], check=True, timeout=5)
        for case in ("steps-chain","steps-stall","steps-preempt","steps-off-override","steps-budget","elbow-normal"):
            with self.subTest(case=case):
                subprocess.run([str(self.step_service), case], check=True, timeout=5)

    def test_elbow_once(self):
        subprocess.run([str(self.service), "elbow-disabled"], check=True, timeout=5)
        for case in ("elbow-normal", "elbow-stall", "elbow-abort"):
            with self.subTest(case=case):
                subprocess.run([str(self.elbow_service), case], check=True, timeout=5)

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
