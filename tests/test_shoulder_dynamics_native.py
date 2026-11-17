"""Compile actual shoulder/arm/driver sources for default and opt-in policies.

Virtual plants exercise software bounds; their constants are assumptions, not
physical measurements or proof that the candidate corrects real stiction.
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class ShoulderDynamicsNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for name in ('clang++', 'g++', 'c++') if (p := shutil.which(name))), None)
        if not compiler:
            raise unittest.SkipTest('C++ compiler unavailable; production sources not executed')
        cls.temp = tempfile.TemporaryDirectory(prefix='motionbrain-shoulder-dynamics-')
        cls.addClassCleanup(cls.temp.cleanup)
        flags = []
        if sys.platform == 'darwin' and shutil.which('xcrun'):
            sdk = subprocess.run(['xcrun', '--sdk', 'macosx', '--show-sdk-path'], check=True,
                                 capture_output=True, text=True, timeout=30).stdout.strip()
            flags = ['-isysroot', sdk]
        cls.binaries = []
        for policy in (0, 1):
            binary = Path(cls.temp.name) / f'shoulder-{policy}'
            args = [compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror', *flags,
                    f'-DMOTIONBRAIN_ENABLE_BOUNDED_SHOULDER_DOWN_CORRECTION={policy}',
                    '-I', str(ROOT/'tests/native/shoulder_stubs'),
                    '-I', str(ROOT/'tests/native/stubs'), '-I', str(ROOT/'src'),
                    *[str(ROOT/p) for p in ('tests/native/test_shoulder_dynamics.cpp',
                       'src/control/shoulder_angle_controller.cpp',
                       'src/motion/robot_arm.cpp', 'src/motor/motor_driver.cpp')],
                    '-o', str(binary)]
            result = subprocess.run(args, capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(result.stdout+result.stderr)
            cls.binaries.append(binary)

    def run_case(self, case):
        for binary in self.binaries:
            with self.subTest(policy=binary.name):
                result = subprocess.run([str(binary), case], capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)

    def test_stiction_assumption_default_miss_and_candidate_reach(self): self.run_case('recover')
    def test_no_motion_stops_and_correction_attempt_cap(self): self.run_case('no_motion')
    def test_global_timeout_preempts_continuing_slow_motion(self): self.run_case('timeout')
    def test_stale_softlimit_safety_disarm_and_cancel_force_off(self): self.run_case('faults')
    def test_high_coast_reverses_correction_but_stays_bounded(self): self.run_case('reverse')
    def test_relative_pwm_ramp_included_timer_and_status(self): self.run_case('timer_pwm')
    def test_target_crossing_forces_off_before_pulse_timeout(self): self.run_case('crossing')
    def test_reversed_correction_no_motion_still_hits_attempt_cap(self): self.run_case('reverse_stuck')

if __name__ == '__main__': unittest.main()
