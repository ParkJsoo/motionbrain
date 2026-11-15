"""Pure proposed firmware guard state machine only; no API/driver integration or devices."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class M4GoalGuardNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((shutil.which(n) for n in ('clang++', 'g++', 'c++') if shutil.which(n)), None)
        if compiler is None:
            raise unittest.SkipTest('C++ compiler unavailable')
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.binaries = {}
        sdk = []
        if sys.platform == 'darwin' and shutil.which('xcrun'):
            sdk = ['-isysroot', subprocess.check_output(['xcrun', '--sdk', 'macosx', '--show-sdk-path'], text=True).strip()]
        for enabled in (0, 1):
            binary = Path(cls.tmp.name) / str(enabled)
            subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror', *sdk,
                f'-DMOTIONBRAIN_ENABLE_M4_GOAL_GUARD={enabled}', '-I', str(ROOT / 'src'),
                str(ROOT / 'src/control/m4_goal_guard.cpp'),
                str(ROOT / 'tests/native/test_m4_goal_guard.cpp'), '-o', str(binary)],
                check=True, capture_output=True, text=True, timeout=60)
            cls.binaries[enabled] = binary

    def test_default_disabled_has_no_nonce_or_output_mutation(self):
        subprocess.run([str(self.binaries[0]), '0'], check=True, timeout=5)

    def test_enabled_goal_fault_deadline_nonce_and_off_proof_cases(self):
        for case in range(1, 15):
            with self.subTest(case=case):
                subprocess.run([str(self.binaries[1]), str(case)], check=True, timeout=5)
