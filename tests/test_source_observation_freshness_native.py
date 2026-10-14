"""Production pure C++ source freshness logic; no ROS runtime or camera required."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SourceObservationNativeTest(unittest.TestCase):
    def test_replay_transport_clocks_metadata_and_lifecycle(self):
        compiler = next((p for n in ('clang++', 'g++', 'c++') if (p := shutil.which(n))), None)
        if compiler is None:
            self.skipTest('C++ compiler unavailable; source freshness not executed')
        flags = []
        if sys.platform == 'darwin':
            sdk = subprocess.run(['xcrun', '--sdk', 'macosx', '--show-sdk-path'],
                                 check=True, capture_output=True, text=True).stdout.strip()
            flags = ['-isysroot', sdk]
        with tempfile.TemporaryDirectory(prefix='motionbrain-source-') as tmp:
            binary = str(Path(tmp) / 'freshness')
            subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', *flags,
                            '-I', str(ROOT / 'ros2_ws/src/motionbrain_control/include'),
                            str(ROOT / 'tests/native/test_source_observation_freshness.cpp'),
                            '-o', binary], check=True, capture_output=True, timeout=60)
            subprocess.run([binary], check=True, capture_output=True, timeout=10)
