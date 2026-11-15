"""Production guard service + driver with fake clock/sensor/IO; no physical proof."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class M4GoalServiceNativeTests(unittest.TestCase):
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
        for enabled in (1,):
            binary = Path(cls.tmp.name) / str(enabled)
            subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror', *sdk,
                f'-DMOTIONBRAIN_ENABLE_M4_GOAL_GUARD={enabled}', '-I', str(ROOT / 'tests/native/m4_goal_stubs'),
                '-I', str(ROOT / 'tests/native/service_stubs'),
                '-I', str(ROOT / 'tests/native/stubs'), '-I', str(ROOT / 'src'),
                str(ROOT / 'src/control/m4_goal_guard.cpp'),
                str(ROOT / 'src/control/m4_goal_service.cpp'),
                str(ROOT / 'src/motor/motor_driver.cpp'),
                str(ROOT / 'tests/native/test_m4_goal_service.cpp'), '-o', str(binary)],
                check=True, capture_output=True, text=True, timeout=60)
            cls.binaries[enabled] = binary

    def test_service_driver_goal_fault_stop_deadline_and_no_replay(self):
        for case in range(1,11):
            with self.subTest(case=case):
                subprocess.run([str(self.binaries[1]),str(case)],check=True,timeout=5,capture_output=True)

    def test_actual_firmware_json_matches_python_terminal_contract(self):
        import json
        from learning.m4_device_guard_transport import capability, POLICY_SHA256
        from learning.m4_device_guard_policy import device_guard_task_pulse
        raw = subprocess.check_output([str(self.binaries[1]), '1'], text=True, timeout=5)
        response = json.loads('{' + raw.strip() + '}')
        request = dict(nonce='b'*32, bootId='a'*32, scopeSha256='c'*64,
                       policySha256=POLICY_SHA256, expectedStartDeg=197.75, goalDeg=197.22)
        record = capability(response, request, state='terminal', pulse=device_guard_task_pulse('down'))
        self.assertEqual(record['reason'], 'goal')

    def test_current_table_float32_terminal_matches_python_at_all_boundaries(self):
        import json
        from learning.m4_device_guard_transport import capability, CURRENT_POLICY_SHA256, validate_request
        from learning.m4_device_guard_policy import device_guard_task_pulse, current_table_device_guard_policy
        for i,table in enumerate((194.32,192.29,197.39,194.32005,194)):
            for j,direction in enumerate(('up','down')):
                with self.subTest(table=table,direction=direction):
                    raw=subprocess.check_output([str(self.binaries[1]),str(20+2*i+j)],text=True,timeout=5)
                    response=json.loads('{'+raw.strip()+'}')
                    policy=current_table_device_guard_policy(table)
                    pulse=device_guard_task_pulse(direction,policy)
                    request=dict(nonce='b'*32,bootId='a'*32,scopeSha256='c'*64,policySha256=CURRENT_POLICY_SHA256,
                        tableReferenceDeg=table,expectedStartDeg=table if direction=='up' else round(table+.63,5),
                        goalDeg=policy['upGoalAngleDeg' if direction=='up' else 'downGoalAngleDeg'])
                    validate_request(request,pulse)
                    record=capability(response,request,state='terminal',pulse=pulse)
                    self.assertEqual(record['reason'],'goal')
