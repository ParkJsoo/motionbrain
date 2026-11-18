"""실제 내장 UI를 실행하는 무장치 세대·리스·입력 회귀 검증."""
import json
from pathlib import Path
import subprocess
import unittest

from tests.controller_ui_source import embedded_scripts

ROOT = Path(__file__).resolve().parents[1]


class ManualLeaseUI(unittest.TestCase):
    def test_original_ui_gestures_and_out_of_order_callbacks(self):
        names = ('manualLease', 'commandHeaders', 'commandFetch', 'motorStart', 'motorStop',
                 'motorStopNow', 'refreshMotorLease', 'jointStart', 'jointStop', 'jointStopNow',
                 'refreshJointLease', 'stopAllMotors', 'initJoystick', 'refreshJoystickLeases',
                 'handleGlobalDrag')
        functions = []
        source = (ROOT/'src/network/web_server.cpp').read_text()
        for value in embedded_scripts(source):
            if (any(value.startswith('function '+name+'(') for name in names)
                    or value.startswith('document.addEventListener("keydown"')
                    or value.startswith('document.addEventListener("keyup"')
                    or value.startswith("window.addEventListener('blur'")):
                functions.append(value)
        self.assertEqual(len(functions), len(names)+3)
        result = subprocess.run(['node', str(ROOT/'tests/controller_ui_manual_lease_fixture.cjs')],
                                input=json.dumps(functions), text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)['checks'], 14)
        self.assertIn('onclick=\\"motorStopNow(', source)

class ProxyLeaseContract(unittest.TestCase):
    def test_strict_start_renew_and_stop_fields(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('manual_ui_proxy_contract', ROOT/'tools/record_controller_ui.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        start = dict(lease=['start'], expectedBootId=['a'*32], expectedGeneration=['0'])
        renew = dict(lease=['renew'], leaseId=['a'*32+'-3-00000001'], leaseSeq=['1'])
        module.validate_manual_lease_params(start, stopping=False)
        module.validate_manual_lease_params(renew, stopping=False)
        module.validate_manual_lease_params({}, stopping=True)
        invalid = [{}, dict(start, leaseId=renew['leaseId']), dict(start, expectedBootId=['wrong']),
                   dict(start, expectedGeneration=['00']), dict(start, expectedGeneration=['4294967296']),
                   dict(renew, expectedGeneration=['0']), dict(renew, leaseId=['invalid']),
                   dict(renew, leaseSeq=['0']), dict(renew, leaseSeq=['01']), dict(renew, leaseSeq=['4294967296'])]
        for params in invalid:
            with self.subTest(params=params), self.assertRaises(ValueError):
                module.validate_manual_lease_params(params, stopping=False)
        with self.assertRaises(ValueError): module.validate_manual_lease_params(renew, stopping=True)


if __name__ == '__main__':
    unittest.main()
