"""Offline CLI and observation commissioning checks without hardware."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.run_visual_action_pilot import main
from learning.visual_action_live import observe_stream
from tests.test_visual_action_live import plan, PumpClock, FakeCamera


class PilotCLITest(unittest.TestCase):
    def test_default_validation_and_mismatch_never_open_devices(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);helper=root/'helper';helper.write_bytes(b'not executable')
            p=plan();p['helperSha256']=hashlib.sha256(helper.read_bytes()).hexdigest()
            path=root/'plan.json';path.write_text(json.dumps(p))
            args=['--plan',str(path),'--helper',str(helper)]
            with patch('subprocess.Popen',side_effect=AssertionError('camera forbidden')),patch('socket.socket',side_effect=AssertionError('network forbidden')),contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(main(args),0)
                self.assertEqual(json.loads(out.getvalue())['captureCalls'],0)
                with self.assertRaises(ValueError):main(args+['--execute','--confirm-plan-sha256','wrong'])
    def test_observation_only_has_no_action_api_and_closes_after_success(self):
        clock=PumpClock();camera=FakeCamera(clock,plan())
        with patch('learning.action_port.BoundedActionHTTP',side_effect=AssertionError('no motor port')),patch('socket.socket',side_effect=AssertionError('network forbidden')):
            result=observe_stream(camera,clock=clock)
        self.assertTrue(result['observationComplete']);self.assertTrue(camera.closed)
        self.assertGreater(result['frameCount'],2);self.assertEqual(result['motorCalls'],0)
    def test_observation_fault_closes_without_retry(self):
        clock=PumpClock();camera=FakeCamera(clock,plan());camera.inject=lambda c:c.fail('tracking_lost')
        result=observe_stream(camera,clock=clock)
        self.assertFalse(result['observationComplete']);self.assertEqual(camera.count,1);self.assertTrue(camera.closed)
