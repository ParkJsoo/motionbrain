import argparse
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch
from tests.test_supervised_collection import plan, Clock, Camera, Port
from learning.supervised_collection import SupervisedCollection, plan_digest

spec=importlib.util.spec_from_file_location('supervised_cli',Path(__file__).resolve().parents[1]/'tools/collect_supervised_actions.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)

class CliTests(unittest.TestCase):
    def test_validation_has_no_capture_or_http(self):
        with tempfile.TemporaryDirectory() as t:
            helper=Path(t)/'helper';helper.write_bytes(b'test helper');p=plan();p['helperSha256']=hashlib.sha256(helper.read_bytes()).hexdigest()
            path=Path(t)/'plan.json';path.write_text(json.dumps(p))
            with patch('subprocess.Popen') as capture,patch('urllib.request.OpenerDirector.open') as http,contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(['validate','--plan',str(path),'--helper',str(helper)]),0)
            capture.assert_not_called();http.assert_not_called()
    def test_single_use_session_and_pending_blocked_before_device(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);(root/'.local').mkdir();p=plan();clock=Clock();camera=Camera(clock);port=Port(clock)
            args=argparse.Namespace(onsite=True,confirm_plan_sha256=plan_digest(p),output_dir=root/'.local/run1',helper=root/'helper')
            factory=lambda p,c,port,s:SupervisedCollection(p,c,port,s,clock=clock)
            with patch.object(cli,'ROOT',root),patch.dict('os.environ',{'MOTIONBRAIN_HTTP_TOKEN':'test-token'}),patch('learning.action_recorder.ActionRecorder',return_value=camera) as ctor,patch('learning.supervised_transport.SequencePort',return_value=port),patch.object(cli,'SupervisedCollection',side_effect=factory),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.run(args,p),0);self.assertEqual(port.executions,3)
                args.output_dir=root/'.local/run2'
                with self.assertRaisesRegex(ValueError,'session_already_consumed'):cli.run(args,p)
                self.assertEqual(ctor.call_count,1)
                db=sqlite3.connect(root/'.local/visual-actions/controller-commands.sqlite')
                db.execute("INSERT INTO supervised_blocks VALUES ('crashed','hash','path','pending',NULL)");db.commit();db.close()
                p['sessionId']='next';p['camera']['sessionId']='next';args.confirm_plan_sha256=plan_digest(p)
                with self.assertRaisesRegex(ValueError,'interrupted_block'):cli.run(args,p)
                self.assertEqual(ctor.call_count,1)
    def test_execute_confirmation_and_private_destination(self):
        p=plan();args=argparse.Namespace(onsite=False,confirm_plan_sha256=plan_digest(p))
        with self.assertRaisesRegex(ValueError,'attended_plan'):cli.run(args,p)
        with self.assertRaisesRegex(ValueError,'private_artifact'):cli.private_path(Path('/tmp/not-private'))

if __name__=='__main__':unittest.main()
