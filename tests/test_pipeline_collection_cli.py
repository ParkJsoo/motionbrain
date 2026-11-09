"""수집 CLI의 정적 거절·인증 대기 중 변경 방지. 실제 장치 호출은 금지한다."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools import collect_pipeline_episode as cli


class CollectionCliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve(); (self.root/'.local').mkdir()
        self.helper = self.root/'helper'; self.helper.write_bytes(b'#!/bin/sh\nexit 0\n'); self.helper.chmod(0o700)
        self.plan = dict(schemaVersion='motionbrain.pipeline_collection_plan.v1',helperPath='helper',
            helperSha256=hashlib.sha256(self.helper.read_bytes()).hexdigest(),controllerUrl='http://127.0.0.1:8181',
            deviceId='synthetic-camera',viewId='test-view',sessionRoot='.local/sessions',requestRoot='.local/requests',
            poseContext='정지한 가상 장치',command=dict(joint='gripper',direction='close',percent=40,durationMs=350))
        self.path = self.root/'plan.json'; self.save()
        self.addCleanup(patch.stopall)
        patch.object(cli,'ROOT',self.root).start()
        self.start = patch.object(cli.ManualCollection,'start',side_effect=AssertionError('device access forbidden')).start()
        self.pulse = patch.object(cli.ManualCollection,'pulse',side_effect=AssertionError('device access forbidden')).start()
        self.collect = patch.object(cli,'collect_once',return_value={'status':'synthetic_no_device'}).start()
        self.auth = patch.object(cli.getpass,'getpass',return_value='synthetic').start()
        patch.object(cli.sys.stdin,'isatty',return_value=True).start()

    def save(self):
        self.path.write_text(json.dumps(self.plan))
        return hashlib.sha256(self.path.read_bytes()).hexdigest()

    def execute(self):
        args=['--plan',str(self.path),'--plan-sha',self.save(),'--request-id','one',
              '--execute','--onsite-confirmed']
        with contextlib.redirect_stdout(io.StringIO()):cli.main(args)

    def assert_no_dispatch(self):
        self.start.assert_not_called();self.pulse.assert_not_called();self.collect.assert_not_called()

    def test_default_check_never_requests_authentication_or_device(self):
        output=io.StringIO()
        with contextlib.redirect_stdout(output):cli.main(['--plan',str(self.path)])
        self.assertEqual(json.loads(output.getvalue())['status'],'plan_checked_no_device_access')
        self.auth.assert_not_called();self.assert_no_dispatch()

    def test_invalid_static_identities_and_pose_are_rejected_before_auth(self):
        original=dict(self.plan)
        for key,value,reason in [('deviceId','','device_identity'),('deviceId',False,'device_identity'),
                ('viewId','bad/id','view_identity'),('viewId','','view_identity'),
                ('poseContext','   ','pose_context'),('poseContext','x'*501,'pose_context')]:
            with self.subTest(key=key,value=value):
                self.plan=dict(original);self.plan[key]=value;self.save()
                with self.assertRaisesRegex(ValueError,reason):cli.main(['--plan',str(self.path)])
        self.auth.assert_not_called();self.assert_no_dispatch()

    def test_non_executable_helper_is_rejected_before_auth(self):
        self.helper.chmod(0o600)
        with self.assertRaisesRegex(ValueError,'not_executable'):cli.main(['--plan',str(self.path)])
        self.auth.assert_not_called();self.assert_no_dispatch()

    def test_plan_changed_during_authentication_is_rejected(self):
        def changed(*_):
            self.plan['command']['percent']=30;self.save();return 'synthetic'
        self.auth.side_effect=changed
        with self.assertRaisesRegex(ValueError,'plan_hash_changed'):self.execute()
        self.assert_no_dispatch()

    def test_helper_changed_during_authentication_is_rejected(self):
        def changed(*_):
            self.helper.write_bytes(b'changed native code');return 'synthetic'
        self.auth.side_effect=changed
        with self.assertRaisesRegex(ValueError,'helper_hash_changed'):self.execute()
        self.assert_no_dispatch()

    def test_helper_permission_removed_during_authentication_is_rejected(self):
        def changed(*_):self.helper.chmod(0o600);return 'synthetic'
        self.auth.side_effect=changed
        with self.assertRaisesRegex(ValueError,'not_executable'):self.execute()
        self.assert_no_dispatch()

    def test_helper_symlink_target_cannot_change_during_authentication(self):
        second=self.root/'second';second.write_bytes(self.helper.read_bytes());second.chmod(0o700)
        link=self.root/'linked';link.symlink_to(self.helper)
        self.plan['helperPath']='linked';self.save()
        def changed(*_):link.unlink();link.symlink_to(second);return 'synthetic'
        self.auth.side_effect=changed
        with self.assertRaisesRegex(ValueError,'binding_changed'):self.execute()
        self.assert_no_dispatch()

    def test_explicit_execution_binds_approval_and_erases_memory_token(self):
        digest=self.save();self.execute()
        self.collect.assert_called_once()
        args,kwargs=self.collect.call_args;manager=args[0]
        self.assertEqual(kwargs['approval_plan_sha256'],digest)
        self.assertEqual(manager.token,'')
        self.start.assert_not_called();self.pulse.assert_not_called()

    def test_optional_paired_camera_is_static_checked_without_capture(self):
        self.plan['gripper']={'url':'http://camera.invalid','bootId':'a'*32};self.save()
        with patch.object(cli.GripperObservationSource,'capture',side_effect=AssertionError('camera access forbidden')):
            with contextlib.redirect_stdout(io.StringIO()):cli.main(['--plan',str(self.path)])
        self.auth.assert_not_called();self.assert_no_dispatch()
        self.plan['gripper']['bootId']='wrong';self.save()
        with self.assertRaisesRegex(ValueError,'boot_binding'):cli.validate(self.path)

    def test_non_tty_execution_never_requests_secret(self):
        with patch.object(cli.sys.stdin,'isatty',return_value=False),contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):self.execute()
        self.auth.assert_not_called();self.assert_no_dispatch()


if __name__=='__main__':unittest.main()
