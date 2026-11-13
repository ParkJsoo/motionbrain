"""합성 영상·모의 수집기만 사용한 비접촉 외부 보호 검사."""
import copy
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.approach_external_guard import (ApproachExternalGuard, CurrentSceneExternalGuard,
    SCHEMA, CURRENT_SCENE_SCHEMA, CURRENT_REFERENCE_MEANING)
from learning.artifacts import ArtifactStore
from learning.relative_model_bundle import sha256
from learning.supervised_collection import plan_digest
from tests.test_grasp_observation import Clock
from tests.test_supervised_transport import Controller


class ExternalGuardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.repo = Path(self.tmp.name); self.assets = self.repo/'.local/assets'; self.assets.mkdir(parents=True)
        self.clock = Clock(); self.calls = []
        self.store = ArtifactStore(self.repo/'.local/observations')
        self.image = np.full((1080, 1920, 3), 100, np.uint8)
        texture = np.random.default_rng(3).integers(5, 220, (64, 57), dtype=np.uint8)
        self.image[400:464, 1000:1057] = texture[:, :, None]
        cv2.rectangle(self.image, (550, 650), (600, 750), (120, 40, 240), -1)
        template = cv2.imencode('.png', texture)[1].tobytes()
        self.template = self.write('template.png', template)
        self.tracking = dict(templateFile='template.png', templateSha256=self.template['sha256'],
            additionalTemplateFile='template.png', additionalTemplateSha256=self.template['sha256'],
            goalTemplateCenter=[1028.5, 432.], searchRoi=[945, 335, 1162, 559],
            minimumNcc=.9, minimumPeakGap=.04, expectedM4=196.17, maximumM4Difference=.5,
            templateRotationsDeg=[-6, -4, -2, 0, 2, 4, 6])
        tracking = self.write_json('tracking.json', self.tracking)
        self.goal = dict(forearmCenter=[1028.5, 432.], objectCenter=[575.5, 700.5])
        goal = self.write_json('goal.json', self.goal)
        sources = {k:self.write(k+'.bin', ('original-'+k).encode()) for k in ('image','metadata','clock','admission')}
        # 고정 물체 원본의 path는 별도 원본 root 기준이다.
        sources = {k:dict(v,path=Path(v['path']).name) for k,v in sources.items()}
        fixed = self.write_json('fixed.json', dict(external=dict(objectBox=[550,650,601,751]),source=sources))
        self.config = dict(schemaVersion=SCHEMA, tracking=tracking, goal=goal, fixedObject=fixed,
            fixedObjectSourceRoot='.local/assets', maxAgeSeconds=3., controllerBootId='a'*32,
            hostClockId='trial', gripperCameraBootId='b'*32, externalViewId='external-fixed', externalDeviceId='iphone',
            executionTimingPolicy='elbow-deadline-350-v1')
        self.binding = self.write_json('config.json', self.config)
        self.camera_root = self.repo/'.local/camera'; self.camera_root.mkdir()
        bgra = cv2.cvtColor(self.image, cv2.COLOR_BGR2BGRA).tobytes()
        self.pixels = bgra
        raw = b'synthetic-compressed-frame'
        (self.camera_root/'frame.bgra.lzbitmap').write_bytes(raw)
        metadata = dict(sessionId='trial',viewConfigurationId='external-fixed',deviceUniqueId='iphone',
            sourceFrameId='trial:1',imageSha256=sha256(raw),pixelSha256=sha256(bgra))
        metadata_raw = json.dumps(metadata).encode();(self.camera_root/'frame.json').write_bytes(metadata_raw)
        self.frame = dict(frameId='trial:1',sourceEarliest=49.9,qualityFlags=[],width=1920,height=1080,
            imageFile='frame.bgra.lzbitmap',metadataFile='frame.json',imageSha256=sha256(raw),
            metadataSha256=sha256(metadata_raw),pixelSha256=sha256(bgra),rawByteLength=len(bgra))
        self.recorder = SimpleNamespace(root=self.camera_root,lock=threading.RLock(),frames=[self.frame],
            plan=dict(sessionId='trial',viewId='external-fixed',deviceId='iphone'),check=lambda:self.calls.append('camera'))
        controller = Controller(self.clock); controller.angle=196.17
        self.port = SimpleNamespace(boot_id='a'*32,check_idle=lambda:self.calls.append('http') or controller.status())
        self.controller=controller
        self.guard = ApproachExternalGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)
        self.record = dict(hostClockId='trial',frame={'frameId':'gripper:1'})
        self.result = dict(cameraBootId='b'*32,sourceFreshMetadataVerified=True,state=[1.,16.,124.],
                           object={'box':[100,100,140,160]})
        self.patch_decode = patch('learning.approach_external_guard.decode',return_value=bgra)
        self.patch_decode.start();self.addCleanup(self.patch_decode.stop)

    def write(self,name,raw):
        p=self.assets/name;p.write_bytes(raw)
        return dict(path=str(p.relative_to(self.repo)),sha256=sha256(raw))

    def write_json(self,name,value):
        return self.write(name,json.dumps(value).encode())

    def review(self):
        return self.guard.review(copy.deepcopy(self.record),copy.deepcopy(self.result),deadline=60.,tick=lambda:None)

    def test_construction_and_check_do_not_query_controller(self):
        self.assertEqual(self.calls,[])
        self.guard.check()
        self.assertEqual(self.calls,['camera'])

    def test_current_originals_approve_only_protection_and_bind_evidence(self):
        result=self.review()
        self.assertTrue(result['accepted'])
        self.assertEqual(result['observationSha256'],plan_digest([self.record,self.result]))
        raw=(self.store.root/result['evidence']['path']).read_bytes()
        self.assertEqual(sha256(raw),result['evidence']['sha256'])
        evidence=json.loads(raw)
        self.assertEqual(evidence['external']['objectBox'],[550,650,601,751])
        self.assertEqual(self.calls.count('http'),1)

    def test_expired_review_prevents_run_but_disarm_allows_off_wait(self):
        self.review();command=dict(joint='elbow',direction='up',percent=60,durationMs=350)
        self.guard.validate_command(command)
        self.clock.sleep(3.)
        with self.assertRaisesRegex(ValueError,'review_expired'):self.guard.check()
        self.guard.disarm();self.guard.check()
        with self.assertRaisesRegex(ValueError,'review_expired'):self.guard.validate_command(command)

    def test_direction_guard_rejects_without_selecting_a_substitute(self):
        self.review()
        with self.assertRaisesRegex(ValueError,'direction_clearance'):
            self.guard.validate_command(dict(joint='elbow',direction='down',percent=60,durationMs=350))
        self.assertFalse(self.guard.active)
        self.assertEqual(self.calls.count('http'),1)

    def test_frozen_650_profile_preserves_its_direction_limits(self):
        self.goal['objectCenter'][1] -= 27
        self.config['goal'] = self.write_json('goal-650.json', self.goal)
        self.config['executionTimingPolicy'] = 'elbow-plateau-650-v1'
        binding = self.write_json('config-650.json', self.config)
        self.guard = ApproachExternalGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)
        self.review()
        self.guard.validate_command(dict(joint='elbow',direction='up',percent=35,durationMs=650))
        self.guard.disarm()
        self.guard.validate_command(dict(joint='elbow',direction='down',percent=35,durationMs=650))
        self.guard.disarm()
        with self.assertRaisesRegex(ValueError, 'timing_policy'):
            self.guard.validate_command(dict(joint='elbow',direction='up',percent=60,durationMs=650))

    def test_policy_cannot_admit_non_elbow_motion(self):
        self.config['executionTimingPolicy'] = 'wrist-deadline-550-v1'
        binding = self.write_json('config-wrong-policy.json', self.config)
        with self.assertRaisesRegex(ValueError,'M3_policy_required'):
            ApproachExternalGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_M4_motion_or_wrong_angle_refuses_review(self):
        self.controller.angle=196.8
        with self.assertRaisesRegex(ValueError,'fixed_M4_changed'):self.review()
        self.assertIsNone(self.guard._measurement)

    def test_byte_changes_or_source_lateness_cannot_reuse_review(self):
        (self.camera_root/'frame.bgra.lzbitmap').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'frame_changed'):self.review()
        self.assertEqual(self.calls.count('http'),0)

    def test_configuration_and_model_boot_changes_refuse_review(self):
        self.config['maxAgeSeconds']=4.
        self.write_json('config.json',self.config)
        with self.assertRaisesRegex(ValueError,'asset_changed'):self.review()

    def test_partial_gripper_scale_and_boot_refused(self):
        for field,value in [('cameraBootId','c'*32),('state',None)]:
            old=self.result[field];self.result[field]=value
            with self.assertRaisesRegex(ValueError,'gripper_identity'):self.review()
            self.result[field]=old
        self.result['object']['box']=[100,100,160,160]
        with self.assertRaisesRegex(ValueError,'object_scale_changed'):self.review()

    def test_fixed_object_shift_and_ambiguous_color_refused(self):
        changed=self.image.copy();changed[650:751,550:601]=100
        cv2.rectangle(changed,(570,650),(620,750),(120,40,240),-1)
        with patch.object(self.guard,'_frame',return_value=(changed,self.frame)):
            with self.assertRaisesRegex(ValueError,'fixed_object_extent_changed'):self.review()
        changed=self.image.copy();cv2.rectangle(changed,(700,650),(750,750),(120,40,240),-1)
        with patch.object(self.guard,'_frame',return_value=(changed,self.frame)):
            with self.assertRaisesRegex(ValueError,'object_ambiguous'):self.review()


class CurrentSceneGuardTests(unittest.TestCase):
    write = ExternalGuardTests.write
    write_json = ExternalGuardTests.write_json
    review = ExternalGuardTests.review

    def setUp(self):
        ExternalGuardTests.setUp(self)
        self.status_calls=[]
        self.port._check_status=lambda status,expected,pulse=None,baseline_angle=None:self.status_calls.append(
            (status,expected,pulse,baseline_angle))
        self.config.update(schemaVersion=CURRENT_SCENE_SCHEMA, referenceMeaning=CURRENT_REFERENCE_MEANING)
        fixed=json.loads((self.repo/self.config['fixedObject']['path']).read_bytes())
        fixed['external'].update(forearmCenter=self.goal['forearmCenter'],objectCenter=self.goal['objectCenter'])
        self.config['fixedObject']=self.write_json('current-fixed.json',fixed)
        self.binding = self.write_json('current-scene-config.json', self.config)
        self.guard = CurrentSceneExternalGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)

    def external(self, *, delta=0., shift_x=0., box_shift=0.):
        return dict(forearmCenter=[1028.5+shift_x,432.-delta], objectCenter=[575.5,700.5],
                    objectBox=[550+box_shift,650,601+box_shift,751], ncc=1., peakGap=1.)

    def review_external(self, **kwargs):
        with patch.object(self.guard,'_measure',return_value=self.external(**kwargs)):
            return self.review()

    def command(self, direction):
        return dict(joint='elbow', direction=direction, percent=60, durationMs=350)

    def test_new_reference_is_explicit_and_not_old_schema_or_model_success(self):
        result = self.review()
        evidence = json.loads((self.store.root/result['evidence']['path']).read_bytes())
        self.assertEqual(evidence['schemaVersion'], CURRENT_SCENE_SCHEMA)
        self.assertEqual(evidence['referenceMeaning'], CURRENT_REFERENCE_MEANING)
        self.assertEqual(evidence['experimentalFence'],dict(objectBoxMaximumChangePx=3,
            forearmMaximumDistancePx=10,upMaximumDeltaPx=6,downMinimumDeltaPx=0))
        with self.assertRaisesRegex(ValueError,'external_configuration'):
            ApproachExternalGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)
        self.config['schemaVersion'] = SCHEMA
        binding = self.write_json('mixed-mode.json',self.config)
        with self.assertRaisesRegex(ValueError,'external_configuration'):
            CurrentSceneExternalGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_reference_meaning_and_exact_fields_cannot_be_omitted_or_changed(self):
        for value in [None,'model_selected_success','old_goal_transferred']:
            config=copy.deepcopy(self.config)
            if value is None:config.pop('referenceMeaning')
            else:config['referenceMeaning']=value
            binding=self.write_json('invalid-reference.json',config)
            with self.subTest(value=value),self.assertRaises(ValueError):
                CurrentSceneExternalGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_current_reference_cannot_mix_template_goal_and_fixed_source_coordinates(self):
        for target,field in [('tracking','goalTemplateCenter'),('goal','forearmCenter'),
                             ('goal','objectCenter'),('fixedObject','forearmCenter'),('fixedObject','objectCenter')]:
            config=copy.deepcopy(self.config)
            body=json.loads((self.repo/config[target]['path']).read_bytes())
            coordinates=body['external'] if target=='fixedObject' else body
            coordinates[field][0] += .5
            config[target]=self.write_json('mismatched-reference.json',body)
            binding=self.write_json('mismatched-config.json',config)
            with self.subTest(target=target,field=field),self.assertRaisesRegex(ValueError,'same_stationary_reference'):
                CurrentSceneExternalGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_only_350ms_M3_policy_and_output_accepted(self):
        self.review()
        for direction in ('up','down'):
            self.guard.validate_command(self.command(direction));self.guard.disarm()
        for change in [dict(joint='wrist'),dict(percent=35),dict(durationMs=450),dict(direction='left')]:
            with self.subTest(change=change),self.assertRaises(ValueError):
                self.guard.validate_command(dict(self.command('up'),**change))
        for policy in ('elbow-fine-450-v1','elbow-plateau-650-v1'):
            config=dict(self.config,executionTimingPolicy=policy)
            binding=self.write_json('wrong-current-policy.json',config)
            with self.subTest(policy=policy),self.assertRaisesRegex(ValueError,'M3_policy_required'):
                CurrentSceneExternalGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_up_accepts_six_pixel_boundary_and_rejects_beyond_it(self):
        self.review_external(delta=6.)
        self.guard.validate_command(self.command('up'));self.guard.disarm()
        self.review_external(delta=6.001)
        with self.assertRaisesRegex(ValueError,'direction_clearance'):
            self.guard.validate_command(self.command('up'))
        self.assertFalse(self.guard.active)

    def test_post_delta_is_not_a_global_zero_to_six_fence(self):
        # 사후 전완거리 7px는 전체 10px 안이다. UP은 거절하지만 DOWN 보정은 허용한다.
        self.review_external(delta=7.)
        with self.assertRaisesRegex(ValueError,'direction_clearance'):
            self.guard.validate_command(self.command('up'))
        self.guard.validate_command(self.command('down'));self.guard.disarm()

    def test_down_accepts_reference_and_rejects_motion_past_reference(self):
        self.review_external(delta=0.)
        self.guard.validate_command(self.command('down'));self.guard.disarm()
        self.review_external(delta=-.001)
        with self.assertRaisesRegex(ValueError,'direction_clearance'):
            self.guard.validate_command(self.command('down'))
        self.assertFalse(self.guard.active)

    def test_new_reference_box_fence_is_three_pixels_including_after_observation(self):
        self.review_external(box_shift=3.)
        before=self.guard._count
        with self.assertRaisesRegex(ValueError,'fixed_object_extent_changed'):
            self.review_external(box_shift=3.001)
        self.assertEqual(self.guard._count,before)

    def test_new_reference_forearm_fence_is_ten_pixels(self):
        self.review_external(shift_x=10.)
        with self.assertRaisesRegex(ValueError,'retreat_scope_exceeded'):
            self.review_external(shift_x=10.001)

    def test_current_config_sha_and_immutable_reference_still_checked(self):
        self.review()
        self.config['referenceMeaning']='model_success'
        self.write_json('current-scene-config.json',self.config)
        with self.assertRaisesRegex(ValueError,'asset_changed'):self.review()

    def test_constructor_does_not_modify_port_status_checker(self):
        original=self.port._check_status
        CurrentSceneExternalGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)
        self.assertIs(self.port._check_status,original)
        self.assertEqual(self.status_calls,[])
        self.assertEqual(self.calls,[])

    def test_active_monitor_checks_fixed_M4_using_existing_status_only_and_restores(self):
        original=self.port._check_status;self.review()
        self.guard.validate_command(self.command('up'))
        wrapped=self.port._check_status
        self.assertIsNot(wrapped,original)
        before_http=self.calls.count('http')
        self.port._check_status(self.controller.status(),'ARMED',self.command('up'))
        self.assertEqual(len(self.status_calls),1)
        self.controller.angle=196.8
        try:
            with self.assertRaisesRegex(ValueError,'active_fixed_M4_changed'):
                self.port._check_status(self.controller.status(),'ARMED',self.command('up'))
        finally:
            self.guard.disarm()
        self.assertEqual(self.calls.count('http'),before_http)
        self.assertIs(self.port._check_status,original)
        self.assertFalse(self.guard.active)

    def test_real_bound_method_identity_is_preserved_on_install_and_restore(self):
        owner=self
        class Port:
            boot_id='a'*32
            def check_idle(self):return owner.controller.status()
            def _check_status(self,status,expected,pulse=None,baseline_angle=None):
                owner.status_calls.append((status,expected,pulse,baseline_angle))
        port=Port();original=port._check_status
        self.guard=CurrentSceneExternalGuard(self.repo,self.binding,self.recorder,port,self.store,clock=self.clock)
        self.review();self.guard.validate_command(self.command('up'))
        port._check_status(self.controller.status(),'ARMED',self.command('up'))
        self.assertEqual(len(self.status_calls),1)
        self.guard.disarm()
        self.assertIs(port._check_status.__self__,original.__self__)
        self.assertIs(port._check_status.__func__,original.__func__)

    def test_wrapped_original_monitor_rejection_is_preserved(self):
        def rejected(*args):raise ValueError('original_monitor_rejected')
        self.port._check_status=rejected
        self.review();self.guard.validate_command(self.command('up'))
        with self.assertRaisesRegex(ValueError,'original_monitor_rejected'):
            self.port._check_status(self.controller.status(),'ARMED',self.command('up'))
        self.guard.disarm();self.assertIs(self.port._check_status,rejected)

    def test_active_shoulder_motion_or_bad_feedback_refuses_existing_monitor_status(self):
        self.review();self.guard.validate_command(self.command('up'))
        status=self.controller.status();status['shoulderAngle']['active']=True
        with self.assertRaisesRegex(ValueError,'active_fixed_M4_changed'):
            self.port._check_status(status,'ARMED',self.command('up'))
        status=self.controller.status();status['shoulderAngle']['sensorFresh']=False
        with self.assertRaisesRegex(ValueError,'m4_feedback_not_ready'):
            self.port._check_status(status,'ARMED',self.command('up'))
        self.guard.disarm()

    def test_duplicate_validation_does_not_install_nested_wrapper(self):
        original=self.port._check_status;self.review()
        self.guard.validate_command(self.command('up'));wrapped=self.port._check_status
        with self.assertRaisesRegex(ValueError,'already_active'):
            self.guard.validate_command(self.command('up'))
        self.assertIs(self.port._check_status,wrapped)
        self.assertIs(self.guard._status_original,original)
        self.guard.disarm();self.assertIs(self.port._check_status,original)

    def test_prior_command_rejection_leaves_port_untouched(self):
        original=self.port._check_status;self.review_external(delta=7.)
        with self.assertRaisesRegex(ValueError,'direction_clearance'):
            self.guard.validate_command(self.command('up'))
        self.assertIs(self.port._check_status,original)
        self.assertIsNone(self.guard._status_wrapper)
        self.assertFalse(self.guard.active)

    def test_disarm_does_not_overwrite_another_status_wrapper(self):
        self.review();self.guard.validate_command(self.command('up'))
        other=lambda *args:None
        self.port._check_status=other
        with self.assertRaisesRegex(ValueError,'status_guard_replaced'):self.guard.check()
        self.guard.disarm()
        self.assertIs(self.port._check_status,other)
        self.assertFalse(self.guard.active)


if __name__=='__main__':unittest.main()
