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

from learning.approach_external_guard import ApproachExternalGuard, SCHEMA
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


if __name__=='__main__':unittest.main()
