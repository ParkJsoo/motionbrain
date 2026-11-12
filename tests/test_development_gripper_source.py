"""새 관측기의 fresh 원본·단발 수집 연결. 모든 장치는 모의 객체다."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2

from learning.development_gripper_source import load_observer, scene_context, observe_bound_frame, observe_saved_pair
from learning.development_gripper_check import run_check
from learning.development_gripper_observer import DevelopmentGripperObserver, LANDMARK_PREFIX, STATE_FIELDS
from learning.grasp_observation import GripperObservationSource
from learning.pipeline_observations import observe_transition
from learning.relative_action_learning import state_fields, validate_motion_scope
from test_development_gripper_observer import CONTEXT, ROIS, POINTS, image
from test_grasp_observation import Camera, Clock, BOOT


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


class DevelopmentSourceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.repo = Path(self.tmp.name); self.root = self.repo/'.local/source'; self.root.mkdir(parents=True)
        self.body = cv2.imencode('.jpg', image())[1].tobytes()
        self.root.joinpath('seed.jpg').write_bytes(self.body)
        ref = dict(id='seed', imageBytes=self.body, imageSha256=sha(self.body), points=POINTS, rois=ROIS)
        self.observer = DevelopmentGripperObserver(CONTEXT, [ref])
        self.scene = scene_context(self.observer)
        self.binding = dict(schemaVersion='motionbrain.development_gripper_source_binding.v1',
            context=CONTEXT, configurationSha256=self.observer.binding,
            references=[dict(id='seed', path='seed.jpg', sha256=sha(self.body), points=POINTS, rois=ROIS)])
        raw = json.dumps(self.binding).encode(); self.root.joinpath('binding.json').write_bytes(raw)
        self.binding_sha = sha(raw)
        for name, at, captured in [('frame', 10., 100), ('before', 10., 100), ('after', 26., 200)]:
            self.write_frame(name, at, captured)
        self.command = dict(joint='gripper', direction='close', percent=40, durationMs=350)
        self.root.joinpath('session.json').write_text('{"phase":"completed"}')
        self.row = dict(sessionId='collected', inputs=dict(image=dict(frameId='external-before'), command=self.command),
            targets=dict(image=dict(frameId='external-after')),
            evidence=dict(commandAt=20., offReceiptAt=22., cameraHostBounds=[0., 60.], cameraIdentity={'camera':'external'}),
            sourceHash=sha(self.root.joinpath('session.json').read_bytes()))

    def write_frame(self, name, at, captured):
        data = dict(sessionId='collected', hostClockId='collected', clockDomain='python_monotonic',
            status=200, method='GET', path='/capture', query={'fresh':'1'}, requestStartedAt=at,
            receivedAt=at+.1, persistedAt=at+.2, sha256=sha(self.body), headers={
                'X-MotionBrain-Camera-Boot-Id':BOOT,'X-MotionBrain-Captured-Us':str(captured),
                'X-MotionBrain-Capture-Requested-Us':str(captured-1),'X-MotionBrain-Frame-Id':f'{BOOT}:{captured}',
                'X-MotionBrain-Capture-Mode':'after_request','X-MotionBrain-Clock-Domain':'camera_boot_us',
                'X-MotionBrain-Timestamp-Semantics':'first_dma_buffer'})
        self.root.joinpath(f'esp32-{name}.jpg').write_bytes(self.body)
        self.root.joinpath(f'esp32-{name}.json').write_text(json.dumps(data))

    def mutate(self, name, operation):
        path = self.root/f'esp32-{name}.json'; data=json.loads(path.read_text()); operation(data)
        path.write_text(json.dumps(data))

    def observe(self, name='frame', **kwargs):
        return observe_bound_frame(self.observer, self.root, name, session_id='collected', camera_boot_id=BOOT,
            actual_scene=self.scene, now=kwargs.pop('now', 10.3), **kwargs)

    def test_fresh_source_is_bound_but_not_promoted(self):
        result = self.observe()
        self.assertTrue(result['sourceFreshMetadataVerified'])
        self.assertEqual(result['stateFields'], STATE_FIELDS)
        self.assertEqual(len(result['state']), 3)
        self.assertFalse(result['trainingEligible'])
        self.assertFalse(result['motionAdmissionVerified'])
        self.assertFalse(result['exposureSynchronizationVerified'])
        self.assertEqual(state_fields(self.scene), STATE_FIELDS)
        with self.assertRaisesRegex(ValueError, 'not_admitted'):
            validate_motion_scope(self.scene, self.command)

    def test_development_namespace_cannot_be_fitted_or_predicted(self):
        from learning.relative_action_learning import fit,predict
        from test_relative_action_learning import row
        rows=[row(0,-4.),row(1,4.)]
        model=fit(rows)
        for item in rows:item['context']=copy.deepcopy(self.scene)
        with self.assertRaisesRegex(ValueError,'not_admitted'):fit(rows)
        model['context']=copy.deepcopy(self.scene)
        with self.assertRaisesRegex(ValueError,'not_admitted'):
            predict(model,[0.,2.,10.],self.command,self.scene)

    def test_stale_future_or_invalid_age_is_refused(self):
        for now in (10.1, 16., float('nan')):
            with self.subTest(now=now), self.assertRaises(ValueError): self.observe(now=now)
        for age in (True, 0, 20, float('inf')):
            with self.subTest(age=age), self.assertRaises(ValueError): self.observe(max_age_seconds=age)

    def test_boot_clock_scene_or_nonfresh_mode_is_refused(self):
        with self.assertRaisesRegex(ValueError, 'scene_changed'):
            observe_bound_frame(self.observer,self.root,'frame',session_id='collected',camera_boot_id=BOOT,
                                actual_scene=dict(self.scene,poseRegime='other'),now=10.3)
        self.mutate('frame', lambda d:d.update(hostClockId='other'))
        with self.assertRaisesRegex(ValueError, 'clock_binding'): self.observe()
        self.write_frame('frame',10.,100)
        self.mutate('frame', lambda d:d['headers'].update({'X-MotionBrain-Camera-Boot-Id':'b'*32,
                                                               'X-MotionBrain-Frame-Id':'b'*32+':100'}))
        with self.assertRaisesRegex(ValueError, 'boot_changed'): self.observe()
        self.write_frame('frame',10.,100)
        self.mutate('frame', lambda d:d['headers'].update({'X-MotionBrain-Capture-Mode':'buffered'}))
        with self.assertRaisesRegex(ValueError, 'fresh_source_identity'): self.observe()

    def test_legacy_plain_capture_or_failed_http_is_refused(self):
        for change in ({'query':{}},{'status':503},{'method':'POST'}):
            self.write_frame('frame',10.,100); self.mutate('frame',lambda d:d.update(change))
            with self.subTest(change=change), self.assertRaisesRegex(ValueError,'fresh_capture_request'):
                self.observe()

    def test_changed_original_during_observation_is_refused(self):
        original = self.observer.observe
        def changed(*args, **kwargs):
            result = original(*args, **kwargs)
            self.mutate('frame', lambda d:d.update(receivedAt=10.15))
            return result
        with patch.object(self.observer,'observe',side_effect=changed):
            with self.assertRaisesRegex(ValueError,'source_bytes_changed'): self.observe()

    def test_duplicate_or_regressing_pair_does_not_become_two_samples(self):
        before = self.observe('before')
        self.write_frame('after',26.,100)
        with self.assertRaisesRegex(ValueError,'reused_or_regressed'):
            self.observe('after',now=26.3,previous=before)

    def test_saved_pair_preserves_planned_command_and_is_historical_only(self):
        original = copy.deepcopy(self.row)
        result=observe_saved_pair(self.observer,self.row,self.root,actual_scene=self.scene,camera_boot_id=BOOT)
        self.assertEqual(self.row,original)
        self.assertEqual(result['plannedCommand'],self.command)
        self.assertEqual(result['status'],'complete_development')
        self.assertTrue(result['savedReceiptReplayOnly'])
        self.assertFalse(result['currentFreshnessVerified'])
        self.assertFalse(result['currentFreshnessVerified'])
        self.assertFalse(result['trainingEligible'])
        self.mutate('before',lambda d:d.update(persistedAt=20.1))
        with self.assertRaisesRegex(ValueError,'input_outside_window'):
            observe_saved_pair(self.observer,self.row,self.root,actual_scene=self.scene,camera_boot_id=BOOT)

    def test_binding_tamper_and_configuration_mismatch_are_refused(self):
        observer=load_observer(self.root,'binding.json',self.binding_sha)
        self.assertEqual(observer.binding,self.observer.binding)
        with self.assertRaisesRegex(ValueError,'binding_hash'):
            load_observer(self.root,'binding.json','0'*64)
        self.binding['configurationSha256']='0'*64
        raw=json.dumps(self.binding).encode();self.root.joinpath('bad.json').write_bytes(raw)
        with self.assertRaisesRegex(ValueError,'configuration_binding'):
            load_observer(self.root,'bad.json',sha(raw))

    def test_pair_metadata_cannot_change_after_timing_audit(self):
        from learning.gripper_camera_dataset import bind_gripper_pair
        def changed(row, root):
            result=bind_gripper_pair(row,root)
            self.mutate('before',lambda d:d.update(requestStartedAt=20.3,receivedAt=20.4,persistedAt=20.5))
            return result
        with patch('learning.development_gripper_source.bind_gripper_pair',side_effect=changed):
            with self.assertRaisesRegex(ValueError,'pair_bytes_changed'):
                observe_saved_pair(self.observer,self.row,self.root,actual_scene=self.scene,camera_boot_id=BOOT)

    def test_existing_pipeline_stores_development_only_without_transition(self):
        # pair 루트는 assets와 같은 실제 폴더가 아닌 명시적 하위 경로여야 한다.
        pair=self.root/'pair';pair.mkdir()
        for name in ('before','after'):
            for suffix in ('json','jpg'):
                pair.joinpath(f'esp32-{name}.{suffix}').write_bytes(self.root.joinpath(f'esp32-{name}.{suffix}').read_bytes())
        spec=dict(kind='development_gripper',context=self.scene,captureGroup='development',gripperPairRoot='pair',
            observerBindingPath='binding.json',observerBindingSha256=self.binding_sha,cameraBootId=BOOT)
        original=self.root.joinpath('session.json').read_bytes()
        result=observe_transition(self.root,self.row,spec,artifact_root=self.root)
        self.assertEqual(result['status'],'development_only',result.get('reason'))
        self.assertIsNone(result['transition'])
        self.assertFalse(result['trainingEligible'])
        self.assertEqual(self.root.joinpath('session.json').read_bytes(),original)
        spec['context']=dict(self.scene,object='other')
        self.assertEqual(observe_transition(self.root,self.row,spec,artifact_root=self.root)['status'],'rejected')

    def config(self):
        data=dict(schemaVersion='motionbrain.development_gripper_source_check.v1',assetRoot=str(self.root),
            observerBindingPath='binding.json',observerBindingSha256=self.binding_sha,cameraBootId=BOOT,
            sessionId='collected',actualScene=self.scene,maxAgeSeconds=5.,cameraUrl='http://camera.invalid')
        path=self.root/'config.json';path.write_text(json.dumps(data));return path

    def test_default_saved_check_never_constructs_capture_source(self):
        with patch('learning.development_gripper_check.GripperObservationSource') as validation:
            result=run_check(self.config(),self.repo/'.local/replay',frame_root=self.root,root=self.repo,
                             source_factory=lambda *a,**k:self.fail('capture source used'))
        self.assertEqual(result['status'],'checked_readonly_development',result)
        self.assertEqual(result['newCameraCaptures'],0)
        self.assertTrue(result['savedReceiptReplayOnly'])

    def test_explicit_capture_gets_once_and_closes_without_motion(self):
        clock=Clock();camera=Camera(clock);camera.body=self.body
        def factory(url,session,boot,scene,store,**kwargs):
            return GripperObservationSource(url,session,boot,scene,store,opener=camera,**kwargs)
        result=run_check(self.config(),self.repo/'.local/capture',capture=True,root=self.repo,
                         clock=clock,source_factory=factory)
        self.assertEqual(result['status'],'checked_readonly_development',result)
        self.assertEqual(camera.calls[0][1],'GET')
        self.assertEqual(len(camera.calls),1)
        self.assertEqual(result['newCameraCaptures'],1)
        self.assertTrue(result['currentFreshnessVerified'])
        self.assertEqual(result['hardwareMotionCommands'],0)
        self.assertFalse(result['trainingEligible'])

    def test_invalid_config_is_refused_before_capture(self):
        path=self.config();d=json.loads(path.read_text());d['maxAgeSeconds']=False;path.write_text(json.dumps(d))
        with self.assertRaisesRegex(ValueError,'age_policy'):
            run_check(path,self.repo/'.local/bad',capture=True,root=self.repo,
                      source_factory=lambda *a,**k:self.fail('capture source used'))

    def test_existing_finite_collector_pair_connects_without_repeating_command(self):
        from tests.test_pipeline_collection import PipelineCollectionTests
        fixture=PipelineCollectionTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        cameras=[]
        def factory(session_id,store):
            camera=Camera(fixture.manager.clock);camera.body=self.body;cameras.append(camera)
            return GripperObservationSource('http://camera.invalid',session_id,BOOT,self.scene,store,
                max_captures=2,clock=fixture.manager.clock,opener=camera)
        collected=fixture.collect(request='new-observer-pair',gripper_source_factory=factory)
        self.assertEqual(collected['status'],'recorded')
        source=Path(collected['sessionPath'])/'session.json';original=source.read_bytes();session=json.loads(original)
        event=next(e for e in session['events'] if e['kind']=='command')
        receipt=next(e['receipt'] for e in session['events'] if e['kind']=='result')
        row=dict(sessionId=session['sessionId'],sourceHash=sha(original),
            inputs=dict(image={'frameId':'external-before'},command=collected['request']['command']),
            targets=dict(image={'frameId':'external-after'}),
            evidence=dict(cameraIdentity={'camera':'external'},cameraHostBounds=[fixture.manager.started,session['endedAt']],
                          commandAt=event['at'],offReceiptAt=receipt['offAt']))
        result=observe_saved_pair(self.observer,row,Path(collected['gripperPair']['path']),
            actual_scene=self.scene,camera_boot_id=BOOT)
        self.assertEqual(result['status'],'complete_development')
        self.assertEqual(result['plannedCommand'],collected['request']['command'])
        self.assertEqual(collected['gripperPair']['before']['context']['stateFields'],STATE_FIELDS)
        self.assertEqual(source.read_bytes(),original)
        calls=len(fixture.requests)
        self.assertEqual(fixture.collect(request='new-observer-pair',gripper_source_factory=factory),collected)
        self.assertEqual(len(fixture.requests),calls)
        self.assertEqual(len(cameras),1)
        self.assertEqual(len(cameras[0].calls),2)


if __name__=='__main__':unittest.main()
