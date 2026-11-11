"""집게 3점의 파일 전용 결합과 실물 진입 차단 회귀."""
import copy
import json
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.batch_runtime import BatchRuntime, _gripper_spec, audit_pair_transition, sha, validate_runtime, observe_gripper_frame_offline
from learning.batch_observation import digest
from learning.experiment_pipeline import encoded
from learning.registered_gripper_landmarks import register
from tests import test_batch_runtime as runtime_fixture
from learning.gripper_camera_dataset import read_frame


class BatchGripperPointsTests(unittest.TestCase):
    def setUp(self):
        f = runtime_fixture.BatchRuntimeTests(); f.setUp(); self.addCleanup(f.doCleanups)
        self.f = f
        self.runtime, self.source, self.row = f.paired_runtime()
        image = np.random.default_rng(19).integers(10, 245, (240, 320, 3), dtype=np.uint8)
        self.raw = cv2.imencode('.jpg', image)[1].tobytes()
        self.registration = register(self.raw, dict(object=[158,98],jawLeft=[121,135],jawRight=[193,135]))
        (f.private/'points.json').write_bytes(encoded(self.registration))
        (f.private/'reference.jpg').write_bytes(self.raw)
        self.wrapper = dict(kind='registered_gripper_points_stationary',registrationPath='.local/points.json',
            registrationSha256=sha(encoded(self.registration)),referenceImagePath='.local/reference.jpg',
            referenceImageSha256=sha(self.raw),context={k:f.spec[k] for k in ('viewId','poseRegime','object','load')},
            maxAgeSeconds=10)
        self.bind()
        for side in ('before','after'):
            self.image(side, self.raw)
        frame, metadata, boot = read_frame(
            self.source/'gripper-pair','before','session')
        (self.source/'batch-gripper-before.json').write_bytes(encoded(dict(frame=frame)))

    def bind(self):
        path=self.f.private/'gripper-points.json';path.write_bytes(encoded(self.wrapper))
        self.runtime.config['gripper'].update(observerSpecPath='.local/gripper-points.json',observerSpecSha256=sha(path.read_bytes()))
        self.runtime.gripper_spec = _gripper_spec(self.f.repository,self.runtime.config,self.f.spec)

    def image(self,side,raw):
        folder=self.source/'gripper-pair'
        (folder/('esp32-'+side+'.jpg')).write_bytes(raw)
        path=folder/('esp32-'+side+'.json');meta=json.loads(path.read_bytes())
        meta['sha256']=sha(raw);path.write_bytes(encoded(meta))

    def test_pair_fusion_and_original_reaudit_preserve_semantics(self):
        runtime=self.runtime
        # This fixture supplies external gripper fields; other joints remain external-only.
        runtime.spec['joints']['gripper']=copy.deepcopy(runtime.spec['joints']['wrist'])
        runtime.spec['joints']['gripper']['nodes'][0]['id']='gripper-node'
        self.f.spec_path.write_bytes(encoded(runtime.spec))
        runtime.config['observerSpecSha256']=sha(self.f.spec_path.read_bytes())
        paired=runtime._pair_transition(self.source,self.row)
        observed=dict(status='accepted',specSha256=digest(runtime.spec),states={'gripper':dict(state=[30,30,60,50])})
        fields,values=runtime._states(observed,paired['beforeObservation'],'gripper')
        self.assertEqual(len(values),7)
        self.assertEqual(fields[-1],'gripper:projectedJawSeparationPx')
        np.testing.assert_allclose(values[-3:],[1,-37,72],atol=.1)
        transition=dict(joint='gripper',before=dict(observation=observed,derivedMetadata=dict(clockId=runtime.clock_id)),
            after=dict(observation=copy.deepcopy(observed)),gripperPair=paired)
        audited=audit_pair_transition(self.source,self.row,transition,self.f.repository)
        self.assertEqual(audited['beforeState'],values)
        self.assertEqual(audited['stateFields'],fields)
        self.assertIn('gripperObserverSha256',audited['extraContext'])
        # Synthetic plumbing check only: these are not real motion/training evidence.
        from tests.test_batch_learning import row
        from learning.batch_learning import fit, predict
        rows=[]
        for i in (1,2):
            item=row(i);item.update(joint='gripper',context=runtime._context(),stateFields=fields,
                before=values,after=audited['afterState'],command=dict(joint='gripper',direction='open',percent=20,durationMs=200))
            rows.append(item)
        model=fit(rows)
        self.assertEqual(len(predict(model,values,rows[0]['command'],runtime._context())),7)
        self.assertFalse(model['controlEligible'])
        obs=paired['beforeObservation']
        for flag in ('trainingEligible','motionTrackingVerified','controlEligible'):
            self.assertFalse(obs[flag])
        changed=copy.deepcopy(transition);changed['gripperPair']['afterObservation']['states']['gripper']['state'][0]+=1
        with self.assertRaisesRegex(ValueError,'paired_transition_changed'):
            audit_pair_transition(self.source,self.row,changed,self.f.repository)

    def test_public_offline_api_reads_saved_pixels_without_live_services(self):
        with patch('learning.batch_runtime.GripperObservationSource',side_effect=AssertionError('camera')), \
             patch('learning.batch_runtime.SequencePort',side_effect=AssertionError('controller')):
            observed=observe_gripper_frame_offline(self.f.repository,self.f.spec,self.runtime.config['gripper'],
                self.source/'gripper-pair','before','session')
        self.assertEqual(observed['status'],'accepted')
        self.assertEqual(observed['frameBinding']['pixelSha256'],observed['annotation']['pixelSha256'])
        frame,_,_=read_frame(self.source/'gripper-pair','before','session')
        self.assertEqual(observed['frameBinding']['pixelSha256'],frame['pixelSha256'])
        self.assertEqual(self.f.credentials,[])
        self.assertFalse(observed['trainingEligible'])
        (self.f.private/'reference.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'hash_changed'):
            observe_gripper_frame_offline(self.f.repository,self.f.spec,self.runtime.config['gripper'],
                self.source/'gripper-pair','before','session')

    def test_jaw_only_contract_fuses_five_states_and_remains_dispatch_blocked(self):
        from learning.registered_gripper_landmarks import register_jaws
        self.registration=register_jaws(self.raw,{k:v for k,v in self.registration['points'].items() if k!='object'},self.registration['parameters'])
        (self.f.private/'points.json').write_bytes(encoded(self.registration))
        self.wrapper['registrationSha256']=sha(encoded(self.registration))
        self.wrapper['kind']='registered_gripper_jaws_stationary';self.bind()
        obs=observe_gripper_frame_offline(self.f.repository,self.f.spec,self.runtime.config['gripper'],
            self.source/'gripper-pair','before','session')
        self.assertEqual(len(obs['states']['gripper']['state']),5)
        self.assertEqual(set(obs['points']),{'jawLeft','jawRight'})
        plan=copy.deepcopy(self.f.plan);plan['runtime']=copy.deepcopy(self.runtime.config)
        with self.assertRaisesRegex(ValueError,'stationary_only'):validate_runtime(plan,self.f.repository)

    def test_live_runtime_blocked_before_device_or_credential_access(self):
        plan=copy.deepcopy(self.f.plan);plan['runtime']=copy.deepcopy(self.runtime.config)
        with patch('learning.batch_runtime.GripperObservationSource',side_effect=AssertionError('camera')) as camera, \
             patch('learning.batch_runtime.SequencePort',side_effect=AssertionError('controller')) as controller:
            with self.assertRaisesRegex(ValueError,'registered_gripper_observer_stationary_only'):
                validate_runtime(plan,self.f.repository)
            with self.assertRaisesRegex(ValueError,'registered_gripper_observer_stationary_only'):
                BatchRuntime(plan,self.f.repository,self.f.private/'blocked',self.f.credential)
        camera.assert_not_called();controller.assert_not_called()
        self.assertEqual(self.f.credentials,[])

    def test_occlusion_and_changed_resolution_reject_pair(self):
        for image in (np.full((240,320,3),127,np.uint8),np.zeros((200,320,3),np.uint8)):
            self.image('after',cv2.imencode('.jpg',image)[1].tobytes())
            with self.assertRaises(ValueError):
                self.runtime._pair_transition(self.source,self.row)

    def test_stale_and_rebooted_input_rejected(self):
        self.runtime.clock.now=21
        with self.assertRaisesRegex(ValueError,'input_stale'):
            self.runtime._observe_gripper(self.source/'gripper-pair','before','session')
        self.runtime.config['gripper']['bootId']='b'*32
        with self.assertRaisesRegex(ValueError,'boot_changed'):
            self.runtime._observe_gripper(self.source/'gripper-pair','before','session',historical=True)

    def test_context_and_reference_tamper_rejected(self):
        for field in ('viewId','object','poseRegime','load'):
            original=self.wrapper['context'][field];self.wrapper['context'][field]='different'
            with self.assertRaisesRegex(ValueError,'context_changed'):self.bind()
            self.wrapper['context'][field]=original
        self.bind()
        (self.f.private/'reference.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'hash_changed'):self.bind()


if __name__ == '__main__': unittest.main()
