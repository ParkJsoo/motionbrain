"""모델 좌표 분리가 현장 보호·원본 실패를 가리지 않는지 검사한다."""
import copy
from types import SimpleNamespace
import unittest
from unittest.mock import patch,Mock
from learning.scene_relative_cycle_source import SceneRelativeCycleSource
from learning.current_full_cycle_source import FullCycleSource
from tests.test_current_full_cycle_source import SourceTests


class SceneSourceTests(SourceTests):
    def setUp(self):
        super().setUp()
        self.port.__class__=SceneRelativeCycleSource
        self.port.bank=SimpleNamespace(is_scene_stage=lambda k:False)
        self.port.sceneRegistrations={}

    # Legacy raw parser/freshness/clock/epoch tests above run unchanged on the wrapper.
    def test_preflight_reports_each_missing_stage_without_factories_or_IO(self):
        from learning.current_full_cycle_source import SourcePreflightError
        self.port.bank.verify_cached=Mock(side_effect=ValueError('model_missing'))
        stages=[dict(kind=k,context={},modelSha256='a'*64) for k in ('approach','close','lift','lower','release')]
        with self.assertRaises(SourcePreflightError) as result:
            self.port.preflight(dict(stages=stages),{})
        self.assertEqual(set(result.exception.missing),{s['kind'] for s in stages})
        self.factory.assert_not_called()

    def test_scene_safety_measurement_is_not_replaced_with_model_state(self):
        self.port.bank.is_scene_stage=lambda k:True
        stage=dict(kind='close',context={})
        context=dict(physicalView='current',object='target',poseRegime='table')
        spec=dict(sourceContext=dict(context,gripperCameraBootId='b'*32))
        tracker=SimpleNamespace(registration=dict(sceneContext=context),shaft_landmark_contract='shaft:test')
        actual=dict(sourceFreshMetadataVerified=True,cameraBootId='b'*32,state=[100.])
        self.port._validate_safety_state(stage,actual,{},tracker,spec)
        for changed in (dict(actual,sourceFreshMetadataVerified=False),dict(actual,cameraBootId='c'*32)):
            with self.assertRaisesRegex(ValueError,'safety_raw'):
                self.port._validate_safety_state(stage,changed,{},tracker,spec)
        with self.assertRaisesRegex(ValueError,'safety_context'):
            self.port._validate_safety_state(stage,actual,{},tracker,
                dict(sourceContext=dict(spec['sourceContext'],poseRegime='other')))

    def test_failed_safety_audit_cannot_be_bypassed_by_model_observer(self):
        self.port.bank.is_scene_stage=lambda k:True
        observer=Mock();self.port.sceneRegistrations={'close':observer}
        with patch.object(FullCycleSource,'_audit',side_effect=ValueError('real_safety_rejected')):
            with self.assertRaisesRegex(ValueError,'real_safety_rejected'):
                self.port._audit(dict(kind='close'),self.root,{}, {},'producer',195.)
        observer.observe.assert_not_called()

    def test_physical_contract_is_bound_into_record_digest(self):
        self.port.bank.is_scene_stage=lambda k:True
        self.port.bank.models={'close':dict(physicalContext={'object':'target'},observationContract={'canonicalFrameSha256':'a'*64})}
        self.port.sceneRegistrations={'close':SimpleNamespace(registration={'registrationSha256':'b'*64})}
        result=self.port._record_context(dict(kind='close'))
        result['physicalContext']['object']='tampered'
        self.assertEqual(self.port.bank.models['close']['physicalContext']['object'],'target')
        self.assertEqual(result['sceneRegistrationSha256'],'b'*64)

    def test_actual_scene_observer_changes_model_state_only(self):
        from tests.test_scene_relative_observation import RelativeObservationTests,encoded
        from learning.scene_relative_observation import SceneRelativeObserver
        from hashlib import sha256
        fixture=RelativeObservationTests();fixture.setUp()
        image,_=fixture.transformed(1.2,(10,10),dict(object=(0,8),jawLeft=(6,0),jawRight=(-6,0)))
        body=encoded(image);folder=self.root/'scene';folder.mkdir()
        (folder/'esp32-current.jpg').write_bytes(body)
        observer=SceneRelativeObserver(fixture.reg,fixture.raw)
        self.port.sceneRegistrations={'close':observer};self.port.bank.is_scene_stage=lambda k:True
        safety={'shaftSeparationPx':88.,'sourceFreshMetadataVerified':True}
        frame={'frameId':'a'*32+':current','imageSha256':sha256(body).hexdigest()}
        with patch.object(FullCycleSource,'_audit',return_value=(['audited-original-frames'],{'shaftSeparationPx':88.},safety)), \
             patch('learning.scene_relative_cycle_source.read_frame',return_value=(frame,{},'a'*32)):
            frames,state,returned=self.port._audit(dict(kind='close'),self.root,{},dict(root='scene',label='current'),'producer',195.38)
        self.assertIs(returned,safety)
        self.assertEqual(frames,['audited-original-frames'])
        self.assertEqual(len(state),4);self.assertEqual(state[-1],195.38)
        self.assertLess(state[2],120.)
