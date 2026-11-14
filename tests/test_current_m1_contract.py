"""새 현재 M1 seed·자세·ARMED OFF와 원본 보존의 모의 장치 검사."""
import copy
import unittest

import cv2
from learning.current_m1_contract import CurrentM1CollectionGuard,SCHEMA,REFERENCE_MEANING,COMMANDS
from learning.m1_shaft_scene import M1ShaftSceneGuard,M1ShaftSourceObserver
from learning.grasp_observation import GripperObservationSource
from learning.artifacts import ArtifactStore
from learning.registered_gripper_crosssection import register,default_parameters
from learning.supervised_transport import SequencePort
from tests import test_m1_shaft_scene as fixtures


class CurrentM1ContractTests(unittest.TestCase):
    write=fixtures.M1ShaftSceneTests.write
    write_json=fixtures.M1ShaftSceneTests.write_json
    capture=fixtures.M1ShaftSceneTests.capture
    review=fixtures.M1ShaftSceneTests.review

    def setUp(self):
        fixtures.M1ContactObjectFragmentTests.setUp(self)
        self.old_config=copy.deepcopy(self.config)
        self.old_registration=copy.deepcopy(self.guard.tracker.registration)
        self.old_reference=self.guard.tracker.reference_image
        self.original_files={str(path):path.read_bytes() for path in self.assets.iterdir() if path.is_file()}
        self.tracking.update(expectedM4=195.29,maximumM4Difference=.25)
        scene=dict(physicalView='current-seed',object='current-cohort',poseRegime='M4-19529')
        image=self.shaft_image.copy()
        image[70:116,65:140]=220;image[70:116,210:255]=220
        image[70:116,110:137]=20;image[70:116,210:237]=20
        raw=cv2.imencode('.jpg',image)[1].tobytes()
        registration=register(raw,[166,95],scene,default_parameters(preset='current_m1_observed_shaft95'))
        self.config.update(schemaVersion=SCHEMA,referenceMeaning=REFERENCE_MEANING,referenceM4Deg=195.29,
            actualScene=scene,tracking=self.write_json('current-tracking.json',self.tracking),
            shaftRegistration=self.write_json('current-registration.json',registration),
            shaftReference=self.write('current-reference.jpg',raw))
        self.binding=self.write_json('current-config.json',self.config)
        self.controller.angle=195.29
        self.guard=CurrentM1CollectionGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)
        self.camera.body=raw
        self.source=GripperObservationSource('http://camera.invalid','trial','b'*32,self.guard.scene_context,
            ArtifactStore(self.repo/'.local/current-M1-camera'),clock=self.clock,opener=self.camera)
        self.observer=M1ShaftSourceObserver(self.source,self.guard,clock=self.clock)

    def armed(self,direction='close'):
        self.controller.state='ARMED';self.port._active_pulse=copy.deepcopy(COMMANDS[0 if direction=='close' else 1])
        self.port.last_sequence=17;self.port.last_outputs={i:0 for i in range(1,6)}
        self.port._status=lambda:self.controller.status()
        self.port._queue=lambda count:self.assertEqual(count,1)
        self.port._outputs=lambda:None
        self.port._check_status=lambda status,expected,pulse=None,baseline_angle=None:SequencePort._check_status(
            self.port,status,expected,pulse,baseline_angle)

    def test_new_baseline_and_context_use_existing_source_without_success_claim(self):
        record,result=self.capture();proof=self.review(record,result)
        self.assertTrue(proof['accepted']);self.assertEqual(self.guard._expected_m4(),195.29)
        self.assertEqual(result['context']['object'],'current-cohort')
        self.assertFalse(result['contactVerified']);self.assertFalse(result['graspVerified'])
        self.assertTrue(66 <= result['shaftSeparationPx'] <=139.5)
        self.assertEqual(self.guard._forearm_limit(),3);self.assertEqual(self.guard._object_box_limit(),8)
        self.assertEqual(self.old_config['actualScene'],self.old_registration['sceneContext'])
        self.assertNotEqual(self.old_registration['sceneContext'],self.guard.tracker.registration['sceneContext'])
        from pathlib import Path
        self.assertTrue(all(Path(path).read_bytes()==raw for path,raw in self.original_files.items()))

    def test_old_guard_does_not_accept_current_schema_and_reference_is_not_ready(self):
        with self.assertRaisesRegex(ValueError,'configuration'):
            M1ShaftSceneGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)
        self.assertIn('not_close_ready_contact_or_grasp_success',self.guard.reference_meaning)
        self.assertEqual(self.camera.calls,[]);self.assertEqual(self.calls,[])

    def test_exact_close_open_two_each_and_monitor_restore(self):
        record,result=self.capture();self.review(record,result);original=self.port._check_status
        for command in (*COMMANDS,*COMMANDS):
            self.guard.validate_command(command);self.guard.disarm()
            self.assertIs(self.port._check_status,original)
        with self.assertRaisesRegex(ValueError,'direction_budget'):self.guard.validate_command(COMMANDS[0])
        self.assertTrue(self.guard._failed)

    def test_bad_command_sticky_rejection(self):
        record,result=self.capture();self.review(record,result)
        with self.assertRaisesRegex(ValueError,'exact_command'):
            self.guard.validate_command(dict(COMMANDS[0],durationMs=450))
        with self.assertRaisesRegex(ValueError,'scope_failed'):self.guard.validate_command(COMMANDS[0])

    def test_armed_current_OFF_review_and_wrong_boot_rejection(self):
        record,result=self.capture();self.armed('open')
        proof=self.guard.review_armed(record,result,expected_sequence=17,deadline=60,tick=lambda:None)
        self.assertTrue(proof['accepted']);self.assertIsNone(self.guard._armed_review_sequence)
        self.port.boot_id='d'*32
        with self.assertRaisesRegex(ValueError,'identity_changed'):
            self.guard.review_armed(record,result,expected_sequence=17,deadline=60,tick=lambda:None)
        self.assertTrue(self.guard._failed)

    def test_armed_nonzero_or_changed_cursor_not_accepted(self):
        record,result=self.capture();self.armed();self.controller.speeds[1]=1
        with self.assertRaisesRegex(ValueError,'unexpected_motor_output'):
            self.guard.review_armed(record,result,expected_sequence=17,deadline=60,tick=lambda:None)

    def test_bad_reference_angle_or_relaxed_pose_rejected(self):
        for value in [True,float('nan'),127.58,296.12]:
            bad=copy.deepcopy(self.config);bad['referenceM4Deg']=value
            binding=self.write_json('bad-current-config.json',bad)
            with self.assertRaises(ValueError):
                CurrentM1CollectionGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)
        self.tracking['maximumM4Difference']=.5
        self.config['tracking']=self.write_json('bad-current-tracking.json',self.tracking)
        binding=self.write_json('bad-current-config.json',self.config)
        with self.assertRaisesRegex(ValueError,'quarter_degree'):
            CurrentM1CollectionGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_armed_cursor_advance_and_shaft_fence_are_not_relaxed(self):
        record,result=self.capture();self.armed()
        self.port._outputs=lambda:setattr(self.port,'last_sequence',18)
        with self.assertRaisesRegex(ValueError,'output_changed'):
            self.guard.review_armed(record,result,expected_sequence=17,deadline=60,tick=lambda:None)
        self.assertTrue(self.guard._failed)

    def test_current_shaft_bounds_reject_before_cached_review(self):
        record,result=self.capture();result['shaftSeparationPx']=65.9
        with self.assertRaisesRegex(ValueError,'shaft_fence'):self.review(record,result)
        self.assertTrue(self.guard._failed)
