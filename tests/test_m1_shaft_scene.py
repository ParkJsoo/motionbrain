"""모의 fresh JPEG와 저장 외부 원본으로 M1 관측·보호 계약을 검사한다."""
import copy
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.artifacts import ArtifactStore
from learning.grasp_observation import GripperObservationSource
from learning.m1_shaft_scene import (M1ShaftSceneGuard,M1ShaftSourceObserver,SCHEMA,MONITOR_POLICY,REFERENCE_MEANING)
from learning.registered_gripper_crosssection import register
from tests import test_approach_external_guard as external_tests
from tests.test_grasp_observation import Camera


class M1ShaftSceneTests(unittest.TestCase):
    write=external_tests.CurrentSceneGuardTests.write
    write_json=external_tests.CurrentSceneGuardTests.write_json

    def setUp(self):
        external_tests.CurrentSceneGuardTests.setUp(self)
        self.shaft_image=np.full((240,320,3),220,np.uint8)
        self.shaft_image[67:133,140:206]=np.random.default_rng(42).integers(40,220,(66,66,3),dtype=np.uint8)
        self.shaft_image[40:67,76:103]=20;self.shaft_image[40:67,220:247]=20
        self.shaft_raw=cv2.imencode('.jpg',self.shaft_image)[1].tobytes()
        scene=dict(physicalView='current-M1-synthetic',object='synthetic-pink-prism',poseRegime='M1-only-M4-fixed')
        registration=register(self.shaft_raw,[166,95],scene)
        self.config.update(schemaVersion=SCHEMA,executionTimingPolicy=MONITOR_POLICY,referenceMeaning=REFERENCE_MEANING,
            actualScene=scene,shaftRegistration=self.write_json('M1-registration.json',registration),
            shaftReference=self.write('M1-reference.jpg',self.shaft_raw))
        self.binding=self.write_json('M1-config.json',self.config)
        self.guard=M1ShaftSceneGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)
        self.camera=Camera(self.clock);self.camera.boot='b'*32;self.camera.body=self.shaft_raw
        self.source=GripperObservationSource('http://camera.invalid','trial','b'*32,self.guard.scene_context,
            ArtifactStore(self.repo/'.local/M1-gripper'),clock=self.clock,opener=self.camera)
        self.observer=M1ShaftSourceObserver(self.source,self.guard,clock=self.clock)

    def capture(self):
        self.observer.start(deadline=60.)
        return self.observer.capture(deadline=60.,tick=lambda:None)

    def review(self,record,result):
        return self.guard.review(record,result,deadline=60.,tick=lambda:None)

    @staticmethod
    def command(direction='close',duration=350,percent=40):
        return dict(joint='gripper',direction=direction,percent=percent,durationMs=duration)

    def test_constructor_does_not_start_camera_or_change_port(self):
        self.assertEqual(self.camera.calls,[])
        self.assertEqual(self.calls,[])
        self.assertIsNone(self.guard._status_wrapper)

    def test_fresh_shaft_source_and_external_proof_without_M3_span_or_contact_claim(self):
        record,result=self.capture()
        self.assertEqual(result['context']['stateFields'],['shaftSeparationPx'])
        self.assertNotIn('executionTimingPolicy',result['context'])
        self.assertGreater(result['shaftSeparationPx'],140)
        self.assertNotEqual(result['shaftSeparationPx'],124)
        self.assertFalse(result['contactVerified']);self.assertFalse(result['graspVerified'])
        proof=self.review(record,result)
        self.assertTrue(proof['accepted'])
        self.assertEqual(self.calls.count('http'),1)

    def test_M1_change_in_shaft_is_measured_without_fixed_124_span_rejection(self):
        _,first=self.capture()
        changed=self.shaft_image.copy();changed[40:67,60:120]=220;changed[40:67,80:107]=20
        self.camera.body=cv2.imencode('.jpg',changed)[1].tobytes()
        record,second=self.observer.capture(deadline=60.,tick=lambda:None)
        self.assertLess(second['shaftSeparationPx'],first['shaftSeparationPx']-3.5)
        self.assertTrue(self.review(record,second)['accepted'])

    def test_allowed_CLOSE_200_350_and_OPEN_350_only(self):
        record,result=self.capture();self.review(record,result)
        for command in [self.command(duration=200),self.command(),self.command('open')]:
            self.guard.validate_command(command);self.guard.disarm()
        for command in [self.command('open',200),self.command(duration=650),self.command(percent=60),
                        dict(joint='elbow',direction='up',percent=60,durationMs=350)]:
            with self.subTest(command=command),self.assertRaisesRegex(ValueError,'bounded_command'):
                self.guard.validate_command(command)

    def test_active_M4_monitor_has_no_additional_HTTP_and_restores_original(self):
        record,result=self.capture();self.review(record,result)
        original=self.port._check_status;before=self.calls.count('http')
        self.guard.validate_command(self.command());self.controller.angle=196.8
        try:
            with self.assertRaisesRegex(ValueError,'active_fixed_M4'):
                self.port._check_status(self.controller.status(),'ARMED',self.command())
        finally:self.guard.disarm()
        self.assertIs(self.port._check_status,original)
        self.assertEqual(self.calls.count('http'),before)

    def test_forearm_or_object_movement_over_three_pixels_blocks_next_command(self):
        record,result=self.capture()
        self.review(record,result)
        external=dict(forearmCenter=[1031.51,432.],objectBox=[550,650,601,751])
        with patch.object(self.guard,'_measure',return_value=external):
            with self.assertRaisesRegex(ValueError,'forearm_fixed_pose'):self.review(record,result)
        with self.assertRaisesRegex(ValueError,'observation_scope_failed'):self.guard.validate_command(self.command())

    def test_object_movement_blocks_next_command(self):
        record,result=self.capture();self.review(record,result)
        external=dict(forearmCenter=[1028.5,432.],objectBox=[554,650,605,751])
        with patch.object(self.guard,'_measure',return_value=external):
            with self.assertRaisesRegex(ValueError,'fixed_object_extent'):self.review(record,result)
        with self.assertRaisesRegex(ValueError,'observation_scope_failed'):self.guard.validate_command(self.command())

    def test_changed_state_cannot_replace_actual_source_measurement(self):
        record,result=self.capture();changed=copy.deepcopy(result);changed['state'][0]-=2
        with self.assertRaisesRegex(ValueError,'bound_observation_changed'):self.review(record,changed)
        with self.assertRaisesRegex(ValueError,'observation_scope_failed'):self.guard.validate_command(self.command())

    def test_source_record_timing_cannot_be_relabelled(self):
        record,result=self.capture()
        record['sourceEarliest']-=.1
        with self.assertRaisesRegex(ValueError,'bound_observation_changed'):self.review(record,result)

    def test_stale_or_reused_source_and_post_OFF_timing_are_rejected(self):
        record,result=self.capture()
        with self.assertRaisesRegex(ValueError,'source_reused'):self.observer.inspect(record)
        with self.assertRaisesRegex(ValueError,'observation_scope_failed'):self.guard.validate_command(self.command())

    def test_stale_frame_cannot_be_fresh_by_reinspection(self):
        record,result=self.capture()
        self.clock.sleep(3.1)
        with self.assertRaisesRegex(ValueError,'source_stale'):self.review(record,result)

    def test_post_OFF_requires_three_seconds_without_shifted_timestamp(self):
        self.observer.start(deadline=60.)
        record=self.source.capture(deadline=60.)
        with self.assertRaisesRegex(ValueError,'post_OFF_window'):
            self.observer.inspect(record,after_off=self.clock.monotonic())

    def test_missing_shaft_does_not_get_filled_from_other_jaw(self):
        changed=self.shaft_image.copy();changed[35:90,205:260]=220
        self.camera.body=cv2.imencode('.jpg',changed)[1].tobytes()
        with self.assertRaisesRegex(ValueError,'crosssection_rows_insufficient'):self.capture()


if __name__=='__main__':unittest.main()
