"""모의 fresh JPEG와 저장 외부 원본으로 M1 관측·보호 계약을 검사한다."""
import copy
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.artifacts import ArtifactStore
from learning.grasp_observation import GripperObservationSource
from learning.m1_shaft_scene import (M1ShaftSceneGuard,M1ShaftSourceObserver,SCHEMA,MONITOR_POLICY,
    REFERENCE_MEANING,OBJECT_FRAGMENT_POLICY,OBSERVED95_LOCAL_COMMAND_POLICY,OBSERVED95_LOWER_COMMAND_POLICY,
    OBSERVED95_GRASP650_COMMAND_POLICY,CONTACT_FRAGMENT_POLICY)
from learning.approach_external_guard import ApproachExternalGuard
from learning.registered_gripper_crosssection import register, default_parameters
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

    def test_explicit_CLOSE_40_200_350_and_CLOSE_OPEN_60_650_OPEN_40_350_only(self):
        record,result=self.capture();self.review(record,result)
        for command in [self.command(duration=200),self.command(),self.command(duration=650,percent=60),
                        self.command('open'),self.command('open',650,60)]:
            self.guard.validate_command(command);self.guard.disarm()
        for command in [self.command('open',200),self.command(duration=650),self.command(percent=60),
                        self.command(duration=650,percent=70),self.command(duration=650,percent=100),
                        self.command(duration=651,percent=60),self.command(duration=700,percent=60),
                        self.command('open',350,60),self.command('open',650,70),self.command('open',651,60),
                        self.command(duration=200,percent=60),
                        dict(joint='elbow',direction='up',percent=60,durationMs=350)]:
            with self.subTest(command=command),self.assertRaisesRegex(ValueError,'bounded_command'):
                self.guard.validate_command(command)

    def install_observed95_policy(self,policy=OBSERVED95_LOCAL_COMMAND_POLICY):
        image = self.shaft_image.copy()
        image[70:116,80:107] = 20
        image[70:116,220:247] = 20
        raw = cv2.imencode('.jpg',image)[1].tobytes()
        reg = register(raw,[166,95],self.config['actualScene'],
                       default_parameters(preset='current_m1_observed_shaft95'))
        self.config.update(commandPolicy=policy,
            shaftRegistration=self.write_json('M1-observed95-registration.json',reg),
            shaftReference=self.write('M1-observed95-reference.jpg',raw))
        self.binding=self.write_json('M1-observed95-config.json',self.config)
        self.guard=M1ShaftSceneGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)
        self.camera.body=raw
        self.source=GripperObservationSource('http://camera.invalid','trial','b'*32,self.guard.scene_context,
            ArtifactStore(self.repo/'.local/M1-observed95-gripper'),clock=self.clock,opener=self.camera)
        self.observer=M1ShaftSourceObserver(self.source,self.guard,clock=self.clock)
        return reg

    def test_lower_policy_adds_only_exact_60_200_250_350_CLOSE_and_keeps_OPEN(self):
        self.install_observed95_policy(OBSERVED95_LOWER_COMMAND_POLICY)
        record,result=self.capture();self.review(record,result)
        original=self.port._check_status
        for command in [self.command('close',200,60),self.command('close',250,60),self.command('close',350,60),self.command('close',450,60),
                        self.command('open',250,40),self.command('open',350,40),self.command('open',450,60),
                        self.command('open',550,60),self.command('open',650,60)]:
            self.guard.validate_command(command);self.guard.disarm()
            self.assertIs(self.port._check_status,original)
        for command in [self.command('close',650,60),self.command('close',200,40),
                        self.command('close',250,40),self.command('close',249,60),
                        self.command('close',251,60),self.command('close',300,60),self.command('close',100,60),
                        self.command('close',349,60),self.command('close',351,60),
                        self.command('open',200,60),self.command('open',350,60)]:
            with self.subTest(command=command),self.assertRaisesRegex(ValueError,'bounded_command'):
                self.guard.validate_command(command)

    def test_grasp650_policy_only_admits_exact_close_and_existing_manual_OPEN(self):
        self.install_observed95_policy(OBSERVED95_GRASP650_COMMAND_POLICY)
        self.assertEqual(self.camera.calls,[]);self.assertEqual(self.calls,[])
        record,result=self.capture();self.review(record,result)
        original=self.port._check_status
        for command in [self.command('close',650,60),self.command('open',250,40),
                        self.command('open',350,40),self.command('open',450,60),
                        self.command('open',550,60),self.command('open',650,60)]:
            self.guard.validate_command(command);self.guard.disarm()
            self.assertIs(self.port._check_status,original)
        for command in [self.command('close',200,60),self.command('close',250,60),
                        self.command('close',350,60),self.command('close',450,60),
                        self.command('close',650,40),self.command('close',650,100),
                        self.command('close',649,60),self.command('close',651,60),
                        self.command('close',650,True),self.command('open',350,60)]:
            with self.subTest(command=command),self.assertRaisesRegex(ValueError,'bounded_command'):
                self.guard.validate_command(command)
            self.assertIs(self.port._check_status,original)
        self.assertFalse(result['contactVerified']);self.assertFalse(result['graspVerified'])

    def test_grasp650_policy_keeps_active_M4_guard_and_disarm(self):
        self.install_observed95_policy(OBSERVED95_GRASP650_COMMAND_POLICY)
        record,result=self.capture();self.review(record,result)
        original=self.port._check_status;before=self.calls.count('http')
        command=self.command('close',650,60);self.guard.validate_command(command)
        self.controller.angle=196.8
        try:
            with self.assertRaisesRegex(ValueError,'active_fixed_M4'):
                self.port._check_status(self.controller.status(),'ARMED',command)
        finally:self.guard.disarm()
        self.assertIs(self.port._check_status,original)
        self.assertEqual(self.calls.count('http'),before)

    def test_observed95_local450_policy_only_allows_exact_new_candidates(self):
        self.install_observed95_policy()
        self.assertEqual(self.camera.calls,[])
        self.assertEqual(self.calls,[])
        record,result=self.capture();self.review(record,result)
        original=self.port._check_status
        for command in [self.command('close',450,60),self.command('open',450,60),self.command('open',550,60),
                        self.command('open',650,60),self.command('open',350,40),self.command('open',250,40)]:
            self.guard.validate_command(command);self.guard.disarm()
            self.assertIs(self.port._check_status,original)
        for command in [self.command(),self.command(duration=200),self.command('close',200,60),self.command('close',250,60),
                        self.command('close',350,60),self.command('close',650,60),
                        self.command('open',350,60),self.command('open',450,40),
                        self.command('open',550,40),self.command('open',650,40),
                        self.command('open',450,70),self.command('open',450,100),
                        self.command('open',449,60),self.command('open',451,60),
                        self.command('open',349,40),self.command('open',351,40),
                        self.command('open',249,40),self.command('open',251,40),self.command('open',250,60),
                        self.command('close',449,60),self.command('close',451,60),
                        self.command('open',549,60),self.command('open',551,60),self.command('open',651,60),
                        self.command('close',450,70),self.command('close',450,100),
                        self.command('close',450,True),dict(joint='elbow',direction='up',percent=60,durationMs=450)]:
            with self.subTest(command=command),self.assertRaisesRegex(ValueError,'bounded_command'):
                self.guard.validate_command(command)
            self.assertIs(self.port._check_status,original)

    def test_observed95_local_policy_keeps_active_fixed_M4_monitor_without_HTTP(self):
        self.install_observed95_policy()
        record,result=self.capture();self.review(record,result)
        original=self.port._check_status;before=self.calls.count('http')
        command=self.command('open',550,60)
        self.guard.validate_command(command);self.controller.angle=196.8
        try:
            with self.assertRaisesRegex(ValueError,'active_fixed_M4'):
                self.port._check_status(self.controller.status(),'ARMED',command)
        finally:self.guard.disarm()
        self.assertIs(self.port._check_status,original)
        self.assertEqual(self.calls.count('http'),before)

    def test_new_policy_requires_exact_observed95_registration_and_known_literal(self):
        for policy in [OBSERVED95_LOCAL_COMMAND_POLICY,OBSERVED95_GRASP650_COMMAND_POLICY,'unknown',None]:
            config=copy.deepcopy(self.config);config['commandPolicy']=policy
            binding=self.write_json('M1-bad-policy-config.json',config)
            with self.subTest(policy=policy),self.assertRaisesRegex(ValueError,'command_policy'):
                M1ShaftSceneGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)
        reg=self.install_observed95_policy()
        parameters=default_parameters(preset='current_m1_observed_shaft95');parameters['minimumNcc']=.9
        reg=register(self.camera.body,[166,95],self.config['actualScene'],parameters)
        self.config['shaftRegistration']=self.write_json('M1-stricter-registration.json',reg)
        binding=self.write_json('M1-stricter-policy-config.json',self.config)
        with self.assertRaisesRegex(ValueError,'observed95_registration_required'):
            M1ShaftSceneGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)
        self.assertEqual(self.calls,[])
        self.assertEqual(self.camera.calls,[])

    def test_legacy_policy_does_not_admit_new_close450_or_open550(self):
        record,result=self.capture();self.review(record,result)
        for command in [self.command('close',450,60),self.command('open',550,60),self.command('open',250,40)]:
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


class M1FixedObjectFragmentTests(unittest.TestCase):
    write=M1ShaftSceneTests.write
    write_json=M1ShaftSceneTests.write_json

    def setUp(self):
        M1ShaftSceneTests.setUp(self)
        fixed=self.guard.fixed_object.copy();fixed['external']=dict(fixed['external'],objectArea=5151)
        self.config['fixedObject']=self.write_json('M1-fragment-fixed.json',fixed)
        self.config['objectObservationPolicy']=OBJECT_FRAGMENT_POLICY
        self.binding=self.write_json('M1-fragment-config.json',self.config)
        self.guard=M1ShaftSceneGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)

    @staticmethod
    def stats(*fragments):
        # 각 입력은 원래 외부 영상의 bbox와 실제 연결 색 영역 면적이다.
        return np.asarray([[0,0,400,400,0]]+[
            [box[0]-450,box[1]-550,box[2]-box[0],box[3]-box[1],area] for box,area in fragments],dtype=np.int32)

    def normal_split(self):
        return self.stats(([550,650,601,690],2040),([550,700,601,751],2601))

    def test_explicit_two_fragment_contract_accepts_occlusion_without_contact_claim(self):
        image=self.image.copy();image[690:700,550:601]=100
        result=self.guard._measure(image)
        self.assertEqual(result['objectFragmentCount'],2)
        self.assertEqual(result['objectBox'],[550,650,601,751])
        self.assertAlmostEqual(result['visibleAreaRatio'],4641/5151)
        self.assertTrue(result['occlusionContractApplied'])
        self.assertEqual(result['objectObservationPolicy'],OBJECT_FRAGMENT_POLICY)
        self.assertIn('접촉',result['fragmentInterpretation'])

    def test_single_component_keeps_original_measurement(self):
        result=self.guard._measure(self.image)
        self.assertEqual(result['objectFragmentCount'],1)
        self.assertEqual(result['objectBox'],[550,650,601,751])
        self.assertEqual(result['objectArea'],5151)
        self.assertEqual(result['visibleAreaRatio'],1.)
        self.assertFalse(result['occlusionContractApplied'])

    def test_original_base_and_M1_without_opt_in_still_reject_two_components(self):
        with self.assertRaisesRegex(ValueError,'object_ambiguous'):
            ApproachExternalGuard._measure_object(self.guard,self.normal_split())
        config=dict(self.config);config.pop('objectObservationPolicy')
        binding=self.write_json('M1-no-fragment-policy.json',config)
        old=M1ShaftSceneGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)
        with self.assertRaisesRegex(ValueError,'object_ambiguous'):old._measure_object(self.normal_split())

    def test_outside_large_blob_or_three_components_are_not_selectively_ignored(self):
        outside=self.stats(([550,650,601,690],2040),([650,700,680,730],900))
        with self.assertRaisesRegex(ValueError,'outside_reference'):self.guard._measure_object(outside)
        three=self.stats(([550,650,601,680],1530),([550,690,601,720],1530),([550,730,601,751],1071))
        with self.assertRaisesRegex(ValueError,'fragment_count'):self.guard._measure_object(three)

    def test_missing_extent_and_low_or_high_total_area_are_rejected(self):
        missing=self.stats(([550,650,601,690],2040),([550,700,601,740],2040))
        with self.assertRaisesRegex(ValueError,'fragment_extent_changed'):self.guard._measure_object(missing)
        small=self.stats(([550,650,601,690],1000),([550,700,601,751],1000))
        with self.assertRaisesRegex(ValueError,'fragment_area'):self.guard._measure_object(small)
        large=self.stats(([547,647,604,699],2600),([547,700,604,754],2900))
        with self.assertRaisesRegex(ValueError,'fragment_area'):self.guard._measure_object(large)

    def test_new_policy_needs_pinned_reference_area_and_exact_policy_name(self):
        config=dict(self.config,objectObservationPolicy='auto_pick_two_blobs')
        binding=self.write_json('M1-wrong-fragment-policy.json',config)
        with self.assertRaisesRegex(ValueError,'explicit_fixed_object_occlusion_policy'):
            M1ShaftSceneGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)
        fixed=copy.deepcopy(self.guard.fixed_object);fixed['external'].pop('objectArea')
        config=dict(self.config,fixedObject=self.write_json('M1-no-reference-area.json',fixed))
        binding=self.write_json('M1-no-reference-area-config.json',config)
        with self.assertRaisesRegex(ValueError,'reference_object_area'):
            M1ShaftSceneGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)


class M1ContactObjectFragmentTests(unittest.TestCase):
    write=M1ShaftSceneTests.write
    write_json=M1ShaftSceneTests.write_json
    capture=M1ShaftSceneTests.capture
    review=M1ShaftSceneTests.review
    stats=staticmethod(M1FixedObjectFragmentTests.stats)

    def setUp(self):
        M1ShaftSceneTests.setUp(self)
        M1ShaftSceneTests.install_observed95_policy(self,OBSERVED95_GRASP650_COMMAND_POLICY)
        fixed=copy.deepcopy(self.guard.fixed_object);fixed['external']['objectArea']=5151
        self.config.update(fixedObject=self.write_json('contact-fixed.json',fixed),
                           objectObservationPolicy=CONTACT_FRAGMENT_POLICY)
        self.binding=self.write_json('contact-config.json',self.config)
        self.guard=M1ShaftSceneGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)
        self.observer=M1ShaftSourceObserver(self.source,self.guard,clock=self.clock)

    def test_three_bounded_fragments_or_single_shift_with_area_are_only_hypotheses(self):
        samples=[self.stats(([558,658,609,759],5151)),
                 self.stats(([550,650,601,680],1500),([550,690,601,720],1500),([550,730,601,751],1000))]
        for count,stats in zip((1,3),samples):
            result=self.guard._measure_object(stats)
            self.assertEqual(result['objectFragmentCount'],count)
            self.assertTrue(result['boundedMotionContractApplied'])
            self.assertEqual(result['objectObservationPolicy'],CONTACT_FRAGMENT_POLICY)
            self.assertIn('안전 검증 아님',result['fragmentInterpretation'])
        for ratio in (.40,1.05):
            # 정수 연결영역 면적을 쓰므로 경계 안쪽 정수로 확인한다.
            area=int(5151*ratio)+1 if ratio==.40 else int(5151*ratio)
            self.guard._measure_object(self.stats(([542,642,609,759],area)))

    def test_image_extraction_keeps_three_visible_components_and_original_union(self):
        image=self.image.copy();image[680:690,550:601]=100;image[710:720,550:601]=100
        result=self.guard._measure(image)
        self.assertEqual(result['objectFragmentCount'],3)
        self.assertEqual(result['objectBox'],[550,650,601,751])
        self.assertAlmostEqual(result['visibleAreaRatio'],4131/5151)

    def test_uncovering_reference_object_uses_projected_box_without_changing_visible_reference(self):
        fixed=copy.deepcopy(self.guard.fixed_object);fixed['external']['objectArea']=3800
        config=dict(self.config,fixedObject=self.write_json('occluded-contact-fixed.json',fixed))
        binding=self.write_json('occluded-contact-config.json',config)
        guard=M1ShaftSceneGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)
        result=guard._measure_object(self.stats(([550,650,601,751],5151)))
        self.assertEqual(result['visibleAreaRatio'],1.)
        self.assertEqual(result['visibleAreaBasis'],'reference_projected_box_area')
        self.assertEqual(result['referenceProjectedBoxAreaPx'],5151)
        self.assertEqual(result['referenceVisibleAreaPx'],3800)
        self.assertEqual(guard.fixed_object['external']['objectArea'],3800)
        # 기존 fixed 가림 정책은 여전히 reference의 실제 가시면적을 기준으로 쓴다.
        old_config=dict(config,objectObservationPolicy=OBJECT_FRAGMENT_POLICY)
        old_binding=self.write_json('occluded-old-fixed-config.json',old_config)
        old=M1ShaftSceneGuard(self.repo,old_binding,self.recorder,self.port,self.store,clock=self.clock)
        split=self.stats(([550,650,601,700],2500),([550,701,601,751],2550))
        self.assertGreater(guard._measure_object(split)['visibleAreaRatio'],.9)
        with self.assertRaisesRegex(ValueError,'fragment_area'):
            old._measure_object(split)

    def test_contact_reference_box_requires_positive_width_and_height(self):
        fixed=copy.deepcopy(self.guard.fixed_object);fixed['external']['objectBox']=[550,650,550,751]
        config=dict(self.config,fixedObject=self.write_json('zero-contact-box.json',fixed))
        binding=self.write_json('zero-contact-box-config.json',config)
        with self.assertRaisesRegex(ValueError,'contact_reference_box_area'):
            M1ShaftSceneGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_count_clip_component_union_extent_and_area_fail_closed(self):
        samples=[('fragment_count',self.stats()),
            ('fragment_count',self.stats(*[([550,650+i*24,601,670+i*24],600) for i in range(4)])),
            ('fragment_clipped',self.stats(([450,650,501,751],4000))),
            ('outside_reference',self.stats(([559,650,610,751],4000))),
            ('extent_changed',self.stats(([550,650,601,742],4000))),
            ('fragment_area',self.stats(([550,650,601,751],2000))),
            ('fragment_area',self.stats(([542,642,609,759],6000)))]
        for reason,stats in samples:
            with self.subTest(reason=reason),self.assertRaisesRegex(ValueError,reason):
                self.guard._measure_object(stats)

    def test_contact_policy_requires_grasp650_and_pinned_reference_area(self):
        for command_policy in (None,OBSERVED95_LOCAL_COMMAND_POLICY,OBSERVED95_LOWER_COMMAND_POLICY):
            config=copy.deepcopy(self.config)
            if command_policy is None:config.pop('commandPolicy')
            else:config['commandPolicy']=command_policy
            binding=self.write_json('bad-contact-config.json',config)
            with self.subTest(policy=command_policy),self.assertRaisesRegex(ValueError,'explicit_fixed_object_occlusion_policy'):
                M1ShaftSceneGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)
        fixed=copy.deepcopy(self.guard.fixed_object);fixed['external'].pop('objectArea')
        config=dict(self.config,fixedObject=self.write_json('no-contact-area.json',fixed))
        binding=self.write_json('no-contact-area-config.json',config)
        with self.assertRaisesRegex(ValueError,'reference_object_area'):
            M1ShaftSceneGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_review_extent_uses_eight_but_forearm_and_M4_stay_fixed(self):
        record,result=self.capture()
        external=dict(forearmCenter=list(self.guard.goal['forearmCenter']),objectBox=[558,658,609,759])
        with patch.object(self.guard,'_measure',return_value=external):
            self.review(record,result)
        self.assertFalse(result['contactVerified']);self.assertFalse(result['graspVerified'])
        external['objectBox']=[559,650,610,751]
        with patch.object(self.guard,'_measure',return_value=external),self.assertRaisesRegex(ValueError,'fixed_object_extent'):
            self.review(record,result)
        self.assertEqual(self.guard._forearm_limit(),3)

    def test_contact_extent_permission_does_not_relax_actual_forearm_or_M4_checks(self):
        record,result=self.capture()
        external=dict(forearmCenter=[self.guard.goal['forearmCenter'][0]+3.01,
                                     self.guard.goal['forearmCenter'][1]],objectBox=[558,658,609,759])
        with patch.object(self.guard,'_measure',return_value=external),self.assertRaisesRegex(ValueError,'forearm_fixed_pose'):
            self.review(record,result)
        # 실패한 guard를 되살리지 않고 별도 fixture의 원본 검사로 M4 경계도 확인한다.

    def test_contact_policy_keeps_M4_half_degree_limit(self):
        record,result=self.capture();self.controller.angle=196.68
        with self.assertRaisesRegex(ValueError,'M1_fixed_M4_changed'):
            self.review(record,result)


if __name__=='__main__':unittest.main()
