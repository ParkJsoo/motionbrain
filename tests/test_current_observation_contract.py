"""새 M3 기준의 명시성과 기존 현장 보호 한도 회귀."""
import unittest
from tests import test_approach_external_guard as fixtures
from learning.current_observation_contract import CurrentTaskM3Guard, SCHEMA, REFERENCE_MEANING
from learning.approach_external_guard import CurrentSceneExternalGuard

class CurrentContractTests(unittest.TestCase):
    def setUp(self):
        owner=fixtures.CurrentSceneGuardTests();owner.setUp();self.addCleanup(owner.doCleanups)
        self.o=owner
        owner.config.update(schemaVersion=SCHEMA,referenceMeaning=REFERENCE_MEANING,referenceM4Deg=195.29,referenceJawSpanPx=63.,objectObservationPolicy='fixed_reference_two_fragment_occlusion_v1')
        owner.tracking['expectedM4']=195.29
        owner.config['tracking']=owner.write_json('new-tracking.json',owner.tracking)
        owner.controller.angle=195.29;owner.result['state']=[0.,-23.5,63.]
        owner.binding=owner.write_json('new-config.json',owner.config)
        owner.guard=CurrentTaskM3Guard(owner.repo,owner.binding,owner.recorder,owner.port,owner.store,clock=owner.clock)

    def test_current_reference_accepted_without_changing_old_guard(self):
        self.assertTrue(self.o.review()['accepted'])
        with self.assertRaisesRegex(ValueError,'external_configuration'):
            CurrentSceneExternalGuard(self.o.repo,self.o.binding,self.o.recorder,self.o.port,self.o.store,clock=self.o.clock)

    def test_current_span_and_M4_still_guarded(self):
        self.o.result['state'][2]=124.
        with self.assertRaisesRegex(ValueError,'jaw_span_changed'):self.o.review()
        self.o.result['state'][2]=63.;self.o.controller.angle=196.17
        with self.assertRaisesRegex(ValueError,'fixed_M4_changed'):self.o.review()

    def test_nonfinite_reference_and_missing_fields_rejected(self):
        for value in [float('nan'),float('inf'),True,121.]:
            self.o.config['referenceM4Deg']=value
            binding=self.o.write_json('invalid.json',self.o.config)
            with self.assertRaises(ValueError):
                CurrentTaskM3Guard(self.o.repo,binding,self.o.recorder,self.o.port,self.o.store,clock=self.o.clock)

    def test_original_direction_fence_is_preserved(self):
        self.o.review_external(delta=6.1)
        with self.assertRaisesRegex(ValueError,'direction_clearance'):
            self.o.guard.validate_command(self.o.command('up'))

    def arm_port(self):
        o=self.o
        o.port._active_pulse=dict(joint='elbow',direction='up',percent=60,durationMs=350)
        o.port.last_sequence=5;o.port.last_outputs={i:0 for i in range(1,6)}
        o.port._status=lambda:dict(o.controller.status(),state='ARMED')
        def checked(status,expected):
            self.assertEqual(status['state'],expected)
            self.assertEqual(expected,'ARMED')
        o.port._check_status=checked
        o.port._queue=lambda count:self.assertEqual(count,1)
        o.port._outputs=lambda:None
        o.port.check_idle=lambda:(_ for _ in ()).throw(ValueError('must_not_check_idle_after_arm'))

    def test_armed_review_uses_armed_OFF_queue_and_preserves_idle_path(self):
        self.arm_port()
        self.assertTrue(self.o.guard.review_armed(self.o.record,self.o.result,expected_sequence=5,deadline=60.,tick=lambda:None)['accepted'])
        with self.assertRaisesRegex(ValueError,'must_not_check_idle'):
            self.o.review()

    def test_armed_output_change_and_wrong_command_rejected(self):
        self.arm_port();self.o.port.last_outputs[1]=1
        with self.assertRaisesRegex(ValueError,'armed_output_changed'):
            self.o.guard.review_armed(self.o.record,self.o.result,expected_sequence=5,deadline=60.,tick=lambda:None)
        self.o.port._active_pulse['joint']='gripper'
        with self.assertRaisesRegex(ValueError,'armed_pulse'):
            self.o.guard.review_armed(self.o.record,self.o.result,expected_sequence=5,deadline=60.,tick=lambda:None)
