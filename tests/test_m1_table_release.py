"""새 탁상 자세 계약의 모의 관측·단발 OPEN. 실제 장치 접근은 없다."""
import copy
import unittest
from learning.m1_table_release import M1TableReleaseGuard, SCHEMA, REFERENCE_MEANING, CONTRACT, COMMAND
from learning.m1_shaft_scene import M1ShaftSceneGuard, M1ShaftSourceObserver
from learning.supervised_transport import SequencePort
from tests.test_m1_shaft_scene import M1ShaftSceneTests, M1ContactObjectFragmentTests


class TableReleaseTests(unittest.TestCase):
    write = M1ShaftSceneTests.write
    write_json = M1ShaftSceneTests.write_json
    capture = M1ShaftSceneTests.capture
    review = M1ShaftSceneTests.review

    def setUp(self):
        M1ContactObjectFragmentTests.setUp(self)
        self.tracking.update(expectedM4=195.55, maximumM4Difference=.25)
        self.config.update(schemaVersion=SCHEMA, referenceMeaning=REFERENCE_MEANING,
            releaseContract=copy.deepcopy(CONTRACT),tracking=self.write_json('table-tracking.json',self.tracking))
        self.binding=self.write_json('table-config.json',self.config)
        self.controller.angle=195.55
        self.guard=M1TableReleaseGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)
        self.observer=M1ShaftSourceObserver(self.source,self.guard,clock=self.clock)

    def test_exact_open_once_and_restore_checker_without_extra_HTTP(self):
        record,result=self.capture();self.review(record,result)
        original=self.port._check_status;calls=self.calls.count('http')
        self.guard.validate_command(COMMAND)
        self.port._check_status(self.controller.status(),'ARMED',COMMAND)
        self.guard.disarm();self.assertIs(self.port._check_status,original)
        self.assertEqual(self.calls.count('http'),calls)
        with self.assertRaisesRegex(ValueError,'one_shot_consumed'):
            self.guard.validate_command(COMMAND)

    def test_active_quarter_degree_rejects_and_restores(self):
        record,result=self.capture();self.review(record,result)
        original=self.port._check_status;self.guard.validate_command(COMMAND)
        self.controller.angle=195.81
        with self.assertRaisesRegex(ValueError,'active_fixed_M4'):
            self.port._check_status(self.controller.status(),'ARMED',COMMAND)
        self.guard.disarm();self.assertIs(self.port._check_status,original)

    def test_idle_quarter_degree_rejects(self):
        record,result=self.capture();self.controller.angle=195.81
        with self.assertRaisesRegex(ValueError,'fixed_M4'):
            self.review(record,result)

    def test_wrong_command_consumes_guard_without_installing_wrapper(self):
        record,result=self.capture();self.review(record,result);original=self.port._check_status
        for command in [dict(COMMAND,direction='close'),dict(COMMAND,durationMs=649),
                        dict(COMMAND,percent=61),dict(COMMAND,joint='elbow')]:
            with self.subTest(command=command),self.assertRaises(ValueError):
                self.guard.validate_command(command)
            self.assertIs(self.port._check_status,original)
        with self.assertRaisesRegex(ValueError,'one_shot_consumed'):
            self.guard.validate_command(COMMAND)

    def test_old_guard_cannot_admit_new_schema(self):
        with self.assertRaisesRegex(ValueError,'configuration'):
            M1ShaftSceneGuard(self.repo,self.binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_contract_and_angle_must_match_exact_new_declaration(self):
        for key,value in [('maxCommands',2),('maxCommands',True),('tableSupportRequired',1),
                          ('tableSupportRequired',False),('automaticRetry',True)]:
            config=copy.deepcopy(self.config);config['releaseContract'][key]=value
            binding=self.write_json('bad-table-config.json',config)
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'explicit_contract'):
                M1TableReleaseGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)
        self.tracking['maximumM4Difference']=.5
        self.config['tracking']=self.write_json('bad-table-tracking.json',self.tracking)
        binding=self.write_json('bad-table-config.json',self.config)
        with self.assertRaisesRegex(ValueError,'quarter_degree'):
            M1TableReleaseGuard(self.repo,binding,self.recorder,self.port,self.store,clock=self.clock)

    def test_preserves_forearm_and_object_limits_and_no_constructor_IO(self):
        self.assertEqual(self.guard._forearm_limit(),3)
        self.assertEqual(self.guard._object_box_limit(),8)
        self.assertEqual(self.camera.calls,[]);self.assertEqual(self.calls,[])

    def armed(self):
        self.controller.state='ARMED';self.port._active_pulse=copy.deepcopy(COMMAND)
        self.port.last_sequence=17;self.port.last_outputs={i:0 for i in range(1,6)}
        self.port._status=lambda:self.controller.status()
        self.port._queue=lambda count:self.assertEqual(count,1)
        self.port._outputs=lambda:None
        self.port._check_status=lambda status,expected,pulse=None,baseline_angle=None:SequencePort._check_status(
            self.port,status,expected,pulse,baseline_angle)

    def test_review_armed_is_explicit_and_keeps_default_IDLE_review(self):
        record,result=self.capture();self.armed()
        self.port.check_idle=lambda:(_ for _ in ()).throw(ValueError('IDLE_only'))
        proof=self.guard.review_armed(record,result,expected_sequence=17,deadline=60,tick=lambda:None)
        self.assertTrue(proof['accepted']);self.assertIsNone(self.guard._armed_review_sequence)
        with self.assertRaisesRegex(ValueError,'IDLE_only'):
            self.review(record,result)

    def test_review_armed_rejects_any_M1_output_and_sequence_change(self):
        record,result=self.capture();self.armed();self.controller.speeds[1]=1
        with self.assertRaisesRegex(ValueError,'unexpected_motor_output'):
            self.guard.review_armed(record,result,expected_sequence=17,deadline=60,tick=lambda:None)
        self.assertTrue(self.guard._failed);self.assertIsNone(self.guard._armed_review_sequence)

    def test_review_armed_rejects_output_cursor_advance(self):
        record,result=self.capture();self.armed()
        self.port._outputs=lambda:setattr(self.port,'last_sequence',18)
        with self.assertRaisesRegex(ValueError,'armed_output_changed'):
            self.guard.review_armed(record,result,expected_sequence=17,deadline=60,tick=lambda:None)

    def test_review_armed_rejects_boot_and_unexpected_pulse(self):
        record,result=self.capture();self.armed();self.port.boot_id='c'*32
        with self.assertRaisesRegex(ValueError,'identity_changed'):
            self.guard.review_armed(record,result,expected_sequence=17,deadline=60,tick=lambda:None)


if __name__=='__main__':unittest.main()
