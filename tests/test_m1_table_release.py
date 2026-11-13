"""새 탁상 자세 계약의 모의 관측·단발 OPEN. 실제 장치 접근은 없다."""
import copy
import unittest
from learning.m1_table_release import M1TableReleaseGuard, SCHEMA, REFERENCE_MEANING, CONTRACT, COMMAND
from learning.m1_shaft_scene import M1ShaftSceneGuard, M1ShaftSourceObserver
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


if __name__=='__main__':unittest.main()
