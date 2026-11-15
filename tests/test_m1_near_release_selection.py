"""탁상 근거에 결합한 별도 열기 목표. 기존 닫기 정책·모델은 바꾸지 않는다."""
import copy
import unittest
from learning import m1_near_action_model as family
from tests import test_m1_near_action_model as model_fixture


def synthetic_table_assessment():
    return dict(schemaVersion=family.RELEASE_TABLE_SUPPORT_SCHEMA,tableSupportVerified=True,
        controllerBootId='a'*32,cameraBootId='b'*32,outputSequence=15,m4Deg=192.92,
        observedAt=100.,assessedAt=101.,sources={key:dict(path='.local/synthetic/'+key+'.json',sha256='c'*64)
            for key in ('external','gripper','observation')})


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.fixture=model_fixture.NearModelTests();self.model=self.fixture.fit()
        self.goal=family.make_release_goal(self.model,synthetic_table_assessment())
    def test_original_opening_goal_selects_OPEN_with_actual_new_error(self):
        current=self.fixture.report(100.)
        decision=family.select_release(self.model,current,self.goal,commands_used=0)
        self.assertEqual(decision['targetOpeningGapPx'],140.)
        self.assertEqual(self.model['targetGapPx'],51.)
        self.assertEqual(decision['selected']['command']['direction'],'open')
        predicted=decision['selected']['predictedState'][2]
        self.assertAlmostEqual(decision['selected']['gapErrorPx'],abs(predicted-140.))
        self.assertFalse(decision['releaseVerified'])
    def test_one_OPEN_budget_cannot_dispatch_second_open(self):
        decision=family.select_release(self.model,self.fixture.report(100.),self.goal,commands_used=1)
        self.assertEqual(decision['selected']['kind'],'stop')
        self.assertEqual(decision['selectionReason'],'release_one_command_budget_exhausted')
    def test_missing_actual_table_support_or_source_binding_rejected(self):
        for mutation in ('false','missing_source','path_escape'):
            proof=synthetic_table_assessment()
            if mutation=='false':proof['tableSupportVerified']=False
            elif mutation=='missing_source':proof['sources'].pop('external')
            else:proof['sources']['external']['path']='.local/../external.json'
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):family.make_release_goal(self.model,proof)
    def test_mutated_goal_or_assessment_rejected(self):
        goal=copy.deepcopy(self.goal);goal['targetOpeningGapPx']=self.model['targetGapPx']
        with self.assertRaisesRegex(ValueError,'fixed_goal'):family.select_release(self.model,self.fixture.report(100.),goal,commands_used=0)
    def test_no_manual_OPEN_if_local_support_rejected(self):
        current=self.fixture.report(100.);current['object']['center'][0]+=31.;current['object']['bbox'][0]+=31.;current['state'][0]+=31.
        result=family.select_release(self.model,current,self.goal,commands_used=0)
        self.assertEqual(result['selected']['kind'],'stop')
        self.assertEqual(result['rejected'][0]['reason'],'near_model_current_outside_local_support')


if __name__=='__main__':unittest.main()
