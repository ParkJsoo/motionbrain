"""OFF 영수증과 파지 보유 실패를 구분하는 전체 과업 회귀.

실측에서 관측한 'UP 뒤 물체 이탈·탁상 복귀' 실패 형태를 합성 포트에
재현한다. 비공개 영상이나 실물 성능 검증을 포함하지 않는다.
"""
import unittest
from tests import test_finite_task_cycle as fixtures


class RetentionFailureRegressionTests(unittest.TestCase):
    def ports(self):
        p=fixtures.Ports(fixtures.scope())
        original=p.goal
        def goal(stage,record):
            proof=original(stage,record)
            if stage['kind']=='lift' and p.count['lift']:
                proof.update(reached=False,bilateralContact=False,airborne=False,
                             retained=False,tableSupported=True)
            return proof
        p.goal=goal
        return p

    def assert_stopped_after_lift(self,p,report):
        self.assertEqual(report['status'],'failed')
        self.assertEqual([c['direction'] for c in p.sent],['up','close','up'])
        self.assertEqual(p.count['lift'],1)
        self.assertEqual(p.count['lower'],0)
        self.assertEqual(p.count['release'],0)
        self.assertEqual(report['confirmedCommands'],3)
        self.assertFalse(report['fullyModelSelectedCycle'])
        self.assertEqual(p.stops,1)

    def test_unstable_after_drop_stops_without_retry_lower_or_open(self):
        p=self.ports()
        def changed(stage,record):
            if stage['kind']=='lift' and p.count['lift']:
                record['stable']=False
        p.mutate=changed
        self.assert_stopped_after_lift(p,p.cycle().run())

    def test_stable_object_back_on_table_is_not_retained_grasp(self):
        p=self.ports()
        def changed(stage,record):
            if stage['kind']=='lift' and p.count['lift']:
                record['entry'].update(bilateralContact=False,airborne=False,tableSupported=True)
        p.mutate=changed
        self.assert_stopped_after_lift(p,p.cycle().run())

    def test_OFF_receipt_cannot_override_false_retention_predicates(self):
        p=self.ports();original=p.goal
        def invalid_goal(stage,record):
            proof=original(stage,record)
            if stage['kind']=='lift' and p.count['lift']:proof['reached']=True
            return proof
        p.goal=invalid_goal
        self.assert_stopped_after_lift(p,p.cycle().run())
