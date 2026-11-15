import copy
import unittest

from learning import experimental_lower_release as model
from tests.test_experimental_lift_response import LiftModelTests


class LowerModelTests(unittest.TestCase):
    def fit(self): return model.fit_lowering(LiftModelTests().rows(),audit_sources=lambda rows:rows)
    def state(self,after=False):
        row=LiftModelTests().rows()[-1]
        return dict(state=row['after' if after else 'before'],observationContractSha256='a'*64,observationSha256='b'*64)

    def test_DOWN_is_explicit_inverse_hypothesis_not_fake_DOWN_training(self):
        rows=LiftModelTests().rows();saved=copy.deepcopy(rows)
        fitted=model.fit_lowering(rows,audit_sources=lambda rows:rows)
        self.assertEqual(rows,saved)
        self.assertEqual(fitted['directDOWNTrainingRows'],0)
        self.assertTrue(all(r['command']['direction']=='up' for r in fitted['originalUPTrainingRows']))
        current=self.state(after=True);goal=model.make_lowering_goal(fitted,self.state())
        decision=model.select_lowering(fitted,current,goal,commands_used=0)
        self.assertEqual(decision['selected']['kind'],'action')
        self.assertEqual(decision['selected']['command'],model.DOWN)
        self.assertLess(decision['selected']['errorPx'],decision['candidates'][-1]['errorPx'])
        self.assertFalse(decision['physicalTableSupportVerified'])
        self.assertFalse(decision['directionAsymmetryVerified'])
        self.assertIn('commandMeanErrorPx',decision['selected'])

    def test_at_saved_goal_or_after_one_command_STOP_never_forces_more_DOWN(self):
        fitted=self.fit();goal=model.make_lowering_goal(fitted,self.state())
        at=model.select_lowering(fitted,self.state(),goal,commands_used=0)
        self.assertEqual(at['selected']['kind'],'stop')
        used=model.select_lowering(fitted,self.state(after=True),goal,commands_used=1)
        self.assertEqual(used['selected']['kind'],'stop')
        self.assertEqual(used['targetVirtualPointPx'],goal['targetVirtualPointPx'])

    def test_unbound_goal_changed_source_or_outside_pose_is_rejected(self):
        fitted=self.fit();goal=model.make_lowering_goal(fitted,self.state());goal['targetVirtualPointPx'][1]+=20.
        with self.assertRaisesRegex(ValueError,'saved_goal_changed'):
            model.select_lowering(fitted,self.state(after=True),goal,commands_used=0)
        current=self.state(after=True);current['state'][2]=150.
        decision=model.select_lowering(fitted,current,model.make_lowering_goal(fitted,self.state()),commands_used=0)
        self.assertEqual(decision['selected']['kind'],'stop')
        self.assertEqual(decision['rejected'][0]['reason'],'lower_current_outside_inverse_support')
        with self.assertRaisesRegex(ValueError,'source_audit_changed'):
            model.fit_lowering(LiftModelTests().rows(),audit_sources=lambda rows:[])


if __name__=='__main__':unittest.main()
