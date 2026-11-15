import copy
import unittest

from learning import experimental_lift_response as model


class LiftModelTests(unittest.TestCase):
    def rows(self):
        rows=[]
        for index, state in enumerate([[1000., 700., 207.77, -25.], [1010., 660., 207.77, -20.], [1020., 650., 192.92, -22.]]):
            rows.append(dict(sessionId=f's{index}', sourceSha256=str(index+1)*64,
                observationContractSha256='a'*64, command=copy.deepcopy(model.COMMAND), before=state,
                after=[state[0]-5., state[1]-80., state[2], state[3]+5.],
                sourceEvidence={**{key: str(index+4)*64 for key in
                    ('beforeImageSha256','afterImageSha256','beforeSensorSha256','afterSensorSha256',
                     'beforeGripperImageSha256','afterGripperImageSha256')},'trainingHeldLoadVerified':False}))
        return rows

    def fit(self): return model.fit(self.rows(),audit_sources=lambda rows:rows)

    def current(self):
        return dict(state=[1020.,650.,192.92,-22.], observationContractSha256='a'*64, observationSha256='b'*64)

    def test_UP_is_model_selected_against_STOP_without_claiming_actual_lift(self):
        fitted=self.fit();current=self.current();goal=model.make_goal(fitted,current)
        result=model.select(fitted,current,goal,commands_used=0)
        self.assertEqual(result['selected']['kind'],'action')
        self.assertEqual(result['selected']['command'],model.COMMAND)
        self.assertLess(result['selected']['errorPx'],80.)
        self.assertIn('commandMeanErrorPx',result['selected'])
        self.assertTrue(result['transferToHeldLoadUnverified'])
        for key in ['currentHeldLoadTrainingSupportVerified','actualObjectTableSeparationVerified','retainedLiftVerified','safetyBound']:
            self.assertFalse(result[key])

    def test_goal_stays_fixed_and_one_command_budget_cannot_repeat_UP(self):
        fitted=self.fit();current=self.current();goal=model.make_goal(fitted,current)
        later=copy.deepcopy(current);later['state'][1]-=60.;later['observationSha256']='c'*64
        exhausted=model.select(fitted,later,goal,commands_used=1)
        self.assertEqual(exhausted['selected']['kind'],'stop')
        self.assertEqual(exhausted['targetVirtualPointPx'],[1020.,570.])
        self.assertEqual(exhausted['selectionReason'],'one_command_budget_exhausted')
        changed=copy.deepcopy(goal);changed['targetVirtualPointPx'][1]-=80.
        with self.assertRaisesRegex(ValueError,'initial_fixed_goal_changed'):model.select(fitted,later,changed,commands_used=0)

    def test_pose_transfer_limits_and_observation_contract_are_explicit(self):
        fitted=self.fit();current=self.current();goal=model.make_goal(fitted,current)
        for state in [[1020.,650.,176.,-22.],[1020.,650.,192.92,20.],[1500.,650.,192.92,-22.]]:
            changed=dict(current,state=state)
            result=model.select(fitted,changed,goal,commands_used=0)
            self.assertEqual(result['selected']['kind'],'stop')
            self.assertEqual(result['rejected'][0]['reason'],'lift_current_outside_local_transfer_support')
        bad=dict(current,observationContractSha256='f'*64)
        with self.assertRaisesRegex(ValueError,'current_observation_binding'):model.select(fitted,bad,goal,commands_used=0)

    def test_regression_on_goal_overshoot_selects_STOP_instead_of_forcing_UP(self):
        rows=self.rows()
        for row in rows:row['after'][1]=row['before'][1]-200.
        fitted=model.fit(rows,audit_sources=lambda rows:rows);current=self.current()
        result=model.select(fitted,current,model.make_goal(fitted,current),commands_used=0)
        self.assertEqual(result['selected']['kind'],'stop')

    def test_minimum_two_distinct_raw_sources_and_unverified_load_marker_required(self):
        for mutation in ['one','duplicate','held_claim','missing_pair']:
            rows=self.rows()
            if mutation=='one':rows=rows[:1]
            elif mutation=='duplicate':rows[1]['sourceSha256']=rows[0]['sourceSha256']
            elif mutation=='held_claim':rows[0]['sourceEvidence']['trainingHeldLoadVerified']=True
            else:del rows[0]['sourceEvidence']['beforeGripperImageSha256']
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):model.fit(rows,audit_sources=lambda rows:rows)
        with self.assertRaisesRegex(ValueError,'source_audit_changed'):model.fit(self.rows(),audit_sources=lambda rows:[])


if __name__=='__main__':unittest.main()
