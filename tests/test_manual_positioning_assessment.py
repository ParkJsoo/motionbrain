"""수동 시작 배치의 관측 delta 사전 거절. 장치·원본 변경·실물 학습은 없다."""
import copy
import unittest
from unittest.mock import patch

from learning.manual_positioning_assessment import assess_manual_positioning
from learning.relative_action_learning import fit, STATE
from learning.supervised_transport import ELBOW_DEADLINE_POLICY
from learning.supervised_collection import plan_digest

UP=dict(joint='elbow',direction='up',percent=60,durationMs=350)
DOWN=dict(UP,direction='down')
FENCE=dict(minimum=[-3.,8.,122.],maximum=[8.,22.,126.])


class ManualPositioningAssessmentTests(unittest.TestCase):
    def setUp(self):
        self.context=dict(physicalView='synthetic-current-view',object='synthetic-pink',
            poseRegime='synthetic-fixed-M4',landmarks='synthetic-points',
            cameraIdentity={'deviceId':'synthetic'},imageShape=[240,320],stateFields=STATE,
            executionTimingPolicy=ELBOW_DEADLINE_POLICY)
        examples=[(UP,[1.,17.,124.],[0.,13.5,124.]),
                  (DOWN,[.5,13.5,124.],[3.,19.5,124.]),
                  (UP,[3.,19.5,124.],[1.,16.5,124.]),
                  (DOWN,[1.,16.5,124.],[3.,20.,124.]),
                  (UP,[2.5,20.,124.],[.5,17.5,124.])]
        self.rows=[dict(sessionId=f'synthetic-{i}',captureGroup='synthetic-development',
            context=copy.deepcopy(self.context),command=copy.deepcopy(command),before=before,after=after,
            episode=['synthetic-boot',f'command-{i}'],frameIds=[f'before-{i}',f'after-{i}'])
            for i,(command,before,after) in enumerate(examples)]
        self.model=fit(self.rows)
        # 검토된 fixture manifest의 행 내용을 변경 전에 별도로 고정한다.
        self.expected_rows_sha256=plan_digest(self.rows)
    def assess(self,state=None,command=None,**kwargs):
        return assess_manual_positioning(self.model,self.rows,state or [1.,17.5,124.],
            self.context,command or DOWN,FENCE,
            expected_training_rows_sha256=self.expected_rows_sha256,**kwargs)
    def test_observed_actual_case_is_rejected_before_DOWN_can_cross_existing_fence(self):
        result=self.assess()
        self.assertEqual(result['status'],'rejected')
        self.assertEqual(result['observedDeltaMinimum'],[2.,3.5,0.])
        self.assertEqual(result['observedDeltaMaximum'],[2.5,6.,0.])
        self.assertEqual(result['observedEndpointEnvelope'],dict(minimum=[3.,21.,124.],maximum=[3.5,23.5,124.]))
        self.assertIn('observed_endpoint_envelope_exceeds_fence',result['reasons'])
        self.assertIn('current_state_outside_command_training_support',result['reasons'])
        self.assertTrue(result['currentInsideFence'])
        self.assertFalse(result['futureOutcomeGuaranteed'])
        self.assertFalse(result['executionAuthorized'])
    def test_within_empirical_fence_only_requests_review_without_granting_execution(self):
        result=self.assess([.5,13.5,124.])
        self.assertEqual(result['status'],'review_required')
        self.assertEqual(result['reasons'],[])
        self.assertTrue(result['currentCommandInSupport'])
        self.assertTrue(result['empiricalEnvelopeNotConfidenceBound'])
        self.assertFalse(result['modelSelected'])
        self.assertFalse(result['controlEligible'])
        self.assertFalse(result['trainingPerformed'])
    def test_current_outside_fence_is_rejected_even_when_command_evidence_exists(self):
        result=self.assess([1.,23.,124.])
        self.assertIn('current_state_outside_fence',result['reasons'])
        self.assertEqual(result['status'],'rejected')
    def test_context_training_provenance_and_changed_delta_are_not_accepted(self):
        with self.assertRaisesRegex(ValueError,'positioning_context_mismatch'):
            assess_manual_positioning(self.model,self.rows,[1.,17.5,124.],dict(self.context,object='another'),DOWN,FENCE,
                expected_training_rows_sha256=self.expected_rows_sha256)
        for mutation in (lambda r:r[0].update(sessionId='other-source'),
                         lambda r:r[0]['context'].update(poseRegime='other-pose'),
                         lambda r:r[1]['after'].__setitem__(1,20.5)):
            changed=copy.deepcopy(self.rows);mutation(changed)
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):
                assess_manual_positioning(self.model,changed,[1.,17.5,124.],self.context,DOWN,FENCE,
                    expected_training_rows_sha256=self.expected_rows_sha256)
    def test_equal_mean_range_shrink_with_same_ids_cannot_replace_reviewed_rows(self):
        baseline=self.assess([1.,16.5,124.])
        self.assertIn('observed_endpoint_envelope_exceeds_fence',baseline['reasons'])
        altered=copy.deepcopy(self.rows)
        altered[1]['after'][1]=altered[1]['before'][1]+4.75
        altered[3]['after'][1]=altered[3]['before'][1]+4.75
        original_mean=sum(self.rows[i]['after'][1]-self.rows[i]['before'][1] for i in (1,3))/2
        altered_mean=sum(altered[i]['after'][1]-altered[i]['before'][1] for i in (1,3))/2
        self.assertEqual(original_mean,altered_mean)
        self.assertEqual([r['sessionId'] for r in altered],[r['sessionId'] for r in self.rows])
        with self.assertRaisesRegex(ValueError,'training_rows_content_digest_mismatch'):
            assess_manual_positioning(self.model,altered,[1.,16.5,124.],self.context,DOWN,FENCE,
                expected_training_rows_sha256=self.expected_rows_sha256)

    def test_external_reviewed_rows_digest_is_mandatory_and_must_match(self):
        with self.assertRaises(TypeError):
            assess_manual_positioning(self.model,self.rows,[1.,17.5,124.],self.context,DOWN,FENCE)
        for value in (None,'not-a-digest','0'*64):
            with self.subTest(value=value),self.assertRaises(ValueError):
                assess_manual_positioning(self.model,self.rows,[1.,17.5,124.],self.context,DOWN,FENCE,
                    expected_training_rows_sha256=value)

    def test_command_without_original_evidence_is_rejected(self):
        rows=[row for row in self.rows if row['command']==UP]
        model=fit(rows);expected_rows_sha256=plan_digest(rows)
        result=assess_manual_positioning(model,rows,[1.,17.5,124.],self.context,DOWN,FENCE,
            expected_training_rows_sha256=expected_rows_sha256)
        self.assertEqual(result['status'],'rejected')
        self.assertEqual(result['reasons'],['command_without_two_original_training_transitions'])
        self.assertEqual(result['observedSamples'],0)
    def test_goal_ready_manual_endpoint_is_not_counted_as_model_approach(self):
        result=self.assess([.5,13.5,124.],target=[1.,16.5,124.],tolerance=[1.5,1.5,1.])
        self.assertIn('observed_endpoint_may_already_satisfy_target',result['reasons'])
        self.assertTrue(any(d['targetAlreadyReached'] for d in result['followupDiagnostics']))
        self.assertFalse(result['modelSelected'])
    def test_fixed_supported_improving_followup_is_diagnostic_only(self):
        result=self.assess([.5,13.5,124.],target=[1.,10.,124.],tolerance=[.5,.5,1.])
        self.assertEqual(result['status'],'review_required',result)
        self.assertTrue(all(d['improvingSupportedCandidates'] for d in result['followupDiagnostics']))
        self.assertFalse(result['executionAuthorized'])
        unsupported=self.assess([1.,17.5,124.],target=[1.,16.5,124.],tolerance=[1.5,1.5,1.])
        self.assertIn('observed_endpoint_lacks_supported_improving_followup',unsupported['reasons'])
    def test_assessment_preserves_inputs_does_not_fit_and_opens_no_device(self):
        original=copy.deepcopy((self.model,self.rows,self.context,FENCE,DOWN))
        with patch('learning.relative_action_learning.fit',side_effect=AssertionError('refit forbidden')), \
                patch('socket.socket',side_effect=AssertionError('device forbidden')), \
                patch('subprocess.Popen',side_effect=AssertionError('process forbidden')):
            self.assess()
        self.assertEqual((self.model,self.rows,self.context,FENCE,DOWN),original)
    def test_invalid_fence_or_unpaired_goal_is_rejected(self):
        with self.assertRaises(ValueError):self.assess(target=[1.,16.5,124.])
        with self.assertRaises(ValueError):self.assess(target=[1.,16.5,124.],tolerance=[0.,1.,1.])
        with self.assertRaises(ValueError):
            assess_manual_positioning(self.model,self.rows,[1.,17.5,124.],self.context,DOWN,
                                      dict(minimum=[-3.,22.,122.],maximum=[8.,8.,126.]),
                                      expected_training_rows_sha256=self.expected_rows_sha256)


if __name__=='__main__':unittest.main()
