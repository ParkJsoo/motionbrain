"""외부 영상 복구 목적과 기존 OPEN 선택의 무장치 회귀."""
import copy
import unittest
from learning import gripper_recovery as recovery
from learning import m1_near_action_model as response
from learning import m1_local_open_response as local
from unittest.mock import patch
from tests.test_limited_scene_cycle_assessment import near, external


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.base=external();self.current=external();self.current['imageSha256']='c'*64
        reference=near(100,version=3);partial=near(60,version=3)
        rows=[]
        for i,(direction,before,after) in enumerate([
            ('close',reference,partial),('close',reference,partial),
            ('open',partial,reference),('open',partial,reference)]):
            rows.append(dict(sessionId=str(i),sourceSha256=str(i)*64,
                command=next(copy.deepcopy(c) for c in response.COMMANDS if c['direction']==direction),
                before=before,after=after,observationContractSha256=reference['observationContractSha256'],
                sourceEvidence=dict(beforeExternalImageSha256='a'*64,afterExternalImageSha256='b'*64,
                    beforeGripperImageSha256=before['imageSha256'],afterGripperImageSha256=after['imageSha256'])))
        self.model=response.fit(rows,reference=reference,audit_sources=lambda r:r)
        self.partial=partial

    def decision(self, near_input=None, assessment=None, used=0):
        return recovery.select_reopening(self.model, self.partial if near_input is None else near_input,
                recovery.assess_table(self.base,self.current) if assessment is None else assessment,commands_used=used)

    def test_supported_input_selects_learned_OPEN_without_changing_close_target(self):
        before=copy.deepcopy(self.model);d=self.decision()
        self.assertEqual(d['selected']['command']['direction'],'open')
        self.assertEqual(d['purposeOwner'],'external_table_image_rule')
        self.assertTrue(d['modelSelectedAction']);self.assertFalse(d['modelSelectedStop'])
        self.assertEqual(before,self.model);self.assertFalse(d['missingNearStateImputed'])
        self.assertNotEqual(d['targetOpeningGapPx'],self.model['targetGapPx'])
        self.assertFalse(d['recoveryCompleted'])

    def test_missing_near_does_not_block_external_assessment_or_invent_model_STOP(self):
        gate=recovery.assess_table(self.base,self.current)
        self.assertTrue(gate['accepted']);self.assertFalse(gate['evidence']['actualEmptyGripperVerified'])
        d=self.decision(near_input={'rejected':'near_tip_missing_or_ambiguous'},assessment=gate)
        self.assertEqual(d['selected']['kind'],'stop');self.assertFalse(d['modelSelectedStop'])
        self.assertEqual(d['selectionOwner'],'admission_guard')
        self.assertIsNone(d['selected']['openingErrorPx'])

    def test_wrong_table_position_registration_or_same_frame_rejected(self):
        for current in (self.base,external(dx=20),external(rise=20)):
            self.assertFalse(recovery.assess_table(self.base,current)['accepted'])
        bad=copy.deepcopy(self.current);bad['registrationSha256']='d'*64
        self.assertFalse(recovery.assess_table(self.base,bad)['accepted'])

    def test_forged_gate_and_unsupported_near_rejected(self):
        gate=recovery.assess_table(self.base,self.current);gate['evidence']['apparentTablePositionVerified']=False
        self.assertEqual(self.decision(assessment=gate)['selectionOwner'],'admission_guard')
        changed=copy.deepcopy(self.partial);changed['state'][2]=0
        self.assertEqual(self.decision(near_input=changed)['selectionOwner'],'admission_guard')

    def test_budget_stop_is_not_model_stop_and_no_second_OPEN(self):
        d=self.decision(used=1)
        self.assertEqual(d['selectionOwner'],'budget_guard');self.assertFalse(d['modelSelectedStop'])
        self.assertEqual(len(d['candidates']),1)
        for used in (-1,2,True):
            with self.assertRaises(ValueError):self.decision(used=used)


    def test_local_OPEN_is_explicit_opt_in_distinct_owner_and_source_binding(self):
        artifact=local.fit(self.model,audit_sources=lambda rows:rows)
        gate=recovery.assess_table(self.base,self.current)
        default=self.decision()
        self.assertEqual(default,recovery.select_reopening(self.model,self.partial,gate,commands_used=0,local_open_model=None))
        self.assertNotIn('localOpenModelSha256',default)
        d=recovery.select_reopening(self.model,self.partial,gate,commands_used=0,local_open_model=artifact)
        self.assertTrue(d['modelSelectedAction']);self.assertFalse(d['controlEligible'])
        self.assertEqual(d['selectionOwner'],'local_observed_delta_OPEN_response_comparison')
        self.assertEqual(d['localOpenModelSha256'],response.digest(artifact))
        self.assertNotEqual(d['localOpenModelSha256'],d['modelSha256'])
        self.assertEqual(d['selected']['response']['modelSha256'],d['localOpenModelSha256'])

    def test_local_rejection_never_falls_back_to_original_ridge_or_second_OPEN(self):
        artifact=local.fit(self.model,audit_sources=lambda rows:rows);gate=recovery.assess_table(self.base,self.current)
        changed=copy.deepcopy(artifact);changed['sourceBinding']['sourceFrozenModelSha256']='f'*64
        with patch.object(response,'predict',side_effect=AssertionError('ridge fallback forbidden')):
            d=recovery.select_reopening(self.model,self.partial,gate,commands_used=0,local_open_model=changed)
            self.assertEqual(d['selected']['kind'],'stop');self.assertFalse(d['modelSelectedStop'])
            self.assertEqual(d['rejected'][0]['reason'],'local_OPEN_source_binding_changed')
            exhausted=recovery.select_reopening(self.model,self.partial,gate,commands_used=1,local_open_model=artifact)
            self.assertEqual(exhausted['selectionOwner'],'budget_guard');self.assertFalse(exhausted['modelSelectedAction'])
            self.assertEqual(len(exhausted['candidates']),1)
