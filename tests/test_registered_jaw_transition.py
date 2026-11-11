import copy
import json
import unittest
from unittest.mock import patch

from learning.registered_jaw_transition import build_transition
from tests import test_batch_gripper_points as fixture
from learning.batch_learning import fit


class JawTransitionTests(unittest.TestCase):
    def setUp(self):
        f=fixture.BatchGripperPointsTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        self.row=copy.deepcopy(f.row)
        self.row['inputs']['command']=dict(joint='gripper',direction='open',percent=20,durationMs=200)
        self.audit=dict(execution=dict(status='verified',episode=['boot','one']),recording=dict(status='verified'),
            visual=dict(status='verified',row=self.row))
        self.context=dict(viewId='empty',poseRegime='fixed',object='none',load='empty')

    def build(self):
        with patch('learning.registered_jaw_transition.ingest_session',return_value=self.audit):
            return build_transition(self.f.source,'a'*64,self.f.registration,self.f.raw,self.context,'development')

    def test_original_pair_to_learning_preserves_identity_and_single_case_deficit(self):
        result=self.build();row=result['learningRow']
        self.assertEqual(len(row['before']),5)
        self.assertEqual(row['pairedFrameIds'],['a'*32+':100','a'*32+':200'])
        self.assertEqual(row['frameIds'],['external-before','external-after'])
        model=fit([row]);self.assertFalse(model['models']);self.assertTrue(model['deficits'])
        self.assertFalse(result['controlEligible'])
        changed=copy.deepcopy(row);changed['sessionId']='two'
        with self.assertRaisesRegex(ValueError,'duplicate_training_identity'):fit([row,changed])

    def test_original_image_tampering_and_target_time_leakage_fail(self):
        path=self.f.source/'gripper-pair/esp32-before.json';meta=json.loads(path.read_bytes())
        meta['persistedAt']=21.;path.write_text(json.dumps(meta))
        with self.assertRaisesRegex(ValueError,'input_outside_window'):self.build()
        (self.f.source/'gripper-pair/esp32-before.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'image_hash'):self.build()

    def test_unverified_execution_and_other_joint_never_form_rows(self):
        self.audit['execution']['status']='unknown'
        with self.assertRaisesRegex(ValueError,'verified_jaw_source'):self.build()
        self.audit['execution']['status']='verified';self.row['inputs']['command']['joint']='wrist'
        with self.assertRaisesRegex(ValueError,'M1_required'):self.build()
