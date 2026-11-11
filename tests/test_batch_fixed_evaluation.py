import copy
import unittest
from learning.batch_learning import fit, predict, model_sha256
from learning.batch_fixed_evaluation import predictions, prospective_evidence, evaluate
from tests.test_batch_learning import row


class FixedEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.model = fit([row(1), row(3)])
        self.row = row(5, group='future')
        self.freeze = dict(modelSha256=model_sha256(self.model), frozenAt=8.)
        self.frames = [dict(frameId=f'camera-{i}', pixelSha256=str(i)*64,
                            capturedAt=9., persistedAt=10.) for i in (1, 2)]

    def proof(self):
        return prospective_evidence(self.model, self.freeze, self.row, self.frames, 11.)

    def test_outside_prediction_does_not_change_control_support(self):
        saved = copy.deepcopy(self.model)
        result = predictions(self.model, self.row['before'], self.row['command'], self.row['context'])
        self.assertFalse(result['insideControlSupport'])
        self.assertFalse(result['controlEligible'])
        with self.assertRaisesRegex(ValueError, 'outside_training_range'):
            predict(self.model, self.row['before'], self.row['command'], self.row['context'])
        self.assertEqual(saved, self.model)

    def test_zero_reverse_and_large_response_remain_evaluation_targets(self):
        proof = self.proof()
        for delta in (0., -5., 100.):
            self.row['after'] = [x+delta for x in self.row['before']]
            result = evaluate(self.model, [self.row], {'s5': proof})
            self.assertEqual(result['evaluated'], 1)
            self.assertEqual(result['outsideControlSupportCount'], 1)
            for baseline in ('stateRidge', 'commandMean', 'noChange'):
                self.assertEqual(result['metrics']['all'][baseline]['n'], 1)
            self.assertFalse(result['controlEligible'])

    def test_pre_freeze_or_unpersisted_frames_rejected(self):
        for field, value in [('capturedAt', 8.), ('persistedAt', 12.)]:
            changed = copy.deepcopy(self.frames); changed[0][field] = value
            with self.assertRaisesRegex(ValueError, 'new_persisted_inputs'):
                prospective_evidence(self.model, self.freeze, self.row, changed, 11.)

    def test_mutated_prediction_or_late_proof_is_counted_rejected(self):
        for mutate in ('prediction', 'late'):
            proof = self.proof()
            if mutate == 'prediction': proof['predictions']['stateRidge'][0] += 1
            else: proof['preparedAt'] = 13.
            result = evaluate(self.model, [self.row], {'s5': proof})
            self.assertEqual(result['listed'], 1)
            self.assertEqual(result['evaluated'], 0)
            self.assertEqual(len(result['rejected']), 1)

    def test_training_group_cannot_be_relabelled_future(self):
        self.row['captureGroup'] = 'train'
        with self.assertRaisesRegex(ValueError, 'group_overlap'):
            evaluate(self.model, [self.row], {'s5': self.proof()})

    def test_original_intent_and_pixel_proof_must_match(self):
        import json
        import tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from learning.artifacts import ArtifactStore
        from learning.experiment_pipeline import encoded
        from learning.batch_runtime import sha
        from learning.batch_fixed_evaluation import freeze_model, audit_proof, seal_before
        with tempfile.TemporaryDirectory() as directory:
            repository=Path(directory);local=repository/'.local';local.mkdir()
            model_path=local/'model.json';model_path.write_bytes(json.dumps(self.model,sort_keys=True,separators=(',', ':'),ensure_ascii=True,allow_nan=False).encode())
            binding=dict(path=str(model_path),sha256=sha(model_path.read_bytes()))
            frozen_binding=freeze_model(repository,binding,local/'freeze.json',clock=SimpleNamespace(monotonic=lambda:8.))
            freeze=json.loads((local/'freeze.json').read_bytes())
            proof=prospective_evidence(self.model,freeze,self.row,self.frames,11.)
            store=ArtifactStore(local/'source')
            manager=SimpleNamespace(store=store,command_count=0,current=dict(phase='recording',sessionId='s5',events=[]))
            manager._event=lambda event:manager.current['events'].append(event)
            seal_before(manager,proof)
            identity=manager.current['fixedEvaluationProofSha256']
            intent=dict(at=12.,fixedEvaluationProofSha256=identity)
            store.write_json('command-001-intent.json',intent)
            store.write_json('session.json',manager.current)
            evaluated=copy.deepcopy(self.row);evaluated['sourceHash']=sha((store.root/'session.json').read_bytes())
            transition=dict(learningRow=evaluated,before=dict(derivedMetadata=self.frames[0]),
                gripperPair=dict(beforeObservation=dict(frameBinding=self.frames[1])))
            self.assertEqual(audit_proof(repository,store.root,transition,self.model,frozen_binding),proof)
            changed=copy.deepcopy(transition);changed['gripperPair']['beforeObservation']['frameBinding']['pixelSha256']='f'*64
            with self.assertRaisesRegex(ValueError,'evidence_changed'):
                audit_proof(repository,store.root,changed,self.model,frozen_binding)
            intent['fixedEvaluationProofSha256']='f'*64
            (store.root/'command-001-intent.json').write_bytes(encoded(intent))
            with self.assertRaisesRegex(ValueError,'precommand_seal'):
                audit_proof(repository,store.root,transition,self.model,frozen_binding)
            manager.command_count=1
            with self.assertRaisesRegex(ValueError,'precede_command'):
                seal_before(manager,proof)
