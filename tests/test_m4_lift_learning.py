import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from learning import m4_lift_learning as lift


class LiftLearningTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.context = dict(scene='fixed_test_view', load='cylinder_between_jaws')
        rng = np.random.default_rng(42)
        self.patch = rng.integers(0, 255, (45, 55, 3), dtype=np.uint8)
        self.record = self.images('reference')
        self.reg = lift.register(self.root/'reference-external.png', self.root/'reference-gripper.png', self.context)

    def images(self, name, rise=0, slip=(0, 0)):
        external = np.full((900, 1000, 3), 255, np.uint8)
        external[730-rise:810-rise, 710:770] = 40
        external[620-rise:630-rise, 890:900] = [255, 0, 255]
        gripper = np.full((200, 240, 3), 220, np.uint8)
        dx, dy = slip
        gripper[100+dy:145+dy, 130+dx:185+dx] = self.patch
        record = dict(id=name, context=copy.deepcopy(self.context))
        for key, image in [('external', external), ('gripper', gripper)]:
            path = self.root / (name+'-'+key+'.png')
            cv2.imwrite(str(path), image)
            record[key] = dict(path=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        return record

    def observation(self, row_id, state):
        return dict(id=row_id, context=copy.deepcopy(self.context), state=state,
                    contractSha256=lift.contract_sha(self.reg), quality=dict(valid=True, ncc=.99, slipDxPx=0),
                    sourceSha256={k: hashlib.sha256((row_id+k).encode()).hexdigest() for k in ('external', 'gripper')})

    def rows(self, prefix='train'):
        return [dict(id=prefix+str(i), context=copy.deepcopy(self.context),
                     contractSha256=lift.contract_sha(self.reg), pulse=copy.deepcopy(lift.PULSES[i//2]),
                     before=self.observation(prefix+str(i)+'before', [i%2, 0, 0]),
                     after=self.observation(prefix+str(i)+'after', [8 if i < 2 else 19, 12, 0])) for i in range(4)]

    def frozen(self):
        model = lift.fit(self.rows(), self.reg, self.context)
        path = self.root/'model.json'
        sha = lift.save_model(path, model)
        return lift.FrozenLiftModel(path, sha, self.reg, self.root/'decisions', self.root)

    def test_metrology_and_source_hash(self):
        result = lift.measure(self.root, self.images('moved', rise=20, slip=(2, 3)), self.reg)
        np.testing.assert_allclose(result['state'], [20, 20, 3])
        self.assertEqual(result['quality']['slipDxPx'], 2)
        record = self.images('tampered')
        (self.root/record['external']['path']).write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'source_hash_changed'):
            lift.measure(self.root, record, self.reg)

    def test_frozen_select_and_persisted_prediction(self):
        frozen = self.frozen()
        selected = frozen.select(self.record)
        self.assertEqual(selected['pulse'], lift.PULSES[1])
        raw = Path(selected['decisionPath']).read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), selected['decisionSha256'])
        self.assertEqual(json.loads(raw)['features']['sourceSha256'], selected['features']['sourceSha256'])
        evaluation = frozen.predict(self.record, lift.PULSES[0])
        self.assertFalse(evaluation['modelSelected'])
        with self.assertRaises(FileExistsError):
            lift.save_model(frozen.path, frozen.model)
        frozen.path.write_bytes(b'{}')
        with self.assertRaisesRegex(ValueError, 'frozen_model_changed'):
            frozen.verify()

    def test_pre_domain_failures_no_clamping_or_fallback(self):
        frozen = self.frozen()
        for record in [self.images('outside', rise=4), self.images('slip', slip=(5, 0))]:
            with self.assertRaisesRegex(ValueError, 'outside_registered_operating_domain'):
                frozen.select(record)
        record = copy.deepcopy(self.record)
        record['actualPwm'] = 127
        with self.assertRaisesRegex(ValueError, 'before_record_fields_invalid'):
            frozen.select(record)
        record = copy.deepcopy(self.record)
        record['context']['load'] = 'other_load'
        with self.assertRaisesRegex(ValueError, 'scene_context_mismatch'):
            frozen.select(record)
        self.assertFalse((self.root/'decisions').exists())

    def test_support_contract_and_holdout_reuse(self):
        rows = self.rows()
        for bad in [rows[:3], rows[:2]+copy.deepcopy(rows[:2])]:
            with self.assertRaises(ValueError):
                lift.fit(bad, self.reg, self.context)
        altered = copy.deepcopy(rows)
        altered[0]['after']['context']['scene'] = 'different'
        with self.assertRaisesRegex(ValueError, 'observation_contract_mismatch'):
            lift.fit(altered, self.reg, self.context)
        model = lift.fit(rows, self.reg, self.context)
        with self.assertRaisesRegex(ValueError, 'evaluation_source_reused'):
            lift.evaluate(model, rows[:2])
        heldout = [self.rows('eval')[0], self.rows('eval')[2]]
        report = lift.evaluate(model, heldout)
        self.assertEqual(set(report['meanAbsoluteErrorPx']), {'local_visual_translation', 'commandOnly', 'noChange'})
        self.assertEqual(report['evaluated'], 2)
        heldout[1]['before']['sourceSha256'] = copy.deepcopy(rows[0]['after']['sourceSha256'])
        with self.assertRaisesRegex(ValueError, 'evaluation_source_reused'):
            lift.evaluate(model, heldout)

    def test_in_memory_change_and_fixed_domain_rejected(self):
        frozen = self.frozen()
        frozen.model['candidates']['0']['delta'][0] += 100
        with self.assertRaisesRegex(ValueError, 'frozen_model_changed'):
            frozen.verify()
        reg = copy.deepcopy(self.reg)
        reg['operatingDomain']['objectRisePx'] = 100
        row = self.observation('changed', [0, 0, 0])
        row['contractSha256'] = lift.contract_sha(reg)
        with self.assertRaisesRegex(ValueError, 'registered_domain_changed'):
            lift._state(row, reg, True)

    def test_quality_and_path_escape_reject(self):
        record = self.images('quality')
        path = self.root/record['gripper']['path']
        cv2.imwrite(str(path), np.zeros((200, 240, 3), np.uint8))
        record['gripper']['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, 'slip_template_quality_insufficient'):
            lift.measure(self.root, record, self.reg)
        record['external']['path'] = '../outside.png'
        with self.assertRaisesRegex(ValueError, 'source_path_escape'):
            lift.measure(self.root, record, self.reg)

    def audited_rows(self, prefix, start):
        rows = self.rows(prefix)
        for i, row in enumerate(rows):
            row['episodeIdentity'] = ['controller-boot', start+i]
            row['sourceCaptureIds'] = [prefix+str(i)+':capture:'+str(j) for j in range(4)]
            row['sourceEvidenceSha256'] = [hashlib.sha256((prefix+str(i)+':receipt:'+str(j)).encode()).hexdigest()
                                          for j in range(4)]
            for side in ('before', 'after'):
                row[side]['sourceSha256'] = {'external': 'a'*64, 'gripper': 'b'*64}
        return rows

    def test_audited_receipts_allow_identical_pixels_and_preserve_manifest(self):
        model = lift.fit(self.audited_rows('audited_train', 1), self.reg, self.context)
        self.assertEqual(len(model['trainingSources']), 4)
        self.assertEqual(model['sourceIdentityMode'], 'audited_receipts_v1')
        heldout = self.audited_rows('audited_eval', 101)[::2]
        self.assertEqual(lift.evaluate(model, heldout)['evaluated'], 2)
        path = self.root/'audited-model.json'
        sha = lift.save_model(path, model)
        self.assertTrue(lift.FrozenLiftModel(path, sha, self.reg, self.root/'decisions').verify())
        for field in ('episodeIdentity', 'sourceCaptureIds', 'sourceEvidenceSha256'):
            reused = copy.deepcopy(heldout)
            reused[0][field] = copy.deepcopy(self.audited_rows('audited_train', 1)[0][field])
            with self.assertRaisesRegex(ValueError, 'evaluation_source_reused'):
                lift.evaluate(model, reused)

    def test_source_proof_modes_cannot_be_mixed(self):
        rows = self.audited_rows('audited_train', 1)
        for field in ('episodeIdentity', 'sourceCaptureIds', 'sourceEvidenceSha256'):
            del rows[0][field]
        with self.assertRaises(ValueError):
            lift.fit(rows, self.reg, self.context)
        rows = self.audited_rows('audited_train', 1)
        del rows[0]['episodeIdentity']
        with self.assertRaisesRegex(ValueError, 'partial_audited_source_identity'):
            lift.fit(rows, self.reg, self.context)

    def test_no_feasible_lift_and_ambiguous_choices_reject_but_evaluation_predicts(self):
        for deltas, error in [([[0, 0, 0], [20, 0, 5]], 'no_feasible_lift_candidate'),
                              ([[19, 0, 0], [19.5, 0, 0]], 'ambiguous_lift_candidates')]:
            model = lift.fit(self.rows(), self.reg, self.context)
            for i, delta in enumerate(deltas):
                model['candidates'][str(i)]['delta'] = delta
            path = self.root/(error+'.json'); sha = lift.save_model(path, model)
            frozen = lift.FrozenLiftModel(path, sha, self.reg, self.root/'decisions', self.root)
            with self.assertRaisesRegex(ValueError, error):
                frozen.select(self.record)
            self.assertFalse(frozen.predict(self.record, lift.PULSES[0])['modelSelected'])


if __name__ == '__main__':
    unittest.main()
