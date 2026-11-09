"""합성 전용 M1 shaft 모델 검사. 장치 요청·실물 성공 주장이 없다."""
import copy
import unittest

from learning import m1_shaft_learning as m
from learning import relative_action_learning as relative
from tests.test_pipeline_learning import row


def sample(name, group, shaft):
    value = row(name, group)
    value['context']['landmarks'] = m.PREFIX+'a'*64
    value['context']['stateFields'] = relative.state_fields(value['context'])
    value['before'] = [0., 0., shaft]
    value['after'] = [0., 0., 120.-shaft]
    return value


def scalar_sample(name, group, shaft):
    value = sample(name, group, shaft)
    value.update(schemaVersion=m.TRANSITION_SCHEMA,
                 beforeShaftPx=value.pop('before')[2], afterShaftPx=value.pop('after')[2])
    value['context']['landmarks'] = m.SHAFT_PREFIX+'b'*64
    value['context']['stateFields'] = ['shaftSeparationPx']
    return value


class ShaftTests(unittest.TestCase):
    def test_native_scalar_needs_no_object_points_or_xy(self):
        rows = [scalar_sample('s1', 'train', 40.), scalar_sample('s2', 'train', 60.)]
        original = copy.deepcopy(rows)
        model = m.fit(rows)
        heldout = scalar_sample('s3', 'eval', 55.)
        report = m.evaluate(model, [heldout])
        self.assertEqual(report['evaluated'], 1)
        self.assertIsNone(report['predictions'][0]['bodyRelativeXYDiagnostic'])
        self.assertEqual(rows, original)
        self.assertEqual(model['context']['stateFields'], ['shaftSeparationPx'])
        for field in ('afterShaftPx', 'sourceHash'):
            bad = copy.deepcopy(rows); bad[0].pop(field)
            with self.subTest(field=field), self.assertRaises(ValueError): m.fit(bad)
        bad = copy.deepcopy(rows); bad[0]['before'] = [0., 0., 40.]
        with self.assertRaisesRegex(ValueError, 'must_not_supply_3d_state'): m.fit(bad)

    def test_scalar_provenance_scope_and_contracts_remain_strict(self):
        rows = [scalar_sample('s1', 'train', 40.), scalar_sample('s2', 'train', 60.)]
        model = m.fit(rows)
        for field in ('sessionId', 'captureGroup', 'episode', 'frameIds', 'sourceHash'):
            heldout = scalar_sample('s3', 'eval', 55.)
            heldout[field] = copy.deepcopy(rows[0][field])
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'overlap'):
                m.evaluate(model, [heldout])
        bad = copy.deepcopy(rows)
        bad[0]['command'] = dict(joint='shoulder', direction='down', percent=20, durationMs=350)
        with self.assertRaisesRegex(ValueError, 'requires_gripper_only'): m.fit(bad)
        with self.assertRaisesRegex(ValueError, 'requires_gripper_only'):
            m.predict(model, 50., bad[0]['command'], model['context'])
        with self.assertRaisesRegex(ValueError, 'schema_changed'):
            m.fit([rows[0], sample('legacy', 'train', 60.)])
        bad = copy.deepcopy(rows); bad[0]['context']['stateFields'] = relative.CROSSSECTION_STATE
        bad[1]['context'] = copy.deepcopy(bad[0]['context'])
        with self.assertRaisesRegex(ValueError, 'crosssection_context_required'): m.fit(bad)

    def test_scalar_frozen_model_cannot_drop_source_hash_provenance(self):
        model = m.fit([scalar_sample('s1', 'train', 40.), scalar_sample('s2', 'train', 60.)])
        model['trainingSourceHashes'] = []
        with self.assertRaisesRegex(ValueError, 'model_source_hashes_required'):
            m.validate_model(model)

    def setUp(self):
        self.rows = [sample('a', 'train', 40.), sample('b', 'train', 60.)]
        self.model = m.fit(self.rows)
        self.test = sample('eval', 'eval', 55.)

    def test_scalar_ridge_beats_baselines_and_xy_is_diagnostic_only(self):
        result = m.evaluate(self.model, [self.test])
        self.assertEqual(result['evaluated'], 1)
        self.assertLess(result['metrics']['shaftRidge']['maePx'], result['metrics']['commandMean']['maePx'])
        self.assertLess(result['metrics']['shaftRidge']['maePx'], result['metrics']['noChange']['maePx'])
        changed = copy.deepcopy(self.test)
        changed['before'][:2] = [9999., -9999.]; changed['after'][:2] = [-12., 123.]
        self.assertEqual(result['metrics'], m.evaluate(self.model, [changed])['metrics'])
        self.assertFalse(result['temporalOrderVerified']); self.assertIsNone(result['evaluationPassed'])
        self.assertFalse(result['controlEligible'])

    def test_strict_shaft_gate_singletons_context_and_scope(self):
        bad = copy.deepcopy(self.test); bad['before'][2] = 60.01
        report = m.evaluate(self.model, [bad])
        self.assertEqual((report['listed'], report['evaluated']), (1, 0))
        self.assertEqual(report['rejected'][0]['reason'], 'shaft_outside_training_range')
        self.assertEqual(m.fit(self.rows[:1])['models'], [])
        with self.assertRaisesRegex(ValueError, 'requires_gripper_only'):
            m.predict(self.model, 50., dict(self.test['command'], joint='shoulder', direction='down', percent=20), self.test['context'])
        with self.assertRaisesRegex(ValueError, 'context_changed'):
            m.predict(self.model, 50., self.test['command'], dict(self.test['context'], poseRegime='other'))
        bad = copy.deepcopy(self.rows)
        for r in bad:
            r['context']['landmarks'] = 'registered-template-points-v1:'+'a'*64
            r['context']['stateFields'] = relative.STATE.copy()
        with self.assertRaisesRegex(ValueError, 'crosssection_context_required'): m.fit(bad)

    def test_provenance_overlap_never_becomes_evaluation(self):
        for field in ('sessionId', 'episode', 'frameIds', 'captureGroup', 'sourceHash'):
            bad = copy.deepcopy(self.test); bad[field] = copy.deepcopy(self.rows[0][field])
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'overlap'):
                m.evaluate(self.model, [bad])

    def test_target_independence_frozen_hash_and_corruption(self):
        before = m.canonical_bytes(self.model)
        first = m.evaluate(self.model, [self.test])['predictions'][0]['shaftRidge']
        self.test['after'][2] = 999.
        self.assertEqual(first, m.evaluate(self.model, [self.test])['predictions'][0]['shaftRidge'])
        self.assertEqual(before, m.canonical_bytes(self.model))
        self.assertEqual(self.model, m.validate_model(self.model))
        for mutation in (lambda x:x.update(controlEligible=True),
                         lambda x:x['models'][0].update(scale=0.),
                         lambda x:x['models'][0].update(coefficient=float('nan'))):
            bad = copy.deepcopy(self.model); mutation(bad)
            with self.assertRaises(ValueError): m.validate_model(bad)


if __name__ == '__main__': unittest.main()
