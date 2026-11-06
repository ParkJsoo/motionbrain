"""Frozen artifacts, independent evaluation and no-refit regression checks."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning import relative_action_learning as learning
from learning import relative_model_bundle as bundle
from tests.test_relative_action_learning import row, C, CONTEXT
from tools import replay_relative_actions as cli


def complete_context():
    return dict(CONTEXT, cameraIdentity={'deviceId': 'synthetic-camera'}, imageShape=[100, 100],
                stateFields=learning.STATE)


def model():
    rows = [row(i, x) for i, x in enumerate((-4., 0., 4.))]
    for r in rows:
        r['context'] = complete_context()
    return learning.fit(rows)


class FrozenRelativeTests(unittest.TestCase):
    def test_valid_historical_model_is_unchanged(self):
        value = model()
        self.assertEqual(bundle.validate_model(value), value)
        self.assertFalse(value['controlEligible'])

    def test_corrupted_parameters_provenance_and_context_rejected(self):
        mutations = [
            lambda d: d.update(controlEligible=True),
            lambda d: d.update(alpha=True),
            lambda d: d['context'].update(stateFields=['wrong']),
            lambda d: d['context'].pop('poseRegime'),
            lambda d: d['models'][0].update(count=1),
            lambda d: d['models'][0].update(count=4),
            lambda d: d['models'][0].update(scale=[1., 0., 1.]),
            lambda d: d['models'][0].update(coefficient=[[1., 2.]] * 3),
            lambda d: d['models'][0].update(deltaMean=[float('nan'), 0., 0.]),
            lambda d: d['models'].append(copy.deepcopy(d['models'][0])),
            lambda d: d['trainingEpisodes'].__setitem__(1, d['trainingEpisodes'][0]),
        ]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                value = model(); mutate(value)
                with self.assertRaises(ValueError):
                    bundle.validate_model(value)

    def test_hash_and_strict_json_binding(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.json'
            raw = json.dumps(model()).encode(); path.write_bytes(raw)
            self.assertEqual(bundle.load_frozen_model(path, bundle.sha256(raw)), model())
            with self.assertRaisesRegex(ValueError, 'hash_changed'):
                bundle.load_frozen_model(path, '0' * 64)
            raw = b'{"schemaVersion":1,"schemaVersion":2}'; path.write_bytes(raw)
            with self.assertRaisesRegex(ValueError, 'duplicate JSON'):
                bundle.load_frozen_model(path, bundle.sha256(raw))

    def test_frozen_cli_evaluates_and_queries_without_fitting_or_rewriting_model(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); local = root / '.local'; local.mkdir()
            source = local / 'model.json'; raw = json.dumps(model()).encode(); source.write_bytes(raw)
            manifest = local / 'manifest.json'
            manifest.write_text(json.dumps(dict(records=[], evaluationGroups=['eval'], query=dict(
                state=[2., 2., 10.], target=[-2., 2., 10.], commands=[C], context=complete_context()))))
            evaluation = row(9, 2., group='eval'); evaluation['context'] = complete_context()
            with patch.object(cli, 'fit', side_effect=AssertionError('refit forbidden')), \
                    patch.object(cli, 'load_manifest', return_value=([evaluation], [])):
                report = cli.run(manifest, local / 'out', frozen_model=source,
                                 model_sha256=bundle.sha256(raw), root=root)
            self.assertEqual(report['evaluated'], 1)
            self.assertFalse(report['modelRefit'])
            self.assertEqual(report['selection']['selected']['kind'], 'action')
            self.assertEqual(source.read_bytes(), raw)
            self.assertFalse((local / 'out' / 'model.json').exists())
            with self.assertRaises(Exception):
                cli.run(manifest, local / 'out', frozen_model=source,
                        model_sha256=bundle.sha256(raw), root=root)

    def test_frozen_cli_rejects_training_declaration_even_when_source_would_fail_audit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); path = root / 'manifest.json'
            path.write_text(json.dumps(dict(records=[{'annotation': {'captureGroup': 'train'}}],
                                            evaluationGroups=['eval'])))
            with patch.object(cli, 'load_manifest', side_effect=AssertionError('must reject first')):
                with self.assertRaisesRegex(ValueError, 'only_list_evaluation'):
                    cli.run(path, root / '.local/out', frozen_model=root / 'missing',
                            model_sha256='0' * 64, root=root)

    def test_evaluation_overlap_is_not_hidden_by_frozen_cli(self):
        value = model()
        overlap = row(0, -4., group='eval'); overlap['context'] = complete_context()
        with self.assertRaisesRegex(ValueError, 'overlap'):
            learning.evaluate(bundle.validate_model(value), [overlap])


if __name__ == '__main__':
    unittest.main()
