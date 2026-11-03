import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.artifacts import ArtifactStore, StorageError
from learning.supervised_evaluation import freeze
from learning.supervised_measurement import sha
from learning.supervised_prediction import run
from test_supervised_prediction import fixture
from tools.freeze_supervised_evaluation import main


def training_fixture(root):
    path, index = fixture(root)
    for i, ref in enumerate(index['sources']):
        source_path = root/ref['path']
        source = json.loads(source_path.read_text())
        source['collectionReportSha256'] = sha(f'report-{i}'.encode())
        source['sourceHashes'] = {'image': sha(f'image-{i}'.encode())}
        for row in source['rows']:
            row['pulse']['durationMs'] = 450 if i % 2 else 550
            next(e for e in index['entries'] if e['row']['id'] == row['id'])['row'] = row
        raw = json.dumps(source).encode()
        source_path.write_bytes(raw); ref['sha256'] = sha(raw)
    path.write_text(json.dumps(index))
    artifact = run(path)
    save_run(root/'training', artifact)
    return path, artifact


def save_run(path, artifact):
    store = ArtifactStore(path)
    ref = store.write_json('development-evaluation.json', artifact)
    store.write_json('result.json', dict(status='trained_and_explored', count=12,
                                        artifact=ref, independentTest=False, controlEligible=False))


class FreezeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.index, self.artifact = training_fixture(self.root)

    def test_model_copied_exactly_baseline_fixed_without_fit_or_post_model(self):
        with patch('learning.supervised_prediction.fit', side_effect=AssertionError('must not fit')):
            frozen = freeze(self.index, self.root/'training')
        self.assertEqual(frozen['model'], self.artifact['models']['preExecution'])
        self.assertNotIn('postExecution', frozen)
        self.assertNotIn('models', frozen)
        data = json.loads(self.index.read_text())['entries']
        for d in ('up', 'down'):
            values = [e['row']['delta'] for e in data if e['row']['pulse']['direction'] == d]
            self.assertAlmostEqual(frozen['baselines']['directionMean'][d]['deltaDeg'], sum(values)/len(values))
        self.assertEqual(frozen['scope']['plannedConditionCount'], 4)
        self.assertEqual(sum(c['developmentCount'] for c in frozen['scope']['conditions']), 12)
        self.assertFalse(frozen['scope']['observedBoundsAreSafetyLimits'])
        self.assertFalse(frozen['evaluationPerformed'])
        self.assertFalse(frozen['collectionReadiness']['collectionAuthorized'])
        self.assertIsNone(frozen['collectionReadiness']['attemptBudget'])

    def test_post_model_changes_cannot_change_primary_comparison(self):
        altered = copy.deepcopy(self.artifact)
        altered['models']['postExecution']['model']['coefficients'] = [999.] * 6
        save_run(self.root/'changed-post', altered)
        first, second = freeze(self.index, self.root/'training'), freeze(self.index, self.root/'changed-post')
        for key in ('model', 'modelSha256', 'baselines', 'scope', 'evaluationPolicy'):
            self.assertEqual(first[key], second[key])

    def test_wrong_model_contract_normalization_or_source_refused(self):
        mutations = [lambda a: a['models'].__setitem__('preExecution', a['models']['postExecution']),
                     lambda a: a['models']['preExecution']['model'].update(scale=[0.]*6),
                     lambda a: a['models']['preExecution']['model'].update(mean=[0.]*6),
                     lambda a: a['models']['preExecution']['model'].update(fitRows=16),
                     lambda a: a.update(contract='command_plus_4s'),
                     lambda a: a['metadata'].update(indexSha256='0'*64),
                     lambda a: a['metadata']['sourceSha256'].update({'learning/joint_prediction.py': '0'*64})]
        for i, mutate in enumerate(mutations):
            with self.subTest(i=i):
                changed = copy.deepcopy(self.artifact); mutate(changed)
                directory = self.root/f'changed-{i}'; save_run(directory, changed)
                with self.assertRaises(ValueError):
                    freeze(self.index, directory)

    def test_modified_artifact_rejected_by_receipt(self):
        (self.root/'training/development-evaluation.json').write_text('{}')
        with self.assertRaises(StorageError):
            freeze(self.index, self.root/'training')

    def test_source_and_identity_fingerprints_retained(self):
        frozen = freeze(self.index, self.root/'training')
        source = frozen['provenance']
        self.assertEqual(len(source['excludedCollectionReportSha256']), 6)
        self.assertEqual(len(source['excludedOriginalSourceSha256']), 6)
        self.assertEqual(len(source['excludedRowIds']), 12)
        self.assertEqual(len(source['excludedRowContentSha256']), 12)
        self.assertFalse(frozen['evaluationPolicy']['newGroupNameAloneProvesIndependence'])
        self.assertEqual(frozen['measurementProtocol']['afterConfirmedOffWindowSeconds'], [2.25, 3.])

    def test_cli_immutable_output_and_refusal(self):
        args = ['--index', str(self.index), '--training-run', str(self.root/'training'),
                '--output-dir', str(self.root/'frozen')]
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(args), 0)
            before = (self.root/'frozen/evaluation-bundle.json').read_bytes()
            self.assertEqual(main(args), 2)
        self.assertEqual((self.root/'frozen/evaluation-bundle.json').read_bytes(), before)
        receipt = json.loads((self.root/'frozen/result.json').read_text())
        self.assertEqual(receipt['artifact']['sha256'], sha(before))


if __name__ == '__main__':
    unittest.main()
