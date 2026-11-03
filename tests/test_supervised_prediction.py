import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from learning.supervised_prediction import (
    CONTRACT, load_index, output_summary, post_features, pre_features, train_and_evaluate,
)
from learning.supervised_measurement import DATA_SCHEMA, sha
from tools.train_supervised_prediction import main


def rows():
    result = []
    for i in range(12):
        sign = 1 if i % 2 else -1
        angle = 5. + i
        delta = sign * (1. + i / 10)
        result.append(dict(id=f'row-{i}', captureGroup='one-group', split='development',
                           measurementSource=f'block-{i//2}.json', status='eligible',
                           executionStatus='executed',
                           pulse=dict(joint='wrist', motor=2, direction='up' if sign == 1 else 'down',
                                      percent=63, expectedPwm=160*sign, durationMs=550),
                           before=dict(value=angle, velocityPerSecond=.1),
                           after=dict(value=angle+delta), delta=delta,
                           outputTrace=[dict(sequence=1, atMs=100, motor=2, pwm=10*sign),
                                        dict(sequence=2, atMs=200, motor=2, pwm=50*sign),
                                        dict(sequence=3, atMs=400, motor=1, pwm=0),
                                        dict(sequence=4, atMs=400, motor=2, pwm=0)]))
    return result


def fixture(root):
    rr = rows()
    config = dict(schemaVersion='motionbrain.endpoint_measurement.v1', width=100, height=100,
                  stabilityThreshold=.5,
                  points=[dict(id=n, roi=[0, 0, 100, 100], hsvLower=[0, 0, 0],
                               hsvUpper=[179, 255, 255], minAreaPx=1, maxAreaPx=100) for n in 'ABCD'],
                  representation=dict(kind='relative_angle', parent=['A', 'B'], child=['C', 'D']))
    config_hash = sha(json.dumps(config, sort_keys=True, separators=(',', ':')).encode())
    index = dict(schemaVersion='motionbrain.local_supervised_endpoint_index.v1', contract=CONTRACT,
                 split='development', captureGroups=['one-group'], count=len(rr),
                 directionCounts=dict(up=6, down=6), independentTestSetAvailable=False,
                 legacyCommandPlus4SecondsDataIncluded=False, sources=[], entries=[])
    for block in dict.fromkeys(r['measurementSource'] for r in rr):
        selected = [dict(r) for r in rr if r['measurementSource'] == block]
        for r in selected:
            r.pop('measurementSource')
            index['entries'].append(dict(measurementSource=block, row=r))
        source = dict(schemaVersion=DATA_SCHEMA, contract=CONTRACT, units='projected_degrees',
                      velocityUnits='projected_degrees_per_second', physicalJointAngleCalibrated=False,
                      captureGroup='one-group', split='development', measurementConfig=config,
                      measurementConfigSha256=config_hash, rows=selected)
        raw = json.dumps(source).encode()
        (root/block).write_bytes(raw)
        index['sources'].append(dict(path=block, sha256=sha(raw), configHash=config_hash))
    path = root/'index.json'
    path.write_text(json.dumps(index))
    return path, index


class PredictionTests(unittest.TestCase):
    def test_piecewise_integral_motor_filter_sign_and_clock_wrap(self):
        for row in rows()[:2]:
            expected = 1 if row['pulse']['direction'] == 'up' else -1
            summary = output_summary(row)
            self.assertEqual(summary['activeDurationMs'], 300)
            self.assertEqual(summary['signedPwmSeconds'], expected*11)
            self.assertEqual(summary['absolutePwmSeconds'], 11)
            self.assertEqual(summary['signedPeakPwm'], expected*50)
            for event in row['outputTrace']:
                event['atMs'] = (event['atMs'] + 2**32 - 150) % 2**32
            self.assertEqual(output_summary(row), summary)

    def test_bad_trace_rejected(self):
        for mutate in (lambda r: r['outputTrace'].pop(),
                       lambda r: r['outputTrace'][1].update(sequence=8),
                       lambda r: r['outputTrace'][1].update(pwm=200),
                       lambda r: r['outputTrace'][1].update(motor=3),
                       lambda r: r['outputTrace'][1].update(atMs=99)):
            row = rows()[0]
            mutate(row)
            with self.assertRaises(ValueError):
                output_summary(row)

    def test_pre_execution_does_not_read_future(self):
        row = rows()[0]
        expected = pre_features(row)
        row.pop('after'); row.pop('delta'); row.pop('outputTrace')
        self.assertEqual(pre_features(row), expected)
        original = rows()
        changed = copy.deepcopy(original)
        for r in changed:
            r['outputTrace'][1]['atMs'] += 25
        first, second = train_and_evaluate(original), train_and_evaluate(changed)
        self.assertEqual(first['models']['preExecution'], second['models']['preExecution'])
        self.assertEqual(first['evaluation']['metrics']['preExecution'],
                         second['evaluation']['metrics']['preExecution'])
        self.assertNotEqual(post_features(original[0]), post_features(changed[0]))

    def test_holdout_labels_and_features_never_fit_that_fold(self):
        rr = rows()
        first = train_and_evaluate(rr)
        changed = copy.deepcopy(rr)
        for r in changed[:2]:
            r['delta'] += 100
            r['before']['value'] += 50
        second = train_and_evaluate(changed)
        self.assertEqual(first['evaluation']['folds'][0]['models'],
                         second['evaluation']['folds'][0]['models'])
        for fold in first['evaluation']['folds']:
            self.assertFalse(set(fold['fitIds']) & set(fold['heldOutIds']))
            fit_rows = [r for r in rr if r['id'] in fold['fitIds']]
            np.testing.assert_allclose(fold['models']['preExecution']['mean'],
                                       np.mean([pre_features(r) for r in fit_rows], axis=0))
        self.assertFalse(first['controlEligible'])
        self.assertFalse(first['evaluation']['independentTest'])
        self.assertEqual(first['evaluation']['metrics']['noChange']['n'], 12)
        self.assertEqual(first['models']['preExecution']['model']['fitRows'], 12)

    def test_small_or_one_block_refused(self):
        with self.assertRaises(ValueError):
            train_and_evaluate(rows()[:6])
        rr = rows()
        for r in rr:
            r['measurementSource'] = 'one'
        with self.assertRaises(ValueError):
            train_and_evaluate(rr)

    def test_loader_hash_rows_contract_counts_and_groups(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path, index = fixture(root)
            loaded, metadata = load_index(path)
            self.assertEqual(len(loaded), 12)
            self.assertFalse(metadata['rawFramesReaudited'])
            for mutate in (lambda d: d.update(contract='command_plus_4s'),
                           lambda d: d.update(split='test'),
                           lambda d: d.update(count=11),
                           lambda d: d.update(captureGroups=['a', 'b']),
                           lambda d: d['entries'][0]['row'].update(delta=99),
                           lambda d: d['entries'].append(d['entries'][0]),
                           lambda d: d['entries'].pop(),
                           lambda d: d['sources'][0].update(path='../escape.json')):
                changed = copy.deepcopy(index); mutate(changed)
                path.write_text(json.dumps(changed))
                with self.assertRaises((ValueError, KeyError)):
                    load_index(path)
            path.write_text(json.dumps(index))
            (root/'block-0.json').write_text('{}')
            with self.assertRaisesRegex(ValueError, 'hash_mismatch'):
                load_index(path)

    def test_cli_stores_model_receipt_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path, _ = fixture(root)
            args = ['--index', str(path), '--output-dir', str(root/'run')]
            self.assertEqual(main(args), 0)
            raw = (root/'run/development-evaluation.json').read_bytes()
            receipt = json.loads((root/'run/result.json').read_text())
            self.assertEqual(receipt['artifact']['sha256'], sha(raw))
            self.assertEqual(main(args), 2)
            self.assertEqual((root/'run/development-evaluation.json').read_bytes(), raw)


if __name__ == '__main__':
    unittest.main()
