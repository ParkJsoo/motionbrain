import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from learning.joint_action_selection import (
    OfflineActionSelector, canonical_sha256, replay_saved_states,
)
from learning.joint_prediction import INPUT_NAMES
from tests.test_joint_collection_prediction import rows
from tools.replay_joint_action_selection import main


def fixture():
    dataset = dict(schemaVersion='motionbrain.audited_projected_joint_dataset.v1',
                   rows=rows(), controlEligible=False)
    x = np.asarray([r['x'] for r in dataset['rows'] if r['split'] == 'train'])
    scale = x.std(axis=0)
    scale[scale < 1e-9] = 1
    # Exact known mapping delta=6*signed_duration, no fitting in this fixture.
    coefficients = np.zeros(6)
    coefficients[4] = 6 * scale[4]
    artifact = dict(schemaVersion='motionbrain.projected_joint_prediction.v1',
                    inputNames=list(INPUT_NAMES), horizonSeconds=4., controlEligible=False,
                    datasetSha256=canonical_sha256(dataset),
                    selectedModel=dict(mean=x.mean(axis=0).tolist(), scale=scale.tolist(),
                                       coefficients=coefficients.tolist(),
                                       intercept=float(6*x[:, 4].mean()), alpha=0., fitRows=16))
    return artifact, dataset


def observation(angle=6., velocity=0.):
    return dict(angleDeg=angle, velocityDegS=velocity, valid=True, settled=True)


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.artifact, self.dataset = fixture()
        self.selector = OfflineActionSelector(self.artifact, self.dataset)

    def test_ranks_both_directions_and_durations_by_target_error(self):
        for goal, direction, duration in [(10.2, 'up', 700), (11.4, 'up', 900),
                                           (1.8, 'down', 700), (.6, 'down', 900)]:
            with self.subTest(goal=goal):
                result = self.selector.select(observation(), goal)
                self.assertEqual(result['status'], 'selected')
                self.assertEqual(len(result['candidates']), 4)
                self.assertEqual(result['selected']['direction'], direction)
                self.assertEqual(result['selected']['durationMs'], duration)
                self.assertLess(result['selected']['predictedGoalErrorDeg'], 1e-10)
                self.assertFalse(result['controlEligible'])

    def test_hold_within_tolerance_or_when_all_moves_worsen_goal(self):
        close = self.selector.select(observation(), 6.5)
        worse = self.selector.select(observation(), 7.1, tolerance_deg=.1)
        self.assertEqual(close['reason'], 'within_goal_tolerance')
        self.assertEqual(worse['reason'], 'no_predicted_improvement')
        for result in (close, worse):
            self.assertEqual(result['status'], 'hold')
            self.assertEqual(result['selected']['assumption'], 'no_change_not_measured_hold_outcome')
            self.assertNotIn('durationMs', result['selected'])

    def test_exact_ties_prefer_shorter_duration_then_fixed_direction(self):
        self.artifact['selectedModel'].update(coefficients=[0.]*6, intercept=5.)
        selector = OfflineActionSelector(self.artifact, self.dataset)
        selected = selector.select(observation(), 11.)['selected']
        self.assertEqual((selected['direction'], selected['durationMs']), ('down', 700))

    def test_command_specific_angle_bounds_exclude_only_unsupported_candidates(self):
        # Move one training condition's A samples inward; update the normalization
        # fixture while retaining its exact, known feature mapping (no fit).
        for r in self.dataset['rows']:
            if r['split'] == 'train' and r['conditionId'] == 'A-up-900':
                r['x'][0] = 7.; r['x'][5] = 7.
        x = np.asarray([r['x'] for r in self.dataset['rows'] if r['split'] == 'train'])
        scale = x.std(axis=0); scale[scale < 1e-9] = 1
        self.artifact['selectedModel'].update(mean=x.mean(axis=0).tolist(), scale=scale.tolist())
        self.artifact['datasetSha256'] = canonical_sha256(self.dataset)
        selector = OfflineActionSelector(self.artifact, self.dataset)
        result = selector.select(observation(8.), 13.4)
        self.assertEqual(result['selected']['durationMs'], 700)
        rejected = [c for c in result['candidates'] if not c['eligible']]
        self.assertEqual([(c['direction'], c['durationMs']) for c in rejected], [('up', 900)])

    def test_outside_training_bounds_abstains_even_if_at_goal(self):
        for obs, reason in [(observation(20.), 'no_supported_candidate'),
                            (observation(6., .01), 'velocity_outside_training_bounds')]:
            result = self.selector.select(obs, obs['angleDeg'])
            self.assertEqual(result['status'], 'abstain')
            self.assertEqual(result['reason'], reason)
            self.assertIsNone(result['selected'])

    def test_invalid_tracking_or_unsettled_observation_abstains(self):
        for field in ('valid', 'settled'):
            for value in (False, None, 1, 'true'):
                obs = observation(); obs[field] = value
                self.assertEqual(self.selector.select(obs, 10.)['reason'],
                                 'valid_settled_observation_required')

    def test_nonfinite_bool_and_outside_envelope_inputs_rejected(self):
        for value in (float('nan'), float('inf'), 10**400, True, '6'):
            for field in ('angleDeg', 'velocityDegS'):
                obs = observation(); obs[field] = value
                with self.assertRaises(ValueError): self.selector.select(obs, 10.)
            with self.assertRaises(ValueError): self.selector.select(observation(), value)
        for goal in (-11., 25.):
            with self.assertRaises(ValueError): self.selector.select(observation(), goal)
        with self.assertRaises(ValueError):
            self.selector.select(observation(), 10., tolerance_deg=-1)

    def test_overflow_and_outside_prediction_rejected(self):
        for intercept in (100., 1e308):
            self.artifact['selectedModel'].update(intercept=intercept, coefficients=[1e308]*6)
            result = OfflineActionSelector(self.artifact, self.dataset).select(observation(), 10.)
            self.assertEqual(result['status'], 'abstain')
            self.assertTrue(all(c['rejection'] == 'prediction_outside_projected_envelope'
                                for c in result['candidates']))

    def test_tampered_dataset_or_wrong_contract_rejected(self):
        for mutate in (lambda a, d: d['rows'][0].update(y=999),
                       lambda a, d: a.update(horizonSeconds=5),
                       lambda a, d: a.update(inputNames=list(reversed(INPUT_NAMES))),
                       lambda a, d: a.update(controlEligible=True)):
            a, d = fixture(); mutate(a, d)
            with self.assertRaises(ValueError): OfflineActionSelector(a, d)

    def test_invalid_model_shapes_scales_and_training_normalization_rejected(self):
        for field, value in [('scale', [0.]*6), ('mean', [0.]*6),
                             ('coefficients', [0.]*5), ('intercept', float('nan')),
                             ('fitRows', 32), ('alpha', True), ('intercept', 10**400),
                             ('coefficients', [10**400]*6)]:
            a, d = fixture(); a['selectedModel'][field] = value
            with self.assertRaises(ValueError): OfflineActionSelector(a, d)

    def test_oversized_dataset_numbers_rejected_before_legacy_validator(self):
        for field in ('x', 'y'):
            a, d = fixture()
            if field == 'x': d['rows'][0]['x'][0] = 10**400
            else: d['rows'][0]['y'] = 10**400
            a['datasetSha256'] = canonical_sha256(d)
            with self.assertRaisesRegex(ValueError, 'finite_numeric_rows_required'):
                OfflineActionSelector(a, d)

    def test_malformed_objects_and_one_shot_goals_rejected(self):
        with self.assertRaises(ValueError): OfflineActionSelector([], self.dataset)
        with self.assertRaises(ValueError): OfflineActionSelector(self.artifact, [])
        self.artifact['selectedModel'] = []
        with self.assertRaises(ValueError): OfflineActionSelector(self.artifact, self.dataset)
        with self.assertRaises(ValueError): self.selector.select([], 10.)
        a, d = fixture()
        with self.assertRaises(ValueError): replay_saved_states(a, d, iter([0., 10.]))

    def test_no_fit_and_no_input_mutation(self):
        before = copy.deepcopy((self.artifact, self.dataset))
        with patch('learning.joint_prediction.fit', side_effect=AssertionError('must not fit')):
            replay_saved_states(self.artifact, self.dataset, [0., 5., 10.])
        self.assertEqual(before, (self.artifact, self.dataset))
        self.artifact['selectedModel']['coefficients'] = [0.]*6
        self.assertEqual(self.selector.select(observation(), 11.4)['selected']['durationMs'], 900)

    def test_heldout_labels_do_not_change_decisions_or_bounds(self):
        before = replay_saved_states(self.artifact, self.dataset, [0., 5., 10.], split='test')
        for r in self.dataset['rows']:
            if r['split'] != 'train': r['y'] += 1000
        self.artifact['datasetSha256'] = canonical_sha256(self.dataset)
        after = replay_saved_states(self.artifact, self.dataset, [0., 5., 10.], split='test')
        self.assertEqual(before['support'], after['support'])
        for first, second in zip(before['decisions'], after['decisions']):
            self.assertEqual(first['selected'], second['selected'])
            self.assertEqual(first['candidates'], second['candidates'])
            self.assertNotEqual(first['loggedEndDeg'], second['loggedEndDeg'])

    def test_heldout_inputs_do_not_expand_support(self):
        for r in self.dataset['rows']:
            if r['split'] != 'train':
                r['x'][0] += 100; r['x'][5] = r['x'][0]*r['x'][2]; r['x'][1] += 10
        self.artifact['datasetSha256'] = canonical_sha256(self.dataset)
        other = OfflineActionSelector(self.artifact, self.dataset)
        self.assertEqual(other.angle_bounds, self.selector.angle_bounds)
        self.assertEqual(other.velocity_bounds, self.selector.velocity_bounds)

    def test_replay_never_invents_counterfactual_or_hold_outcomes(self):
        report = replay_saved_states(self.artifact, self.dataset, [0., 5., 10., 15.])
        self.assertEqual(len(report['decisions']), 32)
        self.assertIsNone(report['policySuccessRate'])
        self.assertFalse(report['counterfactualOutcomesKnown'])
        self.assertFalse(report['trainingPerformed'])
        for r in report['decisions']:
            if r['selectedMatchesLoggedAction']:
                self.assertEqual(r['selectedObservedGoalErrorDeg'], r['loggedGoalErrorDeg'])
            else:
                self.assertIsNone(r['selectedObservedGoalErrorDeg'])


class SelectionCliTests(unittest.TestCase):
    def test_report_creation_and_existing_source_preservation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); a, d = fixture()
            model = root/'model.json'; dataset = root/'dataset.json'; out = root/'new'/'replay.json'
            model.write_text(json.dumps(a)); dataset.write_text(json.dumps(d))
            original = (model.read_bytes(), dataset.read_bytes())
            args = ['--model', str(model), '--dataset', str(dataset), '--goals', '0', '10',
                    '--output', str(out)]
            with contextlib.redirect_stdout(io.StringIO()): self.assertEqual(main(args), 0)
            self.assertEqual(len(json.loads(out.read_text())['decisions']), 16)
            saved = out.read_bytes()
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): main(args)
            self.assertEqual(out.read_bytes(), saved)
            self.assertEqual(original, (model.read_bytes(), dataset.read_bytes()))

    def test_invalid_input_or_execute_flag_creates_no_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); a, d = fixture()
            model = root/'model.json'; dataset = root/'dataset.json'; out = root/'new'/'replay.json'
            model.write_text(json.dumps(a)); dataset.write_text(json.dumps(d))
            base = ['--model', str(model), '--dataset', str(dataset), '--output', str(out)]
            for extra in (['--goals', 'nan'], ['--goals', '10', '--execute']):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(base+extra)
                self.assertFalse(out.parent.exists())


if __name__ == '__main__':
    unittest.main()
