import copy
import unittest

import numpy as np

from learning import conditioned_coarse_action_model as model


class ConditionedModelTests(unittest.TestCase):
    def rows(self):
        rows = []
        for index, command in enumerate(model.COMMANDS):
            for sample in range(2):
                before = [-200., 350.+sample*60., 228.+index, -20.+sample*5.]
                delta = ([10., 70.+sample*10., 0., 6.] if command['direction'] == 'up' else
                         [30., -80.-sample*10., -5. if command['joint'] == 'shoulder' else 0., -5.])
                rows.append(dict(sessionId=f's{index}-{sample}', sourceSha256=f'{index*2+sample+1:064x}',
                    command=copy.deepcopy(command), before=before, after=(np.array(before)+delta).tolist(),
                    observationContractSha256='a'*64,
                    sourceEvidence={key: str(sample+1)*64 for key in
                        ('beforeImageSha256', 'afterImageSha256', 'beforeSensorSha256', 'afterSensorSha256')}))
        return rows

    def fit(self): return model.fit(self.rows(), audit_sources=lambda rows: rows)

    def test_reaudit_is_required_and_mutated_reaudit_rejected(self):
        with self.assertRaisesRegex(ValueError, 'raw_audit_required'): model.fit(self.rows(), audit_sources=None)
        with self.assertRaisesRegex(ValueError, 'raw_audit_changed'): model.fit(self.rows(), audit_sources=lambda rows: [])

    def test_duplicate_sources_contract_changes_or_missing_sensor_proof_rejected(self):
        for mutation in ['source', 'session', 'contract', 'sensor']:
            rows = self.rows()
            if mutation == 'source': rows[1]['sourceSha256'] = rows[0]['sourceSha256']
            if mutation == 'session': rows[1]['sessionId'] = rows[0]['sessionId']
            if mutation == 'contract': rows[1]['observationContractSha256'] = 'b'*64
            if mutation == 'sensor': del rows[1]['sourceEvidence']['beforeSensorSha256']
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                model.fit(rows, audit_sources=lambda rows: rows)

    def test_local_extrapolation_is_separate_from_old_exact_minmax(self):
        frozen = self.fit()
        current = [-170., 330., 232., -25.]
        self.assertGreater(current[0], max(row['before'][0] for row in self.rows()))
        decision = model.select(frozen, current, [0., 0.], copy.deepcopy(model.COMMANDS), commands_used=0)
        self.assertEqual(decision['selected']['kind'], 'action')
        self.assertEqual(decision['selected']['command']['direction'], 'down')
        self.assertLess(decision['selected']['errorPx'], decision['currentDistancePx'])
        self.assertFalse(decision['safetyBound']); self.assertFalse(decision['controlEligible'])
        self.assertFalse(decision['calibratedUncertainty'])
        self.assertIn('commandMeanErrorPx', decision['selected'])
        self.assertTrue(decision['selected']['support']['developmentLocalExtrapolation'])
        self.assertFalse(decision['selected']['support']['safetyBound'])

    def test_far_sensor_heading_or_image_state_cannot_use_old_response(self):
        frozen = self.fit()
        for state in [[-200., 350., 242., -20.], [-200., 350., 207.77, -20.],
                      [-200., 350., 228., 10.], [1000., 350., 228., -20.]]:
            with self.subTest(state=state):
                decision = model.select(frozen, state, [0., 0.], model.COMMANDS, commands_used=0)
                self.assertEqual(decision['selected']['kind'], 'stop')
                self.assertTrue(all(r['reason'] == 'conditioned_state_outside_local_support' for r in decision['rejected']))

    def test_one_row_command_is_deficit_not_predictor(self):
        rows = self.rows()[:1]
        frozen = model.fit(rows, audit_sources=lambda rows: rows)
        self.assertEqual(frozen['models'], [])
        self.assertEqual(frozen['deficits'][0]['rows'], 1)
        with self.assertRaisesRegex(ValueError, 'two_transitions_required'):
            model.predict(frozen, rows[0]['before'], rows[0]['command'])

    def test_budget_current_near_and_predicted_near_stop(self):
        frozen = self.fit()
        budget = model.select(frozen, [-200., 350., 228., -20.], [0., 0.], model.COMMANDS, commands_used=3)
        self.assertEqual(budget['selectionReason'], 'command_budget_exhausted')
        self.assertEqual(budget['selected']['kind'], 'stop')
        near = model.select(frozen, [20., 30., 228., -20.], [0., 0.], model.COMMANDS, commands_used=0)
        self.assertEqual(near['selectionReason'], 'current_near_gripper_phase')
        command = copy.deepcopy(model.COMMANDS[1])
        rows = [copy.deepcopy(row) for row in self.rows() if row['command'] == command]
        for row in rows:
            row['before'][:2] = [0., 150.]; row['after'][:2] = [0., 30.]
        narrow = model.fit(rows, audit_sources=lambda rows: rows)
        decision = model.select(narrow, rows[0]['before'], [0., 0.], [command], commands_used=0)
        self.assertEqual(decision['selected']['kind'], 'stop')
        self.assertEqual(decision['rejected'][0]['reason'], 'predicted_near_gripper_phase_requires_separate_approach')

    def test_predicted_sixty_enters_near_phase_then_reobservation_stops_coarse(self):
        command = copy.deepcopy(model.COMMANDS[1])
        rows = [copy.deepcopy(row) for row in self.rows() if row['command'] == command]
        for row in rows:
            row['before'][:2] = [0., 150.]; row['after'][:2] = [0., 60.]
        frozen = model.fit(rows, audit_sources=lambda rows: rows)
        decision = model.select(frozen, rows[0]['before'], [0., 0.], [command], commands_used=0)
        self.assertEqual(decision['selected']['kind'], 'action')
        self.assertEqual(decision['selected']['predictedPhase'], 'near_gripper')
        self.assertAlmostEqual(decision['selected']['errorPx'], 60.)
        self.assertEqual(decision['minimumPredictedDistancePx'], 40.)
        after = decision['selected']['predictedState']
        next_decision = model.select(frozen, after, [0., 0.], [command], commands_used=1)
        self.assertEqual(next_decision['selected']['kind'], 'stop')
        self.assertEqual(next_decision['selectionReason'], 'current_near_gripper_phase')

    def test_heading_wrap_uses_shortest_angle_and_does_not_leak_after_state(self):
        rows = self.rows()[2:4]
        for row in rows:
            row['before'][3] = 179.; row['after'][3] = -179.
        frozen = model.fit(rows, audit_sources=lambda rows: rows)
        current = rows[0]['before'].copy(); current[3] = -179.
        prediction = model.predict(frozen, current, rows[0]['command'])
        self.assertAlmostEqual(prediction['support']['toolAxisDifferenceDeg'], 2.)
        self.assertLess(abs(prediction['predictedState'][3]+177.), 1.)
        self.assertEqual(len(prediction['predictedState']), 4)

    def test_policy_mutation_or_unsupported_candidate_rejected(self):
        frozen = self.fit(); frozen['supportPolicy']['maximumM4DifferenceDeg'] = 30.
        with self.assertRaisesRegex(ValueError, 'development_model_required'):
            model.predict(frozen, [-200., 350., 228., -20.], model.COMMANDS[0])
        for command in [dict(joint='shoulder', direction='up', percent=70, durationMs=1500),
                        dict(joint='elbow', direction='down', percent=70, durationMs=350)]:
            with self.assertRaisesRegex(ValueError, 'explicit_candidate_command'):
                model.select(self.fit(), [-200., 350., 228., -20.], [0., 0.], [command], commands_used=0)


if __name__ == '__main__': unittest.main()
