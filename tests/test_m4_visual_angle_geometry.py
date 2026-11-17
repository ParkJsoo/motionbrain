import copy
import unittest

import numpy as np

from learning import m4_visual_angle_geometry as geometry


class GeometryTests(unittest.TestCase):
    def rows(self):
        pivot = np.array([1600., 450.]); beta = 1.05
        first = np.array([1000., 600.]); rows = []
        for i, (before_angle, after_angle) in enumerate([(207.77, 200.39), (200.39, 194.1)]):
            after = geometry._rotation(beta*(after_angle-before_angle))@(first-pivot)+pivot
            rows.append(dict(sessionId=f's{i}', sourceSha256=str(i+1)*64,
                observationContractSha256='a'*64, command=copy.deepcopy(geometry.TRAINING_COMMAND),
                before=dict(virtualPointPx=first.tolist(), objectCenterPx=[1100., 760.], m4Deg=before_angle),
                after=dict(virtualPointPx=after.tolist(), objectCenterPx=[1100., 760.], m4Deg=after_angle),
                sourceEvidence={key: str(i+3)*64 for key in
                    ('beforeImageSha256', 'afterImageSha256', 'beforeSensorSha256', 'afterSensorSha256')}))
            first = after
        return rows

    def fit(self): return geometry.fit(self.rows(), audit_sources=lambda rows: rows)

    def current(self):
        return dict(self.rows()[1]['before'], observationContractSha256='a'*64, observationSha256='b'*64)

    def test_two_transition_geometry_reproduces_known_image_pivot_and_beta(self):
        model = self.fit()
        self.assertAlmostEqual(model['beta'], 1.05, places=6)
        np.testing.assert_allclose(model['pivotPx'], [1600., 450.], atol=1e-5)
        self.assertLess(model['fitDiagnostics']['maximumPointErrorPx'], 1e-5)
        self.assertFalse(model['calibratedUncertainty'])
        self.assertIsNone(model['uncertaintyInterval'])
        self.assertFalse(model['controlEligible'])
        self.assertIn('not_calibrated_physical_joint_center', model['pivotMeaning'])

    def test_current_image_point_drives_prediction_and_command_mean_is_reported(self):
        model = self.fit(); current = self.current(); current['virtualPointPx'][0] -= 50.
        prediction = geometry.predict(model, current, current['m4Deg']-4.)
        expected = geometry._rotation(1.05*-4.)@(np.array(current['virtualPointPx'])-[1600., 450.])+[1600., 450.]
        np.testing.assert_allclose(prediction['predictedVirtualPointPx'], expected, atol=1e-5)
        self.assertGreater(np.linalg.norm(np.array(prediction['predictedVirtualPointPx'])-prediction['commandMeanPointPx']), .5)
        self.assertIn('commandMeanErrorPx', prediction)
        self.assertFalse(prediction['support']['safetyBound'])

    def test_current_video_target_selects_angle_or_stop(self):
        current = self.current(); candidates = [current['m4Deg']-1., current['m4Deg']-4., current['m4Deg']-8.]
        action = geometry.select(self.fit(), current, candidates)
        self.assertEqual(action['selected']['kind'], 'target_angle')
        self.assertLess(action['selected']['errorPx'], action['currentDistancePx'])
        current['objectCenterPx'] = [800., 500.]
        stop = geometry.select(self.fit(), current, candidates)
        self.assertEqual(stop['selected']['kind'], 'stop')

    def test_after_sensor_is_fit_label_and_not_an_inference_input(self):
        model = self.fit(); current = self.current()
        self.assertNotIn('after', current)
        result = geometry.predict(model, current, current['m4Deg']-2.)
        self.assertEqual(result['deltaAngleDeg'], -2.)
        invalid = copy.deepcopy(current); invalid['observationContractSha256'] = 'c'*64
        with self.assertRaisesRegex(ValueError, 'current_raw_observation_binding'):
            geometry.predict(model, invalid, current['m4Deg']-2.)

    def test_UP_over_eight_or_outside_source_expansion_are_rejected(self):
        current = self.current(); model = self.fit()
        result = geometry.select(model, current, [current['m4Deg'], current['m4Deg']+1., current['m4Deg']-8.1])
        self.assertEqual(result['selected']['kind'], 'stop')
        self.assertEqual(len(result['rejected']), 2)
        current['m4Deg'] = model['sourceAngleRangeDeg'][0]-8.1
        with self.assertRaisesRegex(ValueError, 'current_angle_outside_local_support'):
            geometry.select(model, current, [current['m4Deg']-1.])

    def test_existing_boundary_margin_and_finite_three_command_budget_remain(self):
        model = self.fit(); current = self.current()
        exhausted = geometry.select(model, current, [current['m4Deg']-4.], commands_used=3)
        self.assertEqual(exhausted['selected']['kind'], 'stop')
        self.assertEqual(exhausted['selectionReason'], 'command_budget_exhausted')
        model['sourceAngleRangeDeg'] = [129., 131.]
        current['m4Deg'] = 130.
        with self.assertRaisesRegex(ValueError, 'target_angle_outside_local_support'):
            geometry.predict(model, current, 127.5)

    def test_reaudit_duplicate_or_non_DOWN_sources_are_refused(self):
        with self.assertRaisesRegex(ValueError, 'two_raw_audited_sources_required'):
            geometry.fit(self.rows()[:1], audit_sources=lambda rows: rows)
        with self.assertRaisesRegex(ValueError, 'raw_audit_changed'):
            geometry.fit(self.rows(), audit_sources=lambda rows: [])
        for kind in ['duplicate', 'wrong_policy', 'up', 'missing_sensor']:
            rows = self.rows()
            if kind == 'duplicate': rows[1]['sourceSha256'] = rows[0]['sourceSha256']
            elif kind == 'wrong_policy': rows[1]['command']['durationMs'] = 1500
            elif kind == 'up': rows[1]['after']['m4Deg'] = rows[1]['before']['m4Deg']+1.
            else: del rows[1]['sourceEvidence']['beforeSensorSha256']
            with self.subTest(kind=kind), self.assertRaises(ValueError): geometry.fit(rows, audit_sources=lambda rows: rows)

    def test_policy_tampering_or_mixed_observation_contract_is_rejected(self):
        model = self.fit(); model['supportPolicy']['maximumPlannedDecreaseDeg'] = 10.
        with self.assertRaisesRegex(ValueError, 'development_model_required'):
            geometry.predict(model, self.current(), self.current()['m4Deg']-2.)
        rows = self.rows(); rows[1]['observationContractSha256'] = 'c'*64
        with self.assertRaisesRegex(ValueError, 'observation_contract_changed'):
            geometry.fit(rows, audit_sources=lambda rows: rows)


    def test_default_objective_retains_existing_results_without_new_binding_fields(self):
        model = self.fit(); current = self.current(); target = current['m4Deg']-4.
        prediction = geometry.predict(model, current, target)
        self.assertEqual(prediction, geometry.predict(model, current, target, objective_point_px=None))
        decision = geometry.select(model, current, [target])
        self.assertEqual(decision, geometry.select(model, current, [target], objective_point_px=None))
        self.assertNotIn('objectiveBinding', prediction)
        self.assertNotIn('objectiveBinding', decision)
        self.assertNotIn('objectiveBindingSha256', decision['candidates'][0])
        observed_target = np.array(current['objectCenterPx'])
        self.assertAlmostEqual(prediction['errorPx'], np.linalg.norm(np.array(prediction['predictedVirtualPointPx'])-observed_target))

    def test_explicit_objective_changes_costs_without_rewriting_raw_observation_or_model(self):
        model = self.fit(); current = self.current(); original = copy.deepcopy(current)
        model_sha = geometry.digest(model); current_sha = geometry.digest(current)
        target = current['m4Deg']-4.; objective = [763., 657.]
        original_prediction = geometry.predict(model, current, target)
        prediction = geometry.predict(model, current, target, objective_point_px=objective)
        self.assertEqual(prediction['predictedVirtualPointPx'], original_prediction['predictedVirtualPointPx'])
        self.assertEqual(prediction['commandMeanPointPx'], original_prediction['commandMeanPointPx'])
        self.assertEqual(prediction['support'], original_prediction['support'])
        self.assertAlmostEqual(prediction['errorPx'], np.linalg.norm(np.array(prediction['predictedVirtualPointPx'])-objective))
        self.assertAlmostEqual(prediction['commandMeanErrorPx'], np.linalg.norm(np.array(prediction['commandMeanPointPx'])-objective))
        decision = geometry.select(model, current, [target], objective_point_px=objective)
        self.assertEqual(current, original); self.assertEqual(geometry.digest(model), model_sha)
        self.assertEqual(decision['inputSha256'], current_sha); self.assertEqual(decision['modelSha256'], model_sha)
        binding = decision['objectiveBinding']
        self.assertEqual(binding['pointPx'], objective)
        self.assertFalse(binding['learnedModelChanged'])
        self.assertIn('not_observed_object_center', binding['meaning'])
        self.assertEqual(decision['objectiveBindingSha256'], geometry.digest(binding))
        self.assertEqual(prediction['objectiveBindingSha256'], decision['objectiveBindingSha256'])
        for candidate in decision['candidates']:
            self.assertEqual(candidate['objectiveBinding'], binding)
            self.assertAlmostEqual(candidate['errorPx'], np.linalg.norm(np.array(candidate['predictedVirtualPointPx'])-objective))

    def test_explicit_goal_ranks_supported_angles_and_STOP_against_same_objective(self):
        model = self.fit(); current = self.current()
        angles = [current['m4Deg']-1., current['m4Deg']-4.]
        decisions = []
        for target in angles:
            goal = geometry.predict(model, current, target)['predictedVirtualPointPx']
            result = geometry.select(model, current, angles, objective_point_px=goal)
            self.assertEqual(result['selected']['targetAngleDeg'], target)
            self.assertAlmostEqual(result['selected']['errorPx'], 0.)
            decisions.append(result)
        self.assertNotEqual(decisions[0]['objectiveBindingSha256'], decisions[1]['objectiveBindingSha256'])
        self.assertEqual(decisions[0]['inputSha256'], decisions[1]['inputSha256'])
        stopped = geometry.select(model, current, angles, objective_point_px=current['virtualPointPx'])
        self.assertEqual(stopped['selected']['kind'], 'stop')
        self.assertEqual(stopped['currentDistancePx'], 0.)
        self.assertEqual(stopped['selected']['commandMeanErrorPx'], 0.)
        exhausted = geometry.select(model, current, angles, commands_used=3, objective_point_px=[763., 657.])
        self.assertEqual(exhausted['selected']['kind'], 'stop')
        self.assertEqual(exhausted['selectionReason'], 'command_budget_exhausted')
        self.assertEqual(exhausted['objectiveBinding'], exhausted['selected']['objectiveBinding'])

    def test_invalid_explicit_goal_cannot_be_swallowed_as_candidate_rejection(self):
        model = self.fit(); current = self.current(); target = current['m4Deg']-2.
        for goal in ([1.], [1., 2., 3.], (1., 2.), 'bad', [True, 10.], [float('nan'), 1.],
                     [1., float('inf')], [-1., 20.], [1920., 0.], [0., 1080.]):
            with self.subTest(goal=goal):
                with self.assertRaisesRegex(ValueError, 'image_point_required'):
                    geometry.predict(model, current, target, objective_point_px=goal)
                with self.assertRaisesRegex(ValueError, 'image_point_required'):
                    geometry.select(model, current, [target], objective_point_px=goal)
                with self.assertRaisesRegex(ValueError, 'image_point_required'):
                    geometry.select(model, current, [target], commands_used=3, objective_point_px=goal)
        invalid = copy.deepcopy(current); invalid['objectCenterPx'] = [float('nan'), 700.]
        with self.assertRaisesRegex(ValueError, 'image_point_required'):
            geometry.select(model, invalid, [target], objective_point_px=[763., 657.])

    def test_explicit_goal_never_relaxes_local_angle_support_and_is_independently_bound(self):
        model = self.fit(); current = self.current(); target = current['m4Deg']-2.
        goal = [763, 657]
        first = geometry.predict(model, current, target, objective_point_px=goal)
        second = geometry.predict(model, current, target, objective_point_px=[763., 657.])
        self.assertEqual(first['objectiveBindingSha256'], second['objectiveBindingSha256'])
        goal[0] = 1
        self.assertEqual(first['objectiveBinding']['pointPx'], [763., 657.])
        rejected = geometry.select(model, current, [current['m4Deg']+1., current['m4Deg']-8.1], objective_point_px=[763., 657.])
        self.assertEqual(rejected['selected']['kind'], 'stop'); self.assertEqual(len(rejected['rejected']), 2)
        current['m4Deg'] = model['sourceAngleRangeDeg'][0]-8.1
        with self.assertRaisesRegex(ValueError, 'current_angle_outside_local_support'):
            geometry.select(model, current, [current['m4Deg']-1.], objective_point_px=[763., 657.])


if __name__ == '__main__': unittest.main()
