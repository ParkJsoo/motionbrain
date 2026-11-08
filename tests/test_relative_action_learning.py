"""Synthetic dynamics checks; these do not establish grasp performance."""
import copy
import unittest
from unittest.mock import patch
from learning.artifacts import StorageError
from learning import manual_collection_dataset as dataset
from learning import relative_action_learning as m

C = dict(joint='shoulder', direction='down', percent=20, durationMs=350)
CONTEXT = {'physicalView': 'fixed', 'object': 'block', 'poseRegime': 'limited', 'landmarks': 'v1'}


def row(i, x, command=None, group='train'):
    return dict(sessionId=str(i), captureGroup=group, context=copy.deepcopy(CONTEXT),
                command=copy.deepcopy(command or C), before=[x, 2., 10.], after=[x-2*x, 2., 10.],
                episode=['boot', str(i)], frameIds=[f'{i}-pre', f'{i}-post'])


class RelativeTests(unittest.TestCase):
    def setUp(self):
        self.rows = [row(0, -4.), row(1, 4.), row(2, 0.)]
        self.model = m.fit(self.rows)

    def test_learned_state_changes_choice_and_beats_command_mean(self):
        test = [row(3, 2., group='eval')]
        report = m.evaluate(self.model, test)
        self.assertEqual(report['evaluated'], 1)
        self.assertLess(report['metrics']['stateRidge']['meanStateErrorPx'],
                        report['metrics']['commandMean']['meanStateErrorPx'])
        result = m.select(self.model, [2., 2., 10.], [-2., 2., 10.], [C], CONTEXT)
        self.assertEqual(result['selected']['kind'], 'action')
        self.assertFalse(result['controlEligible'])
        stop = m.select(self.model, [0., 2., 10.], [0., 2., 10.], [C], CONTEXT)
        self.assertEqual(stop['selected']['kind'], 'stop')

    def test_no_input_target_leakage_in_evaluation(self):
        test = [row(3, 2., group='eval')]
        first = m.evaluate(self.model, test)['predictions'][0]['stateRidge']
        test[0]['after'] = [1000., -1000., 12.]
        self.assertEqual(first, m.evaluate(self.model, test)['predictions'][0]['stateRidge'])

    def test_unsupported_state_command_and_context(self):
        for state, command in (([5., 2., 10.], C), ([0., 2., 10.], dict(C, percent=21))):
            r = m.select(self.model, state, [0., 0., 10.], [command], CONTEXT)
            self.assertEqual(r['status'], 'abstained')
            self.assertEqual(len(r['rejected']), 1)
        with self.assertRaisesRegex(ValueError, 'context_changed'):
            m.select(self.model, [0., 2., 10.], [0., 0., 10.], [C], dict(CONTEXT, physicalView='changed'))
        with self.assertRaisesRegex(ValueError, 'direction_not_enabled'):
            m.select(self.model, [0., 2., 10.], [0., 0., 10.], [dict(C, direction='up')], CONTEXT)

    def test_duplicates_overlap_singletons_and_finite_values(self):
        for change in ('sessionId', 'episode', 'frameIds'):
            bad = copy.deepcopy(self.rows); bad[1][change] = bad[0][change]
            with self.assertRaises(ValueError): m.fit(bad)
        with self.assertRaisesRegex(ValueError, 'overlap'): m.evaluate(self.model, [dict(self.rows[0], captureGroup='eval')])
        singleton = m.fit([self.rows[0]])
        self.assertEqual(singleton['models'], [])
        for state in ([float('nan'), 0., 2.], [0., 0., -1.], [True, 0., 2.]):
            with self.assertRaises(ValueError): m.select(self.model, state, [0., 0., 2.], [C], CONTEXT)
        with self.assertRaisesRegex(ValueError, 'duplicate_candidate'):
            m.select(self.model, [0., 2., 10.], [0., 0., 10.], [C, C], CONTEXT)

    def test_frame_bound_points_reject_ambiguous_and_target_assisted(self):
        frame = dict(frameId='f', pixelSha256='h', width=100, height=100)
        a = dict(frameId='f', pixelSha256='h', accepted=True, method='analyst_points', inputFrameOnly=True,
                 points=dict(object=[50, 20], jawLeft=[40, 40], jawRight=[60, 40]))
        self.assertEqual(m.point_state(a, frame), [0., -20., 20.])
        for patch in (dict(accepted=False), dict(inputFrameOnly=False), dict(pixelSha256='other')):
            with self.assertRaises(ValueError): m.point_state(dict(a, **patch), frame)
        a['points']['jawLeft'] = [float('inf'), 0.]
        with self.assertRaises(ValueError): m.point_state(a, frame)

    def test_group_overlap_and_empty_evaluation(self):
        with self.assertRaisesRegex(ValueError, 'group_overlap'):
            m.evaluate(self.model, [row(3, 2.)])
        self.assertEqual(m.evaluate(self.model, [])['status'], 'unavailable')

    def test_scene_registration_cannot_impersonate_legacy_contract(self):
        frame = dict(frameId='scene', pixelSha256='a'*64, width=100, height=100)
        annotation = dict(frameId='scene', pixelSha256='a'*64, accepted=True,
            method='registered_scene_template_points', inputFrameOnly=True,
            points=dict(object=[50, 20], jawLeft=[40, 40], jawRight=[60, 40]),
            registrationSha256='b'*64, parametersSha256='c'*64, referenceImageSha256='d'*64,
            landmarkContract='registered-scene-template-points-v1:'+'b'*64)
        self.assertEqual(m.point_state(annotation, frame), [0., -20., 20.])
        annotation['landmarkContract'] = 'registered-template-points-v1:'+'b'*64
        with self.assertRaisesRegex(ValueError, 'registered_annotation_contract_required'):
            m.point_state(annotation, frame)

    def test_visual_adapter_does_not_require_sensor_and_legacy_still_does(self):
        r = row(9, 2.)
        r['inputs'] = {'command': {'joint': 'gripper'}}
        r['evidence'] = {}
        with patch.object(dataset, '_visual_session', return_value=(r, [], [0, 1], [2, 3], [0, 4])):
            with patch.object(dataset, 'sensor_window', side_effect=AssertionError('sensor read')):
                self.assertEqual(dataset.build_visual_session('/tmp', 'hash'), r)
            with self.assertRaisesRegex(ValueError, 'session_plan_identity'):
                dataset.build_session('/tmp', 'hash')

    def test_missing_sources_are_preserved_as_rejections(self):
        item = dict(sessionRoot='missing', sessionSha256='hash', annotation={'sessionId': 'missing'})
        with patch.object(m, 'build_visual_session', side_effect=StorageError('missing source')):
            rows, rejected = m.load_manifest({'records': [item, item]}, '/tmp')
        self.assertEqual(rows, [])
        self.assertEqual(len(rejected), 2)

    def test_context_mixing_and_oos_evaluation_denominator(self):
        bad = copy.deepcopy(self.rows); bad[1]['context']['physicalView'] = 'new'
        with self.assertRaisesRegex(ValueError, 'context_changed'): m.fit(bad)
        report = m.evaluate(self.model, [row(3, 20., group='eval')])
        self.assertEqual((report['listed'], report['evaluated']), (1, 0))
        self.assertEqual(report['metrics'], {})
        self.assertEqual(len(report['rejected']), 1)


if __name__ == '__main__': unittest.main()
