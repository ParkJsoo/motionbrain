import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.attended_collection_profile import ATTENDED_60_1000, collection_profile, validate_command, audit_profile
from learning.manual_collection import ManualCollection
from learning.supervised_transport import validate_pulse
from tests.test_manual_collection import Camera, Clock, Port


class AttendedProfileTests(unittest.TestCase):
    def test_all_supported_directions_keep_transport_validation(self):
        profile = collection_profile(ATTENDED_60_1000)
        for joint, directions in profile['directions'].items():
            for direction in directions:
                for percent, duration in [(60, 1000), (100, 1500)]:
                    command = dict(joint=joint, direction=direction, percent=percent, durationMs=duration)
                    pulse = validate_command(ATTENDED_60_1000, command)
                    self.assertEqual(validate_pulse(pulse), pulse)

    def test_invalid_output_and_M4_UP_are_rejected(self):
        base = dict(joint='gripper', direction='close', percent=70, durationMs=1000)
        for changes in [dict(percent=59), dict(percent=101), dict(percent=True),
                        dict(durationMs=999), dict(durationMs=1501), dict(durationMs=True),
                        dict(joint='shoulder', direction='up')]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_command(ATTENDED_60_1000, dict(base, **changes))

    def test_unknown_profile_rejected_before_any_device(self):
        with self.assertRaisesRegex(ValueError, 'unknown_attended_collection_profile'):
            ManualCollection('/unused', '/unused', 'http://127.0.0.1', '', 'cam', collection_profile='typo')

    def test_profile_returns_independent_metadata(self):
        p = collection_profile(ATTENDED_60_1000)
        p['directions']['shoulder'].append('up')
        self.assertEqual(collection_profile(ATTENDED_60_1000)['directions']['shoulder'], ['down'])

    def test_dataset_requires_matching_exact_plan_session_and_pulse(self):
        profile = collection_profile(ATTENDED_60_1000)
        plan = dict(collectionProfile=copy.deepcopy(profile)); session = copy.deepcopy(plan)
        pulse = validate_command(ATTENDED_60_1000, dict(joint='gripper', direction='close', percent=70, durationMs=1000))
        self.assertEqual(audit_profile(plan, session, pulse), profile)
        for altered_plan, altered_session, altered_pulse in [
                ({}, session, pulse), (plan, {}, pulse),
                (plan, session, dict(pulse, durationMs=999)),
                (plan, session, dict(pulse, expectedPwm=0))]:
            with self.subTest(altered_plan=altered_plan), self.assertRaises(ValueError):
                audit_profile(altered_plan, altered_session, altered_pulse)
        plan['collectionProfile']['maximumPercent'] = 101
        session = copy.deepcopy(plan)
        with self.assertRaises(ValueError): audit_profile(plan, session, pulse)

    def test_session_provenance_single_command_and_failure_no_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); helper = root/'helper'; helper.write_text('test')
            for fail in [False, True]:
                with self.subTest(fail=fail):
                    manager = ManualCollection(root/f'sessions-{fail}', helper, 'http://127.0.0.1', '', 'cam',
                        journal_path=root/f'journal-{fail}.sqlite', clock=Clock(), camera_factory=Camera,
                        port_factory=Port, collection_profile=ATTENDED_60_1000)
                    try:
                        with patch('learning.manual_collection.shutil.disk_usage', return_value=type('Disk', (), {'free': 20_000_000_000})()):
                            manager.start(dict(joint='gripper', poseContext='L-pose', onsiteConfirmed=True))
                        plan = json.loads((manager.store.root/'plan.json').read_text())
                        self.assertEqual(plan['collectionProfile'], collection_profile(ATTENDED_60_1000))
                        port = manager.port; port.fail_execute = fail
                        command = dict(joint='gripper', direction='close', percent=70, durationMs=1000)
                        if fail:
                            with self.assertRaisesRegex(ValueError, 'unknown'): manager.pulse(command)
                            with self.assertRaisesRegex(ValueError, 'recording_required'): manager.pulse(command)
                        else:
                            manager.pulse(command)
                            with self.assertRaisesRegex(ValueError, 'session_command_budget_reached'): manager.pulse(command)
                            manager.finish()
                        self.assertEqual(len(port.executions), 1)
                        session = json.loads((manager.store.root/'session.json').read_text())
                        self.assertEqual(session['collectionProfile'], plan['collectionProfile'])
                        self.assertEqual(session['finalIdleOffVerified'], True)
                    finally: manager.close()

    def test_changed_session_profile_rejected_before_execution(self):
        manager = ManualCollection('/unused', '/unused', 'http://127.0.0.1', '', 'cam',
                                   collection_profile=ATTENDED_60_1000)
        manager.current = dict(joint='gripper', collectionProfile=collection_profile(ATTENDED_60_1000))
        manager.current['collectionProfile']['maximumPercent'] = 101
        with self.assertRaisesRegex(ValueError, 'attended_collection_profile_changed'):
            manager._validate_pulse(dict(joint='gripper', direction='close', percent=70, durationMs=1000))

    def test_relative_training_frozen_load_and_selection_keep_explicit_profile(self):
        from learning import relative_action_learning as relative
        from learning.relative_model_bundle import validate_model
        command = dict(joint='gripper', direction='close', percent=70, durationMs=1000)
        context = dict(physicalView='current', object='block', poseRegime='L-pose', landmarks='analyst-3points',
                       cameraIdentity=dict(camera='external'), imageShape=[1080, 1920],
                       stateFields=relative.STATE.copy(), collectionProfile=collection_profile(ATTENDED_60_1000))
        rows = [dict(sessionId=f's{i}', captureGroup='training', context=copy.deepcopy(context),
                     command=copy.deepcopy(command), before=[10., 1., width], after=[10., 1., width-10.],
                     episode=['boot', f'command{i}'], frameIds=[f'before{i}', f'after{i}'])
                for i, width in enumerate([100., 110.])]
        with self.assertRaisesRegex(ValueError, 'joint_output_limit'): relative.command_key(command)
        model = validate_model(relative.fit(rows))
        result = relative.select(model, [10., 1., 100.], [10., 1., 90.], [command], context)
        self.assertEqual(result['selected']['kind'], 'action')
        self.assertEqual(result['selected']['command'], command)
        self.assertEqual(model['context']['collectionProfile'], collection_profile(ATTENDED_60_1000))
        bad = copy.deepcopy(model); del bad['context']['collectionProfile']
        with self.assertRaisesRegex(ValueError, 'joint_output_limit'): validate_model(bad)
        rows[1]['context']['collectionProfile']['minimumDurationMs'] = 999
        with self.assertRaisesRegex(ValueError, 'observation_context_changed'): relative.fit(rows)


if __name__ == '__main__': unittest.main()
