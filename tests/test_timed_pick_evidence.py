import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from learning.open_side_approach import LEGACY_SIDE_VIEW
from learning.timed_pick_evidence import CHANNELS, LABELS, TimedPickEvidence


class TimedPickEvidenceTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.path = Path(tmp.name)/'weights.json'
        cmd = dict(motor=1, kind='pulse', pwm=-80, percent=0, targetDegrees=0, durationMs=550)
        self.plan = dict(controllerBootId='c',gripperBootId='b', profiles={k:dict(command=cmd) for k in LABELS})
        self.sample = dict(label='gripper/close', command=cmd.copy(), sourceEpisode='s', nearCameraBoot='a',
                           pre=[0,1,119], delta=[1,-8,-11], startShoulderDeg=215.06, terminal='timed_out')
        self.artifact = dict(schema='motionbrain.field_near_response.v1', sourceView=LEGACY_SIDE_VIEW,
                             channels=CHANNELS, samples=[self.sample])
        self.features = dict(near=dict(state=[22,4,132], insideJaws=True, closeVisibility=True, margins=[60,18],jawGap=132),
            sideGeometry=dict(offset=[0,-90],overlap=False), planShoulderDeg=214.71,
            support={k:dict(allowed=True,catalogStartPoseInRange=False) for k in LABELS})

    def build(self):
        self.path.write_text(json.dumps(self.artifact))
        return TimedPickEvidence(self.plan,dict(artifact=str(self.path),sha256=hashlib.sha256(self.path.read_bytes()).hexdigest()))

    def test_reference_projection_never_becomes_a_wide_pulse_or_loaded_response(self):
        value = self.build().evaluate(self.features,None)
        close = value['candidates']['gripper/close']
        self.assertEqual(value['action'],'hold')
        self.assertEqual(close['nearIfMedianTransferred'], [23,-4,121])
        self.assertIn('outside_observed_near_start_box', close['reasons'])
        self.assertIn('close_outside_side_object', close['reasons'])
        self.assertIsNone(close['predictedWideSideDelta'])
        self.assertEqual(close['evidence']['terminalStates'], ['timed_out'])
        self.assertIn('completed_close_required',value['candidates']['elbow/up']['reasons'])
        self.assertIsNone(value['candidates']['elbow/up']['predictedObjectRise'])

    def test_unresolved_close_never_retries_or_allows_lift_in_zone(self):
        self.features['sideGeometry'] = dict(offset=[1,-10],overlap=False)
        vote = self.build().evaluate(self.features,{'executionState':'unresolved'})
        for label in ('elbow/down','shoulder/down'):
            self.assertIn('approach_stop_band',vote['candidates'][label]['reasons'])
        self.assertIn('unbound_close_receipt',vote['candidates']['gripper/close']['reasons'])
        self.assertIn('completed_close_required',vote['candidates']['elbow/up']['reasons'])

    def test_completed_receipt_without_same_session_post_off_pair_cannot_validate_lift(self):
        cmd = dict(self.sample['command'], bootId='c', commandId='id')
        action = dict(request=cmd, record=dict(motor=1, commandId='id', bootId='c',
            ordinal=1, state='completed', outputOff=True), outputLog=dict(entries=[
                dict(actionOrdinal=1,motor=1,pwm=-80),dict(actionOrdinal=1,motor=1,pwm=0)]))
        self.features['sideGeometry'] = dict(offset=[0,1], overlap=True)
        self.features['near'].update(margins=[0,0],jawGap=48.85)
        for session, off in [('other-session',10), ('current-session',30)]:
            with self.subTest(session=session):
                receipt = dict(executionState='known_terminal', session=session,
                               action=dict(action, outputOffHostBounds=dict(latest=off)))
                vote = self.build().evaluate(self.features,receipt)
                reasons = vote['candidates']['elbow/up']['reasons']
                self.assertIn('completed_close_required',reasons)
                self.assertIn('unbound_close_receipt',reasons)
                self.assertEqual(vote['action'],'hold')

    def test_other_pulse_or_duplicate_episode_rejected(self):
        self.sample['command']['durationMs'] = 500
        with self.assertRaisesRegex(ValueError,'invalid_timed_response_sample'):
            self.build()
        self.sample['command']['durationMs'] = 550
        self.artifact['samples'] *= 2
        with self.assertRaisesRegex(ValueError,'invalid_timed_response_sample'):
            self.build()
