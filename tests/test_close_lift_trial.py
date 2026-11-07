"""Synthetic stage-boundary failures; no physical manipulation claims."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning import close_lift_trial as link
from learning.model_grasp_trial import CONTACT_SCHEMA
from learning.supervised_collection import plan_digest
from tests.test_supervised_transport import Clock, Store


class ConnectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.clock = Clock(); self.clock.now = 20.
        self.store = Store(); self.calls = []
        self.port = SimpleNamespace(last_sequence=100, check_idle=lambda: None)
        close_scope = dict(schemaVersion=CONTACT_SCHEMA, maxCommands=1, maxSeconds=30.,
                           controllerBootId='a'*32, stages=[dict(kind='close')])
        self.close_result = dict(status='closed_off', finalIdleOffVerified=True,
                                 modelSelectedMotorCommands=1, errors=[], stages=[dict(steps=[
            dict(receipt=dict(offAt=10., offVerified=True, motor=1))])])
        self.close = SimpleNamespace(scope=close_scope, port=self.port, run=self.close_run)
        self.context = dict(supportMode='table_supported', object='same-object')
        lift_scope = dict(schemaVersion=link.m4.MODEL_SHORT_SCHEMA,
                          phase='model_selection', maxCommands=1, controllerBootId='a'*32,
                          maxObservedDeltaDeg=3., maxSeconds=30., context=self.context, hostClockId='lift-host')
        self.frame = dict(configurationSha256='b'*64, width=1920, height=1080,
                          videoMirrored=False, videoRotationAngle=0)
        binding = dict(external=self.frame, gripper=dict(width=320, height=240))
        self.lift_result = dict(status='completed', finalIdleOffVerified=True,
                                modelSelectedCommandExecuted=True)
        self.lift = SimpleNamespace(scope=lift_scope, model=SimpleNamespace(verify=lambda: True),
            registration=dict(cameraBinding=binding), port=SimpleNamespace(last_sequence=100),
            _before_run=lambda: None, run=self.lift_run)
        self.contact_scope = dict(controllerBootId='a'*32, context=self.context, hostClockId='contact-host')
        self.scope = dict(schemaVersion=link.SCHEMA, trialId='one-link', maxSeconds=120.,
            closeScopeSha256=plan_digest(close_scope), liftScopeSha256=plan_digest(lift_scope),
            contactScopeSha256=plan_digest(self.contact_scope))
        self.pairs = []
        self.slips = [[0., 0.], [0., 0.]]
        pixels = np.zeros((1080, 1920, 4), np.uint8)
        pixel_sha = link.m4.sha(pixels.tobytes())
        self.proof = dict(status='table_supported_contact_ready', method='onsite_dual_camera_review',
                          sameObjectVerified=True, depthVerified=True, clearanceVerified=True, observations=[])
        for index, at in enumerate((13., 14.2)):
            folder = self.root / str(index); folder.mkdir()
            record = dict(hostClockId='contact-host', context=copy.deepcopy(self.context), stable=True,
                          onsite={k: True for k in ('operatorPresent', 'clearanceVerified',
                                                   'jawContactVerified', 'tableSupportVerified')},
                          external=dict(root='external', ingressFile='host-ingress/0000.json'),
                          gripper=dict(root='gripper', label='before', imageSha256='c'*64))
            (folder/'external/host-ingress').mkdir(parents=True)
            (folder/'gripper').mkdir()
            cv2.imwrite(str(folder/'before-external.png'), pixels)
            self.save(folder/'external/host-ingress', '0000.json',
                      dict(metadataFile='frame.json', imageFile='frame.bin'))
            for name in ['external/frame.json', 'external/frame.bin',
                         'external/host-ingress/clock-before.json',
                         'gripper/esp32-before.jpg', 'gripper/esp32-before.json']:
                (folder/name).write_bytes(b'original-test-bytes')
            self.save(folder, 'review.json', record)
            record.update(evidenceFile='review.json', evidenceSha256=link.m4.sha((folder/'review.json').read_bytes()))
            self.proof['observations'].append(dict(root=str(index), record=record))
            self.pairs.append(dict(external=dict(self.frame, pixelSha256=pixel_sha, frameId=f'ext{index}', sourceEarliest=at,
                sourceLatest=at+.01, persistedAt=at+.05), gripper=dict(width=320, height=240,
                frameId=f'grip{index}', sourceEarliest=at, sourceLatest=at+.01, persistedAt=at+.05)))
        self.clock.now = 20.
        self.seal_proof()
        self.addCleanup(patch.stopall)
        patch.object(link.m4, '_external_frame', side_effect=self.external).start()
        patch.object(link.m4, '_gripper_frame', side_effect=self.gripper).start()
        patch.object(link.metrology, 'measure', side_effect=self.measure).start()
        patch.object(link.metrology, 'validate_before', return_value=[0, 0, 0]).start()
        # Camera-source verification itself is exercised by the existing real
        # provenance tests. Here we isolate failure ordering between stages.

    def save(self, root, name, value):
        (root/name).write_text(json.dumps(value))

    def seal_proof(self):
        declaration = {k:v for k,v in self.proof.items() if k not in ('evidenceFile', 'evidenceSha256')}
        self.save(self.root, 'contact.json', declaration)
        self.proof.update(evidenceFile='contact.json', evidenceSha256=link.m4.sha((self.root/'contact.json').read_bytes()))

    def external(self, root, source, scope): return self.pairs[int(root.name)]['external']
    def gripper(self, root, source, scope, label): return self.pairs[int(root.name)]['gripper']
    def measure(self, root, source, registration):
        dy, dx = self.slips[int(root.name)]
        return dict(state=[0, 0, dy], quality=dict(slipDxPx=dx))
    def close_run(self, **kwargs): self.calls.append('close'); return copy.deepcopy(self.close_result)
    def lift_run(self, **kwargs):
        self.lift._before_run(); self.calls.append('lift'); return copy.deepcopy(self.lift_result)
    def review(self, **kwargs): kwargs['tick'](); return self.proof
    def trial(self):
        # New schema is additive; old code must expose the new exact literal.
        return link.CloseLiftTrial(self.scope, self.close, self.lift, self.contact_scope,
                                   self.review, self.store, evidence_root=self.root, clock=self.clock)
    def run_trial(self): return self.trial().run(confirmed_scope_sha256=plan_digest(self.scope))

    def test_two_model_commands_separate_contact_from_airborne_retention(self):
        result = self.run_trial()
        self.assertEqual(self.calls, ['close', 'lift'])
        self.assertEqual(result['status'], 'lift_completed_pending_retention_review')
        self.assertTrue(result['tableContactReady']); self.assertFalse(result['liftVerified'])
        self.assertFalse(result['graspVerified']); self.assertEqual(result['modelSelectedMotorCommands'], 2)

    def test_failed_or_unknown_close_never_starts_contact_or_lift(self):
        self.close_result['status'] = 'failed'
        result = self.run_trial()
        self.assertEqual(self.calls, ['close']); self.assertEqual(result['status'], 'failed')

    def test_contact_failure_or_missing_depth_blocks_lift(self):
        self.proof['depthVerified'] = False; self.seal_proof()
        result = self.run_trial()
        self.assertEqual(self.calls, ['close']); self.assertEqual(result['status'], 'failed')

    def test_table_support_false_blocks_lift(self):
        record = self.proof['observations'][0]['record']; record['onsite']['tableSupportVerified'] = False
        self.seal_proof()
        self.assertEqual(self.run_trial()['status'], 'failed'); self.assertEqual(self.calls, ['close'])

    def test_old_or_same_frames_and_insufficient_span_block_lift(self):
        self.pairs[1]['gripper']['sourceEarliest'] = 13.5
        self.assertEqual(self.run_trial()['status'], 'failed'); self.assertEqual(self.calls, ['close'])

    def test_camera_reconfiguration_blocks_lift(self):
        self.pairs[1]['external']['videoMirrored'] = True
        self.assertEqual(self.run_trial()['status'], 'failed'); self.assertEqual(self.calls, ['close'])

    def test_contact_slip_across_two_otherwise_supported_frames_blocks_lift(self):
        self.slips = [[-3., 0.], [3., 0.]]
        self.assertEqual(self.run_trial()['status'], 'failed'); self.assertEqual(self.calls, ['close'])

    def test_metrology_input_domain_rejection_blocks_lift(self):
        with patch.object(link.metrology, 'validate_before', side_effect=ValueError('outside_domain')):
            self.assertEqual(self.run_trial()['status'], 'failed')
        self.assertEqual(self.calls, ['close'])

    def test_original_source_tamper_in_lift_preflight_blocks_run(self):
        def lift(**kwargs):
            (self.root/'0/external/frame.bin').write_bytes(b'tampered')
            self.lift._before_run(); self.calls.append('lift')
        self.lift.run = lift
        self.assertEqual(self.run_trial()['status'], 'failed'); self.assertEqual(self.calls, ['close'])

    def test_changed_review_file_blocks_lift(self):
        (self.root/'0/review.json').write_text('{}')
        self.assertEqual(self.run_trial()['status'], 'failed'); self.assertEqual(self.calls, ['close'])

    def test_intervening_output_during_lift_preflight_blocks_run(self):
        def lift(**kwargs): self.lift.port.last_sequence += 1; self.lift._before_run(); self.calls.append('lift')
        self.lift.run = lift
        self.assertEqual(self.run_trial()['status'], 'failed'); self.assertEqual(self.calls, ['close'])

    def test_unknown_lift_is_not_retried(self):
        def lift(**kwargs): self.lift._before_run(); self.calls.append('lift'); raise TimeoutError()
        self.lift.run = lift
        trial = self.trial(); result = trial.run(confirmed_scope_sha256=plan_digest(self.scope))
        self.assertEqual(result['status'], 'failed'); self.assertFalse(result['automaticRetry'])
        self.assertFalse(result['finalIdleOffVerified'])
        with self.assertRaises(ValueError): trial.run(confirmed_scope_sha256=plan_digest(self.scope))
        self.assertEqual(self.calls, ['close', 'lift'])

    def test_legacy_650_model_cannot_enter_connector(self):
        self.lift.scope['schemaVersion'] = link.m4.MODEL_SCHEMA
        self.scope['liftScopeSha256'] = plan_digest(self.lift.scope)
        with self.assertRaises(ValueError): self.trial()
        self.assertEqual(self.calls, [])

    def test_insufficient_overall_time_never_closes(self):
        self.scope['maxSeconds'] = 1.
        self.assertEqual(self.run_trial()['status'], 'failed'); self.assertEqual(self.calls, [])

    def test_guard_rechecks_contact_integrity_and_deadline(self):
        def lift(**kwargs):
            (self.root/'contact.json').write_text('{}')
            self.lift._before_run(); self.calls.append('lift')
        self.lift.run = lift
        self.assertEqual(self.run_trial()['status'], 'failed'); self.assertEqual(self.calls, ['close'])
