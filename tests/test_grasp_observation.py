"""Fake HTTP and immutable-image checks; never contacts a real camera or motor."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from learning.artifacts import ArtifactStore
from learning import grasp_observation as m
from learning import relative_action_learning as dynamics
from learning.model_grasp_trial import FrozenModelBank, ModelGraspTrial, SCHEMA
from learning.supervised_collection import plan_digest


SCENE = dict(physicalView='actual-fixed-view', object='block', poseRegime='checked-pose', landmarks='object-jaws-v1')
BOOT = 'a'*32
COMMAND = dict(joint='gripper', direction='close', percent=40, durationMs=350)


class Clock:
    def __init__(self): self.now = 50.
    def monotonic(self): return self.now
    def sleep(self, value):
        assert value >= 0
        self.now += value


class Response:
    def __init__(self, body, headers, clock, status=200):
        self.body, self.headers, self.clock, self.status = body, headers, clock, status
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def read(self, maximum):
        self.clock.sleep(.01)
        return self.body[:maximum]


class Camera:
    def __init__(self, clock):
        self.clock, self.calls = clock, []
        _, jpeg = cv2.imencode('.jpg', np.zeros((24, 32, 3), np.uint8))
        self.body, self.boot, self.status = jpeg.tobytes(), BOOT, 200
        self.error, self.regress = None, False
    def open(self, request, timeout):
        self.calls.append((request.full_url, request.get_method(), timeout))
        if self.error: raise self.error
        self.clock.sleep(.01)
        captured = 1000 if self.regress else len(self.calls)*1000
        headers = {'X-MotionBrain-Camera-Boot-Id': self.boot,
                   'X-MotionBrain-Captured-Us': str(captured),
                   'X-MotionBrain-Capture-Requested-Us': str(captured-1),
                   'X-MotionBrain-Frame-Id': f'{self.boot}:{captured}',
                   'X-MotionBrain-Capture-Mode': 'after_request',
                   'X-MotionBrain-Clock-Domain': 'camera_boot_us',
                   'X-MotionBrain-Timestamp-Semantics': 'first_dma_buffer'}
        return Response(self.body, headers, self.clock, self.status)


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.store = ArtifactStore(self.root/'run')
        self.clock = Clock(); self.camera = Camera(self.clock)
        self.source = m.GripperObservationSource('http://camera.invalid', 'trial', BOOT,
                                                 SCENE, self.store, clock=self.clock, opener=self.camera)

    def review(self, record, *, deadline=None, tick=None):
        frame = record['frame']
        return dict(stable=True, annotation=dict(frameId=frame['frameId'], pixelSha256=frame['pixelSha256'],
            accepted=True, method='analyst_points', inputFrameOnly=True,
            points=dict(object=[16, 8], jawLeft=[8, 12], jawRight=[24, 12])))

    def observer(self, **kwargs):
        return m.ReviewedGraspObserver(self.source, self.review, clock=self.clock, **kwargs)

    def test_constructor_and_start_do_not_capture(self):
        self.assertEqual(self.camera.calls, [])
        self.source.start(deadline=60.)
        self.source.check()
        self.assertEqual(self.camera.calls, [])

    def test_single_capture_preserves_original_bytes_times_and_actual_context(self):
        self.source.start(deadline=60.)
        record = self.source.capture(deadline=60.)
        self.assertEqual(self.camera.calls[0][:2], ('http://camera.invalid/capture?fresh=1', 'GET'))
        self.assertLessEqual(self.camera.calls[0][2], 6.)
        self.assertEqual((self.store.root/'capture-0001/esp32-frame.jpg').read_bytes(), self.camera.body)
        self.assertEqual(record['sourceEarliest'], 50.)
        self.assertAlmostEqual(record['sourceLatest'], 50.02)
        self.assertEqual(record['hostClockId'], 'trial')
        self.assertEqual(record['context']['cameraIdentity']['cameraBootId'], BOOT)
        self.assertEqual(record['context']['imageShape'], [24, 32])
        self.source.verify(record)

    def test_failed_jpeg_is_preserved_and_not_retried(self):
        self.camera.body = b'not a jpeg'
        self.source.start(deadline=60.)
        with self.assertRaises(ValueError): self.source.capture(deadline=60.)
        self.assertEqual(len(self.camera.calls), 1)
        self.assertEqual((self.store.root/'capture-0001/esp32-frame.jpg').read_bytes(), b'not a jpeg')
        self.assertTrue((self.store.root/'capture-0001/capture-failure.json').exists())
        with self.assertRaisesRegex(ValueError, 'not_active'): self.source.capture(deadline=60.)
        self.assertEqual(len(self.camera.calls), 1)

    def test_timeout_is_a_single_request_with_failure_evidence(self):
        self.camera.error = TimeoutError('fake timeout')
        self.source.start(deadline=60.)
        with self.assertRaises(TimeoutError): self.source.capture(deadline=60.)
        failure = json.loads((self.store.root/'capture-0001/capture-failure.json').read_bytes())
        self.assertEqual(failure['errorType'], 'TimeoutError')
        self.assertEqual(len(self.camera.calls), 1)

    def test_reboot_and_repeated_source_are_rejected(self):
        self.source.start(deadline=60.)
        self.source.capture(deadline=60.)
        self.camera.regress = True
        with self.assertRaisesRegex(ValueError, 'reused_or_regressed'): self.source.capture(deadline=60.)
        self.assertEqual(len(self.camera.calls), 2)

    def test_other_camera_boot_is_not_relabelled(self):
        self.camera.boot = 'b'*32
        self.source.start(deadline=60.)
        with self.assertRaisesRegex(ValueError, 'boot_binding_changed'): self.source.capture(deadline=60.)
        metadata = json.loads((self.store.root/'capture-0001/esp32-frame.json').read_bytes())
        self.assertEqual(metadata['headers']['X-MotionBrain-Camera-Boot-Id'], 'b'*32)

    def test_cached_checks_do_not_capture_or_repeat_review(self):
        observer = self.observer(); observer.start(deadline=60.)
        observer.check()  # Cold startup is valid before the first reviewed frame.
        record = observer.observe({'requestedContext': 'must-not-be-used'}, after_off=None, deadline=60., tick=observer.check)
        self.assertEqual(record['context']['poseRegime'], SCENE['poseRegime'])
        calls = len(self.camera.calls)
        for _ in range(3): observer.latest({}); observer.check()
        self.assertEqual(len(self.camera.calls), calls)
        self.assertEqual(observer.review_count, 1)
        self.clock.sleep(3.)
        with self.assertRaisesRegex(ValueError, 'stale'): observer.latest({})

    def test_off_wait_ticks_without_refreshing_old_frame_timestamp(self):
        observer = self.observer(); observer.start(deadline=65.)
        first = observer.observe({}, after_off=None, deadline=65., tick=observer.check)
        count = [0]
        def tick(): count[0] += 1; observer.check()
        second = observer.observe({}, after_off=51., deadline=65., tick=tick)
        self.assertGreater(count[0], 10)
        self.assertGreaterEqual(second['sourceEarliest'], 54.)
        self.assertNotEqual(first['frame']['frameId'], second['frame']['frameId'])

    def test_point_review_cannot_change_context_or_frame_identity(self):
        def bad(record, **kwargs):
            reply = self.review(record); reply['context'] = {'poseRegime': 'copied-from-model'}
            return reply
        observer = m.ReviewedGraspObserver(self.source, bad, clock=self.clock); observer.start(deadline=60.)
        with self.assertRaisesRegex(ValueError, 'point_review_required'):
            observer.observe({}, after_off=None, deadline=60., tick=observer.check)

    def test_frame_hash_and_source_tampering_during_review_are_rejected(self):
        def bad(record, **kwargs):
            reply = self.review(record)
            (self.store.root/'capture-0001/esp32-frame.jpg').write_bytes(b'changed')
            return reply
        observer = m.ReviewedGraspObserver(self.source, bad, clock=self.clock); observer.start(deadline=60.)
        with self.assertRaisesRegex(ValueError, 'source_bytes_changed'):
            observer.observe({}, after_off=None, deadline=60., tick=observer.check)

    def test_slow_review_keeps_original_source_time_and_is_rejected(self):
        def slow(record, **kwargs):
            self.clock.sleep(3.)
            return self.review(record)
        observer = m.ReviewedGraspObserver(self.source, slow, clock=self.clock); observer.start(deadline=60.)
        with self.assertRaisesRegex(ValueError, 'stale'):
            observer.observe({}, after_off=None, deadline=60., tick=observer.check)
        metadata = json.loads((self.store.root/'capture-0001/esp32-frame.json').read_bytes())
        self.assertEqual(metadata['requestStartedAt'], 50.)

    def test_deadline_and_capture_budget_prevent_extra_capture(self):
        self.source.max_captures = 1
        self.source.start(deadline=60.)
        with self.assertRaisesRegex(ValueError, 'deadline'): self.source.capture(deadline=50.)
        self.assertEqual(self.camera.calls, [])
        self.source.capture(deadline=60.)
        with self.assertRaisesRegex(ValueError, 'budget'): self.source.capture(deadline=60.)
        self.assertEqual(len(self.camera.calls), 1)

    def test_absent_retention_reviewer_never_creates_held(self):
        observer = self.observer(); observer.start(deadline=60.)
        self.assertEqual(observer.verify_grasp({}, after_off=50., deadline=60., tick=observer.check), {'status': 'unknown'})
        self.assertEqual(self.camera.calls, [])

    def test_retention_review_preserves_actual_observations_and_declaration(self):
        def retention(records, **kwargs):
            self.assertEqual(len(records), 2)
            return dict(status='held', method='analyst_retention_review', contactVerified=True,
                        retentionVerified=True, supportDistinguished=True)
        observer = self.observer(retention_reviewer=retention); observer.start(deadline=65.)
        proof = observer.verify_grasp({}, after_off=50., deadline=65., tick=observer.check)
        self.assertEqual(proof['status'], 'held')
        first, second = proof['observations']
        self.assertGreaterEqual(second['sourceEarliest']-first['sourceLatest'], 1.)
        raw = (self.store.root/proof['evidenceFile']).read_bytes()
        self.assertEqual(m.digest(raw), proof['evidenceSha256'])
        self.assertEqual(json.loads(raw), {k: v for k, v in proof.items() if not k.startswith('evidence')})

    def test_fabricated_retention_flags_are_not_filled_by_adapter(self):
        observer = self.observer(retention_reviewer=lambda records, **kwargs: {'status': 'held'})
        observer.start(deadline=65.)
        with self.assertRaisesRegex(ValueError, 'explicit_retention_review'):
            observer.verify_grasp({}, after_off=50., deadline=65., tick=observer.check)

    def test_full_trial_connects_real_capture_adapter_and_frozen_model(self):
        self.source.start(deadline=100.)
        sample = self.source.capture(deadline=100.)
        # A new source object/new store is required for the real trial lifecycle.
        store = ArtifactStore(self.root/'trial')
        source = m.GripperObservationSource('http://camera.invalid', 'trial', BOOT, SCENE, store,
                                          clock=self.clock, opener=self.camera)
        context = sample['context']
        bindings = []
        for name, command in [('position', dict(joint='base', direction='left', percent=40, durationMs=350)),
                              ('close', COMMAND)]:
            rows = []
            for i, opening in enumerate((16., 18.)):
                before = [0., -4., opening]
                rows.append(dict(sessionId=f'{name}-{i}', captureGroup='train', context=context,
                    command=command, before=before, after=[0., -4., opening-3.],
                    episode=['boot', f'{name}-{i}'], frameIds=[f'{name}-{i}-pre', f'{name}-{i}-post']))
            raw = json.dumps(dynamics.fit(rows)).encode(); path = self.root/(name+'.json'); path.write_bytes(raw)
            bindings.append(dict(id=name, path=path, sha256=m.digest(raw)))
        bank = FrozenModelBank(bindings)
        class Port:
            boot_id = 'b'*32
            may_have_run = False
            closed = False
            def preflight(self): pass
            def check_idle(self): pass
            def execute(port, pulse, tick, *, before_run):
                tick(); self.clock.sleep(.2); before_run(); port.may_have_run = True
                before = self.clock.monotonic(); self.clock.sleep(.35); port.closed = True
                return dict(offVerified=True, requestedPulse=pulse, motor=pulse['motor'],
                    dispatchBefore=before, dispatchAfter=before+.01, offAt=self.clock.monotonic(),
                    outputTrace=[dict(motor=pulse['motor'], pwm=pulse['expectedPwm']), dict(motor=pulse['motor'], pwm=0)])
            def finish(self): return True
        port = Port()
        count = [0]
        def review(record, **kwargs):
            reply = self.review(record)
            count[0] += 1
            proof = dict(frameId=record['frame']['frameId'], pixelSha256=record['frame']['pixelSha256'],
                         method='onsite_observation', objectBetweenJawsVerified=True, depthVerified=True, clearanceVerified=True)
            ref = store.write_json(f'entry-review-{count[0]}.json', proof)
            reply['closeEntryEvidence'] = dict(proof, evidenceFile=ref['artifactRef'], evidenceSha256=ref['sha256'])
            return reply
        observer = m.ReviewedGraspObserver(source, review, clock=self.clock,
            retention_reviewer=lambda records, **kwargs: dict(status='held', method='analyst_retention_review',
                    contactVerified=True, retentionVerified=True, supportDistinguished=True))
        scope = dict(schemaVersion=SCHEMA, trialId='trial', hostClockId='trial', controllerBootId=port.boot_id,
            modelHashes=bank.hashes, maxCommands=2, maxRequestedMs=700, maxSeconds=30.,
            maxObservationAgeSeconds=2., maxDecisionAgeSeconds=.25, postOffSeconds=3., retentionSeconds=1.,
            stages=[dict(id='position', kind='position', modelId='position', target=[0., -4., 16.],
                         tolerancePx=[.1]*3, commands=[dict(joint='base', direction='left', percent=40, durationMs=350)], maxCommands=1),
                    dict(id='close', kind='close', modelId='close', target=[0., -4., 13.], tolerancePx=[.1]*3,
                         commands=[COMMAND], maxCommands=1)])
        trial = ModelGraspTrial(scope, bank, observer, port, store, evidence_root=store.root, clock=self.clock)
        report = trial.run(confirmed_scope_sha256=plan_digest(scope))
        self.assertEqual(report['status'], 'held', report['errors'])
        self.assertEqual(report['modelSelectedMotorCommands'], 1)
        self.assertFalse(report['fullyAutonomous'])
        self.assertEqual(source.count, 4)

    def test_intent_storage_failure_marks_source_failed_before_any_request(self):
        write = self.store.write_json
        def broken(name, value):
            if name.endswith('capture-intent.json'):
                raise OSError('fake disk failure')
            return write(name, value)
        self.store.write_json = broken
        self.source.start(deadline=60.)
        with self.assertRaises(OSError): self.source.capture(deadline=60.)
        self.assertTrue(self.source.failed)
        self.assertEqual(self.camera.calls, [])
        with self.assertRaisesRegex(ValueError, 'not_active'): self.source.capture(deadline=60.)

    def test_close_entry_declaration_is_saved_without_inventing_flags(self):
        def review(record, **kwargs):
            reply = self.review(record)
            reply['closeEntryEvidence'] = dict(frameId=record['frame']['frameId'],
                pixelSha256=record['frame']['pixelSha256'], method='onsite_observation',
                objectBetweenJawsVerified=True, depthVerified=True, clearanceVerified=True)
            return reply
        observer = m.ReviewedGraspObserver(self.source, review, clock=self.clock); observer.start(deadline=60.)
        record = observer.observe({}, after_off=None, deadline=60., tick=observer.check)
        proof = record['closeEntryEvidence']; raw = (self.store.root/proof['evidenceFile']).read_bytes()
        self.assertEqual(m.digest(raw), proof['evidenceSha256'])
        self.assertTrue(json.loads(raw)['depthVerified'])

    def test_incomplete_close_entry_declaration_is_not_completed(self):
        def review(record, **kwargs):
            reply = self.review(record)
            reply['closeEntryEvidence'] = dict(frameId=record['frame']['frameId'],
                pixelSha256=record['frame']['pixelSha256'], method='onsite_observation',
                objectBetweenJawsVerified=True, depthVerified=False, clearanceVerified=True)
            return reply
        observer = m.ReviewedGraspObserver(self.source, review, clock=self.clock); observer.start(deadline=60.)
        with self.assertRaisesRegex(ValueError, 'explicit_close_entry_review'):
            observer.observe({}, after_off=None, deadline=60., tick=observer.check)
        self.assertFalse(list(self.store.root.glob('close-entry-review-*.json')))

    def test_retention_callback_cannot_tamper_with_first_original_then_claim_held(self):
        def tampered(records, **kwargs):
            (self.store.root/'capture-0001/esp32-frame.jpg').write_bytes(b'changed original')
            return dict(status='held', method='analyst_retention_review', contactVerified=True,
                        retentionVerified=True, supportDistinguished=True)
        observer = self.observer(retention_reviewer=tampered); observer.start(deadline=65.)
        with self.assertRaisesRegex(ValueError, 'source_bytes_changed'):
            observer.verify_grasp({}, after_off=50., deadline=65., tick=observer.check)
        self.assertIsNotNone(observer.fault)
        self.assertFalse((self.store.root/'retention-review.json').exists())
        calls = len(self.camera.calls)
        with self.assertRaisesRegex(ValueError, 'not_active'):
            observer.observe({}, after_off=None, deadline=65., tick=observer.check)
        self.assertEqual(len(self.camera.calls), calls)
        self.assertEqual(observer.close()['status'], 'failed')

    def test_retention_callback_cannot_tamper_with_second_metadata(self):
        def tampered(records, **kwargs):
            path = self.store.root/'capture-0002/esp32-frame.json'
            metadata = json.loads(path.read_bytes()); metadata['persistedAt'] += .1
            path.write_text(json.dumps(metadata))
            return dict(status='held', method='analyst_retention_review', contactVerified=True,
                        retentionVerified=True, supportDistinguished=True)
        observer = self.observer(retention_reviewer=tampered); observer.start(deadline=65.)
        with self.assertRaisesRegex(ValueError, 'source_bytes_changed'):
            observer.verify_grasp({}, after_off=50., deadline=65., tick=observer.check)
        self.assertIsNotNone(observer.fault)
        self.assertEqual(observer.close()['status'], 'failed')

    def test_stale_check_and_latest_faults_cannot_be_refreshed_by_new_get(self):
        observer = self.observer(); observer.start(deadline=65.)
        observer.observe({}, after_off=None, deadline=65., tick=observer.check)
        self.clock.sleep(3.)
        calls = len(self.camera.calls)
        with self.assertRaisesRegex(ValueError, 'stale'): observer.check()
        self.assertIsNotNone(observer.fault)
        # Changing a threshold does not authorize recovery after a terminal fault.
        observer.max_age_seconds = 5.
        with self.assertRaisesRegex(ValueError, 'not_active'): observer.latest({})
        with self.assertRaisesRegex(ValueError, 'not_active'):
            observer.observe({}, after_off=None, deadline=65., tick=observer.check)
        self.assertEqual(len(self.camera.calls), calls)
        self.assertEqual(observer.close()['status'], 'failed')

    def test_source_check_failure_is_latched_even_if_source_recovers(self):
        observer = self.observer(); observer.start(deadline=65.)
        observer.observe({}, after_off=None, deadline=65., tick=observer.check)
        check = self.source.check
        def fail(): raise ValueError('source disappeared')
        self.source.check = fail
        with self.assertRaisesRegex(ValueError, 'source disappeared'): observer.check()
        self.source.check = check
        calls = len(self.camera.calls)
        with self.assertRaisesRegex(ValueError, 'not_active'):
            observer.observe({}, after_off=None, deadline=65., tick=observer.check)
        self.assertEqual(len(self.camera.calls), calls)
        self.assertEqual(observer.close()['status'], 'failed')

    def test_point_reviewer_receives_deadline_and_monitors_outputs_while_waiting(self):
        waiting = [False]
        def review(record, *, deadline, tick):
            self.assertEqual(deadline, 60.)
            waiting[0] = True
            self.clock.sleep(.1)
            tick()  # A real trial tick also checks quiet output history.
            return self.review(record)
        observer = m.ReviewedGraspObserver(self.source, review, clock=self.clock)
        observer.start(deadline=60.)
        def tick():
            observer.check()
            if waiting[0]: raise ValueError('unexpected_output_during_review')
        with self.assertRaisesRegex(ValueError, 'unexpected_output_during_review'):
            observer.observe({}, after_off=None, deadline=60., tick=tick)
        self.assertEqual(len(self.camera.calls), 1)
        self.assertIsNotNone(observer.fault)
        self.assertEqual(observer.close()['status'], 'failed')

    def test_retention_reviewer_uses_quiet_tick_during_wait(self):
        waiting = [False]
        def retention(records, *, deadline, tick):
            self.assertEqual(deadline, 65.)
            self.assertEqual(len(records), 2)
            waiting[0] = True
            tick()
            return dict(status='held', method='analyst_retention_review', contactVerified=True,
                        retentionVerified=True, supportDistinguished=True)
        observer = self.observer(retention_reviewer=retention); observer.start(deadline=65.)
        def tick():
            observer.check()
            if waiting[0]: raise ValueError('unexpected_output_during_retention_review')
        with self.assertRaisesRegex(ValueError, 'unexpected_output_during_retention_review'):
            observer.verify_grasp({}, after_off=50., deadline=65., tick=tick)
        self.assertEqual(len(self.camera.calls), 2)
        self.assertFalse((self.store.root/'retention-review.json').exists())
        self.assertEqual(observer.close()['status'], 'failed')

    def test_reviewer_deadline_expiry_is_terminal_without_another_capture(self):
        def review(record, *, deadline, tick):
            self.clock.now = deadline+.01
            tick()
            return self.review(record)
        observer = m.ReviewedGraspObserver(self.source, review, clock=self.clock); observer.start(deadline=60.)
        def tick():
            observer.check()
            if self.clock.monotonic() >= 60.: raise ValueError('trial_deadline')
        with self.assertRaisesRegex(ValueError, 'trial_deadline'):
            observer.observe({}, after_off=None, deadline=60., tick=tick)
        with self.assertRaisesRegex(ValueError, 'not_active'):
            observer.observe({}, after_off=None, deadline=70., tick=tick)
        self.assertEqual(len(self.camera.calls), 1)
        self.assertEqual(observer.close()['status'], 'failed')


if __name__ == '__main__': unittest.main()
