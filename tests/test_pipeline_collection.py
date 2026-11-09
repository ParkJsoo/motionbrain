"""실제 ManualCollection/SequencePort + 가짜 장치로 수집 수명주기를 검증한다."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from learning.pipeline_collection import collect_once
from learning.pipeline_observations import observe_transition
from tests import test_manual_collection as fixture
from tests.test_grasp_observation import Camera, BOOT, SCENE
from learning.grasp_observation import GripperObservationSource
from learning.manual_collection_dataset import sensor_window, POLICY
from learning.gripper_camera_dataset import bind_gripper_pair


class PipelineCollectionTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.ManualCollectionTests()
        self.fixture.setUp(); self.addCleanup(self.fixture.tearDown)
        self.device, self.requests, _ = self.fixture.transport()
        self.manager = self.fixture.manager
        self.root = self.fixture.root/'requests'
        self.command = dict(joint='gripper', direction='close', percent=40, durationMs=350)

    def collect(self, request='one', command=None, **kwargs):
        return collect_once(self.manager, request, self.command if command is None else command,
                            'fixed-current-view', onsite_confirmed=True, request_root=self.root, **kwargs)

    def runs(self):
        return [r for r in self.requests if r[0] == 'POST' and r[1].endswith('/sequence')]

    def test_raw_is_closed_before_processing_and_same_request_never_runs_twice(self):
        result = self.collect()
        self.assertEqual(result['status'], 'recorded'); self.assertTrue(result['offVerified'])
        source = Path(result['sessionPath'])/'session.json'; original = source.read_bytes()
        session = json.loads(original)
        receipt = next(e['receipt'] for e in session['events'] if e['kind'] == 'result')
        self.assertGreaterEqual(session['endedAt']-receipt['offAt'], 6.5)
        self.assertEqual(session['phase'], 'completed')
        # A bad observation configuration stays a later, independent error.
        rejected = observe_transition(source.parent, {}, {})
        self.assertEqual(rejected['status'], 'rejected')
        self.assertEqual(source.read_bytes(), original)
        count = len(self.requests)
        self.assertEqual(self.collect(), result)
        self.assertEqual(len(self.requests), count)

    def test_reserved_request_and_changed_payload_never_dispatch(self):
        self.root.mkdir()
        # A power interruption may leave the durable reservation, never permission to replay.
        first_start = self.manager.start
        def start(payload):
            replay = self.collect()
            self.assertEqual(replay['status'], 'reserved')
            return first_start(payload)
        self.manager.start = start
        self.collect()
        count = len(self.requests)
        with self.assertRaisesRegex(ValueError, 'payload_changed'):
            self.collect(command=dict(self.command, percent=30))
        self.assertEqual(len(self.requests), count)

    def test_approval_plan_is_bound_to_the_single_request(self):
        report = self.collect(approval_plan_sha256='a'*64)
        self.assertEqual(report['request']['approvalPlanSha256'], 'a'*64)
        count = len(self.requests)
        with self.assertRaisesRegex(ValueError, 'payload_changed'):
            self.collect(approval_plan_sha256='b'*64)
        self.assertEqual(len(self.requests), count)

    def test_caller_mutation_cannot_change_the_reserved_command(self):
        original_start = self.manager.start
        def start(payload):
            self.command['percent'] = 30
            return original_start(payload)
        self.manager.start = start
        report = self.collect()
        self.assertEqual(report['request']['command']['percent'], 40)
        session = json.loads((Path(report['sessionPath'])/'session.json').read_text())
        intent = next(e for e in session['events'] if e['kind'] == 'command')
        self.assertEqual(intent['pulse']['percent'], 40)

    def test_post_recording_failure_retains_source_and_never_retries(self):
        original_pulse = self.manager.pulse
        def pulse(*args, **kwargs):
            original_pulse(*args, **kwargs)
            raise ValueError('injected_after_output_failure')
        self.manager.pulse = pulse
        report = self.collect()
        self.assertEqual(report['status'], 'failed')
        self.assertTrue(report['offVerified'])
        self.assertTrue(Path(report['sessionPath']).is_dir())
        self.assertEqual(len(report['sourceSha256']), 64)
        count = len(self.requests)
        self.assertEqual(self.collect(), report); self.assertEqual(len(self.requests), count)

    def test_reservation_directory_is_synced_before_device_start(self):
        events = []
        start = self.manager.start
        def started(payload):
            events.append('start'); return start(payload)
        self.manager.start = started
        with patch('learning.pipeline_collection.sync_directory', side_effect=lambda path: events.append('sync')):
            self.collect()
        self.assertIn('sync', events[:events.index('start')])

    def paired_factory(self, *, before_error=False, after_error=False, delay=0.):
        cameras = []
        def factory(session_id, store):
            camera = Camera(self.manager.clock); cameras.append(camera)
            original = camera.open
            def open_camera(request, timeout):
                self.manager.clock.sleep(delay)
                if before_error or (after_error and camera.calls):
                    camera.error = TimeoutError('synthetic paired-camera failure')
                return original(request, timeout)
            camera.open = open_camera
            return GripperObservationSource('http://camera.invalid',session_id,BOOT,SCENE,store,
                clock=self.manager.clock,opener=camera,max_captures=2)
        return factory,cameras

    def test_paired_snapshots_keep_original_metadata_and_complete_raw_window(self):
        factory,cameras=self.paired_factory(delay=5.)
        report=self.collect(gripper_source_factory=factory)
        self.assertEqual(report['status'],'recorded')
        pair=report['gripperPair'];self.assertEqual(pair['status'],'recorded')
        before=json.loads((Path(pair['path'])/'esp32-before.json').read_text())
        after=json.loads((Path(pair['path'])/'esp32-after.json').read_text())
        session=json.loads((Path(report['sessionPath'])/'session.json').read_text())
        command=next(e for e in session['events'] if e['kind']=='command')
        receipt=next(e['receipt'] for e in session['events'] if e['kind']=='result')
        self.assertEqual(before['hostClockId'],session['sessionId'])
        self.assertLessEqual(before['persistedAt'],command['at']-.1)
        self.assertGreaterEqual(after['requestStartedAt'],receipt['offAt']+3.2)
        self.assertGreaterEqual(session['endedAt'],receipt['offAt']+6.5)
        self.assertGreaterEqual(session['endedAt'],after['persistedAt'])
        self.assertGreater(after['persistedAt'],receipt['offAt']+6.5)
        self.assertEqual(len(cameras[0].calls),2)
        count=len(self.requests);self.assertEqual(self.collect(gripper_source_factory=factory),report)
        self.assertEqual(len(cameras),1);self.assertEqual(len(self.requests),count)

    def test_paired_pre_failure_never_dispatches_and_preserves_source(self):
        factory,cameras=self.paired_factory(before_error=True)
        report=self.collect(gripper_source_factory=factory)
        self.assertEqual(report['status'],'failed');self.assertTrue(report['offVerified'])
        self.assertEqual(report['gripperPair']['status'],'rejected')
        session=json.loads((Path(report['sessionPath'])/'session.json').read_text())
        self.assertFalse(any(e['kind']=='command' for e in session['events']))
        self.assertEqual(len(cameras[0].calls),1)
        self.assertIn('sourceSha256',report)

    def test_paired_post_failure_does_not_fail_or_shorten_raw_recording(self):
        factory,cameras=self.paired_factory(after_error=True,delay=1.)
        report=self.collect(gripper_source_factory=factory)
        self.assertEqual(report['status'],'recorded');self.assertTrue(report['offVerified'])
        self.assertEqual(report['gripperPair']['status'],'rejected')
        self.assertTrue(report['gripperPair']['reason'].startswith('after:'))
        session=json.loads((Path(report['sessionPath'])/'session.json').read_text())
        self.assertEqual(session['phase'],'completed')
        receipt=next(e['receipt'] for e in session['events'] if e['kind']=='result')
        self.assertGreaterEqual(session['endedAt'],receipt['offAt']+6.5)
        self.assertEqual(len(cameras[0].calls),2)
        self.assertTrue((Path(report['sessionPath'])/'gripper-camera/capture-0002/capture-failure.json').is_file())

    def test_no_onsite_confirmation_never_touches_device(self):
        with self.assertRaisesRegex(ValueError, 'onsite'):
            collect_once(self.manager, 'one', self.command, 'fixed', onsite_confirmed=False,
                         request_root=self.root)
        self.assertEqual(self.requests, [])

    def test_terminal_camera_failure_is_not_reported_as_recorded(self):
        original = self.manager.finish
        def finish():
            self.manager.camera.failed = True
            return original()
        self.manager.finish = finish
        report = self.collect()
        self.assertEqual(report['status'], 'failed')
        self.assertTrue(report['offVerified'])
        self.assertIn('sourceSha256', report)

    def test_unverified_off_retains_unknown_output_and_duplicate_block(self):
        original = self.manager.finish
        def finish():
            self.manager.port.finish = lambda: False
            return original()
        self.manager.finish = finish
        report = self.collect()
        self.assertEqual(report['status'], 'unknown_output')
        self.assertFalse(report['offVerified'])
        count = len(self.requests)
        self.assertEqual(self.collect(), report)
        self.assertEqual(len(self.requests), count)

    def check_windows_and_pair(self,report):
        session=json.loads((Path(report['sessionPath'])/'session.json').read_text())
        command=next(e for e in session['events'] if e['kind']=='command')
        receipt=next(e['receipt'] for e in session['events'] if e['kind']=='result')
        statuses=[e for e in session['events'] if e.get('path') in ('/status','/collection_status')]
        before=sensor_window(statuses,[command['at']+x for x in POLICY['preSeconds']])
        after=sensor_window(statuses,[receipt['offAt']+x for x in POLICY['postSeconds']])
        self.assertGreaterEqual(len(before['samples']),3)
        self.assertGreaterEqual(len(after['samples']),3)
        pair=report['gripperPair'];pre=pair['before']
        self.assertLessEqual(pre['persistedAt'],command['at']-3.5)
        if pair['status']=='recorded':
            self.assertGreaterEqual(pair['after']['sourceEarliest'],receipt['offAt']+6.5)
            row=dict(sessionId=session['sessionId'],inputs={'image':{'frameId':'external-before'}},
                targets={'image':{'frameId':'external-after'}},evidence=dict(commandAt=command['at'],
                offReceiptAt=receipt['offAt'],cameraHostBounds=[self.manager.started,session['endedAt']],
                cameraIdentity={'viewId':'synthetic-external'}))
            bound=bind_gripper_pair(row,Path(pair['path']))
            self.assertEqual(bound['evidence']['cameraIdentity']['camera'],'gripper')
        self.assertEqual(session['phase'],'completed')
        self.assertGreaterEqual(session['endedAt'],receipt['offAt']+6.5)
        return len(before['samples']),len(after['samples'])

    def test_M4_slow_paired_cameras_preserve_both_fixed_sensor_windows(self):
        self.command=dict(joint='shoulder',direction='down',percent=25,durationMs=350)
        factory,cameras=self.paired_factory(delay=5.)
        report=self.collect(gripper_source_factory=factory)
        self.assertEqual(report['status'],'recorded');self.assertEqual(report['gripperPair']['status'],'recorded')
        counts=self.check_windows_and_pair(report)
        self.assertEqual(len(cameras[0].calls),2)
        self.assertEqual(counts, (4, 5))

    def test_M4_slow_post_failure_preserves_sensor_windows_and_raw_recording(self):
        self.command=dict(joint='shoulder',direction='down',percent=25,durationMs=350)
        factory,cameras=self.paired_factory(after_error=True,delay=5.)
        report=self.collect(gripper_source_factory=factory)
        self.assertEqual(report['status'],'recorded');self.assertEqual(report['gripperPair']['status'],'rejected')
        counts=self.check_windows_and_pair(report)
        self.assertEqual(len(cameras[0].calls),2)
        self.assertEqual(counts, (4, 5))


if __name__ == '__main__':
    unittest.main()
