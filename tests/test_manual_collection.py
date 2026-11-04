import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.manual_collection import ManualCollection, RecordingPort
from learning.supervised_collection import PROFILES


class Clock:
    def __init__(self): self.now = 100.
    def monotonic(self): return self.now
    def sleep(self, n): self.now += n


class Camera:
    def __init__(self, root, helper, plan):
        self.root, self.plan = root, plan
        self.frames = []
        self.failed = False
    def start(self): self.root.mkdir()
    def wait_ready(self, timeout):
        self.frames = [dict(frameIndex=0, sourceEarliest=100., sourceLatest=100.001)]
    def check(self):
        if self.failed: raise ValueError('stale')
    def close(self):
        return dict(status='failed' if self.failed else 'partial', frames=copy.deepcopy(self.frames))


class Port:
    instances = []
    def __init__(self, url, token, boot_id, store, *, observer, clock):
        self.observer, self.clock = observer, clock
        self.boot_id = boot_id
        self.last_sequence = 12
        self.executions = []
        self.fail_execute = self.fail_idle = False
        self.finish_count = 0
        Port.instances.append(self)
    def _call(self, path): return {'bootId': 'a'*32}
    def preflight(self): return {}
    def check_idle(self, *, allow_quiet_disconnect_retry=False):
        if self.fail_idle: raise ValueError('idle_failure')
    def execute(self, pulse, tick):
        self.executions.append(pulse)
        tick()
        if self.fail_execute: raise ValueError('unknown')
        return dict(offVerified=True, outputTrace=[{'motor': pulse['motor'], 'pwm': pulse['expectedPwm']}])
    def finish(self):
        self.finish_count += 1
        return not self.fail_idle


class ManualCollectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.helper = self.root/'helper'; self.helper.write_text('test')
        self.clock = Clock()
        self.manager = self.make()
        self.disk = patch('learning.manual_collection.shutil.disk_usage', return_value=type('Disk', (), {'free': 20_000_000_000})())
        self.disk.start()
    def tearDown(self):
        self.manager.close()
        self.disk.stop()
        self.tmp.cleanup()
    def make(self):
        return ManualCollection(self.root/'sessions', self.helper, 'http://127.0.0.1', 'secret', 'cam',
                journal_path=self.root/'journal.sqlite', clock=self.clock, camera_factory=Camera, port_factory=Port)
    def start(self, joint='wrist'):
        return self.manager.start(dict(joint=joint, poseContext='fixed', onsiteConfirmed=True))
    def pulse(self, joint='wrist', direction='up'):
        return self.manager.pulse(dict(joint=joint, direction=direction, percent=20, durationMs=200))

    def test_all_joints_record_and_replay_optional_sensor(self):
        for joint, profile in PROFILES.items():
            with self.subTest(joint=joint):
                self.start(joint)
                self.pulse(joint, next(iter(profile['directions'])))
                report = self.manager.finish()
                self.assertEqual(report['phase'], 'completed')
                self.assertEqual(report['outputBaselineSequence'], 12)
                self.assertTrue(report['finalIdleOffVerified'])
                self.assertFalse(report['datasetEligible'])
                self.assertEqual(len(report['frames']), 1)
                sid = report['sessionId']
                disk_report = json.loads((self.manager.root/sid/'session.json').read_text())
                self.assertEqual(disk_report, report)
        self.assertEqual(len(self.manager.sessions()['sessions']), 5)

    def test_attended_confirmation_and_exact_payload(self):
        for payload in ({'joint':'wrist','poseContext':'fixed','onsiteConfirmed':False},
                        {'joint':'wrist','poseContext':'','onsiteConfirmed':True},
                        {'joint':'unknown','poseContext':'fixed','onsiteConfirmed':True}):
            with self.assertRaises(ValueError): self.manager.start(payload)
        self.assertFalse(self.manager.root.exists())

    def test_session_joint_and_limits_enforced_before_dispatch(self):
        self.start()
        for payload in (dict(joint='elbow',direction='up',percent=20,durationMs=200),
                        dict(joint='wrist',direction='up',percent=64,durationMs=200),
                        dict(joint='wrist',direction='up',percent=20,durationMs=901),
                        dict(joint='wrist',direction='up',percent=True,durationMs=200)):
            with self.assertRaises(ValueError): self.manager.pulse(payload)
        self.assertEqual(self.manager.port.executions, [])
        self.assertEqual(self.manager.status()['phase'], 'recording')

    def test_m4_up_and_second_pulse_rejected(self):
        self.start('shoulder')
        with self.assertRaises(ValueError): self.pulse('shoulder', 'up')
        self.pulse('shoulder', 'down')
        with self.assertRaises(ValueError): self.pulse('shoulder', 'down')
        self.assertEqual(len(self.manager.port.executions), 1)

    def test_busy_prevents_overlap_and_finish(self):
        self.start(); self.manager.busy = True
        for action in (self.pulse, self.manager.finish, self.manager.stop):
            with self.assertRaises(ValueError): action()
        self.manager.busy = False

    def test_unknown_command_is_preserved_never_replayed(self):
        self.start(); self.manager.port.fail_execute = True
        with self.assertRaises(ValueError): self.pulse()
        self.assertEqual(self.manager.status()['phase'], 'failed')
        self.assertTrue((self.manager.store.root/'command-001-intent.json').exists())
        with self.assertRaises(ValueError): self.pulse()
        self.assertEqual(len(self.manager.port.executions), 1)

    def test_failed_off_blocks_new_process_and_keeps_report(self):
        self.start(); self.manager.port.fail_idle = True
        report = self.manager.finish()
        self.assertFalse(report['finalIdleOffVerified'])
        self.manager.close(); self.manager = self.make()
        with self.assertRaisesRegex(ValueError, 'interrupted_collection'): self.start()
        self.assertEqual(self.manager.session(report['sessionId'])['phase'], 'failed')

    def test_interrupted_pending_row_blocks_and_remains_viewable(self):
        self.start(); sid = self.manager.current['sessionId']
        self.manager.journal.close(); self.manager.journal = None
        self.manager.current = None  # simulate a crashed process without cleanup
        self.manager = self.make()
        with self.assertRaisesRegex(ValueError, 'interrupted_collection'): self.start()
        report = self.manager.session(sid)
        self.assertEqual(report['phase'], 'interrupted')
        self.assertFalse(report['datasetEligible'])

    def test_camera_failure_stops_preserves_and_finishes_once(self):
        self.start(); port = self.manager.port
        self.manager.camera.failed = True; self.clock.now += 1
        self.manager.tick()
        self.assertEqual(self.manager.status()['phase'], 'failed')
        self.assertEqual(port.finish_count, 1)
        self.manager.finish()
        self.assertEqual(port.finish_count, 1)

    def test_budget_finishes_without_additional_command(self):
        self.start(); self.clock.now += 56
        self.manager.tick()
        self.assertEqual(self.manager.status()['phase'], 'completed')
        self.assertEqual(self.manager.port.executions, [])

    def test_operator_stop_is_partial_not_success(self):
        self.start()
        report = self.manager.stop()
        self.assertEqual(report['phase'], 'failed')
        self.assertEqual(report['error'], 'operator_stopped')

    def test_persist_failure_leaves_global_pending(self):
        self.start()
        with patch.object(self.manager.store, 'write_json', side_effect=OSError('disk full')):
            with self.assertRaises(OSError): self.manager.finish()
        row = self.manager.journal.connection.execute('SELECT phase FROM supervised_blocks').fetchone()
        self.assertEqual(row[0], 'pending')
        self.assertEqual(self.manager.status()['phase'], 'failed')
        self.assertEqual(self.manager.status()['error'], 'terminal_persistence_failed')
        self.manager.current = None

    def test_status_timeout_observer_keeps_original_error(self):
        self.start()
        port = RecordingPort('http://127.0.0.1', 'secret', 'a'*32, None,
                             observer=self.manager._event, clock=self.clock)
        with patch('learning.supervised_transport.SequencePort._call', side_effect=TimeoutError()):
            with self.assertRaises(TimeoutError): port._call('/status')
        self.assertEqual(self.manager.current['events'][-1]['kind'], 'http_error')

    def test_tick_does_not_swallow_process_interrupt(self):
        self.start(); self.clock.now += 1
        with patch.object(self.manager.port, 'check_idle', side_effect=KeyboardInterrupt()):
            with self.assertRaises(KeyboardInterrupt): self.manager.tick()

    def test_finish_closes_camera_and_reraises_process_interrupt(self):
        self.start()
        with patch.object(self.manager.port, 'check_idle', side_effect=KeyboardInterrupt()):
            with self.assertRaises(KeyboardInterrupt): self.manager.finish()
        self.assertEqual(self.manager.current['phase'], 'failed')
        self.assertIsNotNone(self.manager.current['recording'])

    def test_cleanup_evidence_failure_never_reports_completed(self):
        self.start()
        self.manager.port.evidence_failed = True
        result = self.manager.finish()
        self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(result['phase'], 'failed')
        self.assertEqual(result['error'], 'evidence_persistence_failed')

    def test_quiet_retry_is_visible_in_live_and_interrupted_replay(self):
        from tests.test_supervised_transport import Controller
        import http.client
        self.start(); events = []
        port = RecordingPort('http://127.0.0.1', 'secret', 'boot', self.manager.store,
                             observer=events.append, clock=self.clock)
        device = Controller(self.clock); port.opener = device
        port.preflight(); original=device.open; attempts=[]
        def disconnect(request, timeout):
            if request.full_url.endswith('/action_outputs'):
                attempts.append(timeout)
                if len(attempts)==1: raise http.client.RemoteDisconnected()
            return original(request, timeout)
        device.open=disconnect
        port.check_idle(allow_quiet_disconnect_retry=True)
        retries=[event for event in events if event['kind']=='http_retry']
        self.assertEqual([event['attempt'] for event in retries], [1, 2])
        self.assertTrue(all(event['path']=='/action_outputs' for event in retries))
        self.assertEqual(retries[0]['detail']['error'], 'RemoteDisconnected')
        sid=self.manager.current['sessionId']
        # View as an interrupted session, without altering any saved artifacts.
        other=self.make()
        replay=other.session(sid)
        self.assertEqual([event for event in replay['events'] if event['kind']=='http_retry'], retries)

    def test_path_traversal_rejected(self):
        with self.assertRaises(ValueError): self.manager.session('../outside')

    def test_http_observation_bracket_keeps_sensor_and_controller_clock_separate(self):
        observations = []
        with patch('learning.supervised_transport.SequencePort._call', return_value={'shoulderAngle':{'angleDeg':234., 'ageMs':20}}):
            port = RecordingPort('http://127.0.0.1', 'secret', 'a'*32, None,
                                 observer=observations.append, clock=self.clock)
            port._call('/status')
        self.assertEqual(observations[0]['requestStartedAt'], 100.)
        self.assertEqual(observations[0]['response']['shoulderAngle']['ageMs'], 20)
        self.assertNotIn('secret', json.dumps(observations))


if __name__ == '__main__': unittest.main()
