import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.manual_collection import ManualCollection, RecordingPort
from learning.supervised_collection import PROFILES
from learning.supervised_transport import SequencePort


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
    def __init__(self, url, token, boot_id, store, *, observer, clock, compact_reads=False, timeout=.8):
        self.observer, self.clock = observer, clock
        self.timeout = timeout
        self.may_have_run = self.writes_possible = False
        self.boot_id = boot_id
        self.last_sequence = 12
        self.executions = []
        self.fail_execute = self.fail_idle = False
        self.finish_count = 0
        Port.instances.append(self)
    def _call(self, path): return {'bootId': 'a'*32}
    def preflight(self): return {}
    def check_idle(self, *, allow_quiet_disconnect_retry=False, allow_quiet_monitor_timeout=False, allow_quiet_read_recovery=False):
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

    def test_after_armed_hook_forwarded_once_without_changing_pulse_caps(self):
        self.start();original=self.manager.port.execute;calls=[]
        def execute(pulse,tick,**kwargs):
            self.assertEqual(set(kwargs),{'after_armed_before_run'})
            kwargs['after_armed_before_run']()
            return original(pulse,tick)
        self.manager.port.execute=execute
        self.manager.pulse(dict(joint='wrist',direction='up',percent=20,durationMs=200),
                           after_armed_before_run=lambda:calls.append(True))
        self.assertEqual(calls,[True]);self.assertEqual(len(self.manager.port.executions),1)

    def test_after_armed_invalid_hook_rejected_before_session_or_hardware(self):
        with self.assertRaisesRegex(ValueError,'after_armed_before_run_callback_required'):
            self.manager.pulse({},after_armed_before_run=True)
        self.assertIsNone(self.manager.current)

    def test_after_armed_hook_keeps_60_and_120_recording_start_cutoffs(self):
        for seconds in (60,120):
            with self.subTest(seconds=seconds):
                manager=ManualCollection(self.root/f'prepared-{seconds}',self.helper,'http://127.0.0.1',
                    'secret','cam',journal_path=self.root/f'prepared-{seconds}.sqlite',clock=self.clock,
                    camera_factory=Camera,port_factory=Port,recording_seconds=seconds)
                try:
                    manager.start(dict(joint='gripper',poseContext='fixed',onsiteConfirmed=True))
                    port=manager.port
                    def execute(pulse,tick,**kwargs):return kwargs['after_armed_before_run']()
                    port.execute=execute
                    self.clock.now=manager.started+seconds-11
                    with self.assertRaisesRegex(ValueError,'recording_near_budget_end'):
                        manager.pulse(dict(joint='gripper',direction='open',percent=60,durationMs=650),
                            after_armed_before_run=lambda:self.clock.sleep(2))
                    self.assertEqual(port.executions,[])
                finally:manager.close()

    def fixed_pose_port(self,angle=195.55):
        from tests.test_supervised_transport import Clock,Controller,Store
        clock=Clock();device=Controller(clock);device.angle=angle;events=[]
        class FixedPosePort(RecordingPort):
            def _check_status(self,status,expected,pulse=None,baseline_angle=None):
                super()._check_status(status,expected,pulse,baseline_angle)
                if abs(self._shoulder_angle(status)-195.55)>.25:
                    raise ValueError('fixed_pose_changed')
        port=FixedPosePort('http://127.0.0.1','secret','boot',Store(),observer=events.append,clock=clock)
        port.opener=device
        return port,device

    def test_emergency_OFF_snapshot_separates_pose_failure_without_relaxing_idle(self):
        port,device=self.fixed_pose_port(195.29)
        with self.assertRaisesRegex(ValueError,'fixed_pose_changed'):port.preflight()
        with self.assertRaisesRegex(ValueError,'fixed_pose_changed'):port.check_idle()
        proof=port._snapshot(emergency=True)
        self.assertTrue(port.off_verified)
        self.assertEqual(proof['statusValidation'],dict(mode='emergency_output_OFF_only',executionScopeValidated=False))
        self.assertEqual(device.requests.count(('/sequence',{'action':'run'})),0)

    def test_active_pose_failure_still_raises_but_emergency_cleanup_proves_OFF(self):
        from tests.test_supervised_transport import pulse
        port,device=self.fixed_pose_port()
        device.after_run=lambda d:setattr(d,'angle',195.29)
        with self.assertRaisesRegex(ValueError,'fixed_pose_changed'):
            port.execute(pulse('gripper','open',60,650),lambda:None)
        self.assertTrue(port.off_verified);self.assertFalse(any(device.speeds.values()))
        self.assertEqual(sum(payload=={'action':'run'} for _,payload in device.requests),1)
        self.assertEqual(sum(payload=={'cmd':'stop'} for _,payload in device.requests),1)

    def test_emergency_OFF_still_rejects_status_output_boot_and_raw_driver_output(self):
        for mode in ('status','boot','driver'):
            with self.subTest(mode=mode):
                port,device=self.fixed_pose_port(195.29)
                if mode=='boot':device.boot='changed'
                else:
                    zero_status=device.status();device.append(1,30)
                    if mode=='driver':device.status=lambda:zero_status
                with self.assertRaises(ValueError):port._snapshot(emergency=True)
                self.assertFalse(port.off_verified)

    def test_finish_preserves_final_scope_failure_even_when_emergency_OFF_succeeds(self):
        self.start()
        def failed():raise ValueError('fixed_pose_changed')
        self.manager.port.check_idle=failed
        self.manager.port.finish=lambda:True
        snapshot=dict(statusValidation=dict(mode='emergency_output_OFF_only',executionScopeValidated=False),
                      actions=dict(bootId='a'*32,active=False),outputs=dict(latestSequence=17))
        self.manager.port.last_snapshot=snapshot;self.manager.port.last_off_at=self.clock.monotonic()
        session=self.manager.finish()
        self.assertEqual(session['phase'],'failed')
        self.assertEqual(session['error'],'final_controller_check_failed')
        self.assertTrue(session['finalIdleOffVerified'])
        raw=json.loads((self.manager.store.root/'session.json').read_bytes())
        self.assertEqual(raw['finalOutputOffProof'],dict(observedAt=self.clock.monotonic(),snapshot=snapshot))

    def test_finish_keeps_original_command_error_when_current_OFF_can_be_proven(self):
        self.start();self.manager.current['error']='command_failed_no_retry'
        def failed():raise ValueError('fixed_pose_changed')
        self.manager.port.check_idle=failed
        self.manager.port.finish=lambda:True
        session=self.manager.finish()
        self.assertEqual(session['phase'],'failed')
        self.assertEqual(session['error'],'command_failed_no_retry')
        self.assertTrue(session['finalIdleOffVerified'])

    def transport(self, fail_start=False):
        from tests.test_supervised_transport import CompactTransportTests
        template, device, _ = CompactTransportTests().setup_port()
        device.boot = 'c'*32
        # A retained baseline is discovered from evidence, never a fixed cursor.
        for entry in device.entries:
            entry['sequence'] += 730
        self.clock = self.manager.clock = device.clock
        requests = []
        original = template.opener.open
        self.transport_failed = fail_start
        def open_request(request, timeout):
            requests.append((request.get_method(), request.full_url, timeout))
            if self.transport_failed:
                raise TimeoutError('discovery_read_failed')
            return original(request, timeout)
        opener = type('Opener', (), {'open': staticmethod(open_request)})()
        def factory(*args, **kwargs):
            port = RecordingPort(*args, **kwargs)
            port.opener = opener
            return port
        def recovery_factory(*args, **kwargs):
            port = SequencePort(*args, **kwargs)
            port.opener = opener
            return port
        self.manager.port_factory = factory
        return device, requests, recovery_factory

    def failed_readonly_start(self):
        device, requests, factory = self.transport(fail_start=True)
        with self.assertRaisesRegex(TimeoutError, 'discovery_read_failed'):
            self.start()
        self.transport_failed = False
        return device, requests, factory

    def test_startup_reads_use_two_seconds_then_restore_normal_deadline(self):
        _, requests, _ = self.transport()
        self.start()
        self.assertTrue(all(1.99 <= timeout <= 2 for _, _, timeout in requests))
        self.assertEqual(self.manager.port.timeout, .8)
        self.assertEqual(self.manager.current['outputBaselineSequence'], 735)
        requests.clear()
        self.manager.port.check_idle()
        self.manager.finish()
        self.assertTrue(requests)
        self.assertTrue(all(0 < timeout <= .8 for _, _, timeout in requests))

    def test_failed_optional_predispatch_guard_never_executes_or_retries(self):
        self.start()
        calls = []
        def execute(pulse, tick, *, before_run):
            tick()
            before_run()
            calls.append(pulse)
        self.manager.port.execute = execute
        def reject():
            raise ValueError('new_observation_not_eligible')
        with self.assertRaisesRegex(ValueError, 'new_observation_not_eligible'):
            self.manager.pulse(dict(joint='wrist', direction='up', percent=20,
                                    durationMs=200), before_run=reject)
        self.assertEqual(calls, [])
        self.assertEqual(self.manager.status()['phase'], 'failed')
        self.assertTrue(self.manager.status()['finalIdleOffVerified'])
        with self.assertRaisesRegex(ValueError, 'recording_required'):
            self.pulse()

    def test_startup_get_failure_restores_cleanup_deadline(self):
        _, requests, _ = self.failed_readonly_start()
        self.assertAlmostEqual(requests[0][2], 2)
        self.assertTrue(all(0 < timeout <= .8 for _, _, timeout in requests[1:]))
        self.assertEqual(self.manager.port.timeout, .8)
        self.assertEqual(self.manager.status()['error'], 'session_start_failed')

    def test_readonly_recovery_preserves_failure_and_only_resolves_owned_block(self):
        device, requests, factory = self.failed_readonly_start()
        root = self.manager.store.root
        originals = {path.name: path.read_bytes() for path in root.iterdir() if path.is_file()}
        before = copy.deepcopy(self.manager.current)
        state = self.manager.status()
        legacy = self.manager.journal.records()
        result_json = self.manager.journal.connection.execute('SELECT result_json FROM supervised_blocks').fetchone()[0]
        requests.clear()
        with patch('learning.manual_collection.SequencePort', side_effect=factory):
            result = self.manager.recover_readonly()
        self.assertEqual(result, dict(sessionId=before['sessionId'], offVerified=True, noWrites=True))
        self.assertEqual(self.manager.current, before)
        self.assertEqual(self.manager.status(), state)
        self.assertEqual(self.manager.journal.records(), legacy)
        self.assertTrue(all(path.read_bytes() == originals[path.name] for path in root.iterdir() if path.is_file()))
        self.assertEqual(self.manager.journal.connection.execute('SELECT phase,result_json FROM supervised_blocks').fetchone(),
                         ('resolved_no_run_after_stop', result_json))
        self.assertTrue(all(method == 'GET' and 1.99 <= timeout <= 2 for method, _, timeout in requests))
        self.assertTrue(all(payload is None for _, payload in device.requests))
        proof = json.loads((root/'readonly-recovery'/'proof.json').read_text())
        self.assertEqual(proof['snapshot']['actions']['bootId'], 'c'*32)
        self.assertEqual(proof['snapshot']['outputs']['entries'][-1]['sequence'], 735)
        with self.assertRaisesRegex(ValueError, 'owned_failed_row'):
            self.manager.recover_readonly()
        self.start()
        self.assertEqual(self.manager.current['phase'], 'recording')

    def test_readonly_recovery_failed_read_consumes_attempt_and_keeps_lock(self):
        _, requests, factory = self.failed_readonly_start()
        self.transport_failed = True
        before = self.manager.status()
        with patch('learning.manual_collection.SequencePort', side_effect=factory):
            with self.assertRaises(TimeoutError): self.manager.recover_readonly()
            count = len(requests)
            with self.assertRaises(RuntimeError): self.manager.recover_readonly()
            self.assertEqual(len(requests), count)
        self.assertEqual(self.manager.status(), before)
        self.assertEqual(self.manager.journal.connection.execute('SELECT phase FROM supervised_blocks').fetchone()[0], 'off_unverified')
        with self.assertRaisesRegex(ValueError, 'interrupted_collection'): self.start()

    def test_readonly_recovery_rejects_active_run_write_camera_and_unknown(self):
        _, _, factory = self.failed_readonly_start()
        cases = [(self.manager, 'busy', True), (self.manager, 'command_count', 1),
                 (self.manager, 'camera', object()), (self.manager.port, 'may_have_run', True),
                 (self.manager.port, 'writes_possible', True)]
        with patch('learning.manual_collection.SequencePort', side_effect=factory) as recovery:
            for target, name, value in cases:
                with self.subTest(name=name), patch.object(target, name, value):
                    with self.assertRaisesRegex(ValueError, 'not_eligible'): self.manager.recover_readonly()
            for field, value in (('phase', 'recording'), ('error', 'command_failed_no_retry')):
                with self.subTest(field=field), patch.dict(self.manager.current, {field: value}):
                    with self.assertRaisesRegex(ValueError, 'not_eligible'): self.manager.recover_readonly()
            self.manager.current['events'].append(dict(kind='http', path='/sequence', payload={'action': 'run'}))
            with self.assertRaisesRegex(ValueError, 'write_or_camera'): self.manager.recover_readonly()
            self.manager.current['events'].pop()
            self.manager.store.write_json('http-999999-intent.json', dict(method='POST', payload={'action': 'clear'}))
            with self.assertRaisesRegex(ValueError, 'write_or_camera'): self.manager.recover_readonly()
            recovery.assert_not_called()
        self.assertFalse((self.manager.store.root/'readonly-recovery').exists())
        with self.assertRaisesRegex(ValueError, 'interrupted_collection'): self.start()

    def test_readonly_recovery_rejects_reloaded_manager_and_unready_legacy(self):
        _, _, factory = self.failed_readonly_start()
        other = self.make()
        with self.assertRaisesRegex(ValueError, 'not_eligible'): other.recover_readonly()
        with patch.object(self.manager.journal, 'readiness', return_value={'ready': False}), \
                patch('learning.manual_collection.SequencePort', side_effect=factory) as recovery:
            with self.assertRaisesRegex(ValueError, 'legacy_journal_unresolved'): self.manager.recover_readonly()
            recovery.assert_not_called()
        self.assertFalse((self.manager.store.root/'readonly-recovery').exists())

    def test_readonly_recovery_requires_fresh_off_and_empty_queue_without_cleanup(self):
        device, requests, factory = self.failed_readonly_start()
        device.queue.append({'joint': 'wrist'})
        original = self.manager.current.copy()
        with patch('learning.manual_collection.SequencePort', side_effect=factory):
            with self.assertRaisesRegex(ValueError, 'single_step_queue_changed'):
                self.manager.recover_readonly()
        self.assertEqual(self.manager.current, original)
        self.assertTrue(all(method == 'GET' for method, _, _ in requests))
        self.assertEqual(len(device.queue), 1)
        self.assertEqual(self.manager.journal.connection.execute('SELECT phase FROM supervised_blocks').fetchone()[0], 'off_unverified')
        self.assertFalse((self.manager.store.root/'readonly-recovery'/'proof.json').exists())
        with self.assertRaisesRegex(ValueError, 'interrupted_collection'): self.start()

    def reload_failed_start(self):
        from learning.action_journal import BoundedActionJournal
        device,requests,factory=self.failed_readonly_start()
        root=self.manager.store.root
        raw=(root/'session.json').read_bytes()
        sid=self.manager.current['sessionId']
        self.manager.close()
        self.manager=self.make()
        self.manager.journal=BoundedActionJournal(str(self.manager.journal_path),10000)
        requests.clear()
        return device,requests,factory,root,raw,sid,hashlib.sha256(raw).hexdigest()

    def test_reloaded_failed_start_get_recovery_preserves_original_result(self):
        device,requests,factory,root,raw,sid,digest=self.reload_failed_start()
        db=self.manager.journal.connection
        original_row=db.execute('SELECT result_json FROM supervised_blocks WHERE session_id=?',(sid,)).fetchone()[0]
        original_files={p.name:p.read_bytes() for p in root.iterdir() if p.is_file()}
        with patch('learning.manual_collection.SequencePort',side_effect=factory):
            result=self.manager.recover_failed_start_readonly(sid,digest)
        self.assertTrue(result['noWrites'])
        self.assertTrue(result.get('offVerified',result.get('currentOffVerified')))
        self.assertEqual((root/'session.json').read_bytes(),raw)
        self.assertTrue(all((root/name).read_bytes()==body for name,body in original_files.items()))
        self.assertEqual(db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',(sid,)).fetchone(),
                         ('resolved_no_run_after_stop',original_row))
        self.assertTrue(requests)
        self.assertTrue(all(method=='GET' for method,_,_ in requests))
        self.assertTrue(all(payload is None for _,payload in device.requests))
        proof=json.loads((root/'readonly-failed-start-recovery/proof.json').read_text())
        self.assertEqual(proof['originalSessionSha256'],digest)
        self.assertTrue(proof['noWrites'])
        self.assertIsNone(self.manager.current)
        count=len(requests)
        with self.assertRaises(Exception):self.manager.recover_failed_start_readonly(sid,digest)
        self.assertEqual(len(requests),count)

    def test_reloaded_failed_start_recovery_rejects_owner_source_and_original_POST(self):
        _,requests,factory,root,raw,sid,digest=self.reload_failed_start()
        with patch('learning.manual_collection.SequencePort',side_effect=factory) as transport:
            for name,value in [('busy',True),('current',{}),('port',object()),('store',object()),('camera',object())]:
                with self.subTest(owner=name),patch.object(self.manager,name,value):
                    with self.assertRaises(ValueError):self.manager.recover_failed_start_readonly(sid,digest)
            with self.assertRaises(ValueError):self.manager.recover_failed_start_readonly(sid,'0'*64)
            (root/'session.json').write_bytes(raw+b' ')
            with self.assertRaises(ValueError):self.manager.recover_failed_start_readonly(sid,digest)
            (root/'session.json').write_bytes(raw)
            (root/'http-999999-intent.json').write_text(json.dumps(dict(method='POST',payload=dict(action='run'))))
            with self.assertRaises(ValueError):self.manager.recover_failed_start_readonly(sid,digest)
            transport.assert_not_called()
        self.assertEqual(requests,[])
        self.assertFalse((root/'readonly-failed-start-recovery').exists())
        self.assertEqual(self.manager.journal.connection.execute('SELECT phase FROM supervised_blocks').fetchone()[0],
                         'off_unverified')

    def test_reloaded_failed_start_recovery_requires_empty_get_only_original(self):
        _,requests,factory,root,raw,sid,digest=self.reload_failed_start()
        db=self.manager.journal.connection
        original=json.loads(raw)
        changes=[lambda d:d.update(phase='recording'),lambda d:d.update(error='command_failed_no_retry'),
            lambda d:d.update(frames=[dict(frameIndex=0)]),
            lambda d:d['events'].append(dict(kind='command')),
            lambda d:d['events'].append(dict(kind='http',path='/sequence',payload=dict(action='run')))]
        with patch('learning.manual_collection.SequencePort',side_effect=factory) as transport:
            for change in changes:
                value=copy.deepcopy(original);change(value);changed=json.dumps(value).encode()
                (root/'session.json').write_bytes(changed)
                db.execute('UPDATE supervised_blocks SET result_json=? WHERE session_id=?',(changed.decode(),sid));db.commit()
                with self.subTest(change=change),self.assertRaises(ValueError):
                    self.manager.recover_failed_start_readonly(sid,hashlib.sha256(changed).hexdigest())
            (root/'session.json').write_bytes(raw)
            db.execute('UPDATE supervised_blocks SET result_json=? WHERE session_id=?',(json.dumps(original),sid));db.commit()
            (root/'native-camera').mkdir()
            with self.assertRaises(ValueError):self.manager.recover_failed_start_readonly(sid,digest)
            transport.assert_not_called()
        self.assertEqual(requests,[])
        self.assertFalse((root/'readonly-failed-start-recovery').exists())

    def test_reloaded_failed_start_recovery_requires_exact_DB_and_unique_legacy_ready(self):
        _,requests,factory,root,raw,sid,digest=self.reload_failed_start()
        db=self.manager.journal.connection
        row=db.execute('SELECT * FROM supervised_blocks WHERE session_id=?',(sid,)).fetchone()
        with patch('learning.manual_collection.SequencePort',side_effect=factory) as transport:
            for field,value in [('plan_hash','wrong'),('output_path',str(root/'foreign')),('result_json','{}')]:
                with self.subTest(field=field):
                    db.execute('UPDATE supervised_blocks SET '+field+'=? WHERE session_id=?',(value,sid));db.commit()
                    with self.assertRaises(ValueError):self.manager.recover_failed_start_readonly(sid,digest)
                    db.execute('DELETE FROM supervised_blocks WHERE session_id=?',(sid,))
                    db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,?)',row);db.commit()
            db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,?)',('other','x','x','pending',None));db.commit()
            with self.assertRaises(ValueError):self.manager.recover_failed_start_readonly(sid,digest)
            db.execute('DELETE FROM supervised_blocks WHERE session_id=?',('other',));db.commit()
            with patch.object(self.manager.journal,'readiness',return_value={'ready':False}):
                with self.assertRaises(ValueError):self.manager.recover_failed_start_readonly(sid,digest)
            transport.assert_not_called()
        self.assertEqual(requests,[])
        self.assertFalse((root/'readonly-failed-start-recovery').exists())

    def test_reloaded_failed_start_GET_timeout_consumes_attempt_without_resolving(self):
        _,requests,factory,root,raw,sid,digest=self.reload_failed_start()
        self.transport_failed=True
        db=self.manager.journal.connection
        original_row=db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',(sid,)).fetchone()
        with patch('learning.manual_collection.SequencePort',side_effect=factory):
            with self.assertRaises(TimeoutError):self.manager.recover_failed_start_readonly(sid,digest)
            count=len(requests)
            with self.assertRaises(Exception):self.manager.recover_failed_start_readonly(sid,digest)
            self.assertEqual(len(requests),count)
        self.assertEqual((root/'session.json').read_bytes(),raw)
        self.assertEqual(db.execute('SELECT phase,result_json FROM supervised_blocks WHERE session_id=?',(sid,)).fetchone(),
                         original_row)
        self.assertFalse((root/'readonly-failed-start-recovery/proof.json').exists())
        self.assertTrue(all(method=='GET' for method,_,_ in requests))

    def test_recording_tick_uses_shared_quiet_idle_policy_at_half_second_pace(self):
        self.start()
        polls=[]
        self.manager.port.check_idle=lambda **kwargs:polls.append((self.clock.monotonic(),kwargs))
        self.clock.sleep(.25);self.manager.tick()
        self.assertEqual(polls,[])
        self.clock.sleep(.25);self.manager.tick()
        self.clock.sleep(.25);self.manager.tick()
        self.clock.sleep(.25);self.manager.tick()
        self.assertEqual(len(polls),2)
        flags=dict(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True,
                   allow_quiet_read_recovery=True)
        self.assertTrue(all(kwargs==flags for _,kwargs in polls))
        self.assertGreaterEqual(polls[1][0]-polls[0][0],.5)

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

    def test_interrupted_paged_baseline_uses_completed_initial_drain(self):
        self.start()
        for index, (has_more, sequence) in enumerate(((True, 8), (False, 12)), 1):
            self.manager.store.write_json(f'http-{index:06d}-intent.json',
                dict(path=f'/action_outputs?after={0 if index == 1 else 8}&limit=8',
                     payload=None, startedAt=100.+index))
            self.manager.store.write_json(f'http-{index:06d}-response.json',
                dict(receivedAt=100.+index, parsed=dict(hasMore=has_more,
                     entries=[dict(sequence=sequence)])))
        replay = self.make().session(self.manager.current['sessionId'])
        self.assertEqual(replay['outputBaselineSequence'], 12)

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

    def test_four_fps_reduces_budget_and_default_eight_stays_compatible(self):
        manager = ManualCollection(self.root, self.helper, 'http://127.0.0.1', 'secret', 'camera',
                                   clock=self.clock, camera_factory=Camera, port_factory=Port, sample_fps=4,
                                   journal_path=self.root/'four-fps-journal.sqlite')
        try:
            manager.start(dict(joint='wrist', poseContext='fixed', onsiteConfirmed=True))
            camera = manager.camera.plan
            self.assertEqual(camera['sampleFps'], 4)
            self.assertEqual(camera['maxFrames'], 260)
            self.assertEqual(camera['maxBytes'], 260*(1920*1080*4+65536))
        finally:
            manager.close()
        for value in (True, 2, 30):
            with self.assertRaisesRegex(ValueError, 'sample_fps'):
                ManualCollection(self.root, self.helper, 'http://127.0.0.1', 'secret', 'camera', sample_fps=value)

    def extended_manager(self, sample_fps=8):
        return ManualCollection(self.root/'extended', self.helper, 'http://127.0.0.1', 'secret', 'camera',
            clock=self.clock, camera_factory=Camera, port_factory=Port, sample_fps=sample_fps,
            recording_seconds=120, journal_path=self.root/'extended-journal.sqlite')

    def test_recording_seconds_default_and_explicit120_have_consistent_camera_budgets(self):
        self.start()
        default=self.manager.camera.plan
        self.assertEqual(default['seconds'],60)
        self.assertEqual(default['maxFrames'],500)
        self.assertEqual(default['maxBytes'],500*(1920*1080*4+65536))
        self.manager.finish()
        for fps in (4,8):
            manager=self.extended_manager(fps)
            try:
                manager.start(dict(joint='gripper',poseContext='fixed',onsiteConfirmed=True))
                camera=manager.camera.plan
                self.assertEqual(camera['seconds'],120)
                self.assertEqual(camera['maxFrames'],fps*120+20)
                self.assertEqual(camera['maxBytes'],camera['maxFrames']*(1920*1080*4+65536))
                from learning.action_recorder import ActionRecorder
                # 실제 촬영 없이 기존 recorder 생성자의 예산 허용 계약도 검사한다.
                ActionRecorder(self.root/'not-started',self.helper,camera)
            finally:manager.close()

    def test_recording_seconds_invalid_values_reject_before_any_io(self):
        for value in (True,False,None,60.,120.,59,61,119,121,180,'120'):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'recording_seconds'):
                ManualCollection(self.root/'must-not-exist',self.root/'missing-helper',
                    'http://127.0.0.1','secret','camera',recording_seconds=value)
        self.assertFalse((self.root/'must-not-exist').exists())

    def test_extended_recording_reserves_full_disk_budget_before_camera_or_port(self):
        manager=self.extended_manager()
        try:
            needed=980*(1920*1080*4+65536)+2*1024**3
            prior_ports=len(Port.instances)
            with patch('learning.manual_collection.shutil.disk_usage',return_value=type('Disk',(),{'free':needed-1})()):
                with self.assertRaisesRegex(ValueError,'storage_reserve_required'):
                    manager.start(dict(joint='gripper',poseContext='fixed',onsiteConfirmed=True))
            self.assertIsNone(manager.camera)
            self.assertEqual(len(Port.instances),prior_ports)
        finally:manager.close()

    def test_extended_recording_tick_and_pulse_admission_keep_ten_and_five_second_reserves(self):
        manager=self.extended_manager()
        try:
            manager.start(dict(joint='gripper',poseContext='fixed',onsiteConfirmed=True))
            self.clock.now=manager.started+56
            manager.tick()
            self.assertEqual(manager.current['phase'],'recording')
            self.clock.now=manager.started+110
            manager.pulse(dict(joint='gripper',direction='close',percent=60,durationMs=200))
            self.assertEqual(len(manager.port.executions),1)
            self.clock.now=manager.started+115
            manager.tick()
            self.assertEqual(manager.current['phase'],'completed')
            self.assertEqual(len(manager.port.executions),1)
        finally:manager.close()

    def test_extended_recording_does_not_expand_motor_caps_or_retry_failed_commands(self):
        manager=self.extended_manager()
        try:
            manager.start(dict(joint='gripper',poseContext='fixed',onsiteConfirmed=True))
            for percent,duration in ((61,200),(60,651)):
                with self.subTest(percent=percent,duration=duration),self.assertRaisesRegex(ValueError,'joint_output_limit'):
                    manager.pulse(dict(joint='gripper',direction='close',percent=percent,durationMs=duration))
            self.assertEqual(manager.command_count,0)
            manager.port.fail_execute=True
            with self.assertRaisesRegex(ValueError,'unknown'):
                manager.pulse(dict(joint='gripper',direction='close',percent=60,durationMs=200))
            with self.assertRaisesRegex(ValueError,'recording_required'):
                manager.pulse(dict(joint='gripper',direction='close',percent=60,durationMs=200))
            self.assertEqual(len(manager.port.executions),1)
        finally:manager.close()

    def test_extended_recording_keeps_twenty_command_session_cap(self):
        manager=self.extended_manager()
        try:
            manager.start(dict(joint='gripper',poseContext='fixed',onsiteConfirmed=True))
            command=dict(joint='gripper',direction='close',percent=60,durationMs=200)
            for _ in range(20):manager.pulse(command)
            with self.assertRaisesRegex(ValueError,'session_command_budget_reached'):
                manager.pulse(command)
            self.assertEqual(manager.command_count,20)
            self.assertEqual(len(manager.port.executions),20)
        finally:manager.close()

    def test_extended_recording_rejects_new_pulse_after110_seconds_without_dispatch(self):
        manager=self.extended_manager()
        try:
            manager.start(dict(joint='gripper',poseContext='fixed',onsiteConfirmed=True))
            self.clock.now=manager.started+110.001
            with self.assertRaisesRegex(ValueError,'recording_near_budget_end'):
                manager.pulse(dict(joint='gripper',direction='close',percent=60,durationMs=200))
            self.assertEqual(manager.port.executions,[])
        finally:manager.close()

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
