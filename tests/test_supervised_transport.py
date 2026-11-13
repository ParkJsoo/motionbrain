import copy
import http.client
import json
import unittest
import urllib.parse

from learning.supervised_transport import SequencePort, validate_pulse, response_json


class Clock:
    def __init__(self): self.now = 10.
    def monotonic(self): return self.now
    def sleep(self, seconds): self.now += seconds


class Store:
    def __init__(self): self.rows = {};self.fail_run_intent = False;self.broken = False
    def write_json(self, name, value):
        if self.broken or (self.fail_run_intent and name.endswith('-intent.json')
                           and value.get('payload') == {'action': 'run'}):
            raise OSError('disk full')
        if name in self.rows: raise ValueError('overwrite')
        self.rows[name] = copy.deepcopy(value)


class Response:
    status = 200
    def __init__(self, payload): self.raw = json.dumps(payload).encode()
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def read(self, size): return self.raw[:size]


class RawResponse(Response):
    def __init__(self, raw): self.raw = raw


class Controller:
    def __init__(self, clock):
        self.clock = clock;self.requests = [];self.state = 'IDLE';self.queue = []
        self.entries = [];self.speeds = {i: 0 for i in range(1, 6)};self.boot = 'boot'
        self.running = False;self.default = 100;self.after_run = None;self.sensor_age = 20
        self.angle = 220.;self.fail_stop = False
        for i in range(1, 6): self.append(i, 0)
    def append(self, motor, pwm):
        self.entries.append({'sequence': self.entries[-1]['sequence'] + 1 if self.entries else 1,
            'atMs': int(self.clock.now * 1000), 'motor': motor, 'pwm': pwm,
            'reason': 'driver_write', 'actionOrdinal': 0})
        self.speeds[motor] = pwm
    def status(self):
        return {'state': self.state, 'motorEnabled': any(self.speeds.values()),
            'motors': {f'M{i}': {'speed': v, 'enabled': v != 0} for i, v in self.speeds.items()},
            'sensor': {'connected': True, 'imuOk': True, 'simulated': False,
                       'faultLatched': False, 'blocked': False, 'lastUpdateMs': self.sensor_age},
            'teleop': {'deadman': False, 'controlActive': False}, 'baseAngle': {'active': False},
            'shoulderAngle': {'active': False, 'available': True, 'sensorConnected': True,
                'sensorFresh': True, 'sensorReady': True, 'manualCalibrationUnlock': False,
                'manualGuardBlocked': False, 'ageMs': 10, 'angleDeg': self.angle,
                'manualDownBoundaryDeg': 123.58, 'manualUpBoundaryDeg': 300.12}}
    def open(self, request, timeout):
        self.clock.now += .01
        path = urllib.parse.urlsplit(request.full_url).path
        payload = None if request.data is None else {k: v[0] for k, v in
                urllib.parse.parse_qs(request.data.decode()).items()}
        self.requests.append((path, payload))
        if payload is None:
            if path == '/status': return Response(self.status())
            if path == '/actions': return Response({'bootId': self.boot, 'active': False})
            if path == '/action_outputs': return Response({'bootId': self.boot,
                'evidence': 'driver_write', 'clockDomain': 'controller_boot_millis', 'entries': self.entries[-128:]})
            if path == '/sequence': return Response({'sequence': {
                'state': 'RUNNING' if self.running else 'IDLE', 'totalCount': len(self.queue)}})
            raise AssertionError(path)
        if path == '/motor': self.default = int(payload['speed'])
        elif path == '/sequence':
            if payload['action'] == 'add': self.queue.append(payload)
            elif payload['action'] == 'clear': self.queue.clear();self.running = False
            elif payload['action'] == 'run':
                step = self.queue[0]
                from learning.supervised_transport import JOINTS
                motor, directions = JOINTS[step['joint']]
                self.append(motor, directions[step['direction']] * (255 * int(step['speed']) // 100))
                self.running = True
                if self.after_run: self.after_run(self)
        elif path == '/command':
            if payload['cmd'] == 'arm': self.state = 'ARMED'
            elif payload['cmd'] == 'stop':
                self.state = 'IDLE';self.running = False
                for i in range(1, 6): self.append(i, 0)
                if self.fail_stop: raise TimeoutError('lost STOP reply')
            # web_server.cpp: sendCommandResult adds the state summary, then
            # handleCommand appends the same root state a second time.
            state = json.dumps(self.state).encode()
            return RawResponse(b'{"success":true,"state":' + state + b',"state":' + state + b'}')
        return Response({'success': True})


def pulse(joint='wrist', direction='up', percent=60, ms=200):
    from learning.supervised_transport import JOINTS
    motor, directions = JOINTS[joint]
    return {'joint': joint, 'motor': motor, 'direction': direction, 'percent': percent,
            'durationMs': ms, 'expectedPwm': directions[direction] * (255 * percent // 100)}


class TransportTests(unittest.TestCase):
    def setup_port(self):
        clock = Clock();store = Store();device = Controller(clock)
        port = SequencePort('http://127.0.0.1:8181', 'private-token', 'boot', store, clock=clock)
        port.opener = device
        return port, device, store

    def test_private_return_goal_stops_once_after_safety_and_preserves_off(self):
        port,device,store=self.setup_port()
        device.after_run=lambda d:setattr(d,'angle',219.8)
        requested=validate_pulse(pulse('shoulder','down',35,550))
        result=port._execute_validated(requested,lambda:None,stop_when=lambda s:s['shoulderAngle']['angleDeg']<=219.9)
        self.assertTrue(result['stoppedEarly']);self.assertTrue(result['offVerified'])
        self.assertEqual(sum(p=={'action':'run'} for _,p in device.requests),1)
        self.assertFalse(any(device.speeds.values()));self.assertFalse(device.queue)
    def test_private_return_goal_cannot_skip_safety_or_retry_failure(self):
        for mode in ('unsafe','exception','not_bool'):
            port,device,_=self.setup_port();calls=[]
            def goal(status):
                calls.append(status)
                if mode=='exception':raise ValueError('goal_failed')
                return 1 if mode=='not_bool' else True
            if mode=='unsafe':device.after_run=lambda d:setattr(d,'angle',216.)
            with self.assertRaises(ValueError):port._execute_validated(validate_pulse(pulse('shoulder','down',35,550)),lambda:None,stop_when=goal)
            self.assertEqual(sum(p=={'action':'run'} for _,p in device.requests),1)
            self.assertFalse(any(device.speeds.values()));self.assertFalse(device.queue)
            if mode=='unsafe':self.assertEqual(calls,[])
        port,device,_=self.setup_port()
        with self.assertRaises(ValueError):port._execute_validated(pulse(),lambda:None,stop_when=True)
        self.assertFalse(device.requests)

    def test_preflight_readonly_and_stale_sensor_no_write(self):
        port, device, store = self.setup_port()
        port.preflight()
        self.assertTrue(all(payload is None for _, payload in device.requests))
        device.sensor_age = 151
        with self.assertRaisesRegex(ValueError, 'safety'): port.execute(pulse(), lambda: None)
        self.assertTrue(all(payload is None for _, payload in device.requests))
        self.assertNotIn('private-token', json.dumps(store.rows))

    def test_all_allowed_axis_directions_and_idle_reuse(self):
        port, device, _ = self.setup_port()
        combinations = [('gripper', 'open'), ('gripper', 'close'), ('wrist', 'up'),
            ('wrist', 'down'), ('elbow', 'up'), ('elbow', 'down'), ('shoulder', 'down'),
            ('base', 'left'), ('base', 'right')]
        for joint, direction in combinations:
            requested = pulse(joint, direction)
            result = port.execute(requested, lambda: None)
            self.assertEqual(result['requestedPulse'], requested)
            self.assertTrue(result['offVerified'])
            self.assertLess(result['dispatchBefore'], result['dispatchAfter'])
            self.assertLess(result['dispatchAfter'], result['offAt'])
            self.assertTrue(any(e['pwm'] == requested['expectedPwm'] for e in result['outputTrace']))
            self.assertEqual(device.default, 100)
            self.assertFalse(device.queue)
            port.check_idle()
        self.assertTrue(port.finish())
        count = len(device.requests)
        self.assertTrue(port.finish())
        self.assertEqual(len(device.requests), count)

    def test_m4_up_and_wrong_pwm_fail_before_device(self):
        port, device, _ = self.setup_port()
        with self.assertRaisesRegex(ValueError, 'm4_up'): port.execute(pulse('shoulder', 'up'), lambda: None)
        wrong = pulse();wrong['expectedPwm'] = -153
        with self.assertRaises(ValueError): port.execute(wrong, lambda: None)
        self.assertEqual(device.requests, [])

    def test_prearm_queue_disconnect_recovers_once_for_each_down_pulse(self):
        port, device, store = self.setup_port()
        original = device.open
        failed = set()
        timeouts = []
        def disconnect(request, timeout):
            if request.data is None and request.full_url.endswith('/sequence') and device.queue:
                timeouts.append(timeout)
                run_count = sum(p == {'action': 'run'} for _, p in device.requests)
                if run_count not in failed:
                    failed.add(run_count);device.clock.sleep(.29)
                    raise http.client.RemoteDisconnected()
            return original(request, timeout)
        device.open = disconnect
        for _ in range(2):
            result = port.execute(pulse(direction='down', percent=63, ms=550), lambda: None)
            self.assertTrue(result['offVerified'])
            device.clock.sleep(3);port.check_idle()
        self.assertEqual(len(failed), 2)
        self.assertEqual(sum(p == {'action': 'run'} for _, p in device.requests), 2)
        self.assertEqual(sum(n.endswith('-attempt-1-error.json') for n in store.rows), 2)
        self.assertEqual(sum(n.endswith('-attempt-2-intent.json') for n in store.rows), 2)
        self.assertAlmostEqual(timeouts[1], .51)
        self.assertAlmostEqual(timeouts[3], .51)
        self.assertFalse(device.queue);self.assertFalse(any(device.speeds.values()))

    def test_repeated_prearm_disconnect_stops_before_arm_and_run(self):
        port, device, store = self.setup_port();original = device.open;attempts = []
        def disconnect(request, timeout):
            if request.data is None and request.full_url.endswith('/sequence') and device.queue:
                attempts.append(timeout);raise http.client.RemoteDisconnected()
            return original(request, timeout)
        device.open = disconnect
        with self.assertRaises(http.client.RemoteDisconnected):
            port.execute(pulse(direction='down'), lambda: None)
        self.assertEqual(len(attempts), 2)
        self.assertFalse(any(p in ({'cmd': 'arm'}, {'action': 'run'}) for _, p in device.requests))
        self.assertFalse(device.queue);self.assertFalse(any(device.speeds.values()))
        self.assertTrue(port.finish())

    def test_disconnect_never_retries_writes_cleanup_or_unverified_reads(self):
        cases = [('/sequence', {'action': 'run'}, {'run': True}, True, False),
                 ('/command', {'cmd': 'arm'}, {}, True, False),
                 ('/command', {'cmd': 'stop'}, {'emergency': True}, True, False),
                 ('/sequence', None, {'emergency': True}, True, False),
                 ('/sequence', None, {}, False, False),
                 ('/sequence', None, {}, True, True),
                 ('/status', None, {}, True, False)]
        for path, payload, kwargs, off, ran in cases:
            with self.subTest(path=path, payload=payload, kwargs=kwargs, off=off, ran=ran):
                port, device, store = self.setup_port();attempts = []
                port.off_verified = off;port.may_have_run = ran
                def disconnect(request, timeout):
                    attempts.append(request);raise http.client.RemoteDisconnected()
                device.open = disconnect
                with self.assertRaises(http.client.RemoteDisconnected):
                    port._call(path, payload, **kwargs)
                self.assertEqual(len(attempts), 1)

    def test_queue_retry_does_not_extend_elapsed_budget(self):
        port, device, store = self.setup_port();port.off_verified = True;attempts = []
        def disconnect(request, timeout):
            attempts.append(timeout);device.clock.sleep(.81)
            raise http.client.RemoteDisconnected()
        device.open = disconnect
        with self.assertRaises(http.client.RemoteDisconnected): port._call('/sequence')
        self.assertEqual(len(attempts), 1)

    def test_retry_evidence_failure_blocks_second_request(self):
        port, device, store = self.setup_port();port.off_verified = True;attempts = []
        def disconnect(request, timeout):
            attempts.append(timeout);store.broken = True
            raise http.client.RemoteDisconnected()
        device.open = disconnect
        with self.assertRaises(OSError): port._call('/sequence')
        self.assertEqual(len(attempts), 1);self.assertTrue(port.evidence_failed)

    def test_queue_invalid_json_and_timeout_are_not_retried(self):
        for error in (ValueError('malformed'), TimeoutError('deadline')):
            port, device, store = self.setup_port();port.off_verified = True;attempts = []
            def fail(request, timeout):
                attempts.append(timeout)
                if isinstance(error, ValueError): return RawResponse(b'{bad')
                raise error
            device.open = fail
            with self.assertRaises(type(error)): port._call('/sequence')
            self.assertEqual(len(attempts), 1)

    def test_opted_in_quiet_disconnect_recovers_once_with_prior_off(self):
        for path in ('/status', '/action_outputs'):
            with self.subTest(path=path):
                port, device, store = self.setup_port();port.preflight()
                # Previous RUN history is not erased to enable quiet recovery.
                port.may_have_run = True
                original = device.open;attempts = []
                def disconnect(request, timeout):
                    if request.full_url.endswith(path):
                        attempts.append(timeout)
                        if len(attempts) == 1:
                            device.clock.sleep(.29)
                            raise http.client.RemoteDisconnected()
                    return original(request, timeout)
                device.open = disconnect
                port.check_idle(allow_quiet_disconnect_retry=True)
                self.assertTrue(port.off_verified);self.assertTrue(port.may_have_run)
                self.assertFalse(port._quiet_retry_off_proof)
                self.assertEqual(len(attempts), 2)
                self.assertAlmostEqual(attempts[1], .51)
                self.assertEqual(sum(n.endswith('-attempt-1-error.json') for n in store.rows), 1)
                self.assertEqual(sum(n.endswith('-attempt-2-intent.json') for n in store.rows), 1)
                self.assertTrue(all(payload is None for _, payload in device.requests))

    def test_extended_quiet_status_and_outputs_without_timeout_retry(self):
        port, device, store = self.setup_port(); port.preflight(); original=device.open; seen=[]
        def inspect(request, timeout):
            path=urllib.parse.urlsplit(request.full_url).path;seen.append((path,timeout))
            if path == '/status': device.clock.sleep(.9)
            return original(request,timeout)
        device.open=inspect
        port.check_idle(allow_quiet_disconnect_retry=True, allow_quiet_monitor_timeout=True)
        self.assertEqual(seen[0],('/status',1.5))
        self.assertTrue(any(v.get('path')=='/status' and v.get('timeoutSeconds')==1.5 for v in store.rows.values()))
        self.assertTrue(all(t <= 1.5 for p,t in seen))
        self.assertTrue(any(p == '/action_outputs' and t == 1.5 for p,t in seen))
        self.assertTrue(port.off_verified)
        seen.clear();port.check_idle();self.assertEqual(seen[0],('/status',.8))
        for error in (TimeoutError(), http.client.RemoteDisconnected()):
            port, device, _ = self.setup_port();port.preflight(); attempts=[]
            def fail(request, timeout):
                attempts.append(timeout);raise error
            device.open=fail
            with self.assertRaises(type(error)):
                port.check_idle(allow_quiet_disconnect_retry=True, allow_quiet_monitor_timeout=True)
            self.assertEqual(attempts,[1.5]);self.assertFalse(port.off_verified)

    def test_extended_quiet_scope_and_validation_remain_strict(self):
        for off, active, broken in ((False,None,False),(True,pulse(),False),(True,None,True)):
            port, device, _=self.setup_port();port.preflight();port.off_verified=off
            port._active_pulse=active;port.evidence_failed=broken;timeouts=[]
            def fail(request, timeout):
                timeouts.append(timeout);raise TimeoutError()
            device.open=fail
            with self.assertRaises((TimeoutError,OSError)):
                port.check_idle(allow_quiet_disconnect_retry=True, allow_quiet_monitor_timeout=True)
            self.assertEqual(timeouts,[.8])
        port, device, _=self.setup_port();port.preflight()
        with self.assertRaises(ValueError): port.check_idle(allow_quiet_monitor_timeout=True)
        for path,payload,kwargs in (('/status',None,{}),('/sequence',None,{}),
                ('/status',None,{'emergency':True}),('/command',{'cmd':'stop'},{})):
            with self.subTest(path=path,kwargs=kwargs):
                port._quiet_retry_off_proof = path != '/status' or bool(kwargs)
                with self.assertRaises(ValueError): port._call(path,payload,quiet_monitor_timeout=True,**kwargs)
        for changed in ('stale','nonzero'):
            port, device, _=self.setup_port();port.preflight()
            if changed=='stale': device.sensor_age=151
            else: device.append(1,10)
            with self.assertRaises(ValueError):
                port.check_idle(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True)
            self.assertFalse(port.off_verified)

    def test_quiet_retry_requires_explicit_monitor_prior_off_and_no_active_pulse(self):
        for optin, off, active, broken in ((False, True, None, False),
                (True, False, None, False), (True, True, pulse(), False),
                (True, True, None, True)):
            with self.subTest(optin=optin, off=off, active=active, broken=broken):
                port, device, _ = self.setup_port();port.preflight();attempts = []
                port.off_verified=off;port._active_pulse=active;port.evidence_failed=broken
                def disconnect(request, timeout):
                    attempts.append(request);raise http.client.RemoteDisconnected()
                device.open = disconnect
                with self.assertRaises(http.client.RemoteDisconnected):
                    port.check_idle(allow_quiet_disconnect_retry=optin)
                self.assertEqual(len(attempts), 1)
                self.assertFalse(port.off_verified);self.assertFalse(port._quiet_retry_off_proof)

    def test_quiet_retry_scope_never_enables_writes_emergency_or_other_endpoints(self):
        for path, payload, options in (
                ('/status', None, {}), # no scoped prior proof
                ('/action_outputs', None, {'emergency':True}),
                ('/command', {'cmd':'arm'}, {}),
                ('/sequence', {'action':'run'}, {'run':True}),
                ('/actions', None, {})):
            with self.subTest(path=path, options=options):
                port, device, _ = self.setup_port();attempts=[]
                port._quiet_retry_off_proof = path != '/status'
                def disconnect(request, timeout):
                    attempts.append(request);raise http.client.RemoteDisconnected()
                device.open = disconnect
                with self.assertRaises(http.client.RemoteDisconnected):
                    port._call(path, payload, quiet_disconnect_retry=True, **options)
                self.assertEqual(len(attempts), 1)

    def test_quiet_retry_second_disconnect_and_exhausted_budget_fail(self):
        for elapsed, count in ((.1, 2), (.81, 1)):
            port, device, _ = self.setup_port();port.preflight();attempts=[]
            def disconnect(request, timeout):
                attempts.append(timeout);device.clock.sleep(elapsed)
                raise http.client.RemoteDisconnected()
            device.open=disconnect
            with self.assertRaises(http.client.RemoteDisconnected):
                port.check_idle(allow_quiet_disconnect_retry=True)
            self.assertEqual(len(attempts), count)
            self.assertFalse(port.off_verified);self.assertFalse(port._quiet_retry_off_proof)

    def test_quiet_retry_does_not_accept_changed_boot_gap_or_nonzero_output(self):
        for changed in ('boot', 'gap', 'nonzero', 'unsafe_status'):
            with self.subTest(changed=changed):
                port, device, _ = self.setup_port();port.preflight();original=device.open;attempts=[]
                path = '/status' if changed == 'unsafe_status' else '/action_outputs'
                def disconnect(request, timeout):
                    if request.full_url.endswith(path):
                        attempts.append(timeout)
                        if len(attempts)==1:
                            if changed=='boot': device.boot='other'
                            elif changed=='gap':
                                for _ in range(130): device.append(1, 0)
                            elif changed=='nonzero': device.append(1, 20)
                            else: device.sensor_age=151
                            raise http.client.RemoteDisconnected()
                    return original(request, timeout)
                device.open=disconnect
                with self.assertRaises(ValueError): port.check_idle(allow_quiet_disconnect_retry=True)
                self.assertEqual(len(attempts), 2);self.assertFalse(port.off_verified)

    def test_quiet_retry_store_failure_timeout_and_invalid_response_do_not_retry(self):
        for mode in ('storage', 'timeout', 'malformed'):
            with self.subTest(mode=mode):
                port, device, store = self.setup_port();port.preflight();attempts=[]
                def fail(request, timeout):
                    attempts.append(timeout)
                    if mode=='storage':
                        store.broken=True;raise http.client.RemoteDisconnected()
                    if mode=='timeout': raise TimeoutError()
                    return RawResponse(b'{bad')
                device.open=fail
                with self.assertRaises((OSError, ValueError)):
                    port.check_idle(allow_quiet_disconnect_retry=True)
                self.assertEqual(len(attempts), 1);self.assertFalse(port.off_verified)

    def test_run_timeout_is_unknown_without_resend(self):
        port, device, _ = self.setup_port()
        def lose_response(controller): raise TimeoutError('lost RUN response')
        device.after_run = lose_response
        with self.assertRaises(TimeoutError): port.execute(pulse(), lambda: None)
        self.assertTrue(port.may_have_run)
        self.assertIsNotNone(port.last_dispatch_before)
        self.assertIsNone(port.last_dispatch_after)
        runs = [p for path, p in device.requests if p == {'action': 'run'}]
        self.assertEqual(len(runs), 1)
        self.assertEqual(device.default, 100);self.assertFalse(any(device.speeds.values()))
        self.assertFalse(device.queue);self.assertTrue(port.finish())

    def test_delayed_run_reply_does_not_shorten_requested_duration(self):
        port, device, store = self.setup_port()
        device.after_run = lambda controller: controller.clock.sleep(.25)
        result = port.execute(pulse(ms=200), lambda: None)
        stops = [row for name, row in store.rows.items() if name.endswith('-intent.json')
                 and row.get('payload') == {'cmd': 'stop'}]
        self.assertEqual(len(stops), 1)
        self.assertGreaterEqual(stops[0]['startedAt'], result['dispatchAfter'] + .2)

    def test_failed_run_intent_blocks_send_but_cleanup_still_runs(self):
        port, device, store = self.setup_port();store.fail_run_intent = True
        with self.assertRaises(OSError): port.execute(pulse(), lambda: None)
        self.assertFalse(port.may_have_run)
        self.assertFalse(any(p == {'action': 'run'} for _, p in device.requests))
        self.assertEqual(device.default, 100);self.assertFalse(device.queue)

    def test_before_run_is_after_durable_intent_and_exactly_once(self):
        port, device, store = self.setup_port();original = store.write_json
        persisted_at, called_at = [], []
        def delayed_intent(name, value):
            result = original(name, value)
            if name.endswith('-intent.json') and value.get('payload') == {'action': 'run'}:
                device.clock.sleep(.4)
                persisted_at.append(device.clock.monotonic())
            return result
        store.write_json = delayed_intent
        def guard():
            self.assertEqual(len(persisted_at), 1)
            self.assertFalse(port.may_have_run)
            self.assertIsNone(port.last_dispatch_before)
            self.assertFalse(any(p == {'action': 'run'} for _, p in device.requests))
            called_at.append(device.clock.monotonic())
        result = port.execute(pulse(), lambda: None, before_run=guard)
        self.assertEqual(called_at, persisted_at)
        self.assertGreaterEqual(result['dispatchBefore'], called_at[0])
        self.assertEqual(sum(p == {'action': 'run'} for _, p in device.requests), 1)
        self.assertTrue(result['offVerified']);self.assertIsNone(port._before_run)

    def test_after_armed_hook_is_once_outside_RUN_timeout_before_intent(self):
        port, device, store = self.setup_port();calls=[]
        def prepare():
            self.assertEqual(device.state,'ARMED')
            self.assertFalse(any(device.speeds.values()))
            self.assertFalse(any(v.get('payload')=={'action':'run'} for v in store.rows.values()))
            device.clock.sleep(port.timeout+1.)
            calls.append('prepared')
        result=port.execute(pulse(),lambda:None,after_armed_before_run=prepare,
                            before_run=lambda:calls.append('final'))
        self.assertEqual(calls,['prepared','final']);self.assertTrue(result['offVerified'])
        self.assertEqual(sum(p=={'action':'run'} for _,p in device.requests),1)

    def test_after_armed_rejection_has_no_RUN_intent_or_send_and_cleans_up(self):
        port,device,store=self.setup_port();calls=[]
        def reject():
            calls.append(True);raise ValueError('no_run_rehearsal_complete')
        with self.assertRaisesRegex(ValueError,'no_run_rehearsal_complete'):
            port.execute(pulse(),lambda:None,after_armed_before_run=reject)
        self.assertEqual(calls,[True]);self.assertFalse(port.may_have_run)
        self.assertFalse(any(v.get('payload')=={'action':'run'} for v in store.rows.values()))
        self.assertFalse(any(p=={'action':'run'} for _,p in device.requests))
        self.assertFalse(device.queue);self.assertFalse(any(device.speeds.values()))
        self.assertIsNone(port._before_run)

    def test_after_armed_noncallable_rejects_before_device_access(self):
        port,device,_=self.setup_port()
        with self.assertRaisesRegex(ValueError,'after_armed_before_run_callback_required'):
            port.execute(pulse(),lambda:None,after_armed_before_run=True)
        self.assertEqual(device.requests,[])

    def test_default_hook_does_not_add_keyword_to_existing_adapter(self):
        class LegacyAdapter(SequencePort):
            def _execute_validated(self,pulse,tick,*,before_run=None,monitor_policy=None):
                return 'unchanged'
        port=object.__new__(LegacyAdapter)
        self.assertEqual(port.execute(pulse(),lambda:None),'unchanged')

    def test_after_armed_hook_camera_failure_is_checked_before_RUN_intent(self):
        port,device,store=self.setup_port();failed=[]
        def tick():
            if failed:raise ValueError('camera_failed_during_preparation')
        with self.assertRaisesRegex(ValueError,'camera_failed_during_preparation'):
            port.execute(pulse(),tick,after_armed_before_run=lambda:failed.append(True))
        self.assertFalse(any(v.get('payload')=={'action':'run'} for v in store.rows.values()))
        self.assertFalse(any(p=={'action':'run'} for _,p in device.requests))
        self.assertFalse(any(device.speeds.values()))

    def test_before_run_rejection_never_sends_and_keeps_cleanup(self):
        for error in (ValueError('stale_observation'), KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__):
                port, device, _ = self.setup_port();calls = []
                def reject():
                    calls.append(True)
                    raise error
                with self.assertRaises(type(error)):
                    port.execute(pulse(), lambda: None, before_run=reject)
                self.assertEqual(calls, [True]);self.assertFalse(port.may_have_run)
                self.assertIsNone(port.last_dispatch_before);self.assertIsNone(port._before_run)
                self.assertFalse(any(p == {'action': 'run'} for _, p in device.requests))
                self.assertEqual(sum(p == {'cmd': 'stop'} for _, p in device.requests), 1)
                self.assertFalse(device.queue);self.assertFalse(any(device.speeds.values()))
                self.assertEqual(device.default, 100)

    def test_before_run_can_reject_observation_aged_by_intent_persistence(self):
        port, device, store = self.setup_port();original = store.write_json
        observation_at = device.clock.monotonic()
        def delayed_intent(name, value):
            result = original(name, value)
            if name.endswith('-intent.json') and value.get('payload') == {'action': 'run'}:
                device.clock.sleep(1.)
            return result
        store.write_json = delayed_intent
        def guard():
            if device.clock.monotonic() - observation_at > .5:
                raise ValueError('stale_observation')
        with self.assertRaisesRegex(ValueError, 'stale_observation'):
            port.execute(pulse(), lambda: None, before_run=guard)
        self.assertFalse(port.may_have_run)
        self.assertFalse(any(p == {'action': 'run'} for _, p in device.requests))

    def test_before_run_time_is_in_request_budget(self):
        port, device, _ = self.setup_port()
        with self.assertRaisesRegex(TimeoutError, 'request_deadline'):
            port.execute(pulse(), lambda: None,
                         before_run=lambda: device.clock.sleep(port.timeout + .01))
        self.assertFalse(port.may_have_run);self.assertIsNone(port.last_dispatch_before)
        self.assertFalse(any(p == {'action': 'run'} for _, p in device.requests))
        self.assertFalse(device.queue);self.assertFalse(any(device.speeds.values()))

    def test_before_run_failure_or_early_error_does_not_leak_to_next_execute(self):
        for mode in ('guard', 'intent', 'preflight'):
            with self.subTest(mode=mode):
                port, device, store = self.setup_port();calls = []
                def guard():
                    calls.append(True)
                    if mode == 'guard':
                        raise ValueError('guard_rejected')
                if mode == 'intent': store.fail_run_intent = True
                if mode == 'preflight': device.sensor_age = 151
                with self.assertRaises((ValueError, OSError)):
                    port.execute(pulse(), lambda: None, before_run=guard)
                self.assertEqual(calls, [True] if mode == 'guard' else [])
                self.assertIsNone(port._before_run)
                count = len(calls)
                # A new explicit execute is not an automatic retry. Its absent
                # guard must not inherit a prior successful or rejected guard.
                store.fail_run_intent = False;device.sensor_age = 20;port.evidence_failed = False
                port.execute(pulse(), lambda: None)
                self.assertEqual(len(calls), count)
                self.assertEqual(sum(p == {'action': 'run'} for _, p in device.requests), 1)

    def test_before_run_noncallable_rejected_before_any_device_access(self):
        for value in (False, True, 1, 'guard', {}):
            with self.subTest(value=value):
                port, device, _ = self.setup_port()
                with self.assertRaisesRegex(ValueError, 'before_run_callback_required'):
                    port.execute(pulse(), lambda: None, before_run=value)
                self.assertEqual(device.requests, [])

    def test_before_run_is_consumed_after_ambiguous_run_without_retry(self):
        port, device, _ = self.setup_port();calls = []
        device.after_run = lambda controller: (_ for _ in ()).throw(TimeoutError('lost RUN response'))
        with self.assertRaises(TimeoutError):
            port.execute(pulse(), lambda: None, before_run=lambda: calls.append(True))
        self.assertEqual(calls, [True]);self.assertIsNone(port._before_run)
        self.assertTrue(port.may_have_run)
        self.assertEqual(sum(p == {'action': 'run'} for _, p in device.requests), 1)

    def test_storage_breaks_after_run_emergency_stop_not_blocked(self):
        port, device, store = self.setup_port()
        device.after_run = lambda c: setattr(store, 'broken', True)
        with self.assertRaises(OSError): port.execute(pulse(), lambda: None)
        self.assertTrue(port.may_have_run);self.assertFalse(any(device.speeds.values()))
        self.assertEqual(device.default, 100);self.assertFalse(device.queue)

    def test_wrong_motor_or_output_gap_or_boot_change_stops(self):
        def wrong_motor(d): d.append(1, 10)
        def gap(d): d.entries[-1]['sequence'] += 1
        def reboot(d): d.boot = 'different'
        for mutation in (wrong_motor, gap, reboot):
            with self.subTest(mutation=mutation.__name__):
                port, device, _ = self.setup_port();device.after_run = mutation
                with self.assertRaises(ValueError): port.execute(pulse(), lambda: None)
                self.assertTrue(port.may_have_run);self.assertFalse(any(device.speeds.values()))
                self.assertEqual(device.default, 100);self.assertFalse(device.queue)

    def test_unexpected_quiet_pulse_even_when_off_again_is_rejected(self):
        port, device, _ = self.setup_port();port.preflight()
        device.append(2, 10);device.append(2, 0)
        with self.assertRaisesRegex(ValueError, 'quiet'): port.check_idle()

    def test_postpulse_quiet_fault_gets_fresh_stop_and_off_proof(self):
        port, device, _ = self.setup_port();port.execute(pulse(), lambda: None)
        prior_stops = sum(p == {'cmd': 'stop'} for _, p in device.requests)
        device.append(2, 10);device.state = 'ARMED'
        with self.assertRaises(ValueError): port.check_idle()
        self.assertFalse(port.off_verified)
        self.assertTrue(port.finish())
        self.assertEqual(sum(p == {'cmd': 'stop'} for _, p in device.requests), prior_stops + 1)
        self.assertFalse(any(device.speeds.values()))

    def test_postpulse_reboot_does_not_reuse_old_off_proof(self):
        port, device, _ = self.setup_port();port.execute(pulse(), lambda: None)
        device.boot = 'rebooted'
        with self.assertRaises(ValueError): port.check_idle()
        self.assertFalse(port.finish())

    def test_m4_sensor_change_stops(self):
        port, device, _ = self.setup_port()
        device.after_run = lambda d: setattr(d, 'angle', 216.)
        with self.assertRaisesRegex(ValueError, 'm4_observed'): port.execute(pulse('shoulder', 'down'), lambda: None)
        self.assertFalse(any(device.speeds.values()));self.assertEqual(device.default, 100)

    def test_m4_change_before_run_blocks_output(self):
        port, device, _ = self.setup_port();original = device.open
        def change_on_arm(request, timeout):
            result = original(request, timeout)
            if request.data == b'cmd=arm': device.angle = 216.
            return result
        device.open = change_on_arm
        with self.assertRaisesRegex(ValueError, 'before_run'):
            port.execute(pulse('shoulder', 'down'), lambda: None)
        self.assertFalse(port.may_have_run)
        self.assertFalse(any(p == {'action': 'run'} for _, p in device.requests))
        self.assertEqual(device.default, 100)

    def test_stop_response_loss_does_not_repeat_write(self):
        port, device, _ = self.setup_port();device.fail_stop = True
        with self.assertRaisesRegex(ValueError, 'cleanup'): port.execute(pulse(), lambda: None)
        self.assertEqual(sum(p == {'cmd': 'stop'} for _, p in device.requests), 1)
        self.assertFalse(port.finish());self.assertFalse(any(device.speeds.values()))
        self.assertEqual(device.default, 100);self.assertFalse(device.queue)

    def test_idle_read_after_failed_cleanup_does_not_repeat_writes(self):
        port, device, _ = self.setup_port();device.fail_stop = True
        with self.assertRaisesRegex(ValueError, 'cleanup'):
            port.execute(pulse(), lambda: None)
        writes = [p for _, p in device.requests if p is not None]
        # ManualCollection finalization checks idle before calling finish again.
        port.check_idle()
        self.assertFalse(port.finish())
        self.assertEqual([p for _, p in device.requests if p is not None], writes)
        self.assertEqual(sum(p == {'cmd': 'stop'} for p in writes), 1)

    def test_queue_race_prevents_arm_and_run(self):
        port, device, _ = self.setup_port();original = device.open
        def mutate(request, timeout):
            result = original(request, timeout)
            if request.data and b'action=add' in request.data: device.queue.append({'foreign': True})
            return result
        device.open = mutate
        with self.assertRaisesRegex(ValueError, 'queue'): port.execute(pulse(), lambda: None)
        self.assertFalse(any(p == {'action': 'run'} or p == {'cmd': 'arm'} for _, p in device.requests))
        self.assertFalse(device.queue);self.assertEqual(device.default, 100)

    def test_legacy_duplicate_state_only_exception(self):
        parsed = response_json(b'{"state":"IDLE","state":"IDLE","sequence":{"state":"IDLE"}}', '/sequence')
        self.assertEqual(parsed['legacyStateValues'], ['IDLE', 'IDLE'])
        for path in ('/status', '/actions'):
            with self.assertRaises(ValueError): response_json(b'{"state":"IDLE","state":"ARMED"}', path)

    def test_legacy_arm_stop_roundtrip_retains_both_states(self):
        port, device, store = self.setup_port()
        result = port.execute(pulse(), lambda: None)
        self.assertTrue(result['offVerified'])
        self.assertTrue(port.finish())
        responses = [v for k, v in store.rows.items() if k.endswith('-response.json')
                     and 'legacyStateValues' in v['parsed']]
        self.assertEqual([r['parsed']['legacyStateValues'] for r in responses],
                         [['ARMED', 'ARMED'], ['IDLE', 'IDLE'], ['IDLE', 'IDLE']])
        self.assertTrue(all(r['rawUtf8'].count('"state"') == 2 for r in responses))
        self.assertEqual(sum(p == {'action': 'run'} for _, p in device.requests), 1)

    def test_legacy_state_exception_rejects_ambiguity_and_nested_duplicates(self):
        invalid = [
            (b'{"state":"IDLE","state":"ARMED"}', '/command'),
            (b'{"state":0,"state":0}', '/command'),
            (b'{"state":"IDLE","state":"IDLE","state":"IDLE"}', '/command'),
            (b'{"success":false,"success":true}', '/command'),
            (b'{"nested":{"state":"IDLE","state":"IDLE"}}', '/command'),
            (b'{"nested":[{"state":"IDLE","state":"IDLE"}]}', '/sequence'),
            (b'{"state":"ARMED","state":"RUNNING","state":"IDLE"}', '/sequence'),
            (b'{"legacyStateValues":["ARMED"],"state":"IDLE","state":"IDLE"}', '/command'),
            (b'{"state":"IDLE","state":"IDLE","legacyStateValues":["ARMED"]}', '/command'),
        ]
        for raw, path in invalid:
            with self.subTest(raw=raw, path=path), self.assertRaises(ValueError):
                response_json(raw, path)
        parsed = response_json(b'{"state":"ARMED","state":"RUNNING"}', '/sequence')
        self.assertEqual(parsed['legacyStateValues'], ['ARMED', 'RUNNING'])

    def test_conflicting_arm_reply_prevents_run_and_still_stops(self):
        port, device, _ = self.setup_port();original = device.open
        def conflict(request, timeout):
            result = original(request, timeout)
            if request.data == b'cmd=arm':
                return RawResponse(b'{"success":true,"state":"IDLE","state":"ARMED"}')
            return result
        device.open = conflict
        with self.assertRaisesRegex(ValueError, 'duplicate_response_key'):
            port.execute(pulse(), lambda: None)
        self.assertFalse(port.may_have_run)
        self.assertFalse(any(p == {'action': 'run'} for _, p in device.requests))
        self.assertEqual(sum(p == {'cmd': 'stop'} for _, p in device.requests), 1)
        self.assertTrue(port.finish())
        self.assertFalse(device.queue)



class CompactTransportTests(unittest.TestCase):
    def setup_port(self):
        port, device, store = TransportTests().setup_port()
        port.compact_reads = True
        original = device.open
        def open_page(request, timeout):
            address = urllib.parse.urlsplit(request.full_url)
            if address.path == '/collection_status':
                device.clock.now += .01
                device.requests.append((address.path, None))
                return Response(dict(device.status(), schemaVersion='collection.v1'))
            if address.path == '/action_outputs':
                device.clock.now += .01
                device.requests.append((address.path+'?'+address.query, None))
                cursor = int(urllib.parse.parse_qs(address.query)['after'][0])
                retained = device.entries[-128:]
                selected = [e for e in retained if e['sequence'] > cursor][:8]
                oldest, latest = retained[0]['sequence'], retained[-1]['sequence']
                return Response(dict(bootId=device.boot, evidence='driver_write',
                    clockDomain='controller_boot_millis', entries=selected,
                    oldestSequence=oldest, latestSequence=latest,
                    gap=bool(cursor and (cursor < oldest-1 or cursor > latest)),
                    hasMore=bool(selected and selected[-1]['sequence'] < latest)))
            return original(request, timeout)
        port.opener = type('Pages', (), {'open': staticmethod(open_page)})()
        return port, device, store

    def test_compact_preflight_empty_tail_and_execute(self):
        port, device, store = self.setup_port()
        port.preflight()
        port.check_idle()
        result = port.execute(pulse('shoulder', 'down', 20, 250), lambda: None)
        self.assertTrue(result['offVerified'])
        self.assertTrue(any(e['pwm'] == -51 for e in result['outputTrace']))
        self.assertTrue(any(path == '/collection_status' for path, _ in device.requests))
        self.assertFalse(any(path == '/status' for path, _ in device.requests))
        self.assertEqual(sum(body == {'action': 'run'} for _, body in device.requests), 1)

    def test_bootstrap_retained_ring_then_page_gap_rejected(self):
        port, device, _ = self.setup_port()
        for i in range(150): device.append(i % 5 + 1, 0)
        port.preflight()
        self.assertEqual(port.last_sequence, 155)
        for i in range(129): device.append(i % 5 + 1, 0)
        with self.assertRaisesRegex(ValueError, 'gap'): port.check_idle()

    def test_missing_axis_history_fails_closed_until_explicit_all_axis_stop(self):
        port, device, _ = self.setup_port()
        # A long manual M4 movement can evict every earlier M1/M2/M3/M5
        # driver write from the retained ring, even when status says IDLE.
        for _ in range(128): device.append(4, 0)
        with self.assertRaisesRegex(ValueError, 'driver_off_not_proven'):
            port.preflight()
        self.assertFalse(port.off_verified)
        self.assertTrue(all(body is None for _, body in device.requests))

        # Only a distinct explicitly issued STOP records a fresh OFF for each
        # axis. It is not an automatic retry or part of the failed preflight.
        port._call('/command', {'cmd': 'stop'})
        fresh, _, _ = self.setup_port()
        fresh.opener = port.opener
        fresh.preflight()
        self.assertTrue(fresh.off_verified)
        self.assertEqual(fresh.last_outputs, {i: 0 for i in range(1, 6)})

    def test_quiet_later_page_nonzero_rejected(self):
        port, device, _ = self.setup_port();port.preflight()
        for _ in range(8): device.append(4, 0)
        device.append(4, -20);device.append(4, 0)
        with self.assertRaisesRegex(ValueError, 'quiet'): port.check_idle()

    def test_paged_boot_identity_change_rejected(self):
        port, device, _ = self.setup_port();port.preflight();device.boot='other'
        with self.assertRaisesRegex(ValueError, 'identity'): port.check_idle()

    def test_page_drain_shares_timeout(self):
        port, device, _ = self.setup_port()
        port.preflight()
        for i in range(24): device.append(i % 5 + 1, 0)
        original = port.opener.open
        def slow(request, timeout):
            if '/action_outputs?' in request.full_url: device.clock.now += .4
            return original(request, timeout)
        port.opener.open = slow
        with self.assertRaisesRegex(TimeoutError, 'deadline'): port.check_idle()
        self.assertTrue(all(body is None for _, body in device.requests))

    def test_extended_quiet_output_drain_has_one_shared_budget_and_no_retry(self):
        port, device, store = self.setup_port(); port.preflight()
        for i in range(16): device.append(i % 5 + 1, 0)
        original = port.opener.open; seen = []
        def slow(request, timeout):
            if '/action_outputs?' in request.full_url:
                seen.append(timeout); device.clock.now += .9
            return original(request, timeout)
        port.opener.open = slow
        with self.assertRaisesRegex(TimeoutError, 'deadline'):
            port.check_idle(allow_quiet_disconnect_retry=True, allow_quiet_monitor_timeout=True)
        self.assertEqual(len(seen), 2)
        self.assertAlmostEqual(seen[0], 1.5); self.assertAlmostEqual(seen[1], .59)
        self.assertFalse(port.off_verified)
        self.assertFalse(any('attempt-2' in name for name in store.rows))

    def test_active_drain_cannot_use_initial_bootstrap_budget(self):
        port, device, _ = self.setup_port()
        port._active_pulse = pulse('shoulder', 'down', 20, 250)
        port.may_have_run = True
        port.writes_possible = True
        for i in range(24): device.append(i % 5 + 1, 0)
        original = port.opener.open
        def delayed(request, timeout):
            if '/action_outputs?' in request.full_url: device.clock.now += .4
            return original(request, timeout)
        port.opener.open = delayed
        with self.assertRaisesRegex(TimeoutError, 'deadline'): port._outputs()

    def test_initial_full_ring_accepts_normal_page_latency(self):
        port, device, _ = self.setup_port()
        for i in range(150): device.append(i % 5 + 1, 0)
        original = port.opener.open
        def delayed(request, timeout):
            if '/action_outputs?' in request.full_url:
                self.assertLessEqual(timeout, 1.5)
                device.clock.now += .05
            return original(request, timeout)
        port.opener.open = delayed
        port.preflight()
        self.assertEqual(port.last_sequence, 155)
        self.assertTrue(port.off_verified)
        self.assertTrue(all(body is None for _, body in device.requests))

    def test_initial_full_ring_still_has_eight_second_limit(self):
        port, device, _ = self.setup_port()
        for i in range(150): device.append(i % 5 + 1, 0)
        original = port.opener.open
        pages = []
        def delayed(request, timeout):
            if '/action_outputs?' in request.full_url:
                pages.append(request.full_url)
                self.assertLessEqual(timeout, 1.5)
                if timeout < .55:
                    device.clock.sleep(timeout)
                    raise TimeoutError('request_deadline')
                device.clock.sleep(.55)
            return original(request, timeout)
        port.opener.open = delayed
        with self.assertRaisesRegex(TimeoutError, 'deadline'): port.preflight()
        self.assertFalse(port.off_verified)
        self.assertLessEqual(len(pages), 16)
        self.assertEqual(len(set(pages)), len(pages))
        self.assertTrue(all(body is None for _, body in device.requests))

    def test_initial_full_ring_accepts_six_second_drain_only_before_first_snapshot(self):
        port, device, store = self.setup_port()
        for i in range(150): device.append(i % 5 + 1, 0)
        original = port.opener.open
        pages = []
        def delayed(request, timeout):
            if '/action_outputs?' in request.full_url:
                pages.append(request.full_url)
                self.assertLessEqual(timeout, 1.5)
                if timeout < .36:
                    device.clock.sleep(timeout)
                    raise TimeoutError('request_deadline')
                device.clock.sleep(.36)
            return original(request, timeout)
        port.opener.open = delayed
        started = device.clock.monotonic()
        port.preflight()
        self.assertTrue(5.8 < device.clock.monotonic()-started < 6.1)
        self.assertEqual(len(pages), 16)
        self.assertEqual(len(set(pages)), 16)
        self.assertTrue(port.off_verified)
        self.assertFalse(any('attempt-2' in name for name in store.rows))
        self.assertTrue(all(body is None for _, body in device.requests))
        # A later multi-page drain still shares the ordinary .8s deadline.
        for i in range(24): device.append(i % 5 + 1, 0)
        with self.assertRaisesRegex(TimeoutError, 'deadline'): port.check_idle()
        self.assertFalse(port.off_verified)

    def test_initial_slow_page_can_complete_without_retry(self):
        port, device, store = self.setup_port()
        for i in range(150): device.append(i % 5 + 1, 0)
        original = port.opener.open; pages = []
        def delayed(request, timeout):
            if '/action_outputs?' in request.full_url:
                pages.append(request.full_url)
                elapsed = .9 if len(pages) == 11 else .08
                if timeout < elapsed:
                    device.clock.sleep(timeout)
                    raise TimeoutError('request_deadline')
                device.clock.sleep(elapsed)
            return original(request, timeout)
        port.opener.open = delayed
        port.preflight()
        self.assertTrue(port.off_verified)
        self.assertEqual(port.last_sequence, 155)
        self.assertEqual(len(pages), 16)
        self.assertEqual(len(set(pages)), 16)
        self.assertFalse(any('attempt-2' in name for name in store.rows))
        self.assertTrue(all(body is None for _, body in device.requests))
        # Once the first snapshot exists, ordinary idle checks retain .8s.
        seen = []
        def monitor(request, timeout):
            if '/action_outputs?' in request.full_url: seen.append(timeout)
            return original(request, timeout)
        port.opener.open = monitor
        port.check_idle()
        self.assertEqual(seen, [.8])

    def test_initial_output_extension_rejects_other_scopes_before_request(self):
        cases = [dict(compact_reads=False), dict(writes_possible=True), dict(may_have_run=True),
                 dict(evidence_failed=True), dict(off_verified=True), dict(last_snapshot={}),
                 dict(_active_pulse=pulse('shoulder', 'down', 20, 250))]
        for attributes in cases:
            with self.subTest(attributes=attributes):
                port, device, _ = self.setup_port()
                for key, value in attributes.items(): setattr(port, key, value)
                with self.assertRaisesRegex(ValueError, 'initial_output_read_scope'):
                    port._call('/action_outputs?after=0&limit=8', request_timeout=4,
                               initial_output_read=True)
                self.assertFalse(device.requests)
        for path, payload, flags in [('/collection_status', None, {}),
                ('/action_outputs', None, {'emergency': True}),
                ('/action_outputs', None, {'run': True}),
                ('/action_outputs', None, {'quiet_disconnect_retry': True}),
                ('/action_outputs', None, {'quiet_monitor_timeout': True}),
                ('/action_outputs', {'action': 'run'}, {})]:
            with self.subTest(path=path, flags=flags):
                port, device, _ = self.setup_port()
                with self.assertRaisesRegex(ValueError, 'initial_output_read_scope'):
                    port._call(path, payload, request_timeout=4, initial_output_read=True, **flags)
                self.assertFalse(device.requests)

    def test_missing_schema_fails_before_write(self):
        port, device, _ = self.setup_port()
        original=port.opener.open
        port.opener.open=lambda request,timeout: (Response(device.status())
            if '/collection_status' in request.full_url else original(request,timeout))
        with self.assertRaisesRegex(ValueError, 'schema'): port.preflight()
        self.assertTrue(all(body is None for _, body in device.requests))

    def test_cross_page_time_regression_rejected(self):
        port, device, _ = self.setup_port();port.preflight()
        device.append(4, 0);device.entries[-1]['atMs']=0
        with self.assertRaisesRegex(ValueError, 'time_regressed'): port.check_idle()

    def test_one_quiet_read_recovery_rechecks_status_and_ring_without_writes(self):
        port,device,store=self.setup_port();port.preflight();original=port.opener.open;calls=[]
        def once(request,timeout):
            path=urllib.parse.urlsplit(request.full_url).path;calls.append(path)
            if path=='/action_outputs' and calls.count(path)==1:
                device.clock.sleep(timeout);raise TimeoutError()
            return original(request,timeout)
        port.opener.open=once;before=len(device.requests)
        port.check_idle(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True,allow_quiet_read_recovery=True)
        self.assertEqual(calls,['/collection_status','/action_outputs','/collection_status','/action_outputs'])
        self.assertTrue(port.off_verified);self.assertTrue(port._quiet_read_recovery_used)
        self.assertTrue(all(payload is None for _,payload in device.requests[before:]))
        self.assertTrue(any(k.endswith('recovery-intent.json') for k in store.rows))
        calls.clear()
        with self.assertRaises(TimeoutError):
            port.check_idle(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True,allow_quiet_read_recovery=True)
        self.assertEqual(calls,['/collection_status','/action_outputs'])

    def test_quiet_recovery_total_deadline_and_repeated_failure(self):
        port,device,store=self.setup_port();port.preflight();started=device.clock.now;calls=[]
        def fail(request,timeout):
            calls.append(timeout);device.clock.sleep(timeout);raise TimeoutError()
        port.opener.open=fail
        with self.assertRaises(TimeoutError):
            port.check_idle(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True,allow_quiet_read_recovery=True)
        self.assertEqual(len(calls),2);self.assertLessEqual(device.clock.now-started,4.5)
        self.assertFalse(port.off_verified)

    def test_quiet_recovery_never_accepts_new_output_or_gap_or_boot_change(self):
        for bad in ('nonzero','gap','boot','sensor'):
            port,device,_=self.setup_port();port.preflight();original=port.opener.open;failed=False
            def fail(request,timeout):
                nonlocal failed
                if not failed:
                    failed=True
                    if bad=='nonzero':device.append(1,10);device.append(1,0)
                    elif bad=='gap':
                        for _ in range(129):device.append(1,0)
                    elif bad=='boot':device.boot='changed'
                    else:device.sensor_age=151
                    raise TimeoutError()
                return original(request,timeout)
            port.opener.open=fail
            with self.subTest(bad=bad),self.assertRaises(ValueError):
                port.check_idle(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True,allow_quiet_read_recovery=True)
            self.assertFalse(port.off_verified)

    def test_quiet_read_recovery_has_no_active_or_unproven_scope(self):
        for kind in ('active','off','evidence'):
            port,device,_=self.setup_port();port.preflight();calls=[]
            if kind=='active':port._active_pulse=pulse()
            elif kind=='off':port.off_verified=False
            else:port.evidence_failed=True
            def fail(request,timeout):calls.append(timeout);raise TimeoutError()
            port.opener.open=fail
            with self.assertRaises(TimeoutError):
                port.check_idle(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True,allow_quiet_read_recovery=True)
            self.assertEqual(len(calls),1);self.assertFalse(port._quiet_read_recovery_used)

    def test_successful_http_but_expired_drain_is_not_a_read_recovery(self):
        port,device,store=self.setup_port();port.preflight();original=port.opener.open
        def slow(request,timeout):
            result=original(request,timeout)
            if '/action_outputs' in request.full_url:device.clock.sleep(timeout)
            return result
        port.opener.open=slow
        with self.assertRaises(TimeoutError):
            port.check_idle(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True,allow_quiet_read_recovery=True)
        self.assertFalse(port._quiet_read_recovery_used)
        self.assertFalse(any(k.endswith('recovery-intent.json') for k in store.rows))


if __name__ == '__main__': unittest.main()
