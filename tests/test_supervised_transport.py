import copy
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


if __name__ == '__main__': unittest.main()
