"""M3 고정 출력 범위의 마감·실패 처리와 모델 조건 분리."""
import copy
import unittest
from tests import test_supervised_transport as fixture
from tests.test_pipeline_learning import examples, manifest
from learning.pipeline_learning import run_learning
from learning.relative_model_bundle import validate_model
from learning.supervised_transport import ELBOW_DEADLINE_POLICY as POLICY


class ElbowDeadlineTests(unittest.TestCase):
    def test_both_directions_share_absolute_deadline_and_stop(self):
        for direction in ('up', 'down'):
            port, device, store = fixture.TransportTests().setup_port()
            original = device.open
            def slow(request, timeout):
                if device.running and request.data is None:
                    self.assertLessEqual(timeout, port.last_dispatch_after+.35-device.clock.monotonic()+1e-8)
                    device.clock.sleep(.06)
                return original(request, timeout)
            device.open = slow
            result = port.execute(fixture.pulse('elbow', direction, 60, 350),
                                  lambda: None, monitor_policy=POLICY)
            stop = next(r for n, r in store.rows.items()
                        if n.endswith('-intent.json') and r['payload'] == {'cmd': 'stop'})
            self.assertAlmostEqual(stop['startedAt'], result['dispatchAfter']+.35)
            self.assertTrue(result['monitorStatusVerified'] and result['offVerified'])
            self.assertEqual(result['monitorTimingPolicy'], POLICY)

    def test_other_profiles_rejected_before_network_access(self):
        for cmd in [fixture.pulse('elbow', 'up', 59, 350),
                    fixture.pulse('elbow', 'down', 60, 351),
                    fixture.pulse('wrist', 'up', 60, 350),
                    fixture.pulse('shoulder', 'down', 35, 350)]:
            port, device, _ = fixture.TransportTests().setup_port()
            with self.assertRaisesRegex(ValueError, 'unsupported_monitor'):
                port.execute(cmd, lambda: None, monitor_policy=POLICY)
            self.assertFalse(device.requests)

    def test_active_timeout_preserves_failure_and_stops_without_replay(self):
        port, device, _ = fixture.TransportTests().setup_port()
        original = device.open
        def timeout(request, timeout):
            if device.running and request.data is None:
                device.clock.sleep(timeout)
                raise TimeoutError('active read deadline')
            return original(request, timeout)
        device.open = timeout
        with self.assertRaises(TimeoutError):
            port.execute(fixture.pulse('elbow', 'up', 60, 350), lambda: None, monitor_policy=POLICY)
        self.assertEqual(sum(p == {'action': 'run'} for _, p in device.requests), 1)
        self.assertEqual(sum(p == {'cmd': 'stop'} for _, p in device.requests), 1)
        self.assertTrue(port.off_verified)

    def test_new_policy_model_is_partitioned_and_cannot_use_wrist_command(self):
        old = examples()[:2]
        for row in old:
            row['command'] = dict(joint='elbow', direction='up', percent=60, durationMs=350)
        new = copy.deepcopy(old)
        for i, row in enumerate(new):
            row.update(sessionId='elbow-new'+str(i), sourceHash=str(i+1)*64,
                       episode=['elbow-new', str(i)], frameIds=['elbow-new'+str(i)+s for s in 'ab'])
            row['context']['executionTimingPolicy'] = POLICY
        report = run_learning(old+new, manifest())
        self.assertEqual(len(report['partitions']), 2)
        model = next(p['model'] for p in report['partitions']
                     if 'executionTimingPolicy' in p['model']['context'])
        validate_model(model)
        model['models'][0]['command']['joint'] = 'wrist'
        with self.assertRaisesRegex(ValueError, 'unsupported_monitor'):
            validate_model(model)
