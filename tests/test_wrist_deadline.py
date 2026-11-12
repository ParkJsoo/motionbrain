"""M2 550ms 명시 모드의 마감·실패·실행 조건 분리 회귀."""
import copy
import unittest
import urllib.parse
from contextlib import contextmanager
from unittest.mock import patch
from tests import test_supervised_transport as fixture
from tests.test_pipeline_learning import examples, manifest
from learning.pipeline_learning import run_learning
from learning.relative_model_bundle import validate_model
from learning.supervised_transport import WRIST_DEADLINE_POLICY as POLICY


class WristDeadlineTests(unittest.TestCase):
    def port(self):
        return fixture.TransportTests().setup_port()

    def test_slow_reads_do_not_start_after_deadline_or_delay_stop(self):
        port, device, store = self.port(); original = device.open
        def slow(request, timeout):
            if device.running and request.data is None:
                self.assertLessEqual(timeout, port.last_dispatch_after+.55-device.clock.monotonic()+1e-8)
                device.clock.sleep(.11)
            return original(request, timeout)
        device.open = slow
        receipt = port.execute(fixture.pulse(ms=550), lambda:None, monitor_policy=POLICY)
        end = receipt['dispatchAfter']+.55
        intents = [r for n,r in store.rows.items() if n.endswith('-intent.json')]
        stop = next(r for r in intents if r['payload']=={'cmd':'stop'})
        self.assertAlmostEqual(stop['startedAt'], end)
        for row in intents:
            if receipt['dispatchAfter']<=row['startedAt']<stop['startedAt'] and row['method']=='GET':
                self.assertLess(row['startedAt'], end)
        self.assertTrue(receipt['monitorStatusVerified'])
        self.assertTrue(receipt['offVerified'])

    def test_read_over_point_two_seconds_can_finish_inside_shared_deadline(self):
        port, device, store=self.port();original=device.open
        def slow(request,timeout):
            if device.running and request.data is None:
                delay=.22 if '/action_outputs' in request.full_url else .08
                if delay+.01>=timeout:raise TimeoutError('read would exceed command deadline')
                device.clock.sleep(delay)
            return original(request,timeout)
        device.open=slow
        result=port.execute(fixture.pulse(ms=550),lambda:None,monitor_policy=POLICY)
        stop=next(r for n,r in store.rows.items() if n.endswith('-intent.json') and r['payload']=={'cmd':'stop'})
        self.assertAlmostEqual(stop['startedAt'],result['dispatchAfter']+.55)
        self.assertTrue(result['offVerified'])

    def test_persisted_read_intent_cannot_open_socket_after_deadline(self):
        port, device, store = self.port(); saved = store.write_json
        def slow_store(name, value):
            saved(name,value)
            if name.endswith('-intent.json'): device.clock.sleep(.03)
        store.write_json = slow_store
        with self.assertRaisesRegex(TimeoutError, 'request_deadline'):
            port._call('/status', read_deadline=device.clock.monotonic()+.02)
        self.assertEqual(device.requests, [])
        self.assertTrue(any(n.endswith('-error.json') for n in store.rows))

    def test_active_read_timeout_stops_once_and_does_not_become_success(self):
        port, device, _ = self.port(); original = device.open
        def timeout(request, timeout):
            if device.running and request.data is None:
                device.clock.sleep(timeout)
                raise TimeoutError('synthetic monitor timeout')
            return original(request,timeout)
        device.open = timeout
        with self.assertRaises(TimeoutError):
            port.execute(fixture.pulse(ms=550),lambda:None,monitor_policy=POLICY)
        self.assertEqual(sum(p=={'action':'run'} for _,p in device.requests),1)
        self.assertEqual(sum(p=={'cmd':'stop'} for _,p in device.requests),1)
        self.assertTrue(port.off_verified)

    def test_alarm_setup_delay_cannot_open_socket_after_absolute_deadline(self):
        port, device, _ = self.port()
        @contextmanager
        def delayed_alarm(seconds):
            device.clock.sleep(.25)
            yield
        with patch('learning.supervised_transport.deadline', delayed_alarm):
            with self.assertRaisesRegex(TimeoutError,'request_deadline'):
                port._call('/status',read_deadline=device.clock.monotonic()+.2)
        self.assertFalse(device.requests)

    def test_late_status_persistence_is_failed_and_stopped_not_promoted(self):
        port, device, store = self.port(); original=store.write_json
        def slow_status(name,value):
            original(name,value)
            if device.running and name.endswith('-response.json') and value.get('parsed',{}).get('state')=='ARMED':
                device.clock.sleep(.55)
        store.write_json=slow_status
        with self.assertRaisesRegex(TimeoutError,'request_deadline'):
            port.execute(fixture.pulse(ms=550),lambda:None,monitor_policy=POLICY)
        self.assertEqual(sum(p=={'cmd':'stop'} for _,p in device.requests),1)
        self.assertTrue(port.off_verified)

    def test_local_tick_expiry_cannot_skip_required_active_status(self):
        port, device, _ = self.port()
        def tick():
            if device.running: device.clock.sleep(.6)
        with self.assertRaisesRegex(ValueError,'active_safety_status_not_verified'):
            port.execute(fixture.pulse(ms=550),tick,monitor_policy=POLICY)
        self.assertEqual(sum(p=={'cmd':'stop'} for _,p in device.requests),1)

    def test_policy_is_explicit_and_cannot_change_M4_or_short_pulses(self):
        for command in [fixture.pulse(ms=200),fixture.pulse('shoulder','down',20,550)]:
            port, device, _ = self.port()
            with self.assertRaisesRegex(ValueError,'unsupported_monitor'):
                port.execute(command,lambda:None,monitor_policy=POLICY)
            self.assertFalse(device.requests)

    def test_new_timing_condition_is_a_separate_model_partition(self):
        old = examples()[:2]
        for row in old:row['command']=dict(joint='wrist',direction='up',percent=63,durationMs=550)
        new=copy.deepcopy(old)
        for i,row in enumerate(new):
            row.update(sessionId='new'+str(i),sourceHash=str(i+1)*64,episode=['new',str(i)],frameIds=['new'+str(i)+'a','new'+str(i)+'b'])
            row['context']['executionTimingPolicy']=POLICY
        report=run_learning(old+new,manifest())
        self.assertEqual(len(report['partitions']),2)
        for partition in report['partitions']:validate_model(partition['model'])


if __name__=='__main__':unittest.main()
