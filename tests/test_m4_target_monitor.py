"""TARGET 전용 공유 deadline·STOP 우선 회귀. 실제 장치/네트워크 요청 없음."""
import unittest
import urllib.parse

from learning.supervised_transport import (SequencePort,M4_TARGET_UP_DEADLINE_POLICY,
    ELBOW_DEADLINE_POLICY,validate_monitor_policy)
from tests import test_supervised_transport as fixtures


class TargetMonitorTests(unittest.TestCase):
    def setUp(self):
        self.clock=fixtures.Clock();self.store=fixtures.Store();self.device=fixtures.Controller(self.clock)
        self.device.angle=195.29
        self.port=SequencePort('http://controller.invalid','private-token','boot',self.store,clock=self.clock)
        self.port.opener=self.device
        self.pulse=fixtures.pulse('shoulder','up',50,650)
        self.calls=[];original=self.device.open
        def timed(request,timeout):
            path=urllib.parse.urlsplit(request.full_url).path
            payload=None if request.data is None else {k:v[0] for k,v in urllib.parse.parse_qs(request.data.decode()).items()}
            self.calls.append(dict(path=path,payload=payload,startedAt=self.clock.now,timeout=timeout,running=self.device.running))
            return original(request,timeout)
        self.device.open=timed
    def execute(self,goal=lambda status:False):
        return self.port._execute_validated(self.pulse,lambda:None,stop_when=goal,
            monitor_policy=M4_TARGET_UP_DEADLINE_POLICY)
    def RUNs(self):return [c for c in self.calls if c['payload']=={'action':'run'}]
    def STOPs(self):return [c for c in self.calls if c['payload']=={'cmd':'stop'}]

    def test_cap_tail_reserves_STOP_without_new_blocking_GET(self):
        receipt=self.execute();end=receipt['dispatchAfter']+.650
        reads=[c for c in self.calls if c['running'] and c['payload'] is None]
        self.assertTrue(reads)
        self.assertTrue(all(c['startedAt']<end-.2 for c in reads))
        self.assertLessEqual(self.STOPs()[0]['startedAt'],end+.001)
        self.assertTrue(receipt['offVerified']);self.assertFalse(receipt['stoppedEarly'])
        self.assertTrue(receipt['targetUpMonitor']['completeSafetyStatusVerified'])
        self.assertEqual(len(self.RUNs()),1)

    def test_goal_before_deadline_is_early_and_raw_observation_is_separate(self):
        self.device.after_run=lambda device:setattr(device,'angle',196.)
        receipt=self.execute(lambda status:status['shoulderAngle']['angleDeg']>=196.)
        self.assertTrue(receipt['stoppedEarly'])
        observed=receipt['targetUpMonitor']['goalObserved']
        self.assertTrue(observed['reached']);self.assertTrue(observed['beforeDeadline'])
        self.assertLess(observed['evaluatedAt'],receipt['targetUpMonitor']['deadline'])
        self.assertEqual(receipt['monitorTimingPolicy'],M4_TARGET_UP_DEADLINE_POLICY)

    def test_goal_evaluated_after_cap_never_becomes_early(self):
        def goal(status):
            self.clock.now=self.port.last_dispatch_after+.655;return True
        receipt=self.execute(goal)
        self.assertFalse(receipt['stoppedEarly'])
        self.assertTrue(receipt['targetUpMonitor']['goalObserved']['reached'])
        self.assertFalse(receipt['targetUpMonitor']['goalObserved']['beforeDeadline'])

    def test_GET_started_before_end_and_finished_after_end_times_out_then_OFF_no_retry(self):
        original=self.device.open;delayed=[]
        def slow(request,timeout):
            if self.device.running and request.data is None and not delayed:
                delayed.append(self.clock.now);self.clock.sleep(.7)
            return original(request,timeout)
        self.device.open=slow
        with self.assertRaisesRegex(TimeoutError,'request_deadline'):self.execute()
        end=self.port.last_dispatch_after+.65
        self.assertLess(delayed[0],end);self.assertGreater(self.STOPs()[0]['startedAt'],end)
        self.assertEqual(len(self.RUNs()),1);self.assertEqual(len(self.STOPs()),1)
        self.assertTrue(self.port.off_verified);self.assertFalse(any(self.device.speeds.values()))
        self.assertEqual(sum(c['running'] and c['payload'] is None for c in self.calls),1)
        self.assertTrue(any(name.endswith('-late-response.json') for name in self.store.rows))

    def test_read_intent_and_response_persistence_share_absolute_deadline(self):
        for suffix in ('-intent.json','-response.json'):
            with self.subTest(suffix=suffix):
                self.setUp();original=self.store.write_json;delayed=[]
                def slow(name,value):
                    original(name,value)
                    active=(self.device.running and name.endswith(suffix)
                            and (suffix=='-intent.json' or value.get('parsed',{}).get('evidence')=='driver_write'))
                    if active and not delayed:
                        delayed.append(name);self.clock.sleep(.7)
                self.store.write_json=slow
                with self.assertRaisesRegex(TimeoutError,'request_deadline'):self.execute()
                self.assertEqual(len(self.RUNs()),1);self.assertTrue(self.port.off_verified)
                self.assertEqual(len(self.STOPs()),1)
                if suffix=='-intent.json':
                    self.assertFalse(any(c['running'] and c['payload'] is None for c in self.calls))

    def test_TARGET_STOP_intent_is_sent_before_slow_fsync_and_flushed_with_original_time(self):
        original=self.store.write_json;stop_saved=[]
        def slow_stop(name,value):
            if name.endswith('-intent.json') and value.get('payload')=={'cmd':'stop'}:
                stop_saved.append((self.clock.now,self.device.running,value['startedAt']))
                self.assertFalse(self.device.running);self.clock.sleep(.7)
            original(name,value)
        self.store.write_json=slow_stop
        receipt=self.execute();end=receipt['dispatchAfter']+.65
        self.assertLessEqual(self.STOPs()[0]['startedAt'],end+.001)
        self.assertEqual(len(stop_saved),1)
        self.assertGreater(stop_saved[0][0],stop_saved[0][2])
        self.assertEqual(stop_saved[0][2],self.STOPs()[0]['startedAt'])
        self.assertTrue(receipt['offVerified'])

    def test_exact_TARGET_profile_and_goal_callback_are_required_before_any_IO(self):
        for key,value in [('joint','elbow'),('direction','down'),('percent',49),('durationMs',649),('expectedPwm',126),('motor',True)]:
            pulse=dict(self.pulse);pulse[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):validate_monitor_policy(pulse,M4_TARGET_UP_DEADLINE_POLICY)
        with self.assertRaisesRegex(ValueError,'goal_callback_required'):
            self.port._execute_validated(self.pulse,lambda:None,monitor_policy=M4_TARGET_UP_DEADLINE_POLICY)
        with self.assertRaisesRegex(ValueError,'does_not_support_return_goal'):
            self.port._execute_validated(fixtures.pulse('elbow','up',60,350),lambda:None,
                stop_when=lambda status:True,monitor_policy=ELBOW_DEADLINE_POLICY)
        self.assertEqual(self.calls,[])
