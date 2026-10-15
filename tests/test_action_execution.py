"""Host journal + bounded API contract tests. No real devices or motor outputs."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.action_execution import ActionLimits, BoundedActionExecutor, canonical_request
from learning.dry_run import SyntheticAdapter
from learning.artifacts import ArtifactStore
from learning.runner import CaptureRunner, audit_run


class FakeClock:
    def __init__(self):
        self.now = 100.0
    def monotonic(self):
        return self.now
    def sleep(self, seconds):
        self.now += seconds


class FakePort:
    host_clock_id = "test-clock"
    def __init__(self, clock):
        self.clock = clock
        self.posts, self.cancels, self.reads = [], [], 0
        self.failure = None
        self.running_reads = 0
        self.last_record = None
        self.before_submit = None
        self.first_sequence = 1

    def preflight(self, request):
        if self.failure == "preflight":raise ValueError("disabled")
        return self.log(request["bootId"], [])

    @staticmethod
    def log(boot, entries):
        return dict(bootId=boot, clockDomain="controller_boot_millis", evidence="driver_write", dropped=0, entries=entries)

    def submit(self, request):
        if self.before_submit:self.before_submit()
        self.posts.append(copy.deepcopy(request))
        if self.failure == "submit":raise TimeoutError("response lost")
        if self.failure == "interrupt":raise KeyboardInterrupt()
        record = {k:v for k,v in request.items() if k!='pwm'}
        record.update(requestedPwm=request['pwm'], ordinal=1, state="running", reason="none",
                      startedAtMs=100000, endedAtMs=0, elapsedMs=0, overrunMs=0, outputOff=False, targetReached=False)
        self.last_record=record
        return dict(accepted=True, code="duplicate" if self.failure=="duplicate" else "accepted", record=copy.deepcopy(record))

    def lookup(self, request):
        self.reads+=1
        if self.failure=="lookup":raise OSError("disconnected")
        if self.running_reads>0:
            self.running_reads-=1
            return copy.deepcopy(self.last_record)
        if self.last_record is None:
            raise ValueError("not_found")
        duration=request['durationMs']
        self.last_record.update(state="completed",reason="duration_elapsed",endedAtMs=100000+duration,
                                elapsedMs=duration,outputOff=True)
        if self.failure=="wrong_id":self.last_record['commandId']='other'
        if self.failure=="bad_terminal":self.last_record.update(elapsedMs=0,endedAtMs=100000)
        if self.failure=="wrong_ordinal":self.last_record['ordinal']=2
        if self.failure=="no_off":self.last_record['outputOff']=False
        return copy.deepcopy(self.last_record)

    def outputs(self):
        r=self.last_record
        entries=[dict(sequence=self.first_sequence, atMs=r['startedAtMs'], actionOrdinal=1,
                      motor=r['motor'],pwm=r['requestedPwm'],reason='driver_write'),
                 dict(sequence=self.first_sequence+1, atMs=r['endedAtMs'], actionOrdinal=1,
                      motor=r['motor'],pwm=0,reason='forced_off')]
        result=self.log(r['bootId'],entries)
        if self.failure=='trace_reverse':entries[0]['atMs'],entries[1]['atMs']=entries[1]['atMs'],entries[0]['atMs']
        if self.failure=='trace_bool':
            for e in entries:e['actionOrdinal']=True
        if self.failure=='dropped':result['dropped']=1
        if self.failure=='trace_gap':entries[-1]['sequence']+=1
        if self.failure=='trace_start_gap':
            for e in entries:e['sequence']+=1
        if self.failure=='other_motor':entries[0]['motor']=2
        if self.failure=='output_boot':result['bootId']='b'*32
        return result

    def cancel(self,request):
        self.cancels.append(copy.deepcopy(request))
        if self.last_record is None:raise OSError('not found')
        self.last_record.update(state='cancelled',reason='cancelled',outputOff=True,elapsedMs=1,endedAtMs=100001)
        return copy.deepcopy(self.last_record)


def limits():
    return ActionLimits(motor=1,max_duration_ms=100,max_pwm=30,max_percent=0,min_degrees=0,max_degrees=0)

def request(cid='first'):
    return dict(bootId='a'*32,commandId=cid,kind='pulse',motor=1,pwm=30,durationMs=100,targetDegrees=0,percent=0)


class ActionExecutionTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.clock=FakeClock();self.port=FakePort(self.clock)
        self.core=self.make()
    def make(self,**kwargs):
        core=BoundedActionExecutor(self.root/'actions.sqlite',self.port,limits(),clock=self.clock,**kwargs)
        self.addCleanup(core.close);return core
    def run_action(self,r=None,guard=lambda r:None):
        r=r or request();self.core.propose(r)
        return self.core.confirm(r['commandId'],confirmed_request=r,dispatch_guard=guard)
    def test_terminal_record_trace_and_causal_bounds(self):
        result=self.run_action()
        self.assertEqual(result['executionState'],'known_terminal',result)
        self.assertEqual(len(self.port.posts),1);self.assertEqual(self.port.cancels,[])
        self.assertFalse(result['trainingEligible']);self.assertFalse(result['physicalArrivalVerified'])
        action=result['action'];self.assertEqual(action['outputSequenceBounds']['first'],1)
        self.assertEqual(action['outputSequenceBounds']['last'],2)
        self.assertEqual(action['startHostBounds']['clockId'],self.port.host_clock_id)
    def test_replay_and_changed_payload_never_post_twice(self):
        self.run_action();self.assertTrue(self.core.confirm('first',confirmed_request=request(),dispatch_guard=lambda r:None)['replayed'])
        self.core.propose(request())
        with self.assertRaises(ValueError):self.core.propose({**request(),'pwm':29})
        self.assertEqual(len(self.port.posts),1)
    def test_confirmation_exact_match_and_guard_required(self):
        for confirmed,guard in [({**request(),'pwm':29},lambda r:None),(request(),None)]:
            with self.subTest(confirmed=confirmed):
                r=request(str(len(self.core.journal.records())))
                self.core.propose(r)
                result=self.core.confirm(r['commandId'],confirmed_request=confirmed,dispatch_guard=guard)
                self.assertEqual(result['executionState'],'not_sent')
        self.assertFalse(self.port.posts)
    def test_expiry_is_not_refreshed_by_repeat_proposal(self):
        self.core.propose(request());self.clock.now+=19;self.core.propose(request());self.clock.now+=1
        result=self.core.confirm('first',confirmed_request=request(),dispatch_guard=lambda r:None)
        self.assertEqual(result['executionState'],'not_sent');self.assertFalse(self.port.posts)
    def test_guard_runs_after_durable_reservation_and_can_refuse(self):
        def guard(r):
            self.assertEqual(self.core.journal.get(r['commandId'])['phase'],'dispatching')
            raise ValueError('stale frame or M4; never weaken 100ms/15s/250ms')
        result=self.run_action(guard=guard)
        self.assertEqual(result['executionState'],'not_sent');self.assertFalse(self.port.posts)
    def test_guard_time_counts_and_clock_regression_rejected(self):
        for delta in [21,-1]:
            r=request(str(delta))
            result=self.run_action(r,guard=lambda r:self.clock.sleep(delta))
            self.assertEqual(result['executionState'],'not_sent')
        self.assertFalse(self.port.posts)
    def test_failures_latch_unknown_and_do_not_retry(self):
        for failure in ['submit','lookup','duplicate','wrong_id','wrong_ordinal','bad_terminal','no_off','dropped','trace_gap','trace_start_gap','trace_reverse','trace_bool','other_motor','output_boot']:
            with self.subTest(failure=failure):
                self.core.close();path=self.root/failure;path.mkdir()
                self.port=FakePort(self.clock);self.port.failure=failure
                self.core=BoundedActionExecutor(path/'actions.sqlite',self.port,limits(),clock=self.clock)
                self.addCleanup(self.core.close)
                result=self.run_action();self.assertEqual(result['executionState'],'outcome_unknown',result)
                self.core.propose(request('second'))
                result=self.core.confirm('second',confirmed_request=request('second'),dispatch_guard=lambda r:None)
                self.assertEqual(result['reason'],'unresolved_outcome_unknown');self.assertEqual(len(self.port.posts),1)
                self.assertEqual(len(self.port.cancels),1)
                self.core.cancel('first');self.assertEqual(len(self.port.cancels),1)
    def test_poll_budget_stops_and_cancels_once(self):
        self.port.running_reads=100
        result=self.run_action();self.assertEqual(result['executionState'],'outcome_unknown')
        self.assertEqual(self.port.reads,20);self.assertEqual(len(self.port.cancels),1)
    def test_stop_signal_cancels_without_needing_executor_lock(self):
        self.port.before_submit=self.core.request_stop
        result=self.run_action();self.assertEqual(result['executionState'],'outcome_unknown')
        self.assertEqual(len(self.port.cancels),1)
    def test_restart_never_replays_pending_or_reserved_dispatch(self):
        self.core.propose(request());self.core.close();self.core=self.make()
        self.assertEqual(self.core.result('first')['executionState'],'not_sent')
        r=request('second');self.core.propose(r)
        self.core.journal.save('second','dispatching',self.core._result('second','outcome_unknown','reserved',True))
        self.core.close();self.core=self.make()
        self.assertEqual(self.core.result('second')['executionState'],'outcome_unknown')
        self.core.confirm('second',confirmed_request=r,dispatch_guard=lambda r:None)
        self.assertFalse(self.port.posts)
    def test_inspection_does_not_unlock_unknown(self):
        self.port.failure='lookup';self.run_action();self.port.failure=None
        result=self.core.inspect('first');self.assertFalse(result['journalChanged'])
        self.assertEqual(self.core.result('first')['executionState'],'outcome_unknown')
    def test_journal_failure_before_submit_prevents_transport(self):
        with patch.object(self.core.journal,'save',side_effect=OSError('full')):
            with self.assertRaises(OSError):self.run_action()
        self.assertFalse(self.port.posts)
        self.assertEqual(self.core.confirm('first',confirmed_request=request(),dispatch_guard=lambda r:None)['executionState'],'outcome_unknown')
    def test_journal_lock_prevents_second_owner(self):
        with self.assertRaises(ValueError):self.make()
    def test_keyboard_interrupt_is_cancelled_and_preserves_unknown(self):
        self.port.failure='interrupt'
        with self.assertRaises(KeyboardInterrupt):self.run_action()
        self.assertEqual(self.core.result('first')['executionState'],'outcome_unknown')
        self.assertEqual(len(self.port.cancels),1)
    def test_invalid_request_types_and_bounds(self):
        for field,value in [('motor',True),('pwm',0),('durationMs',0),('durationMs',10001),('targetDegrees',float('nan')),('commandId','unsafe id'),('bootId','bad'),('kind','arm')]:
            with self.subTest(field=field), self.assertRaises(ValueError):canonical_request({**request(),field:value})
        with self.assertRaises(ValueError):self.core.propose({**request(),'pwm':31})
        r={**request(), 'motor':4,'kind':'shoulder','pwm':0,'targetDegrees':234.1,'percent':75}
        with self.assertRaisesRegex(ValueError,'echoable'):canonical_request(r)
        canonical_request({**r,'targetDegrees':234.25})


class JournaledCollectorIntegrationTest(unittest.TestCase):
    def test_synthetic_before_action_after_uses_durable_executor_and_stays_excluded(self):
        from learning.action_capture import JournaledSyntheticCaptureAdapter
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=SyntheticAdapter()
            source.clock.sleep=source.clock.advance
            class Port(FakePort):
                synthetic=True
                def preflight(self,r):return self.log(r['bootId'],[])
                def submit(self,r):
                    self.posts.append(r)
                    self.action=source.execute(r)
                    self.last_record=self.action['record']
                    return dict(accepted=True,code='accepted',record=self.last_record)
                def outputs(self):return self.action['outputLog']
            port=Port(source.clock);port.host_clock_id=source.host_clock_id
            core=BoundedActionExecutor(root/'journal.sqlite',port,limits(),clock=source.clock)
            self.addCleanup(core.close)
            before={};holder={}
            def guard(r):holder['runner']._check_before(before['observation'])
            adapter=JournaledSyntheticCaptureAdapter(source,core,dispatch_guard=guard)
            original=adapter.observe
            def observe(phase):
                value=original(phase)
                if phase=='before':before['observation']=value
                return value
            adapter.observe=observe
            runner=CaptureRunner(ArtifactStore(root/'run'),adapter);holder['runner']=runner
            result=runner.capture();runner.close()
            self.assertEqual(result['status'],'completed',result)
            self.assertEqual(result['executionState'],'known_terminal')
            self.assertFalse(result['trainingEligible'])
            self.assertEqual(result['validation']['exclusionReasons'],['synthetic_dry_run'])
            self.assertTrue(audit_run(root/'run')['ok'])
            self.assertEqual(len(port.posts),1)
    def test_http_port_cannot_be_hidden_as_synthetic_collector(self):
        from learning.action_capture import JournaledSyntheticCaptureAdapter
        from types import SimpleNamespace
        with self.assertRaises(ValueError):
            JournaledSyntheticCaptureAdapter(SyntheticAdapter(),SimpleNamespace(port=SimpleNamespace()),dispatch_guard=lambda r:None)


if __name__=='__main__':unittest.main()
