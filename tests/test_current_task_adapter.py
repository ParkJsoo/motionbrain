"""현장 결합의 미지원 입구·단발 훅·분모와 기존 유한 코어 연결 검증."""
import copy
import unittest

from learning.current_task_adapter import CurrentTaskAdapter, ManualCollectionCyclePort, SCHEMA
from learning.manual_collection import ManualCollection
from learning.supervised_transport import ELBOW_DEADLINE_POLICY
from unittest.mock import Mock
from tests import test_task_cycle_workflow as fixtures
ModelPorts, UP = fixtures.ModelPorts, fixtures.UP


class Clock:
    def __init__(self):self.value=10.
    def monotonic(self):return self.value


class Store:
    def __init__(self):self.values={}
    def write_json(self,name,value):
        if name in self.values:raise ValueError('consumed')
        self.values[name]=copy.deepcopy(value)


class CurrentTaskAdapterTests(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.CycleWorkflowTests();self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.bank=self.fixture.freeze();self.clock=Clock();self.store=Store();self.index=0
        self.scope=dict(schemaVersion=SCHEMA,sessionId='current-session',controllerBootId='a'*32,
            cameraBootId='b'*32,object='block',M4ReferenceDeg=196.,maxObservationAgeSeconds=3.,
            budgets=dict(preparation=1,M1=1,M3=1,M4_OPEN=1),maxRequestedMs=2600,
            contexts={kind:copy.deepcopy(model['context']) for kind,model in self.bank.models.items()})
        self.requests=[dict(id='lift-1',category='M4_OPEN',stage='lift',command=copy.deepcopy(UP))]
        self.collect_calls=0
    def inspect(self,request):
        self.index+=1
        return dict(controllerBootId='a'*32,cameraBootId='b'*32,m4AngleDeg=196.,
            context=copy.deepcopy(self.scope['contexts'][request['stage']]),
            frames=[dict(camera=c,frameId=f'{c}-{self.index}',pixelSha256='c'*64,
                         sourceEarliest=10.,sourceLatest=10.,persistedAt=10.)
                    for c in ('external','gripper')])
    def collect(self,request,*,prepare,before_run):
        self.collect_calls+=1;prepare();before_run()
        return dict(status='completed',offVerified=True)
    def adapter(self,**overrides):
        args=dict(inspect=self.inspect,audit_entry=lambda *args:True,collect=self.collect,
            audit_row=lambda req,res:dict(command=req['command'],context=self.scope['contexts'][req['stage']]),
            freeze=lambda rows:self.bank,make_cycle=lambda bank:ModelPorts(bank).cycle(),
            verify_assets=lambda:None,check_idle=lambda:'a'*32,
            check_armed=lambda request:dict(bootId='a'*32,state='ARMED',queueCount=1,allFiveOff=True,evidenceSha256='d'*64),
            store=self.store,clock=self.clock)
        args.update(overrides)
        # Existing test cycle has its own fake current boot; bind expected adapter boot to it.
        old_make=args['make_cycle']
        def cycle(bank):
            result=old_make(bank);result.scope['controllerBootId']='a'*32
            from learning.supervised_collection import plan_digest
            result.scope_sha=plan_digest(result.scope)
            # Ports idle method captured by cycle belongs to the old scope object.
            result.check_idle=lambda:'a'*32
            original_observe=result.observe
            def observe(*a,**kw):
                record=original_observe(*a,**kw);record['controllerBootId']='a'*32;return record
            result.observe=observe
            return result
        args['make_cycle']=cycle
        return CurrentTaskAdapter(self.scope,self.requests,**args)
    def test_collect_then_existing_frozen_finite_cycle_once(self):
        adapter=self.adapter();result=adapter.run()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(result['collectionAttempts']['M4_OPEN'],1)
        self.assertEqual(result['collectionCompleted']['M4_OPEN'],1)
        self.assertEqual(result['modelCycleAttempts'],1)
        self.assertEqual(result['modelCycleResult']['confirmedCommands'],5)
        with self.assertRaisesRegex(ValueError,'single_use'):adapter.run()
    def test_current_boot_or_registration_rejection_is_zero_commands(self):
        for key,override in [('check_idle',lambda:'d'*32),('audit_entry',lambda *args:False)]:
            self.store=Store();self.collect_calls=0
            result=self.adapter(**{key:override}).run()
            self.assertEqual(self.collect_calls,0)
            self.assertEqual(sum(result['collectionAttempts'].values()),0)
            self.assertEqual(result['modelCycleAttempts'],0)
    def test_ambiguous_collection_consumed_no_freeze_no_retry(self):
        def collect(request,**hooks):
            self.collect_calls+=1;hooks['prepare']();hooks['before_run']()
            return dict(status='unknown_output',offVerified=False)
        result=self.adapter(collect=collect,freeze=lambda rows:self.fail('freeze after unknown')).run()
        self.assertEqual(self.collect_calls,1)
        self.assertEqual(result['collectionAttempts']['M4_OPEN'],1)
        self.assertEqual(result['modelCycleAttempts'],0)
        self.assertIn('failed_no_retry',result['errors'][0]['reason'])
    def test_stale_after_arm_does_not_dispatch(self):
        dispatched=[]
        def collect(request,**hooks):
            hooks['prepare']();self.clock.value+=3.1;hooks['before_run']();dispatched.append(True)
        result=self.adapter(collect=collect).run()
        self.assertEqual(dispatched,[])
        self.assertIn('stale',result['errors'][0]['reason'])
        self.assertEqual(result['modelCycleAttempts'],0)
    def test_duplicate_prepare_rejected(self):
        def collect(request,**hooks):hooks['prepare']();hooks['prepare']()
        result=self.adapter(collect=collect).run()
        self.assertIn('prepare_repeated',result['errors'][0]['reason'])
    def test_budget_and_foreign_object_rejected_in_constructor(self):
        self.scope['budgets']['M4_OPEN']=0
        with self.assertRaisesRegex(ValueError,'budget'):self.adapter()
        self.scope['budgets']['M4_OPEN']=1
        self.scope['contexts']['close']['object']='foreign'
        with self.assertRaisesRegex(ValueError,'context_object'):self.adapter()

    def test_prepare_checks_armed_not_idle_and_rejects_wrong_queue_or_output(self):
        state=['IDLE'];idle_calls=[]
        def idle():
            idle_calls.append(state[0]);self.assertEqual(state[0],'IDLE');return 'a'*32
        def collect(request,**hooks):
            state[0]='ARMED';hooks['prepare']();hooks['before_run']()
            return dict(status='completed',offVerified=True)
        result=self.adapter(check_idle=idle,collect=collect).run()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(idle_calls,['IDLE'])
        for field,value in [('state','IDLE'),('queueCount',0),('queueCount',2),('allFiveOff',False)]:
            self.store=Store();self.index=0
            def bad(request):
                p=dict(bootId='a'*32,state='ARMED',queueCount=1,allFiveOff=True,evidenceSha256='d'*64)
                p[field]=value;return p
            result=self.adapter(check_armed=bad).run()
            self.assertEqual(result['modelCycleAttempts'],0)
            self.assertIn('armed_or_off',result['errors'][0]['reason'])

    def test_general_port_preserves_M3_deadline_policy_and_armed_hooks(self):
        manager=Mock(spec=ManualCollection)
        manager.current=dict(phase='recording',events=[]);manager.command_count=0
        seen=[]
        def pulse(command,**kwargs):
            self.assertEqual(kwargs['monitor_policy'],ELBOW_DEADLINE_POLICY)
            kwargs['after_armed_before_run']();kwargs['before_run']()
            manager.command_count+=1
            manager.current['events'].append(dict(kind='result',receipt=dict(offVerified=True)))
        manager.pulse.side_effect=pulse
        port=ManualCollectionCyclePort(lambda command:manager)
        result=port(dict(joint='elbow',direction='up',percent=60,durationMs=350),
            after_armed_before_run=lambda:seen.append('armed'),before_run=lambda:seen.append('run'),decision={})
        self.assertTrue(result['offVerified']);self.assertEqual(seen,['armed','run'])

    def test_armed_port_uses_driver_and_single_queue_proof(self):
        manager=Mock(spec=ManualCollection);manager.port=Mock()
        p=manager.port;p.boot_id='a'*32
        p._call.return_value=dict(bootId='a'*32,active=False)
        p._status.return_value=dict(state='ARMED')
        p._outputs.return_value=(dict(entries=[]),[])
        p.last_outputs={i:0 for i in range(1,6)}
        p._queue.return_value=dict(sequence=dict(state='IDLE',totalCount=1))
        port=ManualCollectionCyclePort(lambda command:manager)
        result=port.check_armed(self.requests[0])
        self.assertEqual(result['state'],'ARMED');self.assertEqual(result['queueCount'],1)
        p._call.assert_called_once_with('/actions')
        p._check_status.assert_called_once_with(p._status.return_value,'ARMED')
        p._queue.assert_called_once_with(1)
        p.last_outputs[2]=1
        with self.assertRaisesRegex(ValueError,'driver_not_off'):port.check_armed(self.requests[0])

    def test_RUN_guard_uses_cached_asset_proof_without_IO(self):
        io_calls=[]
        def verify_assets():
            io_calls.append(True);return dict(sourceSha256='f'*64)
        def collect(request,**hooks):
            hooks['prepare']();before=len(io_calls)
            hooks['before_run']()
            self.assertEqual(len(io_calls),before)
            return dict(status='completed',offVerified=True)
        result=self.adapter(verify_assets=verify_assets,collect=collect).run()
        self.assertEqual(result['status'],'completed',result)
        self.assertGreater(len(io_calls),0)

    def test_RUN_guard_rejects_changed_memory_scope_without_IO(self):
        io_calls=[];holder=[]
        def collect(request,**hooks):
            hooks['prepare']();before=len(io_calls)
            holder[0].scope['M4ReferenceDeg']=197.
            with self.assertRaisesRegex(ValueError,'scope_changed'):hooks['before_run']()
            self.assertEqual(len(io_calls),before)
            raise ValueError('scope_changed')
        adapter=self.adapter(verify_assets=lambda:io_calls.append(True),collect=collect)
        holder.append(adapter);result=adapter.run()
        self.assertEqual(result['modelCycleAttempts'],0)
