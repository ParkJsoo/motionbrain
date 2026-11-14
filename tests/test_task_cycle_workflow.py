"""실제 합성 모델 학습·파일 고정·후보 선택을 전체 유한 경로로 연결한다.

카메라·실행 포트는 가짜이며 실물 성공·실물 제어 적격을 뜻하지 않는다.
"""
import copy
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning import m1_shaft_learning as shaft
from learning import relative_action_learning as relative
from learning import task_action_models as actions
from learning.finite_task_cycle import FiniteTaskCycle, STAGES
from learning.pipeline_learning import canonical_bytes
from learning.task_cycle_workflow import FrozenCycleModelBank, CollectionToCycle, train_cycle_bundle
from tests.test_finite_task_cycle import Ports, Store, scope
from tests.test_m1_shaft_learning import scalar_sample
from tests.test_relative_model_bundle import complete_context
from tests.test_relative_action_learning import row as relative_row

UP=dict(joint='shoulder',direction='up',percent=50,durationMs=650)
DOWN=dict(joint='shoulder',direction='down',percent=35,durationMs=400)
OPEN=dict(joint='gripper',direction='open',percent=60,durationMs=650)
CLOSE=dict(OPEN,direction='close')
APPROACH=dict(joint='elbow',direction='up',percent=60,durationMs=350)
CONTEXT=dict(object='block',physicalView='current',poseRegime='loaded',registrationSha256='a'*64)


def state(rise, angle):
    return dict(objectRisePx=rise,toolRisePx=rise,slipDyPx=0.,startAngleDeg=angle)


def m4_rows():
    result=[]
    for i,(command,support,rise,angle) in enumerate([
        (UP,'table_supported',0.,196.),(UP,'table_supported',2.,197.),
        (DOWN,'gripper_supported',1.,196.5),(DOWN,'gripper_supported',2.,197.)]):
        sign=1 if command==UP else -1
        result.append(dict(id=f'm{i}',context=copy.deepcopy(CONTEXT),supportMode=support,
            command=copy.deepcopy(command),before=state(rise,angle),after=state(rise+sign,angle+.5*sign),
            sourceHash=hashlib.sha256(str(i).encode()).hexdigest(),episode=['boot',str(i)],
            frameIds=[f'm{i}-{camera}-{side}' for camera in ('external','gripper') for side in ('before','after')]))
    return result


def shaft_rows(command, lo, hi, delta):
    rows=[scalar_sample(f'{command["direction"]}-{i}','train',x) for i,x in enumerate((lo,hi))]
    for item in rows:
        item['context']['object']='block';item['command']=copy.deepcopy(command)
        item['afterShaftPx']=item['beforeShaftPx']+delta
    return rows


class ModelPorts(Ports):
    def __init__(self, bank, *, multiple=False):
        planned=scope();self.bank=bank;self.rise=0.
        lift_count=2 if multiple else 1
        templates=[]
        targets=[[-2.,2.,10.],{'shaftSeparationPx':68.},state(float(lift_count),196.+.5*lift_count),
                 state(0.,196.),{'shaftSeparationPx':100.,'expectedM4AngleDeg':196.}]
        for kind,command,target in zip(STAGES,[APPROACH,CLOSE,UP,DOWN,OPEN],targets):
            templates.append(dict(kind=kind,maxCommands=3,commands=[copy.deepcopy(command)],target=target))
        planned['stages']=bank.scope_stages(templates)
        super().__init__(planned)
        if multiple:self.required.update(approach=2,lift=2,lower=2)

    def observe(self, stage, *, after_off, deadline):
        record=super().observe(stage,after_off=after_off,deadline=deadline)
        kind=stage['kind']
        record['state']=([-2.,2.,10.] if self.count[kind]>=self.required[kind] else [2.,2.,10.]) if kind=='approach' else (
            {'shaftSeparationPx':90.} if kind=='close' else
            {'shaftSeparationPx':69.} if kind=='release' else state(self.rise,self.angle))
        if kind in ('lift','lower'):record['supportMode']='table_supported' if kind=='lift' else 'gripper_supported'
        return record

    def execute(self,stage,command,**kwargs):
        result=super().execute(stage,command,**kwargs)
        if stage['kind'] in ('lift','lower'):
            sign=1 if stage['kind']=='lift' else -1
            self.rise+=sign;self.angle+=.5*sign
        return result

    def cycle(self):
        return FiniteTaskCycle(self.scope,observe=self.observe,verify_observation=self.verify,
            choose=self.bank.choose,goal=self.goal,verify_goal=lambda *args:True,
            execute=self.execute,verify_assets=self.bank.verify_cached,check_idle=self.idle,
            stop=self.stop,store=self.store,clock=self.clock)


class CycleWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        rows=[relative_row(i,x,APPROACH) for i,x in enumerate((-4.,0.,4.))]
        context=complete_context()
        for item in rows:item['context']=copy.deepcopy(context)
        self.approach_model=relative.fit(rows)
        self.close_model=shaft.fit(shaft_rows(CLOSE,80.,100.,-20.))
        self.approach=self.binding('approach.json',self.approach_model)
        self.close=self.binding('close.json',self.close_model)
        self.m4=m4_rows();self.open=shaft_rows(OPEN,68.,70.,30.)

    def binding(self,name,model):
        path=self.root/name;raw=canonical_bytes(model);path.write_bytes(raw)
        return dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest())

    def freeze(self, name='bundle', audit=lambda *args:True):
        return train_cycle_bundle(self.root/name,m4_rows=self.m4,open_rows=self.open,
            approach_binding=self.approach,close_binding=self.close,audit_rows=audit)

    def test_actual_models_choose_all_five_stages_without_refit(self):
        bank=self.freeze();ports=ModelPorts(bank)
        originals={stage:Path(binding['path']).read_bytes() for stage,binding in bank.bindings.items()}
        with patch.object(actions,'fit_m4',side_effect=AssertionError('refit')), \
             patch.object(shaft,'fit',side_effect=AssertionError('refit')), \
             patch.object(relative,'fit',side_effect=AssertionError('refit')):
            result=ports.cycle().run()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(result['confirmedCommands'],5)
        self.assertTrue(result['fullyModelSelectedCycle'])
        self.assertEqual([c['direction'] for c in ports.sent],['up','close','up','down','open'])
        self.assertTrue(all(Path(bank.bindings[k]['path']).read_bytes()==v for k,v in originals.items()))

    def test_multiple_reobserved_model_corrections_complete(self):
        ports=ModelPorts(self.freeze(),multiple=True)
        result=ports.cycle().run()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(result['confirmedCommands'],8)
        self.assertEqual(ports.count['lift'],2);self.assertEqual(ports.count['lower'],2)
        self.assertGreater(ports.n,16)

    def test_unsupported_before_stops_without_manual_substitute(self):
        ports=ModelPorts(self.freeze());base=ports.observe
        def observe(stage,**kwargs):
            record=base(stage,**kwargs)
            if stage['kind']=='lift':record['state']['objectRisePx']=99.
            return record
        ports.observe=observe
        result=ports.cycle().run()
        self.assertEqual(result['status'],'failed');self.assertEqual(len(ports.sent),2)
        self.assertIn('abstained',str(result))

    def test_source_audit_rejects_before_freeze(self):
        with self.assertRaisesRegex(ValueError,'source_audit'):
            self.freeze(audit=lambda *args:False)
        self.assertFalse((self.root/'bundle').exists())

    def test_bank_rejects_file_or_cached_model_change(self):
        bank=self.freeze();bank.models['lift']['models'][0]['maximum'][0]+=10
        with self.assertRaisesRegex(ValueError,'cached_model_changed'):bank.verify_cached()
        bindings=copy.deepcopy(bank.bindings);Path(bindings['release']['path']).write_bytes(b'{}')
        with self.assertRaisesRegex(ValueError,'file_changed'):FrozenCycleModelBank(bindings)

    def test_no_open_supported_candidate_is_not_a_ready_bundle(self):
        self.open=self.open[:1]
        with self.assertRaisesRegex(ValueError,'OPEN_model_without_supported_candidate'):self.freeze()
        self.assertFalse((self.root/'bundle').exists())

    def test_collection_audit_freeze_then_one_model_cycle(self):
        events=[];ports=[]
        requests=[dict(id='new-open',stage='release',command=OPEN)]
        seed=self.m4+self.open[:1]
        def collect(request):events.append('collect');return dict(status='completed',offVerified=True)
        def audit(request,result):events.append('audit');return copy.deepcopy(self.open[1])
        def freeze(rows):
            events.append('freeze')
            return train_cycle_bundle(self.root/'from-collection',m4_rows=[r for r in rows if r['command']['joint']=='shoulder'],
                open_rows=[r for r in rows if r['command']['joint']=='gripper'],approach_binding=self.approach,
                close_binding=self.close,audit_rows=lambda *args:True)
        def make_cycle(bank):events.append('cycle');ports.append(ModelPorts(bank));return ports[-1].cycle()
        workflow=CollectionToCycle(requests,seed_rows=seed,collect=collect,audit=audit,freeze=freeze,
            make_cycle=make_cycle,store=Store())
        result=workflow.run()
        self.assertEqual(result['status'],'completed',result);self.assertEqual(events,['collect','audit','freeze','cycle'])
        self.assertEqual(result['manualCollectionAttempts'],1);self.assertEqual(result['modelCycleAttempts'],1)
        with self.assertRaisesRegex(ValueError,'single_use'):workflow.run()

    def test_missing_lower_candidate_stops_before_bundle_output(self):
        self.m4=self.m4[:2]
        with self.assertRaisesRegex(ValueError,'M4_missing_lower'):self.freeze()
        self.assertFalse((self.root/'bundle').exists())

    def test_empty_collection_reuses_audited_seed_rows_once(self):
        bank=self.freeze()
        ports=ModelPorts(bank);events=[]
        workflow=CollectionToCycle([],seed_rows=self.m4+self.open,
            collect=lambda request:events.append('collect'),audit=lambda *args:events.append('audit'),
            freeze=lambda rows:bank,make_cycle=lambda frozen:ports.cycle(),store=Store())
        result=workflow.run()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(result['manualCollectionAttempts'],0);self.assertEqual(result['modelCycleAttempts'],1)
        self.assertEqual(events,[])

    def test_failed_collection_never_trains_or_runs(self):
        called=[]
        workflow=CollectionToCycle([dict(id='new-open',stage='release',command=OPEN)],seed_rows=[],
            collect=lambda request:dict(status='unknown_output',offVerified=False),audit=lambda *args:called.append('audit'),
            freeze=lambda *args:called.append('freeze'),make_cycle=lambda *args:called.append('cycle'),store=Store())
        result=workflow.run()
        self.assertEqual(result['status'],'failed');self.assertEqual(result['manualCollectionAttempts'],1)
        self.assertEqual(result['modelCycleAttempts'],0);self.assertEqual(called,[])


class ScopedExecutorTests(unittest.TestCase):
    def setUp(self):
        from learning.task_cycle_workflow import ScopedCycleExecutor
        from learning.m4_up_commissioning import M4UpCommissioning, CONTACT_MANUAL650_SCHEMA
        from tests.test_m4_up_commissioning import Device
        from tests.test_supervised_transport import Clock
        self.events=[];self.run_calls=0;self.actor=None;self.device=Device(Clock())
        self.device.angle=196.;self.mode=None
        self.safety=dict(M4ReferenceDeg=196.,maximumM4ExcursionDeg=3.,controllerBootId=self.device.boot)
        def factory(stage,command,hooks):
            actor=object.__new__(M4UpCommissioning)
            actor.task_hooks=hooks
            actor.scope=dict(controllerBootId=self.device.boot,pulse=dict(command),schemaVersion=CONTACT_MANUAL650_SCHEMA)
            if self.mode=='boot':actor.scope['controllerBootId']='f'*32
            if self.mode=='pulse':actor.scope['pulse']['durationMs']-=1
            if self.mode=='hooks':actor.task_hooks=dict(hooks)
            def run(*,confirmed_scope_sha256):
                self.assertEqual(confirmed_scope_sha256,actor.digest)
                self.run_calls+=1
                hooks['status_guard'](self.device.status())
                hooks['prepare']();hooks['before_run']()
                return dict(status='unknown_output' if self.mode=='failed' else 'completed',
                            finalIdleOffVerified=self.mode!='failed',errors=[],receipt={'testReceipt':True})
            actor.digest=hashlib.sha256(canonical_bytes(actor.scope)).hexdigest()
            actor.run=run;self.actor=actor
            return actor,('f'*64 if self.mode=='approval' else actor.digest)
        def other(command,**kwargs):
            self.events.append(('other',command))
            kwargs['after_armed_before_run']();kwargs['before_run']()
            return {'otherReceipt':True}
        self.executor=ScopedCycleExecutor(factory,other)

    def call(self,stage='lift',command=UP):
        return self.executor({'kind':stage},command,prepare=lambda:self.events.append('prepare'),
            before_run=lambda:self.events.append('before_run'),decision={'model':'frozen'},safety=self.safety)

    def test_m4_scoped_actor_receipt_and_callbacks_are_bound(self):
        result=self.call()
        self.assertEqual(result,{'testReceipt':True});self.assertEqual(self.run_calls,1)
        self.assertEqual(self.events,['prepare','before_run'])

    def test_wrong_scope_boot_pulse_or_hook_identity_never_runs(self):
        for mode in ('boot','pulse','hooks'):
            self.mode=mode
            with self.subTest(mode=mode),self.assertRaisesRegex(ValueError,'scoped_actor_mismatch'):self.call()
            self.assertEqual(self.run_calls,0)

    def test_unapproved_scope_never_runs(self):
        self.mode='approval'
        with self.assertRaisesRegex(ValueError,'scope_not_confirmed'):self.call()
        self.assertEqual(self.run_calls,0)

    def test_scoped_failure_is_not_retried(self):
        self.mode='failed'
        with self.assertRaisesRegex(ValueError,'actor_failed_no_retry'):self.call()
        self.assertEqual(self.run_calls,1)

    def test_parent_total_excursion_checked_during_actor_status(self):
        self.device.angle=199.01
        with self.assertRaisesRegex(ValueError,'active_M4_total_excursion'):self.call()
        self.assertEqual(self.run_calls,1);self.assertEqual(self.events,[])

    def test_other_joint_receives_existing_late_prepare_bridge(self):
        result=self.call('release',OPEN)
        self.assertEqual(result,{'otherReceipt':True});self.assertEqual(self.run_calls,0)
        self.assertEqual(self.events[1:],['prepare','before_run'])


class RealScopedHookTests(unittest.TestCase):
    def setUp(self):
        from tests import test_m4_up_commissioning as fixtures
        self.fixture=fixtures.CommissioningTests()
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)

    def test_after_arm_before_capture_is_single_use_and_parent_bound(self):
        fixture=self.fixture;original=fixture.observer.observe;labels=set();events=[]
        def strict_observe(*,after_off,**kwargs):
            label='before' if after_off is None else 'after'
            if label in labels:raise ValueError('strict_camera_no_retake')
            labels.add(label)
            if label=='before':self.assertEqual(fixture.device.state,'ARMED')
            return original(after_off=after_off,**kwargs)
        fixture.observer.observe=strict_observe
        fixture.trial.task_hooks=dict(prepare=lambda:events.append('prepare'),before_run=lambda:events.append('before_run'),
            status_guard=lambda status:None,decisionSha256='c'*64)
        result=fixture.run_trial()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(labels,{'before','after'});self.assertEqual(events,['prepare','before_run'])
        self.assertTrue(result['parentModelSelected']);self.assertEqual(result['parentModelDecisionSha256'],'c'*64)
        self.assertEqual(sum(payload=={'action':'run'} for _,payload in fixture.posts()),1)

    def test_parent_active_guard_failure_stops_after_one_run(self):
        fixture=self.fixture
        def monitor(status):
            if fixture.trial.port.may_have_run:raise ValueError('test_parent_active_guard')
        fixture.trial.task_hooks=dict(prepare=lambda:None,before_run=lambda:None,status_guard=monitor,
                                     decisionSha256='c'*64)
        result=fixture.run_trial()
        self.assertEqual(result['status'],'failed',result)
        self.assertEqual(sum(payload=={'action':'run'} for _,payload in fixture.posts()),1)
        self.assertTrue(result['finalIdleOffVerified']);self.assertTrue(result['parentModelSelected'])


if __name__=='__main__':unittest.main()
