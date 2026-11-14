"""실제 ManualCollection/SequencePort와 M4 actor를 HTTP 모형에 연결한다.

원본 카메라와 phase 판독만 합성이다. 실제 장치·네트워크·촬영은 없다.
"""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from learning.current_cycle_runtime import CurrentCycleRuntime,CycleReadOnlyPort
from learning.manual_collection import ManualCollection,RecordingPort
from learning.m4_up_commissioning import M4UpCommissioning,CONTACT_MANUAL650_SCHEMA,RETURN_SCHEMA,LEARNING_PULSE,RETURN_PULSE
from learning import m4_up_commissioning as m4
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort
from learning.artifacts import ArtifactStore
from tests import test_task_cycle_workflow as models
from tests.test_manual_collection import Camera
from tests.test_m4_up_commissioning import Device,Observer,scope as actor_scope
from tests.test_supervised_transport import Clock


class ClosedCamera(Camera):
    def close(self):
        self.closed=True
        return super().close()


class Source:
    def __init__(self,test):self.t=test;self.routes=[];self.closed=False
    def preflight(self,scope,assets):
        return dict(scopeSha256=plan_digest(scope),assetsSha256=plan_digest(assets),supportedStages=list(models.STAGES))
    def _record(self,stage,after_off,deadline):
        record=self.t.ports.observe(stage,after_off=after_off,deadline=deadline)
        record['controllerBootId']=self.t.device.boot;record['m4AngleDeg']=self.t.device.angle
        if stage['kind'] in ('lift','lower'):
            record['state']['startAngleDeg']=record['m4AngleDeg']
        return record
    def observe_idle(self,stage,*,after_off,deadline):
        self.t.assertIsNone(self.t.runtime.manager);self.t.assertIsNone(self.t.runtime.actor)
        self.routes.append('idle');return self._record(stage,after_off,deadline)
    def observe_general_armed(self,stage,manager,*,after_off,deadline):
        self.t.assertEqual(self.t.device.state,'ARMED');self.t.assertTrue(manager.busy)
        self.t.assertIs(manager.camera,self.t.managers[-1].camera)
        self.routes.append('general_armed');return self._record(stage,after_off,deadline)
    def observe_m4_armed(self,stage,actor,*,after_off,deadline):
        self.t.assertEqual(self.t.device.state,'ARMED');self.t.assertTrue(actor.observer.started)
        self.t.assertIsNone(self.t.runtime.manager);self.routes.append('M4_armed')
        record=self._record(stage,after_off,deadline)
        frames=dict(external=m4._external_frame(actor.root,actor.before['external'],actor.scope),
                    gripper=m4._gripper_frame(actor.root,actor.before['gripper'],actor.scope,'before'))
        record['frames']=[dict(camera=c,**{k:f[k] for k in ('frameId','pixelSha256','sourceEarliest','sourceLatest','persistedAt')}) for c,f in frames.items()]
        return record
    def observe_general_after(self,stage,manager,receipt,*,deadline):
        self.t.assertFalse(manager.busy);self.t.assertEqual(self.t.device.state,'IDLE')
        self.t.assertGreaterEqual(self.t.clock.now,receipt['offAt']+3.)
        self.routes.append('general_after');return self._record(stage,receipt['offAt'],deadline)
    def verify_observation(self,stage,record):return not self.t.reject_observation
    def verify_clock_bridge(self,stage,record,route,owner):
        if route=='general_armed':self.t.assertIs(owner,self.t.runtime.manager)
        if route=='M4_armed':self.t.assertIs(owner,self.t.runtime.actor)
        return not self.t.reject_bridge
    def validate_general_memory(self,stage,manager,command):
        self.t.assertEqual(self.t.device.state,'ARMED');self.t.memory_guards+=1
        if self.t.reject_RUN_guard:raise ValueError('test_memory_guard_rejected')
    def close_idle(self):self.routes.append('close_idle')
    def close_general(self,manager):self.routes.append('close_general')
    def close_m4(self,actor):self.routes.append('close_m4');self.t.assertTrue(actor.observer.closed)
    def close(self):self.closed=True


class Phase:
    def __init__(self,test):self.t=test
    def preflight(self,scope,assets):
        return dict(scopeSha256=plan_digest(scope),assetsSha256=plan_digest(assets),supportedStages=list(models.STAGES))
    def goal(self,stage,record):
        result=self.t.ports.goal(stage,record)
        if self.t.fail_retention and stage['kind']=='lift' and result['reached']:
            result['retained']=False
        return result
    def verify_goal(self,stage,record,result):return True


class EpochObserver(Observer):
    """같은 부팅의 camera-us는 새 actor에서도 연속 증가한다."""
    def observe(self,**kwargs):
        record=super().observe(**kwargs)
        label=record['gripper']['label'];path='gripper/esp32-'+label+'.json'
        receipt=json.loads((self.root/path).read_text());stamp=int(self.clock.now*1_000_000)
        receipt['headers'].update({'X-MotionBrain-Captured-Us':str(stamp),
            'X-MotionBrain-Capture-Requested-Us':str(stamp-1),
            'X-MotionBrain-Frame-Id':'b'*32+':'+str(stamp)})
        record['gripper']['metadataSha256']=self.save(path,receipt)
        record['evidenceSha256']=self.save(record['evidenceFile'],{k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')})
        self.latest_record=copy.deepcopy(record)
        return record


class Factory:
    def __init__(self,test):self.t=test
    def preflight(self,scope,assets):
        return dict(scopeSha256=plan_digest(scope),assetsSha256=plan_digest(assets),supportedStages=['lift','lower'] if not self.t.missing_M4 else ['lift'])
    def __call__(self,stage,command,hooks):
        t=self.t
        t.assertIsNone(t.runtime.manager)
        for manager in t.managers:
            t.assertEqual(manager.current['phase'],'completed')
            t.assertTrue(manager.camera.closed)
        index=len(t.actors)+1
        scope=actor_scope();scope.update(schemaVersion=CONTACT_MANUAL650_SCHEMA if stage['kind']=='lift' else RETURN_SCHEMA,
            trialId='runtime-M4-'+str(index),hostClockId='runtime-M4-clock-'+str(index),
            expectedStartAngleDeg=t.device.angle,maxObservedDeltaDeg=3.,
            pulse=copy.deepcopy(LEARNING_PULSE if stage['kind']=='lift' else RETURN_PULSE))
        scope['context'].update(object='block',supportMode='table_supported' if stage['kind']=='lift' else 'gripper_supported')
        if stage['kind']=='lower':scope['goalAngleDeg']=196.
        root=t.repo/'.local'/('actor-'+str(index));root.mkdir()
        observer=EpochObserver(root,t.clock,scope)
        if stage['kind']=='lower':
            def before(record):
                record['onsite']['tableSupportVerified']=False
                record['evidenceSha256']=observer.save(record['evidenceFile'],{k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')})
                observer.latest_record=copy.deepcopy(record)
            observer.on_before=before
        actor=M4UpCommissioning(scope,observer,'private-token',ArtifactStore(root/'actor-result'),evidence_root=root,clock=t.clock,task_hooks=hooks)
        actor.port.opener=t.device;t.actors.append(actor);t.observers.append(observer)
        return actor,actor.digest


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        fixture=models.CycleWorkflowTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        self.bank=fixture.freeze();self.ports=models.ModelPorts(self.bank)
        self.repo=fixture.root.resolve();(self.repo/'.local').mkdir()
        self.clock=Clock();self.ports.clock=self.clock;self.device=Device(self.clock);self.device.angle=196.
        self.scope=self.ports.scope;self.scope['controllerBootId']=self.device.boot
        self.ports.scope=self.scope
        self.managers=[];self.actors=[];self.observers=[];self.memory_guards=0
        self.reject_observation=self.reject_bridge=self.reject_RUN_guard=self.fail_retention=self.missing_M4=False
        self.root_patch=patch.object(m4,'REPO',self.repo);self.root_patch.start();self.addCleanup(self.root_patch.stop)
        self.decode_patch=patch.object(m4,'decode',side_effect=self.decode);self.decode_patch.start();self.addCleanup(self.decode_patch.stop)
        disk=patch('learning.manual_collection.shutil.disk_usage',return_value=SimpleNamespace(free=20_000_000_000));disk.start();self.addCleanup(disk.stop)
        self.helper=self.repo/'helper';self.helper.write_text('synthetic camera helper')
        self.assets=dict(journalPath=str(self.repo/'.local/visual-actions/controller-commands.sqlite'))
        self.source=Source(self);self.phase=Phase(self);self.factory=Factory(self)
        self.reader=SequencePort('http://controller.invalid','private-token',self.device.boot,models.Store(),clock=self.clock,compact_reads=True)
        self.reader.opener=self.device
        self.readonly=CycleReadOnlyPort(self.reader)
        self.device.after_run=self.plant
        self.runtime=self.make()
    def decode(self,body,size,digest):
        for observer in self.observers:
            if body in observer.payloads:return observer.decode(body,size,digest)
        raise ValueError('unknown_camera_body')
    def plant(self,device):
        stage=self.runtime.stage['kind'];self.ports.count[stage]+=1
        if stage in ('lift','lower'):
            sign=1 if stage=='lift' else -1;self.ports.rise+=sign;device.angle+=.5*sign
    def idle(self):self.reader.preflight();return self.reader.boot_id
    def stop(self):
        self.assertFalse(any(self.device.speeds.values()))
        self.reader.preflight()
    def manager(self,stage,command):
        def port_factory(*args,**kwargs):
            port=RecordingPort(*args,**kwargs);port.opener=self.device;return port
        manager=ManualCollection(self.repo/('sessions-'+str(len(self.managers))),self.helper,'http://controller.invalid',
            'private-token','camera-id','test-view',journal_path=self.assets['journalPath'],clock=self.clock,
            camera_factory=ClosedCamera,port_factory=port_factory)
        self.managers.append(manager);self.addCleanup(manager.close);return manager
    def make(self):
        return CurrentCycleRuntime(self.scope,self.bank,assets=self.assets,
          authorization=dict(approved=True,scopeSha256=plan_digest(self.scope),automaticRetry=False,onsiteConfirmed=True),
          source=self.source,phase=self.phase,manager_factory=self.manager,m4_factory=self.factory,
          read_only=self.readonly,store=models.Store(),clock=self.clock)
    def run_cycle(self):return self.runtime.run(confirmed_scope_sha256=plan_digest(self.scope))
    def posts(self):return [(path,payload) for path,payload in self.device.requests if payload is not None]
    def test_real_general_sessions_and_scoped_M4_ports_complete_five_stages(self):
        result=self.run_cycle()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(result['cycleResult']['confirmedCommands'],5)
        self.assertTrue(result['cycleResult']['fullyModelSelectedCycle'])
        self.assertEqual(result['runtimeCounts']['generalSessionsStarted'],3)
        self.assertEqual(result['runtimeCounts']['generalSessionsFinished'],3)
        self.assertEqual(result['runtimeCounts']['M4ActorCalls'],2)
        self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),5)
        self.assertEqual(self.memory_guards,3)
        self.assertEqual(self.source.routes.count('general_armed'),3)
        self.assertEqual(self.source.routes.count('M4_armed'),2)
        self.assertEqual(self.source.routes.count('general_after'),3)
        self.assertTrue(self.source.closed)
        with self.assertRaisesRegex(ValueError,'single_use'):self.run_cycle()
    def test_lift_retention_failure_stops_before_DOWN_and_OPEN_without_retry(self):
        self.fail_retention=True;result=self.run_cycle()
        self.assertEqual(result['status'],'failed',result)
        self.assertEqual(result['cycleResult']['confirmedCommands'],3)
        runs=[p for path,p in self.posts() if path=='/sequence' and p.get('action')=='run']
        self.assertEqual(len(runs),3)
        adds=[p for path,p in self.posts() if path=='/sequence' and p.get('action')=='add']
        self.assertFalse(any(p['direction']=='down' or p['direction']=='open' for p in adds))
        self.assertTrue(result['cycleResult']['finalOffVerified'])
        self.assertEqual(result['runtimeCounts']['M4ActorCalls'],1)
    def test_missing_M4_sources_preflight_is_zero_sessions_and_zero_requests(self):
        self.missing_M4=True
        result=self.run_cycle()
        self.assertEqual(result['modelCycleAttempts'],0);self.assertEqual(self.managers,[])
        self.assertEqual(self.device.requests,[])
        self.assertIn('M4_stage_unsupported',str(result))
    def test_unsupported_observation_or_clock_bridge_does_not_send_motion(self):
        for mode in ('observation','bridge'):
            self.runtime=self.make();self.device.requests.clear()
            self.reject_observation=mode=='observation';self.reject_bridge=mode=='bridge'
            result=self.run_cycle();self.assertEqual(result['status'],'failed')
            self.assertFalse(any(p=={'action':'run'} for _,p in self.posts()))
    def test_ARM_memory_guard_rejection_has_zero_RUN_and_no_replacement(self):
        self.reject_RUN_guard=True;result=self.run_cycle()
        self.assertEqual(result['status'],'failed')
        self.assertEqual(result['runtimeCounts']['generalPulseRequests'],1)
        self.assertFalse(any(p=={'action':'run'} for _,p in self.posts()))
        self.assertEqual(len(self.managers),1)
        self.assertTrue(result['cycleResult']['finalOffVerified'])
    def test_unapproved_scope_rejected_without_factories(self):
        with self.assertRaisesRegex(ValueError,'not_confirmed'):
            self.runtime.run(confirmed_scope_sha256='f'*64)
        self.assertFalse(self.runtime.used);self.assertEqual(self.device.requests,[])

    def test_lost_RUN_reply_is_one_attempt_and_preserves_failed_session(self):
        original=self.device.after_run
        def lost(device):
            original(device)
            raise TimeoutError('synthetic_lost_RUN_reply')
        self.device.after_run=lost
        result=self.run_cycle()
        self.assertEqual(result['status'],'failed')
        self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),1)
        self.assertEqual(result['cycleResult']['confirmedCommands'],0)
        self.assertEqual(len(self.managers),1)
        self.assertEqual(self.managers[0].current['phase'],'failed')
        self.assertEqual(result['runtimeCounts']['generalOffReceipts'],0)
        self.assertTrue(result['cycleResult']['finalOffVerified'])
        with self.assertRaisesRegex(ValueError,'single_use'):self.run_cycle()

    def test_after_source_rejection_prevents_all_followup_stages(self):
        original=self.source.observe_general_after
        def rejected(*args,**kwargs):
            record=original(*args,**kwargs);record['context']={'object':'foreign'};return record
        self.source.observe_general_after=rejected
        result=self.run_cycle()
        self.assertEqual(result['status'],'failed')
        self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),1)
        self.assertEqual(result['runtimeCounts']['M4ActorCalls'],0)
        self.assertEqual(result['runtimeCounts']['generalSessionsFinished'],0)
        self.assertEqual(self.managers[0].current['phase'],'completed')

    def test_unrelated_M4_frames_cannot_replace_actor_sources(self):
        original=self.source.observe_m4_armed
        def unrelated(*args,**kwargs):
            record=original(*args,**kwargs);record['frames'][0]['frameId']='standalone-unrelated';return record
        self.source.observe_m4_armed=unrelated
        result=self.run_cycle()
        self.assertEqual(result['status'],'failed')
        self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),2)
        self.assertEqual(result['runtimeCounts']['M4ActorCalls'],1)
        self.assertEqual(len(self.actors),1)
        actor_result=json.loads((self.actors[0].store.root/'result.json').read_text())
        self.assertEqual(actor_result['commandOutcome'],'not_started')
        self.assertTrue(actor_result['finalIdleOffVerified'])

    def test_readonly_cursor_accepts_only_the_owned_nonzero_trace(self):
        self.reader.preflight()
        self.device.append(2,20);self.device.append(2,0)
        with self.assertRaisesRegex(ValueError,'unowned_output'):
            self.readonly.acknowledge_execution(dict(offVerified=True,outputTrace=[]))
