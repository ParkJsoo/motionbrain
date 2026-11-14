"""고정 모델 과업과 실제 단발 관절 포트의 수명 주기를 연결한다.

카메라/인증/장치 주소의 기본값은 없다. source는 원본·시계·등록을 검증하는
실제 관측 포트이고 phase는 원본에 결합된 물리 단계 판독 포트다. 생성·preflight는
구동하지 않는다. run은 정확한 scope SHA와 명시적 승인이 있어야 한 번 소비된다.
"""
from __future__ import annotations

import copy
from pathlib import Path
import time

from learning.current_task_adapter import ManualCollectionCyclePort
from learning.finite_task_cycle import FiniteTaskCycle, STAGES, validate_scope, bounded_command
from learning.manual_collection import ManualCollection
from learning.manual_collection_dataset import require
from learning.m4_up_commissioning import M4UpCommissioning
from learning.model_grasp_trial import ModelGraspTrial
from learning.supervised_transport import SequencePort
from learning.supervised_collection import plan_digest
from learning.task_cycle_workflow import FrozenCycleModelBank, ScopedCycleExecutor

SCHEMA='motionbrain.current_cycle_runtime.v1'


def _methods(value,names,code):
    require(all(callable(getattr(value,name,None)) for name in names),code)


class CycleReadOnlyPort:
    """관측 전용 cursor에 소유한 실행 영수증의 출력만 명시적으로 결합한다.

    일반 quiet 검사를 풀지 않는다. 허용된 영수증 이후 실제 cursor의 비영
    entry가 영수증 원본 trace와 같아야 하며, 추가 비영 출력은 거절한다.
    stop은 기존 STOP/OFF·큐 비우기·기본 출력 복원 경로만 사용한다.
    """
    def __init__(self,port):
        require(isinstance(port,SequencePort),'runtime_readonly_SequencePort_required')
        self.port=port

    def check_idle(self):
        self.port.preflight()
        require(self.port.off_verified,'runtime_readonly_current_OFF_required')
        return self.port.boot_id

    def acknowledge_execution(self,receipt):
        require(isinstance(receipt,dict) and receipt.get('offVerified') is True
                and isinstance(receipt.get('outputTrace'),list),'runtime_readonly_owned_receipt_required')
        _,new=self.port._outputs()
        allowed=receipt['outputTrace']
        require(all(e.get('pwm')==0 or e in allowed for e in new),
                'runtime_readonly_unowned_output')
        require(self.port.last_outputs=={i:0 for i in range(1,6)},'runtime_readonly_post_execution_OFF')
        self.port.preflight()

    def stop(self):
        require(self.port._stop_and_clear(restore_default=True,emergency=True),
                'runtime_cleanup_OFF_unverified')


class CurrentCycleRuntime:
    """기존 유한 코어의 포트 조율만 담당한다. 명령 선택은 FrozenBank만 사용한다.

    source.preflight(scope, assets)는 파일/등록/시계 계약과 모든 단계 지원을
    검사한다. source.observe_idle은 journal을 claim하지 않는 별도 정지 촬영이다.
    observe_general_armed는 전달된 manager.camera/source를, observe_m4_armed는
    전달된 actor.observer의 현재 원본을 사용해야 한다. 호스트 시계 이름을 단순
    교체하거나 '..'/symlink로 actor root를 우회해서는 안 된다.

    source.verify_observation/phase.verify_goal은 실제 원본을 재감사한다.
    source.validate_general_memory는 RUN 직전 메모리 검사만 수행한다.
    manager_factory(stage,command)는 아직 시작하지 않은 ManualCollection을
    반환한다. m4_factory(stage,command,hooks)는 scoped actor와 이미 허용된
    scope SHA를 반환하며, 기본 승인이나 일반 포트의 M4 UP 경로는 없다.
    """
    def __init__(self,scope,bank,*,assets,authorization,source,phase,manager_factory,
                 m4_factory,read_only,store,clock=time):
        self.scope=validate_scope(scope)
        require(isinstance(bank,FrozenCycleModelBank),'runtime_frozen_bank_required')
        bank.verify_cached()
        require(all(s['context']==bank.models[s['kind']]['context']
                    and s['modelSha256']==bank.hashes[s['kind']] for s in self.scope['stages']),
                'runtime_model_scope_mismatch')
        require(isinstance(assets,dict) and isinstance(assets.get('journalPath'),str)
                and Path(assets['journalPath']).is_absolute(),'runtime_shared_journal_asset')
        require(isinstance(authorization,dict) and set(authorization)=={
                    'approved','scopeSha256','automaticRetry','onsiteConfirmed'}
                and authorization['approved'] is True and authorization['automaticRetry'] is False
                and authorization['onsiteConfirmed'] is True
                and authorization['scopeSha256']==plan_digest(self.scope),
                'runtime_explicit_execution_scope')
        _methods(source,('preflight','observe_idle','observe_general_armed','observe_m4_armed',
                    'observe_general_after','verify_observation','validate_general_memory',
                    'verify_clock_bridge','close_idle','close_general','close_m4','close'),'runtime_actual_source_port_required')
        _methods(phase,('preflight','goal','verify_goal'),'runtime_raw_phase_port_required')
        _methods(read_only,('check_idle','acknowledge_execution','stop'),'runtime_readonly_and_cleanup_port_required')
        require(callable(manager_factory) and callable(m4_factory),'runtime_actual_factories_required')
        _methods(m4_factory,('preflight',),'runtime_M4_factory_preflight_required')
        require(callable(getattr(store,'write_json',None)),'runtime_durable_store_required')
        self.bank,self.assets,self.authorization=bank,copy.deepcopy(assets),copy.deepcopy(authorization)
        self.source,self.phase,self.manager_factory,self.m4_factory=source,phase,manager_factory,m4_factory
        self.read_only,self.store,self.clock=read_only,store,clock
        self.memory_sha=plan_digest(dict(scope=self.scope,assets=self.assets,authorization=self.authorization))
        self.used=False;self.manager=None;self.actor=None;self.stage=None;self.cycle=None
        self.source_frames=set();self.events=[]
        self.counts=dict(generalSessionsStarted=0,generalPulseRequests=0,generalOffReceipts=0,
                         generalSessionsFinished=0,M4ActorCalls=0)
        self.executor=ScopedCycleExecutor(self._make_m4,self._execute_general)

    def _memory(self):
        require(plan_digest(dict(scope=self.scope,assets=self.assets,authorization=self.authorization))==self.memory_sha,
                'runtime_memory_contract_changed')
        self.bank.verify_cached()

    def preflight(self):
        """파일/지원 계약 검사만 한다. 실제 세션·actor를 만들거나 시작하지 않는다."""
        self._memory()
        require(self.manager is None and self.actor is None,'runtime_owner_not_idle')
        proofs={}
        for name,port in [('source',self.source),('phase',self.phase),('M4',self.m4_factory)]:
            proof=port.preflight(copy.deepcopy(self.scope),copy.deepcopy(self.assets))
            require(isinstance(proof,dict) and proof.get('scopeSha256')==plan_digest(self.scope)
                    and proof.get('assetsSha256')==plan_digest(self.assets),
                    'runtime_'+name+'_preflight_audit_required')
            supported=proof.get('supportedStages')
            expected=set(('lift','lower')) if name=='M4' else set(STAGES)
            require(isinstance(supported,list) and len(supported)==len(expected)
                    and set(supported)==expected,'runtime_'+name+'_stage_unsupported')
            proofs[name]=copy.deepcopy(proof)
        self._memory()
        return dict(schemaVersion=SCHEMA,scopeSha256=plan_digest(self.scope),proofs=proofs,
                    hardwareCommands=0,sourceAndFactoryBindingsRequired=True)

    def _fresh_source_ids(self,record):
        require(isinstance(record,dict) and isinstance(record.get('frames'),list)
                and len(record['frames'])==2,'runtime_dual_source_required')
        ids=[f.get('frameId') for f in record['frames']]
        require(all(isinstance(i,str) and i for i in ids) and len(set(ids))==2
                and not self.source_frames.intersection(ids),'runtime_source_frame_reused')
        self.source_frames.update(ids)

    def _observe(self,stage,*,after_off,deadline):
        self._memory()
        if self.manager is not None:
            require(self.actor is None and self.manager.current is not None
                    and self.manager.current['phase']=='recording' and self.manager.busy,
                    'runtime_general_ARM_owner_required')
            record=self.source.observe_general_armed(copy.deepcopy(stage),self.manager,
                          after_off=after_off,deadline=deadline)
            route='general_armed'
        elif self.actor is not None:
            require(self.actor.before is not None and self.actor.observer is not None,
                    'runtime_M4_ARM_source_not_ready')
            record=self.source.observe_m4_armed(copy.deepcopy(stage),self.actor,
                          after_off=after_off,deadline=deadline)
            route='M4_armed'
            # Normalize only the actor's own immutable before sources; no copied or
            # unrelated native frames may stand in for this scoped execution.
            from learning.m4_up_commissioning import _external_frame,_gripper_frame
            frames=dict(external=_external_frame(self.actor.root,self.actor.before['external'],self.actor.scope),
                        gripper=_gripper_frame(self.actor.root,self.actor.before['gripper'],self.actor.scope,'before'))
            by_camera={f['camera']:f for f in record['frames']}
            require(set(by_camera)==set(frames) and all(all(by_camera[c][k]==frame[k]
                for k in ('frameId','pixelSha256','sourceEarliest','sourceLatest','persistedAt'))
                for c,frame in frames.items()),'runtime_M4_actor_source_route_changed')
        else:
            record=self.source.observe_idle(copy.deepcopy(stage),after_off=after_off,deadline=deadline)
            route='idle'
        require(self.source.verify_clock_bridge(copy.deepcopy(stage),copy.deepcopy(record),route,
                    self.manager if self.manager is not None else self.actor) is True,
                'runtime_host_monotonic_bridge_rejected')
        self._fresh_source_ids(record)
        self.events.append(dict(kind='observation_route',stage=stage['kind'],route=route))
        return record

    def _manager_for(self,command):
        require(self.manager is not None and self.stage is not None
                and command in self.stage['commands'],'runtime_current_general_owner')
        return self.manager

    def _execute_general(self,command,*,after_armed_before_run,before_run,decision):
        stage=copy.deepcopy(self.stage)
        require(stage['kind'] in ('approach','close','release') and self.manager is None
                and self.actor is None,'runtime_general_stage_owner')
        self.source.close_idle()
        manager=self.manager_factory(stage,copy.deepcopy(command))
        require(isinstance(manager,ManualCollection) and manager.current is None and not manager.busy
                and manager.port is None and manager.camera is None
                and manager.journal_path.resolve()==Path(self.assets['journalPath']).resolve(),
                'runtime_new_shared_ManualCollection_required')
        self.manager=manager
        try:
            manager.start(dict(joint=command['joint'],poseContext=stage['context']['poseRegime'],
                               onsiteConfirmed=self.authorization['onsiteConfirmed']))
            self.counts['generalSessionsStarted']+=1
            require(manager.port.boot_id==self.scope['controllerBootId'],'runtime_general_boot_changed')
            port=ManualCollectionCyclePort(self._manager_for)
            def gate():
                self._memory()
                self.source.validate_general_memory(copy.deepcopy(stage),manager,copy.deepcopy(command))
                before_run()
            self.counts['generalPulseRequests']+=1
            receipt=port(copy.deepcopy(command),after_armed_before_run=after_armed_before_run,
                         before_run=gate,decision=copy.deepcopy(decision))
            self.counts['generalOffReceipts']+=1
            # Keep the owner recording through the raw three-second OFF window.
            while self.clock.monotonic()<receipt['offAt']+3.:
                require(self.clock.monotonic()<self.cycle.deadline,'runtime_general_post_OFF_deadline')
                manager.tick();self.clock.sleep(.05)
            post=self.source.observe_general_after(stage,manager,copy.deepcopy(receipt),deadline=self.cycle.deadline)
            require(self.source.verify_clock_bridge(stage,copy.deepcopy(post),'general_after',manager) is True,
                    'runtime_host_monotonic_bridge_rejected')
            self._fresh_source_ids(post)
            require(self.source.verify_observation(stage,copy.deepcopy(post)) is True,
                    'runtime_general_after_source_rejected')
            # Reuse the finite core's raw/freshness/boot/pose checks, without a new selection.
            self.cycle._validate_record(stage,post,after_off=receipt['offAt'])
            finished=manager.finish()
            require(finished.get('phase')=='completed' and finished.get('finalIdleOffVerified') is True,
                    'runtime_general_finish_OFF_unverified')
            self.counts['generalSessionsFinished']+=1
            self.events.append(dict(kind='general_finished',stage=stage['kind'],sessionId=finished['sessionId']))
            return receipt
        finally:
            # close owns an interrupted session's existing STOP/OFF cleanup. No second pulse.
            try:self.source.close_general(manager)
            finally:
                try:manager.close()
                finally:
                    manager.token='';self.manager=None

    def _make_m4(self,stage,command,hooks):
        require(self.manager is None and self.actor is None,'runtime_M4_general_session_pending')
        self.source.close_idle()
        # Factories retain the original scoped actor's root/observer source bindings.
        actor,confirmed_sha=self.m4_factory(copy.deepcopy(stage),copy.deepcopy(command),hooks)
        require(isinstance(actor,M4UpCommissioning),'runtime_scoped_M4_actor_required')
        from learning import m4_up_commissioning as m4
        expected=m4.REPO/'.local/visual-actions/controller-commands.sqlite'
        require(expected.resolve()==Path(self.assets['journalPath']).resolve(),'runtime_M4_shared_journal_changed')
        self.actor=actor;self.counts['M4ActorCalls']+=1
        return actor,confirmed_sha

    def _execute(self,stage,command,**kwargs):
        self.stage=copy.deepcopy(stage)
        try:
            # ScopedCycleExecutor directly calls the validated M4 actor.run; general route
            # above directly starts/pulses/finishes the real ManualCollection.
            receipt=self.executor(stage,command,**kwargs)
            ModelGraspTrial._receipt(self.cycle,receipt,bounded_command(stage['kind'],command))
            self.read_only.acknowledge_execution(copy.deepcopy(receipt))
            return receipt
        finally:
            if self.actor is not None:
                actor=self.actor
                try:self.source.close_m4(actor)
                finally:actor.port.token='';self.actor=None

    def _idle(self):
        require(self.manager is None and self.actor is None,'runtime_idle_owner_still_active')
        return self.read_only.check_idle()

    def _stop(self):
        # Existing owners have already attempted their own OFF cleanup in finally.
        # The explicitly injected cleanup port may STOP/OFF, never issue another RUN.
        self.read_only.stop()

    def run(self,*,confirmed_scope_sha256):
        require(not self.used,'runtime_single_use')
        require(confirmed_scope_sha256==self.authorization['scopeSha256'],'runtime_execution_scope_not_confirmed')
        self.used=True
        report=dict(schemaVersion=SCHEMA,status='failed',modelCycleAttempts=0,cycleResult=None,
                    errors=[],runtimeCounts=self.counts,events=self.events)
        try:
            preflight=self.preflight()
            self.store.write_json('runtime-consumed.json',dict(scope=self.scope,authorization=self.authorization,
                                 assets=self.assets,preflight=preflight))
            self.cycle=FiniteTaskCycle(self.scope,observe=self._observe,
                verify_observation=self.source.verify_observation,choose=self.bank.choose,
                goal=self.phase.goal,verify_goal=self.phase.verify_goal,execute=self._execute,
                verify_assets=self._memory,check_idle=self._idle,stop=self._stop,
                store=self.store,clock=self.clock)
            report['modelCycleAttempts']=1
            report['cycleResult']=self.cycle.run();report['status']=report['cycleResult']['status']
        except (Exception,KeyboardInterrupt) as exc:
            report['errors'].append(dict(type=type(exc).__name__,reason=str(exc) if isinstance(exc,ValueError)
                                         else 'runtime_preflight_or_port_failed'))
        finally:
            try:self.source.close()
            except (Exception,KeyboardInterrupt):
                report['status']='failed';report['errors'].append(dict(type='cleanup',reason='runtime_source_close_failed'))
            self.store.write_json('runtime-result.json',report)
        return copy.deepcopy(report)
