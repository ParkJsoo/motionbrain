"""명시한 비공개 자산과 고정 모델을 실제 과업 포트에 조립한다.

생성/preflight는 촬영·HTTP·SSH를 실행하지 않는다. manager.start와 actor.run은
CurrentCycleRuntime 소유다. 주소·인증·모델·helper 기본값이나 수동 방향 대체는 없다.
"""
from __future__ import annotations

import copy
import math
from hashlib import sha256
from pathlib import Path
import time
import uuid

from learning.artifacts import ArtifactStore,_read,_decode_json
from learning.action_recorder import ActionRecorder
from learning.current_cycle_phase import CurrentCyclePhase,source_record
from learning.current_full_cycle_source import FullCycleSource,OwnedDualSource,SourcePreflightError
from learning.current_m1_contract import CurrentM1CollectionGuard
from learning.current_observation_contract import CurrentTaskM3Guard
from learning.current_m4_review import StationaryM4BeforeReviewer,SCHEMA as ANCHOR_SCHEMA
from learning.current_m4_short import short_task_pulse,validate_current_short_policy
from learning.m4_effective_policy import (POLICY_SCHEMA as EFFECTIVE_POLICY_SCHEMA,
    validate_effective_policy,effective_task_pulse,SCHEMA as EFFECTIVE_SCHEMA)
from learning.development_gripper_source import scene_context
from learning.m1_shaft_scene import scene_context as shaft_scene
from learning.finite_task_cycle import STAGES,validate_scope,bounded_command
from learning.grasp_observation import GripperObservationSource
from learning.manual_collection import ManualCollection
from learning.manual_collection_dataset import require
from learning.m4_up_commissioning import M4UpCommissioning,CURRENT_SHORT_TASK_SCHEMA,implementation_digest
from learning.m4_up_observer import M4DualCameraObserver
from learning.supervised_transport import SequencePort,validate_target
from learning.supervised_collection import plan_digest
from learning.task_cycle_workflow import FrozenCycleModelBank
from learning.pipeline_learning import digest as decision_digest


class FactoryPreflightError(ValueError):
    def __init__(self,missing):
        self.missing=copy.deepcopy(missing)
        super().__init__('cycle_factory_missing:'+str(missing))


def _bound(root,ref):
    require(isinstance(ref,dict) and set(ref)=={'path','sha256'},'factory_asset_binding')
    raw=_read(root,ref['path']);require(sha256(raw).hexdigest()==ref['sha256'],'factory_asset_changed')
    return raw


def _execution_policy(value):
    if isinstance(value,dict) and value.get('schemaVersion')==EFFECTIVE_POLICY_SCHEMA:
        return validate_effective_policy(value)
    return validate_current_short_policy(value)


def _execution_pulse(direction,policy):
    return (effective_task_pulse(direction,policy) if policy['schemaVersion']==EFFECTIVE_POLICY_SCHEMA
            else short_task_pulse(direction,policy))


def _model_stage_support(bank,scope):
    missing={};owned={}
    for stage in scope['stages']:
        kind=stage['kind']
        if kind not in ('lift','lower'):continue
        support='table_supported' if kind=='lift' else 'gripper_supported'
        entries=bank.models[kind]['models'];rows=bank.models[kind]['trainingRows']
        for command in stage['commands']:
            matches=[e for e in entries if e['command']==command and e['supportMode']==support and e['count']>=2]
            if not matches:missing.setdefault(kind,[]).append('factory_M4_candidate_support_two_rows_missing')
        selected=[r for r in rows if r['command'] in stage['commands'] and r['supportMode']==support]
        owned[kind]={tag+':'+value for r in selected for tag,value in
            [('source',r['sourceHash']),('episode',plan_digest(r['episode']))]+[('frame',f) for f in r['frameIds']]}
    if set(owned)=={'lift','lower'} and owned['lift'].intersection(owned['lower']):
        for kind in ('lift','lower'):missing.setdefault(kind,[]).append('factory_M4_UP_DOWN_source_overlap')
    if missing:raise FactoryPreflightError(missing)


def _stage_contracts(root,scope,assets):
    missing={};policies={}
    for stage in scope['stages']:
        kind=stage['kind']
        try:
            if kind in ('lift','lower'):
                policy=_execution_policy(stage['context'].get('executionPolicy'))
                policies[kind]=policy
                pulse=_execution_pulse('up' if kind=='lift' else 'down',policy)
                expected=[{k:pulse[k] for k in ('joint','direction','percent','durationMs')}]
                require(stage['commands']==expected,'factory_current_short_candidates_required')
            else:
                joint='elbow' if kind=='approach' else 'gripper'
                directions=('up','down') if kind=='approach' else (('close',) if kind=='close' else ('open',))
                allowed=[dict(joint=joint,direction=d,percent=60,durationMs=350 if kind=='approach' else 650) for d in directions]
                require(bool(stage['commands']) and all(c in allowed for c in stage['commands']),
                        'factory_general_exact_candidates_required')
        except (ValueError,KeyError) as exc:missing.setdefault(kind,[]).append(str(exc))
    if set(policies)=={'lift','lower'}:
        if policies['lift']!=policies['lower']:
            for kind in ('lift','lower'):missing.setdefault(kind,[]).append('factory_paired_short_policy_mismatch')
        policy=policies['lift']
        for stage in scope['stages']:
            kind=stage['kind']
            if kind in ('approach','close','release'):
                try:
                    config=_decode_json(_bound(root,assets['sourceStages'][kind]['guardConfig']))
                    # These are declared nominal table-pose contracts. Live
                    # sensor variation remains the existing guard's ±.25° rule.
                    require(type(config.get('referenceM4Deg')) in (int,float)
                            and config['referenceM4Deg']==policy['tableReferenceDeg'],
                            'factory_guard_table_reference_mismatch')
                    if kind=='release':
                        require(stage['target'].get('expectedM4AngleDeg')==policy['downGoalAngleDeg'],
                                'factory_release_table_goal_mismatch')
                except (ValueError,KeyError,OSError,RuntimeError) as exc:missing.setdefault(kind,[]).append(str(exc))
    if missing:raise FactoryPreflightError(missing)


def _audit_source_assets(root,bank,scope,assets):
    class Probe:
        def bind_source(self,source):pass
    def forbidden(*args,**kwargs):raise ValueError('offline_preflight_factory_forbidden')
    source=FullCycleSource(root,bank=bank,idle_factory=forbidden,general_factory=forbidden,phase=Probe())
    missing={}
    try:source.preflight(scope,assets)
    except SourcePreflightError as exc:missing=copy.deepcopy(exc.missing)
    identities=set()
    for stage in scope['stages']:
        kind=stage['kind']
        try:
            spec=assets['sourceStages'][kind];actual=spec['sourceContext']
            identities.add((actual['gripperCameraBootId'],actual['externalDeviceId'],actual['externalViewId']))
            if kind in ('approach','close','release'):
                config=_decode_json(_bound(root,spec['guardConfig']))
                require(stage['context']['cameraIdentity']['cameraBootId']==actual['gripperCameraBootId']
                    and all(config.get(k)==actual[k] for k in ('gripperCameraBootId','externalDeviceId','externalViewId')),
                    'factory_current_camera_guard_model_mismatch')
        except (ValueError,KeyError,OSError,RuntimeError) as exc:missing.setdefault(kind,[]).append(str(exc))
    if len(identities)!=1:
        for kind in STAGES:missing.setdefault(kind,[]).append('factory_mixed_camera_identity')
    try:_stage_contracts(root,scope,assets)
    except FactoryPreflightError as exc:
        for kind,errors in exc.missing.items():missing.setdefault(kind,[]).extend(errors)
    try:_model_stage_support(bank,scope)
    except FactoryPreflightError as exc:
        for kind,errors in exc.missing.items():missing.setdefault(kind,[]).extend(errors)
    if missing:raise FactoryPreflightError(missing)


def build_cycle_assets(root,scope,*,model_bindings,source_stages,journal_path,helper_binding,m1_observation_policies=None):
    """실제 source preflight를 재사용한다. 부족 자산을 ready로 대신하지 않는다.

model_bindings는 FrozenCycleModelBank의 절대 경로/SHA 계약, 나머지 binding은
root 아래 안전한 상대 경로/SHA다. 상태 등록과 픽셀 계측 등록은 source 계약대로
각각 modelStateRegistration/registration에 넣는다.
    """
    root=Path(root);missing={}
    for kind in STAGES:
        errors=[];binding=model_bindings.get(kind)
        if not isinstance(binding,dict):errors.append('frozen_model_missing')
        else:
            try:
                require(set(binding)=={'path','sha256'} and Path(binding['path']).is_absolute(),'model_absolute_binding')
                require(sha256(Path(binding['path']).read_bytes()).hexdigest()==binding['sha256'],'frozen_model_file_changed')
            except (ValueError,KeyError,OSError) as exc:errors.append(str(exc))
        if not isinstance(source_stages.get(kind),dict):errors.append('source_registration_missing')
        if errors:missing[kind]=errors
    if missing:raise FactoryPreflightError(missing)
    bank=FrozenCycleModelBank(model_bindings,allow_development_approach=True,
                             m1_observation_policies=m1_observation_policies)
    scope=validate_scope(scope)
    _bound(root,helper_binding)
    require(Path(journal_path).is_absolute(),'factory_absolute_journal_required')
    assets=dict(journalPath=str(journal_path),sourceStages=copy.deepcopy(source_stages),
        helper=copy.deepcopy(helper_binding),m4ImplementationSha256=implementation_digest(),
        phaseMode='source_bound_human',phaseProtocol='nonce_seed_anchor_fresh_once_v1')
    if m1_observation_policies:
        assets['m1ObservationPolicies']=copy.deepcopy(m1_observation_policies)
    # No factory, recorder, phase request, or device is created by this audit.
    _audit_source_assets(root,bank,scope,assets)
    for kind in ('lift','lower'):
        try:_execution_policy(bank.models[kind]['context'].get('executionPolicy'))
        except ValueError as exc:missing[kind]=[str(exc)]
    if missing:raise FactoryPreflightError(missing)
    return assets,bank


class _ReadyObserver(M4DualCameraObserver):
    def __init__(self,*args,resource_check,**kwargs):
        self.resource_check=resource_check
        super().__init__(*args,**kwargs)

    def start(self,*,deadline):
        self.resource_check();super().start(deadline=deadline)
        remaining=deadline-self.clock.monotonic();require(remaining>0,'factory_native_start_deadline')
        self.recorder.wait_ready(timeout=min(15.,remaining))
        if self.scope['schemaVersion']==EFFECTIVE_SCHEMA:
            self._wait_effective_native_warmup(deadline)

    def _wait_effective_native_warmup(self,deadline):
        # Wait for source-time progress on this same recording, not another
        # capture or relabelled frame. Gripper capture still happens after ARM.
        with self.recorder.lock:
            require(bool(self.recorder.frames),'factory_effective_native_first_frame_required')
            first_latest=self.recorder.frames[0]['sourceLatest']
        require(type(first_latest) in (int,float) and math.isfinite(first_latest),
                'factory_effective_native_source_time_required')
        threshold=first_latest+2.
        while True:
            now=self.clock.monotonic()
            require(now<deadline,'factory_effective_native_warmup_deadline')
            self.resource_check();self.recorder.check()
            with self.recorder.lock:
                latest=self.recorder.frames[-1]['sourceEarliest']
            require(type(latest) in (int,float) and math.isfinite(latest),
                    'factory_effective_native_source_time_required')
            require(self.clock.monotonic()<deadline,'factory_effective_native_warmup_deadline')
            if latest>=threshold:return
            self.clock.sleep(min(.05,deadline-self.clock.monotonic()))

    def check(self):
        self.resource_check();return super().check()

    def observe(self,*,after_off,deadline,tick):
        if after_off is None and self.scope['schemaVersion']==EFFECTIVE_SCHEMA:
            self._armed_source_not_before=self.clock.monotonic()
        return super().observe(after_off=after_off,deadline=deadline,tick=tick)

    def _external(self,not_before,deadline,tick):
        if self.scope['schemaVersion']==EFFECTIVE_SCHEMA:
            not_before=max(not_before,getattr(self,'_armed_source_not_before',not_before))
        return super()._external(not_before,deadline,tick)


class _CheckedRecorder(ActionRecorder):
    def __init__(self,*args,resource_check,**kwargs):
        self.resource_check=resource_check;super().__init__(*args,**kwargs)
    def start(self):
        self.resource_check();return super().start()
    def check(self):
        self.resource_check();return super().check()


class _ReadOnlyPort(SequencePort):
    def _call(self,path,payload=None,**kwargs):
        require(payload is None and not kwargs.get('run'),'factory_idle_controller_write_forbidden')
        return super()._call(path,payload,**kwargs)


class _CycleM1Guard(CurrentM1CollectionGuard):
    """기존 현장 OPEN 수집기의 +40px 여유를 모델 실행에서도 보존한다."""
    def review_armed(self,record,result,**kwargs):
        proof=super().review_armed(record,result,**kwargs)
        self._cycle_shaft=result['shaftSeparationPx']
        return proof
    def validate_command(self,command):
        if command.get('direction')=='open':
            require(hasattr(self,'_cycle_shaft') and self._cycle_shaft+40<=139.5,
                    'factory_current_M1_open_headroom')
        return super().validate_command(command)


class _M4Factory:
    def __init__(self,owner):self.owner=owner
    def preflight(self,scope,assets):
        self.owner.preflight(scope,assets)
        return dict(scopeSha256=plan_digest(scope),assetsSha256=plan_digest(assets),supportedStages=['lift','lower'])
    def __call__(self,stage,command,hooks):return self.owner._m4(stage,command,hooks)


class CurrentRuntimeFactory:
    def __init__(self,scope,assets,bank,*,repo_root,asset_root,run_root,controller_url,camera_url,
                 token,authorization,resource_check,after_reviewer_factory,clock=time):
        self.scope=validate_scope(scope);self.assets=copy.deepcopy(assets);self.bank=bank
        self.repo=Path(repo_root).resolve();self.asset_root=Path(asset_root).resolve();self.root=Path(run_root).resolve()
        require(self.asset_root.is_relative_to(self.repo/'.local') and self.root.is_relative_to(self.repo/'.local'),
                'factory_private_roots_required')
        self.controller_url=validate_target(controller_url)
        # Reuse camera-origin validation without starting/capturing anything.
        probe=GripperObservationSource(camera_url,'factory-origin-check','0'*32,
            dict(physicalView='check',object='check',poseRegime='check',landmarks='check'),None,clock=clock)
        self.camera_url=probe.url
        require(isinstance(token,str) and token and callable(resource_check) and callable(after_reviewer_factory),
                'factory_private_credentials_and_resource_reviewer_required')
        require(authorization==dict(approved=True,scopeSha256=plan_digest(self.scope),
                    automaticRetry=False,onsiteConfirmed=True),'factory_parent_scope_authorization')
        self.token=token;self.authorization=copy.deepcopy(authorization)
        self.resource_check,self.after_reviewer_factory,self.clock=resource_check,after_reviewer_factory,clock
        self.source=None;self.managers=[];self.actors=[];self.readers=[];self.serial=0;self.closed=False
        self._verified=False;self._manifest_sha=plan_digest(dict(scope=self.scope,assets=self.assets))
        self._model_bindings=copy.deepcopy(bank.bindings) if isinstance(bank,FrozenCycleModelBank) else None
        self.m4=_M4Factory(self);self.preflight(self.scope,self.assets)

    def bind_source(self,source):
        require(isinstance(source,FullCycleSource) and source.root.resolve()==self.asset_root
                and source.clock is self.clock and source.bank is self.bank,
                'factory_actual_source_required')
        require(self.source is None or self.source is source,'factory_source_owner_changed')
        self.source=source

    def assemble(self,*,review_root,phase_request_store,store):
        """실제 source/phase/포트를 조립한다. run이나 장치 preflight는 호출하지 않는다."""
        require(self.source is None and Path(store.root).resolve().is_relative_to(self.root),
                'factory_new_private_runtime_store_required')
        from learning.current_cycle_runtime import CurrentCycleRuntime,CycleReadOnlyPort
        phase=CurrentCyclePhase(review_root,audit_observation=lambda stage,record:source.verify_observation(stage,record),
                                clock=self.clock,request_store=phase_request_store)
        source=FullCycleSource(self.asset_root,bank=self.bank,idle_factory=self.idle,
                               general_factory=self.general_source,phase=phase,clock=self.clock)
        self.bind_source(source)
        # This separate reader owns the explicitly authorized runtime STOP/OFF
        # cleanup. Individual IDLE acquisition ports remain GET-only.
        port=SequencePort(self.controller_url,self.token,self.scope['controllerBootId'],
                          ArtifactStore(store.root/'read-only-controller'),clock=self.clock,compact_reads=True)
        self.readers.append(port)
        return CurrentCycleRuntime(self.scope,self.bank,assets=self.assets,authorization=self.authorization,
            source=source,phase=phase,manager_factory=self.manager,m4_factory=self.m4,
            read_only=CycleReadOnlyPort(port),store=store,clock=self.clock)

    def preflight(self,scope,assets):
        require(scope==self.scope and assets==self.assets and not self.closed,'factory_contract_changed')
        require(plan_digest(dict(scope=self.scope,assets=self.assets))==self._manifest_sha,'factory_manifest_memory_changed')
        require(isinstance(self.bank,FrozenCycleModelBank),'factory_frozen_bank_required');self.bank.verify_cached()
        require(self.bank.bindings==self._model_bindings,'factory_model_bindings_changed')
        require(assets['journalPath']==str(self.repo/'.local/visual-actions/controller-commands.sqlite'),
                'factory_actor_shared_journal_required')
        require(assets['m4ImplementationSha256']==implementation_digest(),'factory_M4_implementation_changed')
        _bound(self.asset_root,assets['helper'])
        _stage_contracts(self.asset_root,scope,assets)
        _model_stage_support(self.bank,scope)
        for kind in STAGES:
            stage=next(s for s in scope['stages'] if s['kind']==kind)
            binding=self._model_bindings[kind]
            require(sha256(Path(binding['path']).read_bytes()).hexdigest()==binding['sha256'],
                    'factory_frozen_model_file_changed')
            require(stage['modelSha256']==self.bank.hashes[kind] and stage['context']==self.bank.models[kind]['context'],
                    'factory_model_stage_changed')
            spec=assets['sourceStages'][kind]
            require(spec['context']==stage['context'],'factory_source_context_changed')
            for name in ('registration','stationaryRegistration','stationaryExternalReference','stationaryGripperReference'):
                _bound(self.asset_root,spec[name])
            if kind in ('lift','lower'):
                policy=_execution_policy(stage['context'].get('executionPolicy'))
                _bound(self.asset_root,spec['modelStateRegistration'])
                pulse=_execution_pulse('up' if kind=='lift' else 'down',policy)
                require(stage['commands']==[{k:pulse[k] for k in ('joint','direction','percent','durationMs')}],
                        'factory_current_short_candidates_required')
            else:_bound(self.asset_root,spec['guardConfig'])
        if not self._verified:
            _audit_source_assets(self.asset_root,self.bank,self.scope,self.assets)
            self._verified=True
        return dict(scopeSha256=plan_digest(scope),assetsSha256=plan_digest(assets),supportedStages=list(STAGES))

    def _start_factory(self):
        self.preflight(self.scope,self.assets)
        require(self.source is not None and self.source.scope==self.scope,'factory_verified_source_not_bound')
        require(all(not m.busy and (m.current is None or m.current['phase']=='completed') for m in self.managers),
                'factory_general_session_pending')
        for actor in self.actors:
            require(actor.used,'factory_prior_M4_not_executed')
            result=_decode_json(_read(actor.store.root,'result.json'))
            require(result.get('status')=='completed' and result.get('finalIdleOffVerified') is True
                    and result.get('errors')==[], 'factory_prior_M4_failed_no_retry')
        self.resource_check();self.serial+=1
        return self.serial

    def _spec(self,stage):
        require(stage in self.scope['stages'],'factory_unlisted_stage')
        return self.assets['sourceStages'][stage['kind']]

    def _scene(self,stage):
        _,observer=self.source.registrations[stage['kind']]
        if stage['kind']=='approach':return scene_context(observer)
        if stage['kind'] in ('close','release'):return shaft_scene(observer)
        actual=self._spec(stage)['sourceContext']
        return dict({k:actual[k] for k in ('physicalView','object','poseRegime')},landmarks='M4 current phase raw review')

    def idle(self,stage):
        serial=self._start_factory();spec=self._spec(stage);ctx=spec['sourceContext']
        store=ArtifactStore(self.root/f'idle-{serial:03d}')
        sid='M4-stationary-anchor-cycle-'+uuid.uuid4().hex
        plan=dict(deviceId=ctx['externalDeviceId'],viewId=ctx['externalViewId'],sessionId=sid,hostClockId=sid,
            maxFrames=500,seconds=120,sampleFps=4,maxBytes=4200000000,width=1920,height=1080)
        recorder=_CheckedRecorder(store.root/'external',self.asset_root/self.assets['helper']['path'],plan,
                                  resource_check=self.resource_check,clock=self.clock)
        source=GripperObservationSource(self.camera_url,sid,ctx['gripperCameraBootId'],self._scene(stage),
            ArtifactStore(store.root/'gripper'),max_captures=min(100,2*len(self.scope['stages'])+2*self.scope['maxCommands']),clock=self.clock)
        port=_ReadOnlyPort(self.controller_url,self.token,self.scope['controllerBootId'],ArtifactStore(store.root/'controller'),
                          clock=self.clock,compact_reads=True)
        self.readers.append(port)
        return OwnedDualSource(store.root,recorder,source,port)

    def manager(self,stage,command):
        serial=self._start_factory();bounded_command(stage['kind'],command)
        require(stage['kind'] in ('approach','close','release') and command in stage['commands'],
                'factory_general_model_command_required')
        ctx=self._spec(stage)['sourceContext']
        def camera_factory(*args,**kwargs):
            require('clock' not in kwargs or kwargs['clock'] is self.clock,'factory_recorder_clock_changed')
            kwargs['clock']=self.clock
            return _CheckedRecorder(*args,resource_check=self.resource_check,**kwargs)
        manager=ManualCollection(self.root/f'general-{serial:03d}',self.asset_root/self.assets['helper']['path'],
            self.controller_url,self.token,ctx['externalDeviceId'],ctx['externalViewId'],
            journal_path=self.assets['journalPath'],clock=self.clock,sample_fps=4,recording_seconds=120,camera_factory=camera_factory)
        self.managers.append(manager)
        return manager

    def general_source(self,stage,manager):
        require(any(m is manager for m in self.managers) and manager.current is not None
                and manager.current['phase']=='recording','factory_owned_started_manager_required')
        spec=self._spec(stage);ctx=spec['sourceContext'];sid=manager.current['sessionId']
        config=_decode_json(_bound(self.asset_root,spec['guardConfig']));config['hostClockId']=sid
        saved=manager.store.write_json('cycle-session-guard.json',config)
        relative=str((manager.store.root/saved['artifactRef']).relative_to(self.repo))
        guard_cls=CurrentTaskM3Guard if stage['kind']=='approach' else _CycleM1Guard
        guard=guard_cls(self.repo,dict(path=relative,sha256=saved['sha256']),manager.camera,manager.port,manager.store,clock=self.clock)
        source=GripperObservationSource(self.camera_url,sid,ctx['gripperCameraBootId'],self._scene(stage),
            ArtifactStore(manager.store.root/'gripper-source'),max_captures=3,clock=self.clock)
        return source,guard

    def _entry(self,stage,command,hooks):
        require(self.source is not None,'factory_source_required')
        candidates=[v for v in self.source.evidence.values() if v['stage']==stage and v['route']=='idle']
        require(candidates,'factory_current_idle_observation_missing')
        raw=max(candidates,key=lambda v:v['record']['auditedAt'])['record']
        record=copy.deepcopy(raw)
        record['entry']=self.source.phase.entry(stage,record)
        if record['entry'].get('tableSupported') is True:record['supportMode']='table_supported'
        elif record['entry'].get('airborne') is True and record['entry'].get('bilateralContact') is True:
            record['supportMode']='gripper_supported'
        require(self.source.verify_observation(stage,record) is True
            and all(0<=self.clock.monotonic()-f['sourceEarliest']<=3. for f in record['frames']),
            'factory_idle_input_stale')
        decision=self.bank.choose(stage,record)
        require(decision['selected'].get('kind')=='action' and decision['selected'].get('command')==command
                and decision_digest(decision)==hooks['decisionSha256'],'factory_frozen_model_decision_changed')
        return record

    def _anchor(self,stage,record,store):
        source=self.source.describe_source(record);root=Path(source['root'])
        external=source['external'];gripper=source['gripper']
        ingress=_decode_json(_read(root,external['root']+'/'+external['ingressFile']))
        from learning.lossless_frame import decode
        import cv2,numpy as np
        raw=decode(_read(root,external['root']+'/'+ingress['imageFile']),ingress['rawByteLength'],ingress['pixelSha256'])
        image=np.frombuffer(raw,np.uint8).reshape(ingress['height'],ingress['width'],4)
        ok,png=cv2.imencode('.png',image);require(ok,'factory_anchor_preview')
        image_ref=store._write('idle-anchor-external.png',png.tobytes())
        def ref(path):
            relative=str(path.relative_to(self.repo/'.local'));raw=_read(self.repo/'.local',relative)
            return dict(path=relative,sha256=sha256(raw).hexdigest())
        labels=record['entry'];spec=self._spec(stage)
        anchor=dict(schemaVersion=ANCHOR_SCHEMA,context=copy.deepcopy(spec['sourceContext']),
            epoch=dict(controllerBootId=record['controllerBootId'],cameraBootId=spec['sourceContext']['gripperCameraBootId'],
                       outputSequence=record['outputSequence'],allFiveOff=True),
            labels=dict(operatorPresent=labels.get('operatorPresent') is True,clearanceVerified=labels.get('clearance') is True,
                        jawContactVerified=labels.get('bilateralContact') is True,tableSupportVerified=labels.get('tableSupported') is True),
            sources=dict(external=dict(image=ref(store.root/image_ref['artifactRef']),
                metadata=ref(root/external['root']/ingress['metadataFile'])),
                gripper=dict(image=ref(root/gripper['root']/('esp32-'+gripper['label']+'.jpg')),
                    metadata=ref(root/gripper['root']/('esp32-'+gripper['label']+'.json')))),
            metrologyRegistration=ref(self.asset_root/spec['registration']['path']),
            reviewBasis='root_visual_stationary_anchor_review',humanAssisted=True)
        saved=store.write_json('idle-anchor.json',anchor)
        return ref(store.root/saved['artifactRef'])

    def _m4(self,stage,command,hooks):
        serial=self._start_factory();require(stage['kind'] in ('lift','lower'),'factory_M4_stage')
        require(all(m.current is None or m.current['phase']=='completed' for m in self.managers)
                and all(not m.busy for m in self.managers),'factory_general_session_pending')
        policy=_execution_policy(stage['context'].get('executionPolicy'))
        effective=policy['schemaVersion']==EFFECTIVE_POLICY_SCHEMA
        direction='up' if stage['kind']=='lift' else 'down';pulse=_execution_pulse(direction,policy)
        require(command=={k:pulse[k] for k in ('joint','direction','percent','durationMs')},'factory_current_short_command_required')
        record=self._entry(stage,command,hooks);ctx=self._spec(stage)['sourceContext']
        require(record['supportMode']==('table_supported' if direction=='up' else 'gripper_supported')
                and ctx['supportMode']==record['supportMode'],'factory_M4_support_changed')
        require((direction=='up' and abs(record['m4AngleDeg']-policy['tableReferenceDeg'])<=.1
                 and record['m4AngleDeg']+policy['maxObservedDeltaDeg']<=policy['originalGlobalUpperDeg'])
                or (direction=='down' and policy['downGoalAngleDeg']+.1<record['m4AngleDeg']<=policy['originalGlobalUpperDeg']),
                'factory_current_short_entry_angle_unsupported')
        trial=('cycle-effective-' if effective else 'cycle-short-')+uuid.uuid4().hex
        scope=dict(schemaVersion=EFFECTIVE_SCHEMA if effective else CURRENT_SHORT_TASK_SCHEMA,trialId=trial,hostClockId=trial,
            controllerUrl=self.controller_url,controllerBootId=self.scope['controllerBootId'],
            implementationSha256=implementation_digest(),context=copy.deepcopy(ctx),pulse=pulse,defaultSpeed=255,
            maxCommands=1,expectedStartAngleDeg=policy['tableReferenceDeg'] if direction=='up' else record['m4AngleDeg'],
            startAngleToleranceDeg=.1,maxObservedDeltaDeg=policy['maxObservedDeltaDeg'],maxSeconds=60.,maxObservationAgeSeconds=3.,postOffSeconds=3.,
            goalAngleDeg=policy['upGoalAngleDeg' if direction=='up' else 'downGoalAngleDeg'])
        scope['effectivePolicy' if effective else 'shortTaskPolicy']=policy
        # Validate before creating an output directory. Parent authorization binds
        # this policy and the exact FrozenBank decision, not an arbitrary pulse.
        from learning.m4_up_commissioning import validate_scope as validate_actor_scope
        validate_actor_scope(scope)
        store=ArtifactStore(self.root/f'M4-{serial:03d}');anchor=self._anchor(stage,record,store)
        slot={}
        def epoch():
            actor=slot['actor']
            if effective:return actor.read_armed_off_epoch(record['outputSequence'])
            snapshot=actor.port._status();actor.port._check_status(snapshot,'ARMED')
            actor.port._outputs();actor.port._queue(1)
            require(actor.port.last_outputs=={i:0 for i in range(1,6)},'factory_M4_current_OFF_required')
            return dict(controllerBootId=actor.port.boot_id,cameraBootId=ctx['gripperCameraBootId'],
                        outputSequence=actor.port.last_sequence,allFiveOff=True)
        reviewer=StationaryM4BeforeReviewer(scope,self.repo/'.local',anchor,store,epoch,
            self.after_reviewer_factory(scope,store),clock=self.clock)
        config=dict(helperPath=str(self.asset_root/self.assets['helper']['path']),helperSha256=self.assets['helper']['sha256'],cameraUrl=self.camera_url)
        observer=_ReadyObserver(scope,config,store,reviewer,resource_check=self.resource_check,clock=self.clock)
        actor=M4UpCommissioning(scope,observer,self.token,store,evidence_root=store.root,clock=self.clock,task_hooks=hooks)
        if effective:
            original_before=actor._before_run
            def before_run():
                original_before();reviewer.check_before_run()
            actor._before_run=before_run
        slot['actor']=actor
        self.actors.append(actor)
        store.write_json('parent-cycle-model-binding.json',dict(parentScopeSha256=plan_digest(self.scope),
            modelSha256=stage['modelSha256'],decisionSha256=hooks['decisionSha256'],
            sourceSha256=plan_digest(source_record(record)),childScopeSha256=plan_digest(scope),modelSelected=True))
        return actor,plan_digest(scope)

    def close(self):
        self.closed=True;self.token=''
        # Lifecycle cleanup belongs to runtime/source; revoke aliases only.
        for port in self.readers:port.token=''
        for manager in self.managers:manager.token=''
        for actor in self.actors:actor.port.token=''
