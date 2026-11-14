"""유한 과업의 실제 생산자 세 경로와 원본 감사를 연결한다.

주소·인증·카메라 기본값은 없다. 생성/preflight는 촬영하지 않는다. idle_factory는
OwnedDualSource를, general_factory는 동일 manager의 GripperObservationSource와
기존 CurrentM1CollectionGuard/CurrentTaskM3Guard를 반환한다. factory는 장치 생성
책임만 가지며 원본 파싱·시계·epoch·신선도 검증은 이 모듈이 직접 수행한다.
"""
import copy
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
import time
from functools import wraps

from learning.artifacts import _read, _decode_json
from learning.action_recorder import ActionRecorder
from learning.grasp_observation import GripperObservationSource
from learning.current_m1_contract import CurrentM1CollectionGuard
from learning.current_observation_contract import CurrentTaskM3Guard
from learning.development_gripper_source import load_observer, observe_bound_frame, scene_context
from learning.m1_shaft_scene import observe_bound_shaft_frame, scene_context as shaft_scene
from learning.registered_gripper_crosssection import RegisteredGripperCrosssection
from learning.m4_up_commissioning import _external_frame, _gripper_frame
from learning.manual_collection_dataset import require
from learning.supervised_transport import SequencePort
from learning.supervised_collection import plan_digest
from learning.task_cycle_workflow import FrozenCycleModelBank
from learning.task_visual_measurement import measure
from learning.finite_task_cycle import STAGES
from learning.current_cycle_phase import source_record
from learning.current_cycle_observation import metrology_review_features


class SourcePreflightError(ValueError):
    def __init__(self, missing):
        self.missing = copy.deepcopy(missing)
        super().__init__('full_source_missing:'+str(missing))


def _fail_closed(method):
    @wraps(method)
    def guarded(self,*args,**kwargs):
        try:return method(self,*args,**kwargs)
        except BaseException:
            self.failed=True
            raise
    return guarded


@dataclass
class OwnedDualSource:
    """독립 정지 촬영 소유자. root 아래 별도 폴더에 두 생산자를 둔다."""
    root: Path
    recorder: ActionRecorder
    source: GripperObservationSource
    port: SequencePort


class FullCycleSource:
    def __init__(self, root, *, bank, idle_factory, general_factory, phase, clock=time):
        require(callable(idle_factory) and callable(general_factory), 'full_source_factories_required')
        self.root=Path(root); self.bank=bank
        self.idle_factory,self.general_factory,self.phase=idle_factory,general_factory,phase
        self.clock=clock;self.scope=None;self.registrations={};self.idle=None
        self.generals={};self.evidence={};self.last_gate=None;self.closed=False;self.failed=False
        self.stationary={};self.pairs={};self.proofs={};self.refreshing=False
        self.phase.bind_source(self)

    def _bound(self, ref):
        require(isinstance(ref,dict) and set(ref)=={'path','sha256'}, 'full_source_asset_binding')
        raw=_read(self.root,ref['path'])
        require(sha256(raw).hexdigest()==ref['sha256'],'full_source_asset_changed')
        return raw

    def preflight(self, scope, assets):
        self.scope=None;self.registrations={}
        require(getattr(self.bank,'m1_observation_policies',{})==assets.get('m1ObservationPolicies',{}),
                'full_source_M1_admission_policy_changed')
        missing={}; registrations={};stationary={}
        for stage in scope['stages']:
            kind=stage['kind']; errors=[]
            try:
                require(isinstance(self.bank,FrozenCycleModelBank), 'frozen_model_missing')
                self.bank.verify_cached()
                model=self.bank.models[kind]
                binding=self.bank.bindings[kind]
                require(sha256(Path(binding['path']).read_bytes()).hexdigest()==binding['sha256'],
                        'frozen_model_file_changed')
                require(self.bank.hashes[kind]==stage['modelSha256'] and model['context']==stage['context'],
                        'frozen_model_stage_context')
            except (ValueError,KeyError,OSError) as exc: errors.append(str(exc))
            try:
                spec=assets.get('sourceStages',{}).get(kind)
                require(isinstance(spec,dict) and spec.get('context')==stage['context'], 'source_registration_missing')
                actual=spec['sourceContext']
                require(isinstance(actual,dict) and all(isinstance(actual.get(k),str) and actual[k]
                    for k in ('externalDeviceId','externalViewId','gripperCameraBootId','physicalView','object','poseRegime'))
                    and all(actual[k]==stage['context'].get(k) for k in ('physicalView','object','poseRegime')),
                    'source_actual_context_missing_or_changed')
                raw=self._bound(spec['registration']); registration=_decode_json(raw)
                if kind in ('approach','close','release'):
                    config=_decode_json(self._bound(spec['guardConfig']))
                    schema=('motionbrain.current_task_M3_guard.v1' if kind=='approach'
                            else 'motionbrain.current_M1_collection_guard.v1')
                    require(config.get('schemaVersion')==schema
                            and config.get('controllerBootId')==scope['controllerBootId'],
                            'source_guard_contract_missing_or_changed')
                if kind=='approach':
                    observer=load_observer(self.root,spec['registration']['path'],spec['registration']['sha256'])
                    registrations[kind]=(spec,observer)
                elif kind in ('close','release'):
                    observer=RegisteredGripperCrosssection(registration,self._bound(spec['reference']))
                    policy=self.bank.m1_observation_policies.get(kind)
                    if policy is not None:
                        from learning.current_m1_uncertainty import audit_calibration
                        require(policy['liveEligible'] is True,'M1_admission_calibration_pending')
                        audit_calibration(self.root,policy,model,raw,self._bound(spec['reference']),
                                          self._bound(spec['guardConfig']))
                    registrations[kind]=(spec,observer)
                else:
                    external=self._bound(spec['externalReference']); gripper=self._bound(spec['gripperReference'])
                    measure(external,gripper,registration)
                    state_registration=_decode_json(self._bound(spec['modelStateRegistration']))
                    require(state_registration.get('schemaVersion')=='motionbrain.task_visual_state_registration.v1'
                        and stage['context']==dict(state_registration['context'],registrationSha256=plan_digest(state_registration))
                        and actual in state_registration['acceptedSourceContexts']
                        and state_registration['measurementMethod']==registration['schemaVersion']+':'+plan_digest(registration),
                        'visual_model_state_measurement_contract_changed')
                    camera=state_registration.get('cameraBinding',{})
                    e_camera,g_camera=camera.get('external',{}),camera.get('gripper',{})
                    require(set(e_camera)=={'configurationSha256','width','height','videoMirrored','videoRotationAngle'}
                        and set(g_camera)=={'bootId','width','height'}
                        and g_camera['bootId']==actual['gripperCameraBootId']
                        and [e_camera['height'],e_camera['width'],3]==registration['imageShapes']['external']
                        and [g_camera['height'],g_camera['width'],3]==registration['imageShapes']['gripper'],
                        'visual_model_camera_registration_changed')
                    require(registration.get('referenceImages')==dict(externalSha256=sha256(external).hexdigest(),
                            gripperSha256=sha256(gripper).hexdigest()), 'visual_reference_binding')
                    registrations[kind]=(spec,registration)
                stationary[kind]=_decode_json(self._bound(spec['stationaryRegistration']))
                # A standalone numeric template is insufficient: prove that the
                # source-bound stationary references actually replay it.
                measure(self._bound(spec['stationaryExternalReference']),
                        self._bound(spec['stationaryGripperReference']),stationary[kind])
                require(stationary[kind].get('referenceImages')==dict(
                    externalSha256=spec['stationaryExternalReference']['sha256'],
                    gripperSha256=spec['stationaryGripperReference']['sha256']),
                    'stationary_reference_binding_changed')
            except (ValueError,KeyError,OSError,RuntimeError) as exc: errors.append(str(exc))
            if errors: missing[kind]=errors
        if missing: raise SourcePreflightError(missing)
        self.scope=copy.deepcopy(scope);self.assets=copy.deepcopy(assets);self.registrations=registrations
        self.stationary=stationary
        return dict(scopeSha256=plan_digest(scope),assetsSha256=plan_digest(assets),supportedStages=list(STAGES))

    def _ready(self):
        require(self.scope is not None and not self.closed and not self.failed, 'full_source_preflight_or_failed')

    def _status(self,port,state):
        status=port._status();port._check_status(status,state);port._outputs()
        port._queue(1 if state=='ARMED' else 0)
        require(port.boot_id==self.scope['controllerBootId'] and port.last_outputs=={i:0 for i in range(1,6)},
                'full_source_current_OFF_or_boot')
        return dict(sequence=port.last_sequence,angle=port._shoulder_angle(status),state=state,
                    bootId=port.boot_id,allFiveOff=True,observedAt=self.clock.monotonic())

    @staticmethod
    def _same_epoch(before,after):
        require(all(before[k]==after[k] for k in ('bootId','sequence','state','allFiveOff')),
                'full_source_epoch_changed')
        # Existing current M1/M3 guards admit ±.25°. This only separates sensor
        # jitter from output identity; guard absolute/reference limits remain.
        require(abs(before['angle']-after['angle'])<=.25,'full_source_stationary_angle_changed')

    def _ownership(self,recorder,source):
        require(isinstance(recorder,ActionRecorder) and isinstance(source,GripperObservationSource)
                and recorder.clock is self.clock and source.clock is self.clock
                and recorder.host_clock_id==source.session_id,
                'full_source_owned_same_python_clock_required')
        recorder.check();source.check()

    def _refs(self,root,recorder,source,record):
        root=Path(root)
        with recorder.lock: frame=copy.deepcopy(recorder.frames[-1])
        external_root=str(recorder.root.relative_to(root))
        ingress=f"host-ingress/{frame['frameIndex']:04d}.json"
        external=dict(root=external_root,ingressFile=ingress,
                      ingressSha256=sha256(_read(root,external_root+'/'+ingress)).hexdigest(),
                      clockSha256=sha256(_read(root,external_root+'/host-ingress/clock-before.json')).hexdigest())
        binding=source._bindings[record['frame']['frameId']]
        folder=source.store.root/binding['folder']
        gripper=dict(root=str(folder.relative_to(root)),label='frame',
                     metadataSha256=binding['metadataSha256'],imageSha256=binding['imageSha256'])
        return external,gripper

    def _scene(self,stage,record):
        actual=self.registrations[stage['kind']][0]['sourceContext']
        require(all(record['context'].get(k)==actual[k] for k in ('physicalView','object','poseRegime')),
                'full_source_producer_scene_changed')

    def _audit(self,stage,root,external,gripper,producer_id,angle,*,audit_at=None):
        spec,observer=self.registrations[stage['kind']]
        rawscope=dict(hostClockId=producer_id,context=spec['sourceContext'])
        # Raw producers keep their own session IDs. The cycle ID is a separate
        # envelope, justified by the live producer clock object identity.
        ext=_external_frame(root,external,rawscope)
        grip=_gripper_frame(root,gripper,rawscope,gripper['label'])
        location=Path(root)/gripper['root'];kind=stage['kind']
        if kind=='approach':
            measured=observe_bound_frame(observer,location,gripper['label'],session_id=producer_id,
                camera_boot_id=spec['sourceContext']['gripperCameraBootId'],actual_scene=scene_context(observer),
                now=self.clock.monotonic() if audit_at is None else audit_at,max_age_seconds=3.)
            require(measured.get('state') is not None,'full_source_relative_measurement_rejected')
            require(stage['context'].get('landmarks')==measured['observationContract']
                    and stage['context'].get('stateFields')==measured['stateFields']
                    and stage['context'].get('cameraIdentity',{}).get('cameraBootId')==measured['cameraBootId']
                    and stage['context'].get('imageShape')==[grip['height'],grip['width']]
                    and all(scene_context(observer)[k]==spec['sourceContext'][k]
                            for k in ('physicalView','object','poseRegime')),
                    'full_source_relative_model_registration_changed')
            state=measured['state']
        elif kind in ('close','release'):
            measured=observe_bound_shaft_frame(observer,location,gripper['label'],session_id=producer_id,
                camera_boot_id=spec['sourceContext']['gripperCameraBootId'],actual_scene=shaft_scene(observer),
                now=self.clock.monotonic() if audit_at is None else audit_at)
            state=dict(shaftSeparationPx=measured['shaftSeparationPx'])
            require(measured['context']==stage['context'],'full_source_shaft_model_context_changed')
        else:
            model_registration=_decode_json(self._bound(spec['modelStateRegistration']))
            camera_binding=dict(external={k:ext[k] for k in
                ('configurationSha256','width','height','videoMirrored','videoRotationAngle')},
                gripper=dict(bootId=spec['sourceContext']['gripperCameraBootId'],width=grip['width'],height=grip['height']))
            require(camera_binding==model_registration['cameraBinding'],'full_source_model_camera_binding_changed')
            ingress=_decode_json(_read(root,external['root']+'/'+external['ingressFile']))
            # Metrology takes encoded original pixels. Native frames are lossless
            # BGRA blobs, so convert the decoded original to PNG in memory only.
            from learning.lossless_frame import decode
            import cv2,numpy as np
            pixels=decode(_read(root,external['root']+'/'+ingress['imageFile']),ingress['rawByteLength'],ingress['pixelSha256'])
            image=np.frombuffer(pixels,np.uint8).reshape(ext['height'],ext['width'],4)
            ok,png=cv2.imencode('.png',image);require(ok,'full_source_png_encode')
            measured=measure(png.tobytes(),_read(root,gripper['root']+'/esp32-'+gripper['label']+'.jpg'),observer)
            state=dict(measured['state'],startAngleDeg=angle)
        frames=[dict(camera=c,**{k:f[k] for k in ('frameId','pixelSha256','sourceEarliest','sourceLatest','persistedAt')})
                for c,f in [('external',ext),('gripper',grip)]]
        return frames,state,measured

    def _visual(self,root,external,gripper,registration):
        from learning.lossless_frame import decode
        import cv2,numpy as np
        ingress=_decode_json(_read(root,external['root']+'/'+external['ingressFile']))
        raw=decode(_read(root,external['root']+'/'+ingress['imageFile']),ingress['rawByteLength'],ingress['pixelSha256'])
        image=np.frombuffer(raw,np.uint8).reshape(ingress['height'],ingress['width'],4)
        ok,png=cv2.imencode('.png',image);require(ok,'full_source_png_encode')
        return measure(png.tobytes(),_read(root,gripper['root']+'/esp32-'+gripper['label']+'.jpg'),registration)

    def _record(self,stage,root,external,gripper,producer_id,epoch,route,owner,after_off,deadline):
        frames,state,measured=self._audit(stage,root,external,gripper,producer_id,epoch['angle'])
        now=self.clock.monotonic()
        require(now<deadline and all(0<=now-f['sourceEarliest']<=3 for f in frames),'full_source_pair_stale')
        if after_off is not None:
            require(all(f['sourceEarliest']>=after_off+3. for f in frames),'full_source_after_OFF_window')
        record=dict(context=copy.deepcopy(stage['context']),hostClockId=self.scope['hostClockId'],
                    producerHostClockId=producer_id,controllerBootId=self.scope['controllerBootId'],
                    frames=frames,state=state,m4AngleDeg=epoch['angle'],stable=True,
                    outputSequence=epoch['sequence'],sourceRoute=route,auditedAt=now)
        policy=getattr(self.bank,'m1_observation_policies',{}).get(stage['kind'])
        if policy is not None:
            from learning.pipeline_learning import digest
            record['m1ObservationPolicySha256']=digest(policy)
        # Actual source scene declaration is checked by the existing bound parser;
        # its registered context must also exactly match the model asset above.
        key=plan_digest(source_record(record))
        self.evidence[key]=dict(record=copy.deepcopy(record),stage=copy.deepcopy(stage),root=Path(root),
            external=external,gripper=gripper,producer=producer_id,epoch=epoch,route=route,owner=owner,
            measured=measured,after_off=after_off)
        if self.refreshing:return record
        def tick():
            if route=='idle':self._ownership(owner.recorder,owner.source)
            elif route.startswith('general_'):owner.tick()
            else:owner.observer.check()
        record,entry=self.phase.resolve(stage,copy.deepcopy(record),deadline=deadline,tick=tick)
        record['entry']=entry
        entry=record['entry']
        if entry.get('tableSupported') is True and entry.get('airborne') is not True:
            record['supportMode']='table_supported'
        elif entry.get('airborne') is True and entry.get('bilateralContact') is True and entry.get('tableSupported') is not True:
            record['supportMode']='gripper_supported'
        else:
            require(stage['kind'] not in ('lift','lower'),'full_source_support_ambiguous')
        return record

    @_fail_closed
    def observe_idle(self,stage,*,after_off,deadline):
        self._ready()
        if self.idle is None:
            self.idle=self.idle_factory(copy.deepcopy(stage))
            require(isinstance(self.idle,OwnedDualSource),'full_source_actual_idle_owner')
            self.idle.recorder.start();self.idle.recorder.wait_ready(timeout=min(15.,deadline-self.clock.monotonic()))
            self.idle.source.start(deadline=deadline)
        owner=self.idle;self._ownership(owner.recorder,owner.source)
        before=self._status(owner.port,'IDLE');raw=owner.source.capture(deadline=deadline)
        self._scene(stage,raw)
        owner.source.verify(raw);external,gripper=self._refs(owner.root,owner.recorder,owner.source,raw)
        after=self._status(owner.port,'IDLE')
        self._same_epoch(before,after)
        return self._record(stage,owner.root,external,gripper,owner.source.session_id,after,'idle',owner,
                            after_off,deadline)

    def _general(self,stage,manager,after_off,deadline,armed):
        self._ready();key=id(manager)
        if key not in self.generals:
            source,guard=self.general_factory(copy.deepcopy(stage),manager)
            expected=CurrentTaskM3Guard if stage['kind']=='approach' else CurrentM1CollectionGuard
            require(isinstance(guard,expected) and guard.recorder is manager.camera and guard.port is manager.port,
                    'full_source_actual_general_guard')
            source.start(deadline=deadline);self.generals[key]=(manager,source,guard)
        _,source,guard=self.generals[key];self._ownership(manager.camera,source)
        if not armed:guard.disarm()
        template=_decode_json(self._bound(self.registrations[stage['kind']][0]['guardConfig']))
        require({k:v for k,v in guard.config.items() if k!='hostClockId'}==
                {k:v for k,v in template.items() if k!='hostClockId'}
                and guard.config['hostClockId']==source.session_id,
                'full_source_general_guard_binding_changed')
        if stage['kind']=='approach':
            require(guard.config['executionTimingPolicy']==stage['context'].get('executionTimingPolicy'),
                    'full_source_M3_model_timing_changed')
        before=self._status(manager.port,'ARMED' if armed else 'IDLE')
        raw=source.capture(deadline=deadline);source.verify(raw)
        self._scene(stage,raw)
        external,gripper=self._refs(manager.store.root,manager.camera,source,raw)
        _,_,measured=self._audit(stage,manager.store.root,external,gripper,source.session_id,before['angle'])
        if armed:
            guard.review_armed(raw,measured,expected_sequence=before['sequence'],deadline=deadline,tick=manager.camera.check)
        else: guard.review(raw,measured,deadline=deadline,tick=manager.tick)
        after=self._status(manager.port,'ARMED' if armed else 'IDLE')
        self._same_epoch(before,after)
        record=self._record(stage,manager.store.root,external,gripper,source.session_id,after,
                            'general_armed' if armed else 'general_after',manager,after_off,deadline)
        if armed:self.last_gate=(manager,guard,source,copy.deepcopy(record),plan_digest(record))
        return record

    @_fail_closed
    def observe_general_armed(self,stage,manager,*,after_off,deadline):
        return self._general(stage,manager,after_off,deadline,True)

    @_fail_closed
    def observe_general_after(self,stage,manager,receipt,*,deadline):
        return self._general(stage,manager,receipt['offAt'],deadline,False)

    @_fail_closed
    def observe_m4_armed(self,stage,actor,*,after_off,deadline):
        self._ready();observer=actor.observer
        self._ownership(observer.recorder,observer.gripper)
        epoch=self._status(actor.port,'ARMED')
        return self._record(stage,actor.root,actor.before['external'],actor.before['gripper'],
            actor.scope['hostClockId'],epoch,'M4_armed',actor,after_off,deadline)

    @_fail_closed
    def verify_observation(self,stage,record):
        evidence=self.evidence.get(plan_digest(source_record(record)));require(evidence is not None,'full_source_unowned_record')
        require(stage==evidence['stage'] and source_record(record)==evidence['record'],'full_source_record_changed')
        frames,state,measured=self._audit(stage,evidence['root'],evidence['external'],evidence['gripper'],
            evidence['producer'],evidence['epoch']['angle'],audit_at=record['auditedAt'])
        return frames==record['frames'] and state==record['state']

    def verify_clock_bridge(self,stage,record,route,owner):
        evidence=self.evidence.get(plan_digest(source_record(record)))
        require(evidence is not None and evidence['route']==route and evidence['stage']==stage,
                'full_source_clock_route_changed')
        require(evidence['owner'] is (self.idle if route=='idle' else owner),'full_source_clock_owner_changed')
        actual=evidence['owner']
        if route=='idle':self._ownership(actual.recorder,actual.source)
        elif route.startswith('general_'):
            self._ownership(actual.camera,self.generals[id(actual)][1])
        else:self._ownership(actual.observer.recorder,actual.observer.gripper)
        return self.verify_observation(stage,record)

    def describe_source(self,record):
        evidence=self.evidence[plan_digest(source_record(record))]
        return dict(root=str(evidence['root']),external=copy.deepcopy(evidence['external']),
                    gripper=copy.deepcopy(evidence['gripper']),producerHostClockId=evidence['producer'])

    def stationary_pair(self,stage,record):
        require(self.verify_observation(stage,record) is True,'full_source_stationary_raw_audit')
        key=plan_digest(source_record(record));evidence=self.evidence[key]
        measured=self._visual(evidence['root'],evidence['external'],evidence['gripper'],self.stationary[stage['kind']])
        pair=dict(context=copy.deepcopy(record['context']),controllerBootId=record['controllerBootId'],
            cameraBootId=self.registrations[stage['kind']][0]['sourceContext']['gripperCameraBootId'],
            outputSequence=record['outputSequence'],frames=copy.deepcopy(record['frames']),
            reviewFeatures=metrology_review_features(measured,record['m4AngleDeg']),sourceSha256=key)
        self.pairs[plan_digest(pair)]=(copy.deepcopy(pair),copy.deepcopy(stage),copy.deepcopy(record))
        return pair

    def audit_stationary_pair(self,pair):
        candidate={k:v for k,v in pair.items() if k!='outputProofObservedAt'}
        owned=self.pairs.get(plan_digest(candidate))
        require(owned is not None and candidate==owned[0],'full_source_unowned_stationary_pair')
        return self.stationary_pair(owned[1],owned[2])==candidate

    def zero_output_bridge(self,anchor,current):
        evidence=self.evidence[plan_digest(source_record(current))];owner=evidence['owner']
        port=owner.port
        require(isinstance(port,SequencePort),'full_source_real_cursor_port_required')
        started=port.serial;epoch=self._status(port,evidence['epoch']['state'])
        require(epoch['sequence']==current['outputSequence'],'full_source_cursor_changed_after_pair')
        require(abs(epoch['angle']-current['m4AngleDeg'])<=.25,'full_source_bridge_sensor_changed')
        entries=[]
        for sequence in range(anchor['outputSequence']+1,current['outputSequence']+1):
            require(sequence in port._known_entries,'full_source_cursor_history_missing')
            entries.append(copy.deepcopy(port._known_entries[sequence]))
        bindings=[]
        for serial in range(started+1,port.serial+1):
            for suffix in ('intent','response'):
                name=f'http-{serial:06d}-{suffix}.json';raw=_read(port.store.root,name)
                if suffix=='intent':require(_decode_json(raw).get('payload') is None,'full_source_bridge_write_forbidden')
                bindings.append(dict(path=name,sha256=sha256(raw).hexdigest()))
        require(bindings,'full_source_cursor_raw_HTTP_required')
        proof=dict(controllerBootId=port.boot_id,beforeSequence=anchor['outputSequence'],
            afterSequence=current['outputSequence'],dropped=0,allFiveOff=True,state=epoch['state'],
            queueCount=1 if epoch['state']=='ARMED' else 0,entries=entries,observedAt=epoch['observedAt'],
            httpBindings=bindings)
        self.proofs[plan_digest(proof)]=(port,copy.deepcopy(proof))
        return proof

    def audit_zero_bridge(self,proof):
        owned=self.proofs.get(plan_digest(proof));require(owned is not None,'full_source_unowned_zero_proof')
        port,original=owned;require(proof==original,'full_source_zero_proof_changed')
        for binding in proof['httpBindings']:
            require(sha256(_read(port.store.root,binding['path'])).hexdigest()==binding['sha256'],
                    'full_source_cursor_HTTP_changed')
        return True

    def refresh_record(self,stage,record,*,deadline,not_before=None):
        evidence=self.evidence[plan_digest(source_record(record))]
        require(not self.refreshing and evidence['route'] in ('idle','general_after'),
                'full_source_single_seed_refresh_only')
        self.refreshing=True
        try:
            if not_before is not None:
                owner=evidence['owner']
                recorder=owner.recorder if evidence['route']=='idle' else owner.camera
                while True:
                    require(self.clock.monotonic()<deadline,'full_source_hold_capture_deadline')
                    recorder.check()
                    with recorder.lock:
                        ready=bool(recorder.frames) and recorder.frames[-1]['sourceEarliest']>=not_before
                    if ready and self.clock.monotonic()>=not_before:break
                    if evidence['route']=='general_after':owner.tick()
                    self.clock.sleep(min(.05,deadline-self.clock.monotonic()))
            if evidence['route']=='idle':return self.observe_idle(stage,after_off=evidence['after_off'],deadline=deadline)
            return self._general(stage,evidence['owner'],evidence['after_off'],deadline,False)
        finally:self.refreshing=False

    def validate_general_memory(self,stage,manager,command):
        require(self.last_gate is not None,'full_source_ARM_gate_missing')
        owned,guard,source,record,digest=self.last_gate
        require(owned is manager and digest==plan_digest(record)
                and record['context']==stage['context'] and manager.port.last_sequence==record['outputSequence']
                and manager.port.last_outputs=={i:0 for i in range(1,6)}
                and all(0<=self.clock.monotonic()-f['sourceEarliest']<=3 for f in record['frames']),
                'full_source_memory_epoch_or_freshness')
        source.check();manager.camera.check();guard.check();guard.validate_command(command)
        self.last_gate=None

    def close_idle(self):
        if self.idle is not None:
            owner=self.idle;self.idle=None
            try:owner.source.close()
            finally:owner.recorder.close()

    def close_general(self,manager):
        owned=self.generals.pop(id(manager),None);self.last_gate=None
        if owned:
            try:owned[2].disarm()
            finally:owned[1].close()

    def close_m4(self,actor):
        # M4 actor owns recorder/source shutdown, including failed execution.
        self.last_gate=None

    def close(self):
        try:self.close_idle()
        finally:
            try:
                for manager,_,_ in list(self.generals.values()):self.close_general(manager)
            finally:self.closed=True
