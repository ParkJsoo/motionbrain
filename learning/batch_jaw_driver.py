"""고정 M1 빈턱 수집 driver. 명령은 고정 규칙이며 모델 선택 제어가 아니다.

생성·검증·reconcile은 장치에 접근하지 않는다. 최초 유한 job 현장 확인을
요구하고, 각 실제 명령은 기존 ManualCollection의 OFF/원장 경계를 재사용한다.
"""
from __future__ import annotations
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import time
import urllib.request
import uuid

from learning.artifacts import ArtifactStore
from learning.batch_campaign import digest, encoded, require, validate_plan
from learning.batch_runtime import journal_clear, private_path
from learning.grasp_observation import GripperObservationSource
from learning.gripper_camera_dataset import read_frame
from learning.manual_collection import ManualCollection, RecordingPort
from learning.pipeline_collection import collect_once
from learning.registered_gripper_landmarks import RegisteredGripperLandmarks
from learning.registered_jaw_transition import build_transition
from learning.supervised_transport import SequencePort

AUTH_SCHEMA='motionbrain.manual_jaw_job_authorization.v1'
ONSITE={'emptyJaws','fixedBodyAndMount','otherMotorsOff','workspaceClear','noEntryDuringJob'}
PROFILE=dict(frameSize='vga',frameWidth=640,frameHeight=480,jpegQuality=15)
RESTORE=dict(frameSize='qvga',frameWidth=320,frameHeight=240,jpegQuality=15)


def sha(raw):return hashlib.sha256(raw).hexdigest()

def read_binding(repository,binding):
    require(isinstance(binding,dict) and set(binding)=={'path','sha256'},'jaw_file_binding')
    raw=private_path(repository,binding['path']).read_bytes()
    require(sha(raw)==binding['sha256'],'jaw_binding_changed')
    return raw


def driver_config_digest(config):
    return digest({k:v for k,v in config.items() if k not in ('plan','root','authorization')})


def check_state(observed,command=None):
    state=observed['state'];quality=observed['quality']
    require(len(state)==5 and 60<=state[-1]<=116,'jaw_operating_gap')
    require(all(lo<=v<=hi for v,lo,hi in zip(state[:4],[258,267,334,266],[302,311,378,310])),
            'jaw_operating_points')
    require(min(24-abs(quality[n]['shiftPx'][axis]) for n in ('jawLeft','jawRight') for axis in (0,1))>=2,
            'jaw_search_margin')
    direction='open' if state[-1]<86 else 'close'
    if command is not None:
        require(command==dict(joint='gripper',direction=direction,percent=40,durationMs=350),'jaw_manual_direction_rule')
        left,right=(quality[n]['shiftPx'][0] for n in ('jawLeft','jawRight'))
        travel=[24+left,24-right] if direction=='open' else [24-left,24+right]
        require(min(travel)-2>=14,'jaw_directional_search_margin')
    return direction


def check_response(transition):
    """관측된 뜻밖의 반응도 정답으로 남기고 후속 동작만 멈춘다."""
    row=transition['learningRow']
    try:
        check_state(transition['observations']['after'])
        delta=row['after'][-1]-row['before'][-1]
        require(abs(delta)>6,'jaw_unresolved_response')
        require(delta>0 if row['command']['direction']=='open' else delta<0,'jaw_opposite_response')
    except (ValueError,KeyError,TypeError) as exc:return str(exc)
    return None


def verify_qualification(repository,binding,registration,reference,context):
    """기존 독립 raw 라벨 자료를 재감사한다. 새 정확도/전체범위 보장이 아니다."""
    value=json.loads(read_binding(repository,binding));sources=value.get('sources',[])
    require(len(sources)>=8,'jaw_eight_labelled_sources_required')
    seen=set();counts={'open':0,'close':0}
    for entry in sources:
        require(entry['sourceSha256'] not in seen,'jaw_qualification_duplicate')
        seen.add(entry['sourceSha256'])
        source=private_path(repository,entry['sourcePath'])
        transition=build_transition(source,entry['sourceSha256'],registration,reference,context,'qualification',phase='development')
        row=transition['learningRow']
        require(row['command'] in [dict(joint='gripper',direction=d,percent=40,durationMs=350) for d in ('open','close')],'jaw_qualification_command')
        for observed in transition['observations'].values():check_state(observed)
        labels=json.loads(read_binding(repository,entry['labels']))
        require(labels.get('trackerValuesViewedBeforeSeal') is False and labels.get('identityVerified') is True
            and labels.get('sourceSha256')==entry['sourceSha256'] and labels.get('stateFields')==row['stateFields']
            and labels.get('labelTolerancePx')==3,'jaw_blind_label_contract')
        for side in ('before','after'):
            label=labels['labels'][side]
            require(label['imageSha256']==sha((source/'gripper-pair'/('esp32-'+side+'.jpg')).read_bytes()),'jaw_label_image_changed')
            manual=label['state'];require(len(manual)==5 and all(abs(a-b)<=3 for a,b in zip(manual,row[side])),
                'jaw_manual_label_disagreement')
        delta=labels['labels']['after']['state'][-1]-labels['labels']['before']['state'][-1]
        require(abs(delta)>6 and (delta>0 if row['command']['direction']=='open' else delta<0),'jaw_qualification_motion_unresolved')
        counts[row['command']['direction']]+=1
    require(min(counts.values())>=4,'jaw_qualification_both_directions')
    return dict(sources=len(seen),counts=counts,physicalAccuracyEstablished=False)


class ReadOnlyPort(SequencePort):
    def _call(self,path,payload=None,**kwargs):
        require(payload is None,'jaw_probe_controller_write_forbidden')
        return super()._call(path,payload,**kwargs)


class JawDriver:
    def __init__(self,repository,config,phase,frozen=None,*,clock=time):
        self.repository=Path(repository).resolve();self.config=copy.deepcopy(config);self.phase=phase;self.clock=clock
        self.plan=config['plan'];require(phase in ('training','evaluation'),'jaw_phase')
        checked=validate_plan(self.plan);require(not checked['rejected'],'jaw_plan_conditions')
        self.root=private_path(self.repository,config['root']);self.journal=self.repository/'.local/visual-actions/controller-commands.sqlite'
        self.plan_sha=digest(self.plan);self.config_sha=driver_config_digest(config)
        self.registration=json.loads(read_binding(self.repository,config['gripper']['registration']))
        self.reference=read_binding(self.repository,config['gripper']['reference'])
        self.helper=private_path(self.repository,config['helper']['path']);read_binding(self.repository,config['helper'])
        self.tracker=RegisteredGripperLandmarks(self.registration,self.reference)
        p=self.registration['parameters']
        require(self.registration['imageShape']==[480,640] and all(p['searchRadii'][n]==[24,24] for n in ('jawLeft','jawRight'))
            and p['minimumNcc']['jawLeft']==p['minimumNcc']['jawRight']==.94 and p['minimumPeakGap']==.04
            and p['minimumOpeningPx']==60 and p['maximumOpeningPx']==120,'jaw_verified_registration_required')
        self.context=copy.deepcopy(config['context'])
        require(self.context.get('object')=='none' and self.context.get('load')=='empty'
            and self.context.get('viewId')==self.plan['viewId'],'jaw_empty_fixed_context')
        require(config['external']==dict(width=1920,height=1080,videoRotationAngle=0,videoMirrored=False,
            configurationSha256=config['external'].get('configurationSha256')),'jaw_external_format')
        self.auth=json.loads(read_binding(self.repository,config['authorization']))
        self._authorization()
        self.qualification=verify_qualification(self.repository,config['qualification'],self.registration,self.reference,self.context)
        self.frozen=copy.deepcopy(frozen);self.model=None;self.freeze=None
        if phase=='evaluation':
            from learning.batch_fixed_evaluation import load_freeze
            require(isinstance(frozen,dict) and 'freeze' in frozen,'jaw_frozen_model_required')
            self.freeze,self.model=load_freeze(self.repository,frozen['freeze'])
            require(self.model['context']==self.model_context(),'jaw_frozen_context')
        self.profile_owned=False;self.manager=None;self.latest=None;self.deadline=None

    def model_context(self):
        return dict(self.context,observerSha256=digest(dict(kind='registered_jaw_points',registration=self.registration,context=self.context)),
                    gripperCameraIdentity='gripper:'+self.config['gripper']['bootId'])

    def _authorization(self):
        a=self.auth
        require(a.get('schemaVersion')==AUTH_SCHEMA and a.get('driverConfigSha256')==self.config_sha
            and self.plan_sha in a.get('planHashes',[]) and a.get('controllerBootId')==self.plan['controllerBootId'],
            'jaw_job_authorization_binding')
        require(a.get('onsite')=={k:True for k in ONSITE},'jaw_initial_onsite_confirmation')
        require(type(a.get('maxPhysicalCommands')) is int and 0<a['maxPhysicalCommands']<=1000
            and type(a.get('expiresAt')) in (int,float),'jaw_authorization_bounds')
        parent=private_path(self.repository,a['outputRoot'])
        require(self.root.is_relative_to(parent),'jaw_authorized_output_root')

    def _verify(self):
        require(json.loads(read_binding(self.repository,self.config['authorization']))==self.auth,'jaw_authorization_changed')
        self._authorization()
        require(self.clock.time()<self.auth['expiresAt'],'jaw_job_authorization_expired')
        require(self.deadline is None or self.clock.time()<self.deadline,'jaw_job_deadline')
        for binding in (self.config['helper'],self.config['gripper']['registration'],self.config['gripper']['reference'],self.config['qualification']):
            read_binding(self.repository,binding)
        if self.frozen:
            from learning.batch_fixed_evaluation import load_freeze
            load_freeze(self.repository,self.frozen['freeze'])

    def _reserve(self,request_id):
        root=self.repository/'.local/jaw-job-reservations';root.mkdir(parents=True,exist_ok=True)
        db=sqlite3.connect(root/(self.config['authorization']['sha256']+'.sqlite'),timeout=2)
        try:
            db.execute('PRAGMA synchronous=FULL');db.execute('CREATE TABLE IF NOT EXISTS reservations (id TEXT PRIMARY KEY)')
            db.execute('BEGIN IMMEDIATE')
            require(db.execute('SELECT COUNT(*) FROM reservations').fetchone()[0]<self.auth['maxPhysicalCommands'],'jaw_authorization_budget_exhausted')
            require(db.execute('SELECT 1 FROM reservations WHERE id=?',(request_id,)).fetchone() is None,'jaw_request_consumed')
            db.execute('INSERT INTO reservations VALUES (?)',(request_id,));db.commit()
        finally:db.close()

    def condition_support(self,condition):
        command=condition.get('command',{})
        return None if (condition.get('phase')==self.phase and command in [dict(joint='gripper',direction=d,percent=40,durationMs=350) for d in ('open','close')]
            and condition.get('fromNodes')==[command['direction']]) else 'jaw_fixed_manual_condition_required'

    def _profile(self,desired=None):
        suffix='' if desired is None else '?framesize='+desired['frameSize']+'&quality=15'
        url=self.config['gripper']['url'].rstrip('/')+'/camera'+suffix
        request=urllib.request.Request(url,method='GET' if desired is None else 'POST')
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(request,timeout=3) as response:value=json.loads(response.read())
        if desired:require(all(value.get(k)==v for k,v in desired.items()),'jaw_camera_profile_changed')
        return value

    def _ready_profile(self):
        current=self._profile()
        if not self.profile_owned:
            require(all(current.get(k)==v for k,v in RESTORE.items()),'jaw_original_profile_required')
            self.profile_owned=True
            self._profile(PROFILE)
        else:require(all(current.get(k)==v for k,v in PROFILE.items()),'jaw_camera_profile_changed')

    def _gripper(self,session,store,max_captures=2):
        return GripperObservationSource(self.config['gripper']['url'],session,self.config['gripper']['bootId'],
            dict(physicalView=self.context['viewId'],object='none',poseRegime=self.context['poseRegime'],landmarks=self.tracker.landmark_contract),
            store,max_captures=max_captures,clock=self.clock)

    def observe(self):
        self._verify();require(journal_clear(self.journal),'jaw_shared_journal_blocked');self._ready_profile()
        sid='jaw-observe-'+uuid.uuid4().hex;store=ArtifactStore(self.root/'observations'/sid)
        port=ReadOnlyPort(self.config['controllerUrl'],'unused-no-write',self.plan['controllerBootId'],store,clock=self.clock,compact_reads=True,timeout=2)
        port.preflight();source=self._gripper(sid,ArtifactStore(store.root/'gripper'),1)
        try:
            source.start(deadline=self.clock.monotonic()+6);record=source.capture(deadline=self.clock.monotonic()+6);source.verify(record)
            binding=source._bindings[record['frame']['frameId']]
            raw=(source.store.root/binding['folder']/'esp32-frame.jpg').read_bytes()
            observed=self.tracker.observe_jaws(raw,record['frame']['frameId']);node=check_state(observed)
            port.check_idle();require(port.off_verified,'jaw_probe_off')
            self.latest=observed
            return dict(nodes=[node],bootId=port.boot_id,viewId=self.plan['viewId'],offVerified=True,journalClear=journal_clear(self.journal),observedAt=record['sourceEarliest'])
        finally:source.close()

    def _external(self,folder,frame):
        folder=Path(folder).resolve();image=(folder/frame['imageFile']).resolve();meta=(folder/frame['metadataFile']).resolve()
        require(image.is_relative_to(folder) and meta.is_relative_to(folder),'jaw_raw_path')
        require(sha(image.read_bytes())==frame['imageSha256'] and sha(meta.read_bytes())==frame['metadataSha256'],'jaw_raw_hash')
        m=json.loads(meta.read_bytes());expected=dict(self.config['external'],deviceUniqueId=self.config['deviceId'],viewConfigurationId=self.plan['viewId'])
        require(all(m.get(k)==v for k,v in expected.items()),'jaw_raw_camera_context')

    def _scope(self):
        return dict(kind='bounded_manual_jaw_job',authorization=self.config['authorization'],
            phasePlanSha256=self.plan_sha,driverConfigSha256=self.config_sha,phase=self.phase,
            root=str(self.root),qualification=self.config['qualification'],modelSelected=False)

    def _before(self,manager,pair,condition):
        self._verify();manager.port.check_idle();frame=manager.camera.frames[-1]
        require(manager.command_count==0 and not any(e.get('kind')=='command' for e in manager.current['events']),
            'jaw_scope_must_precede_command')
        scope=dict(scope=self._scope(),sessionId=manager.current['sessionId'],at=self.clock.monotonic())
        manager.store.write_json('commissioning-validation-scope.json',scope)
        sealed=sha((manager.store.root/'commissioning-validation-scope.json').read_bytes())
        manager.current['validationScopeSha256']=sealed
        manager._event(dict(kind='commissioning_validation_scope',at=scope['at'],sha256=sealed))
        self._external(manager.camera.root,frame)
        grip,meta,boot=read_frame(manager.store.root/'gripper-pair','before',manager.current['sessionId'])
        require(boot==self.config['gripper']['bootId'] and grip['width']==640 and grip['height']==480,'jaw_gripper_boot_or_shape')
        raw=(manager.store.root/'gripper-pair/esp32-before.jpg').read_bytes()
        observed=self.tracker.observe_jaws(raw,grip['frameId'])
        require(observed['pixelSha256']==grip['pixelSha256'],'jaw_before_pixels')
        check_state(observed,condition['command'])
        require(self.latest is not None and all(abs(a-b)<=2 for a,b in zip(self.latest['state'],observed['state'])),'jaw_observe_before_changed')
        self.cached=dict(folder=manager.camera.root,frame=frame,gripFolder=manager.store.root/'gripper-pair',
                         sessionId=manager.current['sessionId'],grip=grip,meta=meta)
        if self.phase=='evaluation':
            from learning.batch_fixed_evaluation import prospective_evidence,seal_before
            row=dict(sessionId=manager.current['sessionId'],stateFields=observed['stateFields'],before=observed['state'],
                command=condition['command'],context=self.model_context(),beforePersistedAt=meta['persistedAt'])
            frames=[dict(frameId=grip['frameId'],pixelSha256=grip['pixelSha256'],capturedAt=meta['requestStartedAt'],persistedAt=meta['persistedAt']),
                dict(frameId=frame['frameId'],pixelSha256=frame['pixelSha256'],capturedAt=frame['sourceEarliest'],persistedAt=frame['stdoutReadAt']+frame['persistenceMs']/1000)]
            seal_before(manager,prospective_evidence(self.model,self.freeze,row,frames,self.clock.monotonic()))
        self._guard(manager)

    def _guard(self,manager):
        self._verify();manager.camera.check();self._ready_profile()
        require(manager.port.boot_id==self.plan['controllerBootId'],'jaw_boot_changed')
        c=self.cached;self._external(c['folder'],c['frame'])
        grip,meta,_=read_frame(c['gripFolder'],'before',c['sessionId'])
        require(grip==c['grip'] and meta==c['meta'],'jaw_before_gripper_changed')
        now=self.clock.monotonic()
        require(0<=now-c['frame']['sourceEarliest']<=3 and 0<=now-meta['requestStartedAt']<=10,'jaw_dispatch_stale')

    def _save(self,path,value):
        path.parent.mkdir(parents=True,exist_ok=True);raw=encoded(value)
        if path.exists():require(path.read_bytes()==raw,'jaw_saved_evidence_changed')
        else:
            from learning.batch_review_response import _publish
            _publish(path,raw)

    def execute(self,condition,request_id,*,deadline_wall):
        self.deadline=deadline_wall;self._verify();require(self.condition_support(condition) is None,'jaw_condition_unsupported')
        require(journal_clear(self.journal),'jaw_shared_journal_blocked');self._reserve(request_id)
        driver=self
        class PinnedPort(RecordingPort):
            def _check_status(port,status,expected,pulse=None,baseline_angle=None):
                super()._check_status(status,expected,pulse,baseline_angle)
                require(port.boot_id==driver.plan['controllerBootId'],'jaw_boot_changed')
        class GuardedCollection(ManualCollection):
            def pulse(manager,payload,*,before_run=None):
                def guard():
                    driver._guard(manager)
                    if before_run:before_run()
                return super().pulse(payload,before_run=guard)
        from learning.macos_credentials import load_keychain_secret
        token=load_keychain_secret(self.config['credential']['service'],self.config['credential']['account'])
        self.manager=GuardedCollection(self.root/'sessions',self.helper,self.config['controllerUrl'],token,self.config['deviceId'],self.plan['viewId'],
            journal_path=self.journal,clock=self.clock,port_factory=PinnedPort)
        try:
            report=collect_once(self.manager,request_id,condition['command'],self.context['poseRegime'],onsite_confirmed=True,
                request_root=self.root/'requests',approval_plan_sha256=self.plan_sha,gripper_source_factory=self._gripper,
                before_command=lambda manager,pair:self._before(manager,pair,condition))
            self._release_manager()
            result=self._classify(condition,report)
            self._save(self.root/'outcomes'/(request_id+'.json'),result)
            return result
        finally:
            try:self._release_manager()
            finally:token=None

    def _classify(self,condition,report):
        result=dict(execution='unknown',offVerified=report.get('offVerified') is True,journalClear=journal_clear(self.journal),
                    capture='failed',observation='rejected',failureScope='global',reason=report.get('reason','jaw_source_unverified'),modelSelected=False)
        if report.get('status')!='recorded':return result
        source=private_path(self.repository,report['sessionPath'])
        try:
            from learning.pipeline_evidence import ingest_session
            audit=ingest_session(source,report['sourceSha256'])
            require(audit['execution']['status']=='verified' and audit['execution']['offVerified'] is True,'jaw_execution_unverified')
            result.update(execution='completed',capture='recorded')
            session=json.loads((source/'session.json').read_bytes())
            scope_raw=(source/'commissioning-validation-scope.json').read_bytes();scope=json.loads(scope_raw)
            intent=json.loads((source/'command-001-intent.json').read_bytes())
            require(scope['scope']==self._scope() and scope['sessionId']==session['sessionId']
                and session.get('validationScopeSha256')==intent.get('validationScopeSha256')==sha(scope_raw)
                and scope['at']<intent['at'] and dict(kind='commissioning_validation_scope',at=scope['at'],sha256=sha(scope_raw)) in session['events'],
                'jaw_original_authorization_scope_changed')
            transition=build_transition(source,report['sourceSha256'],self.registration,self.reference,self.context,condition['captureGroup'],phase=condition['phase'])
            row=transition['learningRow'];require(row['command']==condition['command'],'jaw_source_command_changed')
            identity=digest(transition);path=self.root/'transitions'/(identity+'.json');self._save(path,transition)
            entry=dict(sourceKind='registered_jaws',sourcePath=str(source),sourceSha256=report['sourceSha256'],transitionPath=str(path),transitionSha256=identity,
                registrationPath=self.config['gripper']['registration']['path'],registrationSha256=self.config['gripper']['registration']['sha256'],
                referenceImagePath=self.config['gripper']['reference']['path'],referenceImageSha256=self.config['gripper']['reference']['sha256'],captureGroup=condition['captureGroup'])
            prospective=False
            if self.phase=='evaluation':
                from learning.batch_fixed_evaluation import audit_jaw_proof
                audit_jaw_proof(self.repository,entry,self.model,self.frozen['freeze']);prospective=True
            result.update(execution='completed',capture='recorded',observation='eligible',failureScope=None,reason='jaw_raw_verified',
                transitionId=identity,manifestEntry=entry,prospectiveEvidenceVerified=prospective,
                dataIdentity={k:copy.deepcopy(row[k]) for k in ('sourceHash','sessionId','episodeId','frameIds','pairedFrameIds')})
            stop=check_response(transition)
            if stop:result['followupStopReason']=stop
        except (ValueError,KeyError,OSError,TypeError) as exc:result['reason']='jaw_audit_rejected:'+str(exc)
        return result

    def reconcile(self,condition,request_id):
        """원본만 재감사하며 예약된 명령을 절대로 다시 전송하지 않는다."""
        path=self.root/'requests'/(request_id+'.json')
        if not path.exists():return None
        report=json.loads(path.read_bytes());request=report.get('request',{})
        require(request.get('requestId')==request_id and request.get('command')==condition['command']
            and request.get('approvalPlanSha256')==self.plan_sha,'jaw_reconcile_request_changed')
        return self._classify(condition,report)

    def _release_manager(self):
        manager=self.manager
        if manager is None:return
        self.manager=None
        try:manager.close()
        finally:
            manager.token=''
            if manager.port:manager.port.token=''

    def close(self):
        try:self._release_manager()
        finally:
            if self.profile_owned:
                self._profile(RESTORE)
                self.profile_owned=False


def create_driver(repository,config,phase,frozen=None):
    return JawDriver(repository,config,phase,frozen)
