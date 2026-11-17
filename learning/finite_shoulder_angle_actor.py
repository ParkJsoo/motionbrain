"""모델이 선택한 단일 DOWN 목표를 기존 AS5600 각도 제어에 전달한다.

native 보정은 모델 선택과 구분한다. 기본 수집 정책·펌웨어·과거 자료는 바꾸지
않으며, native 도달 보고·현재 각도 오차·사후 영상의 작업 진전은 별도 결과다.
deviceTargetReached는 native 보고와 현재 native 허용오차를 함께 확인한 값이며,
명령의 completed 상태는 작업 목표 성공이나 예측 정확도를 뜻하지 않는다.
장치·인증 탐색 없음.
"""
from __future__ import annotations
import copy
import hashlib
import math
from pathlib import Path
import time

import cv2
import numpy as np

from learning.action_journal import BoundedActionJournal
from learning.artifacts import _read, _decode_json
from learning.manual_collection_dataset import require
from learning.supervised_collection import identifier, plan_digest
from learning.supervised_transport import SequencePort

SCHEMA = 'motionbrain.finite_shoulder_angle_actor.v1'
OBSERVATION_SCHEMA = 'motionbrain.finite_shoulder_dual_selection.v1'


def _finite(x):
    return type(x) in (int,float) and math.isfinite(x)


def validate_scope(scope):
    fields={'schemaVersion','trialId','controllerUrl','controllerBootId','cameraBootId','model',
            'initialDecision','initialDecisionSha256','expectedStartAngleDeg','targetAngleDeg','percent',
            'maxMotionSeconds','maxObservationAgeSeconds'}
    require(isinstance(scope,dict) and set(scope)==fields and scope['schemaVersion']==SCHEMA,
            'shoulder_angle_scope_fields')
    _validate_motion_fields(scope)
    require(isinstance(scope['initialDecision'],dict)
            and scope['initialDecision'].get('targetAngleDeg')==scope['targetAngleDeg']
            and plan_digest(scope['initialDecision'])==scope['initialDecisionSha256'],
            'shoulder_angle_initial_selection')
    return copy.deepcopy(scope)


def _validate_motion_fields(scope):
    """모델/사용자 목표 역할과 무관한 동일 native 동작 한도."""
    require(identifier(scope['trialId']) and all(isinstance(scope[k],str) and len(scope[k])==32
            and all(c in '0123456789abcdef' for c in scope[k]) for k in ('controllerBootId','cameraBootId')),
            'shoulder_angle_identity')
    require(all(_finite(scope[k]) for k in ('expectedStartAngleDeg','targetAngleDeg'))
            and 1. <= scope['expectedStartAngleDeg']-scope['targetAngleDeg'] <= 8.
            and 127.58 < scope['targetAngleDeg'] < scope['expectedStartAngleDeg'] < 296.12,
            'shoulder_angle_finite_down_goal')
    require(type(scope['percent']) is int and scope['percent']==75 and scope['maxMotionSeconds']==12.
            and scope['maxObservationAgeSeconds']==3.,'shoulder_angle_fixed_policy')


class _AnglePort(SequencePort):
    def _consume(self,payload,*,quiet=False):
        # Native closed-loop correction may use either M4 direction. Keep the
        # original ring/boot/known-history/gap audits; audit both PWM signs here.
        new=super()._consume(payload,quiet=quiet)
        if self.may_have_run:
            require(all(e['pwm']==0 or e['motor']==4 and abs(e['pwm'])<=191 for e in new),
                    'shoulder_angle_unexpected_driver_output')
            self.angle_trace.extend(copy.deepcopy(new))
        return new


class FiniteShoulderAngleActor:
    validate_scope = staticmethod(validate_scope)

    def __init__(self,scope,store,token,*,journal_path,asset_root,observe_and_select,clock=time):
        self.scope=self.validate_scope(scope)
        # 역할별 schema 검증과 별도로 동일 native 동작 한도는 항상 적용한다.
        _validate_motion_fields(self.scope);self.digest=plan_digest(self.scope)
        require(callable(observe_and_select),'shoulder_angle_observer_required')
        self.store=store;self.asset_root=Path(asset_root).resolve();self.journal_path=Path(journal_path)
        self.observe_and_select=observe_and_select;self.clock=clock;self.used=False;self.observations=[]
        self.port=_AnglePort(self.scope['controllerUrl'],token,self.scope['controllerBootId'],store,
            clock=clock,compact_reads=True,timeout=2.,full_output_snapshot=True)
        self.port.angle_trace=[]

    def _binding(self,ref):
        require(isinstance(ref,dict) and set(ref)=={'path','sha256'},'shoulder_angle_source_binding')
        raw=_read(self.asset_root,ref['path'])
        require(hashlib.sha256(raw).hexdigest()==ref['sha256'],'shoulder_angle_source_changed')
        return raw

    def _memory(self):
        require(plan_digest(self.scope)==self.digest,'shoulder_angle_scope_changed')

    def _observation_asset(self):
        return self.scope['model']

    def _role_report_fields(self):
        return dict(modelSelected=True)

    def _validate_role_record(self,record,stage):
        require(isinstance(record,dict) and record.get('schemaVersion')==OBSERVATION_SCHEMA
                and record.get('stage')==stage and record.get('modelSha256')==self.scope['model']['sha256']
                and isinstance(record.get('decision'),dict)
                and record.get('decisionSha256')==plan_digest(record['decision']),
                'shoulder_angle_observation_selection')
        if stage!='after':
            require(record.get('targetAngleDeg')==self.scope['targetAngleDeg']
                    and record['decision'].get('targetAngleDeg')==self.scope['targetAngleDeg'],
                    'shoulder_angle_fresh_selection_changed')

    def _observe(self,stage,snapshot):
        self._memory();self._binding(self._observation_asset())
        record=self.observe_and_select(stage,copy.deepcopy(snapshot))
        self._validate_role_record(record,stage)
        sources=record.get('sources')
        require(isinstance(sources,dict) and set(sources)=={'external','gripper'},'shoulder_angle_dual_sources')
        for camera,source in sources.items():
            require(isinstance(source,dict) and isinstance(source.get('frameId'),str) and source['frameId']
                    and all(_finite(source.get(k)) for k in ('sourceEarliest','sourceLatest'))
                    and source['sourceEarliest']<=source['sourceLatest']<=self.clock.monotonic()
                    and self.clock.monotonic()-source['sourceEarliest']<=3.,'shoulder_angle_observation_stale')
            image_raw=self._binding(source['image']);metadata=_decode_json(self._binding(source['metadata']))
            if camera=='gripper':
                require(source.get('cameraBootId')==self.scope['cameraBootId'],'shoulder_angle_camera_boot_changed')
                headers={k.lower():v for k,v in metadata.get('headers',{}).items()}
                require(metadata.get('requestStartedAt')==source['sourceEarliest']
                        and metadata.get('receivedAt')==source['sourceLatest']
                        and metadata.get('sha256')==source['image']['sha256']
                        and headers.get('x-motionbrain-frame-id')==source['frameId']
                        and headers.get('x-motionbrain-camera-boot-id')==self.scope['cameraBootId']
                        and headers.get('x-motionbrain-capture-mode')=='after_request',
                        'shoulder_angle_gripper_raw_binding')
            else:
                # External metadata is the original ActionRecorder ingress
                # frame, whose pixel digest binds the derived lossless PNG.
                image=cv2.imdecode(np.frombuffer(image_raw,np.uint8),cv2.IMREAD_UNCHANGED)
                require(metadata.get('frameId')==source['frameId']
                        and metadata.get('sourceEarliest')==source['sourceEarliest']
                        and metadata.get('sourceLatest')==source['sourceLatest']
                        and image is not None and image.ndim==3 and image.shape[2]==4
                        and list(image.shape[:2])==[metadata.get('height'),metadata.get('width')]
                        and hashlib.sha256(image.tobytes()).hexdigest()==metadata.get('pixelSha256'),
                        'shoulder_angle_external_raw_binding')
            if self.observations:
                prior=self.observations[-1]['sources'][camera]
                require(source['frameId']!=prior['frameId'] and source['sourceEarliest']>prior['sourceLatest'],
                        'shoulder_angle_frame_reused')
        self.store.write_json(stage+'-dual-selection.json',record)
        self.observations.append(copy.deepcopy(record))
        return record

    def _native_goal(self,status):
        shoulder=status.get('shoulderAngle',{})
        require(isinstance(shoulder,dict) and type(shoulder.get('active')) is bool
                and _finite(shoulder.get('targetDeg')) and type(shoulder.get('requestedPercent')) is int,
                'shoulder_angle_native_goal_fields_missing')
        require(abs(shoulder['targetDeg']-self.scope['targetAngleDeg'])<=.01
                and shoulder['requestedPercent']==75,'shoulder_angle_native_goal_changed')
        return shoulder

    def _active_status(self,status,start):
        self._native_goal(status)
        angle=self.port._shoulder_angle(status)
        checked=copy.deepcopy(status);checked['shoulderAngle']['active']=False
        speed=status.get('motors',{}).get('M4',{}).get('speed')
        require(type(speed) is int and abs(speed)<=191,'shoulder_angle_native_pwm')
        pulse=dict(motor=4,expectedPwm=191 if speed>0 else -191)
        self.port._check_status(checked,'ARMED',pulse,angle)
        require(start-10. <= angle <= start+3.,'shoulder_angle_observed_corridor')
        return angle

    def run(self,*,confirmed_scope_sha256):
        require(not self.used and confirmed_scope_sha256==self.digest,'shoulder_angle_unconsumed_scope')
        self.used=True;self._memory();journal=None;claimed=False;phase='journal';cleanup=False
        initial_full_output_snapshot=self.port.full_output_snapshot
        report=dict(schemaVersion=SCHEMA,scopeSha256=self.digest,status='failed',commandOutcome='not_started',
            mayHaveRun=False,finalIdleOffVerified=False,nativeTerminalReason=None,deviceTargetReached=False,
            nativeReportedTargetReached=False,actualAngleWithinNativeTolerance=None,
            taskGoalEvaluated=False,automaticRetry=False,errors=[],**self._role_report_fields())
        try:
            require(self.journal_path.is_file(),'shoulder_angle_existing_shared_journal_required')
            journal=BoundedActionJournal(str(self.journal_path),10000)
            require(journal.readiness()['ready'],'shoulder_angle_shared_journal_unresolved')
            db=journal.connection
            with db:
                db.execute('CREATE TABLE IF NOT EXISTS supervised_blocks (session_id TEXT PRIMARY KEY,plan_hash TEXT NOT NULL,output_path TEXT NOT NULL,phase TEXT NOT NULL,result_json TEXT)')
                require(not db.execute("SELECT 1 FROM supervised_blocks WHERE phase IN ('pending','off_unverified') LIMIT 1").fetchone(),
                        'shoulder_angle_shared_block_unresolved')
                require(not db.execute('SELECT 1 FROM supervised_blocks WHERE session_id=?',(self.scope['trialId'],)).fetchone(),
                        'shoulder_angle_scope_consumed')
                require(db.execute('SELECT COUNT(*) FROM supervised_blocks').fetchone()[0]<10000,
                        'shoulder_angle_scope_capacity')
                db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,NULL)',
                    (self.scope['trialId'],self.digest,str(self.store.root),'pending'))
            claimed=True;self.store.write_json('scope.json',self.scope)
            phase='preflight';snapshot=self.port.preflight()
            start=self.port._shoulder_angle(snapshot['status'])
            require(abs(start-self.scope['expectedStartAngleDeg'])<=.5
                    and snapshot['status']['shoulderAngle']['manualDownBoundaryDeg']+4<self.scope['targetAngleDeg'],
                    'shoulder_angle_start_changed')
            phase='before_observation';self._observe('before',snapshot)
            self.port.timeout=.8;self.port.writes_possible=True
            phase='arm';self.port._call('/command',{'cmd':'arm'})
            self.port.timeout=2.;status=self.port._status();self.port._check_status(status,'ARMED')
            require(abs(self.port._shoulder_angle(status)-start)<=.5,'shoulder_angle_ARM_drift')
            queue=self.port._queue(0);self.port._outputs(quiet=True)
            require(self.port.last_outputs=={i:0 for i in range(1,6)},'shoulder_angle_ARM_not_OFF')
            phase='armed_observation';armed=self._observe('armed',dict(status=status,sequence=queue))
            armed_sha=plan_digest(armed)
            # Camera acquisition and selection may take seconds. Recheck the
            # unchanged ARMED/OFF controller epoch after that work, before RUN.
            phase='armed_final_proof';self.port._outputs(quiet=True)
            status=self.port._status();self.port._check_status(status,'ARMED')
            require(abs(self.port._shoulder_angle(status)-start)<=.5,'shoulder_angle_ARM_drift')
            self.port._queue(0)
            require(self.port.last_outputs=={i:0 for i in range(1,6)},'shoulder_angle_ARM_not_OFF')
            def before_run():
                self._memory()
                require(plan_digest(self.observations[-1])==armed_sha
                        and all(self.clock.monotonic()-s['sourceEarliest']<=3. for s in armed['sources'].values()),
                        'shoulder_angle_RUN_observation_changed')
            self.port._before_run=before_run
            request=dict(action='angle',degrees=self.scope['targetAngleDeg'],percent=75)
            self.port._finite_shoulder_request=copy.deepcopy(request);self.port.timeout=.8
            phase='native_angle_POST';accepted=self.port._call('/shoulder',request,run=True)
            report['commandOutcome']='receipt_recorded'
            self._native_goal(accepted)
            self.store.write_json('native-angle-accepted.json',dict(targetAngleDeg=self.scope['targetAngleDeg'],
                request=request,response=accepted))
            # Large unpaged responses block the firmware's control loop while
            # HTTP is sent. During native angle control drain small incremental
            # pages; retain the same shared deadline and every output audit.
            self.port.full_output_snapshot=False
            deadline=self.port.last_dispatch_before+12.;phase='native_monitor'
            terminal=None
            while self.clock.monotonic()<deadline:
                remaining=deadline-self.clock.monotonic()
                # collection.v1 deliberately omits target/requestedPercent and
                # terminal reason. Native goal verification requires /status;
                # output history remains on the small incremental endpoint.
                status=self.port._call('/status',request_timeout=min(.8,remaining))
                angle=self._active_status(status,start)
                self.port._outputs(deadline=deadline)
                if status['shoulderAngle']['active'] is False:
                    terminal=copy.deepcopy(status)
                    reported=status['shoulderAngle'].get('lastStopReason')=='TARGET_REACHED'
                    error=angle-self.scope['targetAngleDeg']
                    tolerance=status['shoulderAngle'].get('settledSuccessToleranceDeg')
                    within=(abs(error)<=tolerance) if _finite(tolerance) and 0<tolerance<=.5 else None
                    report.update(nativeTerminalReason=status['shoulderAngle'].get('lastStopReason'),
                        nativeReportedTargetReached=reported,deviceTargetReached=reported and within is True,
                        actualAngleDeg=angle,actualAngleErrorDeg=error,nativeSettledToleranceDeg=tolerance,
                        actualAngleWithinNativeTolerance=within)
                    self.store.write_json('native-terminal-status.json',terminal);break
                self.clock.sleep(min(.2,max(0.,deadline-self.clock.monotonic())))
            require(terminal is not None,'shoulder_angle_host_deadline')
            report['status']='completed' if report['nativeReportedTargetReached'] else 'native_failed'
        except BaseException as exc:
            report['errors'].append(dict(phase=phase,type=type(exc).__name__,reason=str(exc) if isinstance(exc,ValueError) else type(exc).__name__))
            if self.port.may_have_run and report['commandOutcome']=='not_started':report['commandOutcome']='unknown'
        finally:
            self.port.full_output_snapshot=initial_full_output_snapshot
            try:
                if claimed:
                    cleanup=self.port.finish();report['finalIdleOffVerified']=cleanup
                    report['mayHaveRun']=self.port.may_have_run
                    report['outputTrace']=copy.deepcopy(self.port.angle_trace)
                    if not cleanup:report['status']='failed'
                    if cleanup and self.port.may_have_run:
                        try:self._observe('after',self.port.last_snapshot)
                        except BaseException as exc:
                            report['errors'].append(dict(phase='after_observation',type=type(exc).__name__))
                            report['afterObservationFailed']=True
                    self.store.write_json('result.json',report)
                    with journal.connection:
                        journal.connection.execute('UPDATE supervised_blocks SET phase=?,result_json=? WHERE session_id=? AND plan_hash=?',
                            ('closed' if cleanup else 'off_unverified',__import__('json').dumps(report,sort_keys=True),self.scope['trialId'],self.digest))
            finally:
                self.port._before_run=None;self.port._finite_shoulder_request=None
                if journal is not None:journal.close()
        return report
