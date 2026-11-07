#!/usr/bin/env python3
"""두 카메라 무구동 점검 또는 명시한 M4 위 200ms 단발 시험."""
from __future__ import annotations

import argparse
import copy
import getpass
import json
from pathlib import Path
import secrets
import sys
import time
import warnings

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.m4_up_commissioning import M4UpCommissioning, _external_frame, _gripper_frame, validate_scope
from learning.m4_up_observer import M4DualCameraObserver, sha
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort


class ReadOnlyPort(SequencePort):
    """The capture-check path cannot send a controller write, including STOP."""
    def _call(self,path,payload=None,**kwargs):
        require(payload is None,'camera_check_controller_write_forbidden')
        return super()._call(path,payload,**kwargs)


class FileReviewer:
    def __init__(self,store,*,clock=time):self.store,self.clock=store,clock
    def __call__(self,request,*,deadline,tick):
        label=request['label'];request=dict(request,nonce=secrets.token_hex(16))
        request_name=f'{label}-review-request.json'
        self.store.write_json(request_name,request)
        request_sha=sha(_read(self.store.root,request_name))
        print(json.dumps(dict(reviewRequired=label,request=str(self.store.root/request_name),
                             requestSha256=request_sha,previews=request['previews'])),flush=True)
        decision_name=f'{label}-review-decision.json'
        while self.clock.monotonic() < min(deadline,request['sourceEarliest']+9. if label=='before' else deadline):
            tick()
            path=self.store.root/decision_name
            if path.exists() or path.is_symlink():
                raw=_read(self.store.root,decision_name)
                require(len(raw)<=16384,'camera_review_too_large')
                result=_decode_json(raw)
                require(set(result)=={'requestSha256','decision'} and result['requestSha256']==request_sha,
                        'camera_review_binding_changed')
                require(sha(_read(self.store.root,request_name))==request_sha,'camera_review_request_changed')
                return result['decision']
            self.clock.sleep(.05)
        raise ValueError('camera_review_expired_no_run')


def capture_check(scope,camera,store,*,clock=time):
    # Placeholder is never used as a credential: this port rejects every POST.
    port=ReadOnlyPort(scope['controllerUrl'],'unused-camera-check-no-write',scope['controllerBootId'],store,
                      compact_reads=True,clock=clock)
    def unreviewed(request,**kwargs):
        return dict(context=copy.deepcopy(scope['context']),stable=False,
                    onsite={k:False for k in ('operatorPresent','clearanceVerified','jawContactVerified','tableSupportVerified')})
    observer=M4DualCameraObserver(scope,camera,store,unreviewed,clock=clock)
    report=dict(status='failed',mode='capture_check',controllerWrites=0,modelSelected=False,
                graspVerified=False,liftVerified=False,liveMotionAuthorized=False,errors=[])
    started=False;baseline=None;deadline=clock.monotonic()+60.
    def tick():
        require(clock.monotonic()<deadline,'camera_check_deadline')
        if started:observer.check()
        status=port.check_idle()
        require(abs(port._shoulder_angle(status)-scope['expectedStartAngleDeg'])<=.5,'camera_check_start_angle_changed')
        require(port.last_sequence==baseline,'camera_check_output_history_changed')
    try:
        port.preflight();baseline=port.last_sequence
        started=True;observer.start(deadline=deadline)
        before=observer.observe(after_off=None,deadline=deadline,tick=tick)
        tick();quiet_at=clock.monotonic()
        after=observer.observe(after_off=quiet_at,deadline=deadline,tick=tick)
        for label,record in [('before',before),('after',after)]:
            frames=dict(external=_external_frame(store.root,record['external'],scope),
                        gripper=_gripper_frame(store.root,record['gripper'],scope,label))
            if label=='after':require(all(f['sourceEarliest']>=quiet_at+3. for f in frames.values()),'camera_check_post_too_early')
            report[label]=frames
        report.update(status='completed',quietObservationAt=quiet_at)
    except (Exception,KeyboardInterrupt) as exc:report['errors'].append(type(exc).__name__)
    finally:
        if started:
            try:report['observerClose']=observer.close()
            except (Exception,KeyboardInterrupt) as exc:report['errors'].append(type(exc).__name__)
        try:
            if baseline is not None:
                # Native close can take time; prove unchanged outputs afterwards.
                status=port.check_idle()
                require(port.last_sequence==baseline,'camera_check_output_history_changed')
                report['finalIdleOffVerified']=True
        except (Exception,KeyboardInterrupt) as exc:report['errors'].append(type(exc).__name__)
        if report['errors'] or report.get('observerClose',{}).get('status')!='completed':report['status']='failed'
        store.write_json('capture-check-result.json',report)
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['capture-check','run'])
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--confirm-scope-sha256')
    p.add_argument('--wait-for-start',action='store_true',help='비공개 토큰 입력 뒤 촬영 시작용 Enter를 별도로 기다림')
    a=p.parse_args(argv)
    require(not a.wait_for_start or a.mode=='run','start_wait_requires_run_mode')
    private=(ROOT/'.local').resolve(strict=True)
    config=_decode_json(_read(private,a.config.absolute().relative_to(private).as_posix()))
    require(set(config)=={'scope','camera'},'m4_trial_config_fields')
    scope=validate_scope(config['scope'])
    require(a.output_dir.resolve().is_relative_to(private),'private_trial_output_required')
    if a.mode=='run':
        require(a.confirm_scope_sha256==plan_digest(scope),'exact_scope_confirmation_required')
        require(sys.stdin.isatty(),'private_terminal_input_required')
        # No environment/other process/browser token extraction or log fallback.
        with warnings.catch_warnings():
            warnings.simplefilter('error',getpass.GetPassWarning)
            token=getpass.getpass('제어 토큰 (비공개 입력): ')
        if a.wait_for_start:
            input('토큰 입력 완료 · 촬영 시작 대기 (에이전트가 준비 후 Enter): ')
    store=ArtifactStore(a.output_dir)
    store.write_json('trial-config.json',config)
    if a.mode=='capture-check':result=capture_check(scope,config['camera'],store)
    else:
        observer=M4DualCameraObserver(scope,config['camera'],store,FileReviewer(store))
        result=M4UpCommissioning(scope,observer,token,store,evidence_root=store.root).run(confirmed_scope_sha256=a.confirm_scope_sha256)
    print(json.dumps(dict(status=result['status'],resultDirectory=str(store.root),mode=a.mode)),flush=True)
    return 0 if result['status']=='completed' else 2


if __name__=='__main__':
    try:raise SystemExit(main())
    except (Exception,KeyboardInterrupt) as exc:
        print(json.dumps(dict(status='failed',errorType=type(exc).__name__)),flush=True)
        raise SystemExit(2)
