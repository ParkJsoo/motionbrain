#!/usr/bin/env python3
"""명시한 M1 접촉 탐색 한 번. 모델 미지원은 수동 시험으로 기록하며 자동 반복하지 않는다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from learning.artifacts import ArtifactStore,_read
from learning.experiment_pipeline import atomic,encoded,sync_directory
from learning.manual_collection import ManualCollection,RecordingPort
from learning.pipeline_collection import collect_once
from learning.grasp_observation import GripperObservationSource
from learning.registered_gripper_crosssection import RegisteredGripperCrosssection
from learning.macos_credentials import load_keychain_secret
from learning import m1_shaft_learning as learning
PRIVATE=ROOT/'.local'
COMMAND=dict(joint='gripper',direction='close',percent=40,durationMs=350)
POSE='current-body-height-M1-contact-probe-M4-near200-other-joints-unmeasured'
def require(ok,why):
    if not ok:raise ValueError(why)
def sha(raw):return hashlib.sha256(raw).hexdigest()
def read(path):return _read(PRIVATE,Path(path).absolute().relative_to(PRIVATE).as_posix())
def load(path):return json.loads(read(path))


def run_probe(config,step,service,account):
    config=Path(config).absolute();raw=read(config);plan=json.loads(raw);base=config.parent
    require(plan['schemaVersion']=='motionbrain.attended_contact_probe.v1','contact_plan_required')
    require(plan.get('automaticRetry') is False and plan.get('automaticStageLoop') is False,'manual_single_step_only')
    auth=plan['authorization']
    require(auth.get('additionalMotionAuthorized') is True and isinstance(auth.get('latestUserInstruction'),str)
        and bool(auth['latestUserInstruction'].strip()),'explicit_current_motion_authorization_required')
    require(type(plan['maximumM1CloseCommands']) is int and 1<=plan['maximumM1CloseCommands']<=6
        and type(step) is int and 1<=step<=plan['maximumM1CloseCommands'],'bounded_explicit_step_required')
    require(plan['M1CloseCommand']==COMMAND,'fixed_M1_close_only')
    request=base/'requests'/f'contact-close-{step:02d}.json';response=base/f'response-{step:02d}.json'
    if request.exists() or request.is_symlink():
        saved=load(request)
        require(saved['request']['requestId']==f'contact-close-{step:02d}' and saved['request']['command']==COMMAND,'saved_request_binding_changed')
        return saved
    require(not response.exists() and not response.is_symlink(),'orphan_response_no_retry')
    if step>1:
        previous=load(base/f'response-{step-1:02d}.json')
        require(previous['status']=='recorded' and previous['offVerified'] is True,'previous_step_incomplete_no_retry')
        review=plan.get('manualReview',{})
        require(review.get('lastCompletedStep')==step-1 and review.get('decision')=='continue_single_close','current_manual_review_required')
        image=ROOT/review['evidencePath']
        require(image.suffix.lower() in ('.jpg','.jpeg','.png') and sha(read(image))==review['evidenceSha256'],'manual_review_image_changed')
    else:
        require(isinstance(auth.get('userInstruction'),str) and bool(auth['userInstruction'].strip()),'initial_user_authorization_required')
        read(ROOT/plan['geometryReviewPath'])
    for path,digest in plan.get('sourceHashes',{}).items():
        require(sha((ROOT/path).read_bytes())==digest,'source_changed:'+path)
    spec=load(ROOT/plan['observerSpecPath'])
    registration=_read(PRIVATE,spec['registrationPath']);reference=_read(PRIVATE,spec['referenceImagePath'])
    require(sha(registration)==spec['registrationSha256'] and sha(reference)==spec['referenceImageSha256'],'observer_assets_changed')
    tracker=RegisteredGripperCrosssection(json.loads(registration),reference);model=None
    if plan.get('modelPath') is not None or plan.get('modelSha256') is not None:
        blob=read(ROOT/plan['modelPath']);require(sha(blob)==plan['modelSha256'],'fixed_model_changed')
        model=learning.validate_model(json.loads(blob))
    class Port(RecordingPort):
        def _check_status(port,status,expected,pulse=None,baseline_angle=None):
            super()._check_status(status,expected,pulse,baseline_angle)
            require(port.boot_id==plan['controllerBootId'],'controller_boot_changed')
            require(abs(port._shoulder_angle(status)-plan['M4BaselineDeg'])<=.5,'M4_pose_changed')
    def factory(sid,store):
        return GripperObservationSource(plan['cameraUrl'],sid,plan['cameraBootId'],
            dict(spec['context'],landmarks=tracker.shaft_landmark_contract),store,max_captures=2)
    def before(manager,pair):
        require(read(config)==raw,'contact_plan_changed')
        frame=pair['before']['frame'];obs=None;rejection=None
        image=_read(Path(pair['path']),'esp32-before.jpg')
        try:obs=tracker.observe_shaft(image,frame['frameId'])
        except ValueError as exc:rejection=str(exc)
        if obs is not None:require(obs['pixelSha256']==frame['pixelSha256'],'prospective_pixels_changed')
        evidence=dict(preparedAt=time.monotonic(),mode='manual_contact_probe_not_model_selected',
            observation=obs,command=COMMAND,modelSelected=False,prediction=None,unsupportedReason=rejection,
            geometryDepthVerified=False,graspVerified=False,
            modelSha256=plan.get('modelSha256'))
        if model is not None and obs is not None:
            try:evidence['prediction']=learning.predict(model,obs['state'][0],COMMAND,pair['before']['context'])
            except ValueError as exc:evidence['unsupportedReason']=str(exc)
        manager.store.write_json('prospective.json',evidence)
    snapshot=base/f'plan-{step:02d}.json'
    try:
        with snapshot.open('xb') as out:
            out.write(raw);out.flush();os.fsync(out.fileno())
        sync_directory(base)
    except FileExistsError:
        require(read(snapshot)==raw,'plan_snapshot_changed')
    token=load_keychain_secret(service,account);manager=None
    try:
        require(read(config)==raw,'contact_plan_changed')
        manager=ManualCollection(base/'sessions',ROOT/plan['helperPath'],plan['controllerUrl'],token,
            plan['deviceId'],plan['viewId'],sample_fps=8,port_factory=Port)
        token=None
        result=collect_once(manager,f'contact-close-{step:02d}',COMMAND,POSE,onsite_confirmed=True,
            request_root=base/'requests',approval_plan_sha256=sha(raw),gripper_source_factory=factory,before_command=before)
        atomic(response,encoded(result));return result
    finally:
        token=None
        if manager is not None:
            try:manager.close()
            finally:
                manager.token=''
                if manager.port:manager.port.token=''


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--step',type=int,required=True);parser.add_argument('--keychain-service',required=True)
    parser.add_argument('--keychain-account',required=True);args=parser.parse_args(argv)
    result=run_probe(args.config,args.step,args.keychain_service,args.keychain_account)
    print(json.dumps(dict(status=result['status'],offVerified=result.get('offVerified'),automaticRetry=False)))
if __name__=='__main__':main()
