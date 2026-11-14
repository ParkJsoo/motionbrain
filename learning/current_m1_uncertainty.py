"""현재 M1에 명시한 개발용 관측 여유. 학습 범위·원본·통계 보증을 바꾸지 않는다."""
from __future__ import annotations

import copy
import re
from learning.manual_collection_dataset import require
from learning.pipeline_learning import digest

SCHEMA='motionbrain.current_m1_observation_admission.v1'
CAMERA_BOOT='45bf07856684bc004dde3d34d8efb1f4'
EPSILON_PX=.125


def audit_calibration(root,policy,model,registration,reference,guard_config):
    """동일 OFF epoch의 실제 여섯 원본을 재측정한다. 현재 실행 freshness는 별도다."""
    import hashlib
    from pathlib import Path
    from learning.artifacts import _read,_decode_json
    from learning.gripper_camera_dataset import read_frame
    from learning.registered_gripper_crosssection import RegisteredGripperCrosssection
    from learning.supervised_transport import response_json,SequencePort
    validate_policy(policy,model)
    require(hashlib.sha256(registration).hexdigest()==policy['registrationSha256']
        and hashlib.sha256(guard_config).hexdigest()==policy['guardConfigSha256'],
        'M1_admission_actual_registration_changed')
    ref=policy['calibration'];raw=_read(root,ref['path'])
    require(hashlib.sha256(raw).hexdigest()==ref['sha256'],'M1_calibration_changed')
    report=_decode_json(raw);folder=Path(root)/Path(ref['path']).parent
    plan=_decode_json(_read(folder,'predeclared-plan.json'))
    require(report.get('status')=='passed' and report.get('maximumShaftSpanPx')==EPSILON_PX
        and report.get('statisticalGuarantee') is False and report.get('motorCommands')==0
        and report.get('cameraBootId')==CAMERA_BOOT and report.get('fiveOff') is True
        and report.get('registrationSha256')==policy['registrationSha256']
        and plan.get('captures')==6 and plan.get('maximumShaftSpanPx')==EPSILON_PX
        and plan.get('statisticalGuarantee') is False and plan.get('motorCommands')==0
        and plan.get('expectedCursor')==report['outputSequence']
        and plan.get('cameraBootId')==CAMERA_BOOT
        and plan.get('controllerBootId')==report['controllerBootId'], 'M1_calibration_contract')
    tracker=RegisteredGripperCrosssection(_decode_json(registration),reference)
    require(tracker.shaft_landmark_contract==model['context']['landmarks']
        and tracker.registration['sceneContext']=={k:model['context'][k] for k in ('physicalView','poseRegime','object')},
        'M1_calibration_model_registration_changed')
    require(isinstance(report.get('rows'),list) and len(report['rows'])==6,'M1_calibration_six_required')
    values=[];ids=[];first=float('inf');last=0.
    for row in report['rows']:
        record=row['record'];location=Path(row['imagePath']).parent
        # _read checks symlinks/traversal before the original frame parser is used.
        image=_read(root,row['imagePath'])
        frame,metadata,boot=read_frame(Path(root)/location,'frame',record['hostClockId'])
        measured=tracker.observe_shaft(image,frame['frameId'])
        require(boot==CAMERA_BOOT and record['context']==model['context']
            and record['frame']=={k:frame[k] for k in ('frameId','height','pixelSha256','width')}
            and measured==row['measurement'] and record['sourceEarliest']==metadata['requestStartedAt']
            and record['sourceLatest']==metadata['receivedAt'] and record['persistedAt']==metadata['persistedAt'],
            'M1_calibration_raw_measurement_changed')
        values.append(measured['shaftSeparationPx']);ids.append(frame['frameId'])
        first=min(first,record['sourceEarliest']);last=max(last,record['persistedAt'])
    require(len(set(ids))==6 and values==report['values'] and max(values)-min(values)==report['spanPx']
        and report['spanPx']<=EPSILON_PX,'M1_calibration_span_exceeded')
    events=[];outputs=[]
    for path in sorted((folder/'controller').glob('http-*-intent.json')):
        intent=_decode_json(path.read_bytes());resp=_decode_json(path.with_name(path.name.replace('-intent','-response')).read_bytes())
        require(intent.get('method')=='GET' and intent.get('payload') is None and resp.get('httpStatus')==200,
                'M1_calibration_GET_only')
        parsed=response_json(resp['rawUtf8'],intent['path'])
        require(parsed==resp['parsed'],'M1_calibration_http_changed')
        events.append((intent,resp,parsed))
        if intent['path'].startswith('/action_outputs'):
            require(parsed['bootId']==report['controllerBootId'] and parsed['latestSequence']==report['outputSequence']
                and parsed.get('gap') is False,'M1_calibration_output_epoch_changed')
            outputs.extend(parsed['entries'])
    require(bool(events) and bool(outputs),'M1_calibration_raw_OFF_missing')
    motors={};previous=None
    for entry in outputs:
        require(previous is None or entry['sequence']==previous+1,'M1_calibration_output_gap')
        previous=entry['sequence'];motors[entry['motor']]=entry['pwm']
    require(previous==report['outputSequence'] and motors=={i:0 for i in range(1,6)},'M1_calibration_driver_not_OFF')
    checker=object.__new__(SequencePort)
    for label,before in [('controllerBefore',True),('controllerAfter',False)]:
        snapshot=report[label]
        for key,endpoint in [('actions','/actions'),('status','/collection_status'),('sequence','/sequence')]:
            require(any(i['path']==endpoint and p==snapshot[key]
                and (r['receivedAt']<=first if before else i['startedAt']>=last) for i,r,p in events),
                'M1_calibration_bracketing_OFF_missing')
        require(snapshot['actions']['bootId']==report['controllerBootId'] and snapshot['actions']['active'] is False
            and snapshot['sequence']['sequence']['state']=='IDLE'
            and snapshot['sequence']['sequence']['totalCount']==0,'M1_calibration_not_idle')
        SequencePort._check_status(checker,snapshot['status'],'IDLE')
        require(abs(SequencePort._shoulder_angle(snapshot['status'])-197.22)<=.25,
                'M1_calibration_pose_changed')
    return dict(policySha256=digest(policy),sampleCount=6,spanPx=report['spanPx'],
                liveEligible=policy['liveEligible'],statisticalCoverageClaim=False)


def validate_policy(policy, model):
    from learning import m1_shaft_learning as shaft
    shaft.validate_model(model)
    require(isinstance(policy,dict) and set(policy)=={'schemaVersion','epsilonPx','developmentOnly',
        'statisticalCoverageClaim','modelSha256','context','command','registrationSha256',
        'guardConfigSha256','calibration','liveEligible'},'M1_admission_policy_fields')
    require(policy['schemaVersion']==SCHEMA and type(policy['epsilonPx']) is float
        and policy['epsilonPx']==EPSILON_PX and policy['developmentOnly'] is True
        and policy['statisticalCoverageClaim'] is False and type(policy['liveEligible']) is bool,
        'M1_admission_policy_changed')
    context=model['context'];command=policy['command']
    require(policy['modelSha256']==digest(model) and policy['context']==context
        and context['cameraIdentity'].get('cameraBootId')==CAMERA_BOOT
        and context['physicalView']=='user-inserted-current-gripper-20261114'
        and context['poseRegime']=='table-supported-M4-19722-20261114'
        and context['object']=='pink-prism-user-inserted-20261114', 'M1_admission_model_context_changed')
    require(command in [entry['command'] for entry in model['models']]
        and command==dict(joint='gripper',direction=command.get('direction'),percent=60,durationMs=650)
        and command['direction'] in ('close','open'),'M1_admission_command_changed')
    require(all(isinstance(policy[k],str) and re.fullmatch('[0-9a-f]{64}',policy[k])
        for k in ('registrationSha256','guardConfigSha256')),'M1_admission_registration_binding')
    require(isinstance(policy['calibration'],dict) and set(policy['calibration'])=={'path','sha256'}
        and isinstance(policy['calibration']['path'],str)
        and re.fullmatch('[0-9a-f]{64}',policy['calibration']['sha256']) is not None,
        'M1_admission_calibration_binding')
    return copy.deepcopy(policy)


def predict_with_admission(model,x,command,context,policy,*,baseline=False):
    """raw x에 같은 고정 계수를 평가한다. clamp/범위 재작성/모델 재학습은 없다."""
    from learning import m1_shaft_learning as shaft
    validate_policy(policy,model)
    require(policy['liveEligible'] is True,'M1_admission_calibration_pending')
    require(context==policy['context'] and command==policy['command'],'M1_admission_identity_changed')
    x=shaft._scalar(x)
    entry=next(e for e in model['models'] if e['command']==command)
    distance=max(entry['minimum']-x,0.,x-entry['maximum'])
    require(distance<=EPSILON_PX,'M1_admission_distance_exceeded')
    if distance==0:
        prediction=shaft.predict(model,x,command,context,baseline=baseline)
    elif model['schemaVersion']==shaft.AFTER_SCHEMA and not baseline:
        prediction=shaft._scalar(entry['afterMean']+(x-entry['mean'])/entry['scale']*entry['afterCoefficient'])
    else:
        delta=entry['deltaMean']
        if not baseline:delta+=(x-entry['mean'])/entry['scale']*entry['coefficient']
        prediction=shaft._scalar(x+delta)
    evidence=dict(policySha256=digest(policy),rawShaftSeparationPx=x,
        originalTrainingRangePx=[entry['minimum'],entry['maximum']],outsideDistancePx=distance,
        epsilonPx=EPSILON_PX,extrapolated=distance>0,uncertaintyAdmitted=distance>0,
        statisticalCoverageClaim=False,trainingSupportExpanded=False)
    return prediction,evidence
