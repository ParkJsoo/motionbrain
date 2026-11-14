"""단발 DOWN band 복구의 원실패·현재 OFF·명시 허용 원본 감사.

경로에 장치 주소·자격·전송은 없다. 파일을 읽고 검증할 뿐이며, 실제 GET과
새 두 영상은 호출자 및 기존 M4 actor가 소유한다. 학습·모델 성과를 만들지 않는다.
"""
from __future__ import annotations

import copy
import hashlib
from pathlib import Path,PurePosixPath
import urllib.parse

from learning.artifacts import _decode_json,_read
from learning.manual_collection_dataset import Evidence,require
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort,response_json,finite_number

OFF_SCHEMA='motionbrain.m4_band_recovery_current_off.v1'
AUTH_SCHEMA='motionbrain.m4_band_recovery_authorization.v1'
CURRENT_OFF_MAX_AGE_SECONDS=30.


def recovery_motion_digest(scope):
    return plan_digest({key:value for key,value in scope.items() if key!='recoveryEvidence'})


def validate_recovery_sources(scope,*,asset_root,now):
    """현재 복구에만 적용하며 원결과·원장·파일을 갱신하지 않는다."""
    from learning import m4_up_commissioning as core
    from learning.task_transition_data import _http
    require(scope.get('schemaVersion')==core.BAND_RETURN_RECOVERY_SCHEMA and finite_number(now),
            'recovery_explicit_scope_required')
    root=Path(asset_root).resolve(strict=True);sources=scope['recoveryEvidence'];hashes={}
    def raw(ref):
        require(isinstance(ref,dict) and set(ref)=={'path','sha256'},'recovery_source_binding')
        body=_read(root,ref['path'])
        require(hashlib.sha256(body).hexdigest()==ref['sha256'],'recovery_source_changed')
        hashes[ref['path']]=ref['sha256'];return body
    source_scope=_decode_json(raw(sources['sourceScope']))
    failed=_decode_json(raw(sources['sourceFailure']))
    core.validate_recorded_scope(source_scope)
    require(source_scope.get('schemaVersion')==core.TARGET_UP_SCHEMA
            and source_scope.get('controllerBootId')==scope['controllerBootId']
            and source_scope.get('pulse')==core.LEARNING_PULSE
            and all(source_scope['context'][key]==scope['context'][key] for key in (
                'physicalView','object','externalDeviceId','externalViewId','gripperCameraBootId')),
            'recovery_owned_failed_UP_scope_required')
    require(failed.get('schemaVersion')==source_scope['schemaVersion']
            and failed.get('scopeSha256')==plan_digest(source_scope)
            and failed.get('status')=='failed' and failed.get('commandOutcome')=='receipt_recorded'
            and failed.get('mayHaveRun') is True and failed.get('finalIdleOffVerified') is True
            and failed.get('modelSelected') is False and failed.get('trainingEligible') is False
            and failed.get('requiresOperatorReview') is False
            and isinstance(failed.get('errors'),list)
            and any(e.get('phase')=='after_observation' and e.get('code')=='m4_target_up_global_excursion'
                    for e in failed['errors'])
            and all(e.get('phase') in ('after_observation','observer_close','final_monitor','finish')
                    for e in failed['errors']), 'recovery_known_failed_UP_and_OFF_required')
    result_path=PurePosixPath(sources['sourceFailure']['path'])
    require(result_path.name=='result.json'
            and PurePosixPath(sources['sourceScope']['path'])==result_path.parent/'scope.json',
            'recovery_original_actor_siblings_required')
    ev=Evidence(root/str(result_path.parent))
    statuses,boot,first,stop_at=_http(ev,failed['receipt'],source_scope['pulse'])
    require(boot==scope['controllerBootId'],'recovery_failed_UP_boot_changed')
    # Include later forced-OFF cleanup writes, not merely the receipt trace end.
    latest=0
    for name in list(ev.hashes):
        if name.endswith('-response.json'):
            payload=ev.json(name).get('parsed',{})
            if payload.get('bootId')==boot and type(payload.get('latestSequence')) is int:
                latest=max(latest,payload['latestSequence'])
    require(latest>0,'recovery_original_final_cursor_missing')
    require(all(hashlib.sha256((ev.root/name).read_bytes()).hexdigest()==digest
                for name,digest in ev.hashes.items()),'recovery_failed_source_changed_during_audit')
    off=_decode_json(raw(sources['currentOff']))
    require(isinstance(off,dict) and set(off)=={'schemaVersion','controllerBootId','cameraBootId',
                'observedAt','outputSequence','snapshot','sourceProofs'} and off['schemaVersion']==OFF_SCHEMA
            and off['controllerBootId']==scope['controllerBootId']
            and off['cameraBootId']==scope['context']['gripperCameraBootId']
            and finite_number(off['observedAt']) and 0<=now-off['observedAt']<=CURRENT_OFF_MAX_AGE_SECONDS
            and type(off['outputSequence']) is int and off['outputSequence']==latest,
            'recovery_fresh_same_boot_current_OFF_required')
    proofs=off['sourceProofs'];require(isinstance(proofs,list) and 4<=len(proofs)<=128,'recovery_current_GET_proofs_required')
    observed={};output_pages=[];last_received=0.
    for proof in proofs:
        require(isinstance(proof,dict) and set(proof)=={'intent','response'},'recovery_GET_proof_fields')
        intent=_decode_json(raw(proof['intent']));response=_decode_json(raw(proof['response']))
        path=intent.get('path');endpoint=urllib.parse.urlsplit(path).path if isinstance(path,str) else None
        require(intent.get('method')=='GET' and intent.get('payload') is None
                and endpoint in ('/actions','/status','/collection_status','/sequence','/action_outputs')
                and response.get('httpStatus')==200
                and finite_number(intent.get('startedAt')) and finite_number(response.get('receivedAt'))
                and intent['startedAt']<=response['receivedAt']<=off['observedAt']
                and off['observedAt']-intent['startedAt']<=CURRENT_OFF_MAX_AGE_SECONDS,
                'recovery_current_GET_only')
        parsed=response_json(response['rawUtf8'],path)
        require(parsed==response.get('parsed'),'recovery_current_raw_response_changed')
        observed[endpoint]=parsed;last_received=max(last_received,response['receivedAt'])
        if endpoint=='/action_outputs':output_pages.append(parsed)
    if output_pages:
        observed['/action_outputs']={**output_pages[-1], 'entries':[entry for page in output_pages for entry in page['entries']]}
    snap=off['snapshot']
    require(isinstance(snap,dict) and set(snap).issuperset({'actions','status','outputs','sequence'})
            and observed.get('/actions')==snap['actions']
            and observed.get('/collection_status',observed.get('/status'))==snap['status']
            and observed.get('/action_outputs')==snap['outputs'] and observed.get('/sequence')==snap['sequence'],
            'recovery_snapshot_raw_proofs_changed')
    require(snap['actions'].get('bootId')==scope['controllerBootId'] and snap['actions'].get('active') is False,
            'recovery_current_action_or_boot_changed')
    checker=object.__new__(SequencePort)
    SequencePort._check_status(checker,snap['status'],'IDLE')
    angle=SequencePort._shoulder_angle(snap['status'])
    require(abs(angle-scope['expectedStartAngleDeg'])<=scope['startAngleToleranceDeg']
            and snap['sequence'].get('sequence',{}).get('state')=='IDLE'
            and snap['sequence']['sequence'].get('totalCount')==0,
            'recovery_current_pose_or_queue_changed')
    output=snap['outputs'];entries=output.get('entries');require(isinstance(entries,list) and bool(entries),
                                                              'recovery_current_driver_entries_required')
    require(output.get('bootId')==scope['controllerBootId'] and output.get('evidence')=='driver_write'
            and output.get('clockDomain')=='controller_boot_millis' and output.get('gap') is False
            and output.get('hasMore') is False and output.get('latestSequence')==latest,
            'recovery_current_driver_cursor_changed')
    previous=None;motors={}
    for entry in entries:
        require(type(entry.get('sequence')) is int and (previous is None or entry['sequence']==previous+1)
                and type(entry.get('motor')) is int and 1<=entry['motor']<=5
                and type(entry.get('pwm')) is int,'recovery_current_driver_entries_invalid')
        previous=entry['sequence'];motors[entry['motor']]=entry['pwm']
    require(previous==latest and motors=={i:0 for i in range(1,6)},'recovery_current_driver_not_OFF')
    auth=_decode_json(raw(sources['authorization']))
    require(isinstance(auth,dict) and set(auth)=={'schemaVersion','approved','automaticRetry','modelSelected',
                'trainingEligible','userInstruction','sourceFailureSha256','currentOffSha256','motionContractSha256'}
            and auth['schemaVersion']==AUTH_SCHEMA and auth['approved'] is True
            and auth['automaticRetry'] is False and auth['modelSelected'] is False
            and auth['trainingEligible'] is False and isinstance(auth['userInstruction'],str) and auth['userInstruction']
            and auth['sourceFailureSha256']==sources['sourceFailure']['sha256']
            and auth['currentOffSha256']==sources['currentOff']['sha256']
            and auth['motionContractSha256']==recovery_motion_digest(scope),
            'recovery_explicit_motion_authorization_required')
    require(all(hashlib.sha256(_read(root,path)).hexdigest()==digest for path,digest in hashes.items()),
            'recovery_source_changed_during_validation')
    return dict(sourceFailureSha256=sources['sourceFailure']['sha256'],currentOutputSequence=latest,
                currentAngleDeg=angle,currentOffValidUntil=off['observedAt']+CURRENT_OFF_MAX_AGE_SECONDS,
                sourceProofsSha256=plan_digest(ev.hashes),currentProofsSha256=plan_digest(hashes),
                originalOutcomeChanged=False,trainingEligible=False,modelSelected=False,devicesAccessed=False)
