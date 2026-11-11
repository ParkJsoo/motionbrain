"""사전 고정 명령의 새 자료 평가. 외삽 예측은 평가 전용이며 실행 선택에 쓰지 않는다."""
import copy
import json
import time
from pathlib import Path

import numpy as np
from learning.batch_observation import require, number, digest
from learning.batch_learning import (validate_model, model_sha256, command_key, vector, _rows, _metrics)
from learning.batch_motion_admission import _read
from learning.artifacts import _read as read_source
from learning.batch_runtime import private_path, sha
from learning.experiment_pipeline import encoded

FREEZE_SCHEMA='motionbrain.fixed_command_model_freeze.v1'
PROOF_SCHEMA='motionbrain.fixed_command_prospective.v1'


def freeze_model(repository,model_binding,path,*,clock=time):
    model=_read(repository,model_binding);validate_model(model)
    require(model_sha256(model)==model_binding['sha256'],'fixed_evaluation_canonical_model_required')
    path=private_path(Path(repository).resolve(),str(path))
    value=dict(schemaVersion=FREEZE_SCHEMA,model=copy.deepcopy(model_binding),
        modelSha256=model_sha256(model),frozenAt=clock.monotonic(),
        semantics='fixed_command_evaluation_not_model_selection',controlEligible=False)
    from learning.batch_review_response import _publish
    path.parent.mkdir(parents=True,exist_ok=True);raw=encoded(value);_publish(path,raw)
    return dict(path=str(path),sha256=sha(raw))


def load_freeze(repository,binding):
    value=_read(repository,binding)
    require(isinstance(value,dict) and set(value)=={'schemaVersion','model','modelSha256','frozenAt','semantics','controlEligible'} and
        value['schemaVersion']==FREEZE_SCHEMA and number(value['frozenAt']) and
        value['semantics']=='fixed_command_evaluation_not_model_selection' and value['controlEligible'] is False,
        'fixed_evaluation_freeze_schema')
    model=_read(repository,value['model']);validate_model(model)
    require(model_sha256(model)==value['modelSha256']==value['model']['sha256'],'fixed_evaluation_model_changed')
    return value,model


def predictions(model,before,command,context):
    validate_model(model)
    require(context==model['context'] and command['joint']==model['joint'],'fixed_evaluation_context_changed')
    x=vector(before,len(model['stateFields']));entry=model['models'].get(command_key(command))
    require(entry is not None,'fixed_evaluation_command_unsupported')
    mean=np.asarray(entry['deltaMean'])
    predicted=x+mean+((x-np.asarray(entry['mean']))/np.asarray(entry['scale']))@np.asarray(entry['coefficient'])
    require(np.isfinite(predicted).all(),'fixed_evaluation_nonfinite_prediction')
    return dict(stateRidge=predicted.tolist(),commandMean=(x+mean).tolist(),noChange=x.tolist(),
        insideControlSupport=all(lo<=v<=hi for v,lo,hi in zip(x,entry['minimum'],entry['maximum'])),
        controlEligible=False,selectionSemantics='predeclared_command_not_model_selected')


def prospective_evidence(model,freeze,row,input_frames,prepared_at):
    require(row.get('stateFields')==model['stateFields'] and freeze['modelSha256']==model_sha256(model),
            'fixed_evaluation_contract_changed')
    require(number(prepared_at), 'fixed_evaluation_input_time')
    require(isinstance(input_frames,list) and len(input_frames)==2 and
        all(isinstance(f,dict) and set(f)=={'frameId','pixelSha256','capturedAt','persistedAt'} and
            isinstance(f['frameId'],str) and f['frameId'] and isinstance(f['pixelSha256'],str) and
            len(f['pixelSha256'])==64 and all(c in '0123456789abcdef' for c in f['pixelSha256']) and
            number(f['capturedAt']) and number(f['persistedAt']) and
            freeze['frozenAt']<f['capturedAt']<=f['persistedAt']<=prepared_at for f in input_frames) and
        len({f['frameId'] for f in input_frames})==2,'fixed_evaluation_new_persisted_inputs_required')
    require(number(prepared_at) and number(row.get('beforePersistedAt')) and row['beforePersistedAt']==input_frames[0]['persistedAt'],
            'fixed_evaluation_input_time')
    return dict(schemaVersion=PROOF_SCHEMA,sessionId=row['sessionId'],modelSha256=model_sha256(model),
        freeze=copy.deepcopy(freeze),beforeStateSha256=digest(row['before']),commandSha256=digest(row['command']),
        predictions=predictions(model,row['before'],row['command'],row['context']),
        inputFrames=copy.deepcopy(input_frames),beforePersistedAt=row['beforePersistedAt'],preparedAt=prepared_at)


def seal_before(manager,proof):
    require(manager.current is not None and manager.current['phase']=='recording' and manager.command_count==0 and
            not any(e.get('kind')=='command' for e in manager.current['events']) and
            proof['sessionId']==manager.current['sessionId'],'fixed_evaluation_seal_must_precede_command')
    manager.store.write_json('batch-fixed-prospective.json',proof)
    identity=sha(read_source(manager.store.root,'batch-fixed-prospective.json'))
    manager.current['fixedEvaluationProofSha256']=identity
    manager._event(dict(kind='fixed_evaluation_proof',sha256=identity,at=proof['preparedAt']))


def audit_proof(repository,source,transition,model,freeze_binding):
    freeze,frozen=load_freeze(repository,freeze_binding)
    require(model_sha256(model)==model_sha256(frozen),'fixed_evaluation_expected_model_changed')
    source=private_path(Path(repository).resolve(),str(source));row=transition['learningRow']
    session_raw=read_source(source,'session.json');session=json.loads(session_raw)
    require(sha(session_raw)==row['sourceHash'],'fixed_evaluation_original_session_changed')
    intent=json.loads(read_source(source,'command-001-intent.json'))
    raw=read_source(source,'batch-fixed-prospective.json');proof=json.loads(raw);identity=sha(raw)
    require(intent.get('fixedEvaluationProofSha256')==identity and session.get('fixedEvaluationProofSha256')==identity and
        dict(kind='fixed_evaluation_proof',sha256=identity,at=proof['preparedAt']) in session['events'] and
        proof['preparedAt']<intent['at']==row['commandAt'],'fixed_evaluation_original_precommand_seal_required')
    frames=[]
    meta=transition['before']['derivedMetadata']
    frames.append({k:meta[k] for k in ('frameId','pixelSha256','capturedAt','persistedAt')})
    paired=transition['gripperPair']['beforeObservation']['frameBinding']
    frames.append({k:paired[k] for k in ('frameId','pixelSha256','capturedAt','persistedAt')})
    expected=prospective_evidence(model,freeze,row,frames,proof['preparedAt'])
    require(proof==expected,'fixed_evaluation_prospective_evidence_changed')
    return proof


def audit_jaw_proof(repository,entry,model,freeze_binding):
    """양턱 5D는 집게 원본으로, 외부는 인과적 원본 증거로만 재감사한다."""
    from learning.registered_jaw_transition import load_learning_transition
    from learning.gripper_camera_dataset import read_frame
    from learning.manual_collection_dataset import select_frames,POLICY
    row,_=load_learning_transition(Path(repository).resolve(),entry,'evaluation')
    freeze,frozen=load_freeze(repository,freeze_binding)
    require(model_sha256(model)==model_sha256(frozen),'fixed_evaluation_expected_model_changed')
    source=private_path(Path(repository).resolve(),entry['sourcePath'])
    session_raw=read_source(source,'session.json');session=json.loads(session_raw)
    require(sha(session_raw)==row['sourceHash'],'fixed_evaluation_original_session_changed')
    intent=json.loads(read_source(source,'command-001-intent.json'))
    raw=read_source(source,'batch-fixed-prospective.json');proof=json.loads(raw);identity=sha(raw)
    require(intent.get('fixedEvaluationProofSha256')==identity and session.get('fixedEvaluationProofSha256')==identity and
        dict(kind='fixed_evaluation_proof',sha256=identity,at=proof['preparedAt']) in session['events'] and
        proof['preparedAt']<intent['at']==row['commandAt'],'fixed_evaluation_original_precommand_seal_required')
    grip,metadata,_=read_frame(source/'gripper-pair','before',row['sessionId'])
    require(grip['frameId']==row['pairedFrameIds'][0] and
        0<=row['commandAt']-metadata['requestStartedAt']<=10.,'fixed_jaw_input_stale_or_changed')
    frames=[dict(frameId=grip['frameId'],pixelSha256=grip['pixelSha256'],
        capturedAt=metadata['requestStartedAt'],persistedAt=metadata['persistedAt'])]
    require(isinstance(proof.get('inputFrames'),list) and len(proof['inputFrames'])==2,'fixed_jaw_two_source_frames_required')
    external_id=proof['inputFrames'][1]['frameId']
    window=[row['commandAt']+offset for offset in POLICY['preSeconds']]
    eligible=select_frames(session['frames'],window,row['commandAt'])
    matches=[frame for frame in eligible if frame['frameId']==external_id]
    require(len(matches)==1,'fixed_jaw_external_not_in_original_pre_window')
    external=matches[0]
    frames.append(dict(frameId=external['frameId'],pixelSha256=external['pixelSha256'],
        capturedAt=external['sourceEarliest'],persistedAt=external['stdoutReadAt']+external['persistenceMs']/1000))
    expected=prospective_evidence(model,freeze,row,frames,proof['preparedAt'])
    require(proof==expected,'fixed_jaw_prospective_evidence_changed')
    return proof


def evaluate(model,rows,proofs):
    validate_model(model);first,identities=_rows(rows)
    require(all(first[k]==model[k] for k in ('joint','context','stateFields')),'fixed_evaluation_context_changed')
    for key,values in identities.items():
        require(not values & set(model['trainingIdentities'][key]),'fixed_evaluation_training_overlap:'+key)
    require(not {r['captureGroup'] for r in rows} & set(model['trainingCaptureGroups']),'fixed_evaluation_group_overlap')
    groups={name:{k:[] for k in ('stateRidge','commandMean','noChange')} for name in ('all','insideControlSupport','outsideControlSupport')}
    accepted=[];rejected=[]
    for row in rows:
        try:
            proof=proofs[row['sessionId']]
            expected=prospective_evidence(model,proof['freeze'],row,proof['inputFrames'],proof['preparedAt'])
            require(proof==expected and proof['preparedAt']<row['commandAt'],'fixed_evaluation_proof_changed')
            prediction=proof['predictions'];group='insideControlSupport' if prediction['insideControlSupport'] else 'outsideControlSupport'
            for key in groups['all']:
                pair=(prediction[key],row['after']);groups['all'][key].append(pair);groups[group][key].append(pair)
            accepted.append(dict(sessionId=row['sessionId'],**copy.deepcopy(prediction)))
        except (ValueError,KeyError,TypeError) as exc:
            rejected.append(dict(sessionId=row['sessionId'],reason=str(exc)))
    return dict(kind='prospectiveFixedCommandEvaluation',listed=len(rows),evaluated=len(accepted),
        predictions=accepted,rejected=rejected,metrics={g:{k:_metrics(v) for k,v in values.items()} for g,values in groups.items()},
        insideControlSupportCount=sum(p['insideControlSupport'] for p in accepted),
        outsideControlSupportCount=sum(not p['insideControlSupport'] for p in accepted),
        prospectiveEvidenceVerified=bool(accepted) and not rejected,modelSha256=model_sha256(model),
        modelSelected=False,controlEligible=False,evaluationPassed=None)
