"""독립 수동 2D 판독과 원본 재관측으로 제한된 추적 범위만 검증한다."""
import copy
import json
from pathlib import Path

import numpy as np
from learning.batch_observation import require, digest, number
from learning.batch_learning import command_key

SCHEMA = 'motionbrain.batch_motion_admission.v1'


def _read(repository, binding):
    from learning.batch_runtime import private_path, sha
    require(isinstance(binding,dict) and set(binding)=={'path','sha256'},'admission_file_binding')
    raw=private_path(Path(repository).resolve(),binding['path']).read_bytes()
    require(sha(raw)==binding['sha256'],'admission_artifact_hash_changed')
    return json.loads(raw)


def install_validation_scope(manager, repository, scope_binding):
    """새 수동 촬영 세션에 명령 전에 고정 근거를 설치한다. 모터 명령은 보내지 않는다."""
    from learning.batch_runtime import sha
    scope=_read(repository,scope_binding)
    _scope(scope)
    require(manager.current is not None and manager.current.get('phase')=='recording'
        and manager.command_count==0 and not any(e.get('kind')=='command' for e in manager.current['events']),
        'admission_scope_must_precede_command')
    envelope=dict(scope=scope,scopeSha256=scope_binding['sha256'],sessionId=manager.current['sessionId'],
                  at=manager.clock.monotonic())
    manager.store.write_json('commissioning-validation-scope.json',envelope)
    # ArtifactStore serialization is authoritative, not a presumed JSON encoding.
    sealed=sha((manager.store.root/'commissioning-validation-scope.json').read_bytes())
    manager.current['validationScopeSha256']=sealed
    manager._event(dict(kind='commissioning_validation_scope',at=envelope['at'],sha256=sealed))
    return dict(status='scope_installed_no_motion',sha256=sealed)


def _scope(scope):
    require(isinstance(scope,dict) and set(scope)=={'schemaVersion','development','observerBinding','maxLabelTolerancePx'}
        and scope['schemaVersion']=='motionbrain.motion_validation_scope.v1'
        and isinstance(scope['development'],list) and bool(scope['development'])
        and number(scope['maxLabelTolerancePx']) and 0<scope['maxLabelTolerancePx']<=3,
        'admission_validation_scope_schema')


def build_admission(repository, manifest):
    """입력 파일을 재감사한다. 실제 실행·인증·파일 작성은 하지 않는다."""
    from learning.batch_commissioning import load_learning_transition
    require(isinstance(manifest,dict) and set(manifest)=={'development','holdout','validationScope'},'admission_manifest')
    require(isinstance(manifest['development'],list) and bool(manifest['development']) and
            isinstance(manifest['holdout'],list) and len(manifest['holdout'])>=2,'admission_independent_evidence_required')
    frozen_scope=_read(repository,manifest['validationScope']);_scope(frozen_scope)
    require(frozen_scope['development']==manifest['development'],'admission_development_scope_changed')
    identities=set();groups=set()
    def identity(row):
        return {row['sourceHash'],row['sessionId'],row['episodeId'],*row['frameIds'],*row.get('pairedFrameIds',[])}
    for entry in manifest['development']:
        row,_=load_learning_transition(repository,entry,'training')
        identities.update(identity(row));groups.add(row['captureGroup'])
    rows=[];label_hashes=set();contract=None
    for item in manifest['holdout']:
        require(isinstance(item,dict) and set(item)=={'transition','labels'},'admission_holdout_entry')
        row,_=load_learning_transition(repository,item['transition'],'evaluation')
        require(not identities & identity(row) and row['captureGroup'] not in groups,'admission_development_overlap')
        identities.update(identity(row))
        from learning.batch_runtime import private_path, sha
        source=private_path(Path(repository).resolve(),item['transition']['sourcePath'])
        envelope_raw=(source/'commissioning-validation-scope.json').read_bytes()
        envelope=json.loads(envelope_raw)
        session_raw=(source/'session.json').read_bytes()
        require(sha(session_raw)==row['sourceHash'],'admission_session_changed')
        session=json.loads(session_raw)
        intent=json.loads((source/'command-001-intent.json').read_bytes())
        require(envelope.get('scope')==frozen_scope and envelope.get('scopeSha256')==manifest['validationScope']['sha256']
            and envelope.get('sessionId')==row['sessionId'] and intent.get('validationScopeSha256')==sha(envelope_raw)
            and session.get('validationScopeSha256')==sha(envelope_raw)
            and number(envelope.get('at')) and envelope['at']<intent['at'], 'admission_original_precommand_scope_required')
        require(dict(kind='commissioning_validation_scope',at=envelope['at'],sha256=sha(envelope_raw)) in session['events'],
            'admission_original_scope_event_required')
        transition_raw=private_path(Path(repository).resolve(),item['transition']['transitionPath']).read_bytes()
        require(sha(transition_raw)==item['transition']['transitionSha256'] and
            json.loads(transition_raw)['binding']==frozen_scope['observerBinding'],'admission_observer_not_prefixed')
        current={k:row[k] for k in ('joint','context','stateFields')}
        require(contract is None or contract==current,'admission_observation_contract_changed');contract=current
        labels=_read(repository,item['labels'])
        require(item['labels']['sha256'] not in label_hashes,'admission_reused_labels')
        label_hashes.add(item['labels']['sha256'])
        require(isinstance(labels,dict) and set(labels)=={'method','reviewer','sourceHash','frameIds',
            'pairedFrameIds','stateFields','before','after','tolerancePx','visible','identityVerified'},'admission_labels_schema')
        require(labels['method']=='manual_projected_state_review' and isinstance(labels['reviewer'],str)
            and bool(labels['reviewer']) and labels['visible'] is True and labels['identityVerified'] is True,
            'admission_manual_visible_identity_review_required')
        require(all(labels[k]==row[k] for k in ('sourceHash','frameIds','pairedFrameIds','stateFields')),
            'admission_labels_source_changed')
        tolerance=labels['tolerancePx']
        require(number(tolerance) and tolerance==frozen_scope['maxLabelTolerancePx'],'admission_label_tolerance')
        size=len(row['stateFields'])
        for side in ('before','after'):
            points=labels[side]
            require(isinstance(points,list) and len(points)==size and all(number(v) for v in points),
                'admission_label_dimension')
            require(np.max(np.abs(np.asarray(points)-row[side]))<=tolerance,'admission_tracker_label_disagreement')
        delta=np.asarray(labels['after'])-labels['before']
        tracked=np.asarray(row['after'])-row['before']
        moving=np.abs(delta)>2*tolerance
        require(bool(moving.any()) and np.all(delta[moving]*tracked[moving]>0), 'admission_motion_not_resolved')
        rows.append(row)
    commands={}
    for row in rows: commands.setdefault(command_key(row['command']),[]).append(row)
    require(all(len(group)>=2 for group in commands.values()),'admission_command_support_required')
    scopes={}
    for key,group in commands.items():
        before=np.asarray([r['before'] for r in group]);after=np.asarray([r['after'] for r in group])
        scopes[key]=dict(command=group[0]['command'],count=len(group),
            beforeMinimum=before.min(axis=0).tolist(),beforeMaximum=before.max(axis=0).tolist(),
            afterMinimum=after.min(axis=0).tolist(),afterMaximum=after.max(axis=0).tolist())
    return dict(schemaVersion=SCHEMA,manifest=copy.deepcopy(manifest),contract=contract,scopes=scopes,
        evidenceSemantics='independent_manual_2d_tracking_agreement_not_measurement_accuracy',
        measurementAccuracyVerified=False,contactVerified=False,physicalGraspVerified=False,
        executionAuthorized=False)


def verify_admission(repository, binding):
    stored=_read(repository,binding)
    require(stored.get('schemaVersion')==SCHEMA,'admission_schema')
    require(build_admission(repository,stored['manifest'])==stored,'admission_evidence_changed')
    return stored


def require_scope(admission, command, context, fields, state=None, *, side='before'):
    contract=dict(joint=command['joint'],context=context,stateFields=fields)
    require(contract==admission['contract'],'admission_runtime_contract_changed')
    scope=admission['scopes'].get(command_key(command))
    require(scope is not None,'admission_command_not_supported')
    require(side in ('before','after'),'admission_state_side')
    if state is not None:
        require(isinstance(state,list) and len(state)==len(fields) and all(number(v) for v in state)
            and all(lo<=v<=hi for v,lo,hi in zip(state,scope[side+'Minimum'],scope[side+'Maximum'])),
            'admission_state_outside_verified_range')
