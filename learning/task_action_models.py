"""장치 접근 없는 현재 과업의 M4/OPEN 후보 비교.

수동 원본 감사는 task_transition_data의 책임이다. 이 모듈은 주어진 개발
전이를 학습할 뿐 원본의 진실성, 접촉, 보유, 실물 제어 적격을 증명하지 않는다.
기존 M4 UP/CLOSE 계약을 변경하지 않으며 선택 결과는 실행 허용이 아니다.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import re

import numpy as np

from learning import m1_shaft_learning as shaft
from learning.pipeline_learning import canonical_bytes, digest
from learning.manual_collection_dataset import require

SCHEMA = 'motionbrain.task_m4_action_model.v1'
FIELDS = ('objectRisePx', 'toolRisePx', 'slipDyPx', 'startAngleDeg')
SUPPORT = ('table_supported', 'gripper_supported')


def _number(x):
    return type(x) in (int, float) and math.isfinite(x)


def _hash(x):
    return isinstance(x, str) and re.fullmatch('[0-9a-f]{64}', x) is not None


def _context(context):
    require(isinstance(context, dict) and _hash(context.get('registrationSha256'))
            and all(isinstance(context.get(k), str) and bool(context[k])
                    for k in ('object', 'physicalView', 'poseRegime')), 'task_registered_context_required')
    canonical_bytes(context)


def _state(value):
    require(isinstance(value, dict) and set(value) == set(FIELDS)
            and all(_number(value[k]) for k in FIELDS), 'task_visual_state_required')
    require(127.58 < value['startAngleDeg'] < 296.12, 'task_start_angle_outside_limits')
    return np.asarray([value[k] for k in FIELDS], dtype=float)


def _command(command, direction=None):
    require(isinstance(command, dict) and set(command) == {'joint', 'direction', 'percent', 'durationMs'},
            'task_command_fields')
    require(command['joint'] == 'shoulder' and command['direction'] in ('up', 'down')
            and (direction is None or command['direction'] == direction), 'task_m4_direction_required')
    ceiling = (50, 650) if command['direction'] == 'up' else (35, 400)
    require(type(command['percent']) is int and 1 <= command['percent'] <= ceiling[0]
            and type(command['durationMs']) is int and 1 <= command['durationMs'] <= ceiling[1],
            'task_m4_command_ceiling')
    return digest(command)


def _group_rows(rows, context):
    _context(context)
    require(isinstance(rows, list), 'task_training_rows_required')
    groups, seen = {}, {k: set() for k in ('id', 'source', 'episode', 'frame')}
    for row in rows:
        require(isinstance(row, dict) and set(row) == {'id','context','supportMode','command','before',
                'after','sourceHash','episode','frameIds'}, 'task_transition_fields')
        require(row['context'] == context and row['supportMode'] in SUPPORT,
                'task_training_context_or_support_changed')
        require(isinstance(row['id'], str) and bool(row['id']) and _hash(row['sourceHash']),
                'task_source_identity_required')
        require(isinstance(row['episode'], list) and len(row['episode']) == 2
                and all(isinstance(v, str) and v for v in row['episode']), 'task_episode_required')
        require(isinstance(row['frameIds'], list) and len(row['frameIds']) >= 2
                and all(isinstance(v,str) and v for v in row['frameIds'])
                and len(set(row['frameIds'])) == len(row['frameIds']), 'task_frame_pair_required')
        identities = dict(id={row['id']},source={row['sourceHash']},episode={tuple(row['episode'])},
                          frame=set(row['frameIds']))
        for name, values in identities.items():
            require(not seen[name].intersection(values), 'task_reused_'+name)
            seen[name].update(values)
        a,b = _state(row['before']), _state(row['after'])
        key = _command(row['command'])+':'+row['supportMode']
        groups.setdefault(key, []).append((row,a,b))
    return groups


def _fit_m4(rows,context):
    require(isinstance(rows,list) and len(rows)>=2, 'task_training_rows_required')
    groups=_group_rows(rows,context)
    entries=[]
    for values in groups.values():
        require(len(values) >= 2, 'task_two_sources_per_candidate_and_support_required')
        x=np.asarray([v[1] for v in values]); delta=np.asarray([v[2]-v[1] for v in values])
        mean=x.mean(axis=0); scale=np.maximum(x.std(axis=0),1.)
        z=(x-mean)/scale; average=delta.mean(axis=0)
        coefficient=np.linalg.solve(z.T@z+np.eye(len(FIELDS)),z.T@(delta-average))
        entries.append(dict(command=copy.deepcopy(values[0][0]['command']),supportMode=values[0][0]['supportMode'],
                            count=len(values),minimum=x.min(axis=0).tolist(),maximum=x.max(axis=0).tolist(),
                            mean=mean.tolist(),scale=scale.tolist(),deltaMean=average.tolist(),
                            coefficient=coefficient.tolist()))
    model=dict(schemaVersion=SCHEMA,context=copy.deepcopy(context),stateFields=list(FIELDS),
               models=entries,trainingRows=copy.deepcopy(rows),controlEligible=False,
               independentTest=False,supportMeaning='per_candidate_support_mode_before_axis_bounds_not_confidence')
    return model


def fit_m4(rows,context):
    """후보·지지 상태별 독립 2행 이상. 원본 감사·실물 검증과 구분한다."""
    model=_fit_m4(rows,context)
    return validate_m4(model)


def validate_m4(model):
    require(isinstance(model,dict) and model.get('schemaVersion') == SCHEMA
            and model.get('stateFields') == list(FIELDS) and model.get('controlEligible') is False
            and model.get('independentTest') is False, 'task_m4_model_schema')
    _context(model['context'])
    require(isinstance(model.get('models'),list) and bool(model['models']), 'task_model_candidates_required')
    seen=set()
    for e in model['models']:
        key=(_command(e['command']),e['supportMode'])
        require(e['supportMode'] in SUPPORT and key not in seen and type(e['count']) is int
                and e['count'] >= 2, 'task_model_candidate_identity')
        seen.add(key)
        for name in ('minimum','maximum','mean','scale','deltaMean'):
            require(isinstance(e[name],list) and len(e[name]) == len(FIELDS)
                    and all(_number(v) for v in e[name]), 'task_model_vector')
        require(np.asarray(e['coefficient']).shape == (len(FIELDS),len(FIELDS))
                and np.isfinite(np.asarray(e['coefficient'],dtype=float)).all(), 'task_model_coefficient')
        require(all(lo <= mid <= hi and s >= 1 for lo,mid,hi,s in
                    zip(e['minimum'],e['mean'],e['maximum'],e['scale'])), 'task_model_support')
    canonical_bytes(model)
    require(model == _fit_m4(model.get('trainingRows'),model['context']), 'task_model_training_derivation_changed')
    return model


def coverage_m4(rows,context,required_probes):
    """부분 자료의 수집 빈칸만 보고한다. 적격 모델이나 실행 권한을 만들지 않는다."""
    groups=_group_rows(rows,context)
    require(isinstance(required_probes,list) and bool(required_probes),'task_required_probes_required')
    reports=[]
    for probe in required_probes:
        require(isinstance(probe,dict) and set(probe)=={'stage','command','supportMode','state'}
                and probe['stage'] in ('lift','lower') and probe['supportMode'] in SUPPORT, 'task_probe_fields')
        key=_command(probe['command'],'up' if probe['stage']=='lift' else 'down')+':'+probe['supportMode']
        state=_state(probe['state']); values=groups.get(key,[])
        minimum=maximum=None; within=False
        if values:
            x=np.asarray([v[1] for v in values]);minimum=x.min(axis=0);maximum=x.max(axis=0)
            within=bool(np.all(state>=minimum) and np.all(state<=maximum))
        reports.append(dict(probe=copy.deepcopy(probe),observations=len(values),
            missingRows=max(0,2-len(values)),observedBeforeBounds=None if minimum is None else
                dict(minimum=dict(zip(FIELDS,minimum.tolist())),maximum=dict(zip(FIELDS,maximum.tolist()))),
            withinObservedBounds=within,modelSupportCovered=within and len(values)>=2,
            needsStateCoverage=not within))
    return dict(context=copy.deepcopy(context),probes=reports,
                controlEligible=False,executionAuthorized=False,
                meaning='two_rows_is_fit_minimum_not_physical_validation_or_success_rate')


def _rank(stop, candidates, rejected, model_sha, context, margin):
    require(_number(margin) and margin >= 0, 'task_nonnegative_margin_required')
    ranked=[stop]+candidates
    ranked.sort(key=lambda x:x['error'])
    selected=ranked[0]
    if stop['error']-selected['error'] <= margin:
        selected=stop
    return dict(selected=copy.deepcopy(selected),candidates=ranked,rejected=rejected,
                modelSha256=model_sha,context=copy.deepcopy(context),controlEligible=False,
                executionAuthorized=False)


def select_m4(model,state,target,commands,context,support_mode,*,minimum_improvement=0.):
    """비교 비용은 지정한 네 상태의 제곱 거리다. 안전성·실제 높이를 증명하지 않는다."""
    validate_m4(model)
    require(context == model['context'] and support_mode in SUPPORT, 'task_observation_context_changed')
    x,y=_state(state),_state(target)
    require(isinstance(commands,list) and bool(commands), 'task_candidates_required')
    keys=[_command(c) for c in commands]
    require(len(keys)==len(set(keys)), 'task_duplicate_candidate')
    stop=dict(kind='stop',command=None,predictedState=copy.deepcopy(state),error=float(np.sum((x-y)**2)))
    accepted,rejected=[],[]
    for command in commands:
        matches=[e for e in model['models'] if e['command']==command and e['supportMode']==support_mode]
        reason=None
        if len(matches)!=1: reason='task_candidate_support_missing'
        elif np.any(x<np.asarray(matches[0]['minimum'])) or np.any(x>np.asarray(matches[0]['maximum'])):
            reason='task_before_outside_training_range'
        if reason:
            rejected.append(dict(command=copy.deepcopy(command),reason=reason));continue
        e=matches[0]; predicted=x+np.asarray(e['deltaMean'])+((x-np.asarray(e['mean']))/np.asarray(e['scale']))@np.asarray(e['coefficient'])
        if not 127.58 < predicted[-1] < 296.12:
            rejected.append(dict(command=copy.deepcopy(command),reason='task_predicted_angle_outside_limits'));continue
        accepted.append(dict(kind='action',command=copy.deepcopy(command),
                             predictedState=dict(zip(FIELDS,predicted.tolist())),error=float(np.sum((predicted-y)**2))))
    return _rank(stop,accepted,rejected,digest(model),context,minimum_improvement)


def select_open(model,shaft_px,target,commands,context,*,table_supported,m4_angle_deg,
                expected_m4_angle_deg,tolerance_deg=.25,minimum_improvement=0.):
    """기존 shaft 예측을 쓰는 OPEN 전용 어댑터. CLOSE 선택 계약은 보존한다."""
    shaft.validate_model(model)
    require(table_supported is True, 'task_release_requires_table_support')
    require(all(_number(v) for v in (m4_angle_deg,expected_m4_angle_deg,tolerance_deg))
            and 0 < tolerance_deg <= .25 and abs(m4_angle_deg-expected_m4_angle_deg)<=tolerance_deg,
            'task_release_M4_pose_changed')
    require(context==model['context'], 'task_observation_context_changed')
    x,y=shaft._scalar(shaft_px),shaft._scalar(target)
    require(isinstance(commands,list) and bool(commands), 'task_candidates_required')
    keys=[];accepted=[];rejected=[]
    for command in commands:
        require(isinstance(command,dict) and set(command)=={'joint','direction','percent','durationMs'}
                and command['joint']=='gripper' and command['direction']=='open'
                and type(command['percent']) is int and 1<=command['percent']<=60
                and type(command['durationMs']) is int and 1<=command['durationMs']<=650,
                'task_bounded_OPEN_only')
        keys.append(digest(command))
        try: prediction=shaft.predict(model,x,command,context)
        except ValueError as exc:
            rejected.append(dict(command=copy.deepcopy(command),reason=str(exc)));continue
        accepted.append(dict(kind='action',command=copy.deepcopy(command),predictedState={'shaftSeparationPx':prediction},
                             error=(prediction-y)**2))
    require(len(set(keys))==len(keys),'task_duplicate_candidate')
    stop=dict(kind='stop',command=None,predictedState={'shaftSeparationPx':x},error=(x-y)**2)
    return _rank(stop,accepted,rejected,digest(model),context,minimum_improvement)


def freeze_model(path,model):
    """새 파일에만 고정하며 JSON 파일 바이트의 SHA를 반환한다."""
    if model.get('schemaVersion') == SCHEMA: validate_m4(model)
    else: shaft.validate_model(model)
    raw=canonical_bytes(model)
    with Path(path).open('xb') as handle: handle.write(raw)
    import hashlib
    return hashlib.sha256(raw).hexdigest()


def load_frozen(path,sha256):
    import hashlib
    raw=Path(path).read_bytes()
    require(_hash(sha256) and hashlib.sha256(raw).hexdigest()==sha256,'task_frozen_model_changed')
    model=json.loads(raw)
    if model.get('schemaVersion') == SCHEMA: validate_m4(model)
    else: shaft.validate_model(model)
    return model
