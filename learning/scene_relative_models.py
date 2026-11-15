"""원본 영상을 다시 관측해 만드는 별도 개발 모델. 기존 고정 모델을 재표기하지 않는다.

관측 좌표 계약과 물리 조건을 독립적으로 고정한다. 좌표 정규화는 다른 M4 자세,
하중, 출력 정책의 동등성 증거가 아니며 이 모델은 실물 제어 적격을 부여하지 않는다.
"""
from __future__ import annotations
import copy
import hashlib
from pathlib import Path
from learning.pipeline_learning import canonical_bytes
import numpy as np
from learning.manual_collection_dataset import require
from learning.pipeline_learning import digest
from learning.relative_action_learning import command_key
from learning.scene_relative_observation import FIELDS

SCHEMA = 'motionbrain.scene_relative_action_ridge.v1'
STATE_FIELDS = [*FIELDS, 'm4AngleDeg']
from learning.scene_object_relative_observation import FIELDS as OBJECT_FIELDS
OBJECT_STATE_FIELDS=[*OBJECT_FIELDS,'m4AngleDeg']
OBJECT_SCHEMA='motionbrain.scene_object_relative_action_ridge.v1'

def vector(value):
    require(isinstance(value, list) and len(value) == 4
            and all(type(v) in (int, float) and np.isfinite(v) for v in value)
            and value[2] >= 0 and 0 <= value[3] < 360, 'scene_four_finite_state_fields_required')
    return np.asarray(value, dtype=float)


def derive_transition(source, observer, before_bytes, after_bytes, *, audit_source):
    """audit_source는 원본 명령/성공/OFF/프레임/물리 조건을 재감사한 정확한 source를 반환한다.

    임의 성공 boolean만으로는 원본을 승인하지 않는다. 원 실패·unknown·미시작은
    독립적으로 거절한다. 관측자의 trainingEligible=False도 보존한다.
    """
    require(callable(audit_source) and audit_source(copy.deepcopy(source)) == source,
            'scene_source_reaudit_required')
    require(source.get('status') == 'completed' and source.get('finalIdleOffVerified') is True
            and source.get('trainingEligible') is True, 'scene_source_not_training_eligible')
    require(isinstance(source.get('physicalContext'), dict)
            and {'object', 'load', 'otherJointConfiguration', 'executionTimingPolicy', 'supportMode'} <= set(source['physicalContext']),
            'scene_physical_context_required')
    require(isinstance(source.get('episode'), list) and len(source['episode']) == 2
            and all(isinstance(v,str) and v for v in source['episode']), 'scene_episode_required')
    command_key(source['command'])
    observed = []
    for side, raw in (('before', before_bytes), ('after', after_bytes)):
        frame = source[side]
        require(hashlib.sha256(raw).hexdigest() == frame['imageSha256'], 'scene_source_image_changed')
        observed.append(observer.observe(raw, frame['frameId'], frame['sourceContext']))
    require(observed[0]['frameId'] != observed[1]['frameId'], 'scene_transition_distinct_frames')
    require(observed[0]['stateFields'] in (list(FIELDS),list(OBJECT_FIELDS))
            and all(o['stateFields'] == observed[0]['stateFields'] for o in observed)
            and observed[0]['canonicalFrameSha256'] == observed[1]['canonicalFrameSha256'],
            'scene_transition_canonical_frame_changed')
    require(observed[0]['registrationSha256'] == observed[1]['registrationSha256'],
            'scene_transition_registration_changed')
    return dict(sourceSha256=digest(source), source=copy.deepcopy(source),
                command=copy.deepcopy(source['command']), physicalContext=copy.deepcopy(source['physicalContext']),
                observationContract=dict(canonicalFrameSha256=observed[0]['canonicalFrameSha256'], stateFields=[*observed[0]['stateFields'],'m4AngleDeg']),
                before=[*observed[0]['state'], source['before']['m4AngleDeg']],
                after=[*observed[1]['state'], source['after']['m4AngleDeg']], observations=observed,
                developmentOnly=True, controlEligible=False)


def fit(rows):
    require(isinstance(rows, list) and bool(rows), 'scene_rows_required')
    first = rows[0]; seen = set(); episodes = set(); frames = set(); models = []; deficits = []
    for row in rows:
        require(row['sourceSha256'] == digest(row['source']) and row['sourceSha256'] not in seen,
                'scene_source_changed_or_duplicate')
        seen.add(row['sourceSha256'])
        episode=tuple(row['source']['episode']); frame_ids={row['source'][side]['frameId'] for side in ('before','after')}
        require(episode not in episodes and len(frame_ids) == 2 and not frames.intersection(frame_ids),
                'scene_training_episode_or_frame_reused')
        episodes.add(episode); frames.update(frame_ids)
        require(row['source'].get('status') == 'completed' and row['source'].get('finalIdleOffVerified') is True
                and row['source'].get('trainingEligible') is True, 'scene_source_not_training_eligible')
        require(row['physicalContext'] == first['physicalContext']
                and row['observationContract'] == first['observationContract'], 'scene_contract_mixed')
        require(row['developmentOnly'] is True and row['controlEligible'] is False, 'scene_development_only')
        require(row['command'] == row['source']['command'] and row['physicalContext'] == row['source']['physicalContext'],
                'scene_source_binding_changed')
        require(isinstance(row.get('observations'),list) and len(row['observations']) == 2,
                'scene_two_bound_observations_required')
        for side, obs in zip(('before', 'after'), row['observations']):
            vector(row[side])
            require(row[side] == [*obs['state'], row['source'][side]['m4AngleDeg']] and obs['imageSha256'] == row['source'][side]['imageSha256']
                    and obs['frameId'] == row['source'][side]['frameId']
                    and obs['canonicalFrameSha256'] == first['observationContract']['canonicalFrameSha256']
                    and obs['sourceContext'] == row['source'][side]['sourceContext'],
                    'scene_observation_binding_changed')
    for key in sorted({command_key(r['command']) for r in rows}):
        group = [r for r in rows if command_key(r['command']) == key]
        if len(group) < 2:
            deficits.append(dict(command=group[0]['command'], availableTransitions=len(group), requiredTransitions=2)); continue
        x = np.array([r['before'] for r in group]); y = np.array([r['after'] for r in group])-x
        mean=x.mean(axis=0); scale=np.maximum(x.std(axis=0),1.); z=(x-mean)/scale; delta=y.mean(axis=0)
        coef=np.linalg.solve(z.T@z+np.eye(4),z.T@(y-delta))
        models.append(dict(command=copy.deepcopy(group[0]['command']),count=len(group),mean=mean.tolist(),
            scale=scale.tolist(),deltaMean=delta.tolist(),coefficient=coef.tolist(),
            minimum=x.min(axis=0).tolist(),maximum=x.max(axis=0).tolist()))
    return dict(schemaVersion=(SCHEMA if first['observationContract']['stateFields']==STATE_FIELDS else OBJECT_SCHEMA), physicalContext=copy.deepcopy(first['physicalContext']),
        observationContract=copy.deepcopy(first['observationContract']), models=models, deficits=deficits,
        trainingSourceSha256=sorted(seen), controlEligible=False, developmentOnly=True, independentTest=False)


def freeze_model(path, model):
    """새 경로에만 모델을 고정한다. 기존 모델 파일은 교체하지 않는다."""
    SceneRelativeModelBank({'approach': model})
    raw = canonical_bytes(model)
    with Path(path).open('xb') as output:
        output.write(raw)
    return dict(path=str(Path(path).resolve()), sha256=hashlib.sha256(raw).hexdigest())


class SceneRelativeModelBank:
    """명시한 단계의 학습 후보와 STOP을 비교한다. 다른 물리 조건은 선택 전에 거절한다."""
    def __init__(self, models):
        require(isinstance(models,dict) and bool(models) and set(models).issubset({'approach','close','release'}), 'scene_models_required')
        for model in models.values():
            require(model.get('schemaVersion') in (SCHEMA,OBJECT_SCHEMA) and model.get('controlEligible') is False
                    and model.get('developmentOnly') is True, 'scene_model_schema')
            require(model['observationContract'].get('stateFields') == (STATE_FIELDS if model['schemaVersion']==SCHEMA else OBJECT_STATE_FIELDS), 'scene_model_state_fields')
            for entry in model['models']:
                command_key(entry['command'])
                require(type(entry['count']) is int and entry['count'] >= 2, 'scene_model_support_count')
                for name in ('mean', 'scale', 'deltaMean', 'minimum', 'maximum'):
                    value=np.asarray(entry[name], dtype=float)
                    require(value.shape == (4,) and np.isfinite(value).all(), 'scene_model_parameters')
                coefficient=np.asarray(entry['coefficient'], dtype=float)
                require(coefficient.shape == (4,4) and np.isfinite(coefficient).all()
                        and np.all(np.asarray(entry['scale']) >= 1.)
                        and np.all(np.asarray(entry['minimum']) <= np.asarray(entry['maximum'])),
                        'scene_model_parameters')
        self.models=copy.deepcopy(models); self.hashes={k:digest(v) for k,v in models.items()}
        self.m1_observation_policies={}

    @classmethod
    def from_bindings(cls, bindings):
        from learning.artifacts import _decode_json
        models={}
        for kind, binding in bindings.items():
            raw=Path(binding['path']).read_bytes()
            require(hashlib.sha256(raw).hexdigest() == binding['sha256'], 'scene_model_file_changed')
            models[kind]=_decode_json(raw)
        bank=cls(models); bank.bindings=copy.deepcopy(bindings)
        return bank

    def scope_stages(self, stages):
        require(isinstance(stages,list) and [s['kind'] for s in stages] == list(self.models),
                'scene_stage_bindings_changed')
        result=copy.deepcopy(stages)
        for stage in result:
            model=self.models[stage['kind']]
            require(isinstance(stage.get('context'),dict) and stage['context'].get('object')==model['physicalContext']['object']
                    and bool(stage['context'].get('poseRegime')), 'scene_current_source_context_required')
            stage['context']['sceneObservationContract']=copy.deepcopy(model['observationContract'])
            stage['modelSha256']=self.hashes[stage['kind']]
        return result

    def is_scene_stage(self,kind):return kind in self.models

    def model_for_kind(self,kind):return self.models[kind]

    def verify_cached(self):
        require(self.hashes == {k:digest(v) for k,v in self.models.items()} and self.m1_observation_policies=={}, 'scene_model_changed')

    def choose(self, stage, record):
        self.verify_cached(); kind=stage['kind']; require(kind in self.models,'scene_stage_model_unavailable')
        model=self.models[kind]
        require(stage['modelSha256']==self.hashes[kind], 'scene_model_binding_changed')
        require(record['physicalContext']==model['physicalContext'], 'scene_physical_context_unsupported')
        require(record['observationContract']==model['observationContract'], 'scene_observation_contract_changed')
        x=vector(record['state']); goal=vector(stage['target'])
        ranked=[dict(kind='stop',command=None,predictedState=x.tolist(),errorPx=float(np.linalg.norm(x[:3]-goal[:3])))]; rejected=[]
        commands=stage['commands']; keys=[command_key(c) for c in commands]
        require(bool(keys) and len(keys)==len(set(keys)), 'scene_distinct_candidates_required')
        for command,key in zip(commands,keys):
            entries=[m for m in model['models'] if command_key(m['command'])==key]
            reason=None
            if len(entries)!=1: reason='command_without_two_training_transitions'
            else:
                m=entries[0]; lo=np.array(m['minimum']); hi=np.array(m['maximum'])
                if not (np.all(x>=lo) and np.all(x<=hi)): reason='state_outside_training_range'
            if reason:
                rejected.append(dict(command=copy.deepcopy(command),reason=reason,
                    supportRange=None if not entries else dict(minimum=entries[0]['minimum'],maximum=entries[0]['maximum']))); continue
            prediction=x+np.array(m['deltaMean'])+((x-np.array(m['mean']))/np.array(m['scale']))@np.array(m['coefficient'])
            if not np.isfinite(prediction).all() or prediction[2]<0 or not 0<=prediction[3]<360:
                rejected.append(dict(command=copy.deepcopy(command),reason='invalid_predicted_state')); continue
            ranked.append(dict(kind='action',command=copy.deepcopy(command),predictedState=prediction.tolist(),
                commandMeanPredictedState=(x+np.array(m['deltaMean'])).tolist(),
                errorPx=float(np.linalg.norm(prediction[:3]-goal[:3]))))
        ranked.sort(key=lambda r:r['errorPx'])
        return dict(selected=copy.deepcopy(ranked[0]),candidates=ranked,rejected=rejected,
            objectiveUnit=('canonical_pixels' if model['schemaVersion']==SCHEMA else 'object_widths'),
            predictor='stateRidge', status='ranked' if len(ranked)>1 else 'abstained',modelSha256=self.hashes[kind],
            physicalContext=copy.deepcopy(model['physicalContext']),observationContract=copy.deepcopy(model['observationContract']),
            controlEligible=False,developmentOnly=True,counterfactualOutcomesObserved=False,graspVerified=False)
