"""최소 수집→원본 감사→학습/고정→유한 과업을 잇는 명시적 호출 경로.

기본 장치 포트는 없다. collect는 기존 승인된 단발 수집기를, run_cycle은 기존
scoped 실물 어댑터 또는 오프라인 재생기를 주입한다. 실패 명령은 재호출하지 않는다.
"""
from __future__ import annotations

import copy
import hashlib
from pathlib import Path

from learning import m1_shaft_learning as shaft
from learning import relative_action_learning as relative
from learning import task_action_models as actions
from learning.finite_task_cycle import STAGES, FiniteTaskCycle, bounded_command
from learning.manual_collection_dataset import require
from learning.pipeline_learning import digest
from learning.relative_model_bundle import validate_model as validate_relative


class ScopedCycleExecutor:
    """기존 scoped M4 actor와 일반 관절 포트를 상위 실행기에 결합한다.

    m4_factory(stage,command,hooks)는 (M4UpCommissioning, 승인된 scope SHA)를
    반환한다. 생성한 actor의 digest를 승인으로 자동 대체하지 않는다.
    다른 관절 execute_other는 기존 ManualCollection/SequencePort의
    after_armed_before_run=prepare, before_run=before_run을 결합해야 한다.
    factory/execute_other에 기본 구현이나 장치 주소가 없어 자동 구동되지 않는다.
    """
    def __init__(self, m4_factory, execute_other):
        require(callable(m4_factory) and callable(execute_other),'cycle_scoped_backend_required')
        self.m4_factory,self.execute_other=m4_factory,execute_other

    def __call__(self, stage, command, *, prepare, before_run, decision, safety):
        bounded_command(stage['kind'],command)
        if stage['kind'] not in ('lift','lower'):
            return self.execute_other(copy.deepcopy(command),after_armed_before_run=prepare,
                                      before_run=before_run,decision=copy.deepcopy(decision))
        from learning.m4_up_commissioning import M4UpCommissioning, CONTACT_MANUAL650_SCHEMA, SCHEMA, RETURN_SCHEMA
        from learning.supervised_transport import SequencePort
        require(safety['maximumM4ExcursionDeg']==3.,'cycle_M4_excursion_policy')
        def monitor(status):
            require(abs(SequencePort._shoulder_angle(status)-safety['M4ReferenceDeg']) <= 3.,
                    'cycle_active_M4_total_excursion')
        hooks=dict(prepare=prepare,before_run=before_run,status_guard=monitor,decisionSha256=digest(decision))
        actor,confirmed_scope_sha=self.m4_factory(copy.deepcopy(stage),copy.deepcopy(command),hooks)
        require(isinstance(actor,M4UpCommissioning) and actor.task_hooks is hooks
                and actor.scope['controllerBootId']==safety['controllerBootId']
                and {k:actor.scope['pulse'][k] for k in command}==command
                and actor.scope['schemaVersion'] in ((SCHEMA,CONTACT_MANUAL650_SCHEMA)
                    if stage['kind']=='lift' else (RETURN_SCHEMA,)), 'cycle_M4_scoped_actor_mismatch')
        require(confirmed_scope_sha==actor.digest,'cycle_M4_scope_not_confirmed')
        result=actor.run(confirmed_scope_sha256=confirmed_scope_sha)
        require(result['status']=='completed' and result['finalIdleOffVerified'] is True
                and result.get('errors')==[], 'cycle_M4_actor_failed_no_retry')
        return result['receipt']


class FrozenCycleModelBank:
    """파일 SHA와 메모리 모델 SHA를 구분한다. 동작 중 재학습하지 않는다."""
    def __init__(self, bindings, *, allow_development_approach=False):
        require(isinstance(bindings, dict) and set(bindings) == set(STAGES), 'cycle_model_bank_stages')
        require(type(allow_development_approach) is bool, 'cycle_development_flag_required')
        self.bindings = copy.deepcopy(bindings)
        self.models = {}
        for stage, binding in bindings.items():
            require(isinstance(binding, dict) and set(binding) == {'path','sha256'}, 'cycle_file_binding')
            raw = Path(binding['path']).read_bytes()
            require(hashlib.sha256(raw).hexdigest() == binding['sha256'], 'cycle_model_file_changed')
            from learning.artifacts import _decode_json
            model = _decode_json(raw)
            if stage == 'approach':
                validate_relative(model, allow_development_observations=allow_development_approach)
            elif stage in ('close','release'):
                shaft.validate_model(model)
            else:
                actions.validate_m4(model)
            self.models[stage] = copy.deepcopy(model)
        require(len({m['context']['object'] for m in self.models.values()}) == 1, 'cycle_object_changed')
        self.hashes = {k:digest(v) for k,v in self.models.items()}

    def verify_cached(self):
        require({k:digest(v) for k,v in self.models.items()} == self.hashes, 'cycle_cached_model_changed')

    def choose(self, stage, record):
        self.verify_cached()
        kind = stage['kind']; model = self.models[kind]
        require(stage['modelSha256'] == self.hashes[kind]
                and stage['context'] == record['context'] == model['context'], 'cycle_stage_model_mismatch')
        if kind == 'approach':
            result = relative.select(model, record['state'], stage['target'], stage['commands'], record['context'])
        elif kind == 'close':
            result = shaft.select(model, record['state']['shaftSeparationPx'], stage['target']['shaftSeparationPx'],
                                  stage['commands'], record['context'])
        elif kind == 'release':
            result = actions.select_open(model, record['state']['shaftSeparationPx'],
                stage['target']['shaftSeparationPx'], stage['commands'], record['context'],
                table_supported=record['entry'].get('tableSupported'), m4_angle_deg=record['m4AngleDeg'],
                expected_m4_angle_deg=stage['target']['expectedM4AngleDeg'])
        else:
            require(record['state']['startAngleDeg'] == record['m4AngleDeg'], 'cycle_M4_input_angle_changed')
            result = actions.select_m4(model, record['state'], stage['target'], stage['commands'],
                                       record['context'], record['supportMode'])
        return dict(result, modelSha256=self.hashes[kind])

    def scope_stages(self, stages):
        require(isinstance(stages, list) and [s['kind'] for s in stages] == list(STAGES), 'cycle_all_stages_required')
        result = copy.deepcopy(stages)
        for stage in result:
            model = self.models[stage['kind']]
            stage.update(context=copy.deepcopy(model['context']), modelSha256=self.hashes[stage['kind']])
        return result


def train_cycle_bundle(root, *, m4_rows, open_rows, approach_binding, close_binding, audit_rows,
                       allow_development_approach=False):
    """감사 통과 후 새 모델 두 개만 고정한다. 기존 모델·원본은 변경하지 않는다.

    audit_rows는 supplied rows를 원본으로 다시 산출·대조하고 True를 반환해야 한다.
    실패/unknown 원본, 임의 주석, 계약 자동 혼합을 승인하는 함수가 아니다.
    root는 새 독립 디렉터리여야 하며 부분 실패도 삭제·재사용하지 않는다.
    """
    require(callable(audit_rows) and audit_rows(copy.deepcopy(m4_rows),copy.deepcopy(open_rows)) is True,
            'cycle_training_source_audit_required')
    require(isinstance(m4_rows,list) and bool(m4_rows) and isinstance(open_rows,list) and bool(open_rows),
            'cycle_training_rows_required')
    require(all(r['command']['direction'] == 'open' for r in open_rows), 'cycle_OPEN_training_only')
    m4 = actions.fit_m4(m4_rows, m4_rows[0]['context'])
    opened = shaft.fit(open_rows)
    require(bool(opened['models']), 'cycle_OPEN_model_without_supported_candidate')
    for kind, direction, support in [('lift','up','table_supported'),('lower','down','gripper_supported')]:
        require(any(e['command']['direction'] == direction and e['supportMode'] == support for e in m4['models']),
                'cycle_M4_missing_'+kind)
    root = Path(root)
    from learning.artifacts import ArtifactStore
    store = ArtifactStore(root)
    bindings = dict(approach=copy.deepcopy(approach_binding),close=copy.deepcopy(close_binding))
    for stages, name, model in [(('lift','lower'),'m4-model.json',m4),(('release',),'open-model.json',opened)]:
        path = root/name
        model_sha = actions.freeze_model(path,model)
        for stage in stages:bindings[stage]=dict(path=str(path.resolve()),sha256=model_sha)
    bank = FrozenCycleModelBank(bindings,allow_development_approach=allow_development_approach)
    store.write_json('bundle.json',dict(bindings=bindings,modelHashes=bank.hashes,
        controlEligible=False,executionAuthorized=False,sourceAuditPerformed=True))
    return bank


class CollectionToCycle:
    """통합 실행 범위의 수동 수집과 모델 실물 시도를 별도 분모로 기록한다.

    collect(request)는 consumed 단발 actor의 완료 결과를 반환한다.
    audit(request,result)는 원본으로 산출한 row를 반환한다. freeze(rows)는
    train_cycle_bundle을 호출하는 결합 함수다. make_cycle(bank)는 새 scope와
    명시적 포트의 FiniteTaskCycle만 반환한다. 모델 고정 뒤 추가 수집·학습 없음.
    """
    def __init__(self, requests, *, seed_rows, collect, audit, freeze, make_cycle, store):
        require(isinstance(requests,list) and len(requests)<=12, 'cycle_minimum_collection_budget')
        for request in requests:
            require(isinstance(request,dict) and set(request)=={'id','stage','command'}
                    and request['stage'] in ('lift','lower','release'), 'cycle_collection_request')
            bounded_command(request['stage'],request['command'])
        require(len({r['id'] for r in requests})==len(requests), 'cycle_collection_duplicate_id')
        require(all(callable(c) for c in (collect,audit,freeze,make_cycle))
                and callable(getattr(store,'write_json',None)), 'cycle_workflow_callbacks_required')
        self.requests,self.seed_rows=copy.deepcopy(requests),copy.deepcopy(seed_rows)
        self.collect,self.audit,self.freeze,self.make_cycle,self.store=collect,audit,freeze,make_cycle,store
        self.used=False

    def run(self):
        require(not self.used,'cycle_workflow_single_use')
        self.used=True
        report=dict(status='failed',manualCollectionAttempts=0,manualCollectionCompleted=0,
                    modelCycleAttempts=0,modelCycleResult=None,errors=[])
        try:
            self.store.write_json('workflow-consumed.json',dict(requests=self.requests,seedRowsDigest=digest(self.seed_rows)))
            rows=copy.deepcopy(self.seed_rows)
            for i,request in enumerate(self.requests,1):
                self.store.write_json(f'collection-{i:03d}-intent.json',request)
                report['manualCollectionAttempts']+=1
                result=self.collect(copy.deepcopy(request))
                self.store.write_json(f'collection-{i:03d}-result.json',result)
                require(result.get('status')=='completed' and result.get('offVerified') is True,
                        'cycle_collection_failed_no_retry')
                row=self.audit(copy.deepcopy(request),copy.deepcopy(result))
                require(isinstance(row,dict) and row.get('command')==request['command'], 'cycle_collection_row_changed')
                rows.append(copy.deepcopy(row));report['manualCollectionCompleted']+=1
            bank=self.freeze(rows)
            require(isinstance(bank,FrozenCycleModelBank),'cycle_frozen_bank_required')
            cycle=self.make_cycle(bank)
            require(isinstance(cycle,FiniteTaskCycle),'cycle_finite_runner_required')
            report['modelCycleAttempts']=1
            report['modelCycleResult']=cycle.run()
            report['status']=report['modelCycleResult']['status']
        except (Exception,KeyboardInterrupt) as exc:
            report['errors'].append(dict(type=type(exc).__name__,reason=str(exc) if isinstance(exc,ValueError)
                                         else 'cycle_workflow_failed'))
        self.store.write_json('workflow-result.json',report)
        return report
