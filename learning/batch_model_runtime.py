"""배치 모델 선택을 기존 수집 실행기·접촉 검토·M4 경로에 연결한다.

생성은 파일 검사만 수행한다. run은 명시한 scope hash 확인 후에만 장치에
접근한다. stationary 관측 차단과 기존 출력/원장/접촉 검토는 그대로 적용된다.
"""
from __future__ import annotations

import copy
from contextlib import ExitStack
import json
import time
import uuid
from pathlib import Path

from learning.artifacts import ArtifactStore, _read
from learning.batch_observation import digest, require
from learning.batch_campaign import digest as plan_digest
from learning.batch_learning import model_sha256
from learning.batch_runtime import BatchRuntime, sha
from learning.batch_model_trial import BatchModelSequence, select


class ModelDecisionRuntime(BatchRuntime):
    """기존 단발 실행 직전 동일 모델의 후보 선택을 새 수집 영상으로 재확인한다."""
    decision_binding = None

    def __init__(self, plan, *args, **kwargs):
        require(not ({'collectionAdmission','fixedCommandEvaluation'} & set(plan.get('runtime',{}))),
                'model_runtime_manual_collection_mode_forbidden')
        super().__init__(plan, *args, **kwargs)

    def _before_review_config(self):
        config=self.config.get('beforeReviewer')
        if config is None:return None
        require(isinstance(config,dict) and set(config)=={'kind','protocolVersion','maxWaitSeconds'} and
                config['kind']=='file' and config['protocolVersion']==1 and
                type(config['maxWaitSeconds']) in (int,float) and 0<=config['maxWaitSeconds']<=3,
                'model_runtime_before_review_configuration')
        return config

    def _validate_interaction_binding(self,condition,binding,route):
        if route=='generic' and self._before_review_config() is not None:
            from learning.batch_interaction import validate_binding
            return validate_binding(self.plan,condition,binding,self.repository,purpose='declaration')
        return super()._validate_interaction_binding(condition,binding,route)

    def _before_binding(self,manager,pair,condition,evidence):
        config=self._before_review_config();binding=self._binding(condition)
        self._current_before_request=None
        if config is None or (binding.get('interaction')=='noncontact' and self.spec['object']=='none'):
            return binding
        require(isinstance(pair,dict) and pair.get('before'), 'model_runtime_review_pair_required')
        frame=evidence['source']['frame'];grip=pair['before']['frame']
        # Existing legacy contact verifier consumes byte-identical snapshots.
        # Its filename label is compatibility metadata, not a claim of a future image.
        snapshot=self._legacy_snapshot(manager.store.root,manager.camera.root,frame,
            manager.store.root/'gripper-camera/capture-0001',0)
        request=dict(schemaVersion='motionbrain.batch_before_review_request.v1',conditionId=condition['id'],
            command=condition['command'],sessionId=manager.current['sessionId'],
            context={k:self.spec[k] for k in ('viewId','poseRegime','object','load')},
            controllerBootId=self.plan['controllerBootId'],outputSequence=manager.port.last_sequence,
            sourcePath=str(manager.store.root),legacySnapshot=snapshot,
            frames=dict(external=[{k:frame[k] for k in ('frameId','pixelSha256')}],
                        gripper=[{k:grip[k] for k in ('frameId','pixelSha256')}]))
        manager.store.write_json('batch-before-review-request.json',request)
        response_path=manager.store.root/'batch-before-review-response.json'
        until=self.clock.monotonic()+config['maxWaitSeconds']
        while not response_path.exists():
            require(self.clock.monotonic()<until,'model_runtime_before_review_not_supplied')
            self._deadline();manager.tick()
            require(manager.current['phase']=='recording','model_runtime_before_review_recording_ended')
            self.clock.sleep(.02)
        raw=_read(manager.store.root,response_path.name)
        require(len(raw)<=1024*1024,'model_runtime_before_review_too_large')
        response=json.loads(raw)
        require(isinstance(response,dict) and set(response)=={'requestSha256','review'} and
                response['requestSha256']==digest(request),'model_runtime_before_review_request_changed')
        envelope=response['review']
        require(isinstance(envelope,dict) and envelope.get('stage')=='before' and
                envelope.get('conditionId')==condition['id'] and envelope.get('outputSequence')==request['outputSequence'],
                'model_runtime_before_review_binding_changed')
        manager.store.write_json('batch-before-review-envelope.json',envelope)
        path=manager.store.root/'batch-before-review-envelope.json'
        self._current_before_request=request
        return dict(binding,reviewPath=str(path.relative_to(self.repository)),reviewSha256=sha(_read(manager.store.root,path.name)))

    def _decision_observation(self, evidence, paired, model, *, m4=False):
        joint=model['joint'];external=evidence['observation']
        require(external.get('status')=='accepted', 'model_runtime_observation_rejected')
        if m4:
            fields=external['states'][joint]['stateFields'];state=external['states'][joint]['state']
            context={**{k:self.spec[k] for k in ('viewId','poseRegime','object','load')},'observerSha256':digest(self.spec)}
        else:
            fields,state=self._states(external,paired,joint);context=self._context()
        metadata=evidence['derivedMetadata']
        ids=[metadata['frameId']];start=metadata['capturedAt'];end=metadata['persistedAt']
        if paired is not None:
            binding=paired['frameBinding'];ids.append(binding['frameId'])
            start=min(start,binding['capturedAt']);end=max(end,binding['persistedAt'])
        return dict(status='accepted',context=context,stateFields=fields,state=state,frameIds=ids,
                    observedAt=start,persistedAt=end)

    def observe_model(self, model, *, m4=False):
        observed=self.observe()
        require(observed['offVerified'] and observed['journalClear'], 'model_runtime_not_idle')
        paired=None
        if not m4 and self.gripper_spec is not None and model['joint'] in self.gripper_spec['joints']:
            sid='model-observe-'+uuid.uuid4().hex
            store=ArtifactStore(self.root/'decision-observations'/sid)
            source=self._gripper_factory()(sid,store)
            try:
                source.start(deadline=self.clock.monotonic()+6)
                record=source.capture(deadline=self.clock.monotonic()+6);source.verify(record)
                binding=source._bindings[record['frame']['frameId']]
                paired=self._observe_gripper(store.root/binding['folder'],'frame',sid)
            finally:source.close()
        return self._decision_observation(observed['evidence'],paired,model,m4=m4)

    def _check_decision(self, evidence, paired, store, *, m4=False):
        require(self.decision_binding is not None,'model_runtime_decision_missing')
        stage,model,command=self.decision_binding
        obs=self._decision_observation(evidence,paired,model,m4=m4)
        now=self.clock.monotonic()
        require(0<=now-evidence['derivedMetadata']['capturedAt']<=self.plan['maxObservationAgeSeconds'],
                'model_runtime_decision_stale')
        if paired is not None:
            require(0<=now-paired['frameBinding']['capturedAt']<=self.gripper_spec['maxAgeSeconds'],
                    'model_runtime_gripper_decision_stale')
        result=select(model,obs,stage['target'],stage['tolerance'],stage['commands'])
        require(result['selected']['kind']=='action' and result['selected']['command']==command,
                'model_runtime_choice_changed_no_dispatch')
        store.write_json('batch-model-decision.json',dict(decision=result,observation=obs,preparedAt=now))

    def _before(self,condition):
        original=super()._before(condition)
        def check(manager,pair):
            original(manager,pair)
            request=getattr(self,'_current_before_request',None)
            if request is not None:
                token=self.interaction_token
                require(token.report.get('stage')=='before','model_runtime_before_review_required')
                require(digest(token.report)==token.reportSha256,'model_runtime_review_cache_changed')
                for camera in ('external','gripper'):
                    allowed={(f['frameId'],f['pixelSha256']) for f in request['frames'][camera]}
                    actual=token.report.get('frameBindings',{}).get(camera,[])
                    require(bool(actual) and all((f['frameId'],f['pixelSha256']) in allowed for f in actual),
                            'model_runtime_before_review_frame_not_requested')
            paired=None
            if self.gripper_spec is not None:
                paired=json.loads(_read(manager.store.root,'batch-gripper-before.json'))['observation']
            self._check_decision(self.latest_before,paired,manager.store)
        return check

    def _m4_route(self):
        route=super()._m4_route()
        route.before_dispatch=lambda evidence,store:self._check_decision(evidence,None,store,m4=True)
        return route


def _verified_interaction(stage, request, audited, observation):
    report=audited.get('interactionAfter',{})
    if report.get('status')!='reviewed':return False
    if stage['kind']=='close':return report.get('tableContactReady') is True
    return report.get('retainedLift') is True and report.get('liftVerified') is True


class BatchModelRunner:
    """고정 stage/condition에 한정된 단일 실행. 실패 후 새 요청으로 대체하지 않는다."""
    def __init__(self, trial_id, stages, runtimes, conditions, models, root, *, max_seconds=120):
        self.runtimes=dict(runtimes);self.conditions=copy.deepcopy(conditions)
        self.root=Path(root).resolve()
        require(type(max_seconds) in (int,float) and 0<max_seconds<=600,'model_runtime_deadline')
        self.max_seconds=min(max_seconds,*(r.plan['budgets']['maxSeconds'] for r in runtimes.values()))
        self.sequence=BatchModelSequence(trial_id,stages,models,
            max_age_seconds=min(r.plan['maxObservationAgeSeconds'] for r in runtimes.values()),review=_verified_interaction)
        for stage in stages:
            runtime=self.runtimes[stage['id']]
            require(isinstance(runtime,ModelDecisionRuntime),'model_runtime_adapter_required')
            require('collectionAdmission' not in runtime.config and 'fixedCommandEvaluation' not in runtime.config,
                    'model_runtime_manual_collection_mode_forbidden')
            if stage['kind'] in ('close','lift'):
                require('afterReviewer' in runtime.config, 'model_runtime_live_after_reviewer_required')
            require(self.root.is_relative_to((runtime.repository/'.local').resolve()),'model_runtime_private_root')
            candidates=self.conditions[stage['id']]
            require(isinstance(candidates,list) and bool(candidates),'model_runtime_conditions_required')
            require([c['command'] for c in candidates]==stage['commands'],'model_runtime_candidate_bindings')
            for condition in candidates:
                require(condition in runtime.plan['conditions'] and condition['phase']=='evaluation', 'model_runtime_pinned_evaluation_condition')
                require(runtime.validate_condition(condition) is None,'model_runtime_condition_rejected')
                m4=runtime._binding(condition).get('route')=='m4_up'
                require(stage['kind']!='lift' or
                        (m4 and runtime._binding(condition).get('interaction')=='grasped'),
                        'model_runtime_lift_route_required')
                if m4 and stage['kind']!='lift':
                    require(stage['kind']=='position' and runtime._binding(condition).get('interaction')=='noncontact',
                            'model_runtime_position_noncontact_route_required')
            require(len({runtime._binding(c).get('route','generic') for c in candidates})==1,
                    'model_runtime_mixed_stage_routes')
            for condition in candidates:
                m4=runtime._binding(condition).get('route')=='m4_up'
                model=runtime._m4_model() if m4 else runtime._model(condition['command']['joint'])
                require(model_sha256(model)==model_sha256(models[stage['modelId']]),'model_runtime_frozen_model_changed')
        self.scope=dict(trialId=trial_id,stages=copy.deepcopy(stages),conditions=self.conditions,
            models={key:model_sha256(value) for key,value in models.items()},
            plans={key:runtime.plan_sha for key,runtime in runtimes.items()},maxSeconds=self.max_seconds)
        self.scope_sha256=digest(self.scope)

    def _after(self,runtime,result,condition):
        require(result.get('execution')=='completed' and result.get('offVerified') is True and
                result.get('journalClear') is True and result.get('observation')=='eligible', 'model_runtime_result_unverified')
        identity=result['transitionId'];raw=_read(runtime.root,'transitions/'+identity+'.json')
        require(sha(raw)==identity,'model_runtime_transition_changed')
        transition=json.loads(raw);row=transition['learningRow']
        require(row['command']==condition['command'],'model_runtime_audited_command_changed')
        evidence=transition['after'];meta=evidence['derivedMetadata']
        ids=[meta['frameId']];start=meta['capturedAt'];end=meta['persistedAt']
        paired=transition.get('gripperPair',{}).get('afterObservation')
        if paired:
            binding=paired['frameBinding'];ids.append(binding['frameId'])
            start=min(start,binding['capturedAt']);end=max(end,binding['persistedAt'])
        if transition.get('sourceKind')=='m4_scope':
            folder=Path(result['evidence']['sourcePath'])
            recorded=json.loads(_read(folder,'result.json'));off=recorded['receipt']['offAt']
        else:
            audit=result['evidence']['audit'];off=audit['visual']['row']['evidence']['offReceiptAt']
        audit=dict(status='verified',offVerified=True,command=row['command'],sourceSha256=row['sourceHash'],
            commandAt=row['commandAt'],offAt=off,
            interactionAfter=result.get('interactionAfter',transition.get('interactionAfter',{})))
        observation=dict(status='accepted',context=row['context'],stateFields=row['stateFields'],state=row['after'],
            frameIds=ids,observedAt=start,persistedAt=end)
        return audit,observation

    def _verify_bindings(self):
        require(digest(self.scope)==self.scope_sha256 and self.sequence.stages==self.scope['stages'] and
                self.conditions==self.scope['conditions'] and
                {k:model_sha256(v) for k,v in self.sequence.models.items()}==self.scope['models'] and
                {k:plan_digest(v.plan) for k,v in self.runtimes.items()}==self.scope['plans'],
                'model_runtime_scope_changed')

    def run(self, *, confirmed_scope_sha256):
        require(confirmed_scope_sha256==self.scope_sha256,'model_runtime_explicit_scope_required')
        self._verify_bindings()
        self.root.mkdir(parents=True,exist_ok=True)
        # Exclusive durable ownership: a crashed/failed attempt never auto-runs again.
        with (self.root/'attempt.json').open('x') as out:
            json.dump(self.scope,out,ensure_ascii=False);out.flush()
            import os
            os.fsync(out.fileno())
        store=ArtifactStore(self.root/'evidence');store.write_json('scope.json',self.scope)
        clock=next(iter(self.runtimes.values())).clock
        end=clock.monotonic()+self.max_seconds;wall=clock.time()+self.max_seconds
        report=dict(status='running',decisions=[],results=[],modelSelected=True,physicalSuccessClaimed=False)
        counts={}
        locks=ExitStack()
        try:
            from learning.batch_campaign import Campaign
            from learning.batch_runtime import private_path
            paths={private_path(r.repository,r.config['dispatchLockPath']) for r in self.runtimes.values()}
            for path in sorted(paths):
                path.parent.mkdir(parents=True,exist_ok=True)
                locks.enter_context(Campaign._Lock(path))
            while self.sequence.index<len(self.sequence.stages):
                self._verify_bindings()
                require(clock.monotonic()<end,'model_runtime_deadline')
                stage=self.sequence.stages[self.sequence.index];runtime=self.runtimes[stage['id']]
                model=self.sequence.models[stage['modelId']]
                m4=runtime._binding(self.conditions[stage['id']][0]).get('route')=='m4_up'
                observation=runtime.observe_model(model,m4=m4)
                decision=self.sequence.propose(observation,now=clock.monotonic())
                report['decisions'].append(decision)
                store.write_json('decision-'+str(len(report['decisions']))+'.json',decision)
                if decision['status']=='observed_target_reached':continue
                if decision['status']!='execution_request':
                    report['status']=decision['status'];break
                request=decision['request']
                matches=[c for c in self.conditions[stage['id']] if c['command']==request['command']]
                require(len(matches)==1,'model_runtime_unique_selected_condition')
                condition=matches[0];key=(runtime.plan_sha,condition['id'])
                counts[key]=counts.get(key,0)+1
                require(counts[key]<=condition['maxAttempts'],'model_runtime_condition_budget')
                total=sum(n for (plan,_),n in counts.items() if plan==runtime.plan_sha)
                require(total<=min(runtime.plan['budgets']['maxPhysicalCommands'],runtime.plan['budgets']['maxAttempts']),
                        'model_runtime_plan_budget')
                self._verify_bindings()
                runtime.decision_binding=(copy.deepcopy(stage),copy.deepcopy(model),copy.deepcopy(request['command']))
                result=runtime.execute(condition,request['requestId'],deadline_wall=wall)
                report['results'].append(result)
                store.write_json('result-'+str(len(report['results']))+'.json',result)
                audit,recorded_after=self._after(runtime,result,condition)
                # The recorded target can be old after finalization/review. Reobserve
                # rather than treating a historical target or prediction as current.
                after=runtime.observe_model(model,m4=m4)
                store.write_json('after-observation-'+str(len(report['results']))+'.json',
                                 dict(recorded=recorded_after,current=after))
                self.sequence.complete(request['requestId'],audit,after,now=clock.monotonic())
            else:report['status']='completed'
        except BaseException as exc:
            report.update(status='halted' if isinstance(exc,Exception) else 'interrupted',
                          reason=type(exc).__name__+':'+str(exc),automaticRetry=False)
        finally:
            cleanup_errors=[];closed=set()
            for runtime in self.runtimes.values():
                if id(runtime) in closed:continue
                closed.add(id(runtime))
                try:runtime.close()
                except BaseException as exc:cleanup_errors.append(type(exc).__name__)
            locks.close()
            if cleanup_errors:
                report.update(status='cleanup_failed',cleanupErrors=cleanup_errors,automaticRetry=False)
            report['completedStages']=self.sequence.index
            store.write_json('report.json',report)
        return report
