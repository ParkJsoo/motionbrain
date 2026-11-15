"""관측된 동역학과 정지 후 이동을 분리하는 단발 개발 목표 선택.

작은 자료의 외삽 가설이며 정지 거리 보장·집기 성공 모델이 아니다. 사람은 후보
범위와 영상 목표를 정하고, 모델은 STOP을 포함한 예측 오차로 후보를 선택한다.
"""
import copy
import math
import numpy as np
from learning.manual_collection_dataset import require
from learning.pipeline_learning import digest
from learning.m4_device_guard_policy import finite_goal_device_guard_policy,guard_request,SCHEMA as ACTOR_SCHEMA

SCHEMA='motionbrain.m4_development_outcome_model.v1'
DECISION_SCHEMA='motionbrain.m4_development_goal_decision.v1'


def _number(x):return type(x) in (int,float) and math.isfinite(x)


def audit_failed_guard_sensor(root):
    """failed 원본을 유지한 센서 동역학 증거. 영상 정답·성공행으로 승격하지 않는다."""
    from pathlib import Path
    from learning.manual_collection_dataset import Evidence
    from learning.task_transition_data import _http
    from learning.supervised_collection import plan_digest
    from learning.supervised_transport import SequencePort
    from learning.m4_device_guard_policy import validate_device_guard_policy
    ev=Evidence(Path(root).resolve());scope=ev.json('scope.json');result=ev.json('result.json')
    require(scope.get('schemaVersion')==ACTOR_SCHEMA and result.get('schemaVersion')==ACTOR_SCHEMA
            and result.get('scopeSha256')==plan_digest(scope) and result.get('status')=='failed'
            and bool(result.get('errors')) and result.get('trainingEligible') is False
            and result.get('commandOutcome')=='receipt_recorded' and result.get('finalIdleOffVerified') is True
            and result.get('mayHaveRun') is True,'goal_failed_sensor_source_required')
    receipt=result['receipt'];policy=validate_device_guard_policy(scope['deviceGuardPolicy'])
    require(receipt.get('plannedDeviceGoalGuard')==guard_request(scope)
            and receipt.get('plannedDeviceGuardPolicy')==policy,'goal_sensor_policy_changed')
    statuses,boot,_,_=_http(ev,receipt,scope['pulse']);terminal=receipt['deviceGoalGuard']
    require(boot==scope['controllerBootId'] and receipt.get('offVerified') is True
            and terminal.get('offVerified') is True and terminal.get('reason')=='goal','goal_sensor_terminal_required')
    after=[e for e in statuses if e['responseReceivedAt']>=receipt['offAt'] and e['response'].get('state')=='IDLE']
    require(bool(after),'goal_sensor_settled_raw_required')
    post_angle=SequencePort._shoulder_angle(after[-1]['response'])
    return dict(sourceSha256=ev.hashes['result.json'],sourceRoot=str(ev.root),originalStatus=result['status'],
        originalErrors=copy.deepcopy(result['errors']),commandOutcome=result['commandOutcome'],offVerified=True,
        terminationReason=terminal['reason'],direction=scope['pulse']['direction'],percent=scope['pulse']['percent'],
        beforeAngleDeg=terminal['startAngleDeg'],stopAngleDeg=terminal['lastAngleDeg'],postOffAngleDeg=post_angle,
        postOffAgeSeconds=after[-1]['responseReceivedAt']-receipt['offAt'],
        postWindowSatisfied=after[-1]['responseReceivedAt']>=receipt['offAt']+scope['postOffSeconds'],
        runMs=(terminal['offAtMs']-terminal['runAtMs'])&0xffffffff,executionPolicy=policy,originalTrainingEligible=False,
        visualAfterAvailable=(ev.root/'after-observation.json').exists(),taskSuccessLabel=False,
        sourceProofs=copy.deepcopy(ev.hashes))


def fit_outcome_model(visual_rows,guard_sensor_rows,*,audit_sources):
    evidence=dict(visualRows=copy.deepcopy(visual_rows),guardSensorRows=copy.deepcopy(guard_sensor_rows))
    require(callable(audit_sources) and audit_sources(copy.deepcopy(evidence))==evidence,'goal_model_raw_audit_required')
    require(bool(visual_rows) and bool(guard_sensor_rows),'goal_model_evidence_required')
    seen=set()
    for row in visual_rows+guard_sensor_rows:
        require(isinstance(row.get('sourceSha256'),str) and len(row['sourceSha256'])==64
                and row['sourceSha256'] not in seen and row.get('offVerified') is True,'goal_model_source_identity')
        seen.add(row['sourceSha256'])
    for row in visual_rows:
        require(row['originalStatus']=='completed' and row['direction']=='up' and row['percent']==50
                and all(_number(row[k]) for k in ('beforeAngleDeg','afterAngleDeg','objectRiseDeltaPx','beforeObjectWidthPx','durationMs'))
                and row['beforeObjectWidthPx']>0 and row['afterAngleDeg']>=row['beforeAngleDeg'], 'goal_visual_transition_invalid')
    # Angle delta is a training label, not an execution-time input. The guarded
    # prediction below supplies its own future-angle hypothesis.
    angles=np.array([r['afterAngleDeg']-r['beforeAngleDeg'] for r in visual_rows])
    rises=np.array([r['objectRiseDeltaPx']/r['beforeObjectWidthPx'] for r in visual_rows])
    require(float(angles@angles)>0,'goal_model_no_observed_response')
    slope=float((angles@rises)/(angles@angles))
    residual=float(np.sqrt(np.mean((rises-angles*slope)**2)))
    coasts=[];rates=[]
    for row in guard_sensor_rows:
        require(row['commandOutcome']=='receipt_recorded' and row['terminationReason']=='goal'
                and row['direction']=='up' and row['percent']==50
                and all(_number(row[k]) for k in ('beforeAngleDeg','stopAngleDeg','postOffAngleDeg','runMs','postOffAgeSeconds'))
                and 0<row['runMs']<=1200 and row['postOffAgeSeconds']>=0 and row['stopAngleDeg']>row['beforeAngleDeg'], 'goal_guard_sensor_evidence_invalid')
        coasts.append(row['postOffAngleDeg']-row['stopAngleDeg'])
        rates.append((row['stopAngleDeg']-row['beforeAngleDeg'])/row['runMs'])
    return dict(schemaVersion=SCHEMA,normalization='external_object_width_px',sourceEvidence=evidence,visualRisePerAngleDeg=slope,
        visualTrainingResidualFraction=residual,
        historicalDurationResponse=[dict(durationMs=r['durationMs'],angleDeltaDeg=r['afterAngleDeg']-r['beforeAngleDeg'],
            normalizedObjectRise=r['objectRiseDeltaPx']/r['beforeObjectWidthPx']) for r in visual_rows],
        commandMeanBaselineRiseFraction=float(np.mean(rises)),coastMeanDeg=float(np.mean(coasts)),
        progressPerMs=float(np.mean(rates)),trainingAngleRange=[min(r['beforeAngleDeg'] for r in visual_rows),max(r['beforeAngleDeg'] for r in visual_rows)],
        assumptions=['visual_projection_and_load_transfer_unverified','goal_time_linear_scaling_unverified',
                     'single_stop_coast_not_safety_bound','firmware_v2_to_v3_stop_mechanism_equivalence_requires_deployment_review'],
        outcomeHorizonSeconds=[r['postOffAgeSeconds'] for r in guard_sensor_rows],
        laterCoastUnobserved=any(not r.get('postWindowSatisfied',False) for r in guard_sensor_rows),
        controlEligible=False,developmentOnly=True,independentTest=False)


def choose_goal(model,current,goal_candidates,target_object_rise_fraction):
    require(model.get('schemaVersion')==SCHEMA and model.get('controlEligible') is False
            and model.get('developmentOnly') is True,'goal_development_model_required')
    require(isinstance(current,dict) and _number(current.get('m4AngleDeg'))
            and isinstance(current.get('observationSha256'),str) and len(current['observationSha256'])==64
            and current.get('bilateralContact') is True
            and ((current.get('tableSupported') is True and current.get('airborne',False) is False)
                 or (current.get('tableSupported') is False and current.get('airborne') is True)),
            'goal_current_observation_required')
    require(_number(target_object_rise_fraction) and target_object_rise_fraction>0,'goal_visual_target_required')
    require(isinstance(goal_candidates,list) and bool(goal_candidates)
            and len(goal_candidates)==len(set(goal_candidates)),'goal_distinct_candidates_required')
    start=current['m4AngleDeg'];observed_rise=current.get('objectRiseFraction',0.)
    require(_number(observed_rise),'goal_current_rise_required')
    candidates=[dict(kind='stop',goalAngleDeg=None,predictedObjectRiseFraction=observed_rise,
        predictedPostOffAngleDeg=start,error=abs(target_object_rise_fraction-observed_rise),estimatedRunMs=0.)];rejected=[]
    for goal in goal_candidates:
        try:
            policy=finite_goal_device_guard_policy(goal)
            # Enforce the actual firmware request domain, not a learned safety interval.
            guard_request(dict(schemaVersion=ACTOR_SCHEMA,controllerBootId='a'*32,trialId='offline-goal',
                expectedStartAngleDeg=start,goalAngleDeg=goal,deviceGuardPolicy=policy,
                pulse=dict(joint='shoulder',motor=4,direction='up',percent=50,durationMs=1200,expectedPwm=127)))
            estimated=(goal-start)/model['progressPerMs']
            require(estimated<=1200.,'predicted_goal_time_exceeds_cap')
            settled=goal+model['coastMeanDeg'];rise=observed_rise+(settled-start)*model['visualRisePerAngleDeg']
            candidates.append(dict(kind='goal',goalAngleDeg=goal,predictedPostOffAngleDeg=settled,
                predictedObjectRiseFraction=rise,commandMeanBaselineRiseFraction=model['commandMeanBaselineRiseFraction'],error=abs(rise-target_object_rise_fraction),estimatedRunMs=estimated))
        except (ValueError,KeyError) as exc:
            rejected.append(dict(goalAngleDeg=goal,reason=str(exc)))
    candidates.sort(key=lambda c:c['error'])
    return dict(schemaVersion=DECISION_SCHEMA,modelSha256=digest(model),inputSha256=digest(current),
        targetObjectRiseFraction=target_object_rise_fraction,selected=copy.deepcopy(candidates[0]),candidates=candidates,rejected=rejected,
        uncertainty=dict(calibrated=False,interval=None,visualTrainingResidualFraction=model['visualTrainingResidualFraction'],
            trainingAngleRange=model['trainingAngleRange'],assumptions=model['assumptions'],
            sourceGuardOutcomes=len(model['sourceEvidence']['guardSensorRows']),
            outcomeHorizonSeconds=model['outcomeHorizonSeconds'],laterCoastUnobserved=model['laterCoastUnobserved']),
        visionRole='current_support_contact_and_scale_evidence_not_fitted_current_state_gain',
        controlEligible=False,developmentOnly=True,graspVerified=False,liftVerified=False)


class BoundedGoalTrial:
    """명시한 한 시도에서만 새 목표를 actor에 고정하고 ARM 후 모델 선택을 다시 확인한다."""
    def __init__(self,model,current,goal_candidates,target_object_rise_fraction,*,store,actor_factory,observe_armed,validate_memory):
        self.model=copy.deepcopy(model);self.current=copy.deepcopy(current);self.goals=copy.deepcopy(goal_candidates)
        self.target=target_object_rise_fraction;self.store=store;self.actor_factory=actor_factory
        self.observe_armed=observe_armed;self.validate_memory=validate_memory
        self.decision=choose_goal(self.model,self.current,self.goals,self.target)
        self.binding=digest(dict(model=self.model,current=self.current,goals=self.goals,target=self.target,decision=self.decision))
        self.used=False;self.actor=None;self.armed=None

    def _memory(self):
        require(digest(dict(model=self.model,current=self.current,goals=self.goals,target=self.target,decision=self.decision))==self.binding,
                'goal_trial_memory_changed')

    def run(self,*,confirmed_decision_sha256,deadline=None,clock=None):
        require(not self.used and confirmed_decision_sha256==digest(self.decision),'goal_trial_unconsumed_confirmation_required')
        self.used=True;self._memory();self.store.write_json('goal-model-decision.json',self.decision)
        if self.decision['selected']['kind']=='stop':
            return dict(status='model_stop',modelSelectedCommands=0,modelAttempts=1,liftVerified=False)
        def prepare():
            self._memory();current=self.observe_armed(self.actor)
            fresh=choose_goal(self.model,current,self.goals,self.target)
            require(fresh['selected']['kind']=='goal' and fresh['selected']['goalAngleDeg']==self.decision['selected']['goalAngleDeg'],
                    'goal_ARM_selection_changed_no_run')
            self.store.write_json('goal-armed-decision.json',fresh);self.armed=digest(fresh)
        def before_run():
            self._memory();require(self.armed is not None,'goal_armed_decision_required')
            require(self.validate_memory(self.actor,self.armed) is True,'goal_RUN_raw_memory_rejected')
        hooks=dict(prepare=prepare,before_run=before_run,status_guard=lambda status:None,decisionSha256=digest(self.decision))
        actor,scope_sha=self.actor_factory(copy.deepcopy(self.decision),hooks);self.actor=actor
        from learning.m4_up_commissioning import M4UpCommissioning
        require(isinstance(actor,M4UpCommissioning) and actor.task_hooks is hooks
                and actor.scope['expectedStartAngleDeg']==self.current['m4AngleDeg']
                and actor.scope['goalAngleDeg']==self.decision['selected']['goalAngleDeg']
                and actor.scope['deviceGuardPolicy']==finite_goal_device_guard_policy(self.decision['selected']['goalAngleDeg'])
                and actor.digest==scope_sha,'goal_scoped_actor_binding_required')
        if deadline is not None:
            require(clock is not None and clock.monotonic()+actor.scope['maxSeconds']+10.<=deadline,
                    'goal_session_actor_time_reservation_required')
        result=actor.run(confirmed_scope_sha256=scope_sha)
        report=dict(status=result['status'],actorScopeSha256=scope_sha,decisionSha256=digest(self.decision),
            modelAttempts=1,modelSelectedCommands=int(result.get('receipt') is not None and result.get('mayHaveRun') is True),
            commandOutcome=result.get('commandOutcome'),finalIdleOffVerified=result.get('finalIdleOffVerified'),
            actorErrors=copy.deepcopy(result.get('errors',[])),
            actorResultUnchanged=True,developmentOnly=True,graspVerified=False,liftVerified=False)
        self.store.write_json('goal-model-trial-result.json',report)
        return report


class BoundedGoalSession:
    """유한 후보를 각각 한 번만 사용한다. failed/unknown에는 후속 actor를 만들지 않는다."""
    def __init__(self,model,current,goal_candidates,target_object_rise_fraction,*,store,trial_factory,observe_after,max_seconds):
        require(_number(max_seconds) and max_seconds>0,'goal_session_time_budget_required')
        self.model=copy.deepcopy(model);self.current=copy.deepcopy(current);self.goals=copy.deepcopy(goal_candidates)
        self.target=target_object_rise_fraction;self.store=store;self.trial_factory=trial_factory;self.observe_after=observe_after
        self.max_seconds=max_seconds;self.used=False
        self.plan=dict(schemaVersion='motionbrain.m4_development_goal_session.v1',modelSha256=digest(model),
            initialInputSha256=digest(current),goalCandidates=copy.deepcopy(self.goals),targetObjectRiseFraction=self.target,
            maxSeconds=max_seconds,eachGoalAtMostOnce=True,automaticRetry=False)
        self._plan_sha=digest(self.plan)

    def _seal(self):
        require(digest(self.plan)==self._plan_sha and digest(self.model)==self.plan['modelSha256']
                and self.goals==self.plan['goalCandidates'] and self.target==self.plan['targetObjectRiseFraction']
                and self.max_seconds==self.plan['maxSeconds'],'goal_session_memory_changed')

    def run(self,*,confirmed_session_sha256,clock):
        require(not self.used and confirmed_session_sha256==self._plan_sha,'goal_session_confirmation_required')
        self._seal();require(digest(self.current)==self.plan['initialInputSha256'],'goal_session_initial_input_changed')
        self.used=True;self.store.write_json('goal-session-plan.json',self.plan)
        started=clock.monotonic();deadline=started+self.max_seconds
        remaining=list(self.goals);trials=[];reason='candidate_budget_exhausted';failure=None;attempts=[];phase='selection'
        try:
            while remaining:
                self._seal()
                if clock.monotonic()>=deadline:reason='time_budget';break
                decision=choose_goal(self.model,self.current,remaining,self.target)
                if decision['selected']['kind']=='stop':reason='model_stop';break
                goal=decision['selected']['goalAngleDeg'];active=list(remaining);remaining.remove(goal)
                phase='trial_factory'
                attempt=dict(ordinal=len(attempts)+1,goalAngleDeg=goal,decisionSha256=digest(decision))
                attempts.append(attempt);self.store.write_json('goal-session-attempt-'+str(len(attempts))+'.json',attempt)
                trial=self.trial_factory(copy.deepcopy(self.current),active,len(trials)+1,deadline=deadline)
                require(isinstance(trial,BoundedGoalTrial) and trial.decision==decision
                        and trial.current==self.current and trial.model==self.model and trial.target==self.target,
                        'goal_session_trial_changed')
                phase='trial'
                result=trial.run(confirmed_decision_sha256=digest(trial.decision),deadline=deadline,clock=clock);trials.append(result)
                if result['status']!='completed' or result.get('commandOutcome')!='receipt_recorded' or result.get('finalIdleOffVerified') is not True:
                    reason='failed_attempt_no_retry';break
                phase='after_observation'
                current=self.observe_after(trial,deadline=deadline)
                require(isinstance(current,dict) and current.get('observationSha256')!=self.current['observationSha256'],
                        'goal_session_fresh_after_required')
                previous=self.current;self.current=copy.deepcopy(current)
                if (current['m4AngleDeg']<=previous['m4AngleDeg']
                        and current.get('objectRiseFraction',0.)<=previous.get('objectRiseFraction',0.)):
                    reason='no_observed_progress';break
                if current.get('objectRiseFraction',float('-inf'))>=self.target:reason='observed_visual_target_reached';break
        except (Exception,KeyboardInterrupt) as exc:
            reason='exception_no_retry';failure=dict(type=type(exc).__name__,phase=phase)
            safe_codes={'goal_session_memory_changed','goal_session_initial_input_changed','goal_session_trial_changed',
                'goal_session_fresh_after_required','goal_session_actor_time_reservation_required','goal_current_observation_required',
                'goal_trial_memory_changed','goal_scoped_actor_binding_required','goal_armed_decision_required',
                'goal_RUN_raw_memory_rejected','goal_ARM_selection_changed_no_run'}
            if isinstance(exc,ValueError) and str(exc) in safe_codes:failure['code']=str(exc)
        report=dict(schemaVersion='motionbrain.m4_development_goal_session_result.v1',reason=reason,trials=trials,failure=failure,attempts=attempts,
            unknownAttemptOutcome=bool(failure and phase=='trial'),
            modelSelectedCommands=sum(r['modelSelectedCommands'] for r in trials),remainingGoals=remaining,
            current=copy.deepcopy(self.current),fullyModelSelectedCycle=False,liftVerified=False,
            meaning='observed_image_plane_progress_not_retention_or_full_cycle')
        self.store.write_json('goal-session-result.json',report);return report
