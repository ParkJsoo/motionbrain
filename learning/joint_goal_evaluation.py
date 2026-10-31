"""Finite, fixed-model goal evaluation through injected observation/execution ports."""
import copy
import time

from learning.action_execution import finite
from learning.joint_action_selection import canonical_sha256
from learning.joint_goal_trial import pre_action_state, latest_settled, window, retain_queued_candidate

# Bands exceed the observed 550ms positioning steps; they are not exact poses.
POSES = {'low': (3.2, 7.), 'high': (7., 10.7)}
GOALS = {'low': 10., 'high': 4.}
POLICIES = ('model', 'fixed700', 'hold')


def schedule():
    blocks = [('model', 'model', 'fixed700', 'fixed700', 'hold', 'hold'),
              ('hold', 'hold', 'fixed700', 'fixed700', 'model', 'model')]
    return [dict(id=f'trial-{s*6+i+1:02d}', session=s, pose='high' if i%2 == 0 else 'low',
                 policy=policy, goalDeg=GOALS['high' if i%2 == 0 else 'low'])
            for s, policies in enumerate(blocks) for i, policy in enumerate(policies)]


def make_scope(selector, campaign_id):
    return dict(schemaVersion='motionbrain.fixed_model_goal_evaluation.v1', campaignId=campaign_id,
                modelSha256=selector.model_sha256, datasetSha256=selector.dataset_sha256,
                trials=schedule(), poses={k:list(v) for k,v in POSES.items()}, toleranceDeg=1.,
                captures=2, captureSeconds=150, maxFramesPerCapture=1220, sampleFps=8,
                maxTargetCommands=8, maxPositioningCommands=36, maxPositioningPerTrial=3,
                pwmMagnitude=160, targetDurationsMs=[700,900], positioningDurationsMs=[550,700,900],
                automaticRetries=0, trainingPerformed=False)


def validate_scope(scope, selector):
    if (not isinstance(scope, dict) or not isinstance(scope.get('campaignId'), str)
            or not scope['campaignId'] or scope != make_scope(selector, scope['campaignId'])):
        raise ValueError('exact_fixed_evaluation_scope_required')
    return copy.deepcopy(scope)


def in_pose(state, pose):
    return POSES[pose][0] <= state['angleDeg'] <= POSES[pose][1]


def positioning(state, pose):
    angle = state['angleDeg']
    if not finite(angle) or not -10 <= angle <= 24:
        raise ValueError('positioning_observation_outside_envelope')
    if in_pose(state, pose):
        return None
    delta = sum(POSES[pose])/2-angle
    return dict(direction='up' if delta > 0 else 'down', durationMs=550 if abs(delta)<=3.5 else 700 if abs(delta)<=6 else 900,
                pwmMagnitude=160)


def decide(selector, state, trial):
    if not in_pose(state, trial['pose']):
        raise ValueError('evaluation_start_pose_changed')
    decision = selector.select(state, trial['goalDeg'], tolerance_deg=1.)
    decision['policy'] = trial['policy']
    if trial['policy'] == 'model':
        return decision
    if trial['policy'] == 'hold':
        decision.update(status='hold', reason='predeclared_measured_no_output_baseline',
                        selected=dict(kind='hold', predictedEndDeg=state['angleDeg']))
        return decision
    direction = 'up' if trial['pose']=='low' else 'down'
    eligible = [c for c in decision['candidates'] if c['direction']==direction and c['durationMs']==700 and c['eligible']]
    if not eligible:
        decision.update(status='abstain', reason='fixed_command_unsupported', selected=None)
    else:
        decision.update(status='selected', reason='predeclared_fixed700_baseline', selected=copy.deepcopy(eligible[0]))
    return decision


def summarize(rows):
    result = {}
    for policy in POLICIES:
        subset = [r for r in rows if r['trial']['policy']==policy]
        measured = [r for r in subset if r.get('measurement') is not None]
        result[policy] = dict(planned=len(subset), measured=len(measured),
            executed=sum(r.get('executed') is True for r in subset),
            executionUnknown=sum(r.get('executionStatus')=='outcome_unknown' for r in subset),
            positioningUnknown=sum(a['phase']=='positioning' and a['executionStatus']=='outcome_unknown' for r in subset for a in r.get('commandAttempts',[])),
            holds=sum(r.get('decision',{}).get('status')=='hold' for r in subset),
            abstentions=sum(r.get('decision',{}).get('status')=='abstain' for r in subset),
            failures=sum(r['status']=='failed' for r in subset),
            notStarted=sum(r['status']=='not_started' for r in subset),
            goalsReached=sum(r['measurement']['goalReached'] for r in measured),
            measuredGoalErrorsDeg=[r['measurement']['afterGoalErrorDeg'] for r in measured],
            measuredErrorReductionsDeg=[r['measurement']['errorReductionDeg'] for r in measured])
        result[policy]['byPose'] = {}
        for pose in POSES:
            group=[r for r in measured if r['trial']['pose']==pose]
            result[policy]['byPose'][pose]=dict(planned=sum(r['trial']['pose']==pose for r in subset),
                measured=len(group),goalsReached=sum(r['measurement']['goalReached'] for r in group),
                meanGoalErrorDeg=None if not group else sum(r['measurement']['afterGoalErrorDeg'] for r in group)/len(group),
                meanErrorReductionDeg=None if not group else sum(r['measurement']['errorReductionDeg'] for r in group)/len(group),
                startingAnglesDeg=[r['measurement']['beforeDeg'] for r in group])

    return result


class GoalEvaluation:
    """One campaign. Each factory-created port owns at most one command.

    port_factory(label) must keep the campaign boot identity and shared HTTP
    deadline/retry/output-history state. Cameras retain fixed point identities
    between captures using the preceding verified frame, without re-detection.
    Any fault closes the campaign; no target is retried or silently replaced.
    """
    def __init__(self, scope, selector, camera_factory, port_factory, store, *, clock=time):
        self.scope=validate_scope(scope,selector);self.selector=selector
        self.camera_factory=camera_factory;self.port_factory=port_factory;self.store=store;self.clock=clock
        self.used=False;self.camera=None;self.active_port=None;self.serial=0
        self.counts={'target':0,'positioning':0};self.last_off=None;self.attempts=[]

    def observe(self, deadline):
        while True:
            at=self.clock.monotonic()
            if at>deadline:raise ValueError('settled_observation_deadline')
            try:
                self.camera.check();records=self.camera.samples();at=self.clock.monotonic()
                state=pre_action_state(records,at);latest_settled(records,at)
                if self.last_off is not None and state['windowStart']<self.last_off:
                    raise ValueError('insufficient_observation_window')
                return at,state
            except ValueError as exc:
                if str(exc) not in ('camera_not_live','insufficient_observation_window','invalid_or_unsettled_observation_window',
                                     'insufficient_velocity_history','latest_observation_not_settled'):
                    raise
                self.clock.sleep(.03)

    def fresh(self):
        self.camera.check();records=self.camera.samples();at=self.clock.monotonic()
        state=pre_action_state(records,at);latest_settled(records,at)
        if self.last_off is not None and state['windowStart']<self.last_off:
            raise ValueError('observation_precedes_off')
        if at-self.camera.first_source_at>140:raise ValueError('insufficient_capture_tail')
        return at,state

    def recheck_after_storage(self, at):
        self.camera.check();latest_settled(self.camera.samples(),self.clock.monotonic())
        if not 0 <= self.clock.monotonic()-at <= .25:raise ValueError('dispatch_decision_expired')

    def open_port(self, label):
        self.active_port=self.port_factory(label)
        self.active_port.preflight()
        return self.active_port

    def close_port(self):
        port=self.active_port
        if port is not None:
            # Clear even on cleanup failure: the campaign stops, never retries writes.
            self.active_port=None
            if port.finalize() is not True:raise ValueError('final_idle_off_unverified')

    def pulse(self, pulse, phase, trial, guard):
        limit=self.scope['maxTargetCommands' if phase=='target' else 'maxPositioningCommands']
        if self.counts[phase]>=limit:raise ValueError('campaign_command_budget')
        self.serial+=1;label=f"{trial['id']}-{phase}-{self.serial:02d}"
        self.store.write_json(label+'-intent.json',dict(trial=trial,pulse=pulse,phase=phase))
        self.counts[phase]+=1  # consumes an attempt even when dispatch is ambiguous
        attempt=dict(label=label,trialId=trial['id'],phase=phase,pulse=copy.deepcopy(pulse),
                     dispatchGuardPassed=False,executionStatus='not_dispatched',offVerified=False)
        self.attempts.append(attempt)
        port=self.open_port(label);calls=0;dispatch=None
        def once():
            nonlocal calls,dispatch
            calls+=1
            if calls!=1:raise ValueError('dispatch_guard_single_use')
            try:dispatch=guard()
            except (Exception,KeyboardInterrupt):
                attempt['executionStatus']='not_dispatched';raise
            attempt.update(dispatchGuardPassed=True,executionStatus='outcome_unknown');return dispatch
        try:
            attempt['executionStatus']='outcome_unknown'
            receipt=port.execute(pulse,once,self.camera.check)
            if (calls!=1 or not isinstance(receipt,dict) or receipt.get('offVerified') is not True
                    or receipt.get('dispatchAt')!=dispatch or not finite(receipt.get('offAt'))
                    or not dispatch<=receipt['offAt']<dispatch+3.75):
                raise ValueError('fixed_horizon_off_receipt_required')
            attempt.update(executionStatus='executed',offVerified=True)
            self.last_off=receipt['offAt'];return receipt
        finally:
            self.close_port();attempt['offVerified']=True

    def measure(self, at, state, decision, trial):
        while self.clock.monotonic()<at+4.75:
            self.camera.check();remaining=at+4.75-self.clock.monotonic()
            if remaining>0:self.clock.sleep(min(.03,remaining))
        self.camera.check()
        observations=window(self.camera.samples(),at+3.75,at+4.25,self.clock.monotonic(),3)
        import statistics
        after=statistics.median(r['angleDeg'] for r in observations)
        before_error=abs(trial['goalDeg']-state['angleDeg']);after_error=abs(trial['goalDeg']-after)
        predicted=(decision.get('selected') or {}).get('predictedEndDeg')
        return dict(beforeDeg=state['angleDeg'],afterDeg=after,beforeGoalErrorDeg=before_error,
                    afterGoalErrorDeg=after_error,errorReductionDeg=before_error-after_error,
                    goalReached=after_error<=1.,predictedEndDeg=predicted,
                    predictionErrorDeg=None if predicted is None else after-predicted,
                    windowStart=at+3.75,windowEnd=at+4.25,frames=len(observations))

    def trial(self, row):
        trial=row['trial'];row['status']='failed';row['positioning']=[]
        if self.clock.monotonic()-self.camera.first_source_at>115:
            raise ValueError('insufficient_capture_time_for_trial')
        for attempt in range(4):
            _,state=self.observe(self.clock.monotonic()+8)
            pulse=positioning(state,trial['pose'])
            if pulse is None:break
            if attempt==3:raise ValueError('trial_positioning_budget')
            def guard():
                at,fresh=self.fresh();needed=positioning(fresh,trial['pose'])
                if needed is None or needed['direction']!=pulse['direction']:
                    raise ValueError('positioning_direction_changed')
                self.store.write_json(f"{trial['id']}-position-{attempt+1}-dispatch.json",dict(at=at,observation=fresh,pulse=pulse))
                self.recheck_after_storage(at);return at
            row['positioning'].append(self.pulse(pulse,'positioning',trial,guard))
        at,state=self.fresh();decision=decide(self.selector,state,trial)
        row['initialDecision']=copy.deepcopy(decision)
        self.store.write_json(trial['id']+'-initial-decision.json',dict(at=at,observation=state,decision=decision))
        if decision['status']=='selected':
            c=decision['selected'];pulse={k:c[k] for k in ('direction','durationMs','pwmMagnitude')}
            def guard():
                nonlocal at,state,decision
                at,state=self.fresh();current=decide(self.selector,state,trial)
                if trial['policy']=='model':current=retain_queued_candidate(current,(pulse['direction'],pulse['durationMs']))
                elif current['status']!='selected':raise ValueError('fixed_command_no_longer_supported')
                if any(current['selected'][k]!=pulse[k] for k in pulse):raise ValueError('evaluation_command_changed')
                decision=current
                self.store.write_json(trial['id']+'-dispatch.json',dict(at=at,observation=state,decision=decision))
                self.recheck_after_storage(at);return at
            row['execution']=self.pulse(pulse,'target',trial,guard);row['executed']=True
            self.open_port(trial['id']+'-outcome-monitor')
        else:
            self.open_port(trial['id']+'-no-output')
            at,state=self.fresh();decision=decide(self.selector,state,trial)
            # A newly actionable state cannot turn an already chosen hold into a pulse.
            if decision['status']=='selected':
                decision.update(status='abstain',reason='no_output_decision_changed',selected=None)
            self.store.write_json(trial['id']+'-no-output-anchor.json',dict(at=at,observation=state,decision=decision))
            self.recheck_after_storage(at)
        row['decision']=decision
        try:measurement=self.measure(at,state,decision,trial)
        finally:self.close_port()
        row['measurement']=measurement;row['status']='measured'
        self.store.write_json(trial['id']+'-result.json',row)

    def run(self, *, confirmed_scope_sha256):
        if self.used or confirmed_scope_sha256!=canonical_sha256(self.scope):raise ValueError('new_exact_evaluation_scope_required')
        self.used=True;self.store.write_json('campaign-intent.json',self.scope)
        rows=[dict(trial=t,status='not_started',executed=False,measurement=None) for t in self.scope['trials']]
        report=dict(status='failed',trials=rows,errors=[],counts=self.counts,trainingPerformed=False,
                    scopeClosed=False,finalIdleOffVerified=False,automaticRetry=False,commandAttempts=self.attempts)
        try:
            self.open_port('campaign-initial');self.close_port();report['finalIdleOffVerified']=True
            for session in range(2):
                self.camera=self.camera_factory(session)
                self.camera.start();self.observe(self.clock.monotonic()+35)
                for row in rows[session*6:session*6+6]:
                    report['finalIdleOffVerified']=False
                    self.trial(row);report['finalIdleOffVerified']=True
                self.camera.finish();self.camera=None
            report['status']='completed'
        except (Exception,KeyboardInterrupt) as exc:
            report['errors'].append(dict(type=type(exc).__name__,reason=str(exc)))
        finally:
            try:
                self.close_port()
                self.open_port('campaign-final');self.close_port();report['finalIdleOffVerified']=True
            except (Exception,KeyboardInterrupt) as exc:
                report['finalIdleOffVerified']=False;report['errors'].append(dict(type='cleanup',reason=type(exc).__name__))
                report['status']='failed'
            if self.camera is not None:
                try:self.camera.close()
                except (Exception,KeyboardInterrupt) as exc:
                    report['errors'].append(dict(type='camera_close',reason=type(exc).__name__));report['status']='failed'
        for row in rows:
            attempts=[a for a in self.attempts if a['trialId']==row['trial']['id']]
            row['commandAttempts']=copy.deepcopy(attempts)
            targets=[a for a in attempts if a['phase']=='target']
            row['executionStatus']=targets[-1]['executionStatus'] if targets else 'not_dispatched'
            row['executed']=True if row['executionStatus']=='executed' else None if row['executionStatus']=='outcome_unknown' else False
        report['scopeClosed']=True;report['summary']=summarize(rows)
        self.store.write_json('campaign-result.json',report);return report
