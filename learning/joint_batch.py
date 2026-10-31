"""Finite M2 batch collection and grouped prediction; injected device ports only.

Current state becomes the next start. No positioning pulses or automatic retries.
The v1 32-row prediction contract is deliberately unchanged.
"""
import copy
import random
import statistics
import time
from collections import Counter

import numpy as np
from learning.action_execution import finite
from learning.joint_action_selection import canonical_sha256
from learning.joint_goal_evaluation import GoalEvaluation
from learning.joint_goal_trial import pre_action_state, latest_settled, window
from learning.joint_prediction import INPUT_NAMES, ALPHAS, fit, predict, scores

SPLITS = ('train', 'train', 'validation', 'train', 'train', 'test', 'train', 'validation', 'train', 'test')
COMMANDS = tuple((d, ms) for d in ('up', 'down') for ms in (550, 700, 900))
LABEL_FAILURES = ('insufficient_observation_window', 'invalid_or_unsettled_observation_window')


def make_scope(campaign_id):
    return dict(schemaVersion='motionbrain.m2_batch_collection.v1', campaignId=campaign_id,
                captures=10, captureSeconds=80, maxFramesPerCapture=660, sampleFps=8,
                maxBytesPerCapture=2110000000, attemptsPerCapture=12, maxTargetCommands=120,
                maxPositioningCommands=0, pwmMagnitude=160, durationsMs=[550,700,900],
                angleEnvelopeDeg=[-10,24], inwardThresholdsDeg=[5,10], splitByCapture=list(SPLITS),
                scheduleSeed=20261031, horizonSeconds=4., automaticRetries=0)


def choose(state, counts, serial):
    angle = state['angleDeg']
    if not finite(angle) or not -10 <= angle <= 24:
        raise ValueError('batch_observation_outside_envelope')
    allowed = [k for k in COMMANDS if (angle > 5 or k[0]=='up') and (angle < 10 or k[0]=='down')]
    random.Random(20261031+serial).shuffle(allowed)
    key = min(allowed, key=lambda k: counts.get(k, 0))
    return dict(direction=key[0], durationMs=key[1], pwmMagnitude=160)


def features(angle, velocity, direction, duration_ms):
    if (direction, duration_ms) not in COMMANDS or type(duration_ms) is not int:
        raise ValueError('unknown_batch_command')
    if not finite(angle) or not finite(velocity):
        raise ValueError('finite_batch_state_required')
    sign = 1 if direction == 'up' else -1
    return [angle, velocity, sign, duration_ms/1000, sign*duration_ms/1000, sign*angle]


class BatchCollection(GoalEvaluation):
    """Reuse tested STOP/OFF, boot, one-use pulse and fresh observation contracts."""
    def __init__(self, scope, camera_factory, port_factory, store, *, clock=time):
        if not isinstance(scope, dict) or not isinstance(scope.get('campaignId'), str) or not scope['campaignId'] or scope != self.expected_scope(scope):
            raise ValueError('exact_batch_scope_required')
        self.scope=copy.deepcopy(scope);self.camera_factory=camera_factory;self.port_factory=port_factory
        self.store=store;self.clock=clock;self.used=False;self.camera=None;self.active_port=None
        self.serial=0;self.counts={'target':0,'positioning':0};self.last_off=None;self.attempts=[]
        self.condition_counts=Counter()
        self.current_capture_seconds=scope['captureSeconds']

    def expected_scope(self, scope):
        return make_scope(scope['campaignId'])

    def scheduled_trials(self):
        return [dict(id=f'trial-{i+1:03d}',session=i//12,split=SPLITS[i//12]) for i in range(120)]

    def capture_seconds(self, session):
        return self.scope['captureSeconds']

    def fresh(self):
        self.camera.check();records=self.camera.samples();at=self.clock.monotonic()
        state=pre_action_state(records,at);latest_settled(records,at)
        if self.last_off is not None and state['windowStart']<self.last_off:
            raise ValueError('observation_precedes_off')
        if at-self.camera.first_source_at>self.current_capture_seconds-6:
            raise ValueError('insufficient_capture_tail')
        state['historyLength']=len(records)
        choose(state,{},0)  # envelope check also applies immediately before dispatch
        return at,state

    def trial(self,row):
        row['status']='failed';trial=row['trial']
        self.observe(self.clock.monotonic()+8)
        _,state=self.fresh()
        pulse=choose(state,self.condition_counts,self.counts['target'])
        at=None
        def guard():
            nonlocal at,state
            at,state=self.fresh()
            # No queue replacement/second command when the required inward direction changes.
            fresh_pulse=choose(state,self.condition_counts,self.counts['target']-1)
            if fresh_pulse!=pulse:raise ValueError('batch_candidate_changed')
            self.store.write_json(trial['id']+'-dispatch.json',dict(at=at,observation=state,pulse=pulse))
            self.recheck_after_storage(at);return at
        receipt=self.pulse(pulse,'target',trial,guard)
        self.condition_counts[(pulse['direction'],pulse['durationMs'])]+=1
        row.update(pulse=pulse,execution=receipt,observation=state,dispatchAt=at,executed=True)
        self.open_port(trial['id']+'-outcome-monitor')
        try:
            while self.clock.monotonic()<at+4.75:
                self.camera.check();remaining=at+4.75-self.clock.monotonic()
                if remaining>0:self.clock.sleep(min(.03,remaining))
            self.camera.check();records=self.camera.samples();now=self.clock.monotonic()
            try:
                selected=window(records,at+3.75,at+4.25,now,3)
                after=statistics.median(r['angleDeg'] for r in selected)
                outcome=dict(status='eligible',measurement=dict(afterDeg=after,deltaDeg=after-state['angleDeg'],
                           windowStart=at+3.75,windowEnd=at+4.25,frames=len(selected)))
            except ValueError as exc:
                if str(exc) not in LABEL_FAILURES:raise
                outcome=dict(status='label_rejected',labelRejection=str(exc))
        finally:self.close_port()
        row.update(outcome)
        self.store.write_json(trial['id']+'-result.json',row)

    def run(self, *, confirmed_scope_sha256):
        if self.used or confirmed_scope_sha256!=canonical_sha256(self.scope):
            raise ValueError('new_exact_batch_scope_required')
        self.used=True;self.store.write_json('campaign-intent.json',self.scope)
        rows=[dict(trial=t,status='not_started',executed=False,measurement=None) for t in self.scheduled_trials()]
        report=dict(schemaVersion='motionbrain.m2_batch_result.v1',status='failed',trials=rows,
                    errors=[],counts=self.counts,commandAttempts=self.attempts,trainingPerformed=False,
                    finalIdleOffVerified=False,scopeClosed=False,completedCaptures=[])
        try:
            self.open_port('campaign-initial');self.close_port();report['finalIdleOffVerified']=True
            for session in range(self.scope['captures']):
                self.current_capture_seconds=self.capture_seconds(session)
                self.camera=self.camera_factory(session);self.camera.start()
                self.observe(self.clock.monotonic()+35)
                for row in [r for r in rows if r['trial']['session']==session]:
                    # Preserve a short block without a new capture or a replacement attempt.
                    if self.clock.monotonic()-self.camera.first_source_at>self.current_capture_seconds-8:
                        row['status']='not_started';row['reason']='capture_tail_reserved';continue
                    report['finalIdleOffVerified']=False;self.trial(row);report['finalIdleOffVerified']=True
                self.open_port(f'capture-{session+1:02d}-tail-monitor')
                try:self.camera.finish()
                finally:self.close_port()
                report['completedCaptures'].append(session);self.camera=None
            report['status']='completed'
        except (Exception,KeyboardInterrupt) as exc:
            report['errors'].append(dict(type=type(exc).__name__,reason=str(exc)))
        finally:
            try:
                self.close_port();self.open_port('campaign-final');self.close_port();report['finalIdleOffVerified']=True
            except (Exception,KeyboardInterrupt) as exc:
                report['finalIdleOffVerified']=False;report['status']='failed'
                report['errors'].append(dict(type='cleanup',reason=type(exc).__name__))
            if self.camera is not None:
                try:self.camera.close()
                except (Exception,KeyboardInterrupt) as exc:
                    report['errors'].append(dict(type='camera_close',reason=type(exc).__name__))
        for row in rows:
            a=[a for a in self.attempts if a['trialId']==row['trial']['id']]
            row['commandAttempts']=copy.deepcopy(a)
            row['executionStatus']=a[-1]['executionStatus'] if a else 'not_dispatched'
            row['executed']=True if row['executionStatus']=='executed' else None if row['executionStatus']=='outcome_unknown' else False
        report['summary']=dict(planned=len(rows),executed=sum(r['executed'] is True for r in rows),
            unknown=sum(r['executed'] is None for r in rows),eligible=sum(r['status']=='eligible' for r in rows),
            rejected=sum(r['status']=='label_rejected' for r in rows),notStarted=sum(r['status']=='not_started' for r in rows))
        report['scopeClosed']=True;self.store.write_json('campaign-result.json',report);return report


def validate_rows(rows):
    """Variable-size v2 data, locked whole-capture splits, explicit command support."""
    if not isinstance(rows,list) or not 1<=len(rows)<=120:raise ValueError('batch_rows_required')
    seen=set();sessions={};counts=Counter()
    for r in rows:
        i=r['sessionIndex'];identity=r['sessionId']
        if type(i) is not int or not 0<=i<10 or r['split']!=SPLITS[i]:raise ValueError('batch_split_mismatch')
        if not isinstance(identity,str) or not identity:raise ValueError('batch_session_required')
        if i in sessions and sessions[i]!=identity:raise ValueError('batch_session_changed')
        sessions[i]=identity
        if r['id'] in seen:raise ValueError('duplicate_batch_row')
        seen.add(r['id']);key=(r['direction'],r['durationMs'])
        if r['x']!=features(r['x'][0],r['x'][1],*key) or not finite(r['y']):raise ValueError('batch_feature_contract')
        counts[(r['split'],key)]+=1
    if len(set(sessions.values()))!=len(sessions):raise ValueError('batch_session_leakage')
    if any(sum(r['sessionIndex']==i for r in rows)>12 for i in sessions):raise ValueError('batch_session_budget')
    if any(counts[(s,k)]<(3 if s=='train' else 1) for s in ('train','validation','test') for k in COMMANDS):
        raise ValueError('batch_command_support_insufficient')
    if any(len({r['sessionIndex'] for r in rows if r['split']==s})<2 for s in ('train','validation','test')):
        raise ValueError('batch_independent_capture_support_insufficient')


def train_and_evaluate(rows):
    validate_rows(rows)
    groups={s:[r for r in rows if r['split']==s] for s in ('train','validation','test')}
    xs={s:np.asarray([r['x'] for r in rr]) for s,rr in groups.items()}
    ys={s:np.asarray([r['y'] for r in rr]) for s,rr in groups.items()}
    candidates=[]
    for alpha in ALPHAS:
        model=fit(xs['train'],ys['train'],alpha)
        candidates.append(dict(model=model,validation=scores(ys['validation'],predict(model,xs['validation']))))
    selected=min(candidates,key=lambda c:(c['validation']['maeDeg'],c['model']['alpha']))['model']
    results={}
    for s,rr in groups.items():
        y=ys[s];x=xs[s];p=predict(selected,x)
        by_command={}
        for d,ms in COMMANDS:
            mask=np.asarray([r['direction']==d and r['durationMs']==ms for r in rr])
            by_command[f'{d}-{ms}']=scores(y[mask],p[mask])
        results[s]=dict(selected=scores(y,p),noChange=scores(y,np.zeros(len(y))),
            constantVelocity=scores(y,x[:,1]*4),maxAbsErrorDeg=float(np.abs(p-y).max()),
            p90AbsErrorDeg=float(np.quantile(np.abs(p-y),.9)),byCommand=by_command,
            predictions=[dict(id=r['id'],sessionId=r['sessionId'],actualDeltaDeg=float(a),predictedDeltaDeg=float(b)) for r,a,b in zip(rr,y,p)])
    return dict(schemaVersion='motionbrain.projected_joint_prediction.v2',inputNames=list(INPUT_NAMES),
        horizonSeconds=4.,selectedModel=selected,candidates=candidates,splits=results,
        controlEligible=False,independentDayOrViewGeneralization=False,physicalJointAngleCalibrated=False)
