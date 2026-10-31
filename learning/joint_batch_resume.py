"""Exact remaining attempts from a closed 19-command batch; no device access."""
import copy
from collections import Counter
from learning.joint_batch import BatchCollection, COMMANDS, SPLITS
from learning.joint_action_selection import canonical_sha256


def checkpoint(report):
    if (report.get('schemaVersion')!='motionbrain.m2_batch_result.v1' or report.get('scopeClosed') is not True
            or report.get('finalIdleOffVerified') is not True or report.get('status')!='failed'
            or report.get('completedCaptures')!=[0] or len(report.get('trials',[]))!=120):
        raise ValueError('closed_batch_checkpoint_required')
    rows=report['trials'];executed=[];remaining=[];counts=Counter()
    for i,r in enumerate(rows):
        t=r['trial']
        if t!=dict(id=f'trial-{i+1:03d}',session=i//12,split=SPLITS[i//12]):raise ValueError('checkpoint_schedule_changed')
        if r['executed'] is True and r['executionStatus']=='executed':
            p=r['pulse'];key=(p['direction'],p['durationMs'])
            if key not in COMMANDS or p['pwmMagnitude']!=160 or r['execution']['offVerified'] is not True:raise ValueError('checkpoint_command_changed')
            executed.append(t['id']);counts[key]+=1
        elif r['executed'] is False and r['executionStatus']=='not_dispatched' and r['status']=='not_started' and not r['commandAttempts']:
            remaining.append(t['id'])
        else:raise ValueError('unknown_or_attempted_checkpoint_row')
    if executed!=[f'trial-{i:03d}' for i in list(range(1,12))+list(range(13,21))] or len(remaining)!=101:
        raise ValueError('exact_19_executed_checkpoint_required')
    return remaining,counts


def make_scope(campaign_id,report):
    remaining,counts=checkpoint(report)
    return dict(schemaVersion='motionbrain.m2_batch_remaining.v1',campaignId=campaign_id,
        previousReportSha256=canonical_sha256(report),remainingTrialIds=remaining,
        captures=9,captureSeconds=90,captureSecondsBySession=[45]+[90]*8,
        maxFramesBySession=[380]+[740]*8,maxBytesBySession=[1080000000]+[2160000000]*8,
        maxTargetCommands=101,maxPositioningCommands=0,pwmMagnitude=160,durationsMs=[550,700,900],
        angleEnvelopeDeg=[-10,24],inwardThresholdsDeg=[5,10],sampleFps=8,
        splitByCapture=list(SPLITS[1:]),priorCommands=19,automaticRetries=0,horizonSeconds=4.)


class RemainingCollection(BatchCollection):
    def __init__(self,scope,report,*args,**kwargs):
        self.checkpoint_report=copy.deepcopy(report)
        super().__init__(scope,*args,**kwargs)
        self.condition_counts=checkpoint(report)[1]
    def expected_scope(self,scope):return make_scope(scope['campaignId'],self.checkpoint_report)
    def capture_seconds(self,session):return self.scope['captureSecondsBySession'][session]
    def scheduled_trials(self):
        rows=[]
        for i,identity in enumerate(self.scope['remainingTrialIds']):
            session=0 if i<5 else 1+(i-5)//12
            rows.append(dict(id=identity,session=session,split=SPLITS[session+1],logicalSessionIndex=session+1))
        return rows
