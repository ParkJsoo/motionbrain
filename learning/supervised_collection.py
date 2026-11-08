"""Attended finite single-joint blocks. No live image measurement or training."""
from __future__ import annotations
import copy
import hashlib
import json
import math
import re
import time
from urllib.parse import urlsplit

SCHEMA = 'motionbrain.supervised_collection.v1'
# Historical observed command ceilings, NOT travel guarantees. M4 up is withheld
# after the previous overshoot; the other directions retain individual limits.
PROFILES = {
    'gripper': {'motor': 1, 'directions': {'open': (1, 60, 650), 'close': (-1, 60, 650)}},
    'wrist': {'motor': 2, 'directions': {'up': (1, 63, 900), 'down': (-1, 63, 900)}},
    'elbow': {'motor': 3, 'directions': {'up': (-1, 75, 1000), 'down': (1, 60, 750)}},
    'shoulder': {'motor': 4, 'directions': {'down': (-1, 35, 650)}},
    'base': {'motor': 5, 'directions': {'left': (1, 60, 700), 'right': (-1, 60, 700)}},
}

def number(x): return type(x) in (int,float) and math.isfinite(x)
def identifier(x): return isinstance(x,str) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,100}',x) is not None

def plan_digest(plan):
    return hashlib.sha256(json.dumps(plan,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def validate_plan(plan):
    required={'schemaVersion','sessionId','captureGroup','split','controllerUrl','controllerBootId','helperSha256','camera','pulses','postSeconds','beforeSeconds','operatorSupervised','poseContext'}
    if not isinstance(plan,dict) or set(plan) not in (required, required | {'armReferenceTemplateSha256'}) or plan['schemaVersion']!=SCHEMA:
        raise ValueError('supervised_plan_schema')
    if not identifier(plan['sessionId']) or not identifier(plan['captureGroup']) or not identifier(plan['poseContext']):
        raise ValueError('plan_identifiers')
    if plan['split'] not in ('development','train','validation','test') or plan['operatorSupervised'] is not True:
        raise ValueError('supervised_split_required')
    if not isinstance(plan['controllerBootId'],str) or not re.fullmatch('[0-9a-f]{32}',plan['controllerBootId']):
        raise ValueError('controller_boot_identity_required')
    u=urlsplit(plan['controllerUrl'])
    if u.scheme!='http' or not u.hostname or u.username or u.password or u.path not in ('','/') or u.query or u.fragment:
        raise ValueError('plain_controller_url_required')
    _=u.port
    if not isinstance(plan['helperSha256'],str) or not re.fullmatch('[0-9a-f]{64}',plan['helperSha256']):
        raise ValueError('helper_hash_required')
    if plan['beforeSeconds']!=1. or plan['postSeconds']!=3.:
        raise ValueError('fixed_observation_waits_required')
    pulses=plan['pulses']
    if not isinstance(pulses,list) or not 1<=len(pulses)<=3: raise ValueError('one_to_three_pulses_required')
    direction=None
    for p in pulses:
        if not isinstance(p,dict) or set(p)!={'joint','motor','direction','percent','durationMs','expectedPwm'}:
            raise ValueError('pulse_schema')
        profile=PROFILES.get(p['joint'],{}); limits=profile.get('directions',{}).get(p['direction'])
        if limits is None: raise ValueError('joint_direction_not_enabled')
        sign,cap,duration=limits
        if type(p['motor']) is not int or p['motor']!=profile['motor'] or type(p['percent']) is not int or not 1<=p['percent']<=cap:
            raise ValueError('joint_output_limit')
        if type(p['durationMs']) is not int or not 100<=p['durationMs']<=duration:
            raise ValueError('joint_duration_limit')
        if type(p['expectedPwm']) is not int or p['expectedPwm']!=sign*(p['percent']*255//100):
            raise ValueError('expected_driver_pwm_mismatch')
        key=(p['joint'],p['direction'])
        if direction is not None and direction!=key: raise ValueError('single_joint_direction_block_required')
        direction=key
        if p['joint']=='shoulder' and len(pulses)!=1: raise ValueError('shoulder_single_pulse_only')
    if 'armReferenceTemplateSha256' in plan:
        if (not isinstance(plan['armReferenceTemplateSha256'], str)
                or not re.fullmatch('[0-9a-f]{64}', plan['armReferenceTemplateSha256'])):
            raise ValueError('arm_reference_template_hash_required')
        if len(pulses) != 1:
            raise ValueError('arm_reference_single_pulse_required')
    c=plan['camera']
    if not isinstance(c,dict) or set(c)!={'deviceId','sessionId','viewId','width','height','maxFrames','seconds','sampleFps','maxBytes'}:
        raise ValueError('camera_schema')
    if not isinstance(c['deviceId'],str) or not c['deviceId'] or c['sessionId']!=plan['sessionId'] or not identifier(c['viewId']):
        raise ValueError('camera_identity')
    if (c['width'],c['height'],c['sampleFps'])!=(1920,1080,8): raise ValueError('recording_format')
    if type(c['seconds']) is not int or not 30<=c['seconds']<=120 or type(c['maxFrames']) is not int or not c['seconds']*8<=c['maxFrames']<=1000:
        raise ValueError('recording_budget')
    if type(c['maxBytes']) is not int or not c['maxFrames']*(1920*1080*4+65536)<=c['maxBytes']<=8_400_000_000:
        raise ValueError('worst_case_storage_budget_required')
    return copy.deepcopy(plan)


def make_plan(*,session_id,capture_group,split,controller_url,boot_id,helper_sha256,device_id,view_id,pose_context,joint,direction,percent,durations):
    profile=PROFILES[joint];sign,_,_=profile['directions'][direction]
    plan=dict(schemaVersion=SCHEMA,sessionId=session_id,captureGroup=capture_group,split=split,
        controllerUrl=controller_url,controllerBootId=boot_id,helperSha256=helper_sha256,
        operatorSupervised=True,poseContext=pose_context,beforeSeconds=1.,postSeconds=3.,
        camera=dict(deviceId=device_id,sessionId=session_id,viewId=view_id,width=1920,height=1080,
                    maxFrames=500,seconds=60,sampleFps=8,maxBytes=500*(1920*1080*4+65536)),
        pulses=[dict(joint=joint,motor=profile['motor'],direction=direction,percent=percent,durationMs=d,
                    expectedPwm=sign*(percent*255//100)) for d in durations])
    return validate_plan(plan)


class SupervisedCollection:
    def __init__(self,plan,camera,port,store,*,clock=time,arm_reference_template=None):
        self.plan=validate_plan(plan);self.camera=camera;self.port=port;self.store=store;self.clock=clock;self.used=False
        self.arm_reference_template=arm_reference_template
        if arm_reference_template is not None or 'armReferenceTemplateSha256' in self.plan:
            from learning.arm_session_reference import load_template
            if arm_reference_template is None or 'armReferenceTemplateSha256' not in self.plan:
                raise ValueError('arm_reference_template_and_plan_pin_required')
            load_template(arm_reference_template,self.plan['armReferenceTemplateSha256'])

    def wait(self,seconds):
        end=self.clock.monotonic()+seconds
        while self.clock.monotonic()<end:
            self.camera.check()
            # Confirm quiet state during the post-output interval; no image gate.
            self.port.check_idle()
            remaining=end-self.clock.monotonic()
            if remaining>0:self.clock.sleep(min(.1,remaining))
        self.camera.check()

    def run(self,confirmed_hash):
        if self.used or confirmed_hash!=plan_digest(self.plan):raise ValueError('one_use_confirmed_plan_required')
        self.used=True
        report=dict(schemaVersion=SCHEMA,plan=self.plan,planSha256=confirmed_hash,status='failed',trials=[],frames=[],errors=[],finalIdleOffVerified=False,trainingPerformed=False)
        for i,pulse in enumerate(self.plan['pulses']):
            report['trials'].append(dict(id=f"{self.plan['sessionId']}-{i+1:03d}",pulse=pulse,status='not_started',offVerified=False))
        touched=False;camera_started=False
        self.store.write_json('plan.json',self.plan)
        try:
            if self.arm_reference_template is not None:
                self.store._write('arm-reference-template.json',self.arm_reference_template)
            self.port.preflight() # no mutations before this succeeds
            camera_started=True;self.camera.start();self.camera.wait_ready(timeout=15)
            if self.arm_reference_template is not None:
                # Optional registration gets a nearby read-only sensor snapshot
                # before feature checks; the default path keeps its old timing.
                self.port.check_idle()
                from learning.arm_session_reference import build_registration
                if not self.camera.frames:raise ValueError('arm_reference_first_frame_missing')
                registration=build_registration(self.store.root,self.plan,self.arm_reference_template,copy.deepcopy(self.camera.frames[0]))
                registration['registeredAt']=self.clock.monotonic()
                receipt=self.store.write_json('arm-reference.json',registration)
                report['armReference']=registration
                report['armReferenceSha256']=receipt['sha256']
            self.wait(self.plan['beforeSeconds'])
            for i,row in enumerate(report['trials']):
                self.camera.check();self.port.preflight()
                self.store.write_json(f'trial-{i+1:03d}-intent.json',dict(id=row['id'],pulse=row['pulse'],at=self.clock.monotonic()))
                row['status']='not_dispatched';touched=True
                try:
                    receipt=self.port.execute(row['pulse'],self.camera.check)
                    if (not isinstance(receipt,dict) or receipt.get('offVerified') is not True or not all(number(receipt.get(k)) for k in ('dispatchBefore','dispatchAfter','offAt'))
                        or not receipt['dispatchBefore']<=receipt['dispatchAfter']<=receipt['offAt']):
                        raise ValueError('execution_receipt_invalid')
                    row.update(receipt);row['status']='executed'
                    self.wait(self.plan['postSeconds'])
                    self.port.check_idle()
                    row['quietVerifiedUntil']=self.clock.monotonic()
                    row['postEnd']=row['quietVerifiedUntil']
                    row['measurementReady']=True
                except BaseException:
                    if row['status']!='executed':
                        row['status']='outcome_unknown' if self.port.may_have_run else 'not_dispatched'
                        for key,attr in (('dispatchBefore','last_dispatch_before'),('dispatchAfter','last_dispatch_after')):
                            value=getattr(self.port,attr,None)
                            if number(value):row[key]=value
                    raise
                finally:
                    self.store.write_json(f'trial-{i+1:03d}-result.json',row)
            report['status']='completed'
        except (Exception,KeyboardInterrupt) as exc:
            report['errors'].append(dict(type=type(exc).__name__,reason=str(exc) if isinstance(exc,ValueError) else 'collection_interrupted_or_failed'))
        finally:
            try:
                if touched:report['finalIdleOffVerified']=self.port.finish() is True
                else:
                    self.port.preflight();report['finalIdleOffVerified']=True
            except (Exception,KeyboardInterrupt): report['errors'].append(dict(type='cleanup',reason='off_not_verified'))
            if camera_started:
                try: report['recording']=self.camera.close()
                except (Exception,KeyboardInterrupt):report['recording']=dict(status='failed',reason='close_failed')
            if report.get('recording',{}).get('status')=='failed':
                report['status']='failed'
                report['errors'].append(dict(type='recording',reason='recording_finalization_failed'))
            report['frames']=copy.deepcopy(self.camera.frames)
            if not report['finalIdleOffVerified']:report['status']='failed'
            report['summary']={key:sum(r['status']==key for r in report['trials']) for key in ('executed','outcome_unknown','not_dispatched','not_started')}
            report['summary']['measurementCandidates']=sum(r.get('measurementReady') is True for r in report['trials'])
            self.store.write_json('report.json',report)
        return report
