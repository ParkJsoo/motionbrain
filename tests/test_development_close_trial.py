"""개발 CLOSE 진입의 합성 반례. 실물 파지 성공 시험이 아니다."""
import copy
import hashlib
import json
import unittest
import cv2
import numpy as np
from learning import development_close_trial as trial
from learning import m1_near_action_model as model
from learning.near_gripper_tip_observer import NearGripperTipObserver
from tests.test_limited_scene_cycle_assessment import external


def near(gap=134, width=39, height=40):
    frame = np.full((240,320,3),180,np.uint8)
    for x in (160-gap//2,160+gap//2):
        cv2.rectangle(frame,(x-5,20),(x+5,95),(30,30,30),-1)
    cv2.rectangle(frame,(142,82),(141+width,81+height),(100,20,240),-1)
    ok, raw = cv2.imencode('.png',frame)
    assert ok
    return NearGripperTipObserver(policy_version=3).observe(raw.tobytes())


def sources():
    rows=[]
    for i,(direction,b,a) in enumerate([('close',134,114),('close',114,94),('open',94,114),('open',114,134),
                                        ('close',94,74),('open',74,94),('close',74,54)]):
        before,after=near(b),near(a)
        rows.append(dict(sessionId=f's{i}',sourceSha256=str(i+1)*64,observationContractSha256=before['observationContractSha256'],
            command=dict(joint='gripper',direction=direction,percent=70,durationMs=1000),before=before,after=after,
            sourceEvidence=dict(beforeExternalImageSha256='a'*64,afterExternalImageSha256='b'*64,
                beforeGripperImageSha256=before['imageSha256'],afterGripperImageSha256=after['imageSha256'])))
    frozen=model.fit(rows,reference=rows[0]['before'],audit_sources=lambda r:r)
    raw_model=json.dumps(frozen).encode(); model_sha=hashlib.sha256(raw_model).hexdigest()
    ext=external(width=59);ext['virtualProxyPx']=[800.,650.]
    ref=dict(upAfterExternal=ext,upAfterNear=near(),closeBeforeExternal=copy.deepcopy(ext),closeBeforeNear=near(height=41),
             modelFileSha256s=dict(near=model_sha),nearContractSha256=frozen['observationContractSha256'])
    reference=dict(contract=ref,sha256=trial.digest(ref));g=dict(referenceContractSha256=reference['sha256'],targetVirtualPointPx=[800.,650.])
    pose=dict(schemaVersion='motionbrain.M1_reported_angle_cohort_admission.v3',baseRadiusDeg=.5,reportedAngleOneLsbDeg=360./4096.,
        admissionRadiusDeg=.5+360./4096.,modelWeightsChanged=False,imageLocalSupportChanged=False,
        frozenModelFileSha256=model_sha,observationContractSha256=frozen['observationContractSha256'],
        beforeM4Mapping=[dict(sessionId=r['sessionId'],sourceSessionSha256=r['sourceSha256'],beforeM4MedianDeg=192.92) for r in rows])
    scope=dict(targetAngleDeg=192.42,controllerBootId='controller',cameraBootId='camera')
    result=dict(scopeSha256=trial.digest(scope),actualAngleDeg=192.12,actualAngleErrorDeg=-.3,status='completed',errors=[],
        commandOutcome='receipt_recorded',modelSelected=True,automaticRetry=False,actualAngleWithinNativeTolerance=True,
        deviceTargetReached=True,nativeReportedTargetReached=True,finalIdleOffVerified=True,nativeTerminalReason='TARGET_REACHED',nativeSettledToleranceDeg=.4)
    proof=dict(controllerBootId='controller',cameraBootId='camera',M4Deg=192.12,allFiveOff=True,currentState='IDLE',nativeTerminalReason='TARGET_REACHED',
        firmwarePolicy=dict(targetDeg=192.42,angleDeg=192.12,settledSuccessToleranceDeg=.4,active=False,correctionActive=False))
    values=dict(reference=reference,goal=dict(goal=g,sha256=trial.digest(g)),model=frozen,pose_policy=pose,
        native_scope=scope,native_result=result,native_proof=proof)
    raw={k:json.dumps(v).encode() for k,v in values.items()};raw['model']=raw_model
    return raw


class DevelopmentCloseTests(unittest.TestCase):
    def setUp(self):
        self.raw=sources();self.hashes={k:hashlib.sha256(v).hexdigest() for k,v in self.raw.items()}
        self.contract=trial.build_contract(self.raw,expected_source_sha256s=self.hashes)
        self.ext=external(width=59);self.ext['virtualProxyPx']=[802.595,655.462]
        self.kw=dict(table_baseline=external(width=59),current_m4_deg=192.12,controller_boot_id='controller',camera_boot_id='camera')

    def assess(self,**change):
        kw=dict(self.kw);kw.update(change)
        return trial.assess(self.contract,self.ext,near(126,height=38),**kw)

    def test_current_geometry_admitted_only_as_trial_and_strict_failure_preserved(self):
        original=copy.deepcopy(self.contract);r=self.assess()
        self.assertTrue(r['accepted'],r);self.assertFalse(r['originalStrictAssessment']['accepted'])
        self.assertFalse(r['controlEligible']);self.assertFalse(r['evidence']['originalPoseSupported'])
        self.assertFalse(r['evidence']['learnedM1PoseSupportExpanded']);self.assertFalse(r['evidence']['physicalGraspVerified'])
        self.assertEqual(original,self.contract)

    def test_old_failed_after_up_size_and_large_goal_error_each_reject(self):
        bad=trial.assess(self.contract,self.ext,near(134,width=33,height=32),**self.kw)
        self.assertFalse(bad['accepted']);self.assertIn('absolute_width',bad['evidence']['reason'])
        self.ext['virtualProxyPx'][1]-=51
        self.assertFalse(self.assess()['accepted'])

    def test_wrong_boot_pose_drift_nonfinite_and_followup_are_not_first_entry(self):
        for change in [dict(controller_boot_id='other'),dict(camera_boot_id='other'),dict(current_m4_deg=192.53),
                       dict(current_m4_deg=float('nan')),dict(commands_used=1),dict(commands_used=True)]:
            self.assertFalse(self.assess(**change)['accepted'],change)

    def test_raw_source_and_faked_anchor_or_target_are_rejected(self):
        changed=dict(self.raw);changed['native_result']+=b' '
        with self.assertRaisesRegex(ValueError,'source_file_changed'):trial.build_contract(changed,expected_source_sha256s=self.hashes)
        for name,key,value in [('native_result','actualAngleDeg',190.),('native_scope','targetAngleDeg',190.),
                               ('native_proof','allFiveOff',False)]:
            changed=dict(self.raw);parsed=json.loads(changed[name]);parsed[key]=value;changed[name]=json.dumps(parsed).encode()
            hashes={k:hashlib.sha256(v).hexdigest() for k,v in changed.items()}
            with self.assertRaises(ValueError):trial.build_contract(changed,expected_source_sha256s=hashes)

    def test_contract_tampering_and_missing_sources_fail_closed(self):
        self.contract['contract']['nativeActualAnchorDeg']=190.
        self.contract['sha256']=trial.digest(self.contract['contract'])
        self.assertFalse(self.assess()['accepted'])
        changed=dict(self.raw);del changed['native_result']
        with self.assertRaises(ValueError):trial.build_contract(changed,expected_source_sha256s=self.hashes)

    def test_visibility_table_contract_and_absolute_size_fail_closed(self):
        bad=near(126,height=38);bad['nearClosingVisibilityGate']['passed']=False
        self.assertFalse(trial.assess(self.contract,self.ext,bad,**self.kw)['accepted'])
        self.assertFalse(self.assess(table_baseline=external(dx=20,width=59))['accepted'])
        bad=near(126,height=44)
        self.assertFalse(trial.assess(self.contract,self.ext,bad,**self.kw)['accepted'])
        bad=near(126,height=38);bad['observationContractSha256']='a'*64
        self.assertFalse(trial.assess(self.contract,self.ext,bad,**self.kw)['accepted'])

if __name__=='__main__':unittest.main()
