"""직접 M3 명령별 반응·명시 도구 전용 투영·코호트 보존 무장치 회귀."""
import copy
import unittest
import cv2
import numpy as np
from learning import m3_direct_tool_response as direct
from learning.marker_virtual_observer import VirtualMarkerObserver
from tests import test_marker_virtual_observer as fixture


class DirectToolTests(unittest.TestCase):
    def setUp(self):
        setup=fixture.VirtualMarkerTests();setup.setUp();self.seed=setup.raw;self.image=setup.image
        self.registration=setup.registration;self.projector=direct.ToolOnlyProjector(self.registration,self.seed)
    def raw(self,dx=0,dy=0,pink=True):
        im=self.image.copy()
        if not pink:im[620:810,780:855]=180
        affine=np.float32([[1,0,dx],[0,1,dy]])
        im=cv2.warpAffine(im,affine,(1920,1080),borderValue=(180,180,180))
        return cv2.imencode('.png',im)[1].tobytes()
    def obs(self,dx=0,dy=0,pink=True,m4=195.29):return self.projector.observe(self.raw(dx,dy,pink),m4)
    def rows(self):
        rows=[]
        for i,(command,before,after) in enumerate([(direct.COMMANDS[0],self.obs(pink=False),self.obs(2,6,pink=False)),
            (direct.COMMANDS[0],self.obs(3,3,pink=False),self.obs(6,12,pink=False)),
            (direct.COMMANDS[1],self.obs(2,-5),self.obs(52,100))]):
            identity=dict(deviceId='camera',helperSha256='c'*64,configuration=['d'*64,1920,1080,False,0],viewId='old-14' if i<2 else 'current-15')
            evidence=dict(finalIdleOffVerified=True,actualMotor=3,recordedCommand=copy.deepcopy(command),
                sessionJsonSha256=str(i+1)*64,beforeImageSha256=before['imageSha256'],afterImageSha256=after['imageSha256'],
                beforePixelSha256='a'*64,afterPixelSha256='b'*64,beforeSensorSha256='a'*64,afterSensorSha256='b'*64,
                commandReceiptSha256='e'*64,rawAuditSha256='f'*64,cameraIdentity=identity,
                controllerBootId=('a' if i<2 else 'c')*32,gripperCameraBootId=('b' if i<2 else 'd')*32,
                sourceRole='historical_manual_collection' if i<2 else 'model_selected_evaluation_reused_for_development',
                meetsCurrentManualCollectionPolicy=False if i<2 else None,newModelIndependentEvaluation=False)
            rows.append(dict(sessionId='s'+str(i),sourceSha256=str(i+1)*64,command=copy.deepcopy(command),before=before,after=after,sourceEvidence=evidence))
        return rows
    def fit(self):return direct.fit(self.rows(),projection_contract=self.projector.contract,audit_sources=lambda rows:rows)
    def test_tool_only_is_new_contract_and_raw_projection_survives_missing_pink(self):
        missing=self.raw(pink=False)
        with self.assertRaises(ValueError):VirtualMarkerObserver(self.registration,self.seed).observe(missing)
        tool=self.projector.observe(missing,195.29)
        np.testing.assert_allclose(tool['state'][:2],[938,375],atol=.001)
        self.assertNotEqual(self.projector.binding,self.projector.source_binding)
        self.assertFalse(tool['objectDetectionRequired']);self.assertFalse(tool['fullObjectObservationClaimed'])
    def test_from_full_matches_raw_state_and_rejects_proxy_axis_and_contract_tampering(self):
        full=VirtualMarkerObserver(self.registration,self.seed).observe(self.seed)
        raw=self.projector.observe(self.seed,195.29);mapped=self.projector.from_full(full,195.29)
        self.assertEqual(mapped['state'],raw['state']);self.assertEqual(mapped['projectionContractSha256'],raw['projectionContractSha256'])
        for kind in ('proxy','axis','registration','pair'):
            bad=copy.deepcopy(full)
            if kind=='proxy':bad['virtualProxyPx'][0]+=1
            elif kind=='axis':bad['toolAxisImageAngleDeg']+=1
            elif kind=='pair':bad['observedMarkerPair']['teal']['center'][0]+=1
            else:bad['registrationSha256']='f'*64
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.projector.from_full(bad,195.29)
    def test_missing_marker_or_dark_support_is_never_relaxed_for_tool_only(self):
        for kind in ('marker','dark'):
            im=self.image.copy()
            if kind=='marker':im[370:400,1065:1100]=30
            else:im[350:401,910:965]=180
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.projector.observe(cv2.imencode('.png',im)[1].tobytes(),195.29)
    def test_exact_commands_source_counts_views_and_roles_remain_separate(self):
        model=self.fit();self.assertEqual([c['sourceCount'] for c in model['categories']],[2,1])
        self.assertEqual([c['singleton'] for c in model['categories']],[False,True])
        self.assertEqual([c['cameraIdentity']['viewId'] for c in model['sourceCohorts']],['old-14','old-14','current-15'])
        self.assertFalse(model['sourceCohorts'][0]['meetsCurrentManualCollectionPolicy'])
        self.assertEqual(model['sourceCohorts'][2]['sourceRole'],'model_selected_evaluation_reused_for_development')
        self.assertFalse(model['controlEligible']);self.assertFalse(model['prospectivelyTested'])
        short=direct.predict(model,self.obs(),direct.COMMANDS[0]);long=direct.predict(model,self.obs(),direct.COMMANDS[1])
        np.testing.assert_allclose(short['observedSourceDeltaState'][:2],[2,6],atol=.01)
        np.testing.assert_allclose(long['observedSourceDeltaState'][:2],[50,105],atol=.01)
        self.assertFalse(short['support']['singleton']);self.assertTrue(long['support']['singleton'])
        with self.assertRaisesRegex(ValueError,'categorical'):direct.predict(model,self.obs(),dict(joint='elbow',direction='down',percent=60,durationMs=500))
    def test_supported_short_beats_long_and_STOP_for_same_explicit_goal_without_scaling(self):
        model=self.fit();current=self.obs();result=direct.select(model,current,[942.,383.])
        self.assertEqual(result['selected']['command'],direct.COMMANDS[0]);self.assertFalse(result['controlEligible'])
        self.assertEqual(len(result['candidates']),3);self.assertEqual(result['maximumCommands'],1)
        stopped=direct.select(model,current,[938.,375.]);self.assertEqual(stopped['selected']['kind'],'stop')
        exhausted=direct.select(model,current,[942.,383.],commands_used=1);self.assertEqual(exhausted['selected']['kind'],'stop')
        self.assertEqual(len(exhausted['candidates']),1)
    def test_local_support_and_reaudited_source_guards_cannot_be_relaxed(self):
        model=self.fit()
        with self.assertRaisesRegex(ValueError,'declared_support'):direct.predict(model,self.obs(m4=212.),direct.COMMANDS[0])
        changed=copy.deepcopy(self.obs());changed['state'][0]+=1
        with self.assertRaisesRegex(ValueError,'tool_evidence_changed'):direct.predict(model,changed,direct.COMMANDS[0])
        bad=self.rows();bad[0]['sourceEvidence']['finalIdleOffVerified']=False
        with self.assertRaises(ValueError):direct.fit(bad,projection_contract=self.projector.contract,audit_sources=lambda rows:rows)
        with self.assertRaisesRegex(ValueError,'reaudit_changed'):direct.fit(self.rows(),projection_contract=self.projector.contract,audit_sources=lambda rows:[])
        model['supportPolicy']['maximumM4DifferenceDeg']=20.
        with self.assertRaises(ValueError):direct.predict(model,self.obs(),direct.COMMANDS[0])
    def test_held_command_out_is_not_replaced_by_other_duration_or_inverse(self):
        rows=self.rows()[:2];model=direct.fit(rows,projection_contract=self.projector.contract,audit_sources=lambda rows:rows)
        with self.assertRaisesRegex(ValueError,'exact_command_not_trained'):direct.predict(model,self.obs(),direct.COMMANDS[1])
        result=direct.select(model,self.obs(),[942.,383.],candidates=[direct.COMMANDS[1]])
        self.assertEqual(result['selected']['kind'],'stop');self.assertEqual(len(result['rejected']),1)

if __name__=='__main__':unittest.main()
