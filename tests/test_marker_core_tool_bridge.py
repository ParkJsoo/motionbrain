"""원 중심 보존·core 품질 분리·원본 감사와 기존 수치 모델 재사용 검증."""
import copy
import hashlib
import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from learning import marker_core_tool_bridge as bridge,marker_virtual_observer as marker,m3_direct_tool_response as direct
from tests import test_m3_direct_tool_response as direct_fixture


class CoreBridgeTests(unittest.TestCase):
    def setUp(self):
        self.fixture=direct_fixture.DirectToolTests();self.fixture.setUp();self.model=self.fixture.fit()
        self.raw_model=json.dumps(self.model,sort_keys=True).encode()
        self.projector=bridge.CoreBridgeProjector(self.raw_model,self.fixture.seed)
        f=self.fixture
        self.sources=[dict(sessionId='s0',before=f.raw(pink=False),after=f.raw(2,6,pink=False)),
            dict(sessionId='s1',before=f.raw(3,3,pink=False),after=f.raw(6,12,pink=False)),
            dict(sessionId='s2',before=f.raw(2,-5),after=f.raw(52,100))]
        self.audit=bridge.audit_sources(self.projector,self.sources)
    def encode(self,image):return cv2.imencode('.png',image)[1].tobytes()
    def fallback_raw(self):
        image=self.fixture.image.copy();cv2.rectangle(image,(1121,380),(1142,382),(80,110,65),-1)
        return self.encode(image)
    def reseal(self,audit):
        audit['auditSha256']=bridge.digest({k:v for k,v in audit.items() if k!='auditSha256'});return audit
    def test_original_default_policy_and_prediction_are_unchanged(self):
        self.assertEqual(marker.digest(marker.detector_policy()),'2bd56d99d4c5cd9f9770da9242f20b2d724faa0a6b523e68565e82432e5216eb')
        current=self.fixture.obs();new=self.projector.observe(self.fixture.raw(),195.29)
        for command in direct.COMMANDS:
            old=direct.predict(self.model,current,command);response=bridge.predict(self.projector,self.audit,new,command)
            for key in ('predictedState','commandMeanState','observedSourceDeltaState','support','modelSha256'):
                self.assertEqual(old[key],response[key])
        self.assertNotEqual(new['schemaVersion'],current['schemaVersion'])
        with self.assertRaisesRegex(ValueError,'tool_projection_required'):direct.observation(new,self.model['projectionContract'])
        with self.assertRaisesRegex(ValueError,'new_observation_contract_required'):bridge.observation(self.projector,current)
    def test_fallback_keeps_full_original_centroid_bbox_area_and_solidity(self):
        raw=self.fallback_raw();image=marker._image(raw)
        original=marker._candidates(image,'teal',dict(marker.detector_policy(),minimumSolidity=0.))[0]
        self.assertLess(original['solidity'],.7)
        with self.assertRaises(ValueError):self.fixture.projector.observe(raw,195.29)
        result=self.projector.observe(raw,195.29);bridge.observation(self.projector,result)
        self.assertEqual(result['markerPair']['teal'],original)
        validation=result['quality']['tealValidation'];self.assertEqual(validation['mode'],'core_quality_original_geometry')
        self.assertEqual(validation['originalSolidity'],original['solidity']);self.assertGreaterEqual(validation['core']['solidity'],.7)
        self.assertGreaterEqual(validation['coreFullAreaFraction'],.5)
        self.assertNotEqual(validation['core']['center'],original['center'])
        self.assertFalse(result['originalContractClaimed']);self.assertEqual(result['newObserverSha256'],self.projector.observer_sha256)
    def test_core_mapping_rejects_cross_component_background_and_shape_mismatch(self):
        full=np.ones((8,8),np.int32);core=np.zeros_like(full);core[1:7,1:7]=1
        stats=np.zeros((2,5),np.int32);stats[1,4]=80
        self.assertEqual(bridge._core_mapping(full,core,stats),{1:[1]})
        for value in (0,2):
            changed=full.copy();changed[3,3]=value
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'not_wholly_unique'):bridge._core_mapping(changed,core,stats)
        with self.assertRaisesRegex(ValueError,'label_shape'):bridge._core_mapping(full[:2],core,stats)
    def test_two_large_cores_are_not_selected_or_merged(self):
        image=np.full((1080,1920,3),180,np.uint8)
        for x in (900,928):cv2.rectangle(image,(x,500),(x+13,513),(80,110,65),-1)
        cv2.rectangle(image,(914,505),(927,507),(80,110,65),-1)
        full=marker._candidates(image,'teal',dict(marker.detector_policy(),minimumSolidity=0.));self.assertEqual(len(full),1);self.assertLess(full[0]['solidity'],.7)
        self.assertEqual(bridge._teal_candidates(image,marker.detector_policy()),[])
    def test_large_junk_core_fraction_and_missing_core_rejected(self):
        for main_square in (True,False):
            image=np.full((1080,1920,3),180,np.uint8)
            for y in (500,508,516):cv2.rectangle(image,(900,y),(929,y+2),(80,110,65),-1)
            cv2.rectangle(image,(900,500),(902,519),(80,110,65),-1)
            if main_square:cv2.rectangle(image,(900,500),(909,509),(80,110,65),-1)
            full=marker._candidates(image,'teal',dict(marker.detector_policy(),minimumSolidity=0.));self.assertEqual(len(full),1);self.assertLess(full[0]['solidity'],.7)
            with self.subTest(main_square=main_square):self.assertEqual(bridge._teal_candidates(image,marker.detector_policy()),[])
    def test_core_area_solidity_or_fraction_metadata_cannot_be_relaxed(self):
        original=self.projector.observe(self.fallback_raw(),195.29)
        for mode in ('area','solidity','fraction','count','mapping'):
            changed=copy.deepcopy(original);v=changed['quality']['tealValidation']
            if mode=='area':v['core']['area']=79;v['core']['fillFraction']=79/np.prod(v['core']['bbox'][2:]);v['coreFullAreaFraction']=79/changed['markerPair']['teal']['area']
            elif mode=='solidity':v['core']['solidity']=.69
            elif mode=='fraction':v['coreFullAreaFraction']=.49
            elif mode=='count':v['eligibleCoreCount']=2
            else:v['corePixelsInsideFull']=False
            with self.subTest(mode=mode),self.assertRaises(ValueError):bridge.observation(self.projector,changed)
    def test_purple_pair_ambiguity_and_dark_support_remain_strict(self):
        for mode in ('purple','duplicate','dark'):
            image=self.fixture.image.copy()
            if mode=='purple':image[370:400,1065:1100]=30
            elif mode=='duplicate':cv2.ellipse(image,(1125,410),(11,8),0,0,360,(110,65,110),-1)
            else:image[330:430,900:990]=180
            with self.subTest(mode=mode),self.assertRaises(ValueError):self.projector.observe(self.encode(image),195.29)
    def test_raw_source_audit_is_required_complete_and_SHA_bound(self):
        current=self.projector.observe(self.fixture.raw(),195.29)
        for audit in (None,{},dict(self.audit,auditSha256='a'*64)):
            with self.subTest(auditType=type(audit).__name__),self.assertRaises(ValueError):bridge.predict(self.projector,audit,current,direct.COMMANDS[0])
        with self.assertRaises(ValueError):bridge.audit_sources(self.projector,self.sources[:2])
        changed=copy.deepcopy(self.sources);changed[0]['before']+=b' '
        with self.assertRaisesRegex(ValueError,'raw_SHA_mismatch'):bridge.audit_sources(self.projector,changed)
        changed=copy.deepcopy(self.sources);changed[2]['sessionId']='s0'
        with self.assertRaisesRegex(ValueError,'unique_source'):bridge.audit_sources(self.projector,changed)
    def test_resealed_audit_coordinates_images_rows_or_delta_mismatch_rejected(self):
        for mode in ('state','image','delta','row','missing'):
            audit=copy.deepcopy(self.audit)
            if mode=='state':audit['sources'][0]['phases']['before']['newState'][0]+=1
            elif mode=='image':audit['sources'][0]['phases']['before']['imageSha256']='a'*64
            elif mode=='delta':audit['sources'][0]['newDelta'][0]+=1
            elif mode=='row':audit['sources'][0]['sourceSha256']='f'*64
            else:audit['sources'].pop()
            self.reseal(audit)
            with self.subTest(mode=mode),self.assertRaises(ValueError):bridge.validate_audit(self.projector,audit)
    def test_projector_model_projection_or_observer_tampering_rejected(self):
        for mode in ('model','model_file_SHA','projection','observer'):
            p=bridge.CoreBridgeProjector(self.raw_model,self.fixture.seed)
            if mode=='model':p.model['controlEligible']=True
            elif mode=='model_file_SHA':p.original_model_file_sha256='a'*64
            elif mode=='projection':p.registration['basisCoefficients'][0]+=1
            else:p.contract['observerPolicy']['minimumCoreFullAreaFraction']=.4
            with self.subTest(mode=mode),self.assertRaisesRegex(ValueError,'projector_changed'):p.observe(self.fixture.raw(),195.29)
    def test_observation_contract_geometry_or_raw_identity_tampering_rejected(self):
        original=self.projector.observe(self.fixture.raw(),195.29)
        for mode in ('model','projection','observer','state','pair','dark','image'):
            value=copy.deepcopy(original)
            if mode=='model':value['originalModelSha256']='a'*64
            elif mode=='projection':value['originalProjectionContractSha256']='a'*64
            elif mode=='observer':value['newObserverSha256']='a'*64
            elif mode=='state':value['state'][0]+=1
            elif mode=='pair':value['markerPair']['teal']['center'][0]+=1
            elif mode=='dark':value['blackOccupancyFraction']=.149
            else:value['imageSha256']='not-a-sha'
            with self.subTest(mode=mode),self.assertRaises(ValueError):bridge.observation(self.projector,value)
    def test_selection_and_budget_use_original_commands_weights_and_support(self):
        raw=self.fixture.raw();current=self.projector.observe(raw,195.29)
        old=direct.select(self.model,self.fixture.obs(),[942.,383.],candidates=[direct.COMMANDS[0]])
        new=bridge.select(self.projector,self.audit,current,[942.,383.],candidates=[direct.COMMANDS[0]])
        self.assertEqual(old['selected']['command'],new['selected']['command']);self.assertEqual(old['selected']['predictedState'],new['selected']['predictedState'])
        exhausted=bridge.select(self.projector,self.audit,current,[942.,383.],commands_used=1)
        self.assertEqual(exhausted['selectionOwner'],'budget_guard');self.assertFalse(exhausted['modelSelectedStop']);self.assertEqual(len(exhausted['candidates']),1)
        outside=self.projector.observe(raw,212.)
        with self.assertRaisesRegex(ValueError,'declared_support'):bridge.predict(self.projector,self.audit,outside,direct.COMMANDS[0])
        with self.assertRaises(ValueError):bridge.select(self.projector,None,current,[942.,383.],commands_used=1)
    def test_real_six_sources_and_four_current_frames_when_available(self):
        root=Path(__file__).resolve().parents[1];audit_path=root/'.local/current-entry-20261118/marker-core-bridge-01/source-audit.json'
        if not audit_path.exists():self.skipTest('private source audit unavailable')
        raw=(root/'.local/current-entry-20261117/m3-down-source-audit-01/direct-tool-model-03/frozen-direct-M3-tool-response.json').read_bytes()
        seed=(root/'.local/current-entry-20261114/L-pose-marker-virtual-observer-20261115-01/reference.png').read_bytes();p=bridge.CoreBridgeProjector(raw,seed)
        audit=json.loads(audit_path.read_bytes());bridge.validate_audit(p,audit);raws={}
        for ref in audit['sourceReadPaths']:
            path=root/ref['path'];self.assertTrue(path.resolve().is_relative_to(root));data=path.read_bytes();self.assertEqual(hashlib.sha256(data).hexdigest(),ref['imageSha256']);raws.setdefault(ref['sessionId'],{})[ref['phase']]=data
        rebuilt=bridge.audit_sources(p,[dict(sessionId=s,**phases) for s,phases in raws.items()]);self.assertEqual(rebuilt['sources'],audit['sources'])
        for source in rebuilt['sources']:
            self.assertEqual(source['originalDelta'],source['newDelta'])
            for phase in ('before','after'):self.assertEqual(source['phases'][phase]['originalState'],source['phases'][phase]['newState'])
        for name in ('marker-readonly-01/current-external-1.png','marker-readonly-01/current-external-2.png','lower-model-11/physical-01/before-external.png','lower-model-10/physical-01/after-external.png'):
            value=p.observe((root/'.local/current-entry-20261118'/name).read_bytes(),191.77);bridge.observation(p,value)
            self.assertEqual(value['quality']['eligiblePairs'],1)
            if name.startswith('lower-model-10'):self.assertEqual(value['quality']['tealValidation']['mode'],'original_strict_solidity')
            else:self.assertEqual(value['quality']['tealValidation']['mode'],'core_quality_original_geometry');self.assertLess(value['markerPair']['teal']['solidity'],.7)

if __name__=='__main__':unittest.main()
