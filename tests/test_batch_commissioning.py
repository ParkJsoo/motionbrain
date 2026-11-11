"""실제 파일 감사·양카메라 재관측·모델 계약 연결. 압축/시계 OS 경계만 대체."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
from learning.batch_commissioning import build_transition, load_learning_transition
from learning.batch_runtime import _load_model, sha
from learning.batch_learning import fit
from learning.batch_observation import SCHEMA
from learning.experiment_pipeline import encoded
from tests.test_experiment_pipeline import fixture
from tests import test_batch_gripper_points as gripper_fixture


class CommissioningTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.repository=Path(self.tmp.name);self.local=self.repository/'.local';self.local.mkdir()
        f=gripper_fixture.BatchGripperPointsTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        for name in ('points.json','reference.jpg','gripper-points.json'):
            (self.local/name).write_bytes((f.f.private/name).read_bytes())
        self.image=np.zeros((32,32,4),np.uint8);markers={}
        for name,hue,x,y in [('a',20,5,5),('b',50,14,5),('c',80,23,5),('d',110,5,12),('e',140,25,12)]:
            color=cv2.cvtColor(np.uint8([[[hue,255,255]]]),cv2.COLOR_HSV2BGR)[0,0]
            cv2.circle(self.image,(x,y),2,tuple(int(v) for v in color)+(255,),-1)
            markers[name]=dict(hsvLower=[hue,200,200],hsvUpper=[hue,255,255],minAreaPx=3,maxAreaPx=20,
                uncertaintyPx=.2,role='fixed_view_anchor' if name in 'de' else 'pose')
        fields=[dict(name=n+axis,kind='relative_'+axis,a=n,b='a') for n in ('b','c') for axis in ('x','y')]
        self.spec=dict(schemaVersion=SCHEMA,cameraIdentity='synthetic-device',imageShape=[32,32],
            viewId='fixed-view',poseRegime='local',object='none',load='empty',maxAgeSeconds=10,
            maxLandmarkUncertaintyPx=1,landmarks=markers,poseLandmarks=['a','b','c'],
            fixedViewAnchors=dict(d=[3,7,10,14],e=[23,27,10,14]),
            joints=dict(gripper=dict(stateFields=fields,nodes=[dict(id='g',bounds={v['name']:[-30,30] for v in fields})])))
        (self.local/'external.json').write_bytes(encoded(self.spec))
        wrapper=json.loads((self.local/'gripper-points.json').read_bytes())
        wrapper['context']={k:self.spec[k] for k in ('viewId','poseRegime','object','load')}
        (self.local/'gripper-points.json').write_bytes(encoded(wrapper))
        self.binding=dict(observerSpecPath='.local/external.json',observerSpecSha256=sha(encoded(self.spec)),
            gripper=dict(url='http://camera.invalid',bootId='a'*32,required=True,
                observerSpecPath='.local/gripper-points.json',observerSpecSha256=sha(encoded(wrapper))))
        def clock(capture, hashes):
            sid=json.loads((capture.parent/'plan.json').read_bytes())['sessionId']
            return dict(basis='before_after_same_mach_clock',hostClockId=sid,validPythonBounds=[0.,100.],initialOffsets=[-.000001,.000001])
        for target,replacement in [('learning.manual_collection_dataset.clock_evidence',clock),
                                   ('learning.batch_runtime.decode',lambda raw,length,digest:raw)]:
            p=patch(target,side_effect=replacement);p.start();self.addCleanup(p.stop)

    def make(self,sid):
        source=self.local/sid
        original_zeros=np.zeros
        count=0
        def pixels(shape,*args,**kwargs):
            nonlocal count
            if shape!=(32,32,4): return original_zeros(shape,*args,**kwargs)
            image=self.image.copy();count+=1
            if getattr(self,'moving',False) and count>30:
                for x in (14,23):
                    patch=image[3:8,x-2:x+3].copy();image[3:8,x-2:x+3]=0
                    image[3:8,x:x+5]=patch
            return image
        with patch('tests.test_experiment_pipeline.np.zeros',side_effect=pixels):
            digest=fixture(source,sid,0,'train',validation_scope=getattr(self,'validation_scope',None))
        session_path=source/'session.json';session=json.loads(session_path.read_bytes())
        session['poseContext']='local';session_path.write_bytes(encoded(session));digest=sha(session_path.read_bytes())
        from learning.pipeline_evidence import ingest_session
        audit=ingest_session(source,digest);row=audit['visual']['row']
        folder=source/'gripper-pair';folder.mkdir()
        for side in ('before','after'):
            meta=json.loads((self.f.source/'gripper-pair'/('esp32-'+side+'.json')).read_bytes())
            meta.update(sessionId=sid,hostClockId=sid)
            at=row['evidence']['commandAt']-1 if side=='before' else row['evidence']['offReceiptAt']+3.1
            meta.update(requestStartedAt=at,receivedAt=at+.1,persistedAt=at+.2)
            # Distinct original frame identities for separate sessions.
            number=(100 if side=='before' else 200)+1000*ord(sid)
            meta['headers'].update({'X-MotionBrain-Captured-Us':str(number),
                'X-MotionBrain-Capture-Requested-Us':str(number-1),'X-MotionBrain-Frame-Id':'a'*32+':'+str(number)})
            (folder/('esp32-'+side+'.json')).write_bytes(encoded(meta))
            (folder/('esp32-'+side+'.jpg')).write_bytes(self.f.raw)
        result=build_transition(self.repository,source,digest,self.binding,'train')
        self.assertEqual(result['status'],'observed',result.get('reason'))
        path=self.local/(sid+'.json');path.write_bytes(encoded(result))
        entry=dict(sourceKind='commissioning_fused',transitionPath=str(path),transitionSha256=sha(path.read_bytes()),
            sourcePath=str(source),sourceSha256=digest)
        return result,entry

    def test_raw_files_reaudit_fit_and_production_model_contract(self):
        rows=[]
        for sid in ('a','b'):
            result,entry=self.make(sid)
            row,proof=load_learning_transition(self.repository,entry,'training')
            self.assertFalse(proof['prospectiveEvidenceVerified']);rows.append(row)
            self.assertNotIn('commandAt',row)
        model=fit(rows);path=self.local/'model.json';path.write_bytes(encoded(model))
        config=dict(self.binding,frozenModels=dict(gripper=dict(path=str(path),sha256=sha(path.read_bytes()))))
        self.assertEqual(_load_model(self.repository,config,self.spec,'gripper'),model)
        self.assertEqual(len(model['stateFields']),7)
        self.assertFalse(model['controlEligible'])

    def test_jaw_only_registration_uses_same_fused_model_contract(self):
        from learning.registered_gripper_landmarks import register_jaws
        registration=self.f.registration
        jaws=register_jaws(self.f.raw,{k:v for k,v in registration['points'].items() if k!='object'},registration['parameters'])
        (self.local/'points.json').write_bytes(encoded(jaws))
        path=self.local/'gripper-points.json';wrapper=json.loads(path.read_bytes())
        wrapper.update(kind='registered_gripper_jaws_stationary',registrationSha256=sha(encoded(jaws)))
        path.write_bytes(encoded(wrapper));self.binding['gripper']['observerSpecSha256']=sha(path.read_bytes())
        rows=[self.make(sid)[0]['learningRow'] for sid in ('a','b')]
        model=fit(rows);path=self.local/'jaws-model.json';path.write_bytes(encoded(model))
        config=dict(self.binding,frozenModels=dict(gripper=dict(path=str(path),sha256=sha(path.read_bytes()))))
        self.assertEqual(_load_model(self.repository,config,self.spec,'gripper'),model)
        self.assertEqual(len(model['stateFields']),9)

    def test_cli_manifest_reaudits_before_training_and_development_evaluation(self):
        from tools.train_batch_campaign import run_manifest
        entries=[self.make(sid)[1] for sid in ('a','b','c')]
        evaluation=entries[-1];path=Path(evaluation['transitionPath'])
        result=build_transition(self.repository,evaluation['sourcePath'],evaluation['sourceSha256'],
                                self.binding,'holdout','evaluation')
        path.write_bytes(encoded(result));evaluation['transitionSha256']=sha(path.read_bytes())
        manifest=dict(schemaVersion='motionbrain.batch_training_manifest.v1',training=entries[:2],evaluation=[evaluation])
        path=self.local/'manifest.json';path.write_bytes(encoded(manifest))
        report=run_manifest(path,self.local/'trained',repository=self.repository)
        self.assertEqual((report['trainingRows'],report['evaluationRows']),(2,1))
        self.assertEqual(report['models'][0]['evaluation']['evaluated'],1)
        self.assertEqual(report['models'][0]['evaluation']['kind'],'developmentReplay')
        self.assertFalse(report['prospectiveEvidenceVerified'])

    def test_tampered_row_registration_and_split_are_rejected(self):
        result,entry=self.make('a')
        with self.assertRaisesRegex(ValueError,'schema_or_split'):
            load_learning_transition(self.repository,entry,'evaluation')
        changed=copy.deepcopy(self.spec);changed['poseRegime']='other-pose'
        (self.local/'external.json').write_bytes(encoded(changed))
        binding=copy.deepcopy(self.binding);binding['observerSpecSha256']=sha(encoded(changed))
        with self.assertRaisesRegex(ValueError,'pose_context_changed'):
            build_transition(self.repository,entry['sourcePath'],entry['sourceSha256'],binding,'train')
        (self.local/'external.json').write_bytes(encoded(self.spec))
        result['learningRow']['after'][0]+=1
        path=Path(entry['transitionPath']);path.write_bytes(encoded(result));entry['transitionSha256']=sha(path.read_bytes())
        with self.assertRaisesRegex(ValueError,'transition_changed'):
            load_learning_transition(self.repository,entry,'training')
        (self.local/'reference.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'hash_changed'):
            build_transition(self.repository,entry['sourcePath'],entry['sourceSha256'],self.binding,'train')

    def test_observer_failure_retains_attempt_without_learning_row(self):
        result,entry=self.make('a')
        (Path(entry['sourcePath'])/'gripper-pair/esp32-after.jpg').write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError,'image_hash'):
            build_transition(self.repository,entry['sourcePath'],entry['sourceSha256'],self.binding,'train')
        # An intact source with an unsupported fixed-view anchor is a rejected attempt.
        self.spec['fixedViewAnchors']['d']=[0,1,0,1]
        (self.local/'external.json').write_bytes(encoded(self.spec))
        self.binding['observerSpecSha256']=sha(encoded(self.spec))
        source=Path(entry['sourcePath']);(source/'gripper-pair/esp32-after.jpg').write_bytes(self.f.raw)
        rejected=build_transition(self.repository,source,entry['sourceSha256'],self.binding,'train')
        self.assertEqual(rejected['status'],'rejected');self.assertNotIn('learningRow',rejected)
        self.assertIn('before',rejected['observations'])
