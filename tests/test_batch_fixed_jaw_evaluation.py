"""실제 파일 감사 경로의 합성 양턱 고정후 평가. 카메라·모터 접근 없음."""
import copy
import json
import unittest
from types import SimpleNamespace
from learning.batch_fixed_evaluation import freeze_model,prospective_evidence,audit_jaw_proof
from learning.batch_learning import fit
from learning.registered_jaw_transition import build_transition
from learning.gripper_camera_dataset import read_frame
from learning.batch_runtime import sha
from learning.experiment_pipeline import encoded
from tools.train_batch_campaign import canonical,run_manifest
from tests import test_batch_fixed_runtime as fixture

class FixedJawEvaluationTests(unittest.TestCase):
    def setUp(self):
        f=fixture.FixedRuntimeTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        self.repo=f.f.repository;self.local=f.f.local;self.source=f.source
        self.reg_path=self.local/'points.json';self.ref_path=self.local/'reference.jpg'
        self.reg=json.loads(self.reg_path.read_bytes());self.ref=self.ref_path.read_bytes()
        self.context={k:f.f.spec[k] for k in ('viewId','poseRegime','object','load')}
        rows=[]
        for sid in ('a','b'):
            path=self.local/sid
            rows.append(build_transition(path,sha((path/'session.json').read_bytes()),self.reg,self.ref,self.context,'train',phase='training')['learningRow'])
        self.model=fit(rows);path=self.local/'jaw-model.json';path.write_bytes(canonical(self.model))
        self.model_binding=dict(path=str(path),sha256=sha(path.read_bytes()))
        self.freeze_binding=freeze_model(self.repo,self.model_binding,self.local/'jaw-freeze.json',clock=SimpleNamespace(monotonic=lambda:0.))
        self.freeze=json.loads((self.local/'jaw-freeze.json').read_bytes())
        row=build_transition(self.source,sha((self.source/'session.json').read_bytes()),self.reg,self.ref,self.context,'future',phase='evaluation')['learningRow']
        grip,meta,_=read_frame(self.source/'gripper-pair','before','c')
        external=json.loads((self.source/'batch-before-observation.json').read_bytes())['source']['frame']
        frames=[dict(frameId=grip['frameId'],pixelSha256=grip['pixelSha256'],capturedAt=meta['requestStartedAt'],persistedAt=meta['persistedAt']),
                dict(frameId=external['frameId'],pixelSha256=external['pixelSha256'],capturedAt=external['sourceEarliest'],persistedAt=external['stdoutReadAt']+external['persistenceMs']/1000)]
        self.proof=prospective_evidence(self.model,self.freeze,row,frames,max(x['persistedAt'] for x in frames)+.001)
        self.reseal()

    def reseal(self):
        raw=encoded(self.proof);(self.source/'batch-fixed-prospective.json').write_bytes(raw)
        session=json.loads((self.source/'session.json').read_bytes());intent=json.loads((self.source/'command-001-intent.json').read_bytes())
        session['fixedEvaluationProofSha256']=sha(raw);intent['fixedEvaluationProofSha256']=sha(raw)
        session['events']=[intent if e.get('kind')=='command' else e for e in session['events'] if e.get('kind')!='fixed_evaluation_proof']
        session['events'].append(dict(kind='fixed_evaluation_proof',sha256=sha(raw),at=self.proof['preparedAt']))
        (self.source/'command-001-intent.json').write_bytes(encoded(intent));(self.source/'session.json').write_bytes(encoded(session))
        source_sha=sha((self.source/'session.json').read_bytes())
        transition=build_transition(self.source,source_sha,self.reg,self.ref,self.context,'future',phase='evaluation')
        path=self.local/'jaw-evaluation-transition.json';path.write_bytes(encoded(transition))
        self.entry=dict(sourceKind='registered_jaws',sourcePath=str(self.source),sourceSha256=source_sha,
            transitionPath=str(path),transitionSha256=sha(path.read_bytes()),registrationPath=str(self.reg_path),
            registrationSha256=sha(self.reg_path.read_bytes()),referenceImagePath=str(self.ref_path),
            referenceImageSha256=sha(self.ref),captureGroup='future')

    def test_original_jaw_before_and_external_proof_reach_official_cli(self):
        self.assertEqual(audit_jaw_proof(self.repo,self.entry,self.model,self.freeze_binding),self.proof)
        manifest=dict(schemaVersion='motionbrain.batch_training_manifest.v1',frozenModels=[self.model_binding],evaluation=[self.entry],fixedCommandEvaluation=self.freeze_binding)
        path=self.local/'jaw-manifest.json';path.write_bytes(encoded(manifest))
        result=run_manifest(path,self.local/'jaw-evaluation-output',repository=self.repo)
        self.assertTrue(result['prospectiveEvidenceVerified']);self.assertEqual(result['trainingRows'],0)
        evaluated=result['models'][0]['evaluation'];self.assertEqual(evaluated['evaluated'],1)
        for name in ('stateRidge','commandMean','noChange'):self.assertEqual(evaluated['metrics']['all'][name]['n'],1)
        self.assertFalse(evaluated['controlEligible'])

    def test_external_after_frame_cannot_be_used_as_before_even_with_new_seal(self):
        session=json.loads((self.source/'session.json').read_bytes());after=session['frames'][-1]
        self.proof['inputFrames'][1]['frameId']=after['frameId'];self.reseal()
        with self.assertRaisesRegex(ValueError,'original_pre_window'):
            audit_jaw_proof(self.repo,self.entry,self.model,self.freeze_binding)

    def test_resealed_changed_pixels_do_not_pass(self):
        self.proof['inputFrames'][0]['pixelSha256']='f'*64;self.reseal()
        with self.assertRaisesRegex(ValueError,'evidence_changed'):
            audit_jaw_proof(self.repo,self.entry,self.model,self.freeze_binding)

    def test_copied_freeze_or_late_preparation_is_rejected(self):
        self.proof['freeze']['frozenAt']=self.proof['preparedAt']+1;self.reseal()
        with self.assertRaises(ValueError):audit_jaw_proof(self.repo,self.entry,self.model,self.freeze_binding)

    def test_actual_freeze_after_capture_is_not_future_evaluation(self):
        freeze=copy.deepcopy(self.freeze);freeze['frozenAt']=self.proof['inputFrames'][0]['capturedAt']+.01
        path=self.local/'changed-freeze.json';path.write_bytes(encoded(freeze))
        binding=dict(path=str(path),sha256=sha(path.read_bytes()))
        self.proof['freeze']=freeze;self.reseal()
        with self.assertRaisesRegex(ValueError,'new_persisted_inputs'):
            audit_jaw_proof(self.repo,self.entry,self.model,binding)

    def test_prediction_sealed_after_command_is_rejected(self):
        intent=json.loads((self.source/'command-001-intent.json').read_bytes())
        self.proof['preparedAt']=intent['at']+.01;self.reseal()
        with self.assertRaisesRegex(ValueError,'precommand_seal'):
            audit_jaw_proof(self.repo,self.entry,self.model,self.freeze_binding)
