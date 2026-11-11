"""합성 원본으로 runtime 전이→공식 CLI의 고정 평가 연결을 검증한다. 장치 없음."""
import copy
import json
import shutil
import unittest
from pathlib import Path
from types import SimpleNamespace
from learning.batch_runtime import BatchRuntime, _gripper_spec, sha
from learning.batch_learning import fit
from learning.batch_fixed_evaluation import freeze_model, prospective_evidence, seal_before
from learning.batch_collection_domain import DOMAIN_SCHEMA, JAW_FIELDS
from learning.experiment_pipeline import encoded
from learning.pipeline_evidence import ingest_session
from tools.train_batch_campaign import canonical, run_manifest
from tests import test_batch_commissioning as commissioning_fixture


class FixedRuntimeTests(unittest.TestCase):
    def setUp(self):
        f=commissioning_fixture.CommissioningTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        from learning.registered_gripper_landmarks import register_jaws
        registration=f.f.registration
        jaws=register_jaws(f.f.raw,{k:v for k,v in registration['points'].items() if k!='object'},registration['parameters'])
        (f.local/'points.json').write_bytes(encoded(jaws))
        path=f.local/'gripper-points.json';wrapper=json.loads(path.read_bytes())
        wrapper.update(kind='registered_gripper_jaws_stationary',registrationSha256=sha(encoded(jaws)))
        path.write_bytes(encoded(wrapper));f.binding['gripper']['observerSpecSha256']=sha(path.read_bytes())
        self.model=fit([f.make(sid)[0]['learningRow'] for sid in ('a','b')])
        model_path=f.local/'model.json';model_path.write_bytes(canonical(self.model))
        self.model_binding=dict(path=str(model_path),sha256=sha(model_path.read_bytes()))
        self.freeze_binding=freeze_model(f.repository,self.model_binding,f.local/'freeze.json',clock=SimpleNamespace(monotonic=lambda:0.))
        _,entry=f.make('c');source=(f.local/'sessions'/'c').resolve();source.parent.mkdir();shutil.move(entry['sourcePath'],source)
        self.source=source
        audit=ingest_session(source,entry['sourceSha256']);original=audit['visual']['row']
        runtime=object.__new__(BatchRuntime);self.runtime=runtime
        runtime.repository=f.repository.resolve();runtime.root=f.local.resolve();runtime.spec=f.spec;runtime.clock_id='c'
        runtime.config=dict(f.binding,deviceId='synthetic-device',poseContext='local',
            frozenModels=dict(gripper=self.model_binding),fixedCommandEvaluation=self.freeze_binding)
        runtime.plan=dict(viewId='fixed-view');runtime.journal=f.local/'journal.sqlite'
        runtime.gripper_spec=_gripper_spec(f.repository,runtime.config,runtime.spec)
        self.condition=dict(id='fixed',phase='evaluation',captureGroup='future',fromNodes=['g'],command=audit['execution']['command'])
        frame=json.loads((source/'session.json').read_bytes())['frames'][original['evidence']['preFrameIds'][-2]]
        before=runtime._observe_native(source/'native-camera',frame,historical=True)
        (source/'batch-before-observation.json').write_bytes(encoded(before))
        from learning.gripper_camera_dataset import bind_gripper_pair
        before_pair=bind_gripper_pair(copy.deepcopy(original),source/'gripper-pair')
        (source/'batch-gripper-before.json').write_bytes(encoded(dict(frame=before_pair['inputs']['image'])))
        paired=runtime._pair_transition(source,original)
        names,values=runtime._states(before['observation'],paired['beforeObservation'],'gripper')
        jaws_values=[values[names.index(k)] for k in JAW_FIELDS]
        domain=dict(schemaVersion=DOMAIN_SCHEMA,semantics='manual_noncontact_collection_not_model_control',
            jawMinimum=[v-3 for v in jaws_values[:4]],jawMaximum=[v+3 for v in jaws_values[:4]],
            minimumSearchMarginPx=2,maximumGapChangePx=26,
            commands=[dict(command=self.condition['command'],beforeGap=[jaws_values[-1]-3,jaws_values[-1]+3],
                           afterGap=[jaws_values[-1]-3,jaws_values[-1]+3])])
        artifact=dict(domain=domain,contract=dict(joint='gripper',context=runtime._context(),stateFields=names))
        path=f.local/'synthetic-domain.json';path.write_bytes(encoded(artifact))
        runtime.config['collectionAdmission']=dict(path=str(path),sha256=sha(path.read_bytes()))
        runtime.collection_admission=artifact
        frames=[before['derivedMetadata'],paired['beforeObservation']['frameBinding']]
        frames=[{k:v[k] for k in ('frameId','pixelSha256','capturedAt','persistedAt')} for v in frames]
        row=dict(sessionId='c',stateFields=names,before=values,command=self.condition['command'],
                 context=runtime._context(),beforePersistedAt=frames[0]['persistedAt'])
        freeze=json.loads((f.local/'freeze.json').read_bytes())
        prepared=max(v['persistedAt'] for v in frames)+.001
        proof=prospective_evidence(self.model,freeze,row,frames,prepared)
        session=json.loads((source/'session.json').read_bytes());manager=SimpleNamespace(
            current=dict(phase='recording',sessionId='c',events=[]),command_count=0,
            store=SimpleNamespace(root=source,write_json=lambda name,value:(source/name).write_bytes(encoded(value))))
        manager._event=lambda event:manager.current['events'].append(event)
        seal_before(manager,proof)
        session.update(fixedEvaluationProofSha256=manager.current['fixedEvaluationProofSha256'])
        session['events']+=manager.current['events']
        intent=json.loads((source/'command-001-intent.json').read_bytes());intent['fixedEvaluationProofSha256']=session['fixedEvaluationProofSha256']
        session['events']=[intent if event.get('kind')=='command' else event for event in session['events']]
        (source/'command-001-intent.json').write_bytes(encoded(intent))
        (source/'session.json').write_bytes(encoded(session))
        (source/'batch-prospective.json').write_bytes(encoded(dict(proof=proof,beforeFrameId=frame['frameId'],beforePixelSha256=frame['pixelSha256'])))
        self.report=dict(sessionPath=str(source),sourceSha256=sha((source/'session.json').read_bytes()))

    def test_zero_motion_is_counted_stops_followup_and_reaudits_in_official_cli(self):
        result=self.runtime._classify(self.condition,'test',self.report,process=True)
        self.assertEqual(result['observation'],'eligible',result.get('observationFailureCode',result['reason']))
        self.assertEqual(result['followupStopReason'],'collection_domain_delta_rejected')
        self.assertTrue(result['prospectiveEvidenceVerified'])
        path=self.runtime.root/'transitions'/(result['transitionId']+'.json')
        entry=dict(transitionPath=str(path),transitionSha256=sha(path.read_bytes()),sourcePath=str(self.source),sourceSha256=self.report['sourceSha256'])
        manifest=dict(schemaVersion='motionbrain.batch_training_manifest.v1',frozenModels=[self.model_binding],
            evaluation=[entry],fixedCommandEvaluation=self.freeze_binding)
        path=self.f.local/'manifest.json';path.write_bytes(encoded(manifest))
        report=run_manifest(path,self.f.local/'result',repository=self.f.repository)
        self.assertTrue(report['prospectiveEvidenceVerified']);self.assertTrue(report['observationRecomputed'])
        self.assertEqual(report['models'][0]['evaluation']['evaluated'],1)

    def test_changed_domain_is_integrity_failure_not_physical_target(self):
        self.runtime.collection_admission=copy.deepcopy(self.runtime.collection_admission)
        self.runtime.collection_admission['domain']['maximumGapChangePx']=25
        result=self.runtime._classify(self.condition,'test',self.report,process=True)
        self.assertEqual(result['observation'],'rejected')
        self.assertNotIn('followupStopReason',result)

    def test_after_range_violation_remains_evaluation_target(self):
        artifact=copy.deepcopy(self.runtime.collection_admission)
        entry=artifact['domain']['commands'][0]
        entry['afterGap']=[entry['beforeGap'][1]+1,entry['beforeGap'][1]+3]
        binding=self.runtime.config['collectionAdmission']
        Path(binding['path']).write_bytes(encoded(artifact));binding['sha256']=sha(encoded(artifact))
        self.runtime.collection_admission=artifact
        result=self.runtime._classify(self.condition,'test',self.report,process=True)
        self.assertEqual(result['observation'],'eligible',result.get('observationFailureCode'))
        self.assertEqual(result['followupStopReason'],'collection_domain_state_outside_declared_bounds')
        self.assertTrue(result['prospectiveEvidenceVerified'])

    def test_after_integrity_failure_is_not_softened_to_response_violation(self):
        from unittest.mock import patch
        original=self.runtime._admission_scope
        def changed(command,names,values,*,side='before'):
            if side=='after': raise ValueError('admission_artifact_hash_changed')
            return original(command,names,values,side=side)
        with patch.object(self.runtime,'_admission_scope',side_effect=changed):
            result=self.runtime._classify(self.condition,'test',self.report,process=True)
        self.assertEqual(result['observation'],'rejected')
        self.assertEqual(result['observationFailureCode'],'admission_artifact_hash_changed')
        self.assertNotIn('followupStopReason',result)
