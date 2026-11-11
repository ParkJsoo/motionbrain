"""합성 파일 근거로 제한된 추적 승인 계약을 검증한다. 실물 승인 아님."""
import copy
import json
import unittest
from pathlib import Path
from unittest.mock import patch

from tests import test_batch_commissioning as fixture
from learning.batch_commissioning import build_transition
from learning.batch_motion_admission import build_admission, verify_admission, require_scope
from learning.batch_runtime import sha
from learning.experiment_pipeline import encoded


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.f=fixture.CommissioningTests();self.f.setUp();self.addCleanup(self.f.doCleanups)
        f=self.f;f.moving=True
        development=f.make('a')[1]
        scope=dict(schemaVersion='motionbrain.motion_validation_scope.v1',development=[development],
            observerBinding=f.binding,maxLabelTolerancePx=.2)
        scope_path=f.local/'scope.json';scope_path.write_bytes(encoded(scope))
        f.validation_scope=dict(path=str(scope_path),sha256=sha(scope_path.read_bytes()))
        entries=[development]+[f.make(sid)[1] for sid in ('b','c')]
        self.manifest=dict(development=entries[:1],holdout=[],validationScope=f.validation_scope)
        self.rows=[]
        for i,entry in enumerate(entries[1:]):
            result=build_transition(f.repository,entry['sourcePath'],entry['sourceSha256'],f.binding,'holdout','evaluation')
            path=Path(entry['transitionPath']);path.write_bytes(encoded(result));entry['transitionSha256']=sha(path.read_bytes())
            row=result['learningRow'];self.rows.append(row)
            label={k:row[k] for k in ('sourceHash','frameIds','pairedFrameIds','stateFields','before','after')}
            label.update(method='manual_projected_state_review',reviewer='synthetic-test-reviewer',
                         tolerancePx=.2,visible=True,identityVerified=True)
            path=f.local/('labels-'+str(i)+'.json');path.write_bytes(encoded(label))
            self.manifest['holdout'].append(dict(transition=entry,labels=dict(path=str(path),sha256=sha(path.read_bytes()))))

    def test_reaudited_evidence_and_runtime_scope(self):
        f=self.f
        artifact=build_admission(f.repository,self.manifest)
        path=f.local/'admission.json';path.write_bytes(encoded(artifact))
        binding=dict(path=str(path),sha256=sha(path.read_bytes()))
        self.assertEqual(verify_admission(f.repository,binding),artifact)
        row=self.rows[0]
        require_scope(artifact,row['command'],row['context'],row['stateFields'],row['before'])
        with self.assertRaisesRegex(ValueError,'outside_verified_range'):
            require_scope(artifact,row['command'],row['context'],row['stateFields'],[v+100 for v in row['before']])
        changed=dict(row['command'],percent=99)
        with self.assertRaisesRegex(ValueError,'command_not_supported'):
            require_scope(artifact,changed,row['context'],row['stateFields'])
        self.assertFalse(artifact['executionAuthorized']);self.assertFalse(artifact['measurementAccuracyVerified'])
        from learning.batch_runtime import validate_runtime, BatchRuntime
        plan=copy.deepcopy(f.f.f.plan);config=plan['runtime']
        (f.local/'helper').write_text('test');(f.local/'helper').chmod(0o700)
        config.update(f.binding,deviceId='synthetic-device',poseContext='local',motionAdmission=binding)
        plan['viewId']='fixed-view';plan['nodes']=['g']
        plan['conditions']=[dict(id='close',command=row['command'],fromNodes=['g'],phase='training',
            captureGroup='next',target=1,maxAttempts=1)]
        with patch('learning.batch_runtime.SequencePort',side_effect=AssertionError('hardware')):
            checked=validate_runtime(plan,f.repository)
        self.assertEqual(checked['motionAdmission'],artifact)
        runtime=object.__new__(BatchRuntime);runtime.repository=f.repository;runtime.config=config
        runtime.spec=f.spec;runtime.gripper_spec=f.f.runtime.gripper_spec
        from learning.batch_runtime import _gripper_spec
        runtime.gripper_spec=_gripper_spec(f.repository,config,f.spec);runtime.motion_admission=artifact
        # Before-dispatch check never reruns expensive video/source review.
        with patch('learning.batch_motion_admission.verify_admission',side_effect=AssertionError('slow reaudit')):
            runtime._admission_scope(row['command'],row['stateFields'],row['before'])

        (Path(self.manifest['holdout'][0]['labels']['path'])).write_bytes(b'{}')
        with self.assertRaisesRegex(ValueError,'hash_changed'):verify_admission(f.repository,binding)

    def test_overlap_and_false_or_stationary_labels_cannot_unlock(self):
        f=self.f
        changed=copy.deepcopy(self.manifest)
        changed['development'].append(changed['holdout'][0]['transition'])
        # Wrong split is rejected even before identity checks.
        with self.assertRaises(ValueError):build_admission(f.repository,changed)
        for side in ('before','after'):
            item=self.manifest['holdout'][0];path=Path(item['labels']['path']);labels=json.loads(path.read_bytes())
            labels[side]=[v+20 for v in labels[side]];path.write_bytes(encoded(labels));item['labels']['sha256']=sha(path.read_bytes())
            with self.assertRaisesRegex(ValueError,'disagreement'):build_admission(f.repository,self.manifest)
            labels[side]=[v-20 for v in labels[side]];path.write_bytes(encoded(labels));item['labels']['sha256']=sha(path.read_bytes())

    def test_old_posthoc_sources_cannot_be_relabelled_as_holdout(self):
        item=self.manifest['holdout'][0];source=Path(item['transition']['sourcePath'])
        (source/'commissioning-validation-scope.json').unlink()
        with self.assertRaises(FileNotFoundError):build_admission(self.f.repository,self.manifest)

    def test_real_manual_pulse_seals_scope_and_tamper_blocks_dispatch(self):
        from tests import test_manual_collection as manual_fixture
        from learning.batch_motion_admission import install_validation_scope
        for tamper in (False,True):
            manual=manual_fixture.ManualCollectionTests();manual.setUp();self.addCleanup(manual.tearDown)
            manual.start();manager=manual.manager
            install_validation_scope(manager,self.f.repository,self.manifest['validationScope'])
            root=manager.store.root;port=manager.port
            if tamper:
                (root/'commissioning-validation-scope.json').write_bytes(b'changed')
                with self.assertRaisesRegex(ValueError,'scope_changed'):manual.pulse()
                self.assertEqual(port.executions,[])
            else:
                manual.clock.sleep(.01);manual.pulse()
                intent=json.loads((root/'command-001-intent.json').read_bytes())
                self.assertEqual(intent['validationScopeSha256'],sha((root/'commissioning-validation-scope.json').read_bytes()))
                self.assertEqual(len(port.executions),1)
                with self.assertRaisesRegex(ValueError,'must_precede_command'):
                    install_validation_scope(manager,self.f.repository,self.manifest['validationScope'])

    def test_missing_and_fabricated_admission_remain_fail_closed(self):
        from learning.batch_runtime import validate_runtime
        f=self.f;plan=copy.deepcopy(f.f.f.plan)
        plan['runtime']=copy.deepcopy(f.f.runtime.config)
        with self.assertRaisesRegex(ValueError,'stationary_only'):validate_runtime(plan,f.f.f.repository)
        path=f.f.f.private/'fake-admission.json';path.write_bytes(encoded(dict(schemaVersion='motionbrain.batch_motion_admission.v1',manifest={})))
        plan['runtime']['motionAdmission']=dict(path=str(path),sha256=sha(path.read_bytes()))
        with self.assertRaisesRegex(ValueError,'admission_manifest'):validate_runtime(plan,f.f.f.repository)
