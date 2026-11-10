"""원본 재감사·전이 binding·오프라인 학습 CLI 검증. 실제 장치 접근 없음."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools import train_batch_campaign as cli
from tests.test_experiment_pipeline import fixture
from learning.pipeline_evidence import ingest_session


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


class BatchTrainingCliTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.local = self.root/'.local'; self.local.mkdir()
        def clock(capture, hashes):
            sid = json.loads((capture.parent/'plan.json').read_text())['sessionId']
            return dict(basis='before_after_same_mach_clock', hostClockId=sid,
                        validPythonBounds=[0.,100.], initialOffsets=[-.000001,.000001])
        mocked = patch('learning.manual_collection_dataset.clock_evidence', side_effect=clock)
        mocked.start(); self.addCleanup(mocked.stop)
        self.manifest = dict(schemaVersion=cli.SCHEMA, training=[], evaluation=[])
        for sid, x, group, split in [('a',0,'train','training'), ('b',10,'train','training'),
                                      ('c',5,'eval','evaluation')]:
            self.manifest[split].append(self.transition(sid,x,group,split))
        self.path = self.local/'manifest.json'

    def transition(self, sid, x, group, split, *, failed=False):
        source = self.local/sid
        source_sha = fixture(source, sid, x, group, failed=failed)
        audit = ingest_session(source, source_sha, allow_failed_development=failed)
        original = audit['visual']['row']; command = audit['execution']['command']
        context = dict(viewId='fixed-view',poseRegime='local',object='synthetic',load='empty')
        fields = ['x','gap']; before = [16.+x,16.]; after = [17.+x,16.]
        session = json.loads((source/'session.json').read_bytes())
        pinned_frame = session['frames'][original['evidence']['preFrameIds'][-2]]
        self.assertNotEqual(pinned_frame['frameId'],original['inputs']['image']['frameId'])
        observed = {}
        for side, ref, state in [('before',pinned_frame,before),
                                  ('after',original['targets']['image'],after)]:
            metadata = dict(frameId=ref['frameId'], capturedAt=ref['sourceEarliest'],
                            persistedAt=ref['stdoutReadAt']+ref['persistenceMs']/1000)
            observed[side] = dict(source=dict(folder=str(source/'native-camera'),frame=ref),
                derivedMetadata=metadata, observation=dict(status='accepted', specSha256='a'*64,
                    frameBinding=metadata,states={'gripper':dict(state=state,stateFields=fields)}))
        row = dict(sessionId=sid,episodeId=':'.join(audit['execution']['episode']),
            sourceHash=source_sha,frameIds=[pinned_frame['frameId'],original['targets']['image']['frameId']],
            captureGroup=group,joint='gripper',context={**context,'observerSha256':'a'*64},
            stateFields=fields,before=before,after=after,command=command,
            commandAt=original['evidence']['commandAt'],beforePersistedAt=observed['before']['derivedMetadata']['persistedAt'])
        transition = dict(schemaVersion='motionbrain.batch_visual_transition.v1',phase=split,
            sourceSha256=source_sha,learningRow=row,joint='gripper',context=context,
            beforeState=before,afterState=after,stateFields=fields,command=command,
            sessionId=sid,episodeId=audit['execution']['episode'],frameIds=row['frameIds'],
            captureGroup=group,selectionSemantics='pinned_pre_window_frame_not_default_audit_input',**observed)
        (source/'batch-before-observation.json').write_bytes(cli.canonical(observed['before']))
        path = self.local/(sid+'-transition.json'); path.write_bytes(cli.canonical(transition))
        return dict(transitionPath=str(path),transitionSha256=sha(path.read_bytes()),
                    sourcePath=str(source),sourceSha256=source_sha)

    def run_cli(self, manifest=None, output='result'):
        self.path.write_text(json.dumps(self.manifest if manifest is None else manifest))
        return cli.run_manifest(self.path,self.local/output,repository=self.root)

    def mutate(self, entry, change):
        path = Path(entry['transitionPath']); value = json.loads(path.read_bytes())
        change(value); path.write_bytes(cli.canonical(value)); entry['transitionSha256'] = sha(path.read_bytes())

    def test_real_raw_audit_fit_and_same_case_baselines(self):
        before = {p:p.read_bytes() for p in self.local.rglob('*') if p.is_file()}
        result = self.run_cli()
        self.assertEqual((result['trainingRows'],result['evaluationRows'],len(result['models'])),(2,1,1))
        model = result['models'][0]
        evaluation = model['evaluation']
        self.assertEqual((result['kind'],evaluation['kind']),('developmentReplay','developmentReplay'))
        self.assertEqual(evaluation['evaluated'],1)
        self.assertEqual(set(evaluation['metrics']),{'stateRidge','commandMean','noChange'})
        self.assertFalse(result['deviceAccess']); self.assertFalse(result['credentialAccess'])
        self.assertEqual(result['motorCommands'],0); self.assertFalse(result['prospectiveEvidenceVerified'])
        raw = (self.local/'result'/model['path']).read_bytes()
        self.assertEqual(sha(raw),model['sha256'])
        for path, original in before.items(): self.assertEqual(path.read_bytes(),original)
        latest = json.loads((self.local/'result/latest.json').read_bytes())
        self.assertEqual(sha((self.local/'result/report.json').read_bytes()),latest['sha256'])

    def test_existing_output_never_overwritten(self):
        self.run_cli(); sentinel = self.local/'result/report.json'; original = sentinel.read_bytes()
        with self.assertRaisesRegex(ValueError,'new_output'): self.run_cli()
        self.assertEqual(sentinel.read_bytes(),original)

    def test_original_raw_tamper_rejected_before_output(self):
        p = Path(self.manifest['training'][0]['sourcePath'])/'session.json'
        p.write_bytes(p.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError,'session_hash_mismatch'): self.run_cli()
        self.assertFalse((self.local/'result').exists())

    def test_transition_raw_hash_tamper_rejected(self):
        p = Path(self.manifest['training'][0]['transitionPath']); p.write_bytes(p.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError,'artifact_hash_changed'): self.run_cli()

    def test_learning_row_and_original_command_bindings_rejected(self):
        self.mutate(self.manifest['training'][0],lambda t:t['learningRow']['before'].__setitem__(0,999))
        with self.assertRaisesRegex(ValueError,'learning_row_changed'): self.run_cli()

    def test_embedded_frame_must_be_original_audited_frame(self):
        self.mutate(self.manifest['training'][0],lambda t:t['before']['source']['frame'].__setitem__('frameId','other'))
        with self.assertRaisesRegex(ValueError,'pinned_input_changed'): self.run_cli()

    def test_pinned_frame_outside_pre_window_is_rejected(self):
        entry = self.manifest['training'][0]
        source = Path(entry['sourcePath'])
        session = json.loads((source/'session.json').read_bytes())
        def change(t):
            t['before']['source']['frame'] = session['frames'][-1]
            (source/'batch-before-observation.json').write_bytes(cli.canonical(t['before']))
        self.mutate(entry,change)
        with self.assertRaisesRegex(ValueError,'pinned_input_not_in_original_window'): self.run_cli()

    def test_persistence_completion_not_stdout_receipt_time(self):
        entry = self.manifest['training'][0]
        source = Path(entry['sourcePath'])
        def change(t):
            before = t['before']
            wrong = before['source']['frame']['stdoutReadAt']
            before['derivedMetadata']['persistedAt'] = wrong
            before['observation']['frameBinding']['persistedAt'] = wrong
            t['learningRow']['beforePersistedAt'] = wrong
            (source/'batch-before-observation.json').write_bytes(cli.canonical(before))
        self.mutate(entry,change)
        with self.assertRaisesRegex(ValueError,'frame_binding_changed'): self.run_cli()

    def test_duplicate_source_cannot_inflate_training_count(self):
        self.manifest['training'].append(copy.deepcopy(self.manifest['training'][0]))
        with self.assertRaisesRegex(ValueError,'identity_overlap'): self.run_cli()

    def test_train_evaluation_capture_group_overlap_rejected(self):
        def change(t): t['captureGroup']='train';t['learningRow']['captureGroup']='train'
        self.mutate(self.manifest['evaluation'][0],change)
        with self.assertRaisesRegex(ValueError,'capture_group_overlap'): self.run_cli()

    def test_preparation_and_failed_original_never_become_training(self):
        self.mutate(self.manifest['training'][0],lambda t:t.__setitem__('phase','preparation'))
        with self.assertRaisesRegex(ValueError,'split_changed'): self.run_cli()
        self.manifest['training'][0] = self.transition('failed',1,'train','training',failed=True)
        with self.assertRaisesRegex(ValueError,'original_source_not_eligible'): self.run_cli()

    def test_model_can_be_loaded_frozen_without_refitting(self):
        report = self.run_cli(); entry = report['models'][0]
        frozen = dict(schemaVersion=cli.SCHEMA,evaluation=self.manifest['evaluation'],
                      frozenModels=[dict(path=str(self.local/'result'/entry['path']),sha256=entry['sha256'])])
        with patch.object(cli,'fit',side_effect=AssertionError('must_not_refit')):
            result = self.run_cli(frozen,output='evaluation-only')
        self.assertEqual(result['models'][0]['sha256'],entry['sha256'])
        self.assertEqual(result['models'][0]['evaluation']['evaluated'],1)

    def test_frozen_evaluation_rejects_training_raw_reuse(self):
        report = self.run_cli(); entry = report['models'][0]
        reused = copy.deepcopy(self.manifest['training'][0])
        self.mutate(reused,lambda t:t.__setitem__('phase','evaluation'))
        frozen = dict(schemaVersion=cli.SCHEMA,evaluation=[reused],frozenModels=[
            dict(path=str(self.local/'result'/entry['path']),sha256=entry['sha256'])])
        with self.assertRaisesRegex(ValueError,'training_evaluation_overlap'): self.run_cli(frozen,output='reuse')

    def test_context_partitions_not_silently_mixed(self):
        def change(t):
            t['context']['poseRegime']='another';t['learningRow']['context']['poseRegime']='another'
        self.mutate(self.manifest['training'][1],change)
        result = self.run_cli()
        self.assertEqual(len(result['models']),2)
        self.assertTrue(all(m['supportedCommands']==0 for m in result['models']))

    def test_outside_private_paths_rejected(self):
        self.manifest['training'][0]['sourcePath']=str(self.root/'outside')
        with self.assertRaisesRegex(ValueError,'private_path'): self.run_cli()
        with self.assertRaisesRegex(ValueError,'private_path'):
            cli.run_manifest(self.path,self.root/'public-output',repository=self.root)


if __name__ == '__main__': unittest.main()
