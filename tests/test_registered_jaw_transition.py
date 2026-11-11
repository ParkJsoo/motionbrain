import copy
import json
import unittest
from unittest.mock import patch

from learning.registered_jaw_transition import build_transition
from tests import test_batch_gripper_points as fixture
from learning.batch_learning import fit


class JawTransitionTests(unittest.TestCase):
    def setUp(self):
        f=fixture.BatchGripperPointsTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        self.row=copy.deepcopy(f.row)
        self.row['inputs']['command']=dict(joint='gripper',direction='open',percent=20,durationMs=200)
        self.audit=dict(execution=dict(status='verified',offVerified=True,episode=['boot','one']),recording=dict(status='verified'),
            visual=dict(status='verified',row=self.row),recoveryDevelopmentOnly=False)
        self.context=dict(viewId='fixed-view',poseRegime='fixed',object='none',load='empty')
        self.row['evidence']['cameraIdentity']['viewId']='fixed-view'
        raw=json.dumps(dict(poseContext='fixed')).encode()
        (self.f.source/'session.json').write_bytes(raw)
        from hashlib import sha256
        self.source_sha=sha256(raw).hexdigest()

    def build(self):
        with patch('learning.registered_jaw_transition.ingest_session',return_value=self.audit):
            return build_transition(self.f.source,self.source_sha,self.f.registration,self.f.raw,self.context,'development')

    def test_original_pair_to_learning_preserves_identity_and_single_case_deficit(self):
        result=self.build();row=result['learningRow']
        self.assertEqual(len(row['before']),5)
        self.assertEqual(row['pairedFrameIds'],['a'*32+':100','a'*32+':200'])
        self.assertEqual(row['frameIds'],['external-before','external-after'])
        model=fit([row]);self.assertFalse(model['models']);self.assertTrue(model['deficits'])
        self.assertFalse(result['controlEligible'])
        changed=copy.deepcopy(row);changed['sessionId']='two'
        with self.assertRaisesRegex(ValueError,'duplicate_training_identity'):fit([row,changed])

    def test_original_image_tampering_and_target_time_leakage_fail(self):
        path=self.f.source/'gripper-pair/esp32-before.json';meta=json.loads(path.read_bytes())
        meta['persistedAt']=21.;path.write_text(json.dumps(meta))
        with self.assertRaisesRegex(ValueError,'input_outside_window'):self.build()
        (self.f.source/'gripper-pair/esp32-before.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'image_hash'):self.build()

    def test_unverified_execution_and_other_joint_never_form_rows(self):
        self.audit['execution']['status']='unknown'
        with self.assertRaisesRegex(ValueError,'verified_jaw_source'):self.build()
        self.audit['execution']['status']='verified';self.row['inputs']['command']['joint']='wrist'
        with self.assertRaisesRegex(ValueError,'M1_required'):self.build()

class JawTrainingCliTests(unittest.TestCase):
    setUp = JawTransitionTests.setUp
    build = JawTransitionTests.build
    def prepare(self, phase='training'):
        from tools import train_batch_campaign as cli
        root=self.f.f.repository; local=self.f.f.private
        with patch('learning.registered_jaw_transition.ingest_session',return_value=self.audit):
            value=build_transition(self.f.source,self.source_sha,self.f.registration,self.f.raw,
                                   self.context,'jaw-training',phase=phase)
        path=local/'jaw-transition.json'; path.write_bytes(cli.canonical(value))
        from hashlib import sha256
        self.entry=dict(sourceKind='registered_jaws',transitionPath=str(path),
            transitionSha256=sha256(path.read_bytes()).hexdigest(), sourcePath=str(self.f.source),
            sourceSha256=self.source_sha, registrationPath=str(local/'points.json'),
            registrationSha256=sha256((local/'points.json').read_bytes()).hexdigest(),
            referenceImagePath=str(local/'reference.jpg'),referenceImageSha256=sha256(self.f.raw).hexdigest(),
            captureGroup='jaw-training')
        return cli,root,local,path

    def test_manifest_cli_recomputes_jaws_and_writes_real_model_artifact(self):
        cli,root,local,_=self.prepare()
        manifest=local/'jaw-manifest.json'
        manifest.write_bytes(cli.canonical(dict(schemaVersion=cli.SCHEMA,training=[self.entry],evaluation=[])))
        with patch('learning.registered_jaw_transition.ingest_session',return_value=self.audit) as audit:
            report=cli.run_manifest(manifest,local/'jaw-model',repository=root)
        self.assertEqual(audit.call_count,1)
        self.assertEqual(report['trainingRows'],1)
        self.assertTrue(report['provenance']['training'][0]['observationRecomputed'])
        self.assertEqual(report['models'][0]['supportedCommands'],0)
        self.assertTrue(report['models'][0]['deficits'])
        self.assertFalse(report['controlEligible'])

    def test_rehashing_forged_derived_state_does_not_bypass_recomputation(self):
        cli,root,_,path=self.prepare()
        value=json.loads(path.read_bytes()); value['learningRow']['after'][0]+=1
        path.write_bytes(cli.canonical(value))
        from hashlib import sha256
        self.entry['transitionSha256']=sha256(path.read_bytes()).hexdigest()
        with patch('learning.registered_jaw_transition.ingest_session',return_value=self.audit):
            with self.assertRaisesRegex(ValueError,'recomputed_changed'):
                cli.load_transition(root,self.entry,'training')

    def test_phase_group_reference_registration_and_raw_pair_tampering_rejected(self):
        cli,root,local,_=self.prepare()
        with self.assertRaisesRegex(ValueError,'manifest_binding'):
            cli.load_transition(root,self.entry,'evaluation')
        entry=copy.deepcopy(self.entry);entry['captureGroup']='renamed'
        with self.assertRaisesRegex(ValueError,'manifest_binding'):
            cli.load_transition(root,entry,'training')
        for name in ('points.json','reference.jpg'):
            path=local/name; original=path.read_bytes();path.write_bytes(original+b' ')
            with self.assertRaisesRegex(ValueError,'artifact_hash'):
                cli.load_transition(root,self.entry,'training')
            path.write_bytes(original)
        (self.f.source/'gripper-pair/esp32-after.jpg').write_bytes(b'changed')
        with patch('learning.registered_jaw_transition.ingest_session',return_value=self.audit):
            with self.assertRaisesRegex(ValueError,'image_hash'):
                cli.load_transition(root,self.entry,'training')

    def test_missing_off_and_recovery_sources_rejected(self):
        for key,value in [('offVerified',False),('offVerified',None)]:
            self.audit['execution'][key]=value
            with self.assertRaisesRegex(ValueError,'off_or_recovery'):self.build()
        self.audit['execution']['offVerified']=True
        self.audit['recoveryDevelopmentOnly']=True
        with self.assertRaisesRegex(ValueError,'off_or_recovery'):self.build()

    def test_raw_session_audit_to_cli_without_injected_audit_or_observer(self):
        from tests.test_experiment_pipeline import fixture as raw_fixture
        from learning.pipeline_evidence import ingest_session
        from hashlib import sha256
        cli,root,local,path=self.prepare()
        source=local/'raw-jaw-session'
        raw_fixture(source,'raw-jaw',0,'raw-group')
        session_path=source/'session.json';session=json.loads(session_path.read_bytes())
        session['poseContext']='fixed';session_path.write_bytes(cli.canonical(session))
        source_sha=sha256(session_path.read_bytes()).hexdigest()
        pair=source/'gripper-pair';pair.mkdir()
        for original in (self.f.source/'gripper-pair').iterdir():
            (pair/original.name).write_bytes(original.read_bytes())
        def clock(capture, hashes):
            return dict(basis='before_after_same_mach_clock',hostClockId='raw-jaw',
                        validPythonBounds=[0.,100.],initialOffsets=[-.000001,.000001])
        # Only the OS clock bridge is synthetic; execution audit, pair binding,
        # image matching, manifest validation and model writer run unchanged.
        with patch('learning.manual_collection_dataset.clock_evidence',side_effect=clock):
            audit=ingest_session(source,source_sha)
            evidence=audit['visual']['row']['evidence']
            for side,t in [('before',evidence['commandAt']-.5),
                           ('after',evidence['offReceiptAt']+3.1)]:
                meta_path=pair/('esp32-'+side+'.json');meta=json.loads(meta_path.read_bytes())
                meta.update(sessionId='raw-jaw',hostClockId='raw-jaw',requestStartedAt=t,
                            receivedAt=t+.01,persistedAt=t+.02)
                meta_path.write_bytes(cli.canonical(meta))
            transition=build_transition(source,source_sha,self.f.registration,self.f.raw,
                                         self.context,'raw-group',phase='training')
            path.write_bytes(cli.canonical(transition))
            self.entry.update(sourcePath=str(source),sourceSha256=source_sha,captureGroup='raw-group',
                              transitionSha256=sha256(path.read_bytes()).hexdigest())
            manifest=local/'raw-manifest.json'
            manifest.write_bytes(cli.canonical(dict(schemaVersion=cli.SCHEMA,
                                                     training=[self.entry],evaluation=[])))
            report=cli.run_manifest(manifest,local/'raw-result',repository=root)
        self.assertEqual(report['trainingRows'],1)
        self.assertTrue(report['provenance']['training'][0]['observationRecomputed'])
        self.assertEqual(report['models'][0]['supportedCommands'],0)


    def test_forged_source_context_cannot_create_other_partition(self):
        cli,root,_,path=self.prepare()
        for key,value in [('viewId','another-view'),('poseRegime','another-pose'),
                          ('object','present'),('load','held')]:
            original=self.context[key];self.context[key]=value
            with self.assertRaisesRegex(ValueError,'original_context_changed'):self.build()
            self.context[key]=original
        transition=json.loads(path.read_bytes());transition['context']['poseRegime']='forged'
        path.write_bytes(cli.canonical(transition))
        from hashlib import sha256
        self.entry['transitionSha256']=sha256(path.read_bytes()).hexdigest()
        with patch('learning.registered_jaw_transition.ingest_session',return_value=self.audit):
            with self.assertRaisesRegex(ValueError,'original_context_changed'):
                cli.load_transition(root,self.entry,'training')
