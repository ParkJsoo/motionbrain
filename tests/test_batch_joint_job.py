"""모든 관절의 합성 원본→실제 관측/학습→단일 원본 fixed 평가 회귀. 장치 없음."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from learning.batch_joint_job import JointSuite,validate_suite,SCHEMA
from learning.batch_jaw_job import JOINT_SCHEMA,validate_job
from learning.batch_runtime import BatchRuntime,sha
from learning.batch_fixed_evaluation import freeze_model,prospective_evidence,seal_before
from learning.experiment_pipeline import encoded
from learning.pipeline_evidence import ingest_session
from tools.train_batch_campaign import run_manifest
from tests.test_experiment_pipeline import fixture
from tests import test_batch_commissioning as commissioning


class ExternalOnlyTests(unittest.TestCase):
    def setUp(self):
        f=commissioning.CommissioningTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f;self.repo=f.repository;self.local=f.local
    def source(self,sid,joint,direction,phase,model=None,freeze=None):
        source=(self.local/(joint+'-runtime')/'sessions'/sid).resolve();command=dict(joint=joint,direction=direction,percent=20,durationMs=200)
        original=__import__('numpy').zeros
        with patch('tests.test_experiment_pipeline.np.zeros',side_effect=lambda shape,*a,**k:self.f.image.copy() if shape==(32,32,4) else original(shape,*a,**k)):
            identity=fixture(source,sid,0,phase,command=command)
        spec=copy.deepcopy(self.f.spec);spec['joints']={joint:spec['joints']['gripper']}
        path=self.local/(joint+'-observer.json');path.write_text(json.dumps(spec,indent=2))
        runtime=object.__new__(BatchRuntime);runtime.repository=self.repo;runtime.root=(self.local/(joint+'-runtime')).resolve();runtime.spec=spec
        runtime.clock_id=sid;runtime.gripper_spec=None;runtime.journal=self.local/'unused-journal'
        runtime.config=dict(observerSpecPath=str(path),observerSpecSha256=sha(path.read_bytes()),deviceId='synthetic-device',poseContext='local')
        runtime.plan=dict(viewId='fixed-view')
        condition=dict(id=sid,phase=phase,captureGroup=phase,fromNodes=['g'],command=command)
        audit=ingest_session(source,identity);self.assertEqual(audit['execution']['status'],'verified',audit['execution'])
        row=audit['visual']['row'];session=json.loads((source/'session.json').read_bytes());frame=session['frames'][row['evidence']['preFrameIds'][-2]]
        before=runtime._observe_native(source/'native-camera',frame,historical=True)
        (source/'batch-before-observation.json').write_bytes(encoded(before))
        if model:
            runtime.config.update(frozenModels={joint:model},fixedCommandEvaluation=freeze)
            model_data=json.loads(Path(model['path']).read_bytes());freeze_data=json.loads(Path(freeze['path']).read_bytes())
            names,values=runtime._states(before['observation'],None,joint)
            meta=before['derivedMetadata'];frames=[{k:meta[k] for k in ('frameId','pixelSha256','capturedAt','persistedAt')}]
            r=dict(sessionId=sid,stateFields=names,before=values,command=command,context=runtime._context(),beforePersistedAt=meta['persistedAt'])
            proof=prospective_evidence(model_data,freeze_data,r,frames,meta['persistedAt']+.001,input_count=1)
            manager=SimpleNamespace(current=dict(phase='recording',sessionId=sid,events=[]),command_count=0,store=SimpleNamespace(root=source,write_json=lambda n,v:(source/n).write_bytes(encoded(v))))
            manager._event=lambda e:manager.current['events'].append(e)
            seal_before(manager,proof);session.update(fixedEvaluationProofSha256=manager.current['fixedEvaluationProofSha256']);session['events']+=manager.current['events']
            intent=json.loads((source/'command-001-intent.json').read_bytes());intent['fixedEvaluationProofSha256']=session['fixedEvaluationProofSha256']
            session['events']=[intent if e.get('kind')=='command' else e for e in session['events']]
            (source/'command-001-intent.json').write_bytes(encoded(intent));(source/'session.json').write_bytes(encoded(session))
            (source/'batch-prospective.json').write_bytes(encoded(dict(proof=proof,beforeFrameId=frame['frameId'],beforePixelSha256=frame['pixelSha256'])))
        identity=sha((source/'session.json').read_bytes())
        result=runtime._classify(condition,sid,dict(sessionPath=str(source),sourceSha256=identity),process=True)
        self.assertEqual(result['observation'],'eligible',result.get('observationFailureCode',result['reason']))
        path=runtime.root/'transitions'/(result['transitionId']+'.json')
        return dict(sourcePath=str(source),sourceSha256=identity,transitionPath=str(path),transitionSha256=result['transitionId'])
    def test_all_five_external_only_official_train_freeze_future_evaluation(self):
        for joint,direction in [('gripper','close'),('wrist','up'),('elbow','down'),('shoulder','down'),('base','left')]:
            with self.subTest(joint=joint):
                entries=[self.source(joint+'a',joint,direction,'training'),self.source(joint+'b',joint,direction,'training')]
                p=self.local/(joint+'train.json');p.write_bytes(encoded(dict(schemaVersion='motionbrain.batch_training_manifest.v1',training=entries,evaluation=[])))
                out=self.local/(joint+'train');trained=run_manifest(p,out,repository=self.repo)
                info=trained['models'][0];model=dict(path=str(out/info['path']),sha256=info['sha256'])
                frozen=freeze_model(self.repo,model,self.local/(joint+'freeze.json'),clock=SimpleNamespace(monotonic=lambda:0.))
                entry=self.source(joint+'c',joint,direction,'evaluation',model,frozen)
                manifest=dict(schemaVersion='motionbrain.batch_training_manifest.v1',training=[],evaluation=[entry],frozenModels=[model],fixedCommandEvaluation=frozen)
                p=self.local/(joint+'eval.json');p.write_bytes(encoded(manifest));report=run_manifest(p,self.local/(joint+'eval'),repository=self.repo)
                self.assertTrue(report['prospectiveEvidenceVerified']);self.assertEqual(report['models'][0]['evaluation']['evaluated'],1)
                source=Path(entry['sourcePath']);intent=json.loads((source/'command-001-intent.json').read_bytes());intent['fixedEvaluationProofSha256']='f'*64;(source/'command-001-intent.json').write_bytes(encoded(intent))
                with self.assertRaises(ValueError):run_manifest(p,self.local/(joint+'tampered'),repository=self.repo)

    def test_suite_all_five_real_trainer_freeze_and_new_source_without_phase_mock(self):
        owner=self;calls=[]
        class Clock:
            def time(self):return 1000.
            def monotonic(self):return 0.
        def factory(repo,config,phase,frozen=None):
            class Driver:
                def observe(self):return dict(nodes=['g'],bootId='a'*32,viewId='fixed-view',observedAt=0.,offVerified=True,journalClear=True)
                def execute(self,c,r,*,deadline_wall):
                    calls.append((c['command']['joint'],phase));joint=c['command']['joint']
                    entry=owner.source(r,joint,c['command']['direction'],phase,**({} if frozen is None else dict(model=frozen['model'],freeze=frozen['freeze'])))
                    row=json.loads(Path(entry['transitionPath']).read_bytes())['learningRow']
                    return dict(execution='completed',offVerified=True,journalClear=True,capture='recorded',observation='eligible',
                        manifestEntry=entry,transitionId=entry['transitionSha256'],prospectiveEvidenceVerified=phase=='evaluation',
                        dataIdentity={k:row[k] for k in ('sourceHash','sessionId','episodeId','frameIds')})
                def reconcile(self,*args):return None
                def close(self):pass
            return Driver()
        jobs=[];root=self.local/'suite'
        for joint,direction in [('gripper','close'),('wrist','up'),('elbow','down'),('shoulder','down'),('base','left')]:
            plans={}
            for phase,count in [('training',2),('evaluation',1)]:
                plans[phase+'Plan']=dict(schemaVersion='motionbrain.batch_campaign.v1',campaignId=joint+phase,viewId='fixed-view',controllerBootId='a'*32,
                    maxObservationAgeSeconds=2,nodes=['g'],budgets=dict(maxPhysicalCommands=count,maxPreparationCommands=1,maxAttempts=count,maxSeconds=100),
                    conditions=[dict(id='c',command=dict(joint=joint,direction=direction,percent=20,durationMs=200),fromNodes=['g'],phase=phase,captureGroup=phase,target=count,maxAttempts=count)])
            jobs.append(dict(schemaVersion=JOINT_SCHEMA,jobId=joint,outputRoot=str(root/joint),dispatchLockPath=str(self.local/'batch-locks'/(sha(('a'*32).encode())+'.lock')),
                maxPhysicalCommands=3,maxSeconds=200,driver={},**plans))
        config=dict(schemaVersion=SCHEMA,suiteId='all',outputRoot=str(root),maxPhysicalCommands=15,maxSeconds=600,jobs=jobs)
        suite=JointSuite(self.repo,config,factory,clock=Clock());report=suite.run()
        self.assertEqual(report['status'],'completed');self.assertEqual(len(calls),15)
        self.assertEqual(len(report['results']),5)
        for result in report['results']:
            evaluated=json.loads(Path(result['evaluationReport']['path']).read_bytes())
            self.assertTrue(evaluated['prospectiveEvidenceVerified']);self.assertEqual(evaluated['evaluationRows'],1)
        self.assertEqual(suite.run(),report);self.assertEqual(len(calls),15)

class SuiteBoundaryTests(unittest.TestCase):
    def setUp(self):
        from tests.test_batch_jaw_job import JobTests
        f=JobTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        root=f.f.local/'suite';jobs=[]
        for i in range(2):
            j=copy.deepcopy(f.config);j.update(schemaVersion=JOINT_SCHEMA,jobId=str(i),outputRoot=str(root/str(i)))
            for phase in ('training','evaluation'):j[phase+'Plan']['campaignId']+=str(i)
            jobs.append(j)
        self.config=dict(schemaVersion=SCHEMA,suiteId='suite',outputRoot=str(root),maxPhysicalCommands=6,maxSeconds=60,jobs=jobs)
    def test_halt_blocks_next_partition_and_restart(self):
        calls=[]
        class Job:
            def __init__(self,*a,**k):calls.append(k['deadline_wall'])
            def run(self):return dict(status='halted')
        with patch('learning.batch_joint_job.JawJob',Job):
            suite=JointSuite(self.f.repo,self.config,lambda *a,**k:SimpleNamespace(close=lambda:None),clock=self.f.clock)
            self.assertEqual(suite.run()['status'],'halted');suite.run()
        self.assertEqual(calls,[1060.])
    def test_global_deadline_not_restarted_for_next_joint(self):
        clock=SimpleNamespace(now=1000.);clock.time=lambda:clock.now
        calls=[]
        class Job:
            def __init__(self,*a,**k):calls.append(k['deadline_wall'])
            def run(self):clock.now+=40;return dict(status='completed')
        with patch('learning.batch_joint_job.JawJob',Job):
            result=JointSuite(self.f.repo,self.config,lambda *a,**k:SimpleNamespace(close=lambda:None),clock=clock).run()
        self.assertEqual(calls,[1060.,1060.]);self.assertEqual(result['deadlineWall'],1060.)
    def test_later_partition_static_failure_precedes_any_job_execution(self):
        calls=[]
        def factory(repo,config,*a,**k):
            calls.append(config['root'])
            if '/1/' in config['root']:raise ValueError('unsupported_later_observer')
            return SimpleNamespace(close=lambda:None)
        with patch('learning.batch_joint_job.JawJob',side_effect=AssertionError('must not start')):
            suite=JointSuite(self.f.repo,self.config,factory,clock=self.f.clock)
            with self.assertRaisesRegex(ValueError,'unsupported_later_observer'):suite.run()
            self.assertEqual(suite.run()['status'],'halted')
        self.assertEqual(len(calls),3)

    def test_single_source_mode_is_explicit_and_still_requires_post_freeze_input(self):
        from tests import test_batch_fixed_evaluation as fixture
        f=fixture.FixedEvaluationTests();f.setUp()
        with self.assertRaisesRegex(ValueError,'new_persisted_inputs'):
            prospective_evidence(f.model,f.freeze,f.row,f.frames[:1],11.)
        proof=prospective_evidence(f.model,f.freeze,f.row,f.frames[:1],11.,input_count=1)
        self.assertEqual(proof['inputMode'],'external_native_only')
        frames=copy.deepcopy(f.frames[:1]);frames[0]['capturedAt']=f.freeze['frozenAt']
        with self.assertRaisesRegex(ValueError,'new_persisted_inputs'):
            prospective_evidence(f.model,f.freeze,f.row,frames,11.,input_count=1)

    def test_mixed_joint_partition_rejected(self):
        self.config['jobs'][0]['evaluationPlan']['conditions'][0]['command']=dict(joint='base',direction='left',percent=20,durationMs=200)
        with self.assertRaisesRegex(ValueError,'joint_partition'):validate_suite(self.config)
    def test_runtime_auth_expiry_caps_existing_final_dispatch_deadline(self):
        from learning.batch_joint_job import RuntimeJobDriver
        d=object.__new__(RuntimeJobDriver);d.auth={'expiresAt':1100.};d._verify=lambda:None;d._reserve=lambda r:None
        d._entry=lambda r:r;d.runtime=SimpleNamespace(execute=lambda c,r,deadline_wall:deadline_wall)
        self.assertEqual(d.execute({},'r',deadline_wall=1200.),1100.)
    def test_cli_validation_never_constructs_driver_or_accesses_device(self):
        from tools.run_joint_collection_job import main
        path=self.f.f.local/'suite-config.json';path.write_bytes(encoded(self.config))
        with patch('tools.run_joint_collection_job.JointSuite',side_effect=AssertionError('device')):
            self.assertEqual(main(['--config',str(path)]),0)

class RuntimeAdapterTests(unittest.TestCase):
    def test_actual_constructor_checks_files_without_credential_or_device(self):
        from tests.test_batch_runtime import BatchRuntimeTests
        from learning.batch_joint_job import RuntimeJobDriver
        from learning.batch_jaw_driver import driver_config_digest
        from learning.batch_campaign import digest
        f=BatchRuntimeTests();f.setUp();self.addCleanup(f.doCleanups)
        # The fixture's colored object is removed from the declared empty scene;
        # all actual observer/file/route validations still execute.
        spec=copy.deepcopy(f.spec);spec.update(object='none',load='empty');f.spec_path.write_bytes(encoded(spec))
        plan=copy.deepcopy(f.plan);plan['runtime']['observerSpecSha256']=sha(f.spec_path.read_bytes())
        config=dict(plan=plan,root=str(f.private/'job'/'training'),adapter='batch_runtime',credential=dict(service='fixture',account='fixture'))
        auth=dict(schemaVersion='motionbrain.joint_job_authorization.v1',driverConfigSha256=driver_config_digest(config),planHashes=[digest(plan)],controllerBootId=plan['controllerBootId'],
            outputRoot=str(f.private/'job'),maxPhysicalCommands=2,expiresAt=9999999999.,onsite=dict(fixedCamera=True,workspaceClear=True,noEntryDuringJob=True,noncontactEmptyLoad=True))
        path=f.private/'auth.json';path.write_bytes(encoded(auth));config['authorization']=dict(path=str(path),sha256=sha(path.read_bytes()))
        with patch('learning.macos_credentials.load_keychain_secret',side_effect=AssertionError('credential')):
            d=RuntimeJobDriver(f.repository,config,'training');self.assertIsNone(d.condition_support(plan['conditions'][0]))
            self.assertEqual(d.runtime.plan['runtime']['jointJobAuthorization'],config['authorization'])
            # Final RUN uses _deadline after capture; changed auth must fail there.
            path.write_bytes(b'{}')
            with self.assertRaisesRegex(ValueError,'binding_changed'):d.runtime._deadline()
            d.close()
