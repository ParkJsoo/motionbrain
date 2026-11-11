"""장치 없이 실제 등록 원본 fixture→Campaign→학습→freeze→새 평가 연결."""
import copy
import json
import unittest
from unittest.mock import patch
from learning.batch_jaw_job import JawJob,validate_job,SCHEMA
from learning.batch_campaign import Campaign
from learning.batch_runtime import sha
from learning.experiment_pipeline import encoded
from learning.registered_jaw_transition import build_transition
from tests import test_batch_fixed_jaw_evaluation as fixture


class Clock:
    def monotonic(self): return 0.
    def time(self): return 1000.


class JobTests(unittest.TestCase):
    def setUp(self):
        f=fixture.FixedJawEvaluationTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        self.repo=f.repo;self.root=f.local/'job';self.clock=Clock();self.calls=[];self.saved={};self.failure=None
        self.command=copy.deepcopy(f.model['models'][next(iter(f.model['models']))]['command'])
        plans={}
        for phase,count,group in [('training',2,'train'),('evaluation',1,'future')]:
            plans[phase+'Plan']=dict(schemaVersion='motionbrain.batch_campaign.v1',campaignId=phase,viewId='fixed',
                controllerBootId='boot',maxObservationAgeSeconds=2,nodes=['ready'],
                budgets=dict(maxPhysicalCommands=count,maxPreparationCommands=1,maxAttempts=count,maxSeconds=100),
                conditions=[dict(id='c',command=self.command,fromNodes=['ready'],phase=phase,captureGroup=group,target=count,maxAttempts=count)])
        self.config=dict(schemaVersion=SCHEMA,jobId='job',outputRoot=str(self.root),dispatchLockPath=str(f.local/'batch-locks'/(sha(b'boot')+'.lock')),
                         maxPhysicalCommands=3,maxSeconds=200,driver={},**plans)
        self.training=[]
        for sid in ('a','b'):
            source=f.local/sid;source_sha=sha((source/'session.json').read_bytes())
            t=build_transition(source,source_sha,f.reg,f.ref,f.context,'train',phase='training')
            path=f.local/(sid+'-job-transition.json');path.write_bytes(encoded(t))
            entry=dict(f.entry,sourcePath=str(source),sourceSha256=source_sha,transitionPath=str(path),transitionSha256=sha(path.read_bytes()),captureGroup='train')
            self.training.append((entry,t['learningRow']))
    def factory(self,repo,config,phase,frozen=None):
        owner=self
        if phase=='evaluation':
            self.f.proof['freeze']=json.loads(__import__('pathlib').Path(frozen['freeze']['path']).read_bytes())
            self.f.reseal()
        class Driver:
            def observe(self): return dict(nodes=['ready'],bootId='boot',viewId='fixed',observedAt=0.,offVerified=True,journalClear=True)
            def execute(self,c,r,*,deadline_wall):
                owner.calls.append((phase,r))
                if owner.failure=='interrupt': raise KeyboardInterrupt()
                if phase=='training': entry,row=owner.training[sum(p=='training' for p,_ in owner.calls)-1]
                else: entry,row=owner.f.entry,json.loads(__import__('pathlib').Path(owner.f.entry['transitionPath']).read_bytes())['learningRow']
                result=dict(execution='completed',offVerified=True,journalClear=True,capture='recorded',observation='eligible',
                    failureScope=None,transitionId=entry['transitionSha256'],manifestEntry=entry,
                    dataIdentity={k:row[k] for k in ('sourceHash','sessionId','episodeId','frameIds','pairedFrameIds')},
                    prospectiveEvidenceVerified=phase=='evaluation')
                if owner.failure=='observation': result['observation']='rejected'
                owner.saved[r]=result;return result
            def reconcile(self,c,r): return owner.saved.get(r)
        return Driver()
    def run_job(self): return JawJob(self.repo,self.config,self.factory,clock=self.clock).run()
    def test_actual_fixture_pipeline_and_completed_resume_never_dispatch(self):
        result=self.run_job();self.assertEqual(result['status'],'completed');self.assertEqual(len(self.calls),3)
        report=json.loads(__import__('pathlib').Path(result['evaluationReport']['path']).read_bytes())
        self.assertEqual((report['trainingRows'],report['evaluationRows']),(0,1));self.assertTrue(report['prospectiveEvidenceVerified'])
        self.assertFalse(result['modelSelected']);self.run_job();self.assertEqual(len(self.calls),3)
    def test_first_quality_failure_halts_without_replacement(self):
        self.failure='observation';self.assertEqual(self.run_job()['status'],'halted');self.run_job();self.assertEqual(len(self.calls),1)
    def test_interrupted_command_reconciles_without_resend(self):
        self.failure='interrupt'
        with self.assertRaises(KeyboardInterrupt): self.run_job()
        self.failure=None;result=self.run_job();self.assertEqual(result['status'],'halted');self.assertEqual(len(self.calls),1)
    def test_changed_plan_and_combined_budget_rejected(self):
        self.config['maxPhysicalCommands']=2
        with self.assertRaisesRegex(ValueError,'combined_budget'):validate_job(self.config)
        self.config['maxPhysicalCommands']=3;self.run_job();self.config['maxSeconds']=201
        with self.assertRaisesRegex(ValueError,'saved_config_changed'):self.run_job()
    def test_phase_group_leak_rejected(self):
        self.config['evaluationPlan']['conditions'][0]['captureGroup']='train'
        with self.assertRaisesRegex(ValueError,'identity_overlap'):validate_job(self.config)
    def test_incomplete_fit_is_not_retrained_on_resume(self):
        with patch('learning.batch_jaw_job.run_manifest',side_effect=RuntimeError('fixture fit failure')):
            with self.assertRaises(RuntimeError):self.run_job()
        self.assertEqual(self.run_job()['status'],'halted');self.assertEqual(len(self.calls),2)
    def test_artifact_tamper_rejected_without_dispatch(self):
        result=self.run_job();__import__('pathlib').Path(result['model']['path']).write_bytes(b'{}')
        with self.assertRaisesRegex(ValueError,'artifact_changed'):self.run_job()
        self.assertEqual(len(self.calls),3)
    def test_controller_lock_is_held_during_training_offline_work(self):
        from learning.batch_jaw_job import run_manifest as actual
        def inspect(*args,**kwargs):
            with self.assertRaisesRegex(ValueError,'already_running'):
                with Campaign._Lock(__import__('pathlib').Path(self.config['dispatchLockPath'])): pass
            return actual(*args,**kwargs)
        with patch('learning.batch_jaw_job.run_manifest',side_effect=inspect):self.run_job()
    def test_published_freeze_interrupt_recovers_same_binding_time(self):
        from learning.batch_jaw_job import freeze_model as actual
        def interrupted(*args,**kwargs):
            actual(*args,**kwargs)
            raise KeyboardInterrupt()
        with patch('learning.batch_jaw_job.freeze_model',side_effect=interrupted):
            with self.assertRaises(KeyboardInterrupt):self.run_job()
        raw=(self.root/'freeze.json').read_bytes()
        with patch('learning.batch_jaw_job.freeze_model',side_effect=AssertionError('must not re-freeze')):
            self.assertEqual(self.run_job()['status'],'completed')
        self.assertEqual((self.root/'freeze.json').read_bytes(),raw);self.assertEqual(len(self.calls),3)
    def test_support_deficit_stops_before_freeze_or_evaluation(self):
        self.config['trainingPlan']['conditions'][0].update(target=1,maxAttempts=1)
        with self.assertRaisesRegex(ValueError,'support_deficit'):self.run_job()
        self.assertEqual(len(self.calls),1);self.assertFalse((self.root/'freeze.json').exists())
        self.assertEqual(self.run_job()['status'],'halted')
    def test_reused_training_source_halts_before_model(self):
        self.training[1]=self.training[0]
        self.assertEqual(self.run_job()['status'],'halted');self.assertFalse((self.root/'freeze.json').exists())
    def test_overall_deadline_not_reset_after_fit(self):
        from learning.batch_jaw_job import run_manifest as actual
        def delayed(*args,**kwargs):
            result=actual(*args,**kwargs);self.clock.time=lambda:1300.;return result
        with patch('learning.batch_jaw_job.run_manifest',side_effect=delayed):
            with self.assertRaisesRegex(ValueError,'deadline_exhausted'):self.run_job()
        self.assertEqual(len(self.calls),2);self.assertEqual(self.run_job()['status'],'halted')
    def test_cleanup_failure_is_persistent_not_completed_on_resume(self):
        actual=self.factory
        def factory(*args,**kwargs):
            driver=actual(*args,**kwargs)
            if args[2]=='evaluation':
                def close(): raise OSError('fixture cleanup failed')
                driver.close=close
            return driver
        with self.assertRaisesRegex(ValueError,'cleanup_failed'):
            JawJob(self.repo,self.config,factory,clock=self.clock).run()
        result=self.run_job();self.assertEqual(result['status'],'halted')
        self.assertEqual(result['reason'],'jaw_job_cleanup_failed');self.assertEqual(len(self.calls),3)
    def test_published_freeze_time_tamper_cannot_recover(self):
        from learning.batch_jaw_job import freeze_model as actual
        def interrupted(*args,**kwargs):
            actual(*args,**kwargs);raise KeyboardInterrupt()
        with patch('learning.batch_jaw_job.freeze_model',side_effect=interrupted):
            with self.assertRaises(KeyboardInterrupt):self.run_job()
        path=self.root/'freeze.json';value=json.loads(path.read_bytes());value['frozenAt']+=1;path.write_bytes(encoded(value))
        with self.assertRaisesRegex(ValueError,'recovered_freeze_changed'):self.run_job()
        self.assertEqual(len(self.calls),2)
    def test_phase_handoff_runs_actual_profile_lifecycle(self):
        from learning.batch_jaw_driver import JawDriver,PROFILE,RESTORE
        from types import MethodType
        original=self.factory;profile=dict(RESTORE);changes=[]
        def factory(*args,**kwargs):
            d=original(*args,**kwargs);d.profile_owned=False;d.manager=None
            def optical(desired=None):
                if desired: profile.clear();profile.update(desired);changes.append(desired['frameSize'])
                return dict(profile)
            d._profile=optical;d._ready_profile=MethodType(JawDriver._ready_profile,d)
            d._release_manager=MethodType(JawDriver._release_manager,d);d.close=MethodType(JawDriver.close,d)
            old=d.observe
            def observe(): d._ready_profile();return old()
            d.observe=observe
            return d
        result=JawJob(self.repo,self.config,factory,clock=self.clock).run()
        self.assertEqual(result['status'],'completed')
        self.assertEqual(changes,[PROFILE['frameSize'],RESTORE['frameSize'],PROFILE['frameSize'],RESTORE['frameSize']])
    def test_real_factory_receives_phase_config_without_device_access(self):
        from tests import test_batch_jaw_driver as driver_fixture
        from learning import batch_jaw_driver as jaw
        f=driver_fixture.JawDriverTests();f.setUp();self.addCleanup(f.doCleanups)
        driver=copy.deepcopy(f.config);driver.pop('plan');driver.pop('root')
        lock=f.local/'batch-locks'/(sha(('b'*32).encode())+'.lock')
        auth=dict(f.auth,maxPhysicalCommands=8,dispatchLockPath=str(lock));binding=f.binding('auth-job.json',auth);driver['authorization']=binding
        config=dict(schemaVersion=SCHEMA,jobId='live-interface',outputRoot=str(f.local/'job'),dispatchLockPath=str(lock),
            maxPhysicalCommands=8,maxSeconds=1000,trainingPlan=driver_fixture.plan(),evaluationPlan=driver_fixture.plan('evaluation'),driver=driver)
        with patch.object(jaw,'RegisteredGripperLandmarks',return_value=f.tracker),patch.object(jaw,'verify_qualification',return_value={}),\
             patch.object(jaw.urllib.request,'build_opener',side_effect=AssertionError('network')),\
             patch('learning.macos_credentials.load_keychain_secret',side_effect=AssertionError('credentials')):
            job=JawJob(f.repo,config,jaw.create_driver);campaign=job._campaign('training')
            self.assertEqual(campaign.compiled['rejected'],[])
            self.assertEqual(campaign.driver.config['root'],str((f.local/'job/training').resolve()))
            campaign.driver.close();campaign.close()
        from tools.run_jaw_collection_job import main
        path=f.local/'job-config.json';path.write_bytes(encoded(config))
        with patch.object(jaw,'create_driver',side_effect=AssertionError('no-run must not construct')):
            self.assertEqual(main(['--config',str(path)]),0)
    def test_controller_lock_cannot_be_redirected(self):
        self.config['dispatchLockPath']=str(self.f.local/'private-lock')
        with self.assertRaisesRegex(ValueError,'shared_dispatch_lock'):self.run_job()
        self.assertEqual(self.calls,[])
    def test_primary_error_is_not_masked_by_cleanup_failure(self):
        actual=self.factory
        def factory(*args,**kwargs):
            driver=actual(*args,**kwargs)
            def execute(*args,**kwargs): raise RuntimeError('original failure')
            def close(): raise OSError('cleanup failure')
            driver.execute=execute;driver.close=close;return driver
        with self.assertRaisesRegex(RuntimeError,'original failure'):
            JawJob(self.repo,self.config,factory,clock=self.clock).run()
        result=self.run_job();self.assertTrue(result['cleanupFailed']);self.assertEqual(result['status'],'halted')
        self.assertEqual(result['reason'],'job_failure:RuntimeError')
