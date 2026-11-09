"""M1 캠페인의 단계·사전 예측·반려를 가짜 수집 경로로 검증한다. 장치 접근 없음."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch
from learning import m1_shaft_campaign as campaign
from learning.artifacts import ArtifactStore


class M1ShaftCampaignTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.source = self.root/'.local/source'
        source_store=ArtifactStore(self.source); (self.source/'gripper-pair').mkdir()
        for side in ('before', 'after'): (self.source/'gripper-pair'/f'esp32-{side}.jpg').write_bytes(b'image')
        self.owner = campaign.M1ShaftCampaign.__new__(campaign.M1ShaftCampaign)
        c = self.owner; c.root=self.root; c.base=self.root/'campaign'; c.store=ArtifactStore(c.base/'runtime')
        c.plan = dict(steps=[self.stage('training')], seeds=[dict(captureGroup='seed')], freezeAfterStep=1,
            maxNewCommands=1, automaticRetry=False, shaftSeparationBandPx=[130,160], sourceHashes={}, poseContext='fixed')
        c.plan_sha=campaign.sha(campaign.learning.canonical_bytes(c.plan))
        c.rows=[{'seed':True}];c.context={'context':'fixed'};c.model=None;c.evaluation=[]
        c.index=1;c.halted=False;c.preparation_recovered=False;c.commands=0;c.completed_commands=0;c.previous_models=[]
        c.spec={};c.tracker=Mock();c.tracker.observe_shaft.return_value=dict(state=[145.],pixelSha256='pixels')
        c.manager=SimpleNamespace(store=source_store,port=None,token='synthetic-test-only')
        c.observer=Mock(return_value=dict(status='eligible',transition=self.row()))
        self.pair=dict(path=str(self.source/'gripper-pair'),before=dict(frame=dict(frameId='before',pixelSha256='pixels')),
                       after=dict(frame=dict(frameId='after',pixelSha256='pixels')))
        self.raw=dict(status='recorded',offVerified=True,sourceSha256='c'*64,sessionPath=str(self.source),gripperPair=dict(self.pair,status='recorded'))
        self.audit=dict(execution=dict(status='verified',offVerified=True),visual=dict(status='verified',row=dict(evidence=dict(commandAt=100.))))
        self.dispatches=[]
        def collect(manager, request, command, pose_context, **kwargs):
            if (self.source/'prospective.json').exists():
                self.source=self.source.with_name('next-source')
                manager.store=ArtifactStore(self.source);(self.source/'gripper-pair').mkdir()
                for side in ('before','after'):(self.source/'gripper-pair'/f'esp32-{side}.jpg').write_bytes(b'image')
                self.pair['path']=str(self.source/'gripper-pair');self.raw['sessionPath']=str(self.source)
            kwargs['before_command'](manager,self.pair)
            self.dispatches.append(command)
            manager.current={'events':[{'kind':'command'}]}
            return copy.deepcopy(self.raw)
        self.collect=patch.object(campaign,'collect_once',side_effect=collect).start();self.addCleanup(patch.stopall)
        patch.object(campaign,'ingest_session',return_value=self.audit).start()
        patch.object(campaign.time,'monotonic',return_value=90.).start()
        patch.object(campaign.learning,'fit',return_value={'frozen':'model'}).start()

    @staticmethod
    def stage(phase,optional=False):
        return dict(phase=phase,captureGroup=phase,optional=optional,
            command=dict(joint='gripper',direction='open' if phase=='preparation' else 'close',percent=40,durationMs=350))
    @staticmethod
    def row():
        return dict(beforeShaftPx=145.,afterShaftPx=143.,sourceHash='c'*64)
    def fix_plan(self):
        self.owner.plan_sha=campaign.sha(campaign.learning.canonical_bytes(self.owner.plan))

    def test_constructor_validates_seeds_before_memory_credential_and_never_runs_device(self):
        assets=self.root/'.local'
        registration=b'{}';reference=b'synthetic-reference'
        (assets/'registration.json').write_bytes(registration);(assets/'reference.jpg').write_bytes(reference)
        spec=dict(kind='registered_gripper_shaft_only',registrationPath='registration.json',
                  registrationSha256=campaign.sha(registration),referenceImagePath='reference.jpg',
                  referenceImageSha256=campaign.sha(reference),context={})
        spec_path=assets/'spec.json';spec_path.write_text(json.dumps(spec))
        plan=copy.deepcopy(self.owner.plan)
        plan.update(observerSpecPath='.local/spec.json',helperPath='helper',controllerUrl='http://device.invalid',
            controllerBootId='boot',M4BaselineDeg=200.,cameraUrl='http://camera.invalid',cameraBootId='camera-boot',
            deviceId='device',viewId='view',seeds=[dict(sourcePath='.local/source',sha256='a'*64,pairRoot='source/gripper-pair',captureGroup='seed')],
            sourceHashes={'.local/spec.json':campaign.sha(spec_path.read_bytes())})
        previous={'fixed':'previous'};previous_raw=campaign.learning.canonical_bytes(previous)
        model_path=assets/'previous-model.json';model_path.write_bytes(previous_raw)
        model_sha=campaign.sha(previous_raw)
        plan['previousModels']=[dict(path='.local/previous-model.json',sha256=model_sha)]
        plan['sourceHashes']['.local/previous-model.json']=model_sha
        credential=Mock(return_value='synthetic-private-value')
        with patch.object(campaign,'RegisteredGripperCrosssection'), patch.object(campaign,'ManualCollection') as manager, \
             patch('learning.m1_shaft_observations.observe_transition',return_value=dict(status='eligible',transition={'seed':'row'})), \
             patch.object(campaign.learning,'_rows',return_value={'context':'scalar'}), \
             patch.object(campaign.learning,'validate_model',return_value=previous):
            created=campaign.M1ShaftCampaign(self.root,self.root/'new-campaign',plan,credential)
        credential.assert_called_once_with();manager.assert_called_once()
        self.assertEqual(manager.call_args.args[3],'synthetic-private-value')
        self.assertEqual(created.rows,[{'seed':'row'}])
        self.assertEqual(created.previous_models,[dict(modelSha256=model_sha,model=previous)])
        self.assertEqual(model_path.read_bytes(),previous_raw)
        for file in created.store.root.rglob('*'):
            if file.is_file():self.assertNotIn(b'synthetic-private-value',file.read_bytes())
        with self.assertRaisesRegex(ValueError,'runtime_already_exists'):
            campaign.M1ShaftCampaign(self.root,self.root/'new-campaign',plan,credential)
        self.assertEqual(credential.call_count,1)

    def test_failed_raw_after_command_charges_attempt_without_verified_completion(self):
        self.raw.update(status='failed',offVerified=True)
        result=self.owner.step(1)
        self.assertEqual(result['status'],'halted_no_retry')
        self.assertEqual(result['state']['commandAttempts'],1)
        self.assertEqual(result['state']['completedCommands'],0)
        self.assertEqual(result['state']['uncertainCommands'],1)
        self.assertEqual(len(self.dispatches),1)
        self.assertEqual(len(self.owner.rows),1)

    def test_valid_post_observation_outside_scope_is_preserved_without_training(self):
        self.owner.observer.return_value=dict(status='eligible',transition=dict(self.row(),afterShaftPx=170.))
        result=self.owner.step(1)
        self.assertEqual(result['status'],'halted_no_retry')
        self.assertEqual(result['transition']['afterShaftPx'],170.)
        self.assertEqual(result['observation']['status'],'eligible')
        self.assertEqual(len(self.owner.rows),1)
        self.assertEqual(result['state']['commandAttempts'],1)
        self.assertEqual(result['state']['completedCommands'],1)
        self.assertEqual(result['state']['uncertainCommands'],0)

    def test_previous_model_predictions_precede_same_holdout_and_overlap_is_preserved(self):
        c=self.owner;c.plan.update(steps=[self.stage('evaluation')],freezeAfterStep=0);self.fix_plan();c._freeze()
        old={'frozen':'previous'};digest=campaign.sha(campaign.learning.canonical_bytes(old))
        c.previous_models=[dict(modelSha256=digest,model=old)]
        def predict(model,*args,**kwargs):
            self.assertEqual(self.dispatches,[])
            return 144. if model==old else 143.
        def evaluate(model,rows):
            self.assertEqual(rows,[self.row()])
            if model==old:raise ValueError('training_evaluation_overlap')
            return {'evaluated':1}
        with patch.object(campaign.learning,'predict',side_effect=predict), patch.object(campaign.learning,'evaluate',side_effect=evaluate):
            result=c.step(1)
        self.assertEqual(result['status'],'completed');self.assertEqual(len(self.dispatches),1)
        prospective=result['prospective']['previousModels'][0]
        self.assertEqual(prospective['modelSha256'],digest);self.assertEqual(prospective['shaftRidge'],144.)
        self.assertTrue(prospective['comparisonOnly'])
        self.assertEqual(result['previousModels'][0]['status'],'rejected')
        self.assertEqual(result['previousModels'][0]['reason'],'training_evaluation_overlap')
        self.assertEqual(len(c.rows),1)

    def test_unsupported_previous_model_does_not_gate_current_evaluation(self):
        c=self.owner;c.plan.update(steps=[self.stage('evaluation')],freezeAfterStep=0);self.fix_plan();c._freeze()
        old={'frozen':'previous'};c.previous_models=[dict(modelSha256=campaign.sha(campaign.learning.canonical_bytes(old)),model=old)]
        def predict(model,*args,**kwargs):
            if model==old:raise ValueError('shaft_outside_training_range')
            return 143.
        with patch.object(campaign.learning,'predict',side_effect=predict), patch.object(campaign.learning,'evaluate',return_value={'evaluated':1}) as evaluate:
            result=c.step(1)
        self.assertEqual(result['status'],'completed');self.assertEqual(len(self.dispatches),1)
        self.assertEqual(result['prospective']['previousModels'][0]['unsupportedReason'],'shaft_outside_training_range')
        self.assertEqual(result['previousModels'][0]['status'],'rejected');self.assertEqual(evaluate.call_count,1)

    def test_plan_bounds_preparation_and_phase_leak_are_rejected(self):
        c=self.owner;c._validate()
        for key,value in (('maxNewCommands',10),('freezeAfterStep',0)):
            old=c.plan[key];c.plan[key]=value
            with self.assertRaises(ValueError):c._validate()
            c.plan[key]=old
        c.plan['steps']=[self.stage('preparation')]*3;c.plan['maxNewCommands']=3;c.plan['freezeAfterStep']=0
        with self.assertRaisesRegex(ValueError,'preparation_budget'):c._validate()
        c.plan.update(steps=[self.stage('training'),self.stage('evaluation')],maxNewCommands=2,freezeAfterStep=1)
        c.plan['steps'][1]['captureGroup']='seed'
        with self.assertRaisesRegex(ValueError,'group_overlap'):c._validate()

    def test_unobserved_preparation_saves_null_and_never_creates_transition(self):
        c=self.owner;c.plan['steps']=[self.stage('preparation')];c.plan['freezeAfterStep']=0;self.fix_plan()
        c.preparation_recovered=True
        c.tracker.observe_shaft.side_effect=ValueError('shaft_unobserved')
        result=c.step(1)
        self.assertFalse(c.preparation_recovered)
        self.assertEqual(result['status'],'preparation_recorded');self.assertIsNone(result['transition'])
        self.assertIsNone(result['prospective']['observation']);self.assertIsNone(result['prospective']['shaftSeparationPx'])
        self.assertIsNone(result['afterObservation']);self.assertEqual(len(c.rows),1)
        c.observer.assert_not_called();self.assertEqual(len(self.dispatches),1)
        self.assertEqual(c.step(1)['status'],'preparation_recorded');self.assertEqual(len(self.dispatches),1)

    def test_valid_preparation_allows_only_explicit_next_optional_skip(self):
        c=self.owner;c.plan.update(steps=[self.stage('preparation'),self.stage('preparation',True)],freezeAfterStep=0,maxNewCommands=2);self.fix_plan()
        self.assertTrue(c.step(1)['preparationRecovered'])
        self.assertEqual(c.index,2);self.assertEqual(len(self.dispatches),1)
        self.assertEqual(c.step(2)['status'],'skipped');self.assertEqual(len(self.dispatches),1)

    def test_training_before_unobserved_stops_before_output(self):
        c=self.owner;c.tracker.observe_shaft.side_effect=ValueError('shaft_unobserved')
        result=c.step(1)
        self.assertEqual(result['status'],'halted_no_retry');self.assertEqual(self.dispatches,[])
        self.assertTrue(c.halted);self.assertEqual(len(c.rows),1)
        self.assertEqual(result['state']['commandAttempts'],0)
        self.assertEqual(result['state']['uncertainCommands'],0)
        self.assertIsNone(json.loads((self.source/'prospective.json').read_text())['observation'])

    def test_post_rejection_retains_successful_raw_and_halts_without_training(self):
        c=self.owner;c.observer.return_value=dict(status='rejected',reason='shaft_missing')
        result=c.step(1)
        self.assertEqual(result['status'],'halted_no_retry');self.assertEqual(result['collection']['status'],'recorded')
        self.assertTrue(result['collection']['offVerified']);self.assertEqual(len(c.rows),1)
        self.assertTrue(c.halted);self.assertEqual(len(self.dispatches),1)
        c.step(1);self.assertEqual(len(self.dispatches),1)

    def test_training_freezes_and_evaluation_saves_prediction_before_dispatch(self):
        c=self.owner;c.plan.update(steps=[self.stage('training'),self.stage('evaluation')],maxNewCommands=2);self.fix_plan()
        self.assertEqual(c.step(1)['status'],'completed');self.assertEqual(len(c.rows),2)
        self.assertTrue((c.store.root/'frozen-model.json').is_file())
        with patch.object(campaign.learning,'predict',return_value=143.) as predict, patch.object(campaign.learning,'evaluate',return_value={'evaluated':1}):
            result=c.step(2)
        self.assertEqual(result['status'],'completed');self.assertTrue(result['prospectiveVerified'])
        self.assertEqual(result['prospective']['shaftRidge'],143.);self.assertEqual(predict.call_count,2)
        self.assertEqual(len(c.rows),2);self.assertEqual(len(c.evaluation),1)

    def test_evaluation_strict_range_failure_saves_rejection_without_output(self):
        c=self.owner;c.plan.update(steps=[self.stage('evaluation')],freezeAfterStep=0);self.fix_plan();c._freeze()
        with patch.object(campaign.learning,'predict',side_effect=ValueError('shaft_outside_training_range')):
            result=c.step(1)
        self.assertEqual(result['status'],'halted_no_retry');self.assertEqual(self.dispatches,[])
        prospective=json.loads((self.source/'prospective.json').read_text())
        self.assertEqual(prospective['strictRejection'],'shaft_outside_training_range')
        self.assertIsNotNone(prospective['modelSha256'])


if __name__=='__main__':unittest.main()
