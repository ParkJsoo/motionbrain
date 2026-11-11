import copy
import json
import unittest
from unittest.mock import patch

from learning.batch_learning import fit
from learning.batch_observation import digest
from learning.batch_runtime import sha
from learning.batch_model_runtime import ModelDecisionRuntime, BatchModelRunner
from learning.experiment_pipeline import encoded
from tests import test_batch_runtime as fixture


class Clock:
    def __init__(self):self.now=100.
    def monotonic(self):return self.now
    def time(self):return 1000+self.now


class FakeRuntime(ModelDecisionRuntime):
    def observe_model(self,model,*,m4=False):
        self.clock.now+=.5
        self.observations+=1
        return dict(status='accepted',context=model['context'],stateFields=model['stateFields'],
            state=[2. if self.executions==0 else 4.]*4,frameIds=['current-'+str(self.observations)],
            observedAt=self.clock.now-.1,persistedAt=self.clock.now)

    def execute(self,condition,request_id,*,deadline_wall):
        self.executions+=1
        command_at=self.clock.now+.2;off=command_at+.5
        # Simulate slow original audit/retention: recorded target is now old.
        self.clock.now+=12
        row=dict(command=condition['command'],sourceHash='a'*64,commandAt=command_at,context=self.model['context'],
            stateFields=self.model['stateFields'],after=[4.]*4)
        transition=dict(learningRow=row,after=dict(derivedMetadata=dict(frameId='old-after',capturedAt=off+3,persistedAt=off+3.1)))
        raw=encoded(transition);identity=sha(raw)
        path=self.root/'transitions';path.mkdir(parents=True,exist_ok=True);(path/(identity+'.json')).write_bytes(raw)
        return dict(execution='completed',offVerified=True,journalClear=True,observation='eligible',transitionId=identity,
            evidence=dict(audit=dict(visual=dict(row=dict(evidence=dict(offReceiptAt=off))))))


class BatchModelRuntimeTests(unittest.TestCase):
    def setUp(self):
        f=fixture.BatchRuntimeTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        fields=[v['name'] for v in f.spec['joints']['wrist']['stateFields']]
        context={**{k:f.spec[k] for k in ('viewId','poseRegime','object','load')},'observerSha256':digest(f.spec)}
        rows=[]
        for i in (1,3):
            rows.append(dict(sessionId='s'+str(i),episodeId='e'+str(i),sourceHash=str(i)*64,frameIds=[str(i)+'a',str(i)+'b'],
                captureGroup='train',joint='wrist',context=context,stateFields=fields,before=[float(i)]*4,
                after=[float(i)+2]*4,command=f.condition['command']))
        self.model=fit(rows);modelpath=f.private/'model.json';modelpath.write_bytes(encoded(self.model))
        plan=copy.deepcopy(f.plan);plan['conditions'][0]['phase']='evaluation';plan['runtime']['frozenModels']={
            'wrist':dict(path='.local/model.json',sha256=sha(modelpath.read_bytes()))}
        self.condition=plan['conditions'][0]
        self.runtime=FakeRuntime(plan,f.repository,f.private/'selected-runtime',f.credential,clock=Clock())
        self.runtime.model=self.model;self.runtime.executions=0;self.runtime.observations=0
        self.stage=dict(id='approach',kind='position',modelId='wrist',target=[4.]*4,tolerance=[.2]*4,
            commands=[self.condition['command']],maxCommands=1)
        self.runner=BatchModelRunner('trial',[self.stage],{'approach':self.runtime},
            {'approach':[self.condition]},{'wrist':self.model},f.private/'selected-trial')

    def test_model_to_existing_runtime_to_fresh_observation_end_to_end(self):
        result=self.runner.run(confirmed_scope_sha256=self.runner.scope_sha256)
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(self.runtime.executions,1)
        self.assertEqual(self.runtime.observations,3)
        self.assertEqual(result['decisions'][0]['request']['command'],self.condition['command'])
        self.assertFalse(result['physicalSuccessClaimed'])
        with self.assertRaises(FileExistsError):self.runner.run(confirmed_scope_sha256=self.runner.scope_sha256)
        self.assertEqual(self.runtime.executions,1)

    def test_shared_campaign_lock_prevents_model_observation_and_dispatch(self):
        from learning.batch_campaign import Campaign
        from learning.batch_runtime import private_path
        path=private_path(self.runtime.repository,self.runtime.config['dispatchLockPath'])
        path.parent.mkdir(parents=True,exist_ok=True)
        with Campaign._Lock(path):
            result=self.runner.run(confirmed_scope_sha256=self.runner.scope_sha256)
        self.assertEqual(result['status'],'halted')
        self.assertIn('dispatcher_already_running',result['reason'])
        self.assertEqual(self.runtime.observations,0)
        self.assertEqual(self.runtime.executions,0)

    def test_wrong_scope_never_observes_or_executes(self):
        with self.assertRaisesRegex(ValueError,'explicit_scope'):
            self.runner.run(confirmed_scope_sha256='wrong')
        self.assertEqual(self.runtime.executions,0);self.assertEqual(self.runtime.observations,0)

    def test_failed_execution_halts_without_replacement(self):
        with patch.object(self.runtime,'execute',return_value=dict(execution='unknown',offVerified=False)) as execute:
            result=self.runner.run(confirmed_scope_sha256=self.runner.scope_sha256)
        self.assertEqual(result['status'],'halted');self.assertFalse(result['automaticRetry']);execute.assert_called_once()
        with self.assertRaises(FileExistsError):self.runner.run(confirmed_scope_sha256=self.runner.scope_sha256)

    def test_precommand_model_reselection_rejects_changed_choice(self):
        from learning.artifacts import ArtifactStore
        stage=self.stage;runtime=self.runtime;runtime.decision_binding=(stage,self.model,self.condition['command'])
        evidence=dict(observation=dict(status='accepted',states={'wrist':dict(stateFields=self.model['stateFields'],state=[2.]*4)}),
            derivedMetadata=dict(frameId='pre',capturedAt=99.5,persistedAt=99.6))
        runtime._check_decision(evidence,None,ArtifactStore(self.f.private/'guard-good'))
        evidence['observation']['states']['wrist']['state']=[3.]*4
        with self.assertRaisesRegex(ValueError,'choice_changed'):
            runtime._check_decision(evidence,None,ArtifactStore(self.f.private/'guard-stop'))

    def test_independent_camera_age_limits_preserve_valid_gripper_pre_window(self):
        runtime=self.runtime;runtime.decision_binding=(self.stage,self.model,self.condition['command'])
        runtime.gripper_spec={'maxAgeSeconds':10}
        obs=dict(status='accepted',context=self.model['context'],stateFields=self.model['stateFields'],state=[2.]*4,
            frameIds=['external','gripper'],observedAt=95.,persistedAt=99.5)
        evidence={'derivedMetadata':{'capturedAt':99.5}}
        pair={'frameBinding':{'capturedAt':95.}}
        from learning.artifacts import ArtifactStore
        with patch.object(runtime,'_decision_observation',return_value=obs):
            runtime._check_decision(evidence,pair,ArtifactStore(self.f.private/'age-good'))
            pair['frameBinding']['capturedAt']=89.
            with self.assertRaisesRegex(ValueError,'gripper_decision_stale'):
                runtime._check_decision(evidence,pair,ArtifactStore(self.f.private/'age-bad'))

    def test_interrupt_is_recorded_and_cleanup_failure_is_not_success(self):
        with patch.object(self.runtime,'execute',side_effect=KeyboardInterrupt('operator')), \
             patch.object(self.runtime,'close',side_effect=RuntimeError('cleanup')):
            result=self.runner.run(confirmed_scope_sha256=self.runner.scope_sha256)
        self.assertEqual(result['status'],'cleanup_failed')
        self.assertIn('KeyboardInterrupt',result['reason'])
        saved=json.loads((self.runner.root/'evidence/report.json').read_bytes())
        self.assertEqual(saved,result)
        with self.assertRaises(FileExistsError):self.runner.run(confirmed_scope_sha256=self.runner.scope_sha256)

    def test_mutated_scope_cannot_reuse_approval(self):
        self.runner.sequence.stages[0]['maxCommands']=4
        with self.assertRaisesRegex(ValueError,'scope_changed'):
            self.runner.run(confirmed_scope_sha256=self.runner.scope_sha256)
        self.assertEqual(self.runtime.executions,0)

    def test_stationary_observer_still_rejected_before_credentials(self):
        plan=copy.deepcopy(self.runtime.plan);spec=self.f.stationary_registered_spec()
        self.f.spec_path.write_bytes(encoded(spec));plan['runtime']['observerSpecSha256']=sha(self.f.spec_path.read_bytes())
        with self.assertRaisesRegex(ValueError,'stationary_only'):
            ModelDecisionRuntime(plan,self.f.repository,self.f.private/'blocked',self.f.credential)
        self.assertEqual(self.f.credentials,[])


class BeforeReviewConnectionTests(unittest.TestCase):
    def setUp(self):
        from tests import test_batch_interaction as fixtures
        from learning.artifacts import ArtifactStore
        from types import SimpleNamespace
        f=fixtures.BatchInteractionTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        self.checked,self.binding=f.checked()
        runtime=object.__new__(ModelDecisionRuntime)
        runtime.repository=f.repository;runtime.plan=f.plan;runtime.spec=f.spec;runtime.clock=f.clock
        runtime.config=copy.deepcopy(f.plan['runtime'])
        runtime.config['conditionBindings']={'close':self.binding}
        runtime.config['beforeReviewer']=dict(kind='file',protocolVersion=1,maxWaitSeconds=0)
        self.runtime=runtime
        proof=f.verify();self.external=proof.report['frameBindings']['external'][0]
        self.pair={'before':{'frame':proof.report['frameBindings']['gripper'][0]}}
        self.evidence={'source':{'frame':self.external}}
        self.store=object.__new__(ArtifactStore);self.store.root=f.root
        self.manager=SimpleNamespace(store=self.store,camera=SimpleNamespace(root=f.root/'external'),
            current={'sessionId':f.scope['hostClockId']},port=SimpleNamespace(last_sequence=101,off_verified=True))

    def respond(self,request,*,wrong_hash=False):
        envelope=copy.deepcopy(self.f.envelope);envelope['outputSequence']=101
        response=dict(requestSha256='wrong' if wrong_hash else digest(request),review=envelope)
        (self.f.root/'batch-before-review-response.json').write_bytes(encoded(response))

    def call(self,*,wrong_hash=False,respond=True):
        original=self.store.write_json
        def write(name,value):
            original(name,value)
            if name=='batch-before-review-request.json' and respond:self.respond(value,wrong_hash=wrong_hash)
        with patch.object(self.store,'write_json',side_effect=write), \
             patch.object(self.runtime,'_legacy_snapshot',return_value={'root':'existing-byte-verified-fixture'}):
            return self.runtime._before_binding(self.manager,self.pair,self.f.condition,self.evidence)

    def test_stage_output_change_requires_new_review_and_accepts_current_originals(self):
        from learning import batch_interaction as interaction
        with self.assertRaisesRegex(ValueError,'intervening_output'):
            interaction.verify_review(self.checked,now=self.f.clock.now,output_sequence=101)
        binding=self.call()
        checked=interaction.validate_binding(self.f.plan,self.f.condition,binding,self.f.repository)
        token=interaction.verify_review(checked,now=self.f.clock.now,output_sequence=101)
        self.assertEqual(token.outputSequence,101)
        self.assertEqual(token.report['contactState'],'unknown')
        self.assertFalse(token.report['tableContactReady'])
        self.assertGreaterEqual(len(token.snapshots),8)

    def test_missing_current_response_blocks(self):
        with self.assertRaisesRegex(ValueError,'not_supplied'):self.call(respond=False)

    def test_wrong_current_response_blocks(self):
        with self.assertRaisesRegex(ValueError,'request_changed'):self.call(wrong_hash=True)

    def test_declaration_is_not_dispatch_evidence(self):
        from learning import batch_interaction as interaction
        checked=interaction.validate_binding(self.f.plan,self.f.condition,
            dict(route='generic',interaction='contact_probe'),self.f.repository,purpose='declaration')
        with self.assertRaisesRegex(ValueError,'verified_review_required'):
            interaction.verify_review(checked,now=self.f.clock.now,output_sequence=101)


class ThreeStageBackendTests(unittest.TestCase):
    setUp = BatchModelRuntimeTests.setUp
    """Sequence integration with injected device backend and real original review verifier."""
    def _three_stage(self, *, damage_retention=False, omit_after_reviewer=False):
        from learning import batch_interaction as interaction
        from learning.batch_campaign import digest as plan_digest
        from tests.test_batch_interaction import BatchConfirmedContactTests
        proof=BatchConfirmedContactTests();proof.setUp();self.addCleanup(proof.doCleanups)
        proof.condition['command']=dict(joint='gripper',direction='close',percent=40,durationMs=350)
        proof.envelope['stage']='after'
        close=interaction.after_command(proof.checked(purpose='label',route='generic'),
            now=20.,output_sequence=100,after_off=10.).report
        proof.held_envelope()
        if damage_retention:
            (proof.root/'retention.json').write_bytes(b'changed-original')
        models={'wrist':self.model};stages=[self.stage];conditions={'approach':[self.condition]}
        runtimes={'approach':self.runtime};clock=self.runtime.clock
        for kind,joint,direction,percent,duration in [('close','gripper','close',40,350),('lift','shoulder','up',60,200)]:
            condition=copy.deepcopy(self.condition);condition.update(id=kind)
            condition['command']=dict(joint=joint,direction=direction,percent=percent,durationMs=duration)
            rows=[dict(sessionId=kind+str(i),episodeId=str(i),sourceHash=str(i)*64,frameIds=[kind+str(i)+'a',kind+str(i)+'b'],
                captureGroup='train',joint=joint,context=self.model['context'],stateFields=self.model['stateFields'],
                before=[float(i)]*4,after=[float(i)+2]*4,command=condition['command']) for i in (1,3)]
            model=fit(rows);models[kind]=model
            stage=dict(self.stage,id=kind,kind=kind,modelId=kind,commands=[condition['command']])
            runtime=copy.copy(self.runtime);runtime.clock=clock;runtime.root=self.f.private/kind
            runtime.plan=copy.deepcopy(runtime.plan);runtime.plan['conditions']=[condition]
            runtime.config=copy.deepcopy(runtime.config)
            runtime.config['conditionBindings']={kind:dict(route='m4_up' if kind=='lift' else 'generic',interaction='grasped' if kind=='lift' else 'contact_probe')}
            if not omit_after_reviewer:
                runtime.config['afterReviewer']=dict(kind='file',protocolVersion=1,maxWaitSeconds=3)
            runtime.plan['runtime']=runtime.config;runtime.plan_sha=plan_digest(runtime.plan)
            runtime.model=model;runtime.executions=0;runtime.observations=0
            # Device backend adapters are injected; scope/file validation is covered
            # separately by BatchRuntime/M4 tests. Candidate and transition logic stays real.
            runtime.validate_condition=lambda c:None
            runtime._model=lambda joint,m=model:m;runtime._m4_model=lambda m=model:m
            original_observe=runtime.observe_model
            runtime.observe_model=lambda m,m4=False,call=original_observe,k=kind:dict(call(m,m4=m4),
                frameIds=[k+'-'+str(clock.monotonic())])
            original_execute=runtime.execute
            def execute(c,request_id,*,deadline_wall,call=original_execute,k=kind):
                result=call(c,request_id,deadline_wall=deadline_wall)
                if k=='close':review=close
                else:
                    review=interaction.verify_review(proof.checked(purpose='label'),now=16.,
                        output_sequence=100,after_off=10.).report
                result['interactionAfter']=dict(review,status='reviewed')
                return result
            runtime.execute=execute
            stages.append(stage);conditions[kind]=[condition];runtimes[kind]=runtime
        runner=BatchModelRunner('three-stage',stages,runtimes,conditions,models,self.f.private/'three-stage')
        result=runner.run(confirmed_scope_sha256=runner.scope_sha256)
        return runner,result,runtimes

    def test_contact_sequence_without_live_after_review_fails_before_device_access(self):
        with self.assertRaisesRegex(ValueError,'live_after_reviewer_required'):
            self._three_stage(omit_after_reviewer=True)
        self.assertEqual(self.runtime.executions,0)
        self.assertEqual(self.runtime.observations,0)

    def test_three_stage_backend_with_original_contact_and_retention_reviews(self):
        runner,result,runtimes=self._three_stage()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(result['completedStages'],3)
        self.assertEqual([runtimes[k].executions for k in ('approach','close','lift')],[1,1,1])
        self.assertEqual([v['request']['stageId'] for v in runner.sequence.history],['approach','close','lift'])
        self.assertFalse(result['physicalSuccessClaimed'])

    def test_three_stage_tampered_retention_does_not_finish_or_repeat(self):
        runner,result,runtimes=self._three_stage(damage_retention=True)
        self.assertEqual(result['status'],'halted',result)
        self.assertEqual(result['completedStages'],2)
        self.assertEqual(runtimes['lift'].executions,1)
        self.assertFalse(result['physicalSuccessClaimed'])
        with self.assertRaises(FileExistsError):runner.run(confirmed_scope_sha256=runner.scope_sha256)

if __name__=='__main__':unittest.main()
