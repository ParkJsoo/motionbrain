"""factory 조립을 오프라인 검증한다. 실제 장치/촬영/학습 실행은 없다.

고정 모델은 합성 학습 결과다. 포트 수명주기 단위 검사는 이미 별도 검증된
source asset 감사만 분리 주입하며, builder 누락/불일치 거절은 실제 감사를 쓴다.
"""
import copy
import json
from hashlib import sha256
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

from learning import current_cycle_factories as factory
from learning.artifacts import ArtifactStore
from learning.current_full_cycle_source import FullCycleSource
from learning.current_m4_short import current_short_policy,short_task_pulse
from learning.manual_collection import ManualCollection
from learning.supervised_collection import plan_digest
from tests import test_task_cycle_workflow as fixtures
from tests.test_supervised_transport import Clock


class FactoryTests(unittest.TestCase):
    def setUp(self):
        fixture=fixtures.CycleWorkflowTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        for row in fixture.m4:
            row['context']['executionPolicy']=current_short_policy()
            pulse=short_task_pulse(row['command']['direction'])
            row['command']={k:pulse[k] for k in ('joint','direction','percent','durationMs')}
            row['before']['startAngleDeg']=197.22 if pulse['direction']=='up' else 197.85
            row['after']['startAngleDeg']=197.85 if pulse['direction']=='up' else 197.22
        self.bank=fixture.freeze();self.repo=fixture.root.resolve()
        self.asset_root=self.repo/'.local/assets';self.asset_root.mkdir(parents=True)
        self.clock=Clock();self.scope=fixtures.ModelPorts(self.bank).scope
        for stage in self.scope['stages']:
            if stage['kind'] in ('lift','lower'):
                pulse=short_task_pulse('up' if stage['kind']=='lift' else 'down')
                stage['commands']=[{k:pulse[k] for k in ('joint','direction','percent','durationMs')}]
                stage['target']['startAngleDeg']=197.85 if stage['kind']=='lift' else 197.22
        self.stage('release')['target']['expectedM4AngleDeg']=197.22
        ref=self.bind('placeholder.json',dict(static='separately audited by source tests',referenceM4Deg=197.22))
        self.assets=dict(journalPath=str(self.repo/'.local/visual-actions/controller-commands.sqlite'),
            sourceStages={},helper=self.bind('helper',b'synthetic helper, never executed'),
            m4ImplementationSha256=factory.implementation_digest(),phaseMode='source_bound_human',
            phaseProtocol='nonce_seed_anchor_fresh_once_v1')
        for stage in self.scope['stages']:
            ctx={k:stage['context'][k] for k in ('physicalView','object','poseRegime')}
            ctx.update(externalDeviceId='camera-id',externalViewId='test-view',gripperCameraBootId='b'*32,
                       supportMode='gripper_supported' if stage['kind']=='lower' else 'table_supported')
            self.assets['sourceStages'][stage['kind']]=dict(context=stage['context'],sourceContext=ctx,
                registration=ref,guardConfig=ref,modelStateRegistration=ref,stationaryRegistration=ref,
                stationaryExternalReference=ref,stationaryGripperReference=ref)
        self.auth=dict(approved=True,scopeSha256=plan_digest(self.scope),automaticRetry=False,onsiteConfirmed=True)
        self.check=Mock();self.after=Mock(return_value=Mock())
        self.audit_patch=patch.object(factory,'_audit_source_assets');self.audit=self.audit_patch.start();self.addCleanup(self.audit_patch.stop)
        self.owner=self.make()
        phase=SimpleNamespace(bind_source=lambda s:None,entry=Mock())
        self.source=FullCycleSource(self.asset_root,bank=self.bank,idle_factory=self.owner.idle,
            general_factory=self.owner.general_source,phase=phase,clock=self.clock)
        self.source.scope=copy.deepcopy(self.scope)
        self.source.registrations={k:(s,{}) for k,s in self.assets['sourceStages'].items()}
        self.owner.bind_source(self.source)

    def bind(self,name,value):
        raw=value if isinstance(value,bytes) else json.dumps(value).encode()
        (self.asset_root/name).write_bytes(raw)
        return dict(path=name,sha256=sha256(raw).hexdigest())

    def make(self,**changes):
        options=dict(repo_root=self.repo,asset_root=self.asset_root,run_root=self.repo/'.local/run',
            controller_url='http://controller.invalid',camera_url='http://camera.invalid',token='private-token',
            authorization=self.auth,resource_check=self.check,after_reviewer_factory=self.after,clock=self.clock)
        options.update(changes)
        return factory.CurrentRuntimeFactory(self.scope,self.assets,self.bank,**options)

    def stage(self,kind):return next(s for s in self.scope['stages'] if s['kind']==kind)

    def test_constructor_and_preflight_do_not_spawn_capture_or_call_controller(self):
        with patch('subprocess.Popen') as spawn,patch.object(factory.SequencePort,'_call') as http:
            owner=self.make();proof=owner.m4.preflight(self.scope,self.assets)
        self.assertEqual(proof['supportedStages'],['lift','lower']);spawn.assert_not_called();http.assert_not_called()
        self.assertFalse((self.repo/'.local/run').exists())

    def test_general_manager_is_new_unstarted_same_journal_and_finite_recording(self):
        stage=self.stage('close');command=stage['commands'][0]
        first=self.owner.manager(stage,command);second=self.owner.manager(stage,command)
        self.assertIsInstance(first,ManualCollection);self.assertIsNot(first,second)
        self.assertIsNone(first.current);self.assertIsNone(first.camera);self.assertIsNone(first.port)
        self.assertEqual(first.journal_path,second.journal_path)
        self.assertEqual(first.recording_seconds,120);self.assertNotEqual(first.root,second.root)
        self.assertFalse(first.root.exists());self.assertFalse(second.root.exists())

    def test_idle_producer_ids_are_anchor_ids_from_creation_not_relabelled(self):
        with patch('subprocess.Popen') as spawn,patch.object(factory.SequencePort,'_call') as http:
            idle=self.owner.idle(self.stage('lift'))
        self.assertTrue(idle.recorder.host_clock_id.startswith('M4-stationary-anchor-'))
        self.assertEqual(idle.recorder.host_clock_id,idle.source.session_id)
        self.assertIs(idle.recorder.clock,idle.source.clock);self.assertIsNone(idle.recorder.started)
        self.assertLessEqual(idle.source.max_captures,100);self.assertGreaterEqual(idle.source.max_captures,10)
        spawn.assert_not_called();http.assert_not_called()
        with self.assertRaisesRegex(ValueError,'controller_write_forbidden'):
            idle.port._call('/command',dict(cmd='stop'))
        self.check.side_effect=ValueError('resource_expired')
        with self.assertRaisesRegex(ValueError,'resource_expired'):idle.recorder.check()

    def test_unbound_source_and_unlisted_general_command_rejected(self):
        owner=self.make()
        with self.assertRaisesRegex(ValueError,'source_not_bound'):owner.manager(self.stage('close'),self.stage('close')['commands'][0])
        with self.assertRaisesRegex(ValueError,'general_model_command'):
            self.owner.manager(self.stage('close'),dict(joint='gripper',direction='close',percent=20,durationMs=200))

    def test_model_policy_and_parent_authorization_cannot_be_replaced(self):
        with self.assertRaisesRegex(ValueError,'scope_authorization'):self.make(authorization=dict(self.auth,approved=False))
        bad=copy.deepcopy(self.assets);bad['journalPath']=str(self.repo/'.local/other.sqlite')
        with self.assertRaisesRegex(ValueError,'shared_journal'):
            factory.CurrentRuntimeFactory(self.scope,bad,self.bank,repo_root=self.repo,asset_root=self.asset_root,
                run_root=self.repo/'.local/run',controller_url='http://controller.invalid',camera_url='http://camera.invalid',
                token='private-token',authorization=self.auth,resource_check=self.check,after_reviewer_factory=self.after,clock=self.clock)

    def test_m4_refuses_manual_command_replacement_or_pending_general_session(self):
        hooks=dict(prepare=lambda:None,before_run=lambda:None,status_guard=lambda s:None,decisionSha256='e'*64)
        with self.assertRaisesRegex(ValueError,'short_command'):
            self.owner.m4(self.stage('lift'),dict(fixtures.UP),hooks)
        manager=self.owner.manager(self.stage('close'),self.stage('close')['commands'][0])
        manager.current=dict(phase='recording')
        with self.assertRaisesRegex(ValueError,'session_pending'):
            self.owner.m4(self.stage('lift'),self.stage('lift')['commands'][0],hooks)

    def test_builder_lists_missing_models_per_stage_with_no_hardware(self):
        self.audit_patch.stop()
        with patch('subprocess.Popen') as spawn,patch.object(factory.SequencePort,'_call') as http:
            with self.assertRaises(factory.FactoryPreflightError) as result:
                factory.build_cycle_assets(self.asset_root,self.scope,model_bindings={},source_stages={},
                    journal_path=self.assets['journalPath'],helper_binding=self.assets['helper'])
        self.assertEqual(set(result.exception.missing),set(fixtures.STAGES))
        self.assertTrue(all('frozen_model_missing' in v for v in result.exception.missing.values()))
        spawn.assert_not_called();http.assert_not_called()

    def test_builder_real_source_audit_rejects_placeholder_registrations(self):
        self.audit_patch.stop()
        with self.assertRaises(factory.FactoryPreflightError) as result:
            factory.build_cycle_assets(self.asset_root,self.scope,model_bindings=self.bank.bindings,
                source_stages=self.assets['sourceStages'],journal_path=self.assets['journalPath'],helper_binding=self.assets['helper'])
        self.assertEqual(set(result.exception.missing),set(fixtures.STAGES))

    def test_cached_manifest_mutation_and_close_rejected(self):
        self.owner.assets['sourceStages']['lift']['context']['object']='another'
        with self.assertRaisesRegex(ValueError,'manifest_memory'):self.owner.idle(self.stage('lift'))
        self.owner.close();self.assertEqual(self.owner.token,'')
        with self.assertRaisesRegex(ValueError,'contract_changed'):self.owner.preflight(self.scope,self.owner.assets)

    def seed_m4(self):
        from tests.test_m4_up_commissioning import Observer,scope as actor_scope
        stage=self.stage('lift');raw_scope=actor_scope()
        raw_scope.update(hostClockId='M4-stationary-anchor-factory-test',context=self.assets['sourceStages']['lift']['sourceContext'])
        native=Observer(self.repo/'.local/raw-idle',self.clock,raw_scope);native.start()
        raw=native.observe(after_off=None,deadline=20.,tick=lambda:None)
        # The older fixture lacks these acquisition fields; preserve a complete
        # synthetic fresh-GET receipt for the actual stationary anchor validator.
        path=native.root/'gripper/esp32-before.json';meta=json.loads(path.read_text())
        meta.update(method='GET',path='/capture',query={'fresh':'1'});path.write_text(json.dumps(meta))
        raw['gripper']['metadataSha256']=sha256(path.read_bytes()).hexdigest()
        labels=dict(operatorPresent=True,clearance=True,bilateralContact=True,tableSupported=True)
        record=dict(context=stage['context'],hostClockId=self.scope['hostClockId'],controllerBootId=self.scope['controllerBootId'],
            state=dict(objectRisePx=0.,toolRisePx=0.,slipDyPx=0.,startAngleDeg=197.22),m4AngleDeg=197.22,
            frames=[dict(frameId='e',sourceEarliest=self.clock.now-.1),dict(frameId='g',sourceEarliest=self.clock.now-.1)],
            outputSequence=5,auditedAt=self.clock.now,sourceRoute='idle')
        self.source.evidence['synthetic-idle']=dict(stage=stage,route='idle',record=record)
        self.source.phase.entry.return_value=labels;self.source.verify_observation=Mock(return_value=True)
        self.source.describe_source=Mock(return_value=dict(root=str(native.root),external=raw['external'],gripper=raw['gripper']))
        decision=self.bank.choose(stage,dict(record,entry=labels,supportMode='table_supported'))
        self.assertEqual(decision['selected']['kind'],'action')
        hooks=dict(prepare=lambda:None,before_run=lambda:None,status_guard=lambda status:None,decisionSha256=factory.decision_digest(decision))
        return stage,stage['commands'][0],hooks,native

    def test_exact_FrozenBank_decision_builds_real_short_actor_without_running(self):
        stage,command,hooks,native=self.seed_m4()
        from learning import m4_up_commissioning as core
        with patch.object(core,'REPO',self.repo),patch('learning.lossless_frame.decode',side_effect=native.decode),\
             patch('learning.current_m4_review.measure',return_value={}),patch('subprocess.Popen') as spawn,\
             patch.object(factory.SequencePort,'_call') as http:
            actor,digest=self.owner.m4(stage,command,hooks)
        self.assertIsInstance(actor,core.M4UpCommissioning);self.assertIs(actor.task_hooks,hooks)
        self.assertEqual(actor.scope['schemaVersion'],core.CURRENT_SHORT_TASK_SCHEMA)
        self.assertEqual(actor.scope['pulse']['durationMs'],350);self.assertEqual(actor.scope['shortTaskPolicy'],current_short_policy())
        self.assertEqual(digest,plan_digest(actor.scope));self.assertFalse(actor.used)
        self.assertFalse(actor.observer.started);spawn.assert_not_called();http.assert_not_called()
        binding=json.loads((actor.store.root/'parent-cycle-model-binding.json').read_text())
        self.assertEqual(binding['decisionSha256'],hooks['decisionSha256'])

    def test_m4_changed_model_decision_rejected_before_new_output_directory(self):
        stage,command,hooks,_=self.seed_m4();hooks['decisionSha256']='f'*64
        with self.assertRaisesRegex(ValueError,'decision_changed'):self.owner.m4(stage,command,hooks)
        self.assertFalse((self.repo/'.local/run').exists())

    def test_general_source_uses_current_session_guard_and_actual_camera_source(self):
        stage=self.stage('close');manager=self.owner.manager(stage,stage['commands'][0])
        manager.current=dict(sessionId='manual-new-session',phase='recording')
        manager.camera=object();manager.port=object();manager.store=ArtifactStore(manager.root/'session')
        self.source.registrations['close']=(self.assets['sourceStages']['close'],SimpleNamespace(
            registration=dict(sceneContext={k:stage['context'][k] for k in ('physicalView','object','poseRegime')}),
            shaft_landmark_contract='registered-gripper-shaft-only-v1:test'))
        with patch.object(factory,'_CycleM1Guard') as guard_cls:
            source,guard=self.owner.general_source(stage,manager)
        self.assertIsInstance(source,factory.GripperObservationSource);self.assertEqual(source.session_id,'manual-new-session')
        self.assertEqual(source.max_captures,3);self.assertFalse(source.started)
        args,kwargs=guard_cls.call_args
        self.assertIs(args[2],manager.camera);self.assertIs(args[3],manager.port)
        config=json.loads((manager.store.root/'cycle-session-guard.json').read_text())
        self.assertEqual(config['hostClockId'],'manual-new-session')
        self.assertEqual(config['static'],'separately audited by source tests')

    def test_original_OPEN_headroom_is_preserved_before_parent_guard(self):
        guard=object.__new__(factory._CycleM1Guard);guard._cycle_shaft=100.
        with patch.object(factory.CurrentM1CollectionGuard,'validate_command') as parent:
            with self.assertRaisesRegex(ValueError,'open_headroom'):guard.validate_command(dict(direction='open'))
            parent.assert_not_called()
            guard._cycle_shaft=85.15;guard.validate_command(dict(direction='open'));parent.assert_called_once()

    def test_assembly_has_actual_runtime_ports_but_missing_real_assets_rejects_without_IO(self):
        owner=self.make();store=ArtifactStore(self.repo/'.local/run/cycle')
        requests=ArtifactStore(self.repo/'.local/phase-requests');reviews=self.repo/'.local/reviews';reviews.mkdir()
        with patch('subprocess.Popen') as spawn,patch.object(factory.SequencePort,'_call') as http:
            runtime=owner.assemble(review_root=reviews,phase_request_store=requests,store=store)
            self.assertIsInstance(runtime.source,FullCycleSource)
            self.assertIsInstance(runtime.phase,factory.CurrentCyclePhase)
            result=runtime.run(confirmed_scope_sha256=self.auth['scopeSha256'])
        self.assertEqual(result['modelCycleAttempts'],0);self.assertEqual(result['status'],'failed')
        self.assertTrue(any('full_source_missing' in e['reason'] for e in result['errors']))
        self.assertEqual(result['runtimeCounts']['generalSessionsStarted'],0)
        spawn.assert_not_called();http.assert_not_called()

    def rebind_bank(self,bank):
        self.bank=bank;self.scope['stages']=bank.scope_stages(self.scope['stages'])
        for stage in self.scope['stages']:
            kind=stage['kind'];spec=self.assets['sourceStages'][kind];spec['context']=stage['context']
            for key in ('object','physicalView','poseRegime'):spec['sourceContext'][key]=stage['context'][key]
            if kind in ('lift','lower'):
                pulse=factory._execution_pulse('up' if kind=='lift' else 'down',stage['context']['executionPolicy'])
                stage['commands']=[{k:pulse[k] for k in ('joint','direction','percent','durationMs')}]
        self.auth['scopeSha256']=plan_digest(self.scope);self.owner=self.make()
        self.source.bank=bank;self.source.scope=copy.deepcopy(self.scope)
        self.source.registrations={k:(s,{}) for k,s in self.assets['sourceStages'].items()}
        self.owner.bind_source(self.source)

    def test_nonASCII_decision_uses_same_digest_as_ScopedCycleExecutor(self):
        from learning.task_cycle_workflow import FrozenCycleModelBank
        bindings={}
        for kind,model in self.bank.models.items():
            model=copy.deepcopy(model);model['context'].update(object='분홍 프리즘',poseRegime='현재 탁상 자세')
            if kind in ('lift','lower'):
                from learning.task_action_models import fit_m4
                for row in model['trainingRows']:row['context']=model['context']
                model=fit_m4(model['trainingRows'],model['context'])
            ref=self.bind(kind+'-ko-model.json',model)
            bindings[kind]=dict(path=str(self.asset_root/ref['path']),sha256=ref['sha256'])
        self.rebind_bank(FrozenCycleModelBank(bindings,allow_development_approach=True))
        stage,command,hooks,_=self.seed_m4()
        record=self.owner._entry(stage,command,hooks)
        decision=self.bank.choose(stage,record)
        self.assertNotEqual(plan_digest(decision),factory.decision_digest(decision))
        self.assertEqual(hooks['decisionSha256'],factory.decision_digest(decision))

    def test_general_unsupported_profiles_reject_before_manager_ARM(self):
        for kind in ('approach','close','release'):
            planned=copy.deepcopy(self.scope);stage=next(s for s in planned['stages'] if s['kind']==kind)
            stage['commands'][0].update(percent=20,durationMs=200)
            with self.subTest(kind=kind),self.assertRaises(factory.FactoryPreflightError) as result:
                factory._stage_contracts(self.asset_root,planned,self.assets)
            self.assertIn('factory_general_exact_candidates_required',result.exception.missing[kind])
        self.assertFalse((self.repo/'.local/run').exists())

    def test_old_guard_pose_and_wrong_release_goal_reject_before_attempt(self):
        bad=copy.deepcopy(self.assets)
        old=self.bind('old-guard.json',dict(referenceM4Deg=196.69))
        bad['sourceStages']['close']['guardConfig']=old
        with self.assertRaises(factory.FactoryPreflightError) as result:factory._stage_contracts(self.asset_root,self.scope,bad)
        self.assertIn('factory_guard_table_reference_mismatch',result.exception.missing['close'])
        planned=copy.deepcopy(self.scope)
        next(s for s in planned['stages'] if s['kind']=='release')['target']['expectedM4AngleDeg']=196.69
        with self.assertRaises(factory.FactoryPreflightError) as result:factory._stage_contracts(self.asset_root,planned,self.assets)
        self.assertIn('factory_release_table_goal_mismatch',result.exception.missing['release'])

    def test_manager_recorder_uses_explicit_owner_clock_without_starting(self):
        stage=self.stage('close');manager=self.owner.manager(stage,stage['commands'][0])
        plan=dict(deviceId='camera-id',sessionId='manual-test',viewId='test-view',maxFrames=500,
                  seconds=120,sampleFps=4,maxBytes=4200000000,width=1920,height=1080)
        recorder=manager.camera_factory(self.repo/'.local/not-started',self.asset_root/'helper',plan)
        self.assertIs(recorder.clock,self.clock);self.assertIs(recorder.clock,manager.clock)
        self.assertIsNone(recorder.started);self.assertFalse(recorder.root.exists())

    def test_450_policy_is_bound_to_model_candidate_and_actor_without_350_fallback(self):
        from learning import task_action_models as actions,m4_up_commissioning as core
        from learning.task_cycle_workflow import FrozenCycleModelBank
        policy=current_short_policy(450);context=copy.deepcopy(self.bank.models['lift']['context'])
        context['executionPolicy']=policy;rows=fixtures.m4_rows()
        for row in rows:
            up=row['command']['direction']=='up';pulse=short_task_pulse('up' if up else 'down',policy)
            row.update(context=context,command={k:pulse[k] for k in ('joint','direction','percent','durationMs')})
            row['before']['startAngleDeg']=197.22 if up else 197.85
            row['after']['startAngleDeg']=197.85 if up else 197.22
        ref=self.bind('450-model.json',actions.fit_m4(rows,context))
        bindings=copy.deepcopy(self.bank.bindings)
        bindings['lift']=bindings['lower']=dict(path=str(self.asset_root/ref['path']),sha256=ref['sha256'])
        self.rebind_bank(FrozenCycleModelBank(bindings,allow_development_approach=True))
        stage,command,hooks,native=self.seed_m4()
        with patch.object(core,'REPO',self.repo),patch('learning.lossless_frame.decode',side_effect=native.decode),\
             patch('learning.current_m4_review.measure',return_value={}):
            actor,_=self.owner.m4(stage,command,hooks)
        self.assertEqual(actor.scope['pulse']['durationMs'],450)
        self.assertEqual(actor.scope['shortTaskPolicy'],policy)
        self.assertEqual(actor.scope['maxObservedDeltaDeg'],policy['maxObservedDeltaDeg'])
        bad=copy.deepcopy(self.scope)
        next(s for s in bad['stages'] if s['kind']=='lift')['commands'][0]['durationMs']=350
        with self.assertRaises(factory.FactoryPreflightError) as result:factory._stage_contracts(self.asset_root,bad,self.assets)
        self.assertIn('factory_current_short_candidates_required',result.exception.missing['lift'])

    def test_constructor_pinned_model_file_change_is_not_hidden_by_memory_cache(self):
        binding=self.bank.bindings['close'];path=Path(binding['path']);original=path.read_bytes()
        path.write_bytes(original+b' ')
        with self.assertRaisesRegex(ValueError,'frozen_model_file_changed'):
            self.owner.preflight(self.scope,self.assets)
        path.write_bytes(original)
        self.bank.bindings['close']=dict(binding,sha256='f'*64)
        with self.assertRaisesRegex(ValueError,'model_bindings_changed'):
            self.owner.preflight(self.scope,self.assets)

    def test_explicit_effective_model_builds_1200_actor_and_ARM_epoch_without_short_relabel(self):
        from learning import task_action_models as actions,m4_up_commissioning as core
        from learning.m4_effective_policy import effective_policy,effective_task_pulse,SCHEMA
        from learning.task_cycle_workflow import FrozenCycleModelBank
        policy=effective_policy();context=copy.deepcopy(self.bank.models['lift']['context']);context['executionPolicy']=policy
        rows=fixtures.m4_rows()
        for row in rows:
            up=row['command']['direction']=='up';pulse=effective_task_pulse('up' if up else 'down',policy)
            row.update(context=context,command={k:pulse[k] for k in ('joint','direction','percent','durationMs')})
            row['before']['startAngleDeg']=197.22 if up else 197.85
            row['after']['startAngleDeg']=197.85 if up else 197.22
        ref=self.bind('effective-model.json',actions.fit_m4(rows,context));bindings=copy.deepcopy(self.bank.bindings)
        bindings['lift']=bindings['lower']=dict(path=str(self.asset_root/ref['path']),sha256=ref['sha256'])
        self.rebind_bank(FrozenCycleModelBank(bindings,allow_development_approach=True))
        stage,command,hooks,native=self.seed_m4()
        with patch.object(core,'REPO',self.repo),patch('learning.lossless_frame.decode',side_effect=native.decode),\
             patch('learning.current_m4_review.measure',return_value={}),patch.object(core.M4UpCommissioning,'_before_run') as original:
            actor,_=self.owner.m4(stage,command,hooks)
            self.assertEqual(actor.scope['schemaVersion'],SCHEMA);self.assertEqual(actor.scope['effectivePolicy'],policy)
            self.assertNotIn('shortTaskPolicy',actor.scope);self.assertEqual(actor.scope['pulse']['durationMs'],1200)
            with patch.object(actor,'read_armed_off_epoch',return_value={}) as epoch:
                actor.observer.reviewer.read_epoch();epoch.assert_called_once_with(5)
            with patch.object(actor.observer.reviewer,'check_before_run') as memory:
                actor._before_run();original.assert_called_once();memory.assert_called_once()
        bad=copy.deepcopy(self.scope)
        next(s for s in bad['stages'] if s['kind']=='lift')['commands'][0]['durationMs']=600
        with self.assertRaises(factory.FactoryPreflightError):factory._stage_contracts(self.asset_root,bad,self.assets)


if __name__=='__main__':unittest.main()
