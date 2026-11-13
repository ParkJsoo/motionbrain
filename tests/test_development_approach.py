"""제한된 개발 접근의 합성 회귀 검사. 실제 카메라·모터 요청은 없다."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import cv2

from learning.artifacts import ArtifactStore
from learning.development_approach import (ADMISSION_SCHEMA, ApproachModelBank,
    ApproachObserver, DevelopmentApproachTrial)
from learning.development_gripper_observer import DevelopmentGripperObserver
from learning.development_gripper_source import scene_context
from learning.grasp_observation import GripperObservationSource
from learning.gripper_camera_dataset import POLICY
from learning.model_grasp_trial import APPROACH_SCHEMA, FrozenModelBank, ModelGraspTrial
from learning.relative_action_learning import fit, state_fields
from learning.relative_model_bundle import sha256
from learning.supervised_collection import plan_digest
from learning.supervised_transport import (ELBOW_DEADLINE_POLICY, ELBOW_PLATEAU_POLICY,
                                           WRIST_READ_BUDGET_REVISION)
from tests.test_development_gripper_observer import CONTEXT, POINTS, ROIS, image
from tests.test_grasp_observation import Camera, Clock
from tests.test_supervised_transport import Store

COMMAND = dict(joint='elbow', direction='up', percent=60, durationMs=350)
BOOT = 'b'*32


class Guard:
    def __init__(self, store, bank, port):
        self.store,self.count,self.port=store,0,port
        self.repository=bank.root
        self.config_binding=copy.deepcopy(bank.admission['externalGuard'])
        self.config=copy.deepcopy(bank.external_guard_config)
    def check(self): pass
    def validate_command(self, command): pass
    def disarm(self): pass
    def review(self, record, result, *, deadline, tick):
        tick(); self.count += 1
        name=f'guard-{self.count}.json'
        proof=dict(accepted=True,observationSha256=plan_digest([record,result]))
        descriptor=self.store.write_json(name,proof)
        return dict(proof,evidence=dict(path=name,sha256=descriptor['sha256']))


class SyntheticObserver(ApproachObserver):
    def __init__(self, bank, clock, states):
        self.bank, self.clock = bank, clock
        self.states = copy.deepcopy(states)
        self.cache = None
        self.count = 0
        self.closed = False
        self.guard = Guard(Store(),bank,None)

    def start(self, *, deadline): pass
    def check(self): pass
    def observe(self, stage, *, after_off, deadline, tick):
        if after_off is not None:
            self.clock.sleep(max(0., after_off+3.1-self.clock.monotonic()))
        self.clock.sleep(.1)
        self.count += 1
        state = self.states.pop(0) if len(self.states)>1 else self.states[0]
        frame = dict(frameId=f'fresh-{self.count}', pixelSha256='c'*64, width=320, height=240)
        at = self.clock.monotonic()
        self.cache = dict(frame=frame, annotation=dict(method='development_visible_state', state=state),
            context=self.bank.context('approach'), hostClockId='host', stable=True,
            sourceEarliest=at-.03, sourceLatest=at-.02, receivedAt=at-.01, persistedAt=at)
        tick()
        return copy.deepcopy(self.cache)
    def state(self, annotation, frame): return copy.deepcopy(annotation['state'])
    def latest(self, stage): return copy.deepcopy(self.cache)
    def close(self): self.closed=True; return dict(status='completed')


class Port:
    def __init__(self, clock):
        self.clock = clock
        self.boot_id = 'a'*32
        self.calls = []
        self.may_have_run = False
        self.fail = False
        self.bad_policy = False
    def preflight(self): pass
    def check_idle(self, **kwargs): pass
    def finish(self): return True
    def execute(self, pulse, tick, *, before_run, monitor_policy):
        self.clock.sleep(.11)
        before_run()
        self.calls.append((copy.deepcopy(pulse), monitor_policy))
        self.may_have_run = True
        if self.fail: raise TimeoutError('ambiguous RUN')
        at = self.clock.monotonic(); self.clock.sleep(pulse['durationMs']/1000)
        return dict(requestedPulse=pulse, motor=pulse['motor'], offVerified=True,
            dispatchBefore=at, dispatchAfter=at+.01, offAt=self.clock.monotonic(),
            outputTrace=[dict(motor=3,pwm=pulse['expectedPwm']),dict(motor=3,pwm=0)],
            monitorTimingPolicy='wrong' if self.bad_policy else monitor_policy,
            monitorStatusVerified=True, monitorReadBudgetRevision=WRIST_READ_BUDGET_REVISION)


class ApproachTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.clock = Clock()
        seed = cv2.imencode('.jpg',image())[1].tobytes()
        (self.root/'seed.jpg').write_bytes(seed)
        ref = dict(id='seed',imageBytes=seed,imageSha256=sha256(seed),points=POINTS,rois=ROIS)
        observer = DevelopmentGripperObserver(CONTEXT,[ref])
        self.scene = scene_context(observer)
        observer_binding = dict(schemaVersion='motionbrain.development_gripper_source_binding.v1',
            context=CONTEXT, configurationSha256=observer.binding,
            references=[dict(id='seed',path='seed.jpg',sha256=sha256(seed),points=POINTS,rois=ROIS)])
        self.observer_binding = self.save('observer.json',observer_binding)
        context = dict(self.scene,cameraIdentity=dict(camera='gripper',cameraBootId='d'*32,
            timestampSemantics='first_dma_buffer',observationPolicy=POLICY),imageShape=[240,320],
            stateFields=state_fields(self.scene),executionTimingPolicy=ELBOW_DEADLINE_POLICY)
        rows=[]
        for i,state in enumerate(([0.,10.,100.],[20.,90.,120.],[10.,50.,110.])):
            rows.append(dict(sessionId=f'train-{i}',captureGroup='training',context=context,
                command=COMMAND,before=state,after=[state[0],state[1]-2,state[2]],
                episode=['oldboot',f'train-{i}'],frameIds=[f'pre-{i}',f'post-{i}'],
                developmentObservationOnly=True,recoveryDevelopmentOnly=True))
        model = fit(rows,allow_development_observations=True)
        self.model_binding = self.save('model.json',model)
        self.scope = dict(schemaVersion=APPROACH_SCHEMA,trialId='synthetic-approach',hostClockId='host',
            controllerBootId='a'*32,modelHashes={'approach':self.model_binding['sha256']},
            stages=[dict(id='approach',kind='position',modelId='approach',target=[10.,20.,110.],
                tolerancePx=[.1,.1,.1],commands=[COMMAND],maxCommands=4)],maxCommands=4,
            maxRequestedMs=1400,maxSeconds=60.,maxObservationAgeSeconds=2.,
            maxDecisionAgeSeconds=.25,postOffSeconds=3.,retentionSeconds=1.)
        self.review = dict(hostClockId='host',controllerBootId='a'*32,cameraBootId=BOOT,
            actualScene=self.scene,reviewer='synthetic-review-only',externalFrame=self.file('external.jpg',b'external'),
            gripperFrame=self.file('review-gripper.jpg',seed),sceneMatched=True,noncontactClearanceVerified=True)
        self.external_guard_config=dict(schemaVersion='motionbrain.approach_external_guard.v1',
            controllerBootId=self.scope['controllerBootId'],hostClockId=self.scope['hostClockId'],
            gripperCameraBootId=BOOT,executionTimingPolicy=ELBOW_DEADLINE_POLICY)
        self.guard_binding=self.save('external-guard.json',self.external_guard_config)
        self.admission = dict(schemaVersion=ADMISSION_SCHEMA,model=self.model_binding,
            observer=self.observer_binding,scope=self.scope,sceneReview=self.save('review.json',self.review),
            externalGuard=self.guard_binding,validUntil=500.,currentCameraBootId=BOOT,minimumState=[0.,0.,90.],maximumState=[30.,100.,130.],
            stopping=dict(minimumPredictedImprovementPx=.1,minimumObservedProgressPx=.25,maxNoProgress=2,
                revisitTolerancePx=.25,maximumPredictionResidualPx=4.))
        self.admission_binding=self.save('admission.json',self.admission)

    def file(self,name,raw):
        (self.root/name).write_bytes(raw); return dict(path=name,sha256=sha256(raw))
    def save(self,name,value): return self.file(name,json.dumps(value).encode())
    def bank(self): return ApproachModelBank(self.root,self.admission_binding)
    def resave(self): self.admission_binding=self.save('admission.json',self.admission)
    def trial(self, states):
        bank=self.bank(); observer=SyntheticObserver(bank,self.clock,states);port=Port(self.clock)
        observer.guard.port=port
        scope=dict(self.scope,admissionSha256=self.admission_binding['sha256'])
        trial=DevelopmentApproachTrial(scope,bank,observer,port,Store(),evidence_root=self.root,clock=self.clock)
        return trial,port,observer
    def run_trial(self,states,**port_changes):
        trial,port,observer=self.trial(states)
        for k,v in port_changes.items(): setattr(port,k,v)
        result=trial.run(confirmed_scope_sha256=plan_digest(trial.scope))
        return result,port,observer

    def test_default_loader_still_refuses_development_model_and_position_only_scope(self):
        with self.assertRaisesRegex(ValueError,'development_model_not_admitted'):
            FrozenModelBank([dict(id='approach',path=self.root/'model.json',sha256=self.model_binding['sha256'])])
        bank=self.bank(); observer=SyntheticObserver(bank,self.clock,[[10.,26.,110.]])
        with self.assertRaisesRegex(ValueError,'grasp_scope_bindings'):
            ModelGraspTrial(self.scope,bank,observer,Port(self.clock),Store(),evidence_root=self.root,clock=self.clock)

    def test_current_boot_mapping_keeps_original_model_and_context_separate(self):
        bank=self.bank(); choice=bank.choose(self.scope['stages'][0],[10.,26.,110.],bank.context('approach'))
        self.assertEqual(choice['selected']['kind'],'action')
        self.assertEqual(choice['sourceModelContext']['cameraIdentity']['cameraBootId'],'d'*32)
        self.assertEqual(choice['actualObservationContext']['cameraIdentity']['cameraBootId'],BOOT)
        self.assertFalse(choice['controlEligible'])

    def test_external_guard_contract_identity_must_match_model_scope_and_current_boot(self):
        original=copy.deepcopy(self.external_guard_config)
        for field,value in [('controllerBootId','f'*32),('hostClockId','another-host'),
                            ('gripperCameraBootId','f'*32),('executionTimingPolicy',ELBOW_PLATEAU_POLICY)]:
            config=copy.deepcopy(original);config[field]=value
            self.admission['externalGuard']=self.save('external-guard.json',config);self.resave()
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'external_guard_contract_changed'):
                self.bank()

    def test_live_guard_binding_config_boot_and_execution_port_cannot_be_substituted(self):
        observer,camera=self.real_observer()
        guard=observer.guard
        original_binding=copy.deepcopy(guard.config_binding)
        original_config=copy.deepcopy(guard.config)
        original_port=guard.port
        # 같은 내용의 다른 파일도 승인된 파일 바인딩을 대신할 수 없다.
        foreign=self.save('foreign-guard.json',original_config)
        for binding,config,port in [
            (foreign,original_config,original_port),
            (dict(original_binding,sha256='f'*64),original_config,original_port),
            (original_binding,dict(original_config,hostClockId='other'),original_port)]:
            guard.config_binding,guard.config,guard.port=binding,config,port
            with self.subTest(binding=binding,config=config),self.assertRaisesRegex(ValueError,'external_guard_binding_changed'):
                ApproachObserver(observer.bank,observer.source,guard,clock=self.clock)
        guard.config_binding=original_binding;guard.config=original_config;guard.port=original_port
        original_port.boot_id='f'*32
        with self.assertRaisesRegex(ValueError,'external_guard_binding_changed'):observer.check()
        original_port.boot_id='a'*32
        scope=dict(self.scope,admissionSha256=self.admission_binding['sha256'])
        with self.assertRaisesRegex(ValueError,'guard_execution_port_changed'):
            DevelopmentApproachTrial(scope,observer.bank,observer,Port(self.clock),Store(),
                                     evidence_root=self.root,clock=self.clock)

    def test_replacing_guard_port_after_trial_construction_stops_before_execution(self):
        trial,port,observer=self.trial([[10.,26.,110.],[10.,24.,110.]])
        replacement=Port(self.clock)
        self.assertEqual(replacement.boot_id,port.boot_id)
        observer.guard.port=replacement
        result=trial.run(confirmed_scope_sha256=plan_digest(trial.scope))
        self.assertEqual(result['status'],'failed',result)
        self.assertEqual(result['errors'][0]['reason'],'approach_guard_execution_port_changed')
        self.assertEqual(result['commandAttempts'],0)
        self.assertEqual(observer.count,0)
        self.assertEqual(port.calls,[])
        self.assertEqual(replacement.calls,[])
        self.assertFalse(result['automaticRetry'])
        self.assertTrue(result['finalIdleOffVerified'])

    def test_hashed_admission_and_review_change_are_refused(self):
        (self.root/'admission.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'artifact_changed'): self.bank()
        self.resave();(self.root/'external.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'reviewed_frame_changed'): self.bank()

    def test_wrong_scene_boot_command_budget_or_contract_refused(self):
        original=copy.deepcopy(self.admission)
        changes=[lambda a:a.update(currentCameraBootId='e'*32),
            lambda a:a['scope']['stages'][0].update(kind='close'),
            lambda a:a['scope']['stages'][0]['commands'][0].update(durationMs=450),
            lambda a:a['scope'].update(maxCommands=5),
            lambda a:a['scope']['stages'][0].update(target=[99.,20.,110.])]
        for change in changes:
            self.admission=copy.deepcopy(original);change(self.admission);self.resave()
            with self.subTest(change=change),self.assertRaises(ValueError):self.bank()

    def test_three_reobserved_steps_reach_target_with_fixed_timing_and_no_grasp_claim(self):
        result,port,observer=self.run_trial([[10.,26.,110.],[10.,24.,110.],[10.,22.,110.],[10.,20.,110.]])
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual(result['modelSelectedMotorCommands'],3)
        self.assertEqual(observer.count,4)
        self.assertTrue(observer.closed)
        self.assertEqual([p for _,p in port.calls],[ELBOW_DEADLINE_POLICY]*3)
        self.assertTrue(result['finalIdleOffVerified'])
        self.assertFalse(result['graspVerified']);self.assertFalse(result['liftVerified'])

    def set_profile(self,policy,percent,duration):
        model=json.loads((self.root/'model.json').read_text())
        model['context']['executionTimingPolicy']=policy
        command=dict(COMMAND,percent=percent,durationMs=duration)
        for entry in model['models']:entry['command']=copy.deepcopy(command)
        self.model_binding=self.save('model.json',model)
        self.scope['modelHashes']={'approach':self.model_binding['sha256']}
        self.scope['stages'][0]['commands']=[command]
        self.scope['maxRequestedMs']=self.scope['maxCommands']*duration
        self.external_guard_config['executionTimingPolicy']=policy
        self.guard_binding=self.save('external-guard.json',self.external_guard_config)
        self.admission.update(model=self.model_binding,scope=self.scope,externalGuard=self.guard_binding)
        self.resave()

    def test_650_model_passes_its_exact_fixed_output_policy_and_receipt(self):
        self.set_profile(ELBOW_PLATEAU_POLICY,35,650)
        result,port,_=self.run_trial([[10.,26.,110.],[10.,24.,110.],[10.,22.,110.],[10.,20.,110.]])
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual(result['requestedMs'],1950)
        self.assertEqual([policy for _,policy in port.calls],[ELBOW_PLATEAU_POLICY]*3)
        self.assertTrue(all(pulse['percent']==35 and pulse['durationMs']==650 for pulse,_ in port.calls))

    def test_current_scene_reference_is_explicit_and_350_only(self):
        from learning.approach_external_guard import CURRENT_REFERENCE_MEANING
        self.external_guard_config.update(schemaVersion='motionbrain.current_scene_external_guard.v1',
                                          referenceMeaning=CURRENT_REFERENCE_MEANING)
        self.admission['externalGuard']=self.save('external-guard.json',self.external_guard_config)
        self.resave();self.bank()
        self.external_guard_config['referenceMeaning']='automatic_goal_transfer'
        self.admission['externalGuard']=self.save('external-guard.json',self.external_guard_config)
        self.resave()
        with self.assertRaisesRegex(ValueError,'current_scene_guard_contract_changed'):self.bank()
        self.external_guard_config['referenceMeaning']=CURRENT_REFERENCE_MEANING
        self.set_profile(ELBOW_PLATEAU_POLICY,35,650)
        with self.assertRaisesRegex(ValueError,'current_scene_guard_contract_changed'):self.bank()

    def test_known_timing_policy_cannot_be_paired_with_different_output_profile(self):
        self.set_profile(ELBOW_PLATEAU_POLICY,60,650)
        with self.assertRaisesRegex(ValueError,'unsupported_monitor_timing_policy'):self.bank()
        self.set_profile('unknown-elbow-policy',35,650)
        with self.assertRaises(ValueError):self.bank()

    def test_high_frequency_idle_ticks_rate_limit_HTTP_but_check_observation_each_tick(self):
        trial,port,observer=self.trial([[10.,26.,110.]])
        trial.started=self.clock.monotonic();trial.deadline=trial.started+60.
        polls=[];checks=[]
        port.check_idle=lambda **kwargs:polls.append((self.clock.monotonic(),kwargs))
        observer.check=lambda:checks.append(self.clock.monotonic())
        for _ in range(20):
            trial._idle_tick();self.clock.sleep(.05)
        self.assertEqual(len(checks),20)
        self.assertGreaterEqual(len(polls),2)
        self.assertLessEqual(len(polls),3)
        self.assertTrue(all(b[0]-a[0]>=.5 for a,b in zip(polls,polls[1:])))
        flags=dict(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True,
                   allow_quiet_read_recovery=True)
        self.assertTrue(all(kwargs==flags for _,kwargs in polls))
        self.assertEqual(port.calls,[])
        observer.check=lambda:(_ for _ in ()).throw(ValueError('fresh_observation_lost'))
        before=len(polls)
        with self.assertRaisesRegex(ValueError,'fresh_observation_lost'):trial._idle_tick()
        self.assertEqual(len(polls),before)
        self.clock.now=trial.deadline
        observer.check=lambda:None
        with self.assertRaisesRegex(ValueError,'trial_deadline'):trial._idle_tick()
        self.assertEqual(len(polls),before)

    def test_active_ticks_never_issue_HTTP_idle_checks(self):
        trial,port,observer=self.trial([[10.,26.,110.]])
        trial.started=self.clock.monotonic();trial.deadline=trial.started+60.
        polls=[];checks=[]
        port.check_idle=lambda **kwargs:polls.append(kwargs)
        observer.check=lambda:checks.append(self.clock.monotonic())
        for _ in range(20):
            trial._active_tick();self.clock.sleep(.05)
        self.assertEqual(len(checks),20)
        self.assertEqual(polls,[])
        self.assertEqual(port.calls,[])

    def test_unsupported_state_abstains_without_command(self):
        result,port,_=self.run_trial([[25.,26.,110.]])
        self.assertEqual(result['status'],'abstained',result)
        self.assertEqual(port.calls,[])
        self.assertEqual(result['commandAttempts'],0)
        rejected=result['stages'][0]['decision']['rejected']
        self.assertEqual(rejected[0]['reason'],'state_outside_training_range')

    def test_ambiguous_execution_never_retries(self):
        result,port,_=self.run_trial([[10.,26.,110.]],fail=True)
        self.assertEqual(result['status'],'failed');self.assertEqual(len(port.calls),1)
        self.assertFalse(result['automaticRetry']);self.assertEqual(result['modelSelectedMotorCommands'],0)

    def test_wrong_timing_receipt_stops_before_new_command(self):
        result,port,_=self.run_trial([[10.,26.,110.],[10.,24.,110.]],bad_policy=True)
        self.assertEqual(result['status'],'failed');self.assertEqual(len(port.calls),1)
        self.assertEqual(result['errors'][0]['reason'],'approach_output_timing_contract_changed')

    def test_no_progress_revisit_residual_and_budget_stop(self):
        for states,status,calls in [
            ([[10.,26.,110.]]*3,'observed_no_progress',2),
            ([[10.,26.,110.],[10.,24.,110.],[10.,26.,110.]],'observed_revisit',2),
            ([[10.,30.,110.],[10.,34.,110.]],'prediction_residual_exceeded',1),
            ([[10.,30.,110.],[10.,29.,110.],[10.,28.,110.],[10.,27.,110.],[10.,26.,110.]],'budget_reached',4)]:
            self.clock.now=50.
            with self.subTest(status=status):
                result,port,_=self.run_trial(states)
                self.assertEqual(result['status'],status,result);self.assertEqual(len(port.calls),calls)

    def test_noncontact_envelope_rejects_observation_and_prediction(self):
        result,port,_=self.run_trial([[31.,26.,110.]])
        self.assertEqual(result['status'],'failed');self.assertEqual(port.calls,[])
        self.admission['minimumState'][1]=19.;self.scope['stages'][0]['target'][1]=19.
        self.resave();bank=self.bank()
        choice=bank.choose(self.scope['stages'][0],[10.,20.,110.],bank.context('approach'))
        self.assertEqual(choice['selected']['kind'],'stop')
        self.assertEqual(choice['rejected'][0]['reason'],'approach_predicted_noncontact_envelope_exceeded')

    def test_expired_contract_or_wrong_scope_hash_cannot_start(self):
        trial,port,_=self.trial([[10.,26.,110.]])
        self.clock.now=450.
        with self.assertRaisesRegex(ValueError,'expired'):trial.run(confirmed_scope_sha256=plan_digest(trial.scope))
        self.assertEqual(port.calls,[])
        self.clock.now=50.
        with self.assertRaisesRegex(ValueError,'exact_grasp_scope'):trial.run(confirmed_scope_sha256='0'*64)
        self.assertEqual(port.calls,[])

    def real_observer(self):
        bank=self.bank();camera=Camera(self.clock);camera.boot=BOOT
        im=image();im[165:206,150:175]=200
        # 두 턱의 Y가 달라도 모델의 수평 폭 계약을 그대로 사용해야 한다.
        x0,y0,x1,y1=ROIS['jawRight'];patch=im[y0:y1,x0:x1].copy()
        im[y0:y1,x0:x1]=200;im[y0+6:y1+6,x0:x1]=patch
        cv2.rectangle(im,(151,164),(175,204),(120,40,240),-1)
        camera.body=cv2.imencode('.jpg',im)[1].tobytes()
        store=ArtifactStore(self.root/'camera-run')
        source=GripperObservationSource('http://camera.invalid','host',BOOT,self.scene,store,
            clock=self.clock,opener=camera)
        return ApproachObserver(bank,source,Guard(store,bank,Port(self.clock)),clock=self.clock),camera

    def test_real_development_source_preserves_horizontal_span_and_current_boot(self):
        observer,camera=self.real_observer();observer.start(deadline=110.)
        record=observer.observe(self.scope['stages'][0],after_off=None,deadline=110.,tick=lambda:None)
        state=observer.state(record['annotation'],record['frame'])
        self.assertEqual(state[2],104)
        self.assertEqual(record['context']['cameraIdentity']['cameraBootId'],BOOT)
        self.assertEqual(camera.calls[0][1],'GET')
        changed=copy.deepcopy(record['annotation']);changed['observationSha256']='f'*64
        with self.assertRaisesRegex(ValueError,'observation_changed'):observer.state(changed,record['frame'])
        camera.regress=True
        with self.assertRaisesRegex(ValueError,'reused_or_regressed'):
            observer.observe(self.scope['stages'][0],after_off=None,deadline=110.,tick=lambda:None)

    def test_frozen_model_scope_and_admission_tampering_cannot_start(self):
        trial,port,_=self.trial([[10.,26.,110.]])
        trial.bank._model['models'][0]['deltaMean'][1]=-20.
        with self.assertRaisesRegex(ValueError,'frozen_contract_changed'):
            trial.run(confirmed_scope_sha256=plan_digest(trial.scope))
        self.assertEqual(port.calls,[])
        trial,port,_=self.trial([[10.,26.,110.]])
        trial.scope['maxCommands']=1
        with self.assertRaisesRegex(ValueError,'exact_admitted_scope'):
            trial.run(confirmed_scope_sha256=plan_digest(trial.scope))
        self.assertEqual(port.calls,[])
        trial,port,_=self.trial([[10.,26.,110.]])
        (self.root/'model.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'artifact_changed'):
            trial.run(confirmed_scope_sha256=plan_digest(trial.scope))
        self.assertEqual(port.calls,[])

    def test_saved_guard_proof_must_match_returned_acceptance(self):
        observer,camera=self.real_observer()
        original=observer.guard.review
        def unrelated_proof(record,result,**kwargs):
            proof=original(record,result,**kwargs)
            descriptor=observer.source.store.write_json('unrelated.json',{})
            proof['evidence']=dict(path='unrelated.json',sha256=descriptor['sha256'])
            return proof
        observer.guard.review=unrelated_proof
        observer.start(deadline=110.)
        with self.assertRaisesRegex(ValueError,'approach_scene_proof_content_changed'):
            observer.observe(self.scope['stages'][0],after_off=None,deadline=110.,tick=lambda:None)
        self.assertIsNone(observer.cache)

    def test_partial_source_never_becomes_accepted_trial_input(self):
        observer,camera=self.real_observer();camera.body=cv2.imencode('.jpg',image()*0)[1].tobytes()
        observer.start(deadline=110.)
        with self.assertRaisesRegex(ValueError,'observation_incomplete'):
            observer.observe(self.scope['stages'][0],after_off=None,deadline=110.,tick=lambda:None)
        self.assertIsNone(observer.cache)


if __name__=='__main__':unittest.main()
