"""특수 M4 위 경로의 exact scope·원본 감사·기한·재개 검사. 실제 장치 없음."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning import batch_m4 as m
from learning import m4_up_commissioning as core
from learning.artifacts import ArtifactStore
from learning.supervised_collection import PROFILES,plan_digest
from learning.supervised_transport import validate_pulse
from tests.test_m4_up_commissioning import scope,Observer,Device
from tests.test_supervised_transport import Clock
from tests import test_m4_lift_data as lift_fixtures
from tests import test_batch_observation as observation_fixtures


class WallClock(Clock):
    def time(self):return self.now+1000


def condition():
    return dict(id='m4-one',command={k:core.PULSE[k] for k in ('joint','direction','percent','durationMs')},
                target=1,maxAttempts=1,phase='training',captureGroup='train',fromNodes=['shoulder-node'])


def spec_for(planned):
    fixture=observation_fixtures.BatchObservationTests();fixture.setUp();spec=fixture.spec
    joint=copy.deepcopy(spec['joints']['wrist']);joint['nodes'][0]['id']='shoulder-node'
    spec.update(cameraIdentity=planned['context']['externalDeviceId'],viewId=planned['context']['externalViewId'],
                poseRegime=planned['context']['poseRegime'],object=planned['context']['object'],
                joints={'shoulder':joint})
    return spec


class BindingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.repo=Path(self.tmp.name);self.local=self.repo/'.local';self.local.mkdir()
        self.scope=scope();helper=self.local/'helper';helper.write_bytes(b'local-helper')
        path=self.local/'scope.json';path.write_text(json.dumps(self.scope))
        self.binding=dict(route='m4_up',interaction='grasped',scopePath=str(path),
            scopeSha256=m.sha(path.read_bytes()),camera=dict(helperPath=str(helper),
            helperSha256=m.sha(helper.read_bytes()),cameraUrl='http://camera.invalid'))
        self.plan=dict(controllerBootId=self.scope['controllerBootId'],viewId=self.scope['context']['externalViewId'],maxObservationAgeSeconds=5,
                       runtime=dict(controllerUrl=self.scope['controllerUrl']))
        self.condition=condition()

    def test_exact_special_route_does_not_enable_general_m4_up(self):
        self.assertNotIn('up',PROFILES['shoulder']['directions'])
        with self.assertRaises(ValueError):validate_pulse(core.PULSE)
        self.assertTrue(m.validate_route_command(self.condition['command'],self.binding))
        checked=m.validate_binding(self.plan,self.condition,self.binding,self.repo)
        self.assertEqual(checked['scope'],self.scope)
        with self.assertRaises(ValueError):validate_pulse(core.PULSE)

    def test_wrong_pulse_and_contact_and_scope_hash_rejected(self):
        altered=copy.deepcopy(self.condition);altered['command']['durationMs']=201
        with self.assertRaisesRegex(ValueError,'exact_command'):m.validate_route_command(altered['command'],self.binding)
        self.binding['interaction']='contact_probe'
        with self.assertRaisesRegex(ValueError,'interaction'):m.validate_binding(self.plan,self.condition,self.binding,self.repo)
        self.binding['interaction']='grasped';self.binding['scopeSha256']='0'*64
        with self.assertRaisesRegex(ValueError,'scope_file'):m.validate_binding(self.plan,self.condition,self.binding,self.repo)

    def test_one_scope_cannot_cover_multiple_attempts(self):
        self.condition['maxAttempts']=2
        with self.assertRaisesRegex(ValueError,'one_attempt'):m.validate_binding(self.plan,self.condition,self.binding,self.repo)

    def test_retention_capture_budget_is_explicit_and_bounded(self):
        self.binding['retentionCaptures']=dict(count=2,intervalSeconds=1)
        m.validate_binding(self.plan,self.condition,self.binding,self.repo)
        self.binding['retentionCaptures']['count']=3
        with self.assertRaisesRegex(ValueError,'retention_configuration'):
            m.validate_binding(self.plan,self.condition,self.binding,self.repo)

    def test_boot_controller_and_view_are_pinned(self):
        for key,value in [('controllerBootId','c'*32),('viewId','different')]:
            planned=copy.deepcopy(self.plan);planned[key]=value
            with self.assertRaisesRegex(ValueError,'context_changed'):
                m.validate_binding(planned,self.condition,self.binding,self.repo)
        planned=copy.deepcopy(self.plan);planned['runtime']['controllerUrl']='http://other.invalid'
        with self.assertRaisesRegex(ValueError,'context_changed'):
            m.validate_binding(planned,self.condition,self.binding,self.repo)

    def test_missing_reviewer_and_expired_deadline_never_construct_core(self):
        clock=WallClock()
        route=m.BatchM4Route(self.plan,self.repo,self.local/'run',lambda:'unused',
                            observer_spec=spec_for(self.scope),clock=clock)
        with patch.object(core,'M4UpCommissioning',side_effect=AssertionError('no core')):
            with self.assertRaisesRegex(ValueError,'explicit_reviewer'):
                route.execute(self.condition,'one',self.binding,deadline_wall=clock.time()+10)
            route.reviewer_factory=lambda *_:lambda *_:None
            with self.assertRaisesRegex(ValueError,'deadline'):
                route.execute(self.condition,'one',self.binding,deadline_wall=clock.time())

    def test_existing_scope_is_consumed_by_core_not_resubmitted(self):
        clock=WallClock();device=Device(clock)
        patched=patch.object(core,'REPO',self.repo);patched.start();self.addCleanup(patched.stop)
        observers=[]
        def observer(planned,config,store,reviewer,clock):
            value=Observer(store.root,clock,planned);observers.append(value);return value
        original=core._CommissioningPort.__init__
        def port(instance,*args,**kwargs):original(instance,*args,**kwargs);instance.opener=device
        route=m.BatchM4Route(self.plan,self.repo,self.local/'run',lambda:'unused',lambda *_:lambda *_:None,
                            observer_spec=spec_for(self.scope),clock=clock)
        with patch.object(m,'M4DualCameraObserver',side_effect=observer),patch.object(core._CommissioningPort,'__init__',port),\
             patch.object(core,'decode',side_effect=lambda raw,size,digest:observers[-1].decode(raw,size,digest)):
            outcome=route.execute(self.condition,'one',self.binding,deadline_wall=clock.time()+60)
            count=sum(payload=={'action':'run'} for _,payload in device.requests)
            self.assertEqual(count,1)
            self.assertEqual(outcome['execution'],'completed')
            # Minimal fake observer lacks full native-close summary: execution is
            # retained, while a valid training transition is correctly refused.
            self.assertEqual(outcome['observation'],'rejected')
            with self.assertRaisesRegex(ValueError,'request_consumed'):
                route.execute(self.condition,'one',self.binding,deadline_wall=clock.time()+60)
            with self.assertRaisesRegex(ValueError,'result_not_durable'):
                route.execute(self.condition,'two',self.binding,deadline_wall=clock.time()+60)
            self.assertEqual(sum(payload=={'action':'run'} for _,payload in device.requests),count)

    def test_campaign_deadline_is_checked_during_core_preparation(self):
        clock=WallClock();device=Device(clock);patcher=patch.object(core,'REPO',self.repo)
        patcher.start();self.addCleanup(patcher.stop)
        def observer(planned,config,store,reviewer,clock):return Observer(store.root,clock,planned)
        original=core._CommissioningPort.__init__
        def port(instance,*args,**kwargs):original(instance,*args,**kwargs);instance.opener=device
        route=m.BatchM4Route(self.plan,self.repo,self.local/'run',lambda:'unused',lambda *_:lambda *_:None,
                            observer_spec=spec_for(self.scope),clock=clock)
        with patch.object(m,'M4DualCameraObserver',side_effect=observer),patch.object(core._CommissioningPort,'__init__',port):
            route.execute(self.condition,'expired-during-preflight',self.binding,deadline_wall=clock.time()+.001)
        self.assertFalse(any(payload=={'action':'run'} for _,payload in device.requests))

    def test_batch_age_tighter_than_scope_rejects_actual_run(self):
        clock=WallClock();device=Device(clock);self.plan['maxObservationAgeSeconds']=.001
        patched=patch.object(core,'REPO',self.repo);patched.start();self.addCleanup(patched.stop)
        observers=[]
        def observer(planned,config,store,reviewer,clock):
            value=Observer(store.root,clock,planned);observers.append(value);return value
        original=core._CommissioningPort.__init__
        def port(instance,*args,**kwargs):original(instance,*args,**kwargs);instance.opener=device
        route=m.BatchM4Route(self.plan,self.repo,self.local/'run',lambda:'unused',lambda *_:lambda *_:None,
                            observer_spec=spec_for(self.scope),clock=clock)
        with patch.object(m,'M4DualCameraObserver',side_effect=observer),patch.object(core._CommissioningPort,'__init__',port),\
             patch.object(core,'decode',side_effect=lambda raw,size,digest:observers[-1].decode(raw,size,digest)):
            outcome=route.execute(self.condition,'stale',self.binding,deadline_wall=clock.time()+60)
        self.assertFalse(any(payload=={'action':'run'} for _,payload in device.requests))
        self.assertFalse(outcome.get('accepted',False))
        self.assertEqual(outcome['execution'],'not_dispatched')
        proof_path=self.local/'run/m4-scopes/stale/batch-dispatch-proof.json'
        proof=json.loads(proof_path.read_bytes());proof['mayHaveRun']=True
        proof_path.write_text(json.dumps(proof))
        self.assertEqual(route.reconcile(self.condition,'stale',self.binding)['execution'],'unknown')
        self.assertFalse(any(payload=={'action':'run'} for _,payload in device.requests))


class SourceAuditTests(unittest.TestCase):
    def setUp(self):
        self.fixture=lift_fixtures.LiftDataTests();self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.root=self.fixture.root
        planned=self.fixture.read('scope.json')
        for key in ('phase','modelPath','modelSha256','observationContractSha256'):planned.pop(key)
        planned.update(schemaVersion=core.SCHEMA,maxObservedDeltaDeg=1.)
        self.fixture.write('scope.json',planned)
        result=self.fixture.read('result.json');result.update(schemaVersion=core.SCHEMA,scopeSha256=plan_digest(planned))
        self.fixture.write('result.json',result);self.result_sha=m.sha((self.root/'result.json').read_bytes())
        self.planned=planned

    def audit(self):return m.audit_source(self.root,self.result_sha)

    def test_complete_dedicated_raw_is_reaudited_without_fake_session(self):
        audited=self.audit()
        self.assertEqual(audited['execution']['status'],'verified')
        self.assertEqual(audited['execution']['command']['direction'],'up')
        self.assertFalse((self.root/'session.json').exists())
        self.assertEqual(set(audited['pairs']),{'before','after'})
        self.assertFalse(audited['physicalSuccessClaimed'])

    def test_changed_raw_and_missing_off_and_excursion_rejected(self):
        with self.assertRaisesRegex(ValueError,'result_hash'):m.audit_source(self.root,'0'*64)
        result=self.fixture.read('result.json');result['angleObservations'][0]['angleDeg']=200.
        self.fixture.write('result.json',result);self.result_sha=m.sha((self.root/'result.json').read_bytes())
        with self.assertRaisesRegex(ValueError,'angle_evidence'):self.audit()
        result['angleObservations'][0]['angleDeg']=195.;result['finalIdleOffVerified']=False
        self.fixture.write('result.json',result);self.result_sha=m.sha((self.root/'result.json').read_bytes())
        with self.assertRaisesRegex(ValueError,'not_completed'):self.audit()

    def test_contact_unknown_remains_unknown_and_is_not_jaw_contact(self):
        planned=copy.deepcopy(self.planned);planned.update(schemaVersion=core.CONTACT_PROBE_SCHEMA,contactState='unknown')
        self.fixture.write('scope.json',planned)
        result=self.fixture.read('result.json');result.update(schemaVersion=core.CONTACT_PROBE_SCHEMA,
            status='probe_completed_contact_unverified',scopeSha256=plan_digest(planned))
        self.fixture.write('result.json',result);self.result_sha=m.sha((self.root/'result.json').read_bytes())
        with self.assertRaisesRegex(ValueError,'contact_semantics'):self.audit()
        record=self.fixture.read('before-observation.json');record['onsite']['jawContactVerified']=False
        evidence={k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
        self.fixture.write(record['evidenceFile'],evidence)
        record['evidenceSha256']=m.sha((self.root/record['evidenceFile']).read_bytes())
        self.fixture.write('before-observation.json',record)
        self.assertEqual(self.audit()['scope']['contactState'],'unknown')

    def test_generic_transition_rebuilt_from_dedicated_audit(self):
        audited=self.audit();spec=spec_for(self.planned)
        def observed(image,metadata,spec,context,now,clock_id):
            return dict(status='accepted',specSha256=m.observer_digest(spec),frameBinding=metadata,
                nodeMatches=['shoulder-node'],states={'shoulder':dict(state=[1.,2.,3.,4.],
                stateFields=['elbowx','elbowy','tipx','tipy'],nodeMatches=['shoulder-node'])})
        with patch.object(m,'observe_frame',side_effect=observed):
            transition=m.make_transition(self.root,audited,spec,condition())
            row,provenance=m.audit_transition_source(self.root,self.result_sha,transition)
            self.assertEqual(row,transition['learningRow'])
            self.assertEqual(provenance['sourceKind'],'m4_scope')
            altered=copy.deepcopy(transition);altered['learningRow']['before'][0]=999
            with self.assertRaisesRegex(ValueError,'transition_changed'):
                m.audit_transition_source(self.root,self.result_sha,altered)


    def test_retention_samples_preserve_base_and_legacy_snapshot_sources(self):
        from types import SimpleNamespace
        from learning.gripper_camera_dataset import read_frame
        audited=self.audit();scope=audited['scope'];clock=WallClock()
        original_files={name:(self.root/name).read_bytes() for name in ('before-observation.json',
                       'after-observation.json','gripper/esp32-after.jpg','gripper/esp32-after.json','result.json')}
        base=json.loads(original_files['gripper/esp32-after.json']);clock.now=base['persistedAt']
        parent=self
        class ExtraSource:
            def __init__(owner):
                owner.store=ArtifactStore(parent.root/'gripper-source');owner._bindings={};owner.count=2
            def capture(owner,*,deadline):
                parent.assertLess(clock.monotonic(),deadline);owner.count+=1
                folder=f'capture-{owner.count:04d}';meta=copy.deepcopy(base);t=clock.monotonic()
                meta.update(requestStartedAt=t,receivedAt=t+.01,persistedAt=t+.02)
                stamp=owner.count*10000
                meta['headers'].update({'X-MotionBrain-Captured-Us':str(stamp),
                    'X-MotionBrain-Capture-Requested-Us':str(stamp-1),'X-MotionBrain-Frame-Id':'b'*32+':'+str(stamp)})
                owner.store._write(folder+'/esp32-frame.jpg',original_files['gripper/esp32-after.jpg'])
                owner.store.write_json(folder+'/esp32-frame.json',meta)
                frame,_,_=read_frame(owner.store.root/folder,'frame',scope['hostClockId'])
                owner._bindings[frame['frameId']]=dict(folder=folder)
                clock.now=t+.02
                return dict(frame=frame,sourceEarliest=t,persistedAt=t+.02)
            def verify(owner,record):
                folder=owner._bindings[record['frame']['frameId']]['folder']
                frame,_,_=read_frame(owner.store.root/folder,'frame',scope['hostClockId'])
                parent.assertEqual(frame,record['frame'])
        class FakeObserver:
            def __init__(owner):owner.gripper=ExtraSource();owner.recorder=SimpleNamespace(root=parent.root/'external')
            def _wait(owner,predicate,deadline,tick):
                while predicate() is None:
                    tick();clock.sleep(.05);parent.assertLess(clock.monotonic(),deadline)
            def _external(owner,not_before,deadline,tick):
                tick()
                frame=next(f for f in audited['result']['observerClose']['native']['frames'] if f['sourceEarliest']>=not_before)
                clock.now=max(clock.now,frame['stdoutReadAt']+.2);return copy.deepcopy(frame)
        observed=FakeObserver();ticks=[]
        result=m.capture_retention(observed,scope,dict(count=2,intervalSeconds=1),self.fixture.store,
                                  deadline=clock.now+10,tick=lambda:ticks.append(clock.now),clock=clock)
        self.assertEqual((result['status'],len(result['entries']),observed.gripper.count),('recorded',2,4))
        previous_external=audited['pairs']['after']['external']
        for sample in result['entries']:
            self.assertGreaterEqual(sample['external']['sourceEarliest']-previous_external['sourceLatest'],1.)
            previous_external=sample['external']
        extra=m.audit_retention_source(self.root,audited)
        self.assertEqual(extra['status'],'recorded');self.assertEqual(extra['physicalOutcome'],'unreviewed')
        self.assertTrue(ticks)
        for name,raw in original_files.items():self.assertEqual((self.root/name).read_bytes(),raw)
        # The snapshot bytes work with unchanged legacy frame validators.
        for entry in extra['entries']:
            root=(self.root/entry['snapshotRoot']).resolve()
            external=core._external_frame(root,entry['sources']['external'],scope)
            gripper=core._gripper_frame(root,entry['sources']['gripper'],scope,'after')
            self.assertEqual(external['frameId'],entry['external']['frameId'])
            self.assertEqual(gripper['frameId'],entry['gripper']['frameId'])
        snapshot=self.root/'retention-01/gripper/esp32-after.jpg'
        snapshot.write_bytes(snapshot.read_bytes()+b'x')
        with self.assertRaises(ValueError):m.audit_retention_source(self.root,audited)

    def test_generic_prospective_evaluation_binds_before_model_and_timing(self):
        from learning.batch_learning import fit,prospective_evidence
        audited=self.audit();spec=spec_for(self.planned);c=condition();c.update(phase='evaluation',captureGroup='eval')
        def observed(image,metadata,spec,context,now,clock_id):
            return dict(status='accepted',specSha256=m.observer_digest(spec),frameBinding=metadata,
                nodeMatches=['shoulder-node'],states={'shoulder':dict(state=[1.,2.,3.,4.],
                stateFields=['elbowx','elbowy','tipx','tipy'],nodeMatches=['shoulder-node'])})
        context={**{k:spec[k] for k in ('viewId','poseRegime','object','load')},'observerSha256':m.observer_digest(spec)}
        training=[]
        for i in (0,2):
            training.append(dict(sessionId='train'+str(i),episodeId='boot:'+str(i),sourceHash=format(i+1,'064x'),
                frameIds=[str(i)+'-before',str(i)+'-after'],captureGroup='train',joint='shoulder',context=context,
                stateFields=['elbowx','elbowy','tipx','tipy'],before=[float(i+j) for j in range(4)],
                after=[float(i+j+1) for j in range(4)],command=c['command']))
        model=fit(training)
        self.fixture.write('batch-frozen-model.json',model)
        frame=audited['pairs']['before']['frame']
        row=dict(sessionId=self.planned['trialId'],before=[1.,2.,3.,4.],context=context,command=c['command'],
                 beforePersistedAt=frame['stdoutReadAt']+frame['persistenceMs']/1000)
        prepared=(row['beforePersistedAt']+audited['result']['receipt']['dispatchBefore'])/2
        proof=prospective_evidence(model,row,prepared)
        self.fixture.write('batch-prospective.json',dict(beforeFrameId=frame['frameId'],
            beforePixelSha256=frame['pixelSha256'],proof=proof))
        with patch.object(m,'observe_frame',side_effect=observed):
            transition=m.make_transition(self.root,audited,spec,c)
            self.assertTrue(transition['evaluation']['prospectiveEvidenceVerified'])
            self.assertEqual(transition['evaluation']['kind'],'prospectiveFrozenEvaluation')
            m.audit_transition_source(self.root,self.result_sha,transition)
            proof['preparedAt']=audited['result']['receipt']['dispatchBefore']+1
            self.fixture.write('batch-prospective.json',dict(beforeFrameId=frame['frameId'],
                beforePixelSha256=frame['pixelSha256'],proof=proof))
            with self.assertRaisesRegex(ValueError,'prospective_not_verified'):
                m.audit_transition_source(self.root,self.result_sha,transition)

    def test_m4_scope_sources_train_cli_then_frozen_development_evaluation(self):
        from tools import train_batch_campaign as cli
        import hashlib
        repo=Path(self.fixture.temp.name)/'batch-repository';local=repo/'.local';local.mkdir(parents=True)
        manifest=dict(schemaVersion=cli.SCHEMA,training=[],evaluation=[])
        fixtures=[]
        def observed(image,metadata,spec,context,now,clock_id):
            return dict(status='accepted',specSha256=m.observer_digest(spec),frameBinding=metadata,
                nodeMatches=['shoulder-node'],states={'shoulder':dict(state=[1.,2.,3.,4.],
                stateFields=['elbowx','elbowy','tipx','tipy'],nodeMatches=['shoulder-node'])})
        class UniqueFixture(lift_fixtures.LiftDataTests):
            def gripper_fixture(owner,label,t):
                value=super().gripper_fixture(label,t)
                metadata=owner.read('gripper/esp32-'+label+'.json')
                metadata['headers']['X-MotionBrain-Camera-Boot-Id']=owner.scope['context']['gripperCameraBootId']
                stamp=metadata['headers']['X-MotionBrain-Captured-Us']
                metadata['headers']['X-MotionBrain-Frame-Id']=owner.scope['context']['gripperCameraBootId']+':'+stamp
                owner.write('gripper/esp32-'+label+'.json',metadata)
                value['metadataSha256']=m.sha((owner.root/('gripper/esp32-'+label+'.json')).read_bytes())
                return value
        # The only substituted component is the color observer; native camera,
        # HTTP/output, review, scope, hashing, fitting and splitting are real paths.
        with patch.object(m,'observe_frame',side_effect=observed):
            for i,split in enumerate(('training','training','evaluation')):
                boot=format(i+11,'032x');camera_boot=format(i+21,'032x')
                def planned_scope(i=i,boot=boot,camera_boot=camera_boot):
                    value=scope();value.update(trialId='m4-source-'+str(i),hostClockId='source-clock-'+str(i),controllerBootId=boot)
                    value['context']['gripperCameraBootId']=camera_boot
                    return value
                class UniqueDevice(Device):
                    def __init__(owner,clock):super().__init__(clock);owner.boot=boot
                f=UniqueFixture()
                with patch.object(lift_fixtures,'scope',side_effect=planned_scope),patch.object(lift_fixtures,'Device',UniqueDevice):
                    f.setUp()
                self.addCleanup(f.doCleanups);fixtures.append(f)
                original=f.root;destination=local/('source-'+str(i));original.rename(destination);f.root=destination
                planned=f.read('scope.json')
                for k in ('phase','modelPath','modelSha256','observationContractSha256'):planned.pop(k)
                planned.update(schemaVersion=core.SCHEMA,maxObservedDeltaDeg=1.)
                f.write('scope.json',planned)
                result=f.read('result.json');result.update(schemaVersion=core.SCHEMA,scopeSha256=plan_digest(planned))
                f.write('result.json',result)
                result_sha=m.sha((destination/'result.json').read_bytes())
                audited=m.audit_source(destination,result_sha)
                c=condition();c.update(id='condition-'+str(i),phase=split,captureGroup='train' if split=='training' else 'eval')
                # For this offline evaluation-only source, the command was not
                # predicted prospectively. Preserve that by generating a training
                # transition then declaring an explicit development replay split.
                build_condition=copy.deepcopy(c);build_condition['phase']='training'
                t=m.make_transition(destination,audited,spec_for(planned),build_condition)
                if split=='evaluation':
                    # Dedicated M4 phases require prospective evidence; keep this
                    # source for a subsequent frozen runtime evaluation instead.
                    manifest['evaluation']=[]
                    evaluation_source=(destination,audited,planned,c,result_sha)
                    continue
                path=local/('transition-'+str(i)+'.json');path.write_bytes(m.encoded(t))
                manifest['training'].append(dict(sourceKind='m4_scope',transitionPath=str(path),
                    transitionSha256=m.sha(path.read_bytes()),sourcePath=str(destination),sourceSha256=result_sha,
                    scopeSha256=t['scopeSha256']))
            manifest_path=local/'train.json';manifest_path.write_text(json.dumps(manifest))
            report=cli.run_manifest(manifest_path,local/'model-output',repository=repo)
            self.assertEqual((report['trainingRows'],len(report['models'])),(2,1))
            binding=report['models'][0];model_path=local/'model-output'/binding['path']
            model=json.loads(model_path.read_bytes())
            destination,audited,planned,c,result_sha=evaluation_source
            before=m._feature(destination,audited['pairs']['before'],spec_for(planned),planned['hostClockId'])
            row=dict(sessionId=planned['trialId'],before=before['observation']['states']['shoulder']['state'],
                     context=model['context'],command=c['command'],beforePersistedAt=before['derivedMetadata']['persistedAt'])
            prepared=(row['beforePersistedAt']+audited['result']['receipt']['dispatchBefore'])/2
            from learning.batch_learning import prospective_evidence
            proof=prospective_evidence(model,row,prepared)
            (destination/'batch-frozen-model.json').write_bytes(m.encoded(model))
            (destination/'batch-prospective.json').write_bytes(m.encoded(dict(proof=proof,
                beforeFrameId=before['derivedMetadata']['frameId'],
                beforePixelSha256=audited['pairs']['before']['frame']['pixelSha256'])))
            transition=m.make_transition(destination,audited,spec_for(planned),c)
            self.assertTrue(transition['evaluation']['prospectiveEvidenceVerified'])
            path=local/'eval-transition.json';path.write_bytes(m.encoded(transition))
            evaluation=dict(schemaVersion=cli.SCHEMA,frozenModels=[dict(path=str(model_path),sha256=binding['sha256'])],
                evaluation=[dict(sourceKind='m4_scope',transitionPath=str(path),transitionSha256=m.sha(path.read_bytes()),
                    sourcePath=str(destination),sourceSha256=result_sha,scopeSha256=transition['scopeSha256'])])
            manifest_path=local/'eval.json';manifest_path.write_text(json.dumps(evaluation))
            result=cli.run_manifest(manifest_path,local/'eval-output',repository=repo)
            self.assertEqual(result['models'][0]['evaluation']['evaluated'],1)
            self.assertEqual(result['models'][0]['sha256'],binding['sha256'])
            self.assertFalse(result['controlEligible'])
            helper=local/'helper';helper.write_bytes(b'local-helper')
            binding=dict(route='m4_up',interaction='grasped',scopePath=str(destination/'scope.json'),
                scopeSha256=m.sha((destination/'scope.json').read_bytes()),camera=dict(helperPath=str(helper),
                helperSha256=m.sha(helper.read_bytes()),cameraUrl='http://camera.invalid'))
            plan=dict(controllerBootId=planned['controllerBootId'],viewId=planned['context']['externalViewId'],
                      maxObservationAgeSeconds=5,runtime=dict(controllerUrl=planned['controllerUrl']))
            route=m.BatchM4Route(plan,repo,local/'runtime',lambda:None,observer_spec=spec_for(planned),frozen_model=model)
            runtime_result=route._classify(c,'m4-evaluation',binding,destination)
            self.assertEqual((runtime_result['execution'],runtime_result['observation']),('completed','eligible'))
            self.assertTrue(runtime_result['prospectiveEvidenceVerified'])
            self.assertEqual(len(runtime_result['dataIdentity']['pairedFrameIds']),2)
            failed=json.loads((destination/'result.json').read_bytes());failed.update(status='failed',
                errors=[dict(phase='after_observation',type='ValueError',code='m4_observed_excursion')])
            (destination/'result.json').write_bytes(m.encoded(failed))
            failure=route._classify(c,'failed-evaluation',binding,destination)
            self.assertEqual((failure['execution'],failure['observation']),('completed','rejected'))
            self.assertEqual((failure['failureScope'],failure['blockedJoint']),('joint','shoulder'))
            self.assertEqual(json.loads((destination/'result.json').read_bytes())['status'],'failed')


if __name__=='__main__':unittest.main()
