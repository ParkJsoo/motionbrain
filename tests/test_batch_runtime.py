import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from learning.batch_campaign import digest
from learning.batch_runtime import BatchRuntime, journal_clear, validate_runtime
from learning.experiment_pipeline import encoded
from tests import test_batch_observation as observation_fixture
from tests import test_manual_collection as manual_fixture


def cli_module():
    path = Path(__file__).resolve().parents[1]/'tools/run_batch_campaign.py'
    spec = importlib.util.spec_from_file_location('batch_cli_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BatchRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.repository = Path(self.tmp.name)
        self.private = self.repository/'.local'
        self.private.mkdir()
        self.helper = self.private/'helper'
        self.helper.write_text('test')
        self.helper.chmod(0o700)
        f = observation_fixture.BatchObservationTests()
        f.setUp()
        self.spec = f.spec
        self.image = f.image
        self.spec_path = self.private/'observer.json'
        self.spec_path.write_bytes(encoded(self.spec))
        self.condition = dict(id='wrist-up', command=dict(joint='wrist', direction='up', percent=20, durationMs=200),
                              fromNodes=['wrist-node'], phase='training', captureGroup='train', target=1, maxAttempts=2)
        self.plan = dict(schemaVersion='motionbrain.batch_campaign.v1', campaignId='one', viewId='fixed',
                         controllerBootId='c'*32, maxObservationAgeSeconds=2,
                         nodes=['wrist-node','elbow-node'],
                         budgets=dict(maxPhysicalCommands=2,maxPreparationCommands=1,maxAttempts=2,maxSeconds=60),
                         conditions=[self.condition], runtime=dict(controllerUrl='http://127.0.0.1', deviceId='camera',
                         helperPath='.local/helper',helperSha256=hashlib.sha256(self.helper.read_bytes()).hexdigest(),
                         poseContext='local',observerSpecPath='.local/observer.json',
                         observerSpecSha256=hashlib.sha256(self.spec_path.read_bytes()).hexdigest(),
                         journalPath='.local/visual-actions/controller-commands.sqlite',
                         dispatchLockPath='.local/batch-locks/'+hashlib.sha256(('c'*32).encode()).hexdigest()+'.lock'))
        self.credentials = []
        self.runtime = BatchRuntime(self.plan,self.repository,self.private/'runtime',self.credential)
        self.addCleanup(self.runtime.close)

    def credential(self):
        self.credentials.append(True)
        return 'private-token'

    def test_plan_and_constructor_do_not_access_devices_credentials_or_create_output(self):
        with patch('learning.batch_runtime.SequencePort',side_effect=AssertionError('device')):
            checked = validate_runtime(self.plan,self.repository)
        self.assertEqual(checked['status'],'runtime_checked_no_device_access')
        self.assertEqual(self.credentials,[])
        self.assertFalse(self.runtime.root.exists())

    def test_changed_helper_and_spec_rejected_before_credentials(self):
        self.helper.write_text('changed')
        with self.assertRaisesRegex(ValueError,'helper_hash_changed'):
            self.runtime.observe()
        self.assertEqual(self.credentials,[])

    def test_object_or_load_requires_explicit_interaction_binding(self):
        for field,value in [('load','held'),('object','cylinder')]:
            spec=copy.deepcopy(self.spec);spec[field]=value
            self.spec_path.write_bytes(encoded(spec))
            plan=copy.deepcopy(self.plan)
            plan['runtime']['observerSpecSha256']=hashlib.sha256(self.spec_path.read_bytes()).hexdigest()
            runtime=BatchRuntime(plan,self.repository,self.private/'interaction-runtime',self.credential)
            self.assertEqual(runtime.condition_support(self.condition),'runtime_explicit_interaction_binding_required')

    def test_optional_broken_frozen_model_only_rejects_evaluation(self):
        plan=copy.deepcopy(self.plan)
        plan['runtime']['frozenModels']={'wrist':dict(path='.local/missing-model.json',sha256='a'*64)}
        runtime=BatchRuntime(plan,self.repository,self.private/'other-runtime',self.credential)
        self.assertIsNone(runtime.condition_support(self.condition))
        condition=copy.deepcopy(self.condition);condition['phase']='evaluation'
        self.assertIn('model_rejected',runtime.condition_support(condition))
        self.assertEqual(self.credentials,[])

    def paired_runtime(self, *, observer=False):
        from tests.test_gripper_camera_dataset import GripperPairTest
        fixture=GripperPairTest();fixture.setUp();self.addCleanup(fixture.doCleanups)
        plan=copy.deepcopy(self.plan)
        plan['runtime']['gripper']=dict(url='http://camera.invalid',bootId='a'*32)
        if observer:
            import cv2
            local=copy.deepcopy(self.spec)
            local.update(cameraIdentity='gripper:'+'a'*32,viewId='fixed:gripper',maxAgeSeconds=10)
            for marker in local['landmarks'].values():
                marker.update(minAreaPx=10,maxAreaPx=100)
            wrapper=dict(kind='camera_local_hsv',spec=local)
            path=self.private/'gripper-observer.json';path.write_bytes(encoded(wrapper))
            plan['runtime']['gripper'].update(observerSpecPath='.local/gripper-observer.json',observerSpecSha256=hashlib.sha256(path.read_bytes()).hexdigest())
            body=cv2.imencode('.jpg',self.image)[1].tobytes()
            for side in ('before','after'):
                (fixture.root/('esp32-'+side+'.jpg')).write_bytes(body)
                path=fixture.root/('esp32-'+side+'.json');metadata=json.loads(path.read_bytes())
                metadata['sha256']=hashlib.sha256(body).hexdigest();path.write_bytes(encoded(metadata))
        runtime=BatchRuntime(plan,self.repository,self.private/'paired-runtime',self.credential)
        self.addCleanup(runtime.close)
        source=runtime.root/'sessions'/'session'
        from learning.artifacts import ArtifactStore
        store=ArtifactStore(source)
        pair=source/'gripper-pair';pair.mkdir()
        for path in fixture.root.iterdir():(pair/path.name).write_bytes(path.read_bytes())
        manager=type('Manager',(),dict(store=store,current={'sessionId':'session'}))()
        from learning.gripper_camera_dataset import read_frame
        frame,_,_=read_frame(pair,'before','session')
        record=dict(path=str(pair),before=dict(frame=dict(frameId=frame['frameId'],pixelSha256=frame['pixelSha256'])))
        runtime.clock=manual_fixture.Clock();runtime.clock.now=13
        runtime._pair_before(manager,record)
        return runtime,source,fixture.row

    def test_gripper_factory_and_static_validation_do_not_capture(self):
        plan=copy.deepcopy(self.plan);plan['runtime']['gripper']=dict(url='http://camera.invalid',bootId='a'*32)
        runtime=BatchRuntime(plan,self.repository,self.private/'paired-runtime',self.credential)
        from learning.artifacts import ArtifactStore
        factory=runtime._gripper_factory()
        camera=factory('session',ArtifactStore(self.private/'source'))
        self.assertFalse(camera.started)
        self.assertEqual(camera.count,0)
        self.assertEqual(self.credentials,[])
        camera.close()

    def test_paired_before_after_binding_and_tamper_rejection(self):
        runtime,source,row=self.paired_runtime()
        paired=runtime._pair_transition(source,row)
        self.assertEqual(paired['before']['frameId'],'a'*32+':100')
        self.assertEqual(paired['after']['frameId'],'a'*32+':200')
        self.assertEqual(len(paired['fileHashes']),4)
        self.assertFalse(paired['exposureSynchronizationVerified'])
        (source/'gripper-pair'/'esp32-after.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'image_hash'):
            runtime._pair_transition(source,row)

    def test_pure_pair_reaudit_recomputes_fused_states_from_original_jpegs(self):
        from learning.batch_runtime import audit_pair_transition
        from learning.batch_observation import digest as observer_digest
        runtime,source,row=self.paired_runtime(observer=True)
        paired=runtime._pair_transition(source,row)
        observed=dict(status='accepted',specSha256=observer_digest(runtime.spec),
            states={'wrist':dict(state=[30,30,60,50])})
        transition=dict(joint='wrist',before=dict(observation=observed,derivedMetadata=dict(clockId=runtime.clock_id)),
            after=dict(observation=copy.deepcopy(observed)),gripperPair=paired)
        expected=audit_pair_transition(source,row,transition,self.repository)
        self.assertEqual(len(expected['stateFields']),8)
        self.assertEqual(len(expected['beforeState']),8)
        self.assertEqual(expected['pairedFrameIds'],['a'*32+':100','a'*32+':200'])
        self.assertIn('gripperObserverSha256',expected['extraContext'])
        changed=copy.deepcopy(transition);changed['gripperPair']['beforeObservation']['states']['wrist']['state'][0]+=1
        with self.assertRaisesRegex(ValueError,'paired_transition_changed'):
            audit_pair_transition(source,row,changed,self.repository)

    def test_camera_local_gripper_features_are_namespaced_and_context_pinned(self):
        runtime,source,row=self.paired_runtime(observer=True)
        paired=runtime._pair_transition(source,row)
        self.assertEqual(paired['beforeObservation']['status'],'accepted',paired['beforeObservation'])
        external=dict(states={'wrist':dict(state=[30,30,60,50])})
        names,values=runtime._states(external,paired['beforeObservation'],'wrist')
        self.assertEqual(len(names),8)
        self.assertEqual(len(values),8)
        self.assertTrue(all(n.startswith('external:') for n in names[:4]))
        self.assertTrue(all(n.startswith('gripper:') for n in names[4:]))
        self.assertIn('gripperObserverSha256',runtime._context())
        self.assertEqual(runtime._context()['gripperCameraIdentity'],'gripper:'+'a'*32)

    def test_explicit_noncontact_object_binding_still_requires_review_and_held_is_not_bypassed(self):
        spec=copy.deepcopy(self.spec);spec['object']='block'
        self.spec_path.write_bytes(encoded(spec))
        plan=copy.deepcopy(self.plan);plan['runtime']['observerSpecSha256']=hashlib.sha256(self.spec_path.read_bytes()).hexdigest()
        plan['runtime']['conditionBindings']={self.condition['id']:dict(route='generic',interaction='noncontact')}
        runtime=BatchRuntime(plan,self.repository,self.private/'object-runtime',self.credential)
        self.assertIn('interaction_rejected',runtime.condition_support(self.condition))
        spec['load']='held';self.spec_path.write_bytes(encoded(spec))
        plan['runtime']['observerSpecSha256']=hashlib.sha256(self.spec_path.read_bytes()).hexdigest()
        runtime=BatchRuntime(plan,self.repository,self.private/'held-runtime',self.credential)
        self.assertIn('interaction_rejected',runtime.condition_support(self.condition))

    def m4_plan(self):
        from tests.test_m4_up_commissioning import scope
        from learning import m4_up_commissioning as m4
        planned=scope();planned['controllerUrl']=self.plan['runtime']['controllerUrl'];planned['controllerBootId']=self.plan['controllerBootId']
        planned['context'].update(externalViewId=self.plan['viewId'],externalDeviceId='camera',poseRegime='local')
        path=self.private/'scope.json';path.write_bytes(encoded(planned))
        spec=copy.deepcopy(self.spec);spec['object']=planned['context']['object']
        spec['joints']['shoulder']=copy.deepcopy(spec['joints']['wrist']);spec['joints']['shoulder']['nodes'][0]['id']='shoulder-node'
        self.spec_path.write_bytes(encoded(spec))
        plan=copy.deepcopy(self.plan);plan['runtime']['observerSpecSha256']=hashlib.sha256(self.spec_path.read_bytes()).hexdigest()
        plan['nodes'].append('shoulder-node')
        condition=copy.deepcopy(self.condition);condition.update(id='m4-test',fromNodes=['shoulder-node'],target=1,maxAttempts=1)
        condition['command']={k:planned['pulse'][k] for k in ('joint','direction','percent','durationMs')}
        binding=dict(route='m4_up',interaction='grasped',scopePath='.local/scope.json',scopeSha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            camera=dict(helperPath='.local/helper',helperSha256=plan['runtime']['helperSha256'],cameraUrl='http://camera.invalid'))
        plan['conditions']=[condition];plan['runtime']['conditionBindings']={condition['id']:binding}
        return plan,condition,binding

    def test_m4_route_requires_explicit_reviewer_and_delegates_without_generic_profile(self):
        plan,condition,binding=self.m4_plan()
        runtime=BatchRuntime(plan,self.repository,self.private/'m4-runtime',self.credential)
        self.assertEqual(runtime.condition_support(condition),'runtime_m4_explicit_reviewer_required')
        runtime.m4_reviewer_factory=lambda scope,store:None
        self.assertIsNone(runtime.condition_support(condition))
        from unittest.mock import Mock
        adapter=Mock();adapter.execute.return_value={'execution':'completed'}
        with patch.object(runtime,'_m4_route',return_value=adapter),patch('learning.batch_runtime.collect_once',side_effect=AssertionError('generic forbidden')):
            import time
            deadline=time.time()+60
            self.assertEqual(runtime.execute(condition,'m4-request',deadline_wall=deadline),{'execution':'completed'})
            adapter.execute.assert_called_once_with(condition,'m4-request',binding,deadline_wall=deadline)
            runtime.reconcile(condition,'m4-request')
            adapter.reconcile.assert_called_once_with(condition,'m4-request',binding)
        self.assertEqual(self.credentials,[])

    def test_duplicate_m4_scope_conditions_are_both_quarantined(self):
        plan,condition,binding=self.m4_plan()
        duplicate=copy.deepcopy(condition);duplicate['id']='m4-duplicate'
        plan['conditions'].append(duplicate);plan['runtime']['conditionBindings'][duplicate['id']]=copy.deepcopy(binding)
        runtime=BatchRuntime(plan,self.repository,self.private/'m4-runtime',self.credential,m4_reviewer_factory=lambda *args:None)
        self.assertEqual(runtime.condition_support(condition),'runtime_m4_scope_reused')
        self.assertEqual(runtime.condition_support(duplicate),'runtime_m4_scope_reused')

    def test_registered_shaft_camera_contract_reuses_existing_observer(self):
        from tests.test_registered_gripper_crosssection import CrosssectionTests
        from learning.registered_gripper_crosssection import register
        import cv2
        fixture=CrosssectionTests();fixture.setUp()
        registration=register(fixture.raw,[166,95],dict(physicalView='local-gripper',object='none',poseRegime='local'))
        regpath=self.private/'reg.json';refpath=self.private/'ref.png'
        regpath.write_bytes(encoded(registration));refpath.write_bytes(fixture.raw)
        spec=dict(kind='registered_gripper_shaft_only',context=registration['sceneContext'],
            registrationPath='.local/reg.json',registrationSha256=hashlib.sha256(regpath.read_bytes()).hexdigest(),
            referenceImagePath='.local/ref.png',referenceImageSha256=hashlib.sha256(refpath.read_bytes()).hexdigest(),maxAgeSeconds=10)
        path=self.private/'shaft.json';path.write_bytes(encoded(spec))
        runtime,source,row=self.paired_runtime()
        config=runtime.plan['runtime']['gripper']
        config.update(observerSpecPath='.local/shaft.json',observerSpecSha256=hashlib.sha256(path.read_bytes()).hexdigest())
        runtime=BatchRuntime(runtime.plan,self.repository,self.private/'shaft-runtime',self.credential)
        runtime.clock=manual_fixture.Clock();runtime.clock.now=13
        body=cv2.imencode('.jpg',fixture.image)[1].tobytes()
        pair=source/'gripper-pair'
        for side in ('before','after'):
            (pair/('esp32-'+side+'.jpg')).write_bytes(body)
            metadata=json.loads((pair/('esp32-'+side+'.json')).read_bytes())
            metadata['sha256']=hashlib.sha256(body).hexdigest();(pair/('esp32-'+side+'.json')).write_bytes(encoded(metadata))
        observed=runtime._observe_gripper(pair,'before','session')
        self.assertEqual(observed['states']['gripper']['stateFields'],['shaftSeparationPx'])
        self.assertGreater(observed['states']['gripper']['state'][0],100)
        self.assertIn('not_actual_tips',observed['semantics'])

    def test_m4_frozen_model_is_external_only_even_when_generic_features_are_fused(self):
        from learning.batch_learning import fit
        from learning.batch_observation import digest as observer_digest
        plan,condition,binding=self.m4_plan();condition['phase']='evaluation';condition['captureGroup']='holdout'
        # Use a dedicated external-only M4 model, separate from generic fused states.
        spec=json.loads(self.spec_path.read_bytes())
        context={**{k:spec[k] for k in ('viewId','poseRegime','object','load')},'observerSha256':observer_digest(spec)}
        rows=[]
        for i in range(2):
            rows.append(dict(sessionId='train'+str(i),episodeId='episode'+str(i),sourceHash=str(i+1)*64,
                frameIds=['before'+str(i),'after'+str(i)],captureGroup='training',joint='shoulder',context=context,
                stateFields=[f['name'] for f in spec['joints']['shoulder']['stateFields']],
                before=[29+2*i,29+2*i,59+2*i,49+2*i],after=[30,30,60,50],command=condition['command']))
        path=self.private/'m4-model.json';path.write_bytes(encoded(fit(rows)))
        plan['runtime']['m4FrozenModel']=dict(path='.local/m4-model.json',sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        local=copy.deepcopy(spec);local.update(cameraIdentity='gripper:'+'b'*32,viewId='fixed:gripper',maxAgeSeconds=10)
        path=self.private/'m4-gripper.json';path.write_bytes(encoded(dict(kind='camera_local_hsv',spec=local)))
        plan['runtime']['gripper']=dict(url='http://camera.invalid',bootId='b'*32,
            observerSpecPath='.local/m4-gripper.json',observerSpecSha256=hashlib.sha256(path.read_bytes()).hexdigest())
        runtime=BatchRuntime(plan,self.repository,self.private/'m4-model-runtime',self.credential,m4_reviewer_factory=lambda *a:None)
        self.assertIsNone(runtime.condition_support(condition))
        self.assertNotIn('gripperObserverSha256',runtime._m4_model()['context'])
        self.assertIn('gripperObserverSha256',runtime._context())

    def test_supplemental_snapshot_is_readable_by_unchanged_legacy_source_validators(self):
        from tests.test_m4_up_commissioning import Observer,scope
        from tests.test_supervised_transport import Clock
        from learning import m4_up_commissioning as legacy
        source=self.runtime.root/'sessions'/'one';source.mkdir(parents=True)
        planned=scope();planned['hostClockId']='one'
        clock=Clock();observer=Observer(source,clock,planned);observer.start()
        record=observer.observe(after_off=clock.now-4,deadline=clock.now+10,tick=lambda:None)
        frame=json.loads((source/'external'/record['external']['ingressFile']).read_bytes())
        grip=source/'gripper-camera'/'capture-0003';grip.mkdir(parents=True)
        for suffix in ('jpg','json'):(grip/('esp32-frame.'+suffix)).write_bytes((source/'gripper'/('esp32-after.'+suffix)).read_bytes())
        self.runtime.config['deviceId']=planned['context']['externalDeviceId']
        self.runtime.plan['viewId']=planned['context']['externalViewId']
        self.runtime.config['gripper']=dict(bootId=planned['context']['gripperCameraBootId'])
        decoder=lambda raw,length,digest:observer.payloads[raw]
        with patch('learning.batch_runtime.decode',side_effect=decoder),patch.object(legacy,'decode',side_effect=decoder):
            snapshot=self.runtime._legacy_snapshot(source,source/'external',frame,grip,1)
            checked=self.runtime._audit_legacy_snapshot(source,snapshot,'one')
            self.assertEqual(checked['external']['frameId'],frame['frameId'])
            self.assertEqual(checked['gripper']['frameId'],'b'*32+':1000')
            descriptor=json.loads((source/snapshot['root']/'source.json').read_bytes())
            self.assertNotIn('onsite',descriptor)
            self.assertNotIn('stable',descriptor)
            self.assertFalse(descriptor['physicalJudgmentGenerated'])
            (source/snapshot['root']/'gripper/esp32-after.jpg').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'snapshot_source_changed'):
                self.runtime._audit_legacy_snapshot(source,snapshot,'one')

    def test_retention_callback_uses_existing_source_four_captures_and_actual_spacing(self):
        from tests.test_grasp_observation import Camera
        from learning.grasp_observation import GripperObservationSource
        from learning.artifacts import ArtifactStore
        from types import SimpleNamespace
        clock=manual_fixture.Clock();clock.time=lambda:clock.now
        self.runtime.clock=clock
        self.runtime.deadline_wall=self.runtime.deadline_mono=clock.now+20
        self.runtime.config['gripper']=dict(url='http://camera.invalid',bootId='a'*32,retentionCaptures=dict(count=2,intervalSeconds=1))
        store=ArtifactStore(self.runtime.root/'sessions'/'retention-callback')
        camera=Camera(clock)
        source=GripperObservationSource('http://camera.invalid','retention-callback','a'*32,
            dict(physicalView='fixed',object='none',poseRegime='local',landmarks='raw'),
            ArtifactStore(store.root/'gripper-camera'),max_captures=4,clock=clock,opener=camera)
        source.start(deadline=clock.now+20)
        source.capture(deadline=clock.now+6);after=source.capture(deadline=clock.now+6)
        class Native:
            frames=[]
            def check(owner):
                at=clock.now-.005
                owner.frames.append(dict(frameIndex=len(owner.frames),frameId='retention-callback:'+str(len(owner.frames)),
                    sourceEarliest=at,sourceLatest=at,stdoutReadAt=at,persistenceMs=0,qualityFlags=[]))
        native=Native();native.root=store.root/'native-camera'
        idle=[]
        manager=SimpleNamespace(store=store,camera=native,current={'phase':'recording'},
            tick=lambda:native.check(),port=SimpleNamespace(check_idle=lambda:idle.append(clock.now)))
        with patch.object(self.runtime,'_legacy_snapshot',return_value={'root':'fake-source-descriptor'}):
            result=self.runtime._after_capture(manager,dict(status='recorded',after=after),source)
        self.assertEqual(result['status'],'recorded',result)
        self.assertEqual(len(camera.calls),4)
        self.assertEqual(len(idle),4)
        a,b=result['entries']
        self.assertGreaterEqual(b['external']['sourceEarliest']-a['external']['sourceLatest'],1)
        self.assertGreaterEqual(b['gripper']['sourceEarliest']-a['gripper']['persistedAt'],1)
        self.assertEqual(len(json.loads((store.root/'batch-retention-captures.json').read_bytes())['entries']),2)
        source.close()

    def retention_fixture(self):
        from tests.test_m4_up_commissioning import Observer,scope
        from tests.test_supervised_transport import Clock
        from learning.gripper_camera_dataset import read_frame
        source=self.runtime.root/'sessions'/'one';source.mkdir(parents=True)
        planned=scope();planned['hostClockId']='one'
        clock=Clock();observer=Observer(source,clock,planned);observer.start()
        self.runtime.config['deviceId']=planned['context']['externalDeviceId']
        self.runtime.plan['viewId']=planned['context']['externalViewId']
        self.runtime.config['gripper']=dict(url='http://camera.invalid',bootId='b'*32,retentionCaptures=dict(count=2,intervalSeconds=1))
        spec=copy.deepcopy(self.spec);spec.update(cameraIdentity='camera-id',viewId='test-view')
        self.spec_path.write_bytes(encoded(spec));self.runtime.spec=spec
        self.runtime.config['observerSpecSha256']=hashlib.sha256(self.spec_path.read_bytes()).hexdigest()
        frames=[];entries=[]
        for i in range(3):
            if i:clock.sleep(1.3)
            record=observer.observe(after_off=6,deadline=clock.now+10,tick=lambda:None)
            frame=json.loads((source/'external'/record['external']['ingressFile']).read_bytes());frames.append(frame)
            # The batch collector waits for an external frame after the JPEG
            # request. Model that ordering in this synthetic legacy fixture.
            metadata_path=source/'gripper/esp32-after.json'
            metadata=json.loads(metadata_path.read_bytes());metadata['requestStartedAt']-=.05
            metadata_path.write_bytes(encoded(metadata))
            if i==0:
                pair=source/'gripper-pair';pair.mkdir()
                for suffix in ('jpg','json'):(pair/('esp32-after.'+suffix)).write_bytes((source/'gripper'/('esp32-after.'+suffix)).read_bytes())
            else:
                folder=source/'gripper-camera'/('capture-'+str(i+2).zfill(4));folder.mkdir(parents=True)
                for suffix in ('jpg','json'):(folder/('esp32-frame.'+suffix)).write_bytes((source/'gripper'/('esp32-after.'+suffix)).read_bytes())
                grip,metadata,_=read_frame(folder,'frame','one')
                decoder=lambda raw,length,digest:observer.payloads[raw]
                with patch('learning.batch_runtime.decode',side_effect=decoder):
                    snapshot=self.runtime._legacy_snapshot(source,source/'external',frame,folder,i)
                entries.append(dict(external=frame,gripper=dict(frame=grip),gripperFolder=str(folder.relative_to(source)),legacySnapshot=snapshot))
        (source/'session.json').write_bytes(encoded(dict(sessionId='one',frames=frames,endedAt=clock.now+.5)))
        manifest=dict(status='recorded',intervalSeconds=1,initialExternal=frames[0],entries=entries)
        (source/'batch-retention-captures.json').write_bytes(encoded(manifest))
        return source,dict(sessionId='one',evidence=dict(offReceiptAt=6)),manifest,observer

    def test_retention_actual_dual_camera_span_and_pure_source_reaudit(self):
        from learning import m4_up_commissioning as legacy
        from learning.batch_runtime import audit_retention_transition
        source,row,manifest,observer=self.retention_fixture()
        decoder=lambda raw,length,digest:observer.payloads[raw]
        with patch.object(legacy,'decode',side_effect=decoder):
            result=self.runtime._retention_transition(source,row)
            self.assertEqual(result['status'],'recorded')
            self.assertGreaterEqual(result['entries'][1]['external']['sourceEarliest']-result['entries'][0]['external']['sourceLatest'],1)
            self.assertGreaterEqual(result['entries'][1]['metadata']['requestStartedAt']-result['entries'][0]['metadata']['persistedAt'],1)
            transition=dict(supplementalObservations=result,gripperPair=dict(observerBinding=self.runtime.config['gripper'],
                externalObserverBinding=dict(path=self.runtime.config['observerSpecPath'],sha256=self.runtime.config['observerSpecSha256'])))
            ids=audit_retention_transition(source,row,transition,self.repository)
            self.assertEqual(len(ids),4);self.assertEqual(len(set(ids)),4)

    def test_retention_short_actual_external_span_is_rejected(self):
        source,row,manifest,observer=self.retention_fixture()
        previous=manifest['entries'][0]['external']
        changed=manifest['entries'][1]['external'];changed['sourceEarliest']=previous['sourceLatest']+.5
        session=json.loads((source/'session.json').read_bytes());session['frames'][changed['frameIndex']]=changed
        (source/'session.json').write_bytes(encoded(session));(source/'batch-retention-captures.json').write_bytes(encoded(manifest))
        from learning import m4_up_commissioning as legacy
        with patch.object(legacy,'decode',side_effect=lambda raw,length,digest:observer.payloads[raw]):
            with self.assertRaisesRegex(ValueError,'retention_external_source_changed'):
                self.runtime._retention_transition(source,row)

    def after_provider_fixture(self, *, change=None, respond=True):
        from tests.test_batch_interaction import BatchInteractionTests
        from learning.gripper_camera_dataset import read_frame
        fixture=BatchInteractionTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        _,packet,_=fixture.after_packet()
        runtime=object.__new__(BatchRuntime)
        runtime.repository=fixture.repository;runtime.root=fixture.root
        runtime.spec=fixture.spec;runtime.clock=fixture.clock;runtime.clock.time=lambda:fixture.clock.now
        runtime.deadline_wall=runtime.deadline_mono=None
        runtime.config=copy.deepcopy(fixture.plan['runtime'])
        runtime.config['afterReviewer']=dict(kind='file',protocolVersion=1,maxWaitSeconds=0)
        runtime.config['conditionBindings']={fixture.condition['id']:dict(route='generic',interaction='contact_probe')}
        runtime.plan=copy.deepcopy(fixture.plan);runtime.plan['runtime']=runtime.config
        frames=[json.loads(path.read_bytes()) for path in sorted((fixture.root/'external/host-ingress').glob('[0-9][0-9][0-9][0-9].json'))]
        (fixture.root/'session.json').write_bytes(encoded(dict(sessionId=fixture.scope['hostClockId'],frames=frames)))
        (fixture.root/'batch-execution-proof.json').write_bytes(encoded(dict(outputSequence=100)))
        pair={side:read_frame(fixture.root/'gripper',side,fixture.scope['hostClockId'])[0] for side in ('before','after')}
        original=runtime._save
        def save(path,value):
            original(path,value)
            if respond and path.name=='batch-after-review-request.json':
                response=dict(requestSha256=hashlib.sha256(encoded(value)).hexdigest(),review=copy.deepcopy(packet['review']))
                if change:change(response)
                (fixture.root/'batch-after-review-response.json').write_bytes(encoded(response))
        runtime._save=save
        row=dict(evidence=dict(offReceiptAt=packet['review']['afterOffAt']))
        with patch.object(runtime,'_pair_transition',return_value=pair):
            result=runtime._interaction_after(fixture.root,fixture.condition,row)
        return runtime,fixture,result

    def test_live_after_provider_binds_actual_request_and_keeps_unknown_separate(self):
        runtime,fixture,result=self.after_provider_fixture()
        self.assertEqual(result['status'],'reviewed',result)
        self.assertEqual(result['contactState'],'unknown')
        self.assertFalse(result['graspVerified'])
        self.assertFalse(result['liftVerified'])
        request=json.loads((fixture.root/'batch-after-review-request.json').read_bytes())
        self.assertEqual(request['source']['sha256'],hashlib.sha256((fixture.root/'session.json').read_bytes()).hexdigest())
        self.assertEqual(request['offAt'],fixture.envelope['afterOffAt'])

    def test_after_provider_wrong_request_or_source_is_rejected_without_success(self):
        changes=[lambda packet:packet.update(requestSha256='0'*64),
                 lambda packet:packet['review']['scope'].update(hostClockId='another-session')]
        for change in changes:
            _,_,result=self.after_provider_fixture(change=change)
            self.assertEqual(result['status'],'rejected')
            self.assertFalse(result['graspVerified'])

    def test_missing_after_provider_does_not_block_or_claim_success(self):
        _,_,result=self.after_provider_fixture(respond=False)
        self.assertEqual(result['status'],'not_reviewed')
        self.assertEqual(result['reason'],'after_review_not_supplied')
        self.assertFalse(result['retainedLift'])

    def test_cli_reconcile_only_never_looks_up_credentials_or_observes(self):
        from learning.batch_campaign import Campaign
        module=cli_module();module.ROOT=self.repository
        path=self.private/'plan.json';path.write_bytes(encoded(self.plan))
        output=self.private/'existing'
        campaign=Campaign(self.plan,output,self.runtime,lock_path=self.private/'lock')
        campaign.close()
        original=module.inspect_plan
        with patch.object(module,'inspect_plan',side_effect=lambda p,**kw:original(p,self.repository,**kw)),patch.object(module,'load_keychain_secret',side_effect=AssertionError('keychain')),patch.object(BatchRuntime,'observe',side_effect=AssertionError('capture')),patch('sys.stdout',new=io.StringIO()):
            result=module.main(['--plan',str(path),'--plan-sha',digest(self.plan),'--output',str(output),'--reconcile-only'])
        self.assertEqual(result['status'],'reconciled')

    def test_m4_after_audit_uses_recorded_scope_bytes_not_configuration_format(self):
        folder=self.runtime.root/'m4-scopes'/'format-test';folder.mkdir(parents=True)
        (folder/'result.json').write_text('{}')
        scope_raw=b'{"test": true}\n'
        (folder/'scope.json').write_bytes(scope_raw)
        self.runtime.config['afterReviewer']=dict(kind='file',protocolVersion=1,maxWaitSeconds=0)
        self.runtime.config['conditionBindings']={self.condition['id']:dict(route='m4_up',scopeSha256='a'*64)}
        with patch('learning.batch_m4.audit_source',side_effect=ValueError('audit-stop')) as audit:
            self.runtime._m4_after_result(self.condition,'format-test',folder,
                dict(execution='completed',offVerified=True))
        self.assertEqual(audit.call_args.args[2],hashlib.sha256(scope_raw).hexdigest())
        self.assertNotEqual(audit.call_args.args[2],'a'*64)

    def test_changed_shared_lock_or_journal_is_rejected(self):
        for key in ('journalPath','dispatchLockPath'):
            plan=copy.deepcopy(self.plan);plan['runtime'][key]='.local/other'
            with self.assertRaisesRegex(ValueError,'runtime_shared_'):
                validate_runtime(plan,self.repository)

    def test_joint_node_and_missing_frozen_evaluation_are_rejected(self):
        condition=copy.deepcopy(self.condition);condition['fromNodes']=['elbow-node']
        self.assertEqual(self.runtime.condition_support(condition),'runtime_condition_nodes_not_bound_to_joint')
        condition=copy.deepcopy(self.condition);condition['phase']='evaluation'
        self.assertEqual(self.runtime.condition_support(condition),'runtime_evaluation_frozen_model_required')

    def test_reconcile_missing_reservation_does_not_open_devices_or_keychain(self):
        with patch('learning.batch_runtime.SequencePort',side_effect=AssertionError('device')):
            self.assertIsNone(self.runtime.reconcile(self.condition,'request'))
        self.assertEqual(self.credentials,[])

    def test_deadline_checks_both_wall_and_monotonic(self):
        clock=manual_fixture.Clock();clock.time=lambda:clock.now
        self.runtime.clock=clock
        self.runtime.deadline_wall=101
        self.runtime.deadline_mono=101
        clock.now=102
        with self.assertRaisesRegex(ValueError,'runtime_deadline'):
            self.runtime._deadline()

    def test_journal_supervised_pending_blocks_even_without_legacy_command(self):
        from learning.action_journal import BoundedActionJournal
        path=self.private/'journal.sqlite'
        journal=BoundedActionJournal(str(path),100)
        journal.connection.execute('CREATE TABLE supervised_blocks (phase TEXT)')
        journal.connection.execute("INSERT INTO supervised_blocks VALUES ('pending')")
        journal.connection.commit();journal.close()
        self.assertFalse(journal_clear(path))

    def test_observer_processing_failure_preserves_completed_output(self):
        source=self.runtime.root/'sessions'/'one';source.mkdir(parents=True)
        audit=dict(execution=dict(status='verified',offVerified=True,episode=['boot','cmd'], command=self.condition['command']),
                   recording=dict(status='verified'),visual=dict(status='verified',row={'sessionId':'one'}))
        (source/'session.json').write_text(json.dumps(dict(joint='wrist')))
        report=dict(sessionPath=str(source),sourceSha256='a'*64)
        with patch('learning.batch_runtime.ingest_session',return_value=audit),patch('learning.batch_runtime.journal_clear',return_value=True):
            result=self.runtime._classify(self.condition,'request',report,process=True)
        self.assertEqual(result['execution'],'completed')
        self.assertTrue(result['offVerified'])
        self.assertEqual(result['observation'],'rejected')

    def test_runtime_one_second_deadline_prevents_real_sequence_run_after_pre_window(self):
        fixture=manual_fixture.ManualCollectionTests();fixture.setUp()
        self.addCleanup(fixture.tearDown)
        device,requests,_=fixture.transport()
        self.runtime.clock=fixture.clock
        self.runtime.clock.time=lambda:self.runtime.clock.monotonic()
        from learning.manual_collection import ManualCollection,RecordingPort
        opener=fixture.manager.port_factory('http://127.0.0.1','private-token','discovering',None,
            observer=lambda _:None,clock=fixture.clock).opener
        class TestPort(RecordingPort):
            def __init__(port,*args,**kwargs):
                super().__init__(*args,**kwargs);port.opener=opener
        class TestManager(ManualCollection):
            def __init__(manager,*args,**kwargs):
                kwargs['camera_factory']=manual_fixture.Camera
                super().__init__(*args,**kwargs)
        with patch('learning.batch_runtime.ManualCollection',TestManager),patch('learning.manual_collection.RecordingPort',TestPort):
            result=self.runtime.execute(self.condition,'deadline-case',deadline_wall=fixture.clock.now+1)
        self.assertFalse(any(payload=={'action':'run'} for path,payload in device.requests))
        self.assertEqual(result['execution'],'not_dispatched')
        self.assertTrue(result['offVerified'])

    def test_native_pixels_pinned_transition_and_frozen_evaluation_roundtrip(self):
        import cv2
        from learning.batch_learning import fit
        from learning.batch_observation import digest as observer_digest
        source=self.runtime.root/'sessions'/'one';folder=source/'native-camera'
        folder.mkdir(parents=True)
        bgra=cv2.cvtColor(self.image,cv2.COLOR_BGR2BGRA).tobytes()
        stored=b'compressed-native-frame'
        def frame(i,t):
            name=f'frame-{i:04d}.bgra.lzbitmap';(folder/name).write_bytes(stored)
            return dict(frameIndex=i,frameId='one:'+str(i),imageFile=name,
                        imageSha256=hashlib.sha256(stored).hexdigest(),pixelSha256=hashlib.sha256(bgra).hexdigest(),
                        rawByteLength=len(bgra),height=100,width=120,sourceEarliest=t,sourceLatest=t+.001,
                        stdoutReadAt=t+.02,persistenceMs=5)
        before,default,after=frame(0,95),frame(1,95.5),frame(2,100)
        session=dict(joint='wrist',frames=[before,default,after]);raw=encoded(session)
        (source/'session.json').write_bytes(raw)
        with patch('learning.batch_runtime.decode',return_value=bgra):
            pinned=self.runtime._observe_native(folder,before,historical=True)
            self.assertEqual(pinned['observation']['status'],'accepted')
            self.assertAlmostEqual(pinned['derivedMetadata']['persistedAt'],95.025)
            (source/'batch-before-observation.json').write_bytes(encoded(pinned))
            audit=dict(execution=dict(status='verified',offVerified=True,episode=['boot','cmd'],command=self.condition['command']),
                recording=dict(status='verified'),visual=dict(status='verified',row=dict(sessionId='one',
                inputs=dict(image=default),targets=dict(image=after),
                evidence=dict(preFrameIds=[0,1],commandAt=96))))
            report=dict(sessionPath=str(source),sourceSha256=hashlib.sha256(raw).hexdigest())
            with patch('learning.batch_runtime.ingest_session',return_value=audit):
                result=self.runtime._classify(self.condition,'request',report,process=True)
            self.assertEqual(result['observation'],'eligible',result)
            transition=json.loads((self.runtime.root/'transitions'/(result['transitionId']+'.json')).read_bytes())
            self.assertEqual(transition['learningRow']['frameIds'],['one:0','one:2'])
            self.assertEqual(transition['selectionSemantics'],'pinned_pre_window_frame_not_default_audit_input')
            row=transition['learningRow'];training=[]
            for i,delta in enumerate((-1,1)):
                r=copy.deepcopy(row)
                r.update(sessionId='train'+str(i),episodeId='train-episode'+str(i),sourceHash=str(i+1)*64,
                         frameIds=['train-before'+str(i),'train-after'+str(i)],captureGroup='training')
                r['before']=[v+delta for v in r['before']]
                training.append(r)
            model=fit(training);model_path=self.private/'model.json';model_path.write_bytes(encoded(model))
            plan=copy.deepcopy(self.plan);plan['conditions'][0]['phase']='evaluation'
            plan['conditions'][0]['captureGroup']='holdout'
            plan['runtime']['frozenModels']={'wrist':dict(path='.local/model.json',sha256=hashlib.sha256(model_path.read_bytes()).hexdigest())}
            runtime=BatchRuntime(plan,self.repository,self.runtime.root,self.credential)
            self.addCleanup(runtime.close)
            self.assertIsNone(runtime.condition_support(plan['conditions'][0]))
            # Re-bind saved before evidence to this process clock identity.
            pinned=runtime._observe_native(folder,before,historical=True)
            (source/'batch-before-observation.json').write_bytes(encoded(pinned))
            from learning.batch_learning import prospective_evidence
            proof=prospective_evidence(model,dict(sessionId='one',before=row['before'],command=row['command'],
                context=row['context'],beforePersistedAt=pinned['derivedMetadata']['persistedAt']),95.1)
            (source/'batch-prospective.json').write_bytes(encoded(dict(proof=proof,beforeFrameId='one:0',beforePixelSha256=before['pixelSha256'])))
            with patch('learning.batch_runtime.ingest_session',return_value=audit):
                result=runtime._classify(plan['conditions'][0],'eval-request',report,process=True)
            self.assertEqual(result['observation'],'eligible',result)
            self.assertTrue(result['prospectiveEvidenceVerified'])
            self.assertEqual(result['dataIdentity']['frameIds'],['one:0','one:2'])

    def test_all_five_joints_use_real_bounded_sequence_transport(self):
        from learning.manual_collection import ManualCollection,RecordingPort
        fixture=manual_fixture.ManualCollectionTests();fixture.setUp()
        self.addCleanup(fixture.tearDown)
        device,requests,_=fixture.transport()
        opener=fixture.manager.port_factory('http://127.0.0.1','private-token','discovering',None,
            observer=lambda _:None,clock=fixture.clock).opener
        fixture.clock.time=lambda:fixture.clock.monotonic()
        class TestPort(RecordingPort):
            def __init__(port,*args,**kwargs):super().__init__(*args,**kwargs);port.opener=opener
        class TestManager(ManualCollection):
            def __init__(manager,*args,**kwargs):
                kwargs['camera_factory']=manual_fixture.Camera;super().__init__(*args,**kwargs)
        spec=copy.deepcopy(self.spec);template=copy.deepcopy(spec['joints']['wrist'])
        from learning.supervised_collection import PROFILES
        for joint in PROFILES:
            spec['joints'][joint]=copy.deepcopy(template)
            spec['joints'][joint]['nodes'][0]['id']=joint+'-node'
        self.spec_path.write_bytes(encoded(spec))
        for joint,profile in PROFILES.items():
            with self.subTest(joint=joint):
                plan=copy.deepcopy(self.plan)
                plan['runtime']['observerSpecSha256']=hashlib.sha256(self.spec_path.read_bytes()).hexdigest()
                condition=copy.deepcopy(self.condition)
                condition.update(id=joint+'-test',fromNodes=[joint+'-node'])
                condition['command']=dict(joint=joint,direction=next(iter(profile['directions'])),percent=20,durationMs=200)
                plan['conditions']=[condition]
                runtime=BatchRuntime(plan,self.repository,self.private/('runtime-'+joint),self.credential,clock=fixture.clock)
                self.addCleanup(runtime.close)
                def before(manager,pair):
                    runtime.latest_before=dict(source=dict(frame=dict(sourceEarliest=fixture.clock.monotonic())),
                        observation=dict(status='accepted',nodeMatches=[joint+'-node']))
                with patch('learning.batch_runtime.ManualCollection',TestManager),patch('learning.manual_collection.RecordingPort',TestPort),patch.object(runtime,'_before',return_value=before):
                    result=runtime.execute(condition,joint+'-request',deadline_wall=fixture.clock.monotonic()+60)
                self.assertEqual(result['execution'],'completed')
                self.assertTrue(result['offVerified'])
                self.assertEqual(result['observation'],'rejected') # intentionally incomplete camera fixture
        self.assertEqual(sum(payload=={'action':'run'} for path,payload in device.requests),5)

    def test_cli_default_does_not_load_keychain_or_create_output(self):
        module=cli_module();module.ROOT=self.repository
        path=self.private/'plan.json';path.write_bytes(encoded(self.plan))
        with patch.object(module,'load_keychain_secret',side_effect=AssertionError('keychain')),patch('sys.stdout',new=io.StringIO()):
            # inspect_plan's default is module bound; pass patched wrapper explicitly.
            original=module.inspect_plan
            with patch.object(module,'inspect_plan',side_effect=lambda p,**kw:original(p,self.repository,**kw)):
                result=module.main(['--plan',str(path)])
        self.assertEqual(result['status'],'plan_checked_no_device_access')
        self.assertFalse(self.runtime.root.exists())

    def test_cli_execute_without_scope_or_matching_hash_never_loads_keychain(self):
        module=cli_module();module.ROOT=self.repository
        path=self.private/'plan.json';path.write_bytes(encoded(self.plan))
        original=module.inspect_plan
        with patch.object(module,'load_keychain_secret',side_effect=AssertionError('keychain')),patch.object(module,'inspect_plan',side_effect=lambda p,**kw:original(p,self.repository,**kw)),patch('sys.stderr',new=io.StringIO()):
            for args in [['--execute'],['--execute','--onsite-confirmed','--plan-sha','wrong']]:
                with self.assertRaises(SystemExit):module.main(['--plan',str(path),*args])


if __name__=='__main__':unittest.main()
