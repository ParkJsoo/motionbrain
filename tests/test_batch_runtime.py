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

    def test_contact_load_and_pair_capture_are_explicitly_unsupported(self):
        for field,value in [('load','held'),('object','cylinder')]:
            spec=copy.deepcopy(self.spec);spec[field]=value
            self.spec_path.write_bytes(encoded(spec))
            plan=copy.deepcopy(self.plan)
            plan['runtime']['observerSpecSha256']=hashlib.sha256(self.spec_path.read_bytes()).hexdigest()
            with self.assertRaisesRegex(ValueError,'noncontact_unloaded_only'):
                validate_runtime(plan,self.repository)

    def test_optional_broken_frozen_model_only_rejects_evaluation(self):
        plan=copy.deepcopy(self.plan)
        plan['runtime']['frozenModels']={'wrist':dict(path='.local/missing-model.json',sha256='a'*64)}
        runtime=BatchRuntime(plan,self.repository,self.private/'other-runtime',self.credential)
        self.assertIsNone(runtime.condition_support(self.condition))
        condition=copy.deepcopy(self.condition);condition['phase']='evaluation'
        self.assertIn('model_rejected',runtime.condition_support(condition))
        self.assertEqual(self.credentials,[])

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
            with patch.object(module,'inspect_plan',side_effect=lambda p:original(p,self.repository)):
                result=module.main(['--plan',str(path)])
        self.assertEqual(result['status'],'plan_checked_no_device_access')
        self.assertFalse(self.runtime.root.exists())

    def test_cli_execute_without_scope_or_matching_hash_never_loads_keychain(self):
        module=cli_module();module.ROOT=self.repository
        path=self.private/'plan.json';path.write_bytes(encoded(self.plan))
        original=module.inspect_plan
        with patch.object(module,'load_keychain_secret',side_effect=AssertionError('keychain')),patch.object(module,'inspect_plan',side_effect=lambda p:original(p,self.repository)),patch('sys.stderr',new=io.StringIO()):
            for args in [['--execute'],['--execute','--onsite-confirmed','--plan-sha','wrong']]:
                with self.assertRaises(SystemExit):module.main(['--plan',str(path),*args])


if __name__=='__main__':unittest.main()
