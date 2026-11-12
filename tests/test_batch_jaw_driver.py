import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from learning import batch_jaw_driver as jaw


def observed(gap=72):
    left=280-(gap-76)/2;right=356+(gap-76)/2
    return dict(pixelSha256='e'*64,state=[left,289,right,288,gap],stateFields=['jawLeftXpx','jawLeftYpx','jawRightXpx','jawRightYpx','projectedJawSeparationPx'],
        quality=dict(jawLeft=dict(shiftPx=[left-280,0]),jawRight=dict(shiftPx=[right-356,0])))


def plan(phase='training'):
    return dict(schemaVersion='motionbrain.batch_campaign.v1',campaignId=phase,viewId='view',controllerBootId='b'*32,
        maxObservationAgeSeconds=3,nodes=['open','close'],budgets=dict(maxPhysicalCommands=4,maxPreparationCommands=1,maxAttempts=4,maxSeconds=600),
        conditions=[dict(id=d,command=dict(joint='gripper',direction=d,percent=40,durationMs=350),fromNodes=[d],phase=phase,captureGroup=phase,target=2,maxAttempts=2) for d in ('open','close')])


class JawDriverTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.repo=Path(self.temp.name);self.local=self.repo/'.local';self.local.mkdir()
        self.clock=SimpleNamespace(time=lambda:100.,monotonic=lambda:12.)
        def binding(name,value):
            p=self.local/name;p.write_bytes(value if isinstance(value,bytes) else jaw.encoded(value))
            return dict(path=str(p),sha256=jaw.sha(p.read_bytes()))
        self.binding=binding
        reg=dict(imageShape=[480,640],parameters=dict(searchRadii=dict(jawLeft=[24,24],jawRight=[24,24]),
            minimumNcc=dict(jawLeft=.94,jawRight=.94),minimumPeakGap=.04,minimumOpeningPx=60,maximumOpeningPx=120))
        self.config=dict(plan=plan(),root=str(self.local/'job/training'),controllerUrl='http://controller',deviceId='device',
            helper=binding('helper',b'helper'),gripper=dict(url='http://camera',bootId='a'*32,registration=binding('reg.json',reg),reference=binding('reference',b'pixels')),
            context=dict(viewId='view',poseRegime='fixed',object='none',load='empty'),
            external=dict(configurationSha256='c'*64,width=1920,height=1080,videoRotationAngle=0,videoMirrored=False),
            qualification=binding('qualification.json',dict(sources=[])),credential=dict(service='service',account='account'))
        self.auth=dict(schemaVersion=jaw.AUTH_SCHEMA,driverConfigSha256=jaw.driver_config_digest(self.config),planHashes=[jaw.digest(plan()),jaw.digest(plan('evaluation'))],
            outputRoot=str(self.local/'job'),controllerBootId='b'*32,maxPhysicalCommands=2,expiresAt=200,onsite={k:True for k in jaw.ONSITE})
        self.config['authorization']=binding('auth.json',self.auth)
        self.tracker=Mock();self.tracker.landmark_contract={};self.tracker.observe_jaws.return_value=observed()

    def driver(self,phase='training',config=None):
        c=copy.deepcopy(config or self.config);c['plan']=plan(phase)
        with patch.object(jaw,'RegisteredGripperLandmarks',return_value=self.tracker),patch.object(jaw,'verify_qualification',return_value={}):
            return jaw.JawDriver(self.repo,c,phase,clock=self.clock)

    def test_constructor_and_unused_close_no_device_or_credential_access(self):
        with patch.object(jaw.urllib.request,'build_opener',side_effect=AssertionError('network')),patch('learning.macos_credentials.load_keychain_secret',side_effect=AssertionError('credential')):
            d=self.driver();d.close()
        self.assertIsNone(d.condition_support(plan()['conditions'][0]))
        changed=copy.deepcopy(plan()['conditions'][0]);changed['command']['percent']=41
        self.assertIsNotNone(d.condition_support(changed))

    def test_shared_authorization_budget_across_output_directories_and_phase(self):
        d=self.driver();d._reserve('one')
        c=copy.deepcopy(self.config);c['root']=str(self.local/'job/other')
        other=self.driver(config=c)
        with self.assertRaisesRegex(ValueError,'consumed'):other._reserve('one')
        other._reserve('two')
        with self.assertRaisesRegex(ValueError,'exhausted'):d._reserve('three')

    def test_initial_confirmation_binding_and_expiry_fail_closed(self):
        d=self.driver();d._verify()
        self.clock.time=lambda:201
        with self.assertRaisesRegex(ValueError,'expired'):d._verify()
        self.clock.time=lambda:100
        p=Path(self.config['helper']['path']);p.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'binding_changed'):d._verify()
        p.write_bytes(b'helper')
        c=copy.deepcopy(self.config);c['root']=str(self.local/'different')
        with self.assertRaisesRegex(ValueError,'output_root'):self.driver(config=c)

    def test_direction_and_operating_window_are_not_response_target(self):
        self.assertEqual(jaw.check_state(observed(72),plan()['conditions'][0]['command']),'open')
        self.assertEqual(jaw.check_state(observed(106),plan()['conditions'][1]['command']),'close')
        with self.assertRaisesRegex(ValueError,'direction_rule'):jaw.check_state(observed(106),plan()['conditions'][0]['command'])
        transition=dict(learningRow=dict(before=observed(72)['state'],after=observed(112)['state'],command=plan()['conditions'][0]['command']),observations=dict(after=observed(112)))
        self.assertIsNone(jaw.check_response(transition)) # large40px is valid
        for gap,reason in [(72,'unresolved'),(64,'opposite'),(118,'operating_gap')]:
            transition['learningRow']['after']=observed(gap)['state'];transition['observations']['after']=observed(gap)
            self.assertIn(reason,jaw.check_response(transition))

    def test_directional_travel_reserves_two_pixels(self):
        o=observed(72);o['quality']['jawLeft']['shiftPx'][0]=-9
        with self.assertRaisesRegex(ValueError,'directional'):jaw.check_state(o,plan()['conditions'][0]['command'])

    def test_cleanup_clears_tokens_even_when_close_raises(self):
        d=self.driver();port=SimpleNamespace(token='hidden')
        manager=SimpleNamespace(token='hidden',port=port,close=Mock(side_effect=ValueError('off unknown')))
        d.manager=manager;d.profile_owned=True;d._profile=Mock()
        with self.assertRaisesRegex(ValueError,'off unknown'):d.close()
        self.assertEqual(manager.token,'');self.assertEqual(port.token,'');self.assertIsNone(d.manager)
        d._profile.assert_called_once_with(jaw.RESTORE);manager.close.assert_called_once()
        d.close();manager.close.assert_called_once()

    def raw_manager(self,d):
        folder=self.local/'raw';folder.mkdir();image=b'raw';(folder/'frame.lzfse').write_bytes(image)
        metadata=dict(d.config['external'],deviceUniqueId='device',viewConfigurationId='view')
        (folder/'frame.json').write_bytes(jaw.encoded(metadata))
        frame=dict(imageFile='frame.lzfse',imageSha256=jaw.sha(image),metadataFile='frame.json',metadataSha256=jaw.sha(jaw.encoded(metadata)),
            sourceEarliest=10.,frameId='external',pixelSha256='d'*64,stdoutReadAt=10.1,persistenceMs=10)
        root=self.local/'session';store=jaw.ArtifactStore(root);(root/'gripper-pair').mkdir(parents=True);(root/'gripper-pair/esp32-before.jpg').write_bytes(b'jpeg')
        manager=SimpleNamespace(port=SimpleNamespace(check_idle=Mock(),boot_id='b'*32),camera=SimpleNamespace(root=folder,frames=[frame],check=Mock()),
            store=store,current=dict(sessionId='session',phase='recording',events=[]),command_count=0)
        manager._event=lambda e:manager.current['events'].append(e)
        grip=dict(frameId='a'*32+':10',pixelSha256='e'*64,width=640,height=480)
        meta=dict(requestStartedAt=9.,persistedAt=9.2)
        d.latest=observed();d._ready_profile=Mock()
        return manager,grip,meta,frame

    def test_real_before_callback_and_final_guard_use_source_times_not_processing_time(self):
        d=self.driver();manager,grip,meta,frame=self.raw_manager(d)
        with patch.object(jaw,'read_frame',return_value=(grip,meta,'a'*32)):
            d._before(manager,{},plan()['conditions'][0]);d._guard(manager)
            self.clock.monotonic=lambda:13.001
            with self.assertRaisesRegex(ValueError,'dispatch_stale'):d._guard(manager)
            self.clock.monotonic=lambda:12
            (manager.camera.root/'frame.lzfse').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'raw_hash'):d._guard(manager)

    def test_final_guard_enforces_stricter_plan_freshness_after_before(self):
        d=self.driver();manager,grip,meta,frame=self.raw_manager(d)
        d.plan['maxObservationAgeSeconds']=.5
        # The signed plan has already been verified; isolate the dispatch age gate.
        with patch.object(d,'_verify'),patch.object(jaw,'read_frame',return_value=(grip,meta,'a'*32)):
            self.clock.monotonic=lambda:10.4
            d._before(manager,{},plan()['conditions'][0])
            self.clock.monotonic=lambda:10.6
            with self.assertRaisesRegex(ValueError,'dispatch_stale'):d._guard(manager)

    def test_reconcile_never_calls_execute_or_network(self):
        d=self.driver();d._classify=Mock(return_value=dict(execution='unknown'))
        path=d.root/'requests/one.json';path.parent.mkdir(parents=True)
        c=plan()['conditions'][0];path.write_bytes(jaw.encoded(dict(request=dict(requestId='one',command=c['command'],approvalPlanSha256=d.plan_sha))))
        with patch.object(d,'execute',side_effect=AssertionError('retry')),patch.object(jaw.urllib.request,'build_opener',side_effect=AssertionError('network')):
            self.assertEqual(d.reconcile(c,'one'),dict(execution='unknown'))
        self.assertIsNone(d.reconcile(c,'missing'))

    def test_before_scope_to_actual_jpeg_transition_manifest_loader_and_scope_tamper(self):
        import cv2
        import numpy as np
        from tests.test_registered_jaw_transition import JawTransitionTests
        from learning.registered_gripper_landmarks import register_jaws, default_parameters
        from learning.registered_jaw_transition import load_learning_transition
        fixture=JawTransitionTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        f=fixture.f;source=f.source;repository=f.f.repository
        image=np.random.default_rng(87).integers(10,245,(480,640,3),dtype=np.uint8)
        raw=cv2.imencode('.jpg',image)[1].tobytes()
        params=default_parameters();params.update(imageShape=[480,640],minimumOpeningPx=60.,maximumOpeningPx=120.)
        params['rois'].update(jawLeft=[258,268,290,300],jawRight=[345,268,377,300])
        params['searchRadii'].update(jawLeft=[24,24],jawRight=[24,24])
        registration=register_jaws(raw,dict(jawLeft=[280,289],jawRight=[356,288]),params)
        for side in ('before','after'):f.image(side,raw)
        d=self.driver();d.repository=repository;d.root=f.f.private/'public-driver'
        d.registration=registration;d.reference=raw;d.context=fixture.context
        d.tracker=jaw.RegisteredGripperLandmarks(registration,raw)
        d.config['gripper']['bootId']='a'*32
        for key,name,value in [('registration','qualified-reg.json',jaw.encoded(registration)),('reference','qualified-ref.jpg',raw)]:
            path=f.f.private/name;path.write_bytes(value);d.config['gripper'][key]=dict(path=str(path),sha256=jaw.sha(value))
        grip,meta,_=jaw.read_frame(source/'gripper-pair','before','session')
        d.latest=d.tracker.observe_jaws(raw,grip['frameId'])
        current=dict(sessionId='session',phase='recording',events=[])
        store=SimpleNamespace(root=source,write_json=lambda name,value:(source/name).write_bytes(jaw.encoded(value)))
        manager=SimpleNamespace(port=SimpleNamespace(check_idle=Mock()),camera=SimpleNamespace(frames=[{}],root=source),
            current=current,command_count=0,store=store)
        manager._event=lambda e:current['events'].append(e)
        condition=plan()['conditions'][0]
        fixture.row['inputs']['command']=condition['command']
        with patch.object(d,'_verify'),patch.object(d,'_external'),patch.object(d,'_guard'):
            d._before(manager,{},condition)
        scope_sha=current['validationScopeSha256']
        intent=dict(at=20.,validationScopeSha256=scope_sha)
        (source/'command-001-intent.json').write_bytes(jaw.encoded(intent))
        session=dict(poseContext='fixed',sessionId='session',events=current['events'],validationScopeSha256=scope_sha)
        (source/'session.json').write_bytes(jaw.encoded(session))
        report=dict(status='recorded',offVerified=True,sessionPath=str(source),sourceSha256=jaw.sha(jaw.encoded(session)))
        with patch('learning.pipeline_evidence.ingest_session',return_value=fixture.audit),patch('learning.registered_jaw_transition.ingest_session',return_value=fixture.audit),patch.object(jaw,'journal_clear',return_value=True):
            result=d._classify(condition,report)
            self.assertEqual(result['observation'],'eligible')
            self.assertEqual(result['followupStopReason'],'jaw_unresolved_response')
            row,provenance=load_learning_transition(repository,result['manifestEntry'],'training')
            self.assertEqual(row['command'],condition['command']);self.assertTrue(provenance['observationRecomputed'])
            scope=json.loads((source/'commissioning-validation-scope.json').read_bytes());scope['scope']['phase']='evaluation'
            (source/'commissioning-validation-scope.json').write_bytes(jaw.encoded(scope))
            denied=d._classify(condition,report)
            self.assertEqual(denied['failureScope'],'global')
            self.assertEqual(denied['observation'],'rejected')
            self.assertIn('authorization_scope_changed',denied['reason'])

    def test_readonly_probe_cannot_issue_controller_post(self):
        port=object.__new__(jaw.ReadOnlyPort)
        with self.assertRaisesRegex(ValueError,'write_forbidden'):port._call('/command',{'cmd':'arm'})


if __name__=='__main__':unittest.main()
