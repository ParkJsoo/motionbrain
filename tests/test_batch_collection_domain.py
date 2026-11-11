import copy
import unittest
from learning.batch_collection_domain import validate_domain, require_scope, require_transition, DOMAIN_SCHEMA, JAW_FIELDS
from learning.batch_model_runtime import ModelDecisionRuntime


class CollectionDomainTests(unittest.TestCase):
    def setUp(self):
        self.command = dict(joint='gripper',direction='open',percent=40,durationMs=350)
        self.domain = dict(schemaVersion=DOMAIN_SCHEMA,semantics='manual_noncontact_collection_not_model_control',
            jawMinimum=[266.,200.,342.,200.],jawMaximum=[294.,228.,370.,228.],
            minimumSearchMarginPx=2.,maximumGapChangePx=26.,
            commands=[dict(command=self.command,beforeGap=[62.,78.],afterGap=[84.,104.])])
        self.context = dict(viewId='fixed',poseRegime='unchanged',object='none',load='empty')
        self.fields = ['external:nuisance']+JAW_FIELDS
        self.artifact = dict(domain=self.domain,contract=dict(joint='gripper',context=self.context,stateFields=self.fields))
        self.before = [999.,286.,214.,356.,214.,70.]

    def test_declared_collection_domain_never_clamps_raw_features(self):
        saved=copy.deepcopy(self.before)
        require_scope(self.artifact,self.command,self.context,self.fields,self.before)
        self.assertEqual(self.before,saved)
        changed=self.before.copy();changed[-1]=79.
        with self.assertRaisesRegex(ValueError,'outside_declared'):
            require_scope(self.artifact,self.command,self.context,self.fields,changed)

    def test_other_pose_command_or_unbounded_profile_rejected(self):
        changed=copy.deepcopy(self.context);changed['poseRegime']='different'
        with self.assertRaisesRegex(ValueError,'contract_changed'):
            require_scope(self.artifact,self.command,changed,self.fields,self.before)
        for key,value in [('percent',41),('durationMs',700),('joint','wrist')]:
            domain=copy.deepcopy(self.domain);domain['commands'][0]['command'][key]=value
            with self.assertRaises(ValueError): validate_domain(domain)

    def test_zero_reverse_overshoot_stop_predicate(self):
        for gap in (70.,65.,100.):
            after=self.before.copy();after[-1]=gap
            with self.assertRaisesRegex(ValueError,'delta_rejected'):
                require_transition(self.artifact,self.command,self.fields,self.before,after)

    def test_model_runtime_rejects_collection_mode_before_file_or_device_access(self):
        for key in ('collectionAdmission','fixedCommandEvaluation'):
            with self.assertRaisesRegex(ValueError,'manual_collection_mode_forbidden'):
                ModelDecisionRuntime(dict(runtime={key:{}}))

    def test_original_registration_search_margin_is_not_expanded(self):
        from unittest.mock import patch
        from learning.batch_collection_domain import _registered_limits
        binding = dict(gripper=dict(observerSpecPath='spec',observerSpecSha256='a'*64))
        spec = dict(kind='registered_gripper_jaws_stationary',context=dict(load='empty'),
                    registrationPath='registration',registrationSha256='b'*64)
        registration = dict(points=dict(jawLeft=[280,214],jawRight=[356,214]),parameters=dict(
            searchRadii=dict(jawLeft=[16,16],jawRight=[16,16]),minimumOpeningPx=60,maximumOpeningPx=120))
        with patch('learning.batch_collection_domain._read',side_effect=[spec,registration]):
            _registered_limits(None,binding,self.domain)
        changed=copy.deepcopy(self.domain);changed['jawMinimum'][0]=265
        with patch('learning.batch_collection_domain._read',side_effect=[spec,registration]):
            with self.assertRaisesRegex(ValueError,'search_margin'):
                _registered_limits(None,binding,changed)

    def test_missing_independent_sources_cannot_qualify_domain(self):
        from learning.batch_collection_domain import build_collection_admission
        with self.assertRaisesRegex(ValueError,'independent_evidence'):
            build_collection_admission('.',dict(development=[],holdout=[],validationScope={}))

    def test_collection_admission_cannot_authorize_contact_or_m4_route(self):
        from unittest.mock import patch
        from tests.test_batch_runtime import BatchRuntimeTests
        from learning.batch_runtime import validate_runtime
        fixture=BatchRuntimeTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        for binding,code in [(dict(route='m4_up',interaction='noncontact'),'generic_route_only'),
                             (dict(route='generic',interaction='grasped'),'noncontact_empty_only'),
                             (dict(route='generic',interaction='contact'),'noncontact_empty_only')]:
            plan=copy.deepcopy(fixture.plan);plan['runtime']['collectionAdmission']={}
            plan['runtime']['conditionBindings']={plan['conditions'][0]['id']:binding}
            with patch('learning.batch_collection_domain.verify_collection_admission',return_value=dict(observerAgreement={})), \
                 patch('learning.batch_runtime.SequencePort',side_effect=AssertionError('device')):
                with self.assertRaisesRegex(ValueError,code): validate_runtime(plan,fixture.repository)

    def test_generic_go_does_not_supply_missing_onsite_authorization(self):
        from learning.batch_collection_domain import validate_manual_authorization
        from unittest.mock import patch
        plan=dict(campaignId='c',controllerBootId='b',viewId='v',budgets={},conditions=[dict(command=self.command)],
            runtime=dict(poseContext='p',collectionAdmission={},observerSpecPath='s',observerSpecSha256='h',gripper={}))
        value=dict(schemaVersion='motionbrain.manual_collection_authorization.v1',campaignId='c',controllerBootId='b',
            viewId='v',budgets={},commands=[self.command],poseContext='p',collectionAdmission={},
            observerBinding=dict(observerSpecPath='s',observerSpecSha256='h',gripper={}),authorizationSource={},
            stagingReviewSource={},selectionSemantics='predeclared_manual_collection_not_model_selected',
            currentFramesReviewed=False,contactVerified=False)
        with patch('learning.batch_collection_domain._read',side_effect=[value,dict(userInstruction='진행해')]):
            with self.assertRaisesRegex(ValueError,'explicit_onsite_authorization'):
                validate_manual_authorization('.',plan,{})

    def test_independent_prefixed_originals_qualify_declared_domain(self):
        import json
        import cv2
        import numpy as np
        from pathlib import Path
        from tests import test_batch_commissioning as commissioning
        from learning.registered_gripper_landmarks import register_jaws
        from learning.batch_commissioning import build_transition
        from learning.batch_collection_domain import build_collection_admission, verify_collection_admission
        from learning.batch_runtime import sha
        from learning.experiment_pipeline import encoded
        f=commissioning.CommissioningTests();f.setUp();self.addCleanup(f.doCleanups)
        reg=f.f.registration
        jaws=register_jaws(f.f.raw,{k:v for k,v in reg['points'].items() if k!='object'},reg['parameters'])
        (f.local/'points.json').write_bytes(encoded(jaws))
        path=f.local/'gripper-points.json';wrapper=json.loads(path.read_bytes())
        wrapper.update(kind='registered_gripper_jaws_stationary',registrationSha256=sha(encoded(jaws)))
        path.write_bytes(encoded(wrapper));f.binding['gripper']['observerSpecSha256']=sha(path.read_bytes())
        dev=f.make('a')[1]
        command=dict(joint='gripper',direction='close',percent=40,durationMs=350)
        domain=dict(schemaVersion=DOMAIN_SCHEMA,semantics='manual_noncontact_collection_not_model_control',
            jawMinimum=[113,127,185,127],jawMaximum=[129,143,201,143],minimumSearchMarginPx=2,maximumGapChangePx=8,
            commands=[dict(command=command,beforeGap=[70,74],afterGap=[66,70])])
        scope=dict(schemaVersion='motionbrain.motion_validation_scope.v1',development=[dev],observerBinding=f.binding,
                   maxLabelTolerancePx=.2,collectionDomain=domain)
        path=f.local/'scope.json';path.write_bytes(encoded(scope));f.validation_scope=dict(path=str(path),sha256=sha(path.read_bytes()))
        manifest=dict(development=[dev],holdout=[],validationScope=f.validation_scope)
        for sid in ('b','c'):
            _,entry=f.make(sid);folder=Path(entry['sourcePath'])/'gripper-pair'
            image=cv2.imdecode(np.frombuffer(f.f.raw,np.uint8),cv2.IMREAD_COLOR)
            image[:,:160]=np.roll(image[:,:160],2,axis=1);image[:,160:]=np.roll(image[:,160:],-2,axis=1)
            raw=cv2.imencode('.jpg',image)[1].tobytes();(folder/'esp32-after.jpg').write_bytes(raw)
            path=folder/'esp32-after.json';meta=json.loads(path.read_bytes());meta['sha256']=sha(raw);path.write_bytes(encoded(meta))
            transition=build_transition(f.repository,entry['sourcePath'],entry['sourceSha256'],f.binding,'independent','evaluation')
            self.assertEqual(transition['status'],'observed',transition.get('reason'))
            path=Path(entry['transitionPath']);path.write_bytes(encoded(transition));entry['transitionSha256']=sha(path.read_bytes())
            row=transition['learningRow'];labels={k:row[k] for k in ('sourceHash','frameIds','pairedFrameIds','stateFields','before','after')}
            labels.update(method='manual_projected_state_review',reviewer='synthetic-only',tolerancePx=.2,visible=True,identityVerified=True)
            path=f.local/(sid+'-labels.json');path.write_bytes(encoded(labels))
            manifest['holdout'].append(dict(transition=entry,labels=dict(path=str(path),sha256=sha(path.read_bytes()))))
        artifact=build_collection_admission(f.repository,manifest)
        self.assertEqual(list(artifact['qualifiedSampleCounts'].values()),[2])
        self.assertFalse(artifact['controlEligible'])
        path=f.local/'qualified.json';path.write_bytes(encoded(artifact));binding=dict(path=str(path),sha256=sha(path.read_bytes()))
        self.assertEqual(verify_collection_admission(f.repository,binding),artifact)

    def test_collection_token_keeps_source_deadline_and_original_bytes(self):
        import tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import patch
        from learning.batch_collection_domain import before_command
        from learning.batch_interaction import before_run_cached
        from learning.batch_runtime import sha
        with tempfile.TemporaryDirectory() as directory:
            repo=Path(directory).resolve();root=repo/'.local'/'source'
            pair=root/'gripper-pair';native=root/'native-camera';pair.mkdir(parents=True);native.mkdir()
            for folder,names in [(pair,('esp32-before.jpg','esp32-before.json')),(native,('frame.bin','frame.json'))]:
                for name in names:(folder/name).write_bytes(b'original')
            auth=root/'auth.json';auth.write_bytes(b'{}');binding=dict(path=str(auth),sha256=sha(b'{}'))
            value=dict(authorizationSource=binding,stagingReviewSource=binding)
            manager=SimpleNamespace(store=SimpleNamespace(root=root),camera=SimpleNamespace(root=native),
                port=SimpleNamespace(off_verified=True,last_sequence=3))
            clock=SimpleNamespace(monotonic=lambda:9.,time=lambda:109.)
            plan=dict(conditions=[self.command],maxObservationAgeSeconds=3)
            external=dict(source=dict(frame=dict(sourceEarliest=9.,imageFile='frame.bin',metadataFile='frame.json',imageSha256=sha(b'original'),metadataSha256=sha(b'original'))))
            paired=dict(observation=dict(status='accepted',frameBinding=dict(capturedAt=0.)),
                fileHashes={name:sha(b'original') for name in ('esp32-before.jpg','esp32-before.json')})
            with patch('learning.batch_collection_domain.validate_manual_authorization',return_value=value), \
                 patch('learning.batch_collection_domain._read',return_value=dict(sourceFrames=[])):
                token=before_command(manager,repo,plan,self.command,binding,120.,clock,
                    external=external,paired=paired,gripper_max_age=10.)
            self.assertEqual(token.expiresAt,10.)
            self.assertFalse(token.report['currentFramesReviewed'])
            self.assertFalse(token.report['graspVerified'])
            with self.assertRaisesRegex(ValueError,'expired'):
                before_run_cached(token,11.,111.,3)
            (pair/'esp32-before.jpg').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'original_evidence_changed'):
                before_run_cached(token,9.,109.,3)
