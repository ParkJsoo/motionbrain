"""명시 M1 개발 여유의 strict 기본/원본 불변/선택 경로 검사. 실물 요청 없음."""
import copy
import unittest
import json
import hashlib
import tempfile
from pathlib import Path
from unittest.mock import patch
from learning import current_m1_uncertainty as admission
from learning import m1_shaft_learning as shaft
from learning import task_action_models as actions
from learning.task_cycle_workflow import FrozenCycleModelBank
from learning.pipeline_learning import digest
from tests.test_m1_shaft_learning import scalar_sample


def fixture(direction='close',after=False):
    rows=[scalar_sample('uncertainty-a','train',85.16),scalar_sample('uncertainty-b','train',85.19)]
    for row in rows:
        row['context'].update(physicalView='user-inserted-current-gripper-20261114',
            poseRegime='table-supported-M4-19722-20261114',object='pink-prism-user-inserted-20261114')
        row['context']['cameraIdentity']['cameraBootId']=admission.CAMERA_BOOT
        row['command']=dict(joint='gripper',direction=direction,percent=60,durationMs=650)
        row['afterShaftPx']=66.5 if direction=='close' else 100.
    model=(shaft.fit_after if after else shaft.fit)(rows)
    policy=dict(schemaVersion=admission.SCHEMA,epsilonPx=.125,developmentOnly=True,
        statisticalCoverageClaim=False,modelSha256=digest(model),context=copy.deepcopy(model['context']),
        command=copy.deepcopy(model['models'][0]['command']),registrationSha256='a'*64,
        guardConfigSha256='b'*64,calibration=dict(path='calibration/result.json',sha256='c'*64),liveEligible=True)
    return model,policy


class AdmissionTests(unittest.TestCase):
    def test_explicit_small_extrapolation_preserves_raw_model_and_old_rejection(self):
        model,policy=fixture();original=copy.deepcopy(model);command=policy['command'];x=85.15
        with self.assertRaisesRegex(ValueError,'outside_training_range'):
            shaft.predict(model,x,command,model['context'])
        prediction,evidence=admission.predict_with_admission(model,x,command,model['context'],policy)
        self.assertEqual(evidence['rawShaftSeparationPx'],x)
        self.assertAlmostEqual(evidence['outsideDistancePx'],.01)
        self.assertTrue(evidence['extrapolated']);self.assertFalse(evidence['trainingSupportExpanded'])
        e=model['models'][0]
        expected=x+e['deltaMean']+(x-e['mean'])/e['scale']*e['coefficient']
        self.assertAlmostEqual(prediction,expected);self.assertEqual(model,original)
        with self.assertRaises(ValueError):shaft.predict(model,x,command,model['context'])

    def test_both_edges_bounded_and_pending_or_wrong_contract_rejected(self):
        model,policy=fixture();e=model['models'][0]
        for x in (e['minimum']-.125,e['maximum']+.125):
            admission.predict_with_admission(model,x,policy['command'],model['context'],policy)
        for x in (e['minimum']-.126,e['maximum']+.126):
            with self.assertRaisesRegex(ValueError,'distance_exceeded'):
                admission.predict_with_admission(model,x,policy['command'],model['context'],policy)
        for key,value in [('liveEligible',False),('epsilonPx',.126),('modelSha256','d'*64),
                          ('statisticalCoverageClaim',True)]:
            bad=copy.deepcopy(policy);bad[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):
                admission.predict_with_admission(model,85.15,policy['command'],model['context'],bad)

    def test_close_and_release_selectors_record_admission_without_clamping(self):
        for direction in ('close','open'):
            model,policy=fixture(direction);target=66.5 if direction=='close' else 100.
            def select(p):
                if direction=='close':return shaft.select(model,85.15,target,[policy['command']],model['context'],observation_admission=p)
                return actions.select_open(model,85.15,target,[policy['command']],model['context'],
                    table_supported=True,m4_angle_deg=197.22,expected_m4_angle_deg=197.22,observation_admission=p)
            self.assertEqual(select(None)['selected']['kind'],'stop')
            selected=select(policy)['selected'];self.assertEqual(selected['kind'],'action')
            self.assertEqual(selected['observationAdmission']['rawShaftSeparationPx'],85.15)
            self.assertTrue(selected['observationAdmission']['uncertaintyAdmitted'])

    def test_bank_reselection_same_policy_and_mutation_source_mismatch_fail(self):
        model,policy=fixture();bank=object.__new__(FrozenCycleModelBank)
        bank.models={'close':model};bank.hashes={'close':digest(model)}
        bank.m1_observation_policies={'close':policy};bank._m1_policies_sha=digest(bank.m1_observation_policies)
        stage=dict(kind='close',modelSha256=digest(model),context=model['context'],
            target={'shaftSeparationPx':66.5},commands=[policy['command']])
        record=dict(context=model['context'],state={'shaftSeparationPx':85.15},m1ObservationPolicySha256=digest(policy))
        first=bank.choose(stage,record);self.assertEqual(first,bank.choose(stage,record))
        with self.assertRaisesRegex(ValueError,'source_binding'):bank.choose(stage,dict(record,m1ObservationPolicySha256='f'*64))
        bank.m1_observation_policies['close']['epsilonPx']=1.
        with self.assertRaisesRegex(ValueError,'policy_changed'):bank.choose(stage,record)

    def test_in_range_is_exact_old_prediction_and_missing_command_not_admitted(self):
        model,policy=fixture();x=85.17
        value,evidence=admission.predict_with_admission(model,x,policy['command'],model['context'],policy)
        self.assertEqual(value,shaft.predict(model,x,policy['command'],model['context']))
        self.assertFalse(evidence['extrapolated'])
        with self.assertRaisesRegex(ValueError,'identity_changed'):
            admission.predict_with_admission(model,x,dict(policy['command'],durationMs=600),model['context'],policy)

    def test_current_after_regression_uses_original_after_coefficients(self):
        model,policy=fixture(after=True);x=85.15;entry=model['models'][0]
        original=copy.deepcopy(model)
        value,_=admission.predict_with_admission(model,x,policy['command'],model['context'],policy)
        self.assertEqual(value,entry['afterMean']+(x-entry['mean'])/entry['scale']*entry['afterCoefficient'])
        self.assertEqual(model,original)
        with self.assertRaisesRegex(ValueError,'outside_training_range'):
            shaft.predict(model,x,policy['command'],model['context'])

    def test_source_record_uses_bank_digest_for_unicode_policy_path(self):
        from types import SimpleNamespace
        from tests.test_current_full_cycle_source import SourceTests
        source=SourceTests();source.setUp();self.addCleanup(source.doCleanups)
        model,policy=fixture();policy['calibration']['path']='관측/정지/result.json'
        source.port.bank=SimpleNamespace(m1_observation_policies={'lift':policy})
        record=source.record()
        self.assertEqual(record['m1ObservationPolicySha256'],digest(policy))


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        from learning.artifacts import ArtifactStore
        from learning.supervised_transport import SequencePort
        from tests.test_supervised_transport import Clock
        from tests.test_m4_up_commissioning import Device
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.folder=self.root/'calibration';self.folder.mkdir()
        self.model,self.policy=fixture();self.clock=Clock();device=Device(self.clock);device.boot='boot';device.angle=197.22
        port=SequencePort('http://controller.invalid','secret','boot',ArtifactStore(self.folder/'controller'),
                          compact_reads=True,clock=self.clock);port.opener=device
        before=port.preflight();start=self.clock.now+1.;self.clock.now=start+2.;after=port.preflight()
        self.registration=b'{}';self.reference=b'image';self.guard=b'guard'
        self.policy['registrationSha256']=hashlib.sha256(self.registration).hexdigest()
        self.policy['guardConfigSha256']=hashlib.sha256(self.guard).hexdigest()
        rows=[];self.frames={};values=[66.6+i*.01 for i in range(6)]
        for i,value in enumerate(values):
            folder=self.folder/f'capture-{i}';folder.mkdir();(folder/'esp32-frame.jpg').write_bytes(str(i).encode())
            frame=dict(frameId=f'frame-{i}',height=240,width=320,pixelSha256='a'*64)
            metadata=dict(requestStartedAt=start+i*.1,receivedAt=start+i*.1+.01,persistedAt=start+i*.1+.02)
            self.frames[str(folder)]=(frame,metadata,admission.CAMERA_BOOT)
            record=dict(context=self.model['context'],hostClockId='stationary',frame=frame,
                        sourceEarliest=metadata['requestStartedAt'],sourceLatest=metadata['receivedAt'],persistedAt=metadata['persistedAt'])
            rows.append(dict(record=record,measurement={'shaftSeparationPx':value},imagePath=f'calibration/capture-{i}/esp32-frame.jpg'))
        self.report=dict(status='passed',maximumShaftSpanPx=.125,statisticalGuarantee=False,motorCommands=0,
            cameraBootId=admission.CAMERA_BOOT,fiveOff=True,registrationSha256=self.policy['registrationSha256'],
            controllerBootId='boot',outputSequence=port.last_sequence,controllerBefore=before,controllerAfter=after,
            rows=rows,values=values,spanPx=max(values)-min(values))
        (self.folder/'predeclared-plan.json').write_text(json.dumps(dict(captures=6,maximumShaftSpanPx=.125,
            statisticalGuarantee=False,motorCommands=0,expectedCursor=port.last_sequence,cameraBootId=admission.CAMERA_BOOT,controllerBootId='boot')))
        self.reader=patch('learning.gripper_camera_dataset.read_frame',side_effect=lambda p,*a:self.frames[str(p)])
        self.reader_mock=self.reader.start();self.addCleanup(self.reader.stop)
        self.tracker=patch('learning.registered_gripper_crosssection.RegisteredGripperCrosssection')
        tracker=self.tracker.start();self.addCleanup(self.tracker.stop);self.tracker_mock=tracker
        tracker.return_value.shaft_landmark_contract=self.model['context']['landmarks']
        tracker.return_value.registration={'sceneContext':{k:self.model['context'][k] for k in ('physicalView','poseRegime','object')}}
        tracker.return_value.observe_shaft.side_effect=lambda raw,frame:self.report['rows'][int(raw)]['measurement']

    def audit(self):
        raw=json.dumps(self.report).encode();(self.folder/'result.json').write_bytes(raw)
        self.policy['calibration']=dict(path='calibration/result.json',sha256=hashlib.sha256(raw).hexdigest())
        return admission.audit_calibration(self.root,self.policy,self.model,self.registration,self.reference,self.guard)

    def test_six_sources_replayed_between_raw_OFF_proofs(self):
        self.assertEqual(self.audit()['sampleCount'],6);self.assertEqual(self.reader_mock.call_count,6)

    def test_matching_file_hash_does_not_hide_wrong_model_registration(self):
        self.tracker_mock.return_value.shaft_landmark_contract='another-registration'
        with self.assertRaisesRegex(ValueError,'model_registration_changed'):self.audit()

    def test_POST_cannot_be_promoted_by_passed_flag(self):
        path=next((self.folder/'controller').glob('*-intent.json'))
        value=json.loads(path.read_text());value['method']='POST';path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError,'GET_only'):self.audit()

    def test_raw_output_epoch_change_is_rejected_despite_passed_summary(self):
        for path in (self.folder/'controller').glob('*-response.json'):
            value=json.loads(path.read_text())
            if 'latestSequence' not in value['parsed']:continue
            value['parsed']['latestSequence']+=1
            value['rawUtf8']=json.dumps(value['parsed']);path.write_text(json.dumps(value));break
        with self.assertRaisesRegex(ValueError,'output_epoch_changed'):self.audit()

    def test_six_samples_or_matching_registration_are_mandatory(self):
        self.report['rows'].pop()
        with self.assertRaisesRegex(ValueError,'six_required'):self.audit()


if __name__=='__main__':unittest.main()
