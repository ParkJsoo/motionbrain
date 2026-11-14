"""읽기 전용 원본 감사 및 명시 주석 결합 회귀검증. 장치 사용 없음."""
import copy
import unittest
from unittest.mock import patch

from learning import task_transition_data as data
from learning import m4_up_commissioning as core
from learning.supervised_collection import plan_digest
from tests import test_m4_lift_data as fixture_module


class TaskTransitionDataTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture_module.LiftDataTests()
        self.fixture.requestedPulse = getattr(self,'pulse',core.PULSE)
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root = self.fixture.root
        scope = self.fixture.read('scope.json')
        for key in ('phase','observationContractSha256','modelPath','modelSha256'):
            scope.pop(key)
        schema=getattr(self,'manual_schema',core.SCHEMA)
        scope.update(schemaVersion=schema,maxObservedDeltaDeg=3. if schema==core.RETURN_SCHEMA else 1.)
        if schema==core.RETURN_SCHEMA:
            scope['goalAngleDeg']=194.
            scope['context']['supportMode']='gripper_supported'
            for side in ('before','after'):
                record=self.fixture.read(side+'-observation.json')
                record['context']=copy.deepcopy(scope['context'])
                record['onsite']['tableSupportVerified']=False
                review={k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
                self.fixture.write(record['evidenceFile'],review)
                record['evidenceSha256']=core.sha((self.root/record['evidenceFile']).read_bytes())
                self.fixture.write(side+'-observation.json',record)
        self.fixture.write('scope.json',scope)
        result = self.fixture.read('result.json')
        result.update(schemaVersion=schema,scopeSha256=plan_digest(scope))
        self.fixture.write('result.json',result)
        # The shared raw fixture has no post-camera sensor polling. Its separate
        # clock-window contract is tested below; actual four sources use real polls.
        self.angle_patch = patch.object(data, '_angle', return_value=195.)
        self.angle_patch.start(); self.addCleanup(self.angle_patch.stop)

    def audit(self):
        return data.audit_manual_m4(self.root)

    def annotations(self):
        report = self.audit()
        registration = dict(schemaVersion=data.REGISTRATION_SCHEMA,
            context={k:report['originalContext'][k] for k in ('object','physicalView','poseRegime')},
            cameraBinding=report['pairs']['before']['cameraBinding'],
            acceptedSourceContexts=[report['originalContext']],measurementMethod='explicit-test-annotation')
        annotation = dict(schemaVersion=data.ANNOTATION_SCHEMA,registrationSha256=plan_digest(registration),
            sourceResultSha256=report['sourceResultSha256'],measurementMethod=registration['measurementMethod'])
        for side in ('before','after'):
            annotation[side] = dict(state=dict(objectRisePx=0.,toolRisePx=0.,slipDyPx=0.,
                startAngleDeg=report['anglesDeg'][side]),
                **{camera:{k:report['pairs'][side][camera][k] for k in ('frameId','pixelSha256')}
                   for camera in ('external','gripper')})
        return registration, annotation

    def test_manual_source_is_readonly_and_needs_annotations(self):
        before = {str(p):p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        result = self.audit()
        self.assertEqual(result['status'],'raw_verified_annotation_required')
        self.assertFalse(result['trainingEligible'])
        self.assertFalse(result['controlEligible'])
        self.assertEqual(before,{str(p):p.read_bytes() for p in self.root.rglob('*') if p.is_file()})

    def test_model_row_uses_only_annotated_visual_state_and_pre_sensor(self):
        registration, annotation = self.annotations()
        row = data.build_manual_m4_row(self.root,registration,annotation)
        self.assertEqual(set(row),{'id','context','supportMode','command','before','after','sourceHash','episode','frameIds'})
        self.assertEqual(row['context']['registrationSha256'],plan_digest(registration))
        self.assertEqual(row['before'],annotation['before']['state'])
        self.assertNotIn('receipt',row)
        self.assertEqual(len(row['frameIds']),4)
        self.assertTrue(all('|' not in identity for identity in row['frameIds']))

    def test_failed_or_unknown_output_never_becomes_row(self):
        for status in ('failed','unknown_output'):
            with self.subTest(status=status):
                original=self.fixture.read('result.json')
                changed=copy.deepcopy(original);changed['status']=status
                self.fixture.write('result.json',changed)
                with self.assertRaisesRegex(ValueError,'task_completed_manual_raw_required'):
                    self.audit()
                self.fixture.write('result.json',original)

    def test_tampered_result_scope_binding_rejected(self):
        result=self.fixture.read('result.json');result['scopeSha256']='f'*64
        self.fixture.write('result.json',result)
        with self.assertRaisesRegex(ValueError,'task_completed_manual_raw_required'):self.audit()

    def test_wrong_pixels_registration_or_input_angle_rejected(self):
        registration, original = self.annotations()
        for mode in ('pixels','registration','angle','output_input'):
            annotation=copy.deepcopy(original)
            if mode=='pixels':annotation['before']['external']['pixelSha256']='f'*64
            if mode=='registration':annotation['registrationSha256']='f'*64
            if mode=='angle':annotation['before']['state']['startAngleDeg']+=1
            if mode=='output_input':annotation['before']['state']['actualPwm']=120
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                data.build_manual_m4_row(self.root,registration,annotation)

    def test_scene_and_camera_cannot_be_implicitly_transferred(self):
        registration,annotation=self.annotations()
        registration['context']['object']='different'
        annotation['registrationSha256']=plan_digest(registration)
        with self.assertRaisesRegex(ValueError,'task_registration_scene_changed'):
            data.build_manual_m4_row(self.root,registration,annotation)
        registration,annotation=self.annotations()
        registration['cameraBinding']['external']['videoMirrored']=True
        annotation['registrationSha256']=plan_digest(registration)
        with self.assertRaisesRegex(ValueError,'task_camera_registration_changed'):
            data.build_manual_m4_row(self.root,registration,annotation)

    def test_missing_raw_camera_rejects_before_annotation(self):
        path=self.root/'gripper/esp32-before.jpg';path.unlink()
        with self.assertRaises(Exception):self.audit()

    def test_parent_model_selection_is_not_relabelled_manual_training(self):
        result=self.fixture.read('result.json')
        for value in (True,None,'false'):
            changed=copy.deepcopy(result);changed['parentModelSelected']=value
            self.fixture.write('result.json',changed)
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'model_trial_not_manual_training'):self.audit()

    def test_missing_angle_window_rejected(self):
        self.angle_patch.stop()
        with self.assertRaisesRegex(ValueError,'task_angle_window_missing'):
            data._angle([], 10., 12.)

    def test_no_relaxation_for_unknown_implementation(self):
        scope=self.fixture.read('scope.json');scope['implementationSha256']='f'*64
        self.fixture.write('scope.json',scope)
        with self.assertRaisesRegex(ValueError,'m4_recorded_implementation_unknown'):self.audit()


class TaskTransitionDownDataTests(TaskTransitionDataTests):
    pulse=core.RETURN_PULSE
    manual_schema=core.RETURN_SCHEMA

    def test_down_signed_output_is_retained(self):
        self.assertEqual(self.audit()['command']['direction'],'down')
        receipt=self.fixture.read('result.json')['receipt']
        self.assertTrue(all(e['pwm'] < 0 for e in receipt['outputTrace'] if e['pwm']))


if __name__=='__main__':unittest.main()
