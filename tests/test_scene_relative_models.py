import copy
import hashlib
import unittest
from learning.scene_relative_models import derive_transition, fit, SceneRelativeModelBank
from learning.scene_relative_observation import FIELDS
from learning.task_cycle_workflow import PrefixCycleModelBank, FrozenCycleModelBank
from tests.test_task_cycle_workflow import CycleWorkflowTests, APPROACH


class Observer:
    def observe(self, raw, frame_id, context):
        return dict(state=[float(raw),0.,10.],imageSha256=hashlib.sha256(raw).hexdigest(),
                    frameId=frame_id,sourceContext=copy.deepcopy(context),registrationSha256='a'*64,canonicalFrameSha256='c'*64,stateFields=list(FIELDS),trainingEligible=False)


def row(i, before, after):
    b=str(before).encode(); a=str(after).encode()
    source=dict(episode=['controller',str(i)],status='completed',finalIdleOffVerified=True,trainingEligible=True,command=APPROACH.copy(),
        physicalContext=dict(object='block',load='block',otherJointConfiguration={'M2': 'fixed', 'M5': 'fixed'},
                             executionTimingPolicy='elbow-deadline-350-v1',supportMode='table_supported'),
        before=dict(frameId=f'b:{i}:before',imageSha256=hashlib.sha256(b).hexdigest(),sourceContext={},m4AngleDeg=197.22),
        after=dict(frameId=f'b:{i}:after',imageSha256=hashlib.sha256(a).hexdigest(),sourceContext={},m4AngleDeg=197.22))
    return derive_transition(source,Observer(),b,a,audit_source=lambda s:s)


class SceneModelTests(unittest.TestCase):
    def setup_choice(self):
        self.rows=[row(0,0,1),row(1,2,3)]
        self.model=fit(self.rows);self.bank=SceneRelativeModelBank({'approach':self.model})
        self.stage=dict(kind='approach',modelSha256=self.bank.hashes['approach'],commands=[APPROACH.copy()],target=[3.,0.,10.,197.22])
        self.record=dict(state=[1.,0.,10.,197.22],physicalContext=copy.deepcopy(self.model['physicalContext']),
                         observationContract=copy.deepcopy(self.model['observationContract']))

    def test_actual_derived_prediction_and_stop_comparison(self):
        self.setup_choice();result=self.bank.choose(self.stage,self.record)
        self.assertEqual(result['selected']['predictedState'],[2.,0.,10.,197.22])
        self.assertEqual(len(result['candidates']),2);self.assertFalse(result['controlEligible'])
        self.assertTrue(all(o['trainingEligible'] is False for o in self.rows[0]['observations']))

    def test_physical_pose_never_normalized_away(self):
        self.setup_choice();self.record['state'][3]=195.38
        result=self.bank.choose(self.stage,self.record)
        self.assertEqual(result['rejected'][0]['reason'],'state_outside_training_range')

    def test_other_registration_not_relabelled(self):
        self.setup_choice();self.record['observationContract']['canonicalFrameSha256']='b'*64
        with self.assertRaisesRegex(ValueError,'observation_contract_changed'):self.bank.choose(self.stage,self.record)

    def test_missing_candidate_and_outside_state_reported(self):
        self.setup_choice();self.record['state']=[99.,0.,10.,197.22]
        self.stage['commands'].append(dict(APPROACH,direction='down'))
        result=self.bank.choose(self.stage,self.record)
        self.assertEqual(result['status'],'abstained');self.assertEqual(len(result['rejected']),2)
        self.assertEqual(result['rejected'][0]['supportRange']['maximum'],[2.,0.,10.,197.22])

    def test_single_row_deficit_no_fake_model(self):
        model=fit([row(0,0,1)]);self.assertEqual(model['models'],[])
        self.assertEqual(model['deficits'][0]['availableTransitions'],1)

    def test_repeated_source_and_changed_observation_rejected(self):
        r=row(0,0,1)
        with self.assertRaisesRegex(ValueError,'duplicate'):fit([r,r])
        r['before'][0]=99
        with self.assertRaisesRegex(ValueError,'binding_changed'):fit([r])

    def test_missing_observations_cannot_bypass_binding(self):
        rows=[row(0,0,1),row(1,2,3)]
        rows[0]['observations']=[];rows[0]['before']=[99.,0.,10.,197.22]
        with self.assertRaisesRegex(ValueError,'two_bound_observations'):fit(rows)

    def test_failed_source_is_not_promoted(self):
        r=row(0,0,1);r['source']['status']='unknown'
        from learning.pipeline_learning import digest
        r['sourceSha256']=digest(r['source'])
        with self.assertRaisesRegex(ValueError,'not_training_eligible'):fit([r])

    def test_image_to_derived_model_to_candidate_choice(self):
        from tests.test_scene_relative_observation import RelativeObservationTests, encoded
        fixture=RelativeObservationTests();fixture.setUp();rows=[]
        for i,start in enumerate((0,2)):
            before,_=fixture.transformed(1.,(10,10),dict(object=(start,0)))
            after,_=fixture.transformed(1.,(10,10),dict(object=(start+1,0)))
            b,a=encoded(before),encoded(after);source=copy.deepcopy(row(i,0,1)['source'])
            for side,raw in (('before',b),('after',a)):
                source[side].update(imageSha256=hashlib.sha256(raw).hexdigest(),
                    frameId='a'*32+f':{i}:{side}',sourceContext=fixture.context.copy())
            rows.append(derive_transition(source,fixture.observer,b,a,audit_source=lambda s:s))
        model=fit(rows);bank=SceneRelativeModelBank({'approach':model})
        state=[1.,-70.,120.,197.22]
        result=bank.choose(dict(kind='approach',modelSha256=bank.hashes['approach'],
            commands=[APPROACH],target=[3.,-70.,120.,197.22]),dict(state=state,
            physicalContext=model['physicalContext'],observationContract=model['observationContract']))
        self.assertEqual(result['selected']['kind'],'action')
        self.assertAlmostEqual(result['selected']['predictedState'][0],2.,places=5)
        self.assertFalse(result['controlEligible'])

    def test_freeze_is_one_time_and_file_hash_is_checked(self):
        import tempfile
        from pathlib import Path
        from learning.scene_relative_models import freeze_model
        self.setup_choice()
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'derived.json';binding=freeze_model(path,self.model)
            loaded=SceneRelativeModelBank.from_bindings({'approach':binding})
            self.assertEqual(loaded.hashes,self.bank.hashes)
            with self.assertRaises(FileExistsError):freeze_model(path,self.model)
            path.write_text('{}')
            with self.assertRaisesRegex(ValueError,'file_changed'):SceneRelativeModelBank.from_bindings({'approach':binding})

    def test_changed_load_rejected_separately_from_numeric_state(self):
        self.setup_choice();self.record['physicalContext']['load']='another-load'
        with self.assertRaisesRegex(ValueError,'physical_context_unsupported'):self.bank.choose(self.stage,self.record)


class PrefixBankTests(CycleWorkflowTests):
    def test_prefix_uses_real_selector_without_future_models(self):
        bank=PrefixCycleModelBank({'approach':self.approach})
        stage=bank.scope_stages([dict(kind='approach',commands=[APPROACH],target=[-2.,2.,10.])])[0]
        result=bank.choose(stage,dict(context=stage['context'],state=[2.,2.,10.]))
        self.assertFalse(result['fullCycleSupported']);self.assertEqual(result['supportedStages'],['approach'])
        with self.assertRaisesRegex(ValueError,'unavailable'):bank.choose(dict(kind='lift'),{})

    def test_old_full_bank_stays_strict_and_prefix_has_no_gaps(self):
        with self.assertRaisesRegex(ValueError,'bank_stages'):FrozenCycleModelBank({'approach':self.approach})
        with self.assertRaisesRegex(ValueError,'prefix_model'):PrefixCycleModelBank({'close':self.close})
