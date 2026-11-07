"""Public learning core with fake hardware and real immutable camera bindings."""
import copy
import json
from pathlib import Path
from unittest.mock import patch

from learning import m4_up_commissioning as core
from learning import m4_lift_learning as learning
from learning.artifacts import _read, _decode_json
from learning.supervised_collection import plan_digest
from tests.test_m4_up_observer import AdapterTests


ORIGINAL_MEASURE=core.M4UpCommissioning._measure

class UpLearningTests(AdapterTests):
    def prepare_learning(self, phase='training'):
        self.registration = dict(context=copy.deepcopy(self.scope['context']),
            cameraBinding=dict(external=dict(configurationSha256=core.sha(b'{"format":"fixed"}'),width=8,height=8,
                                            videoMirrored=False,videoRotationAngle=0),gripper=dict(width=32,height=32)))
        self.scope.update(schemaVersion=core.MODEL_SCHEMA if phase=='model_selection' else core.LEARNING_SCHEMA,
                          phase=phase,observationContractSha256=learning.contract_sha(self.registration),
                          modelPath=None,modelSha256=None,maxObservedDeltaDeg=3.)
        self.scope['pulse']=copy.deepcopy(core.LEARNING_PULSE)
        owner=self
        class Frozen:
            def __init__(self):
                self.registration=copy.deepcopy(owner.registration)
                self.path=owner.repo/'.local/frozen-test-model.json'
                self.path.write_bytes(b'{"synthetic_frozen_model":1}')
                self.sha=core.sha(self.path.read_bytes())
            def verify(self):
                if core.sha(self.path.read_bytes()) != self.sha:raise ValueError('frozen_model_changed')
                return True
            def decide(self,source,pulse,selected):
                self.verify()
                decision=dict(features=_decode_json(_read(owner.store.root,'before-measurement.json')),
                              pulse=copy.deepcopy(pulse),modelSha256=self.sha,
                              contractSha256=learning.contract_sha(owner.registration),modelSelected=selected,
                              predictions=[dict(pulse=p,state=[10,10,0]) for p in learning.PULSES])
                descriptor=owner.store.write_json('test-frozen-decision.json',decision)
                return dict(decision,decisionPath=str(owner.store.root/'test-frozen-decision.json'),
                            decisionSha256=descriptor['sha256'])
            def select(self,source):return self.decide(source,learning.PULSES[0],True)
            def predict(self,source,pulse):return self.decide(source,pulse,False)
        model=None if phase=='training' else Frozen()
        if model is not None:self.scope.update(modelPath=str(model.path),modelSha256=model.sha)
        observer=self.adapter()
        observer.wait_pre_window=lambda **kwargs:None
        observer.wait_post_window=lambda *args,**kwargs:None
        def measured(trial,label,record,frames):
            # Only metrology is synthetic; original native/JPEG provenance and
            # core pre-RUN source/decision guards still execute normally.
            external=label+'-external.png';grip=record['gripper']
            source=dict(id=trial.scope['hostClockId']+':'+label,context=copy.deepcopy(trial.scope['context']),
                        external=dict(path=external,sha256=core.sha(_read(trial.root,external))),
                        gripper=dict(path=grip['root']+f'/esp32-{label}.jpg',sha256=grip['imageSha256']))
            features=dict(id=source['id'],context=source['context'],contractSha256=learning.contract_sha(trial.registration),
                          state=[0,0,0] if label=='before' else [12,9,0],quality=dict(valid=True,ncc=1,slipDxPx=0),
                          sourceSha256={k:source[k]['sha256'] for k in ('external','gripper')})
            trial.store.write_json(label+'-measurement-source.json',source)
            trial.store.write_json(label+'-measurement.json',features)
            trial.measurement_records.append(source)
            return source,features
        patcher=patch.object(core.M4UpCommissioning,'_measure',measured)
        patcher.start();self.addCleanup(patcher.stop)
        trial=core.M4UpCommissioning(self.scope,observer,'unused-private',self.store,evidence_root=self.store.root,
                                   clock=self.clock,registration=self.registration,model=model)
        trial.port.opener=self.device
        return trial,model

    def run_learning(self,trial):
        return trial.run(confirmed_scope_sha256=plan_digest(self.scope))

    def test_registered_camera_change_rejects_before_run(self):
        trial,_=self.prepare_learning('training')
        trial.registration['cameraBinding']['external']['configurationSha256']='0'*64
        trial.scope['observationContractSha256']=learning.contract_sha(trial.registration)
        trial.digest=plan_digest(trial.scope)
        trial._measure=ORIGINAL_MEASURE.__get__(trial)
        result=trial.run(confirmed_scope_sha256=trial.digest)
        self.assertEqual(result['status'],'failed')
        self.assertFalse(any(v=={'action':'run'} for _,v in self.device.requests))

    def test_training_domain_rejection_precedes_run(self):
        trial,_=self.prepare_learning('training');trial._measure=ORIGINAL_MEASURE.__get__(trial)
        with patch.object(learning,'measure',return_value={'synthetic':'features'}),patch.object(learning,'validate_before',side_effect=ValueError('outside_registered_operating_domain')) as checked:
            result=self.run_learning(trial)
        self.assertEqual(result['status'],'failed');checked.assert_called_once()
        self.assertFalse(any(v=={'action':'run'} for _,v in self.device.requests))

    def test_model_ceiling_high_can_select_low_actual_wire(self):
        trial,model=self.prepare_learning('model_selection')
        checked=[]
        def at_run(device):
            decision=_decode_json(_read(self.store.root,'test-frozen-decision.json'))
            checked.append(decision['pulse'])
            model.verify()
        self.device.after_run=at_run
        result=self.run_learning(trial)
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(checked,[learning.PULSES[0]])
        self.assertEqual(result['receipt']['requestedPulse'],core.PULSE)
        self.assertTrue(result['modelSelected'])
        adds=[v for p,v in self.device.requests if p=='/sequence' and v and v.get('action')=='add']
        self.assertEqual(adds,[dict(action='add',joint='shoulder',direction='up',speed='16',duration='200')])

    def test_evaluation_persists_prediction_before_fixed_high_run(self):
        trial,model=self.prepare_learning('evaluation')
        seen=[]
        self.device.after_run=lambda device:seen.append((self.store.root/'test-frozen-decision.json').exists())
        result=self.run_learning(trial)
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(seen,[True]);self.assertFalse(result['modelSelected'])
        self.assertEqual(result['receipt']['requestedPulse'],core.LEARNING_PULSE)
        self.assertEqual(result['modelDecision']['modelSha256'],model.sha)

    def test_modified_decision_prevents_run_without_automatic_replacement(self):
        trial,_=self.prepare_learning('evaluation')
        original=trial._before_run
        def changed():
            (self.store.root/'test-frozen-decision.json').write_bytes(b'{}')
            return original()
        trial._before_run=changed
        result=self.run_learning(trial)
        self.assertEqual(result['status'],'failed')
        self.assertFalse(any(v=={'action':'run'} for _,v in self.device.requests))
        self.assertFalse(any(e['pwm'] for e in self.device.entries))

    def test_frozen_model_tampering_prevents_all_run_requests(self):
        trial,model=self.prepare_learning('model_selection')
        model.path.write_bytes(b'{}')
        result=self.run_learning(trial)
        self.assertEqual(result['status'],'failed')
        self.assertFalse(any(v=={'action':'run'} for _,v in self.device.requests))

    def test_new_training_allows_two_degree_change_without_changing_old_limit(self):
        self.assertEqual(self.scope['maxObservedDeltaDeg'],1.)
        trial,_=self.prepare_learning('training')
        self.device.after_run=lambda device:setattr(device,'angle',222.)
        result=self.run_learning(trial)
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(self.scope['maxObservedDeltaDeg'],3.)
        self.assertFalse(result['modelSelected'])

    def test_original_one_degree_commissioning_still_aborts_two_degree_change(self):
        observer=self.adapter()
        trial=core.M4UpCommissioning(self.scope,observer,'unused-private',self.store,
                                   evidence_root=self.store.root,clock=self.clock)
        trial.port.opener=self.device
        self.device.after_run=lambda device:setattr(device,'angle',222.)
        result=trial.run(confirmed_scope_sha256=plan_digest(self.scope))
        self.assertEqual(result['status'],'failed')
        self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.device.requests),1)
