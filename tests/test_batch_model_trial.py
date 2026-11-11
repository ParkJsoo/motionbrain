import copy
import unittest

from learning.batch_learning import fit
from learning.batch_model_trial import BatchModelSequence, select
from tests.test_batch_learning import row


class BatchModelTrialTests(unittest.TestCase):
    def setUp(self):
        self.model=fit([row(1),row(3)])
        self.command=row(1)['command']
        self.observation=dict(status='accepted',context=self.model['context'],stateFields=self.model['stateFields'],
            state=[2.]*6,frameIds=['fresh'],observedAt=10.,persistedAt=10.1)
        self.stage=dict(id='approach',kind='position',modelId='wrist',target=[4.]*6,tolerance=[.2]*6,
            commands=[self.command],maxCommands=1)

    def sequence(self,**kwargs):
        return BatchModelSequence('one',[self.stage],{'wrist':self.model},**kwargs)

    def test_real_batch_model_type_selects_prediction_and_stop_ties(self):
        d=select(self.model,self.observation,[4.]*6,[1.]*6,[self.command])
        self.assertEqual(d['selected']['kind'],'action')
        self.assertEqual(d['selected']['predictedState'],[4.]*6)
        d=select(self.model,self.observation,[3.]*6,[1.]*6,[self.command])
        self.assertEqual(d['selected']['kind'],'stop')
        self.assertFalse(d['executionAuthorized'])

    def test_unsupported_candidates_remain_visible_and_do_not_execute(self):
        command=dict(self.command,durationMs=300)
        d=select(self.model,self.observation,[4.]*6,[1.]*6,[command])
        self.assertEqual(d['selected']['kind'],'stop')
        self.assertEqual(d['rejected'][0]['reason'],'unsupported_command')
        obs=copy.deepcopy(self.observation);obs['state']=[99.]*6
        self.assertEqual(select(self.model,obs,[4.]*6,[1.]*6,[self.command])['selected']['kind'],'stop')

    def test_changed_fields_context_and_bad_dimensions_rejected(self):
        for key,value in [('stateFields',['wrong']*6),('context',{}),('state',[1.])]:
            obs=copy.deepcopy(self.observation);obs[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):
                select(self.model,obs,[4.]*6,[1.]*6,[self.command])

    def complete(self,trial,request,**changes):
        audit=dict(status='verified',offVerified=True,command=self.command,sourceSha256='a'*64,commandAt=11.,offAt=12.)
        audit.update(changes)
        obs=copy.deepcopy(self.observation);obs.update(state=[4.]*6,frameIds=['after'],observedAt=12.1,persistedAt=12.2)
        return trial.complete(request['requestId'],audit,obs,now=12.3)

    def test_prediction_does_not_advance_stage_without_new_actual_observation(self):
        trial=self.sequence();request=trial.propose(self.observation,now=10.2)['request']
        self.assertEqual(trial.index,0)
        with self.assertRaisesRegex(ValueError,'not_ready'):trial.propose(self.observation,now=10.3)
        self.assertFalse(self.complete(trial,request)['complete'])
        obs=copy.deepcopy(self.observation);obs.update(state=[4.]*6,frameIds=['next'],observedAt=12.4,persistedAt=12.5)
        self.assertTrue(trial.propose(obs,now=12.6)['complete'])

    def test_failed_command_consumed_and_cannot_be_replaced(self):
        trial=self.sequence();request=trial.propose(self.observation,now=10.2)['request']
        with self.assertRaisesRegex(ValueError,'not_verified'):self.complete(trial,request,offVerified=False)
        with self.assertRaisesRegex(ValueError,'not_ready'):trial.propose(self.observation,now=10.3)
        with self.assertRaisesRegex(ValueError,'no_pending'):self.complete(trial,request)

    def test_reused_and_predispatch_frames_do_not_count_as_after(self):
        for ids,start in [(['fresh'],12.1),(['new'],11.)]:
            trial=self.sequence();request=trial.propose(self.observation,now=10.2)['request']
            obs=copy.deepcopy(self.observation);obs.update(frameIds=ids,observedAt=start,persistedAt=12.2)
            audit=dict(status='verified',offVerified=True,command=self.command,sourceSha256='a'*64,commandAt=11.,offAt=12.)
            with self.subTest(ids=ids),self.assertRaises(ValueError):trial.complete(request['requestId'],audit,obs,now=12.3)
            self.assertIsNotNone(trial.halted)

    def test_close_and_lift_require_external_review_and_new_observations(self):
        stages=[];models={};observations=[]
        for kind,joint,direction in [('close','gripper','close'),('lift','shoulder','up')]:
            rows=[row(1),row(3)]
            for item in rows:item['joint']=joint;item['command'].update(joint=joint,direction=direction)
            model=fit(rows);models[kind]=model
            stages.append(dict(self.stage,id=kind,kind=kind,modelId=kind,commands=[rows[0]['command']]))
            observations.append(dict(self.observation,context=model['context'],stateFields=model['stateFields']))
        blocked=BatchModelSequence('two',stages,models)
        with self.assertRaisesRegex(ValueError,'review_required'):blocked.propose(observations[0],now=10.2)
        seen=[]
        trial=BatchModelSequence('two',stages,models,review=lambda *args:seen.append(args[0]['kind']) or True)
        for i in range(2):
            start=10.+i*5
            obs=copy.deepcopy(observations[i]);obs.update(frameIds=['before'+str(i)],observedAt=start,persistedAt=start+.1)
            request=trial.propose(obs,now=start+.2)['request']
            after=dict(obs,state=[4.]*6,frameIds=['after'+str(i)],observedAt=start+2.1,persistedAt=start+2.2)
            result=trial.complete(request['requestId'],dict(status='verified',offVerified=True,command=request['command'],
                sourceSha256='a'*64,commandAt=start+1,offAt=start+2),after,now=start+2.3)
        self.assertTrue(result['complete']);self.assertFalse(result['physicalSuccessClaimed'])
        self.assertEqual(seen,['close','lift'])

    def test_contact_review_rejection_stops_lift(self):
        rows=[row(1),row(3)]
        for item in rows:item['joint']='gripper';item['command'].update(joint='gripper',direction='close')
        model=fit(rows);stage=dict(self.stage,id='close',kind='close',modelId='close',commands=[rows[0]['command']])
        trial=BatchModelSequence('close',[stage],{'close':model},review=lambda *args:False)
        request=trial.propose(self.observation,now=10.2)['request']
        with self.assertRaisesRegex(ValueError,'retention_unverified'):
            self.complete(trial,request,command=request['command'])
        self.assertEqual(trial.index,0);self.assertIsNotNone(trial.halted)

if __name__=='__main__':unittest.main()
