import copy,unittest
import numpy as np
from learning.joint_collection import conditions,action_for,validate_action,SPLITS
from learning.joint_prediction import features,fit,predict,validate_rows,train_and_evaluate


def rows():
    out=[]
    for i in range(4):
        for c in conditions(i):
            angle=(8.5 if c['pose']=='A' else 4.5)+i*.03
            x=features(angle,0.,c['direction'],c['durationMs'])
            out.append(dict(sessionIndex=i,sessionId=f's{i}',split=SPLITS[i],conditionId=c['conditionId'],x=x,y=6*x[4]))
    return out


class CollectionTests(unittest.TestCase):
    def test_each_session_has_same_eight_unique_conditions_and_locked_splits(self):
        for i in range(4):self.assertEqual({c['conditionId'] for c in conditions(i)},{c['conditionId'] for c in conditions(0)})
        self.assertEqual(SPLITS,('train','train','validation','test'))
    def test_target_and_positioning_are_separate_and_bounded(self):
        c=conditions(0)[0]
        self.assertEqual(action_for(c,8.5,0,0)['kind'],'target')
        for angle,ms in [(6.9,550),(4,700),(-1,900)]:
            a=action_for(c,angle,0,0);self.assertEqual(a['durationMs'],ms);self.assertEqual(a['direction'],'up');validate_action(a)
        for used,total in [(2,0),(0,16)]:
            with self.assertRaises(ValueError):action_for(c,4.,used,total)
        self.assertEqual(action_for(c,8.5,2,16)['kind'],'target')
    def test_outside_envelope_and_invalid_commands_rejected(self):
        for angle in [-11,25,float('nan'),True]:
            with self.assertRaises(ValueError):action_for(conditions(0)[0],angle,0,0)
        a=action_for(conditions(0)[0],8.5,0,0)
        for k,v in [('motor',4),('expectedPwm',255),('durationMs',1000),('kind','home'),('direction','up')]:
            b=dict(a);b[k]=v
            with self.assertRaises(ValueError):validate_action(b)


class PredictionTests(unittest.TestCase):
    def test_complete_locked_group_split(self):
        validate_rows(rows())
        for mutate in [lambda r:r.pop(),lambda r:r[0].update(split='test'),lambda r:r[8].update(sessionId='s0'),lambda r:r[1].update(conditionId=r[0]['conditionId']),lambda r:r[0]['x'].__setitem__(2,99)]:
            rr=rows();mutate(rr)
            with self.assertRaises(ValueError):validate_rows(rr)
    def test_test_labels_do_not_select_model_or_normalization(self):
        rr=rows();first=train_and_evaluate(rr)
        for r in rr:
            if r['split']=='test':r['y']+=1000
        second=train_and_evaluate(rr)
        self.assertEqual(first['selectedModel'],second['selectedModel']);self.assertEqual(first['candidates'],second['candidates'])
        self.assertNotEqual(first['splits']['test'],second['splits']['test'])
    def test_train_only_scaling_and_known_mapping(self):
        rr=rows();result=train_and_evaluate(rr);model=result['selectedModel']
        x=np.asarray([r['x'] for r in rr if r['split']=='train'])
        np.testing.assert_allclose(model['mean'],x.mean(axis=0));self.assertEqual(model['fitRows'],16)
        self.assertLess(result['splits']['test']['selected']['maeDeg'],.01)
        self.assertGreater(result['splits']['test']['noChange']['maeDeg'],1)
        self.assertFalse(result['controlEligible'])
    def test_invalid_fit_inputs(self):
        with self.assertRaises(ValueError):fit([[1]*6],[1],1)
        with self.assertRaises(ValueError):fit(np.full((16,6),np.nan),np.zeros(16),1)
    def test_after_output_trace_not_a_feature(self):
        r=rows()[0];before=r['x'].copy();r['actualPwm']=200;r['y']=999
        self.assertEqual(r['x'],before)
if __name__=='__main__':unittest.main()
