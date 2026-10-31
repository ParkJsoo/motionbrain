import copy
import unittest
from collections import Counter
from unittest.mock import patch

from learning.joint_batch import BatchCollection, COMMANDS, SPLITS, make_scope, choose, features, validate_rows, train_and_evaluate
from tests.test_joint_goal_evaluation import Sim


def simulation():
    s=Sim();s.angle=7.49;s.scope=make_scope('batch-test')
    s.runner=BatchCollection(s.scope,s.camera,s.port,s.store,clock=s.clock)
    return s


def rows():
    return [dict(id=f'{i}-{j}',sessionIndex=i,sessionId=f'S{i}',split=SPLITS[i],direction=d,durationMs=ms,
                 x=features(3+i*.4+j*.1, .001*j,d,ms),y=(1 if d=='up' else -1)*(ms/100-1))
            for i in range(10) for j,(d,ms) in enumerate(COMMANDS*2)]


class BatchTests(unittest.TestCase):
    def test_120_targets_without_positioning_and_single_use(self):
        s=simulation();r=s.run()
        self.assertEqual(r['status'],'completed',r['errors']);self.assertEqual(r['summary']['eligible'],120)
        self.assertEqual(r['counts'],dict(target=120,positioning=0));self.assertEqual(len(s.commands),120)
        self.assertEqual(len(s.cameras),10);self.assertTrue(r['finalIdleOffVerified'])
        self.assertEqual(set((p['direction'],p['durationMs']) for _,p in s.commands),set(COMMANDS))
        with self.assertRaisesRegex(ValueError,'new_exact'):s.run()
    def test_edges_choose_inward_without_positioning(self):
        for a,d in [(-10,'up'),(5,'up'),(10,'down'),(24,'down')]:
            for i in range(12):self.assertEqual(choose({'angleDeg':a},{},i)['direction'],d)
        for a in [-10.01,24.01,float('nan')]:
            with self.assertRaises(ValueError):choose({'angleDeg':a},{},0)
    def test_unknown_execution_stops_without_retry(self):
        s=simulation();s.failure='response';r=s.run()
        self.assertEqual(len(s.commands),1);self.assertEqual(r['summary']['unknown'],1)
        self.assertEqual(r['summary']['eligible'],0);self.assertEqual(len(s.cameras),1)
    def test_label_rejection_continues_without_replacement(self):
        s=simulation()
        from learning.joint_batch import window
        calls=0
        def reject_once(*args):
            nonlocal calls
            calls+=1
            if calls==1:raise ValueError('insufficient_observation_window')
            return window(*args)
        with patch('learning.joint_batch.window',side_effect=reject_once):r=s.run()
        self.assertEqual(r['status'],'completed',r['errors']);self.assertEqual(r['summary']['rejected'],1)
        self.assertEqual(r['summary']['eligible'],119);self.assertEqual(len(s.commands),120)
    def test_tracking_fault_and_off_failure_are_fatal(self):
        for failure in ('camera','off'):
            s=simulation()
            if failure=='camera':s.camera_failure='tracking_lost'
            else:s.failure='off'
            r=s.run();self.assertEqual(r['status'],'failed');self.assertLessEqual(len(s.commands),1)
    def test_partial_block_keeps_denominator_and_next_capture(self):
        s=simulation();original=s.runner.trial
        def slow(row):original(row);s.clock.now+=3
        s.runner.trial=slow;r=s.run()
        self.assertEqual(r['status'],'completed',r['errors']);self.assertGreater(r['summary']['notStarted'],0)
        self.assertEqual(len(s.cameras),10);self.assertLess(len(s.commands),120)
    def test_dispatch_failure_and_invalid_receipt_never_label(self):
        for attr in ('bad_receipt','missing_guard','double_guard'):
            s=simulation();setattr(s,attr,True);r=s.run()
            self.assertEqual(r['status'],'failed');self.assertEqual(r['summary']['eligible'],0)
    def test_whole_capture_splits_and_variable_rows(self):
        rr=rows();validate_rows(rr)
        self.assertEqual(Counter(r['split'] for r in rr),dict(train=72,validation=24,test=24))
        validate_rows(rr[6:])
        rr[0]['split']='test'
        with self.assertRaisesRegex(ValueError,'split'):validate_rows(rr)
    def test_leakage_missing_conditions_duplicate_rejected(self):
        for change in ('session','duplicate','condition'):
            rr=rows()
            if change=='session':
                for r in rr:
                    if r['sessionIndex']==5:r['sessionId']='S0'
            elif change=='duplicate':rr[1]['id']=rr[0]['id']
            else:rr=[r for r in rr if not (r['split']=='test' and r['durationMs']==550)]
            with self.assertRaises(ValueError):validate_rows(rr)
    def test_test_labels_do_not_change_selection_or_training_scaling(self):
        a=rows();b=copy.deepcopy(a)
        for r in b:
            if r['split']=='test':r['y']+=1000
        x=train_and_evaluate(a);y=train_and_evaluate(b)
        self.assertEqual(x['selectedModel'],y['selectedModel']);self.assertEqual(x['candidates'],y['candidates'])
        self.assertEqual(x['selectedModel']['fitRows'],72);self.assertFalse(x['controlEligible'])
        self.assertGreater(y['splits']['test']['selected']['maeDeg'],900)
        self.assertEqual(len(x['splits']['test']['byCommand']),6)
    def test_old_contract_still_excludes_550(self):
        from learning.joint_prediction import features as old_features
        with self.assertRaises(ValueError):old_features(4,0,'up',550)
        self.assertEqual(features(4,0,'up',550),[4,0,1,.55,.55,4])
