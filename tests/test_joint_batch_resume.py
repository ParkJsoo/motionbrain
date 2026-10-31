import copy
import unittest
from collections import Counter
from learning.joint_batch import SPLITS,COMMANDS
from learning.joint_batch_resume import RemainingCollection,checkpoint,make_scope
from tests.test_joint_batch import simulation


def report():
 rows=[];executed=set(range(1,12))|set(range(13,21))
 for i in range(1,121):
  yes=i in executed;d,ms=COMMANDS[i%6]
  rows.append(dict(trial=dict(id=f'trial-{i:03d}',session=(i-1)//12,split=SPLITS[(i-1)//12]),
   executed=yes,executionStatus='executed' if yes else 'not_dispatched',status='eligible' if yes else 'not_started',
   commandAttempts=[{'recorded':True}] if yes else [],pulse=dict(direction=d,durationMs=ms,pwmMagnitude=160),execution=dict(offVerified=True)))
 return dict(schemaVersion='motionbrain.m2_batch_result.v1',scopeClosed=True,finalIdleOffVerified=True,status='failed',completedCaptures=[0],trials=rows)

class ResumeTests(unittest.TestCase):
 def test_only_101_unstarted_ids_and_prior_condition_counts(self):
  r=report();s=simulation();s.scope=make_scope('remaining',r);s.runner=RemainingCollection(s.scope,r,s.camera,s.port,s.store,clock=s.clock)
  self.assertEqual(sum(s.runner.condition_counts.values()),19)
  result=s.run();self.assertEqual(result['status'],'completed',result['errors'])
  self.assertEqual(result['summary']['planned'],101);self.assertEqual(result['summary']['executed'],101)
  ids={x['trial']['id'] for x in result['trials']}
  self.assertEqual(ids,{'trial-012'}|{f'trial-{i:03d}' for i in range(21,121)})
  self.assertEqual(Counter(x['trial']['split'] for x in result['trials']),dict(train=53,validation=24,test=24))
  self.assertEqual(Counter(x['trial']['session'] for x in result['trials']),{0:5,**{i:12 for i in range(1,9)}})
 def test_unknown_attempt_failed_off_or_changed_schedule_rejected(self):
  for change in ('unknown','off','attempt','schedule','executed'):
   r=report()
   if change=='unknown':r['trials'][11]['executed']=None
   elif change=='off':r['finalIdleOffVerified']=False
   elif change=='attempt':r['trials'][11]['commandAttempts']=[{}]
   elif change=='schedule':r['trials'][11]['trial']['split']='test'
   else:r['trials'][11]['executed']=True
   with self.assertRaises(ValueError):checkpoint(r)
 def test_scope_is_bound_to_checkpoint(self):
  r=report();scope=make_scope('remaining',r);r['extra']='different'
  s=simulation()
  with self.assertRaises(ValueError):RemainingCollection(scope,r,s.camera,s.port,s.store)
 def test_no_reexecution_on_ambiguous_run_response(self):
  r=report();s=simulation();s.scope=make_scope('remaining',r);s.runner=RemainingCollection(s.scope,r,s.camera,s.port,s.store,clock=s.clock);s.failure='response'
  result=s.run();self.assertEqual(len(s.commands),1);self.assertEqual(result['summary']['unknown'],1)
