"""고정 모델의 UP 목표·RETURN 목표 정책이 실제 actor와 같아야 한다."""
import unittest
from learning.m4_up_commissioning import TARGET_UP_SCHEMA,RETURN_SCHEMA,target_up_policy
from learning.pipeline_learning import digest
from tests import test_task_cycle_workflow as fixtures

class TargetModelBindingTests(unittest.TestCase):
    def setUp(self):
        self.o=fixtures.ScopedExecutorTests();self.o.setUp()
        self.base=self.o.executor.m4_factory
        self.target=dict(schemaVersion=TARGET_UP_SCHEMA,goalAngleDeg=197.5,
                         returnGoalAngleDeg=196.59,globalReferenceDeg=195.29)
        self.policy=target_up_policy(self.target)
        self.lower=False;self.mutate=None
        def factory(stage,command,hooks):
            actor,_=self.base(stage,command,hooks)
            actor.scope.update(self.target)
            if self.lower:actor.scope.update(schemaVersion=RETURN_SCHEMA,goalAngleDeg=196.59)
            if self.mutate:self.mutate(actor)
            actor.digest=digest(actor.scope)
            return actor,actor.digest
        self.o.executor.m4_factory=factory

    def call(self):
        return self.o.executor(dict(kind='lower' if self.lower else 'lift',context={'executionPolicy':self.policy}),
            fixtures.DOWN if self.lower else fixtures.UP,prepare=lambda:None,before_run=lambda:None,
            decision={'model':'frozen'},safety=self.o.safety)

    def test_target_and_return_policy_bound_before_actor_run(self):
        self.assertEqual(self.call(),{'testReceipt':True});self.assertEqual(self.o.run_calls,1)
        self.lower=True
        self.assertEqual(self.call(),{'testReceipt':True});self.assertEqual(self.o.run_calls,2)

    def test_changed_UP_or_RETURN_goal_never_runs(self):
        self.mutate=lambda actor:actor.scope.update(goalAngleDeg=197.6)
        with self.assertRaisesRegex(ValueError,'target_up_policy_mismatch'):self.call()
        self.assertEqual(self.o.run_calls,0)
        self.lower=True
        with self.assertRaisesRegex(ValueError,'target_return_policy_mismatch'):self.call()
        self.assertEqual(self.o.run_calls,0)

    def test_missing_or_false_policy_never_promotes_TARGET(self):
        self.policy=None
        with self.assertRaisesRegex(ValueError,'target_up_policy_mismatch'):self.call()
        self.assertEqual(self.o.run_calls,0)

    def test_original_global_limit_applies_even_with_different_cycle_start(self):
        self.o.safety['M4ReferenceDeg']=196.69;self.o.device.angle=198.3
        with self.assertRaisesRegex(ValueError,'target_global_excursion'):self.call()
        self.assertEqual(self.o.run_calls,1)
