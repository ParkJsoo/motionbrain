import copy
import unittest

from learning.batch_dispatch_budget import reserve
from tests import test_batch_runtime as runtime_fixture


class DispatchBudgetTests(unittest.TestCase):
    def setUp(self):
        fixture = runtime_fixture.BatchRuntimeTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.f = fixture

    def test_failed_attempt_consumed_and_new_request_cannot_reset_condition(self):
        plan = copy.deepcopy(self.f.plan)
        plan['conditions'][0]['maxAttempts'] = 1
        condition = plan['conditions'][0]
        reserve(self.f.repository, plan, condition, 'failed-trial')
        with self.assertRaisesRegex(ValueError, 'request_consumed'):
            reserve(self.f.repository, plan, condition, 'failed-trial')
        with self.assertRaisesRegex(ValueError, 'condition_exhausted'):
            reserve(self.f.repository, plan, condition, 'new-trial-root')

    def test_shared_plan_budget_across_conditions(self):
        plan = copy.deepcopy(self.f.plan)
        second = dict(plan['conditions'][0], id='wrist-other')
        plan['conditions'].append(second)
        plan['budgets']['maxPhysicalCommands'] = 1
        reserve(self.f.repository, plan, plan['conditions'][0], 'first')
        with self.assertRaisesRegex(ValueError, 'plan_exhausted'):
            reserve(self.f.repository, plan, second, 'second')

    def test_preparation_and_unknown_conditions(self):
        plan = copy.deepcopy(self.f.plan)
        condition = plan['conditions'][0]
        condition['phase'] = 'preparation'
        reserve(self.f.repository, plan, condition, 'first')
        with self.assertRaisesRegex(ValueError, 'preparation_exhausted'):
            reserve(self.f.repository, plan, condition, 'second')
        with self.assertRaisesRegex(ValueError, 'condition_not_in_plan'):
            reserve(self.f.repository, plan, dict(condition, id='invented'), 'third')
