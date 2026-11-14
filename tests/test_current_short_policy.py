"""새 짧은 paired 정책의 고정 계약과 기존 프로필 비혼합 검사."""
import unittest
from learning.current_m4_short import current_short_policy,validate_current_short_policy,short_task_pulse


class CurrentShortPolicyTests(unittest.TestCase):
    def test_exact_new_paired_policy_preserves_both_planned_goals_and_caps(self):
        policy=current_short_policy();self.assertEqual(validate_current_short_policy(policy),policy)
        self.assertEqual((short_task_pulse('up')['durationMs'],short_task_pulse('down')['durationMs']),(350,200))
        self.assertEqual(policy['originalGlobalUpperDeg'],198.29)
        self.assertEqual(policy['maxObservedDeltaDeg'],.9)
        self.assertLessEqual(policy['tableReferenceDeg']+.10+policy['maxObservedDeltaDeg'],policy['originalGlobalUpperDeg'])
    def test_no_boolean_cap_or_changed_goal_global_or_monitor_policy(self):
        for key,value in [('upDurationCapMs',650),('downDurationCapMs',400),('maxObservedDeltaDeg',1.),('maxObservedDeltaDeg',3.),
                          ('upPercent',True),('tableReferenceDeg',195.29),('downGoalAngleDeg',196.59),
                          ('upGoalAngleDeg',198.0),('originalGlobalUpperDeg',199.),
                          ('upMonitorTimingPolicy','m4-target-up-deadline-650-v1')]:
            with self.subTest(key=key),self.assertRaises(ValueError):
                validate_current_short_policy(dict(current_short_policy(),**{key:value}))
        with self.assertRaises(ValueError):validate_current_short_policy(dict(current_short_policy(),extra=True))
        with self.assertRaises(ValueError):short_task_pulse('close')
