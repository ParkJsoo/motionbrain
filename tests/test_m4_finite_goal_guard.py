"""명시 UP 목표 v3의 오프라인 계약. 구동·배포하지 않는다."""
import hashlib
import unittest
from learning import m4_device_guard_transport as wire
from learning.m4_device_guard_policy import finite_goal_device_guard_policy,device_guard_task_pulse,validate_device_guard_policy
from tests import test_m4_device_guard_transport as fixtures


class FiniteGoalTests(unittest.TestCase):
    def setup(self):
        port,controller,store,request=fixtures.GuardTests().setup_port()
        controller.angle=195.38
        request.update(policySha256=wire.FINITE_POLICY_SHA256,expectedStartDeg=195.38,goalDeg=197.5)
        original=controller.envelope
        def envelope():
            result=original();result['supportedPolicySha256'].append(wire.FINITE_POLICY_SHA256);return result
        controller.envelope=envelope;port.opener=controller
        return port,controller,store,request

    def test_explicit_goal_beyond_old_local_limit_uses_one_run(self):
        p,c,s,r=self.setup();policy=finite_goal_device_guard_policy(197.5)
        result=p._execute_validated(device_guard_task_pulse('up',policy),lambda:None,device_goal_guard=r)
        self.assertEqual(result['deviceGoalGuard']['goalDeg'],197.5)
        self.assertTrue(result['offVerified'])
        self.assertEqual(sum(bool(d and d.get('action')=='run_m4_guarded') for _,d in c.requests),1)

    def test_old_capability_rejects_before_guard_queue(self):
        p,c,s,r=self.setup();old=c.envelope
        def envelope():
            value=old();value['supportedPolicySha256'].remove(wire.FINITE_POLICY_SHA256);return value
        c.envelope=envelope
        with self.assertRaisesRegex(ValueError,'policy_capability_required'):
            p._execute_validated(device_guard_task_pulse('up'),lambda:None,device_goal_guard=r)
        self.assertFalse(any(d and d.get('action') in ('add_m4_guarded','run_m4_guarded') for _,d in c.requests))

    def test_limits_direction_and_exact_policy(self):
        _,_,_,r=self.setup();pulse=device_guard_task_pulse('up')
        for changes in (dict(goalDeg=197.8),dict(expectedStartDeg=194.9),dict(goalDeg=195.4),dict(tableReferenceDeg=194.32)):
            with self.subTest(changes=changes),self.assertRaises(ValueError):wire.validate_request(dict(r,**changes),pulse)
        policy=finite_goal_device_guard_policy(197.5)
        with self.assertRaises(ValueError):device_guard_task_pulse('down',policy)
        policy['maxObservedDeltaDeg']=4.
        with self.assertRaises(ValueError):validate_device_guard_policy(policy)
        self.assertEqual(hashlib.sha256(wire.FINITE_POLICY_CONTRACT.encode()).hexdigest(),wire.FINITE_POLICY_SHA256)

    def test_terminal_still_rejects_global_limit_and_lost_reply_never_retries(self):
        p,c,s,r=self.setup();c.bad=dict(lastAngleDeg=198.4)
        with self.assertRaisesRegex(ValueError,'terminal_angle'):
            p._execute_validated(device_guard_task_pulse('up'),lambda:None,device_goal_guard=r)
        self.assertEqual(sum(bool(d and d.get('action')=='run_m4_guarded') for _,d in c.requests),1)
        p,c,s,r=self.setup();c.lost_run=True
        with self.assertRaises(TimeoutError):
            p._execute_validated(device_guard_task_pulse('up'),lambda:None,device_goal_guard=r)
        self.assertTrue(p.may_have_run)
        self.assertEqual(sum(bool(d and d.get('action')=='run_m4_guarded') for _,d in c.requests),1)
