"""실제 scoped actor/단일촬영 observer 결합. 합성 장치·파일만 사용한다."""
import unittest

from learning import m4_up_commissioning as core
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort
from tests import test_m4_up_observer as fixtures


class TaskHookIntegrationTests(unittest.TestCase):
    setUp=fixtures.AdapterTests.setUp
    review=fixtures.AdapterTests.review
    adapter=fixtures.AdapterTests.adapter

    def make_trial(self,monitor=lambda status:None):
        observer=self.adapter();self.calls=[]
        def prepare():
            self.calls.append(('prepare',self.gripper.count,self.device.state))
            self.assertEqual(self.gripper.count,1)
            self.assertEqual(self.device.state,'ARMED')
        def before_run():
            self.calls.append(('before_run',self.gripper.count,self.device.state))
            self.assertEqual(self.calls[0][0],'prepare')
        hooks=dict(prepare=prepare,before_run=before_run,status_guard=monitor,decisionSha256='f'*64)
        trial=core.M4UpCommissioning(self.scope,observer,'private-token',self.store,
            evidence_root=self.store.root,clock=self.clock,task_hooks=hooks)
        trial.port.opener=self.device
        return observer,trial

    def test_after_arm_first_capture_preserves_real_observer_no_retake(self):
        observer,trial=self.make_trial()
        result=trial.run(confirmed_scope_sha256=plan_digest(self.scope))
        self.assertEqual(result['status'],'completed',result)
        self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual([c[0] for c in self.calls],['prepare','before_run'])
        self.assertEqual(observer.labels,{'before','after'})
        self.assertEqual(self.gripper.count,2)
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.device.requests),1)
        self.assertTrue((self.store.root/'before-observation.json').is_file())
        self.assertTrue((self.store.root/'task-parent-decision-binding.json').is_file())

    def test_active_cumulative_guard_stops_even_within_local_actor_delta(self):
        states=[]
        def monitor(status):
            angle=SequencePort._shoulder_angle(status);states.append(angle)
            require(abs(angle-217.5)<=3.,'cycle_active_M4_total_excursion')
        observer,trial=self.make_trial(monitor)
        self.device.after_run=lambda d:setattr(d,'angle',220.8)
        result=trial.run(confirmed_scope_sha256=plan_digest(self.scope))
        self.assertEqual(result['status'],'failed',result)
        self.assertTrue(result['finalIdleOffVerified'],result)
        self.assertIn(220.8,states)
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.device.requests),1)
        self.assertTrue(all(pwm==0 for pwm in self.device.speeds.values()))


if __name__=='__main__':unittest.main()
