import tempfile
import unittest
from pathlib import Path

from PIL import Image

from learning.action_execution import BoundedActionExecutor
from learning.dual_view_policy import ACTIONS, Catalog, DualViewPolicy, VisualControl, run_actions, thumbnail
from tests.test_action_execution import FakeClock, FakePort


def command(motor, sign):
    shoulder = motor == 4
    return dict(kind='shoulder' if shoulder else 'pulse', motor=motor, durationMs=500,
                pwm=0 if shoulder else 30*sign, percent=75 if shoulder else 0,
                targetDegrees=210 + 10*sign if shoulder else 0)


def pair(label, stamp, index):
    clock = dict(sourceEarliest=stamp, sourceLatest=stamp, availableAt=stamp)
    return dict(action=label, lastDissent={}, extClock=clock, nearClock=clock,
                frameId=f'side-{index}', nearFrameId=f'near-{index}')


class VisualPort(FakePort):
    def cancel(self, request):
        return dict(super().cancel(request), cancelApplied=True)


class DualViewTest(unittest.TestCase):
    def test_two_views_must_agree_on_demonstrated_action(self):
        images = [Image.new('RGB', (32, 24), (20*i, 50, 220-20*i)) for i in range(10)]
        examples = [(label, thumbnail(im), thumbnail(im)) for label, im in zip(ACTIONS, images)]
        policy = DualViewPolicy(examples, [.05, .05])
        for i, label in enumerate(ACTIONS):
            self.assertEqual(policy.predict(images[i], images[i])['action'], label)
            self.assertEqual(policy.predict(images[i-1], images[i])['action'], 'hold')

    def test_changes_cancel_all_ten_running_commands_with_driver_off_proof(self):
        self.assertEqual(len(ACTIONS), 10)
        for label, (motor, sign) in ACTIONS.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                clock = FakeClock()
                port = VisualPort(clock)
                port.running_reads = 4
                cmd = command(motor, sign)
                catalog = Catalog({label: dict(command=cmd, shoulderRangeDeg=[208, 212])})
                core = BoundedActionExecutor(Path(tmp)/'commands.db', port, catalog, clock=clock)
                def latest():
                    stamp = clock.monotonic()
                    current = 'hold' if port.reads else label
                    sample = pair(current, stamp, stamp)
                    sample['previousPair'] = pair(current, stamp-.01, stamp-.01)
                    return sample
                control = VisualControl(latest, .5, .4, clock=clock.monotonic)
                log = []
                try:
                    outcome = run_actions(control, catalog, core, 'a'*32,
                                          lambda: (210, clock.monotonic()), 1, log.append, sleep=clock.sleep)
                finally:
                    core.close()
                self.assertEqual(outcome, 'action_budget', log)
                self.assertEqual([r['motor'] for r in port.cancels], [motor])
                self.assertTrue(log[0]['cancelConfirmed'])
                self.assertEqual(log[0]['motionFeedback'][-1]['reason'], 'visual_choice_changed')
                self.assertTrue(log[0]['action']['record']['outputOff'])

    def test_new_joint_needs_new_post_off_pairs_and_own_cancel_ack(self):
        first, second = 'elbow/down', 'gripper/close'
        catalog = Catalog({label: dict(command=command(*ACTIONS[label]), shoulderRangeDeg=[208, 212])
                           for label in (first, second)})
        for owned, expected in ((True, [3, 1]), (False, [3])):
            with self.subTest(owned=owned), tempfile.TemporaryDirectory() as tmp:
                clock = FakeClock()

                class SwitchingPort(VisualPort):
                    def submit(self, request):
                        self.running_reads = 2
                        self.reads = 0
                        return super().submit(request)

                    def cancel(self, request):
                        return dict(super().cancel(request), cancelApplied=owned)

                port = SwitchingPort(clock)
                core = BoundedActionExecutor(Path(tmp)/'commands.db', port, catalog, clock=clock)
                log = []

                def latest():
                    label = second if log or (port.posts and port.reads) else first
                    now = clock.monotonic()
                    sample = pair(label, now, now)
                    sample['previousPair'] = pair(label, now-.01, now-.01)
                    return sample

                control = VisualControl(latest, .5, .25, clock=clock.monotonic)
                try:
                    outcome = run_actions(control, catalog, core, 'a'*32, lambda: (210, clock.monotonic()),
                                          2, log.append, sleep=clock.sleep)
                finally:
                    core.close()
                self.assertEqual([r['motor'] for r in port.posts], expected)
                self.assertEqual(outcome, 'action_budget' if owned else 'unowned_or_late_cancel')
                self.assertEqual(len(port.cancels), 1)
                if owned:
                    self.assertTrue(log[1]['observedRunningPairs'])
                    self.assertGreater(log[1]['offConfirmedAt'], log[0]['offConfirmedAt'])

    def test_lost_visual_cancel_is_never_retried(self):
        clock = FakeClock()
        class LostCancel(FakePort):
            def cancel(self, request):
                super().cancel(request)
                raise TimeoutError('lost cancel reply')

        port = LostCancel(clock)
        label = 'wrist/up'
        catalog = Catalog({label: dict(command=command(*ACTIONS[label]), shoulderRangeDeg=[208, 212])})
        with tempfile.TemporaryDirectory() as tmp:
            core = BoundedActionExecutor(Path(tmp)/'commands.db', port, catalog, clock=clock)
            try:
                req = catalog.request(label, 210, 'a'*32, 'uncertain')
                core.propose(req)
                result = core.confirm('uncertain', confirmed_request=req, dispatch_guard=lambda _: None,
                                      motion_feedback=lambda *_: 'cancel')
                self.assertEqual(result['executionState'], 'outcome_unknown')
                self.assertEqual(len(port.cancels), 1)
                self.assertFalse(result['cancelConfirmed'])
            finally:
                core.close()

    def test_dissent_survives_mailbox_overwrite_and_old_prior_expires(self):
        item = pair('base/right', 2., 0)
        now = [2.1]
        control = VisualControl(lambda: item, .5, .25, clock=lambda: now[0])
        item['previousPair'] = pair('base/right', 1.4, -1)
        self.assertIsNone(control.admission(item, 1.))
        check = control.feedback('base/right', 1.)
        item.update(pair('base/right', 2.2, 2), lastDissent={'base/right': 2.15})
        now[0] = 2.3
        self.assertEqual(check({}, 2.11), 'cancel')

    def test_mixed_pre_off_pair_and_expired_motion_evidence(self):
        item = pair('base/right', 2., 0)
        control = VisualControl(lambda: item, .5, .25, clock=lambda: now[0])
        now = [2.1]
        item['extClock'] = dict(item['extClock'], sourceEarliest=1.)
        self.assertEqual(control.decision(1.), 'hold')
        item.update(pair('base/right', 2., 0))
        check = control.feedback('base/right', 1.)
        self.assertEqual(check({}, 2.1), 'continue')
        now[0] = 2.51
        self.assertEqual(check({}, 2.1), 'cancel')
