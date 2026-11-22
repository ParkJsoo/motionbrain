import tempfile
import threading
import time
import unittest
from pathlib import Path

from PIL import Image

from learning.action_execution import BoundedActionExecutor
from learning.dual_view_policy import (ACTIONS, Catalog, DualViewPolicy, GripperDispatchGate,
                                       VisualControl, run_actions, thumbnail)
from tests.test_action_execution import FakeClock, FakePort, LookupWorkerFixture
from unittest.mock import Mock


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
    def test_model_hold_cancels_with_status_lookup_outstanding(self):
        clock = FakeClock()
        port = VisualPort(clock)
        label = 'wrist/down'
        catalog = Catalog({label:dict(command=command(*ACTIONS[label]),shoulderRangeDeg=[208,212])})
        active_at = []
        def active(req,vote,ack):
            if req is not None:
                active_at.append(ack)
        def latest():
            now = clock.monotonic()
            choice = 'hold' if active_at and now-active_at[0] >= .06 else label
            return dict(pair(choice,now,now),previousPair=pair(label,now-.01,now-.01))
        never_ready = Mock()
        never_ready.done.return_value = False
        worker = LookupWorkerFixture(lambda _:never_ready)
        control = VisualControl(latest,.5,.4,clock=clock.monotonic)
        log = []
        with tempfile.TemporaryDirectory() as tmp:
            core = BoundedActionExecutor(Path(tmp)/'commands.db',port,catalog,clock=clock)
            try:
                outcome = run_actions(control,catalog,core,'a'*32,lambda:(210,clock.monotonic()),
                                      1,log.append,sleep=clock.sleep,on_active=active,lookup_worker=worker)
            finally:
                core.close()
        self.assertEqual(outcome,'action_budget')
        self.assertEqual(port.posts,port.cancels)
        self.assertEqual(len(port.posts),1)
        self.assertTrue(log[0]['cancelConfirmed'])
        self.assertTrue(log[0]['lookupOutstandingAtCancel'])
        self.assertEqual(log[0]['motionFeedback'][-1]['modelAction'],'hold')
        never_ready.result.assert_not_called()

    def test_gripper_request_waits_for_release_and_stops_if_session_ends(self):
        for interrupted in (False, True):
            with self.subTest(interrupted=interrupted):
                gate = GripperDispatchGate(threading.Event(), threading.Event())
                stop = threading.Event()
                gate.request_pause()
                may_capture = []
                child = threading.Thread(target=lambda:may_capture.append(
                    gate.before_capture(stop, time.monotonic()+2.,check=lambda:None)), daemon=True)
                child.start()
                try:
                    self.assertTrue(gate.paused.wait(1.))
                    self.assertFalse(may_capture)
                    if interrupted:
                        stop.set()
                    else:
                        gate.resume()
                    child.join(1.)
                    self.assertEqual(may_capture, [not interrupted])
                    self.assertFalse(gate.paused.is_set())
                finally:
                    stop.set()
                    gate.resume()
                    child.join(1.)

    def test_gripper_pause_detects_external_camera_failure(self):
        gate = GripperDispatchGate(threading.Event(), threading.Event())
        gate.request_pause()
        check = Mock(side_effect=[None, ValueError('external_camera_lost')])
        with self.assertRaisesRegex(ValueError,'external_camera_lost'):
            gate.before_capture(threading.Event(),time.monotonic()+1.,check=check)
        self.assertFalse(gate.paused.is_set())

    def test_ack_alignment_holds_gripper_through_submit_and_releases_on_ack(self):
        for pause_available in (True, False):
            with self.subTest(pause_available=pause_available), tempfile.TemporaryDirectory() as tmp:
                clock = FakeClock()
                port = VisualPort(clock)
                port.running_reads = 3
                gate = GripperDispatchGate(threading.Event(), threading.Event())
                label = 'wrist/down'
                catalog = Catalog({label:dict(command=command(*ACTIONS[label]),shoulderRangeDeg=[208,212])})
                core = BoundedActionExecutor(Path(tmp)/'commands.db', port, catalog, clock=clock)
                ack = []
                def before_submit():
                    self.assertTrue(gate.requested.is_set())
                    self.assertTrue(gate.paused.is_set())
                    self.assertFalse(ack)
                port.before_submit = before_submit
                def active(req, vote, at):
                    if req is not None:
                        self.assertTrue(gate.requested.is_set())
                        ack.append(at)
                def next_sample():
                    now = clock.monotonic()
                    if ack and not gate.requested.is_set() and now >= ack[0]+.02:
                        return pair(label,ack[0]+.02,3)
                    return dict(pair(label,100.01,2),previousPair=pair(label,100.005,1))
                def sleep(delta):
                    clock.sleep(delta)
                    if pause_available and gate.requested.is_set():
                        gate.paused.set()
                    else:
                        gate.paused.clear()
                control = VisualControl(next_sample,.5,.4,clock=clock.monotonic)
                log = []
                try:
                    outcome = run_actions(control,catalog,core,'a'*32,lambda:(210,100. if not port.posts else clock.monotonic()),
                                          1,log.append,sleep=sleep,gripper_gate=gate,on_active=active)
                finally:
                    core.close()
                self.assertFalse(gate.requested.is_set())
                self.assertEqual(len(port.posts), int(pause_available))
                if pause_available:
                    self.assertEqual(outcome,'action_budget')
                    self.assertGreaterEqual(log[0]['gripperReleaseAfterAckAt'],ack[0])
                    self.assertEqual(log[0]['observedRunningPairs'],1)
                    self.assertFalse(port.cancels)
                else:
                    self.assertEqual(outcome,'not_sent')
                    self.assertFalse(ack)

    def test_two_views_must_agree_on_demonstrated_action(self):
        images = [Image.new('RGB', (32, 24), (20*i, 50, 220-20*i)) for i in range(10)]
        examples = [(label, thumbnail(im), thumbnail(im)) for label, im in zip(ACTIONS, images)]
        policy = DualViewPolicy(examples, [.05, .05])
        for i, label in enumerate(ACTIONS):
            self.assertEqual(policy.predict(images[i], images[i])['action'], label)
            self.assertEqual(policy.predict(images[i-1], images[i])['action'], 'hold')

    def test_synchronized_guard_rechecks_pose_after_independent_new_vote(self):
        now = [2.15]
        def latest():
            seq = 1 if now[0] < 2.18 else 2
            stamp = 2.05 + .1*(seq-1)
            return dict(pair('wrist/down', stamp, seq),
                        previousPair=pair('wrist/down', stamp-.1, seq-1))
        control = VisualControl(latest, .5, .2, clock=lambda: now[0])
        checks = []
        def verify():
            checks.append(now[0])
            now[0] += .04
        def sleep(delta):
            now[0] += delta
        admitted = control.dispatch_admission('wrist/down', 1.8, verify, synchronize=True, sleep=sleep)
        self.assertEqual(len(checks), 1)
        self.assertGreaterEqual(checks[0], 2.18)
        self.assertEqual(admitted[1]['frameId'], 'side-2')

    def test_synchronized_guard_rejects_hidden_dissent_or_stalled_camera_before_submit(self):
        for failure in ('stalled', 'lost', 'hidden', 'during_pose'):
            with self.subTest(failure=failure):
                now = [2.15]
                def latest():
                    if now[0] < 2.18 or failure == 'stalled':
                        return dict(pair('wrist/down', 2.05, 1), previousPair=pair('wrist/down', 1.95, 0))
                    if failure == 'lost':
                        return {'fault':True}
                    s = dict(pair('wrist/down', 2.16, 2), previousPair=pair('wrist/down', 2.1, 3))
                    if failure == 'hidden' or (failure == 'during_pose' and now[0] >= 2.3):
                        s['lastDissent'] = {'wrist/down':2.11}
                    return s
                def sleep(delta):
                    now[0] += delta
                control = VisualControl(latest, .5, .2, clock=lambda:now[0])
                with self.assertRaisesRegex(ValueError,'visual_start_changed|final_admission_expired'):
                    control.dispatch_admission('wrist/down', 1.8, lambda:sleep(.2), synchronize=True, sleep=sleep)

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

    def test_side_guard_rejects_uninspected_admission_and_late_hazard(self):
        for covered in (False, True):
            with self.subTest(covered=covered), tempfile.TemporaryDirectory() as tmp:
                clock = FakeClock()
                port = VisualPort(clock)
                label = 'elbow/down'
                catalog = Catalog({label:dict(command=command(*ACTIONS[label]), shoulderRangeDeg=[208, 212])})
                core = BoundedActionExecutor(Path(tmp)/'commands.db', port, catalog, clock=clock)
                def latest():
                    stamp = clock.monotonic()
                    return dict(pair(label, stamp, stamp), previousPair=pair(label, stamp-.01, stamp-.01))
                guard = Mock()
                guard.covers.return_value = covered
                guard.check.side_effect = lambda *args, **kwargs: (
                    dict(reason='tip_missing') if port.reads and port.last_record['state']=='completed' else None)
                guard.last_source = 0.
                log = []
                try:
                    outcome = run_actions(VisualControl(latest,.5,.4,clock=clock.monotonic), catalog, core,
                                          'a'*32, lambda:(210,clock.monotonic()), 2, log.append,
                                          sleep=clock.sleep, side_stop=guard)
                finally:
                    core.close()
                self.assertEqual(len(port.cancels), 0)
                if covered:
                    self.assertEqual(outcome, 'side_stop')
                    self.assertTrue(log[0]['motionFeedback'][-1]['terminalAlreadyObserved'])
                else:
                    self.assertFalse(log[0]['dispatchAttempted'])

    def test_finish_after_first_lift(self):
        clock = FakeClock()
        port = VisualPort(clock)
        label = 'elbow/up'
        catalog = Catalog({label:dict(command=command(*ACTIONS[label]), shoulderRangeDeg=[208, 212])})
        def latest():
            t = clock.monotonic()
            return dict(pair(label,t,t), previousPair=pair(label,t-.01,t-.01))
        with tempfile.TemporaryDirectory() as tmp:
            core = BoundedActionExecutor(Path(tmp)/'commands.db', port, catalog, clock=clock)
            try:
                outcome = run_actions(VisualControl(latest,.5,.4,clock=clock.monotonic),catalog,core,
                                      'a'*32,lambda:(210,clock.monotonic()),3,lambda r:None,
                                      sleep=clock.sleep,finish_after=label)
            finally:
                core.close()
        self.assertEqual(outcome,'observe_after_lift')
        self.assertEqual(len(port.posts),1)

    def test_feedback_trace_distinguishes_raw_vote_from_effective_hold(self):
        admitted = pair('wrist/up', 1.1, 1)
        sample = dict(pair('hold', 1.4, 2), modelAction='wrist/up')
        control = VisualControl(lambda: sample, .5, .2, clock=lambda: 1.5)
        trace = []
        check = control.feedback('wrist/up', 1., trace, admitted_pair=admitted)
        self.assertEqual(check({}, 1.2), 'cancel')
        self.assertEqual(trace[-1]['modelAction'], 'wrist/up')
        self.assertEqual(trace[-1]['effectiveAction'], 'hold')

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
