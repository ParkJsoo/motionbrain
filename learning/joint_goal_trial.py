"""One explicitly scoped observation/selection/execution trial via injected ports.

No network, camera discovery, ARM or firmware implementation lives here. A live
adapter must enforce the single-pulse, STOP/OFF and evidence contracts below.
"""
import copy
import time

import numpy as np

from learning.action_execution import finite
from learning.joint_action_selection import canonical_sha256


def validate_scope(scope):
    fields = {'schemaVersion', 'trialId', 'goalDeg', 'toleranceDeg', 'modelSha256',
              'datasetSha256', 'maxMotorCommands', 'captureSeconds', 'maxFrames',
              'pwmMagnitude', 'durationsMs'}
    if (not isinstance(scope, dict) or set(scope) != fields
            or scope['schemaVersion'] != 'motionbrain.single_m2_goal_trial.v1'
            or not isinstance(scope['trialId'], str) or not scope['trialId']
            or any(not isinstance(scope[k], str) or len(scope[k]) != 64
                   or any(c not in '0123456789abcdef' for c in scope[k])
                   for k in ('modelSha256', 'datasetSha256'))
            or not finite(scope['goalDeg']) or not -10 <= scope['goalDeg'] <= 24
            or not finite(scope['toleranceDeg']) or not 0 <= scope['toleranceDeg'] <= 2
            or any(type(scope[k]) is not int or scope[k] != value for k, value in
                   [('maxMotorCommands', 1), ('captureSeconds', 30), ('maxFrames', 320),
                    ('pwmMagnitude', 160)])
            or scope['durationsMs'] != [700, 900]
            or any(type(v) is not int for v in scope['durationsMs'])):
        raise ValueError('explicit_single_m2_goal_scope_required')
    return copy.deepcopy(scope)


def window(records, start, end, now, minimum):
    selected = []
    for r in records:
        if (not isinstance(r, dict)
                or not all(finite(r.get(k)) for k in ('sourceAt', 'receivedAt', 'angleDeg'))
                or not r['sourceAt'] <= r['receivedAt'] <= now):
            raise ValueError('invalid_observation_timing_or_angle')
        if start <= r['sourceAt'] <= end:
            selected.append(r)
    if len(selected) < minimum:
        raise ValueError('insufficient_observation_window')
    if (any(b['sourceAt'] <= a['sourceAt'] for a, b in zip(selected, selected[1:]))
            or any(r.get('valid') is not True or r.get('stable') is not True for r in selected)):
        raise ValueError('invalid_or_unsettled_observation_window')
    return selected


def pre_action_state(records, now):
    selected = window(records, now-.9, now-.1, now, 5)
    if selected[-1]['sourceAt']-selected[0]['sourceAt'] < .5:
        raise ValueError('insufficient_velocity_history')
    t = np.asarray([r['sourceAt'] for r in selected])
    a = np.asarray([r['angleDeg'] for r in selected])
    angle = float(np.median(a))
    velocity = float(np.polyfit(t-t[-1], a, 1)[0])
    if not finite(angle) or not finite(velocity):
        raise ValueError('nonfinite_observation_features')
    return dict(angleDeg=angle, velocityDegS=velocity, valid=True, settled=True,
                windowStart=float(t[0]), windowEnd=float(t[-1]), frames=len(selected))


def command_key(decision):
    selected = decision.get('selected')
    if decision.get('status') != 'selected' or not isinstance(selected, dict):
        return None
    if (selected.get('kind') != 'pulse' or selected.get('direction') not in ('up', 'down')
            or type(selected.get('durationMs')) is not int or selected['durationMs'] not in (700, 900)
            or type(selected.get('pwmMagnitude')) is not int or selected['pwmMagnitude'] != 160):
        raise ValueError('selector_candidate_outside_scope')
    return selected['direction'], selected['durationMs']


def latest_settled(records, now):
    if not records:
        raise ValueError('insufficient_observation_window')
    latest = records[-1]
    if (not isinstance(latest, dict)
            or not all(finite(latest.get(k)) for k in ('sourceAt', 'receivedAt'))
            or not latest['sourceAt'] <= latest['receivedAt'] <= now
            or not 0 <= now-latest['sourceAt'] <= .5):
        raise ValueError('latest_observation_stale')
    if latest.get('stable') is not True or latest.get('valid') is not True:
        raise ValueError('latest_observation_not_settled')


class SingleGoalTrial:
    """Camera: start/check/samples/finish; port: preflight/execute/finalize.

    execute(pulse, guard, tick) must call guard exactly once immediately before
    its only run POST, poll tick while active, then STOP and verify all outputs
    OFF. finalize must also handle ambiguous writes without retransmitting a
    motor command. A durable store must reject existing trial directories.
    """
    def __init__(self, scope, selector, camera, port, store, *, clock=time):
        self.scope = validate_scope(scope)
        if (scope['modelSha256'] != selector.model_sha256
                or scope['datasetSha256'] != selector.dataset_sha256):
            raise ValueError('scope_model_binding_mismatch')
        self.selector, self.camera, self.port, self.store, self.clock = selector, camera, port, store, clock
        self.used = False

    def run(self, *, confirmed_scope_sha256):
        if self.used:
            raise ValueError('goal_trial_single_use')
        if confirmed_scope_sha256 != canonical_sha256(self.scope):
            raise ValueError('exact_goal_scope_confirmation_required')
        self.used = True
        # Persist before even read-only device checks. Never reuse this attempt.
        self.store.write_json('trial-intent.json', self.scope)
        report = dict(scopeSha256=confirmed_scope_sha256, status='failed', errors=[],
                      executionAttempted=False, measurement=None, automaticRetry=False,
                      trainingPerformed=False, controlEligible=False, finalIdleOffVerified=False)
        final_decision = None
        dispatch_at = None
        guard_calls = 0

        def decide():
            self.camera.check()
            records = self.camera.samples()
            at = self.clock.monotonic()
            state = pre_action_state(records, at)
            # Feature history excludes the last .1s; still require the newest
            # observation to be settled immediately before arming a decision.
            latest_settled(records, at)
            decision = self.selector.select(state, self.scope['goalDeg'],
                                            tolerance_deg=self.scope['toleranceDeg'])
            return at, state, decision

        try:
            self.port.preflight()
            self.camera.start()
            startup = self.clock.monotonic()
            while True:
                if self.clock.monotonic()-startup > 35:
                    raise ValueError('initial_observation_deadline')
                try:
                    at, state, decision = decide()
                except ValueError as exc:
                    if str(exc) not in ('camera_not_live', 'insufficient_observation_window',
                                        'invalid_or_unsettled_observation_window',
                                        'insufficient_velocity_history', 'latest_observation_not_settled'):
                        raise
                    self.clock.sleep(.03)
                    continue
                break
            report['initialDecision'] = decision
            report['initialObservation'] = state
            self.store.write_json('initial-decision.json', dict(at=at, observation=state, decision=decision))
            key = command_key(decision)
            if key is None:
                report['status'] = decision['status']
            else:
                pulse = dict(direction=key[0], durationMs=key[1], pwmMagnitude=160)

                def dispatch_guard():
                    nonlocal final_decision, dispatch_at, guard_calls
                    guard_calls += 1
                    if guard_calls != 1:
                        raise ValueError('dispatch_guard_single_use')
                    dispatch_at, current, final_decision = decide()
                    if command_key(final_decision) != key:
                        raise ValueError('candidate_changed_before_dispatch')
                    if not finite(self.camera.first_source_at) or dispatch_at-self.camera.first_source_at > 20:
                        raise ValueError('insufficient_capture_tail')
                    self.store.write_json('dispatch-decision.json', dict(at=dispatch_at,
                                          observation=current, decision=final_decision))
                    self.camera.check()
                    newest = self.camera.samples()
                    latest_settled(newest, self.clock.monotonic())
                    if not 0 <= self.clock.monotonic()-dispatch_at <= .25:
                        raise ValueError('dispatch_decision_expired')
                    return dispatch_at

                report['executionAttempted'] = True
                receipt = self.port.execute(pulse, dispatch_guard, self.camera.check)
                report['execution'] = receipt
                if (guard_calls != 1 or not isinstance(receipt, dict)
                        or receipt.get('offVerified') is not True
                        or not finite(receipt.get('offAt'))
                        or not dispatch_at <= receipt['offAt'] < dispatch_at+3.75
                        or receipt.get('dispatchAt') != dispatch_at):
                    raise ValueError('fixed_horizon_off_receipt_required')
                # Preserve +4s labels. A later settling time never moves the label.
                while self.clock.monotonic() < dispatch_at+4.75:
                    self.camera.check()
                    remaining = dispatch_at+4.75-self.clock.monotonic()
                    if remaining > 0:
                        self.clock.sleep(min(.03, remaining))
                self.camera.check()
                selected = window(self.camera.samples(), dispatch_at+3.75, dispatch_at+4.25,
                                  self.clock.monotonic(), 3)
                after = float(np.median([r['angleDeg'] for r in selected]))
                before_error = final_decision['noChangeGoalErrorDeg']
                after_error = abs(self.scope['goalDeg']-after)
                report['measurement'] = dict(
                    beforeDeg=final_decision['angleDeg'], afterDeg=after,
                    predictedEndDeg=final_decision['selected']['predictedEndDeg'],
                    predictionErrorDeg=after-final_decision['selected']['predictedEndDeg'],
                    beforeGoalErrorDeg=before_error, afterGoalErrorDeg=after_error,
                    errorReductionDeg=before_error-after_error,
                    goalErrorReduced=after_error < before_error,
                    goalReached=after_error <= self.scope['toleranceDeg'],
                    windowStart=dispatch_at+3.75, windowEnd=dispatch_at+4.25,
                    physicalJointAngleCalibrated=False,
                )
                report['status'] = 'measured'
        except (Exception, KeyboardInterrupt) as exc:
            report['errors'].append(dict(type=type(exc).__name__, reason=str(exc)))
            report['status'] = 'failed'
        finally:
            # Stop/OFF cleanup precedes camera tail/storage, even on interrupts.
            try:
                report['finalIdleOffVerified'] = self.port.finalize() is True
                if not report['finalIdleOffVerified']:
                    raise ValueError('final_idle_off_unverified')
            except (Exception, KeyboardInterrupt) as exc:
                report['errors'].append(dict(type='finalize', reason=type(exc).__name__))
                report['status'] = 'failed'
            try:
                self.camera.finish()
            except (Exception, KeyboardInterrupt) as exc:
                report['errors'].append(dict(type='camera_finish', reason=type(exc).__name__))
                report['status'] = 'failed'
        report['scopeClosed'] = True
        self.store.write_json('trial-result.json', report)
        return report
