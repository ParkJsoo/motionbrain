"""One exact M2-up request with continuous evidence; never arm, retry or home."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import time

from learning.action_execution import ActionLimits, BoundedActionExecutor, canonical_request, finite
from learning.visual_action_collection import LIVE_SESSION_SCHEMA, VisualActionCollector, identifier


def validate_plan(plan):
    expected = {'schemaVersion', 'request', 'camera', 'points', 'beforeSeconds', 'afterSeconds',
                'maxTravelPx', 'referenceLimitPx', 'maxDispatchLagSeconds', 'helperSha256', 'startPoseId'}
    if not isinstance(plan, dict) or set(plan) != expected or plan['schemaVersion'] != 'motionbrain.visual_action_pilot.v1':
        raise ValueError('explicit_pilot_plan_required')
    r = canonical_request(plan['request'])
    if r['kind'] != 'pulse' or r['motor'] != 2 or not 1 <= r['pwm'] <= 180 or r['durationMs'] > 1500:
        raise ValueError('only_m2_positive_pwm_up_pilot_supported')
    identifier(plan['startPoseId'])
    c = plan['camera']
    if not isinstance(c, dict) or set(c) != {'deviceId', 'sessionId', 'viewId', 'width', 'height', 'videoMirrored', 'videoRotationAngle', 'maxFrames', 'seconds', 'sampleFps', 'maxBytes', 'maxFrameAgeSeconds'}:
        raise ValueError('camera_plan_fields_required')
    for k in ('deviceId', 'sessionId', 'viewId'): identifier(c[k])
    for k, lo, hi in (('width',32,1920),('height',32,1080),('maxFrames',2,600),('seconds',2,15),('sampleFps',1,30),('maxBytes',4,2**31)):
        if type(c[k]) is not int or not lo <= c[k] <= hi: raise ValueError('camera_budget_invalid')
    if type(c['videoMirrored']) is not bool or type(c['videoRotationAngle']) is not int or c['videoRotationAngle'] not in (0,90,180,270):
        raise ValueError('camera_orientation_required')
    for value, lo, hi in ((c['maxFrameAgeSeconds'],.05,.5),(plan['beforeSeconds'],.2,3),
                          (plan['afterSeconds'],1,5),(plan['maxDispatchLagSeconds'],.02,.25),
                          (plan['maxTravelPx'],1,200),(plan['referenceLimitPx'],.1,10)):
        if not finite(value) or not lo <= value <= hi: raise ValueError('pilot_budget_invalid')
    # Permit capture oversampling jitter; do not start a plan doomed to hit a budget.
    required_frames = c['seconds']*c['sampleFps']+2
    if c['maxFrameAgeSeconds'] < 1/c['sampleFps']+.1:
        raise ValueError('frame_age_requires_sampling_jitter_margin')
    if (c['maxFrames'] < required_frames or c['maxBytes'] < c['width']*c['height']*4*required_frames
            or c['seconds'] < plan['beforeSeconds']+r['durationMs']/1000+plan['afterSeconds']+2):
        raise ValueError('insufficient_camera_budget')
    if not isinstance(plan['helperSha256'], str) or len(plan['helperSha256']) != 64 or any(x not in '0123456789abcdef' for x in plan['helperSha256']):
        raise ValueError('helper_hash_required')
    # Validate all point fields before any device is opened; dependency is optional.
    from learning.visual_features import PointTracker
    PointTracker(c['width'],c['height'],plan['points'],max_displacement_px=plan['maxTravelPx'],reference_limit_px=plan['referenceLimitPx'])
    return copy.deepcopy(plan)


def plan_digest(plan):
    return hashlib.sha256(json.dumps(plan,sort_keys=True,separators=(',', ':'),allow_nan=False).encode()).hexdigest()


def observe_stream(camera, *, clock=time):
    """Commission the exact camera/tracker path without constructing a motor port."""
    interrupted = False
    error = None
    try:
        camera.start()
        start = clock.monotonic()
        while True:
            # Native spool and ingress receipts retain each frame; release only
            # redundant queue references. No action-labelled collection is made.
            camera.drain()
            if camera.fault: raise ValueError(camera.fault)
            if camera.done: break
            if clock.monotonic()-start > 47: raise ValueError('camera_process_timeout')
            clock.sleep(.01)
    except (Exception, KeyboardInterrupt) as exc:
        interrupted = isinstance(exc, KeyboardInterrupt)
        error = 'operator_interrupted' if interrupted else 'observation_failed'
    finally:
        try: camera.close()
        except Exception: error = error or 'camera_close_failed'
    return dict(mode='observation_only',observationComplete=error is None and camera.done,
                frameCount=camera.count,error=error,cameraFault=camera.fault,
                operatorInterrupted=interrupted,motorCalls=0,trainingEligible=False)


class RecordingPort:
    """Keep the executor's baseline and recheck the final no-I/O submit gate."""
    def __init__(self, port):
        self.port = port
        self.host_clock_id = port.host_clock_id
        self.baseline = None
        self.final_gate = lambda: None

    def preflight(self, request):
        self.baseline = self.port.preflight(request)
        return copy.deepcopy(self.baseline)

    def submit(self, request):
        self.final_gate()
        return self.port.submit(request)

    def __getattr__(self, name):
        return getattr(self.port, name)


class LiveActionSession:
    def __init__(self, output_dir, plan, camera, executor, port, *, clock=time):
        self.plan = validate_plan(plan)
        self.camera, self.executor, self.port, self.clock = camera, executor, port, clock
        c = plan['camera']
        self.session = dict(schemaVersion=LIVE_SESSION_SCHEMA,sessionId=c['sessionId'],viewId=c['viewId'],
                            streamId=c['sessionId']+'-stream',sourceClockId=c['sessionId']+'-pts',hostClockId=port.host_clock_id,
                            mode='live_observation',width=c['width'],height=c['height'],rowBytes=c['width']*4,
                            cameraViewFixed=True,videoMirrored=c['videoMirrored'],videoRotationAngle=c['videoRotationAngle'],
                            maxAttempts=1,maxFrames=c['maxFrames'],maxDurationSeconds=50)
        self.collector = VisualActionCollector(output_dir, self.session)
        self.collector.store.write_json('pilot-plan.json', self.plan)
        self.dispatched = None
        self.result_recorded = False
        self.frame_count = 0
        self.first_frame_at = None
        self.last_frame_at = None
        self.used = False
        port.final_gate = self._final_gate
        camera.on_fault = executor.request_stop

    def _record(self, kind, at=None, **fields):
        e = dict(type=kind, at=self.clock.monotonic() if at is None else at,
                 hostClockId=self.session['hostClockId'],actionId=self.plan['request']['commandId'],**fields)
        # end after a recorded failure legitimately returns false (run is halted).
        ok = self.collector.record(e)
        if not ok and kind != 'end': raise ValueError('collector_rejected_event')

    def _drain(self):
        for p in self.camera.drain():
            e = dict(type='frame',at=p['at'],hostClockId=self.session['hostClockId'],actionId=self.plan['request']['commandId'],
                     frameId=p['frameId'],viewId=self.session['viewId'],streamId=self.session['streamId'],
                     sourceClockId=self.session['sourceClockId'],pts=p['pts'],timestampSemantics='presentation_time',
                     pixelFormat='BGRA8',sha256=p['sha256'],features=p['features'])
            started = self.clock.monotonic()
            if not self.collector.record(e,self.camera.pixels(p)): raise ValueError('collector_rejected_frame')
            if self.clock.monotonic()-started > .25: raise ValueError('collector_storage_late')
            self.frame_count += 1
            if self.first_frame_at is None: self.first_frame_at=p['at']
            self.last_frame_at=p['at']

    def _final_gate(self):
        self.camera.check_fresh()
        if self.dispatched is None or not 0 <= self.clock.monotonic()-self.dispatched <= self.plan['maxDispatchLagSeconds']:
            raise ValueError('dispatch_intent_expired')

    def _guard(self, request):
        with self.camera.lock:
            self._drain()
            self.camera.check_fresh()
            remaining = self.plan['camera']['seconds']-(self.clock.monotonic()-self.first_frame_at)
            if remaining < request['durationMs']/1000+self.plan['afterSeconds']+1:
                raise ValueError('insufficient_post_observation_budget')
            self.dispatched = self.clock.monotonic()
            self._record('dispatch',self.dispatched)
            self._final_gate()

    def _result(self, result):
        with self.camera.lock:
            self._drain()
            # Intention persisted but submit guard failed: retain conservative unknown.
            # The exact executor not_sent receipt is preserved separately below.
            self.collector.store.write_json('executor-result.json',result)
            state = result['executionState']
            if self.dispatched is not None and state == 'not_sent': state='outcome_unknown'
            action = result.get('action') if state == 'known_terminal' else None
            self._record('result',executionState=state,record=action['record'] if action else None,
                         baselineOutputs=self.port.baseline if action else None,outputLog=action['outputLog'] if action else None)
            self.result_recorded = True

    def run(self, *, confirmed_request):
        if self.used: raise ValueError('live_session_single_use')
        self.used = True
        if canonical_request(confirmed_request) != self.plan['request']:
            self.executor.close()
            raise ValueError('exact_request_confirmation_required')
        interrupted = False
        try:
            self._record('begin',episodeId=self.session['sessionId'],startPoseId=self.plan['startPoseId'],request=self.plan['request'])
            if not self.executor.journal.readiness(self.plan['request']['commandId'])['ready']:
                raise ValueError('journal_not_ready_before_capture')
            self.camera.start()
            start = self.clock.monotonic()
            while True:
                with self.camera.lock:
                    self._drain()
                    if self.camera.fault or self.camera.done: raise ValueError('camera_not_ready')
                    if self.frame_count >= 3 and self.last_frame_at-self.first_frame_at >= self.plan['beforeSeconds']:
                        self.camera.check_fresh(); break
                if self.clock.monotonic()-start > 32: raise ValueError('camera_startup_timeout')
                self.clock.sleep(.01)
            cid=self.plan['request']['commandId']
            self.executor.propose(self.plan['request'])
            result=self.executor.confirm(cid,confirmed_request=confirmed_request,dispatch_guard=self._guard)
            self._result(result)
            result_at=self.clock.monotonic()
            while True:
                with self.camera.lock:
                    self._drain()
                    if self.camera.fault: raise ValueError(self.camera.fault)
                    if self.camera.done: break
                if self.clock.monotonic()-start > 47: raise ValueError('camera_process_timeout')
                self.clock.sleep(.01)
            if self.last_frame_at-result_at < self.plan['afterSeconds']:
                raise ValueError('post_observation_too_short')
            self._record('end',motionObservation='unknown',settlingObservation='unverified')
        except BaseException as exc:
            interrupted = isinstance(exc, KeyboardInterrupt)
            self.executor.request_stop()
            if not self.collector.halted and not self.collector.closed:
                with self.camera.lock:
                    # Preserve a valid queued prefix even on camera/transport faults.
                    try: self._drain()
                    except Exception: pass
                    if not self.collector.halted:
                        self._record('failure',reason='operator_interrupted' if interrupted else 'live_session_failed')
                        if not self.result_recorded:
                            result=self.executor.result(self.plan['request']['commandId'])
                            if self.dispatched is None:
                                result={**result,'executionState':'not_sent'}
                            self._result(result)
                        self._record('end',motionObservation='unknown',settlingObservation='unverified')
            if not isinstance(exc, Exception) and not interrupted: raise
        finally:
            self.executor.request_stop()
            try:
                self.camera.close()
            finally:
                self.executor.close()
        result=self.collector.close()
        return {**result,'operatorInterrupted':interrupted}
