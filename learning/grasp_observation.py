"""Explicit read-only gripper capture and analyst-reviewed grasp observations.

Construction, latest() and check() never contact a device. Only capture()/observe()
issue a single fresh-camera GET. No motor transport, perception model, context
adaptation, retry or automatic affirmative contact/retention judgment is present.
"""
from __future__ import annotations

import copy
import math
import re
import time
import urllib.error
import urllib.parse
import urllib.request

from learning.artifacts import _read
from learning.controller_latency import deadline as request_deadline
from learning.gripper_camera_dataset import POLICY, read_frame
from learning.manual_collection_dataset import digest, require
from learning.relative_action_learning import STATE, point_state


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError('camera_redirect_refused')


class GripperObservationSource:
    """Bounded original JPEG/metadata acquisition with no controller access.

    scene_context is an explicitly checked actual scene, or a no-argument
    provider of that scene. It is not obtained from a requested stage/model.
    session_id is the existing read_frame hostClockId/sessionId identity.
    """
    def __init__(self, url, session_id, expected_boot_id, scene_context, store, *,
                 timeout=6., max_captures=24, clock=time, opener=None):
        parsed = urllib.parse.urlsplit(url)
        require(parsed.scheme == 'http' and parsed.hostname and not parsed.username
                and not parsed.password and parsed.path in ('', '/') and not parsed.query
                and not parsed.fragment and not any(c.isspace() for c in url), 'plain_camera_origin_required')
        parsed.port
        require(isinstance(session_id, str) and re.fullmatch('[A-Za-z0-9][A-Za-z0-9_.-]{0,100}', session_id),
                'camera_session_identity_required')
        require(isinstance(expected_boot_id, str) and re.fullmatch('[0-9a-f]{32}', expected_boot_id),
                'camera_boot_binding_required')
        require(_number(timeout) and 0 < timeout <= 6
                and type(max_captures) is int and 1 <= max_captures <= 100, 'bounded_camera_capture_required')
        require(callable(scene_context) or isinstance(scene_context, dict), 'actual_scene_context_required')
        self.url, self.session_id, self.boot = url.rstrip('/'), session_id, expected_boot_id
        self.scene_context, self.store, self.clock = scene_context, store, clock
        self.timeout, self.max_captures = timeout, max_captures
        self.opener = opener or urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
        self.started = self.closed = self.failed = False
        self.count, self.previous_capture_us = 0, None
        self._bindings = {}

    def start(self, *, deadline):
        require(not self.started and not self.closed, 'camera_source_single_use')
        require(_number(deadline) and self.clock.monotonic() < deadline, 'observation_deadline')
        self.started = True

    def _scene(self):
        scene = self.scene_context() if callable(self.scene_context) else self.scene_context
        require(isinstance(scene, dict) and set(scene) == {'physicalView', 'object', 'poseRegime', 'landmarks'}
                and all(isinstance(v, str) and v for v in scene.values()), 'actual_scene_context_required')
        return copy.deepcopy(scene)

    def capture(self, *, deadline):
        require(self.started and not self.closed and not self.failed, 'camera_source_not_active')
        require(self.count < self.max_captures, 'camera_capture_budget')
        remaining = deadline-self.clock.monotonic()
        require(_number(remaining) and remaining > 0, 'observation_deadline')
        scene = self._scene()
        self.count += 1
        folder = f'capture-{self.count:04d}'
        name = folder + '/esp32-frame'
        metadata = dict(sessionId=self.session_id, hostClockId=self.session_id,
                        clockDomain='python_monotonic', requestStartedAt=self.clock.monotonic(),
                        method='GET', path='/capture', query={'fresh': '1'}, retry=False, headers={})
        try:
            self.store.write_json(folder+'/capture-intent.json', metadata)
            remaining = min(self.timeout, deadline-self.clock.monotonic())
            require(remaining > 0, 'observation_deadline')
            with request_deadline(remaining):
                try:
                    response = self.opener.open(urllib.request.Request(self.url+'/capture?fresh=1',
                                                                     method='GET'), timeout=remaining)
                except urllib.error.HTTPError as exc:
                    response = exc
                with response:
                    metadata['status'] = response.status
                    metadata['headers'] = {k: v for k, v in response.headers.items()
                                           if k.lower().startswith('x-motionbrain-')}
                    body = response.read(2_000_001)
                    metadata['receivedAt'] = self.clock.monotonic()
            # Preserve received bytes even if JPEG, headers or status are bad.
            image = self.store._write(name+'.jpg', body)
            metadata.update(sha256=image['sha256'], persistedAt=self.clock.monotonic())
            self.store.write_json(name+'.json', metadata)
            require(metadata['status'] == 200 and len(body) <= 2_000_000, 'camera_capture_response_invalid')
            frame, checked_metadata, boot = read_frame(self.store.root/folder, 'frame', self.session_id)
            require(boot == self.boot, 'camera_boot_binding_changed')
            require(self.previous_capture_us is None or frame['requestedUs'] > self.previous_capture_us,
                    'camera_frame_reused_or_regressed')
            require(self.clock.monotonic() < deadline, 'observation_deadline')
            self.previous_capture_us = frame['capturedUs']
            context = dict(scene, cameraIdentity=dict(camera='gripper', cameraBootId=boot,
                timestampSemantics='first_dma_buffer', observationPolicy=copy.deepcopy(POLICY)),
                imageShape=[frame['height'], frame['width']], stateFields=copy.deepcopy(STATE))
            record = dict(frame={k: frame[k] for k in ('frameId', 'pixelSha256', 'width', 'height')},
                context=context, hostClockId=self.session_id,
                sourceEarliest=checked_metadata['requestStartedAt'],
                sourceLatest=checked_metadata['receivedAt'], receivedAt=checked_metadata['receivedAt'],
                persistedAt=checked_metadata['persistedAt'])
            self._bindings[frame['frameId']] = dict(folder=folder, record=copy.deepcopy(record),
                metadataSha256=digest(_read(self.store.root, name+'.json')), imageSha256=image['sha256'])
            self.store.write_json(folder+'/source-binding.json', self._bindings[frame['frameId']])
            return record
        except BaseException as exc:
            self.failed = True
            self.store.write_json(folder+'/capture-failure.json', dict(errorType=type(exc).__name__,
                                                                      failedAt=self.clock.monotonic()))
            raise

    def verify(self, record):
        """Recheck raw bytes after review, outside the nonblocking RUN guard."""
        binding = self._bindings.get(record['frame']['frameId'])
        require(binding is not None and record == binding['record'], 'source_record_changed')
        folder = binding['folder']
        require(digest(_read(self.store.root, folder+'/esp32-frame.json')) == binding['metadataSha256']
                and digest(_read(self.store.root, folder+'/esp32-frame.jpg')) == binding['imageSha256'],
                'reviewed_source_bytes_changed')
        frame, _, boot = read_frame(self.store.root/folder, 'frame', self.session_id)
        require(boot == self.boot and all(frame[k] == record['frame'][k] for k in record['frame']),
                'reviewed_source_identity_changed')

    def check(self):
        require(self.started and not self.closed and not self.failed, 'camera_source_not_active')

    def close(self):
        self.closed = True
        return dict(status='failed' if self.failed else 'completed', captures=self.count,
                    controllerWrites=0, automaticRetry=False)


class ReviewedGraspObserver:
    """Real source plus explicit analyst callbacks; no generated point/held labels.

    reviewer(record, *, deadline, tick) returns annotation/stable and optionally
    closeEntryEvidence. retention_reviewer(observations, *, deadline, tick)
    returns status, and for held returns method and explicit contactVerified/
    retentionVerified/supportDistinguished judgments. Callbacks must either
    return immediately or call tick() regularly while waiting and honor the
    absolute deadline. A blocking callback that ignores this contract must not
    be used for a live trial. Declarations are saved with original observations.
    """
    def __init__(self, source, reviewer, *, retention_reviewer=None, retention_seconds=1.,
                 max_age_seconds=2., clock=time):
        require(callable(reviewer) and (retention_reviewer is None or callable(retention_reviewer)),
                'explicit_observation_reviewer_required')
        require(_number(retention_seconds) and 1 <= retention_seconds <= 10
                and _number(max_age_seconds) and .1 <= max_age_seconds <= 5, 'bounded_observer_timing_required')
        self.source, self.reviewer, self.retention_reviewer, self.clock = source, reviewer, retention_reviewer, clock
        self.retention_seconds, self.max_age_seconds = retention_seconds, max_age_seconds
        self.cache = None
        self.waiting, self.started, self.closed = False, False, False
        self.fault = None
        self.review_count = 0

    def start(self, *, deadline):
        require(not self.started and not self.closed, 'reviewed_observer_single_use')
        self.started = True
        self.source.start(deadline=deadline)

    def _wait(self, until, deadline, tick):
        while self.clock.monotonic() < until:
            require(self.clock.monotonic() < deadline, 'observation_deadline')
            tick()
            now = self.clock.monotonic()
            require(now < deadline, 'observation_deadline')
            if now < until:
                self.clock.sleep(min(.1, until-now, deadline-now))

    def _review(self, *, deadline, tick):
        require(self.clock.monotonic() < deadline, 'observation_deadline')
        tick()
        record = self.source.capture(deadline=deadline)
        tick()
        require(self.clock.monotonic() < deadline, 'observation_deadline')
        reply = self.reviewer(copy.deepcopy(record), deadline=deadline, tick=tick)
        tick()
        require(self.clock.monotonic() < deadline, 'observation_deadline')
        require(isinstance(reply, dict) and set(reply) <= {'annotation', 'stable', 'closeEntryEvidence'}
                and reply.get('stable') is True and 'annotation' in reply, 'accepted_stable_point_review_required')
        point_state(reply['annotation'], record['frame'])
        require(self.clock.monotonic() < deadline, 'observation_deadline')
        self.source.verify(record)
        require(0 <= self.clock.monotonic()-record['sourceEarliest'] <= self.max_age_seconds,
                'reviewed_observation_stale')
        if 'closeEntryEvidence' in reply:
            proof = reply['closeEntryEvidence']
            require(isinstance(proof, dict) and set(proof) in (
                    {'frameId', 'pixelSha256', 'method', 'objectBetweenJawsVerified', 'depthVerified', 'clearanceVerified'},
                    {'frameId', 'pixelSha256', 'method', 'objectBetweenJawsVerified', 'depthVerified', 'clearanceVerified',
                     'evidenceFile', 'evidenceSha256'})
                    and proof['frameId'] == record['frame']['frameId']
                    and proof['pixelSha256'] == record['frame']['pixelSha256']
                    and proof['method'] == 'onsite_observation'
                    and all(proof[k] is True for k in
                            ('objectBetweenJawsVerified', 'depthVerified', 'clearanceVerified')),
                    'explicit_close_entry_review_required')
            declaration = {k: v for k, v in proof.items() if k not in ('evidenceFile', 'evidenceSha256')}
            ref = self.source.store.write_json(f'close-entry-review-{self.review_count+1:04d}.json', declaration)
            reply = dict(reply, closeEntryEvidence=dict(declaration, evidenceFile=ref['artifactRef'],
                                                       evidenceSha256=ref['sha256']))
        record.update(copy.deepcopy(reply))
        self.review_count += 1
        self.source.store.write_json(f'review-{self.review_count:04d}.json', record)
        self.cache = copy.deepcopy(record)
        return record

    def observe(self, stage, *, after_off, deadline, tick):
        require(self.started and not self.closed and self.fault is None, 'reviewed_observer_not_active')
        self.waiting = True
        try:
            if after_off is not None:
                require(_number(after_off), 'off_timestamp_required')
                self._wait(after_off+3., deadline, tick)
            tick()
            record = self._review(deadline=deadline, tick=tick)
            tick()
            return copy.deepcopy(record)
        except BaseException as exc:
            self.fault = type(exc).__name__
            raise
        finally:
            self.waiting = False

    def latest(self, stage):
        try:
            self.check()
            require(self.cache is not None, 'reviewed_observation_missing')
            require(0 <= self.clock.monotonic()-self.cache['sourceEarliest'] <= self.max_age_seconds,
                    'reviewed_observation_stale')
            return copy.deepcopy(self.cache)
        except BaseException as exc:
            self.fault = self.fault or type(exc).__name__
            raise

    def check(self):
        try:
            require(self.started and not self.closed and self.fault is None, 'reviewed_observer_not_active')
            self.source.check()
            if self.cache is not None and not self.waiting:
                require(0 <= self.clock.monotonic()-self.cache['sourceEarliest'] <= self.max_age_seconds,
                        'reviewed_observation_stale')
        except BaseException as exc:
            self.fault = self.fault or type(exc).__name__
            raise

    def verify_grasp(self, stage, *, after_off, deadline, tick):
        require(self.started and not self.closed and self.fault is None, 'reviewed_observer_not_active')
        if self.retention_reviewer is None:
            return dict(status='unknown')
        first = self.observe(stage, after_off=after_off, deadline=deadline, tick=tick)
        self.waiting = True
        try:
            self._wait(first['sourceLatest']+self.retention_seconds, deadline, tick)
            second = self._review(deadline=deadline, tick=tick)
            tick()
            require(self.clock.monotonic() < deadline, 'observation_deadline')
            proof = self.retention_reviewer(copy.deepcopy([first, second]), deadline=deadline, tick=tick)
            tick()
            require(self.clock.monotonic() < deadline, 'observation_deadline')
            # Reviewer callbacks run outside the guard and may take time. Bind
            # acceptance to both originals again, not merely cached point IDs.
            for observation in (first, second):
                self.source.verify({key: observation[key] for key in (
                    'frame', 'context', 'hostClockId', 'sourceEarliest', 'sourceLatest',
                    'receivedAt', 'persistedAt')})
            require(isinstance(proof, dict) and proof.get('status') in ('held', 'failed', 'unknown'),
                    'explicit_retention_outcome_required')
            if proof['status'] != 'held':
                require(set(proof) == {'status'}, 'retention_outcome_fields')
                return copy.deepcopy(proof)
            require(set(proof) == {'status', 'method', 'contactVerified', 'retentionVerified', 'supportDistinguished'}
                    and proof['method'] == 'analyst_retention_review'
                    and all(proof[k] is True for k in ('contactVerified', 'retentionVerified', 'supportDistinguished')),
                    'explicit_retention_review_required')
            require(self.clock.monotonic() < deadline, 'observation_deadline')
            self.latest(stage)
            declaration = dict(proof, observations=[first, second])
            ref = self.source.store.write_json('retention-review.json', declaration)
            return dict(declaration, evidenceFile=ref['artifactRef'], evidenceSha256=ref['sha256'])
        except BaseException as exc:
            self.fault = type(exc).__name__
            raise
        finally:
            self.waiting = False

    def close(self):
        self.closed = True
        result = self.source.close()
        if self.fault is not None:
            result = dict(result, status='failed', reviewError=self.fault)
        return result
