"""현재 FileReviewer 원본을 보여 주고 명시한 사람 판단만 제출한다. 장치 접근 없음."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import re
import time

import cv2
import numpy as np

from learning import m4_up_commissioning as core
from learning.artifacts import _read, _decode_json
from learning.batch_review_response import _publish
from learning.experiment_pipeline import encoded


class FileReviewConsole:
    def __init__(self, repository, scope_path, request_root, *, clock=time, read_only_review=False):
        core.require(type(read_only_review) is bool, 'review_console_mode_required')
        self.read_only_review = read_only_review
        self.repository = Path(repository).resolve()
        self.scope_path = Path(scope_path).resolve(strict=True)
        self.root = Path(request_root).resolve()
        private = self.repository/'.local'
        core.require(self.scope_path.is_relative_to(private) and self.root.is_relative_to(private),
                     'review_console_private_paths_required')
        self.scope_raw = _read(self.scope_path.parent, self.scope_path.name)
        self.scope = core.validate_scope(_decode_json(self.scope_raw))
        self.clock = clock
        self.current = None
        self.delivered = set()

    def _load(self):
        core.require(_read(self.scope_path.parent,self.scope_path.name) == self.scope_raw,
                     'review_console_scope_changed')
        paths = [self.root/(label+'-review-request.json') for label in ('after','before')]
        path = next((p for p in paths if p.exists() or p.is_symlink()), None)
        if path is None:
            return None
        raw = _read(self.root,path.name); identity = core.sha(raw)
        if self.current is not None and self.current['sha'] == identity:
            return self.current
        request = _decode_json(raw)
        core.require(isinstance(request,dict) and set(request) ==
            {'label','hostClockId','sources','previews','sourceEarliest','nonce'}, 'review_console_request_schema')
        label = request['label']
        core.require(label in ('before','after') and path.name == label+'-review-request.json' and
            request['hostClockId'] == self.scope['hostClockId'] and
            isinstance(request['nonce'],str) and re.fullmatch('[0-9a-f]{32}',request['nonce']) is not None,
            'review_console_request_identity')
        sources = request['sources']
        core.require(isinstance(sources,dict) and set(sources) == {'external','gripper'},
                     'review_console_sources_required')
        frames = dict(external=core._external_frame(self.root,sources['external'],self.scope),
                      gripper=core._gripper_frame(self.root,sources['gripper'],self.scope,label))
        now = self.clock.monotonic()
        core.require(all(f['sourceEarliest'] <= f['sourceLatest'] <= f['persistedAt'] <= now
                         for f in frames.values()), 'review_console_source_not_yet_available')
        earliest = min(f['sourceEarliest'] for f in frames.values())
        core.require(request['sourceEarliest'] == earliest, 'review_console_source_time_changed')
        previews = request['previews']
        core.require(previews == dict(external=label+'-external.png',gripper=label+'-gripper.jpg'),
                     'review_console_preview_paths')
        images = {camera:_read(self.root,name) for camera,name in previews.items()}
        pixels = cv2.imdecode(np.frombuffer(images['external'],np.uint8),cv2.IMREAD_UNCHANGED)
        frame = frames['external']
        core.require(pixels is not None and pixels.shape == (frame['height'],frame['width'],4) and
                     core.sha(pixels.tobytes()) == frame['pixelSha256'], 'review_console_preview_pixels_changed')
        source = sources['gripper']
        core.require(images['gripper'] == _read(self.root,source['root']+'/esp32-'+label+'.jpg'),
                     'review_console_gripper_preview_changed')
        # Seal every original used by the existing native/JPEG validators.
        ext = sources['external']; prefix = ext['root']+'/'
        ingress = _decode_json(_read(self.root,prefix+ext['ingressFile']))
        names = [path.name, *previews.values(), prefix+ext['ingressFile'],
                 prefix+'host-ingress/clock-before.json',prefix+ingress['metadataFile'],prefix+ingress['imageFile'],
                 source['root']+'/esp32-'+label+'.jpg',source['root']+'/esp32-'+label+'.json']
        hashes = {name:core.sha(_read(self.root,name)) for name in names}
        self.current = dict(sha=identity,request=request,images=images,hashes=hashes,
                            expiresAt=earliest+(9. if label=='before' else 10.))
        self.delivered = set()
        return self.current

    def status(self):
        current = self._load()
        if current is None:
            return dict(status='waiting',readOnlyReview=self.read_only_review)
        label = current['request']['label']
        consumed = (self.root/(label+'-review-decision.json')).exists()
        remaining = current['expiresAt']-self.clock.monotonic()
        return dict(readOnlyReview=self.read_only_review,status='submitted' if consumed else 'expired' if remaining<=0 else 'ready',
            requestSha256=current['sha'],label=label,remainingSeconds=max(0.,remaining),
            action={k:self.scope['pulse'][k] for k in ('joint','direction','percent','durationMs')},
            scene={k:self.scope['context'][k] for k in ('object','poseRegime')},
            previews={name:'/api/preview?camera='+name+'&request='+current['sha'] for name in ('external','gripper')})

    def preview(self, camera, request_sha):
        current = self._load()
        core.require(current is not None and request_sha == current['sha'] and camera in ('external','gripper'),
                     'review_console_preview_request_changed')
        self.delivered.add(camera)
        return current['images'][camera]

    def submit(self, payload):
        current = self._load()
        core.require(current is not None and isinstance(payload,dict) and set(payload) ==
            {'requestSha256','framesReviewed','stable','onsite','outcome'}, 'review_console_explicit_decision_required')
        core.require(payload['requestSha256'] == current['sha'] and payload['framesReviewed'] is True and
                     self.delivered == {'external','gripper'}, 'review_console_current_frames_not_reviewed')
        core.require(self.clock.monotonic() < current['expiresAt'], 'review_console_expired')
        core.require(payload['outcome'] in ('reviewed','unknown') and type(payload['stable']) is bool,
                     'review_console_decision_invalid')
        onsite = payload['onsite']
        core.require(isinstance(onsite,dict) and set(onsite) ==
            {'operatorPresent','clearanceVerified','jawContactVerified','tableSupportVerified'} and
            all(type(v) is bool for v in onsite.values()), 'review_console_onsite_invalid')
        core.require(not self.read_only_review or payload['outcome']=='unknown',
                     'review_console_read_only_unknown_required')
        if payload['outcome'] == 'unknown':
            core.require(payload['stable'] is False and not any(onsite.values()), 'review_console_unknown_not_affirmative')
        core.require(all(core.sha(_read(self.root,name)) == expected for name,expected in current['hashes'].items()),
                     'review_console_original_changed')
        core.require(_read(self.scope_path.parent,self.scope_path.name) == self.scope_raw,
                     'review_console_scope_changed')
        core.require(self.clock.monotonic() < current['expiresAt'], 'review_console_expired')
        response = dict(requestSha256=current['sha'],decision=dict(context=copy.deepcopy(self.scope['context']),
            stable=payload['stable'],onsite=copy.deepcopy(onsite)))
        label = current['request']['label']
        _publish(self.root/(label+'-review-submission.json'), encoded(dict(
            requestSha256=current['sha'],responseSha256=core.sha(encoded(response)),
            outcome=payload['outcome'],currentFramesReviewed=True,physicalJudgmentGenerated=False,
            decision=copy.deepcopy(response['decision']))))
        _publish(self.root/(label+'-review-decision.json'),encoded(response))
        return dict(status='submitted',outcome=payload['outcome'],physicalJudgmentGenerated=False)
