"""새 정지 anchor와 새 M4 before 원본의 좁은 동일 모습 검증.

과거 FileReviewer 판정·만료 요청을 재사용하지 않는다. root의 명시 영상 판독을
동일 부팅·동일 출력 cursor·엄격한 영상 정지 조건에만 결합한다. 접촉 힘·물리
안전·계측 정확도 보증이 아니며 사람 보조 판독 근거를 별도로 남긴다.
"""
from __future__ import annotations

import copy
import hashlib
import math
import time
from pathlib import Path

import cv2
import numpy as np

from learning.artifacts import _decode_json,_read
from learning.m4_up_commissioning import _external_frame,_gripper_frame
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest
from learning.task_visual_measurement import measure

SCHEMA='motionbrain.current_M4_stationary_anchor_review.v1'
ROIS={'external':[650,200,1500,870],'gripper':[65,65,255,165]}
LABELS={'operatorPresent','clearanceVerified','jawContactVerified','tableSupportVerified'}


def _sha(raw):return hashlib.sha256(raw).hexdigest()

def _image(raw):
    image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_UNCHANGED)
    require(image is not None and image.ndim==3 and image.shape[2] in (3,4),'M4_stationary_image_required')
    return image

def _gray(image):
    return cv2.cvtColor(image,cv2.COLOR_BGRA2GRAY if image.shape[2]==4 else cv2.COLOR_BGR2GRAY)


def same_picture(anchor,current,camera):
    """고정 ROI의 매우 좁은 정지 일치. 정확도·기구 안전 판정이 아니다."""
    a,b=_image(anchor),_image(current)
    require(a.shape==b.shape,'M4_stationary_image_shape_changed')
    x0,y0,x1,y1=ROIS[camera]
    require(x1<=a.shape[1] and y1<=a.shape[0],'M4_stationary_roi_outside_image')
    a,b=_gray(a)[y0:y1,x0:x1],_gray(b)[y0:y1,x0:x1]
    require(float(a.std())>=3. and float(b.std())>=3.,'M4_stationary_roi_texture')
    ncc=float(cv2.matchTemplate(b,a,cv2.TM_CCOEFF_NORMED)[0,0])
    mean=float(np.abs(a.astype(np.float32)-b.astype(np.float32)).mean())
    require(math.isfinite(ncc) and ncc>=.995 and mean<=3.,'M4_stationary_picture_changed')
    return dict(ncc=ncc,meanAbsoluteGrayDifference=mean,roi=list(ROIS[camera]))


class StationaryM4BeforeReviewer:
    """before에서만 새 anchor를 검증하고 after는 기존 reviewer로 전달한다.

    read_epoch()는 동일 시계의 현재 GET 검증이며 RUN 제한시간 밖에서만 호출된다.
    그 결과는 anchor epoch와 완전히 같아야 한다. 0 PWM 이력 추가도 허용하지 않는다.
    asset_root는 비공개 원본 루트, refs는 그 아래 상대 경로·SHA다.
    """
    def __init__(self,scope,asset_root,anchor_binding,store,read_epoch,after_reviewer,*,clock=time):
        require(callable(read_epoch) and callable(after_reviewer),'M4_stationary_callbacks_required')
        self.scope=copy.deepcopy(scope);self.scope_sha=plan_digest(scope)
        self.root=Path(asset_root).resolve(strict=True)
        self.anchor_binding=copy.deepcopy(anchor_binding);self.store=store
        self.read_epoch,self.after_reviewer,self.clock=read_epoch,after_reviewer,clock
        self.used=False;self.accepted=False;self._run_check_used=False
        self._accepted_frames=None;self._accepted_memory_sha=None;self._accepted_deadline=None
        self.anchor=_decode_json(self._bound(self.anchor_binding))
        self._validate_anchor()

    def _bound(self,ref):
        require(isinstance(ref,dict) and set(ref)=={'path','sha256'},'M4_stationary_source_binding')
        raw=_read(self.root,ref['path'])
        require(_sha(raw)==ref['sha256'],'M4_stationary_source_changed')
        return raw

    def _validate_anchor(self):
        a=self.anchor
        require(isinstance(a,dict) and set(a)=={'schemaVersion','context','epoch','labels','sources',
            'metrologyRegistration','reviewBasis','humanAssisted'} and a['schemaVersion']==SCHEMA
            and a['reviewBasis']=='root_visual_stationary_anchor_review' and a['humanAssisted'] is True,
            'M4_new_stationary_anchor_review_required')
        require(a['context']==self.scope['context'],'M4_stationary_context_changed')
        epoch=a['epoch']
        require(isinstance(epoch,dict) and set(epoch)=={'controllerBootId','cameraBootId','outputSequence','allFiveOff'}
            and epoch['controllerBootId']==self.scope['controllerBootId']
            and epoch['cameraBootId']==self.scope['context']['gripperCameraBootId']
            and type(epoch['outputSequence']) is int and epoch['outputSequence']>0
            and epoch['allFiveOff'] is True,'M4_stationary_anchor_epoch')
        require(isinstance(a['labels'],dict) and set(a['labels'])==LABELS
            and all(type(v) is bool for v in a['labels'].values()),'M4_stationary_explicit_root_labels')
        require(isinstance(a['sources'],dict) and set(a['sources'])=={'external','gripper'},'M4_stationary_anchor_sources')
        anchor_sessions=[]
        for camera,source in a['sources'].items():
            require(isinstance(source,dict) and set(source)=={'image','metadata'},'M4_stationary_anchor_source_fields')
            _image(self._bound(source['image']))
            meta=_decode_json(self._bound(source['metadata']))
            require(isinstance(meta,dict),'M4_stationary_metadata_required')
            session=meta.get('sessionId')
            require(isinstance(session,str) and session.startswith('M4-stationary-anchor-')
                    and session!=self.scope['hostClockId'],'M4_stationary_independent_anchor_session')
            anchor_sessions.append(session)
            image=self._bound(source['image'])
            if camera=='external':
                decoded=_image(image)
                require(decoded.shape[2]==4 and _sha(decoded.tobytes())==meta.get('pixelSha256'),
                        'M4_stationary_anchor_external_pixel_binding')
            else:
                require(meta.get('sha256')==_sha(image) and meta.get('method')=='GET'
                        and meta.get('path')=='/capture' and meta.get('query')=={'fresh':'1'}
                        and isinstance(meta.get('headers'),dict),'M4_stationary_anchor_gripper_binding')
                headers={k.lower():v for k,v in meta['headers'].items()}
                require(headers.get('x-motionbrain-camera-boot-id')==epoch['cameraBootId']
                        and headers.get('x-motionbrain-capture-mode')=='after_request'
                        and headers.get('x-motionbrain-timestamp-semantics')=='first_dma_buffer',
                        'M4_stationary_anchor_gripper_identity')
        require(len(set(anchor_sessions))==1,'M4_stationary_anchor_session_mismatch')
        registration=_decode_json(self._bound(a['metrologyRegistration']))
        measure(self._bound(a['sources']['external']['image']),self._bound(a['sources']['gripper']['image']),registration)

    def _epoch_matches(self):
        current=self.read_epoch()
        require(isinstance(current,dict) and set(current)==set(self.anchor['epoch'])
                and type(current.get('outputSequence')) is int and current.get('allFiveOff') is True
                and current==self.anchor['epoch'],'M4_stationary_current_epoch_changed')

    def __call__(self,request,*,deadline,tick):
        if request.get('label')!='before':
            require(request.get('label')=='after' and self.accepted,'M4_stationary_before_not_accepted')
            return self.after_reviewer(request,deadline=deadline,tick=tick)
        require(not self.used,'M4_stationary_before_consumed');self.used=True
        require(self.clock.monotonic()<deadline and plan_digest(self.scope)==self.scope_sha,
                'M4_stationary_before_deadline_or_scope')
        require(_decode_json(self._bound(self.anchor_binding))==self.anchor,'M4_stationary_anchor_changed')
        self._validate_anchor();tick()
        self._epoch_matches()
        require(request.get('hostClockId')==self.scope['hostClockId'] and request.get('label')=='before'
                and isinstance(request.get('sources'),dict) and set(request['sources'])=={'external','gripper'}
                and isinstance(request.get('previews'),dict) and set(request['previews'])=={'external','gripper'},
                'M4_stationary_fresh_request_required')
        current={camera:_read(self.store.root,request['previews'][camera]) for camera in ('external','gripper')}
        frames=dict(external=_external_frame(self.store.root,request['sources']['external'],self.scope),
            gripper=_gripper_frame(self.store.root,request['sources']['gripper'],self.scope,'before'))
        anchor_meta=_decode_json(self._bound(self.anchor['sources']['external']['metadata']))
        require(all(anchor_meta.get(key)==frames['external'].get(key) for key in
                    ('configurationSha256','videoMirrored','videoRotationAngle')),
                'M4_stationary_external_view_changed')
        require(_sha(_image(current['external']).tobytes())==frames['external']['pixelSha256']
                and _sha(current['gripper'])==request['sources']['gripper']['imageSha256'],
                'M4_stationary_preview_source_changed')
        age_limit=min(3.,self.scope['maxObservationAgeSeconds'])
        now=self.clock.monotonic()
        require(all(0<=now-f['sourceEarliest']<=age_limit
                    for f in frames.values()),'M4_stationary_fresh_frames_expired')
        anchor={camera:self._bound(self.anchor['sources'][camera]['image']) for camera in current}
        picture={camera:same_picture(anchor[camera],current[camera],camera) for camera in current}
        registration=_decode_json(self._bound(self.anchor['metrologyRegistration']))
        before=measure(anchor['external'],anchor['gripper'],registration)
        fresh=measure(current['external'],current['gripper'],registration)
        limits=dict(objectRisePx=1.,toolRisePx=1.,slipDyPx=.5)
        differences={key:abs(fresh['state'][key]-before['state'][key]) for key in limits}
        require(all(differences[key]<=limits[key] for key in limits),'M4_stationary_metrology_changed')
        # GET once more after file/metrology work; no output cursor drift is accepted.
        self._epoch_matches()
        require(self.clock.monotonic()<deadline and all(
            0<=self.clock.monotonic()-f['sourceEarliest']<=age_limit
            for f in frames.values()),'M4_stationary_verification_expired')
        decision=dict(context=copy.deepcopy(self.scope['context']),stable=True,onsite=copy.deepcopy(self.anchor['labels']))
        self.store.write_json('before-current-metrology.json',fresh)
        self.store.write_json('before-stationary-anchor-verification.json',dict(
            anchorBinding=self.anchor_binding,requestSha256=plan_digest(request),frames=frames,
            picture=picture,metrologyDifferences=differences,epoch=self.anchor['epoch'],
            humanAssisted=True,reviewBasis=self.anchor['reviewBasis'],candidateDecision=decision,
            meaning='same_stationary_pixels_not_force_safety_or_metrology_accuracy_guarantee'))
        require(self.clock.monotonic()<deadline and plan_digest(self.scope)==self.scope_sha and all(
            0<=self.clock.monotonic()-f['sourceEarliest']<=age_limit for f in frames.values()),
            'M4_stationary_evidence_write_expired')
        self._accepted_frames=copy.deepcopy(frames)
        self._accepted_deadline=deadline
        self._accepted_memory_sha=plan_digest(dict(scope=self.scope,anchor=self.anchor,
            binding=self.anchor_binding,frames=self._accepted_frames))
        self.accepted=True
        return decision

    def check_before_run(self):
        """RUN 마지막 gate: 메모리만 읽으며 성공·거절 모두 단 한 번 소비한다."""
        require(self.accepted and not self._run_check_used,'M4_stationary_RUN_gate_consumed')
        self._run_check_used=True
        require(plan_digest(dict(scope=self.scope,anchor=self.anchor,binding=self.anchor_binding,
                    frames=self._accepted_frames))==self._accepted_memory_sha
                and plan_digest(self.scope)==self.scope_sha,'M4_stationary_RUN_memory_changed')
        now=self.clock.monotonic();limit=min(3.,self.scope['maxObservationAgeSeconds'])
        require(now<self._accepted_deadline and all(0<=now-frame['sourceEarliest']<=limit
                    for frame in self._accepted_frames.values()),'M4_stationary_RUN_freshness_expired')
