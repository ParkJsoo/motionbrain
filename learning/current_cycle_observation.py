"""현재 출력 epoch에 묶인 두 영상 판독. 명령/파지 성공을 픽셀값으로 대체하지 않는다.

판독은 정지 원본 pair에 결합되며, 별도로 확보한 새 pair를 재검증한다.
새 출력/부팅/구도 또는 정지 모습이 달라지면 기존 판독을 재사용하지 않는다.
"""
from __future__ import annotations
import copy
import math
import time
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest



def same_stationary_view(left,right,tolerances):
    """출력 epoch가 같은 정지 판독만 전이한다. 모델 입력 범위는 늘리지 않는다."""
    require(left['controllerBootId']==right['controllerBootId']
            and left['cameraBootId']==right['cameraBootId']
            and left['outputSequence']==right['outputSequence']
            and left['context']==right['context'],'cycle_review_epoch_changed')
    require(isinstance(tolerances,dict) and bool(tolerances) and all(
        type(v) in (int,float) and math.isfinite(v) and 0 <= v <= 1.5
        for v in tolerances.values()), 'cycle_review_stationary_tolerances')
    a,b=left['reviewFeatures'],right['reviewFeatures']
    require(set(a)==set(b)==set(tolerances),'cycle_review_features_changed')
    for name,limit in tolerances.items():
        x,y=a[name],b[name]
        if isinstance(x,list):
            require(bool(x) and isinstance(y,list) and len(x)==len(y)
                    and all(type(v) in (int,float) and math.isfinite(v) for v in x+y)
                    and all(abs(i-j)<=limit for i,j in zip(x,y)), 'cycle_review_stationary_changed')
        else:require(type(x) in (int,float) and type(y) in (int,float)
                     and math.isfinite(x) and math.isfinite(y) and abs(x-y)<=limit,
                     'cycle_review_stationary_changed')
    return True


ANCHOR_SCHEMA='motionbrain.stationary_dual_view_review_anchor.v1'
FEATURE_SHAPES=dict(externalObjectBox=4,externalToolCenter=2,gripperObjectBox=4,gripperReferencePoints=4,M4AngleDeg=0)
ONSITE=('operatorPresent','clearanceVerified','jawContactVerified','tableSupportVerified')


def verify_zero_output_bridge(anchor,current,proof):
    """판독 이후 연속 driver cursor가 전부 PWM0인 경우만 같은 물리 epoch다.

    원본 HTTP 응답의 진실성은 호출자의 audit_cursor가 별도로 입증해야 한다.
    여기서는 누락·중복·순서·비영출력·부팅·현재OFF의 논리 조건을 검사한다.
    """
    require(isinstance(proof,dict) and proof.get('controllerBootId')==anchor['controllerBootId']==current['controllerBootId']
            and proof.get('beforeSequence')==anchor['outputSequence']
            and proof.get('afterSequence')==current['outputSequence']
            and type(proof.get('beforeSequence')) is int and proof['beforeSequence']>=0
            and type(proof.get('afterSequence')) is int and proof['afterSequence']>=proof['beforeSequence']
            and proof.get('dropped')==0 and type(proof.get('dropped')) is int
            and proof.get('allFiveOff') is True
            and proof.get('state') in ('IDLE','ARMED')
            and type(proof.get('queueCount')) is int
            and proof['queueCount']==(1 if proof['state']=='ARMED' else 0),
            'stationary_review_current_OFF_epoch')
    entries=proof.get('entries')
    require(isinstance(entries,list) and len(entries)==proof['afterSequence']-proof['beforeSequence'],
            'stationary_review_cursor_gap')
    for expected,entry in enumerate(entries,proof['beforeSequence']+1):
        require(isinstance(entry,dict) and type(entry.get('sequence')) is int
                and entry['sequence']==expected and type(entry.get('motor')) is int
                and 1<=entry['motor']<=5 and type(entry.get('pwm')) is int and entry['pwm']==0,
                'stationary_review_new_motion_or_cursor_changed')
    return True


class StationaryReviewAnchor:
    """새 정지 두 영상 판독을 같은 현재 출력 epoch의 새 원본과 결합한다.

    만료된 actor 판독을 소급 통과시키지 않는다. 새 scope는 별도 anchor를
    만들어야 하며, anchorPair/audit_cursor 모두 실제 원본 재감사를 요구한다.
    review()는 단 한 번의 관측/준비 단계용이며 거절도 소비된다.
    실제 actor의 내구성 있는 scope 소비·원장 차단을 대체하지 않는다.
    RUN 직전 callback 안에서 호출하지 않는다.
    """
    def __init__(self,anchor,*,audit_pair,audit_cursor,clock=time):
        require(callable(audit_pair) and callable(audit_cursor),'stationary_review_source_auditors')
        require(isinstance(anchor,dict) and set(anchor)=={'schemaVersion','anchorId','pair','review','tolerances','maxFrameAgeSeconds'}
                and anchor['schemaVersion']==ANCHOR_SCHEMA
                and isinstance(anchor['anchorId'],str) and anchor['anchorId']
                and type(anchor['maxFrameAgeSeconds']) in (int,float)
                and 0<anchor['maxFrameAgeSeconds']<=3.,'stationary_review_anchor_contract')
        review=anchor['review'];pair=anchor['pair']
        require(isinstance(review,dict) and set(review)=={'context','stable','onsite'}
                and review['context']==pair['context'] and review['stable'] is True
                and isinstance(review['onsite'],dict) and set(review['onsite'])==set(ONSITE)
                and all(type(v) is bool for v in review['onsite'].values())
                and review['onsite']['operatorPresent'] and review['onsite']['clearanceVerified'],
                'stationary_review_explicit_visual_labels')
        self.anchor=copy.deepcopy(anchor);self.digest=plan_digest(self.anchor);self.used=False
        self.audit_pair,self.audit_cursor,self.clock=audit_pair,audit_cursor,clock
        self._pair_identity(pair)
        require(set(anchor['tolerances'])==set(FEATURE_SHAPES)
                and anchor['tolerances']['M4AngleDeg']<=.1,'stationary_review_dual_view_tolerances')
        require(self.audit_pair(copy.deepcopy(pair)) is True,'stationary_review_anchor_source_audit')
        # Validate the declared tolerance policy without transferring any labels yet.
        same_stationary_view(pair,pair,anchor['tolerances'])

    @staticmethod
    def _pair_identity(pair):
        require(isinstance(pair,dict) and isinstance(pair.get('context'),dict)
                and type(pair.get('outputSequence')) is int and pair['outputSequence']>=0,
                'stationary_review_pair_identity')
        for key in ('controllerBootId','cameraBootId'):
            value=pair.get(key)
            require(isinstance(value,str) and len(value)==32 and all(c in '0123456789abcdef' for c in value),
                    'stationary_review_pair_boot')
        features=pair.get('reviewFeatures')
        require(isinstance(features,dict) and set(features)==set(FEATURE_SHAPES),
                'stationary_review_dual_view_features')
        for name,length in FEATURE_SHAPES.items():
            values=features[name] if length else [features[name]]
            require(isinstance(values,list) and len(values)==(length or 1)
                    and all(type(v) in (int,float) and math.isfinite(v) for v in values),
                    'stationary_review_feature_shape')
        frames=pair.get('frames')
        require(isinstance(frames,list) and len(frames)==2
                and {f.get('camera') for f in frames}=={'external','gripper'},'stationary_review_two_cameras')
        ids=[]
        for frame in frames:
            times=[frame.get(k) for k in ('sourceEarliest','sourceLatest','persistedAt')]
            require(all(type(t) in (int,float) and math.isfinite(t) for t in times)
                    and times==sorted(times) and isinstance(frame.get('frameId'),str) and frame['frameId']
                    and isinstance(frame.get('pixelSha256'),str) and len(frame['pixelSha256'])==64
                    and all(c in '0123456789abcdef' for c in frame['pixelSha256']),
                    'stationary_review_frame_binding')
            ids.append(frame['frameId'])
        require(len(set(ids))==2,'stationary_review_duplicate_frame')

    def review(self,current,proof,*,deadline,tick):
        require(not self.used,'stationary_review_one_shot_consumed')
        self.used=True  # 원본/형상/기한 거절도 같은 anchor의 재검사로 구제하지 않는다.
        require(callable(tick) and self.clock.monotonic()<deadline,'stationary_review_deadline')
        require(plan_digest(self.anchor)==self.digest,'stationary_review_anchor_changed')
        self._pair_identity(current)
        require(current['cameraBootId']==self.anchor['pair']['cameraBootId']
                and current['context']==self.anchor['pair']['context'],'stationary_review_scene_changed')
        old_ids={f['frameId'] for f in self.anchor['pair']['frames']}
        require(not old_ids.intersection(f['frameId'] for f in current['frames']),
                'stationary_review_fresh_pair_required')
        require(self.audit_pair(copy.deepcopy(self.anchor['pair'])) is True
                and self.audit_pair(copy.deepcopy(current)) is True
                and self.audit_cursor(copy.deepcopy(proof)) is True,'stationary_review_raw_audit')
        verify_zero_output_bridge(self.anchor['pair'],current,proof)
        proof_at=proof.get('observedAt')
        require(type(proof_at) in (int,float) and math.isfinite(proof_at)
                and proof_at==current.get('outputProofObservedAt')
                and proof_at>=max(f['sourceLatest'] for f in current['frames'])
                and 0<=self.clock.monotonic()-proof_at<=self.anchor['maxFrameAgeSeconds'],
                'stationary_review_cursor_not_current_after_pair')
        # Bridge permits OFF writes, but preserves the original geometry/boot/camera contract.
        baseline=copy.deepcopy(self.anchor['pair']);baseline['outputSequence']=current['outputSequence']
        same_stationary_view(baseline,current,self.anchor['tolerances'])
        now=self.clock.monotonic()
        require(now<deadline and all(f['persistedAt']<=now and 0<=now-f['sourceEarliest']<=self.anchor['maxFrameAgeSeconds']
                and all(f[k]>=a[k] for k in ('sourceEarliest','sourceLatest','persistedAt'))
                for f,a in zip(sorted(current['frames'],key=lambda f:f['camera']),
                               sorted(self.anchor['pair']['frames'],key=lambda f:f['camera']))),
                'stationary_review_current_pair_stale')
        tick()
        require(self.clock.monotonic()<deadline
                and all(self.clock.monotonic()-f['sourceEarliest']<=self.anchor['maxFrameAgeSeconds'] for f in current['frames']),
                'stationary_review_current_pair_stale')
        decision=copy.deepcopy(self.anchor['review'])
        evidence=dict(anchorSha256=self.digest,anchorId=self.anchor['anchorId'],
            anchorPairSha256=plan_digest(self.anchor['pair']),currentPairSha256=plan_digest(current),
            zeroOutputBridgeSha256=plan_digest(proof),reviewedAt=self.clock.monotonic(),
            humanAssisted=True,originalActorOutcomeChanged=False,
            meaning='새 정지 시각판독과 새 scope의 현재 두 영상 정합; 깊이/힘 계측이나 과거 실패 복구 아님')
        return decision,evidence


def metrology_review_features(measured, m4_angle_deg):
    """같은 등록의 원본 두영상 계측에서 정지 비교용 형상을 만든다."""
    require(measured.get('quality',{}).get('valid') is True,'stationary_review_metrology_required')
    marks=measured['landmarks']
    result=dict(externalObjectBox=copy.deepcopy(marks['externalObject']['bbox']),
        externalToolCenter=copy.deepcopy(marks['externalTool']['centroid']),
        gripperObjectBox=copy.deepcopy(marks['gripperObject']['bbox']),
        gripperReferencePoints=[v for ref in marks['gripperReferences'] for v in ref['center']],
        M4AngleDeg=m4_angle_deg)
    # The caller still owns raw replay/audits; no physical labels are inferred here.
    return result
