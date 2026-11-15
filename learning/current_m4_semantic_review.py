"""배경 픽셀 대신 명시 물체·도구·팔 기하로 검사하는 별도 정지 관측 계약.

접촉·충돌·힘의 자동 판정이 아니다. 같은 출력 epoch에서 사람이 확인한 anchor의
의미 있는 영상 기하를 비교한다. 기존 reviewer의 원본·시계·OFF·집게 검사는 유지한다.
"""
from __future__ import annotations
import copy
import cv2
import numpy as np
from learning.artifacts import _decode_json
from learning.current_m4_review import StationaryM4BeforeReviewer, _image, _gray, same_picture
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest

SCHEMA='motionbrain.current_M4_semantic_stationary_geometry.v1'
REGIONS={'distalArm','elbow','proximalArm'}


def _flow_geometry(a,b,roi):
    x,y,X,Y=roi;a=a[y:Y,x:X];b=b[y:Y,x:X]
    points=cv2.goodFeaturesToTrack(a,150,.03,8)
    require(points is not None and len(points)>=20,'M4_semantic_arm_texture_missing')
    current,ok,_=cv2.calcOpticalFlowPyrLK(a,b,points,None)
    require(current is not None,'M4_semantic_arm_tracking_failed')
    back,reverse,_=cv2.calcOpticalFlowPyrLK(b,a,current,None)
    require(current is not None and back is not None,'M4_semantic_arm_tracking_failed')
    valid=(ok[:,0]>0)&(reverse[:,0]>0)&(np.linalg.norm((back-points)[:,0],axis=1)<=.5)
    movement=np.linalg.norm((current-points)[:,0],axis=1)
    require(float(valid.mean())>=.9,'M4_semantic_arm_occluded')
    # 양방향 추적 오차와 95% 이동 1px를 분리한다. 조명 NCC를 정지 기준으로 쓰지 않는다.
    require(float(np.quantile(movement[valid],.95))<=1.,'M4_semantic_arm_moved')
    tiles=[]
    for row in range(2):
        for col in range(2):
            p=points[:,0];selected=((p[:,0]>=col*a.shape[1]/2)&(p[:,0]<(col+1)*a.shape[1]/2)
                &(p[:,1]>=row*a.shape[0]/2)&(p[:,1]<(row+1)*a.shape[0]/2))
            require(int(selected.sum())>=3,'M4_semantic_arm_spatial_coverage')
            good=selected&valid
            require(good.sum()/selected.sum()>=.8,'M4_semantic_arm_local_occlusion')
            shift=float(np.median(movement[good]));tiles.append(shift)
            require(shift<=1.,'M4_semantic_arm_local_moved')
    return dict(featureCount=len(points),trackedFraction=float(valid.mean()),
                p95DisplacementPx=float(np.quantile(movement[valid],.95)),tileMedianDisplacementPx=tiles)


def semantic_picture(anchor,current,before,fresh,contract):
    require(set(contract)=={'schemaVersion','anchorSha256','externalArmRegions'}
            and contract['schemaVersion']==SCHEMA,'M4_semantic_contract_required')
    regions=contract['externalArmRegions']
    require(isinstance(regions,dict) and set(regions)==REGIONS,'M4_semantic_arm_regions_required')
    a,b=_image(anchor),_image(current)
    require(a.shape==b.shape,'M4_semantic_shape_changed')
    geometry={}
    for name in ('externalObject','externalTool'):
        old,new=before['landmarks'][name],fresh['landmarks'][name]
        shift=np.abs(np.asarray(new['centroid'])-old['centroid'])
        # 1px 기하 기준은 기존 metrology의 정지 오차와 동일. 색 경계의 1px씩
        # 양측 흔들림은 bbox 2px, 실제 선택 성분 면적은 10%까지 별도로 검사한다.
        require(bool(np.all(shift<=1.)),'M4_semantic_landmark_moved')
        shape=np.abs(np.asarray(new['bbox'][2:])-old['bbox'][2:])
        require(bool(np.all(shape<=2.)) and abs(new['area']/old['area']-1.)<=.1,
                'M4_semantic_landmark_shape_changed')
        geometry[name]=dict(centroidDeltaPx=shift.tolist(),bboxSizeDeltaPx=shape.tolist(),areaRatio=new['area']/old['area'])
    arm={};gray_a,gray_b=_gray(a),_gray(b)
    for name,roi in regions.items():
        require(isinstance(roi,list) and len(roi)==4 and all(type(v) is int for v in roi)
                and 0<=roi[0]<roi[2]<=a.shape[1] and 0<=roi[1]<roi[3]<=a.shape[0],
                'M4_semantic_roi_invalid')
        arm[name]=_flow_geometry(gray_a,gray_b,roi)
    return dict(schemaVersion=SCHEMA,contractSha256=plan_digest(contract),
                externalArmRegions=copy.deepcopy(regions),landmarkGeometry=geometry,armGeometry=arm,
                backgroundPixelEqualityRequired=False,physicalSafetyVerified=False)


class SemanticM4BeforeReviewer(StationaryM4BeforeReviewer):
    """명시 계약을 선택한 호출자만 사용한다. v1/v2 anchor 자체를 재표기하지 않는다."""
    def __init__(self,*args,geometry_binding,**kwargs):
        super().__init__(*args,**kwargs)
        self.geometry_binding=copy.deepcopy(geometry_binding)
        self.geometry=_decode_json(self._bound(self.geometry_binding))
        require(self.geometry.get('anchorSha256')==self.anchor_binding['sha256'],
                'M4_semantic_anchor_binding_changed')

    def _compare_pictures(self,anchor,current,registration,before,fresh):
        require(_decode_json(self._bound(self.geometry_binding))==self.geometry
                and self.geometry['anchorSha256']==self.anchor_binding['sha256'],
                'M4_semantic_contract_changed')
        self._geometry_sha=plan_digest(dict(binding=self.geometry_binding,contract=self.geometry))
        return dict(external=semantic_picture(anchor['external'],current['external'],before,fresh,self.geometry),
                    gripper=same_picture(anchor['gripper'],current['gripper'],'gripper'))

    def check_before_run(self):
        require(plan_digest(dict(binding=self.geometry_binding,contract=self.geometry))==self._geometry_sha,
                'M4_semantic_RUN_contract_changed')
        return super().check_before_run()
