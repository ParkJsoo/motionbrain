"""고정 배경 affine의 기준 좌표 관측 개발 모드. 실물 실행 자격을 만들지 않는다."""
from __future__ import annotations
import copy
import hashlib
from pathlib import Path
import cv2
import numpy as np
from learning import batch_template_observation as template
from learning.batch_observation import _prepare_frame, digest, require

SCHEMA='motionbrain.batch_background_registered_observer.v1'
# This mask belongs to one declared empty-load M1 view, not arbitrary robot poses.
REGIONS=[[0,0,430,900],[450,20,1350,175]]
PARAMETERS=dict(resizeFactor=.5,maxCorners=700,qualityLevel=.01,minDistance=8,blockSize=7,
    windowSize=21,pyramidLevels=1,iterations=40,epsilon=.001,maskInsetPx=52,
    maximumRoundtripPx=1.,ransacThresholdPx=1.5,ransacIterations=3000,ransacConfidence=.999,
    refineIterations=20,minimumFeatures=30,minimumInliersPerRegion=4,minimumInlierFraction=.90,
    minimumHullAreaPx2=100000.,maximumResidualP95Px=.75,scaleMinimum=.985,scaleMaximum=1.015,
    maximumRotationDegrees=.15,maximumCenterShiftPx=2.,maximumCornerShiftPx=20.,maximumShear=.003)


def implementation_sha256():
    from learning import visual_registration
    return hashlib.sha256(b'\0'.join([Path(__file__).read_bytes(),Path(template.__file__).read_bytes(),
        Path(visual_registration.__file__).read_bytes(),cv2.__version__.encode()])).hexdigest()


def make_spec(legacy):
    """명시적 새 개발 계약. 현재 영상이나 정답을 입력받지 않는다."""
    base=template.validate_spec(legacy)
    require(base['load']=='empty','background_empty_load_only')
    base['joints']={'gripper':base['joints']['gripper']}
    for name,bounds in base['fixedViewAnchors'].items():
        point=base['landmarks'][name]['point'];dx=(bounds[1]-bounds[0])/2;dy=(bounds[3]-bounds[2])/2
        base['fixedViewAnchors'][name]=[point[0]-dx,point[0]+dx,point[1]-dy,point[1]+dy]
    base['schemaVersion']=SCHEMA
    base['backgroundNormalization']=dict(regions=copy.deepcopy(REGIONS),parameters=copy.deepcopy(PARAMETERS),
        implementationSha256=implementation_sha256(),coordinateSystem='immutable_reference_image_pixels',
        purpose='M1_noncontact_recording_development_only',physicalPoseEstablished=False)
    return validate_spec(base)


def _legacy(spec):
    base=copy.deepcopy(spec);base.pop('backgroundNormalization');base['schemaVersion']=template.SCHEMA
    return base


def validate_spec(spec):
    require(isinstance(spec,dict) and spec.get('schemaVersion')==SCHEMA,'background_observer_schema')
    cfg=spec.get('backgroundNormalization')
    require(cfg==dict(regions=REGIONS,parameters=PARAMETERS,implementationSha256=implementation_sha256(),
        coordinateSystem='immutable_reference_image_pixels',purpose='M1_noncontact_recording_development_only',
        physicalPoseEstablished=False),'background_contract_changed')
    base=template.validate_spec(_legacy(spec))
    require(base['load']=='empty' and set(base['joints'])=={'gripper'},'background_M1_empty_only')
    for name,bounds in base['fixedViewAnchors'].items():
        point=base['landmarks'][name]['point']
        require(abs((bounds[0]+bounds[1])/2-point[0])<1e-9 and
            abs((bounds[2]+bounds[3])/2-point[1])<1e-9,'background_reference_anchor_not_fixed')
    return copy.deepcopy(spec)


def _inside(points,region,inset):
    x0,y0,x1,y1=region
    return ((points[:,0]>=x0+inset)&(points[:,0]<x1-inset)&
            (points[:,1]>=y0+inset)&(points[:,1]<y1-inset))


def validate_transform(matrix,shape):
    a=np.asarray(matrix,dtype=float);require(a.shape==(2,3) and np.isfinite(a).all(),'background_affine_invalid')
    h,w=shape;linear=a[:,:2];scales=np.linalg.norm(linear,axis=0)
    require(np.linalg.det(linear)>0,'background_reflection_rejected')
    rotation=float(np.degrees(np.arctan2(a[1,0],a[0,0])))
    shear=float(abs(np.dot(linear[:,0],linear[:,1])/(scales[0]*scales[1])))
    center=np.array([w/2,h/2]);shift=float(np.linalg.norm(a@np.r_[center,1]-center))
    corners=np.array([[0,0],[w,0],[0,h],[w,h]],float)
    corner_shift=float(np.max(np.linalg.norm(np.c_[corners,np.ones(4)]@a.T-corners,axis=1)))
    p=PARAMETERS
    require(np.all((scales>=p['scaleMinimum'])&(scales<=p['scaleMaximum'])) and
        abs(rotation)<=p['maximumRotationDegrees'] and shear<=p['maximumShear'] and
        shift<=p['maximumCenterShiftPx'] and corner_shift<=p['maximumCornerShiftPx'],
        'background_transform_outside_declared_domain')
    return dict(scales=scales.tolist(),rotationDegrees=rotation,shear=shear,centerShiftPx=shift,maximumCornerShiftPx=corner_shift)


def fit_background(reference,current):
    require(reference.shape==current.shape and reference.shape[:2]==(1080,1920),'background_image_shape')
    p=PARAMETERS
    def gray(image):return cv2.resize(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),(960,540),interpolation=cv2.INTER_AREA)
    before,after=gray(reference),gray(current);mask=np.zeros(before.shape,np.uint8)
    for x0,y0,x1,y1 in REGIONS:
        m=p['maskInsetPx'];mask[(y0+m)//2:(y1-m)//2,(x0+m)//2:(x1-m)//2]=255
    points=cv2.goodFeaturesToTrack(before,maxCorners=p['maxCorners'],qualityLevel=p['qualityLevel'],
        minDistance=p['minDistance'],mask=mask,blockSize=p['blockSize'])
    require(points is not None and len(points)>=p['minimumFeatures'],'background_features_insufficient')
    kwargs=dict(winSize=(p['windowSize'],)*2,maxLevel=p['pyramidLevels'],
        criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,p['iterations'],p['epsilon']))
    moved,ok,_=cv2.calcOpticalFlowPyrLK(before,after,points,None,**kwargs)
    require(moved is not None,'background_flow_failed')
    back,bok,_=cv2.calcOpticalFlowPyrLK(after,before,moved,None,**kwargs)
    require(back is not None,'background_backward_flow_failed')
    x=points.reshape(-1,2)*2;y=moved.reshape(-1,2)*2
    roundtrip=np.linalg.norm(points.reshape(-1,2)-back.reshape(-1,2),axis=1)*2
    valid=(ok.ravel()!=0)&(bok.ravel()!=0)&np.isfinite(y).all(axis=1)&(roundtrip<=p['maximumRoundtripPx'])
    same_region=np.zeros(len(x),bool)
    for region in REGIONS:same_region|=_inside(x,region,p['maskInsetPx'])&_inside(y,region,p['maskInsetPx'])
    valid&=same_region;x=x[valid];y=y[valid]
    require(len(x)>=p['minimumFeatures'],'background_roundtrip_features_insufficient')
    a,inliers=cv2.estimateAffine2D(x,y,method=cv2.RANSAC,ransacReprojThreshold=p['ransacThresholdPx'],
        maxIters=p['ransacIterations'],confidence=p['ransacConfidence'],refineIters=p['refineIterations'])
    require(a is not None and inliers is not None,'background_affine_fit_failed')
    keep=inliers.ravel().astype(bool);require(keep.mean()>=p['minimumInlierFraction'],'background_inlier_fraction')
    require(all(int((_inside(x,region,p['maskInsetPx'])&keep).sum())>=p['minimumInliersPerRegion'] for region in REGIONS),
        'background_inlier_region_coverage')
    hull=float(cv2.contourArea(cv2.convexHull(x[keep])))
    require(hull>=p['minimumHullAreaPx2'],'background_inlier_spatial_coverage')
    errors=np.linalg.norm(np.c_[x,np.ones(len(x))]@a.T-y,axis=1);p95=float(np.quantile(errors[keep],.95))
    require(p95<=p['maximumResidualP95Px'],'background_residual_exceeded')
    metrics=validate_transform(a,current.shape[:2])
    return dict(referenceToCurrent=a.tolist(),currentToReference=cv2.invertAffineTransform(a).tolist(),
        referenceFeatures=x.tolist(),currentFeatures=y.tolist(),inlierMask=keep.tolist(),
        inlierFraction=float(keep.mean()),inlierHullAreaPx2=hull,residualP95Px=p95,**metrics,
        accuracyCalibrated=False,residualSemantics='background_fit_residual_not_foreground_accuracy_or_pose_proof')


def observe_frame(image_bytes,frame_metadata,spec,context,now,clock_id):
    result=dict(status='rejected',reason=None,states={},nodeMatches=[],landmarks={},frameBinding=copy.deepcopy(frame_metadata),
        controlEligible=False,trainingEligible=False,motionTrackingVerified=False,physicalValidationEstablished=False,
        readonlyStationaryEligible=False,physicalPoseEstablished=False)
    try:
        spec=validate_spec(spec);base=_legacy(spec)
        reference=template._reference(base)[:,:,:3]
        current=_prepare_frame(image_bytes,frame_metadata,base,context,now,clock_id)
        fit=fit_background(reference,current);result['backgroundFit']=fit
        inverse=np.asarray(fit['currentToReference']);a=np.asarray(fit['referenceToCurrent'])
        normalized=cv2.warpAffine(current,inverse,(1920,1080),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        ok,png=cv2.imencode('.png',normalized);require(ok,'background_normalized_encoding_failed')
        raw=png.tobytes();metadata=copy.deepcopy(frame_metadata)
        metadata.update(imageSha256=hashlib.sha256(raw).hexdigest(),pixelSha256=hashlib.sha256(normalized.tobytes()).hexdigest())
        raw_observed=template.observe_frame(image_bytes,frame_metadata,base,context,now,clock_id)
        observed=template.observe_frame(raw,metadata,base,context,now,clock_id)
        for landmark in observed['landmarks'].values():
            landmark['uncertaintyMeaning']='legacy_declared_detector_bound_excludes_uncalibrated_affine_and_resampling_error'
        result.update(observed)
        result['stateUncertaintyMeaning']='legacy_detector_propagation_excludes_uncalibrated_affine_and_resampling_error_total_unknown'
        result.update(frameBinding=copy.deepcopy(frame_metadata),specSha256=digest(spec),backgroundFit=fit,
            normalizedFrameBinding=metadata,normalizedFrameBindingSemantics='derived_pixels_original_frame_id_not_another_capture',coordinateSystem='immutable_reference_image_pixels',
            referenceImageSha256=spec['referenceImageSha256'],backgroundNormalizationImplementationSha256=implementation_sha256(),
            rawCurrentLandmarks=raw_observed['landmarks'],rawCurrentObservationStatus=raw_observed['status'],
            rawCurrentObservationReason=raw_observed['reason'],
            mappedCurrentLandmarks={name:dict(point=(a@np.r_[v['point'],1]).tolist(),
                semantics='inverse_mapped_detection_in_original_frame_not_independent_raw_detector') for name,v in observed['landmarks'].items()},
            transformUncertaintyCalibrated=False,physicalPoseEstablished=False,
            semantics='bounded_background_normalized_2d_development_not_physical_pose_depth_contact_or_control')
    except (ValueError,TypeError,KeyError,OverflowError,cv2.error) as exc:
        result.update(status='rejected',reason=str(exc),states={},nodeMatches=[],readonlyStationaryEligible=False)
    return result
