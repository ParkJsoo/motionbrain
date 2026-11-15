"""현재 등록 영상의 상대 관측. 제한된 평행이동·등방 확대만 검사한다.

전체 영상에서 각 의미점의 패치를 독립 탐색한다. 임의 homography, 깊이,
접촉, 모델 전이 또는 실행 권한은 만들지 않는다. 등록은 현재 구도에서
의미점과 패치를 명시하는 사람 보조이며 관측 중 자동 갱신하지 않는다.
"""
from __future__ import annotations
import copy
import re
import cv2
import numpy as np
from learning import registered_scene_landmarks as scene
from learning.task_visual_measurement import _image

SCHEMA='motionbrain.scene_relative_observation.v1'
FIELDS=('objectMinusJawMidXCanonicalPx','objectMinusJawMidYCanonicalPx','jawHorizontalSpanCanonicalPx')
NAMES=scene.NAMES


def default_parameters():
    return dict(scales=[round(.8+.05*i,2) for i in range(9)],minimumNcc=.90,
                minimumPeakGap=.06,minimumQuadrantNcc=.75,maximumScaleDisagreement=.051)


def _context(value):
    scene._require(isinstance(value,dict) and set(value)=={'physicalView','object','poseRegime','cameraBootId'}
        and all(isinstance(v,str) and 0<len(v)<=160 for v in value.values())
        and re.fullmatch('[0-9a-f]{32}',value['cameraBootId']) is not None,'relative_source_context_required')


def _parameters(p):
    scene._require(isinstance(p,dict) and set(p)==set(default_parameters()),'relative_parameter_fields')
    scales=p['scales']
    scene._require(isinstance(scales,list) and 1<=len(scales)<=29 and 1. in scales
        and all(scene._number(v) and .7<=v<=1.4 for v in scales)
        and scales==sorted(set(scales)),'relative_scale_bank_invalid')
    for key,low,high in [('minimumNcc',.85,1.),('minimumPeakGap',.04,.5),
                         ('minimumQuadrantNcc',.65,1.),('maximumScaleDisagreement',0.,.1)]:
        scene._require(scene._number(p[key]) and low<=p[key]<=high,'relative_quality_limits')


def _decode(raw):
    image=_image(raw)
    scene._require(image.shape[0]<=480 and image.shape[1]<=640,'relative_image_size_limit')
    return image


def _quadrants(image):
    h,w=image.shape[:2]
    return [image[y:y+h//2,x:x+w//2] for y in (0,h-h//2) for x in (0,w-w//2)]


def _match(image,reference,roi,point,p):
    x0,y0,x1,y1=roi;patch=reference[y0:y1,x0:x1]
    matches=[]
    for scale in p['scales']:
        w,h=round(patch.shape[1]*scale),round(patch.shape[0]*scale)
        if w+2>=image.shape[1] or h+2>=image.shape[0]:continue
        template=cv2.resize(patch,(w,h),interpolation=cv2.INTER_LINEAR)
        scores=cv2.matchTemplate(image,template,cv2.TM_CCOEFF_NORMED)
        scene._require(np.isfinite(scores).all(),'relative_nonfinite_match')
        _,ncc,_,(x,y)=cv2.minMaxLoc(scores)
        matches.append(dict(score=float(ncc),scale=float(scale),x=x,y=y,w=w,h=h,scores=scores,template=template))
    scene._require(matches,'relative_scale_bank_outside_image')
    best=max(matches,key=lambda row:row['score']);x,y,w,h=(best[k] for k in ('x','y','w','h'))
    scene._require(best['score']>=p['minimumNcc'],'relative_patch_missing_or_deformed')
    scene._require(x>0 and y>0 and x+w<image.shape[1] and y+h<image.shape[0],'relative_patch_clipped')
    # Competing scale hypotheses at the same spatial peak are one location.
    # A distant duplicate anywhere in the image must still reject the match.
    cx,cy=x+w/2,y+h/2;competitor=-1.
    radius=max(3.,min(w,h)/4.)
    for row in matches:
        scores=row['scores'].copy();yy,xx=np.indices(scores.shape)
        same=(xx+row['w']/2-cx)**2+(yy+row['h']/2-cy)**2<=radius**2
        scores[same]=-1.;competitor=max(competitor,float(scores.max()))
    gap=best['score']-competitor
    if gap<p['minimumPeakGap']:
        error=ValueError('relative_patch_ambiguous')
        error.diagnostics=dict(ncc=best['score'],peakGap=gap,scale=best['scale'])
        raise error
    observed=image[y:y+h,x:x+w];quadrant_scores=[]
    for template,actual in zip(_quadrants(best['template']),_quadrants(observed)):
        # A blank quadrant provides no evidence against local occlusion.
        scene._require(float(cv2.cvtColor(template,cv2.COLOR_BGR2GRAY).std())>=3.,'relative_patch_texture_insufficient')
        score=float(cv2.matchTemplate(actual,template,cv2.TM_CCOEFF_NORMED)[0,0])
        scene._require(np.isfinite(score) and score>=p['minimumQuadrantNcc'],
                       'relative_patch_occluded_or_deformed')
        quadrant_scores.append(score)
    # Scale the analyst's point inside the detected patch; never infer an
    # occluded tip from a separately moving shaft or an unconstrained transform.
    actual_scale=np.array([w/patch.shape[1],h/patch.shape[0]])
    scene._require(abs(actual_scale[0]-actual_scale[1])<=.04,'relative_scale_rounding_anisotropic')
    xy=(np.array([x,y])+(np.asarray(point)-[x0,y0])*actual_scale).tolist()
    return dict(point=xy,scale=best['scale'],ncc=best['score'],peakGap=gap,
                quadrantNcc=quadrant_scores,bbox=[x,y,w,h])


def _geometry(points,patch_rois,image):
    scene._require(isinstance(points,dict) and isinstance(patch_rois,dict)
        and set(points)==set(patch_rois)==set(NAMES),'relative_three_landmarks_required')
    for name in NAMES:
        x0,y0,x1,y1=scene._rectangle(patch_rois[name],image.shape[:2])
        point=points[name]
        scene._require(isinstance(point,list) and len(point)==2 and all(scene._number(v) for v in point)
            and x0<=point[0]<x1 and y0<=point[1]<y1 and min(x1-x0,y1-y0)>=12,
            'relative_point_patch_invalid')
    for i,name in enumerate(NAMES):
        a,b,c,d=patch_rois[name]
        for other in NAMES[i+1:]:
            x,y,z,w=patch_rois[other]
            scene._require(min(c,z)<=max(a,x) or min(d,w)<=max(b,y),
                           'relative_landmark_patches_overlap')


def canonical_frame_sha256(registration):
    """원본 등록 좌표의 정체성. 카메라 부팅·현재 원본 증거와는 별도다."""
    return scene._digest({k:registration[k] for k in
        ('referenceImageSha256','points','patchRois','stateFields','parameters')})


def rebind(registration,reference_bytes,current_bytes,frame_id,source_context):
    """명시 현재 원본 대응 검사 후 새 source 등록. 원본 등록은 변경하지 않는다."""
    SceneRelativeObserver(registration,reference_bytes).verify();_context(source_context)
    scene._require(all(source_context[k]==registration['sourceContext'][k]
        for k in ('object','poseRegime')),'relative_rebind_physical_context_changed')
    candidate=copy.deepcopy(registration)
    candidate['sourceContext']=copy.deepcopy(source_context)
    candidate.pop('rebindEvidence',None)
    candidate['registrationSha256']=scene._digest({k:v for k,v in candidate.items() if k!='registrationSha256'})
    observed=SceneRelativeObserver(candidate,reference_bytes).observe(current_bytes,frame_id,source_context)
    candidate['rebindEvidence']=dict(parentRegistrationSha256=registration['registrationSha256'],
        currentImageSha256=scene._sha(current_bytes),frameId=frame_id,
        observationSha256=scene._digest(observed),method='explicit_full_image_bounded_template_correspondence',
        physicalContextVerified=False,modelTransferVerified=False)
    candidate['registrationSha256']=scene._digest({k:v for k,v in candidate.items() if k!='registrationSha256'})
    return candidate


def register(reference_bytes,points,patch_rois,source_context,parameters=None):
    image=_decode(reference_bytes);p=copy.deepcopy(default_parameters() if parameters is None else parameters)
    _parameters(p);_context(source_context)
    _geometry(points,patch_rois,image)
    result=dict(schemaVersion=SCHEMA,points=copy.deepcopy(points),patchRois=copy.deepcopy(patch_rois),
        sourceContext=copy.deepcopy(source_context),parameters=p,imageShape=list(image.shape),
        referenceImageSha256=scene._sha(reference_bytes),stateFields=list(FIELDS),
        meaning='registered_image_plane_relative_points_not_shaft_contact_depth_or_model_transfer',
        seedSemantics='analyst_visible_points_unverified_pixel_accuracy',
        controlEligible=False,trainingEligible=False)
    result['canonicalFrameSha256']=canonical_frame_sha256(result)
    result['registrationSha256']=scene._digest(result)
    SceneRelativeObserver(result,reference_bytes).observe(reference_bytes,source_context['cameraBootId']+':registration',source_context)
    return result


class SceneRelativeObserver:
    def __init__(self,registration,reference_bytes):
        self.registration=copy.deepcopy(registration);self.reference_bytes=reference_bytes
        self.verify()

    def verify(self):
        r=self.registration
        scene._require(isinstance(r,dict) and r.get('schemaVersion')==SCHEMA
            and r.get('registrationSha256')==scene._digest({k:v for k,v in r.items() if k!='registrationSha256'})
            and r.get('referenceImageSha256')==scene._sha(self.reference_bytes)
            and r.get('canonicalFrameSha256')==canonical_frame_sha256(r)
            and r.get('stateFields')==list(FIELDS) and r.get('controlEligible') is False
            and r.get('trainingEligible') is False,'relative_registration_changed')
        _parameters(r['parameters']);_context(r['sourceContext'])
        reference=_decode(self.reference_bytes)
        scene._require(list(reference.shape)==r['imageShape'],'relative_reference_shape_changed')
        _geometry(r['points'],r['patchRois'],reference)

    def observe(self,image_bytes,frame_id,source_context):
        self.verify();r=self.registration;_context(source_context)
        scene._require(source_context==r['sourceContext'],'relative_source_context_changed')
        scene._require(isinstance(frame_id,str) and len(frame_id)<=200
            and frame_id.startswith(source_context['cameraBootId']+':'),'relative_frame_boot_changed')
        reference=_decode(self.reference_bytes);image=_decode(image_bytes)
        matches={}
        for name in NAMES:
            try:matches[name]=_match(image,reference,r['patchRois'][name],r['points'][name],r['parameters'])
            except ValueError as error:
                error.landmark=name
                raise
        for i,name in enumerate(NAMES):
            x,y,w,h=matches[name]['bbox']
            for other in NAMES[i+1:]:
                a,b,c,d=matches[other]['bbox']
                scene._require(min(x+w,a+c)<=max(x,a) or min(y+h,b+d)<=max(y,b),
                               'relative_matched_landmark_patches_overlap')
        scales=[v['scale'] for v in matches.values()];scale=float(np.median(scales))
        scene._require(max(scales)-min(scales)<=r['parameters']['maximumScaleDisagreement'],
                       'relative_nonuniform_scale_or_viewpoint_changed')
        points={name:matches[name]['point'] for name in NAMES}
        obj,left,right=(np.asarray(points[name]) for name in NAMES)
        scene._require(right[0]>left[0] and np.linalg.norm(right-left)>8*scale,'relative_jaw_geometry_invalid')
        relative=(obj-(left+right)/2)/scale
        state=[float(relative[0]),float(relative[1]),float((right[0]-left[0])/scale)]
        return dict(schemaVersion=SCHEMA,frameId=frame_id,state=state,features=dict(zip(FIELDS,state)),
            stateFields=list(FIELDS),pointsCurrentPx=points,observedScale=scale,
            sourceContext=copy.deepcopy(source_context),registrationSha256=r['registrationSha256'],
            canonicalFrameSha256=r['canonicalFrameSha256'],
            referenceImageSha256=r['referenceImageSha256'],imageSha256=scene._sha(image_bytes),
            quality=dict(landmarks=matches,coordinateMeaning='registration_image_scale',
                         allowedTransform='translation_and_bounded_isotropic_scale_only'),
            controlEligible=False,trainingEligible=False,depthVerified=False,contactVerified=False,
            measurementAccuracyVerified=False,modelTransferVerified=False)
