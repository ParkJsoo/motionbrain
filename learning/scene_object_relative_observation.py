"""물체 폭 단위의 영상 평면 관측. 개별 구도의 턱 등록과 모델 좌표를 분리한다.

색 영역의 보이는 사각 외곽과 명시한 보이는 턱 모서리만 관측한다. 등록 사이
같은 3D 점, 깊이, 접촉, 관절 응답의 동등성이나 모델 적용 가능성을 증명하지 않는다.
"""
import copy
import cv2
import numpy as np
from learning import scene_relative_observation as base
from learning import registered_scene_landmarks as common

SCHEMA='motionbrain.scene_object_relative_observation.v1'
FIELDS=('objectMinusJawMidXObjectWidths','objectMinusJawMidYObjectWidths','jawHorizontalSpanObjectWidths')
SEMANTICS='visible_outer_lower_jaw_corners_and_visible_color_box_center_v1'


def parameters():
    return dict(hsvLower=[145,90,100],hsvUpper=[179,255,255],minimumColorArea=100,
        colorOpeningKernel=3,minimumRectangleFill=.93,maximumAspectRatioChange=.12,minimumObjectWidth=12,
        maximumObjectWidth=240,frameEdgeMargin=4,maximumObjectJawScaleDifference=.12,
        jawMatching=base.default_parameters())


def _object(image,p):
    mask=cv2.inRange(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),np.array(p['hsvLower']),np.array(p['hsvUpper']))
    # Opening removes isolated JPEG/color-edge spurs; it does not fill a hole or
    # synthesize a missing object region. Quality uses the surviving mask.
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((p['colorOpeningKernel'],p['colorOpeningKernel']),np.uint8))
    _,_,stats,_=cv2.connectedComponentsWithStats(mask)
    candidates=[s for s in stats[1:] if s[4]>=p['minimumColorArea']]
    common._require(len(candidates)==1,'object_relative_color_missing_or_ambiguous')
    x,y,w,h,area=map(int,candidates[0]);m=p['frameEdgeMargin']
    common._require(x>m and y>m and x+w<image.shape[1]-m and y+h<image.shape[0]-m,
                    'object_relative_clipped')
    common._require(p['minimumObjectWidth']<=w<=p['maximumObjectWidth'] and h>=12
        and area/(w*h)>=p['minimumRectangleFill'],'object_relative_shape_or_occlusion')
    return dict(box=[x,y,x+w,y+h],center=[x+w/2,y+h/2],width=w,height=h,
                area=area,fill=area/(w*h))


def canonical_frame_sha256(source_context):
    return common._digest(dict(schemaVersion=SCHEMA,stateFields=list(FIELDS),
        pointSemantics=SEMANTICS,parameters=parameters(),object=source_context['object']))


def register(reference_bytes,points,patch_rois,source_context):
    image=base._decode(reference_bytes);base._context(source_context)
    common._require(set(points)==set(patch_rois)=={'jawLeft','jawRight'},'object_relative_two_jaws_required')
    obj=_object(image,parameters())
    # Object is independently segmented: a jaw patch may not contain its pixels.
    geometry_points=dict(points,object=obj['center']);geometry_rois=dict(patch_rois,object=obj['box'])
    base._geometry(geometry_points,geometry_rois,image)
    result=dict(schemaVersion=SCHEMA,sourceContext=copy.deepcopy(source_context),points=copy.deepcopy(points),
        patchRois=copy.deepcopy(patch_rois),referenceImageSha256=common._sha(reference_bytes),
        imageShape=list(image.shape),referenceObject=obj,parameters=parameters(),
        stateFields=list(FIELDS),pointSemantics=SEMANTICS,
        canonicalFrameSha256=canonical_frame_sha256(source_context),
        pointCorrespondenceEvidence='explicit_analyst_visible_corner_declaration_not_3d_equivalence',
        controlEligible=False,trainingEligible=False)
    result['registrationSha256']=common._digest(result)
    SceneObjectRelativeObserver(result,reference_bytes).observe(reference_bytes,source_context['cameraBootId']+':registration',source_context)
    return result


class SceneObjectRelativeObserver:
    def __init__(self,registration,reference_bytes):
        self.registration=copy.deepcopy(registration);self.reference_bytes=reference_bytes;self.verify()

    def verify(self):
        r=self.registration;base._context(r['sourceContext'])
        common._require(r.get('schemaVersion')==SCHEMA and r.get('parameters')==parameters()
            and r.get('registrationSha256')==common._digest({k:v for k,v in r.items() if k!='registrationSha256'})
            and r.get('referenceImageSha256')==common._sha(self.reference_bytes)
            and r.get('canonicalFrameSha256')==canonical_frame_sha256(r['sourceContext'])
            and r.get('stateFields')==list(FIELDS) and r.get('pointSemantics')==SEMANTICS
            and r.get('controlEligible') is False and r.get('trainingEligible') is False,
            'object_relative_registration_changed')
        image=base._decode(self.reference_bytes)
        common._require(list(image.shape)==r['imageShape'] and _object(image,r['parameters'])==r['referenceObject'],
                        'object_relative_reference_changed')
        base._geometry(dict(r['points'],object=r['referenceObject']['center']),
                       dict(r['patchRois'],object=r['referenceObject']['box']),image)

    def observe(self,image_bytes,frame_id,source_context):
        self.verify();r=self.registration;p=r['parameters'];base._context(source_context)
        common._require(source_context==r['sourceContext'],'object_relative_source_context_changed')
        common._require(isinstance(frame_id,str) and len(frame_id)<=200
            and frame_id.startswith(source_context['cameraBootId']+':'),'object_relative_frame_boot_changed')
        image=base._decode(image_bytes);reference=base._decode(self.reference_bytes);obj=_object(image,p)
        reference_obj=r['referenceObject'];scale=obj['width']/reference_obj['width']
        ratio=(obj['height']/obj['width'])/(reference_obj['height']/reference_obj['width'])
        common._require(abs(ratio-1)<=p['maximumAspectRatioChange'],'object_relative_aspect_deformed')
        matches={}
        for name in ('jawLeft','jawRight'):
            try:matches[name]=base._match(image,reference,r['patchRois'][name],r['points'][name],p['jawMatching'])
            except ValueError as error:error.landmark=name;raise
        common._require(all(abs(v['scale']/scale-1)<=p['maximumObjectJawScaleDifference'] for v in matches.values()),
                        'object_relative_nonuniform_scale')
        boxes=[obj['box']]+[[v['bbox'][0],v['bbox'][1],v['bbox'][0]+v['bbox'][2],v['bbox'][1]+v['bbox'][3]] for v in matches.values()]
        for i,(a,b,c,d) in enumerate(boxes):
            for x,y,z,w in boxes[i+1:]:
                common._require(min(c,z)<=max(a,x) or min(d,w)<=max(b,y),'object_relative_matched_patches_overlap')
        left,right=(np.array(matches[name]['point']) for name in ('jawLeft','jawRight'))
        common._require(right[0]-left[0]>8*scale,'object_relative_jaw_geometry')
        relative=(np.array(obj['center'])-(left+right)/2)/obj['width']
        state=[float(relative[0]),float(relative[1]),float((right[0]-left[0])/obj['width'])]
        return dict(schemaVersion=SCHEMA,frameId=frame_id,state=state,features=dict(zip(FIELDS,state)),
            stateFields=list(FIELDS),sourceContext=copy.deepcopy(source_context),object=obj,
            pointsCurrentPx=dict(object=obj['center'],**{k:v['point'] for k,v in matches.items()}),
            registrationSha256=r['registrationSha256'],canonicalFrameSha256=r['canonicalFrameSha256'],
            referenceImageSha256=r['referenceImageSha256'],imageSha256=common._sha(image_bytes),
            quality=dict(landmarks=matches,coordinateMeaning='visible_object_width_units',
                         allowedTransform='translation_and_bounded_isotropic_scale_with_independent_jaw_motion'),
            controlEligible=False,trainingEligible=False,depthVerified=False,contactVerified=False,
            measurementAccuracyVerified=False,modelTransferVerified=False)
