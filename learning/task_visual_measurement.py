"""현재 등록한 두 영상의 픽셀 관측. 장치·센서·모델 학습 경로는 없다.

외부 물체 아래끝과 공구 마커, 집게 영상의 물체-두 기준점 상대 Y만 측정한다.
값은 투영 영상 변화이며 물리 높이·접촉·실제 미끄러짐을 증명하지 않는다.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import math

import cv2
import numpy as np

from learning.manual_collection_dataset import require
from learning.pipeline_learning import digest

SCHEMA = 'motionbrain.task_visual_measurement.v1'
STATE = ('objectRisePx', 'toolRisePx', 'slipDyPx')


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _number(x):
    return type(x) in (int, float) and math.isfinite(x)


def _image(raw):
    require(isinstance(raw, bytes) and 0 < len(raw) <= 30*1024*1024, 'task_visual_image_bytes')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    require(image is not None and image.shape[0] >= 32 and image.shape[1] >= 32, 'task_visual_invalid_image')
    return image


def _roi(box, shape):
    require(isinstance(box, list) and len(box) == 4 and all(type(x) is int for x in box),
            'task_visual_roi_fields')
    x0,y0,x1,y1 = box
    require(0 <= x0 < x1 <= shape[1] and 0 <= y0 < y1 <= shape[0], 'task_visual_roi_bounds')
    return x0,y0,x1,y1


def _parameters(parameters, shapes):
    require(isinstance(parameters,dict) and set(parameters) == {
        'externalObject','externalTool','gripperObject','gripperReferences'}, 'task_visual_parameters')
    for name in ('externalObject','externalTool','gripperObject'):
        value = parameters[name]
        require(isinstance(value,dict) and set(value) == {'roi','hsvLower','hsvUpper','areaRange',
                'widthRange','heightRange','ambiguityRatio'}, 'task_visual_component_parameters')
        _roi(value['roi'], shapes['gripper' if name == 'gripperObject' else 'external'])
        low,high = value['hsvLower'],value['hsvUpper']
        require(isinstance(low,list) and isinstance(high,list) and len(low)==len(high)==3
                and all(type(x) is int for x in low+high)
                and all(0 <= a <= b <= limit for a,b,limit in zip(low,high,(179,255,255))),
                'task_visual_hsv_range')
        for key in ('areaRange','widthRange','heightRange'):
            limits=value[key]
            require(isinstance(limits,list) and len(limits)==2
                    and all(_number(x) for x in limits) and 0 < limits[0] <= limits[1],
                    'task_visual_component_limits')
        require(_number(value['ambiguityRatio']) and 0 < value['ambiguityRatio'] <= 1,
                'task_visual_ambiguity_ratio')
    refs=parameters['gripperReferences']
    require(isinstance(refs,list) and len(refs)==2, 'task_visual_two_references_required')
    for ref in refs:
        require(isinstance(ref,dict) and set(ref)=={'templateRoi','searchRoi','minNcc','minPeakGap'},
                'task_visual_reference_parameters')
        x0,y0,x1,y1=_roi(ref['templateRoi'],shapes['gripper'])
        a,b,c,d=_roi(ref['searchRoi'],shapes['gripper'])
        require(a <= x0 < x1 <= c and b <= y0 < y1 <= d
                and c-a > x1-x0+2 and d-b > y1-y0+2
                and _number(ref['minNcc']) and 0 < ref['minNcc'] <= 1
                and _number(ref['minPeakGap']) and 0 <= ref['minPeakGap'] <= 1,
                'task_visual_reference_search')


def _component(image, parameter):
    x0,y0,x1,y1=parameter['roi']
    hsv=cv2.cvtColor(image[y0:y1,x0:x1],cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.asarray(parameter['hsvLower']),np.asarray(parameter['hsvUpper']))
    count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
    choices=[]
    for i in range(1,count):
        x,y,w,h,area=map(int,stats[i])
        if all(lo <= value <= hi for value,(lo,hi) in zip((area,w,h),
                (parameter['areaRange'],parameter['widthRange'],parameter['heightRange']))):
            require(x>0 and y>0 and x+w<x1-x0 and y+h<y1-y0, 'task_visual_component_clipped')
            choices.append(i)
    require(choices, 'task_visual_component_missing')
    choices.sort(key=lambda i:int(stats[i,4]),reverse=True)
    selected=choices[0]
    require(len(choices)==1 or stats[choices[1],4] < stats[selected,4]*parameter['ambiguityRatio'],
            'task_visual_component_ambiguous')
    ys,xs=np.where(labels==selected)
    columns=np.unique(xs)
    # 가운데 80% 열의 아래 경계를 사용한다. 상단 가림과 가장자리 모서리를 제외한다.
    a,b=np.quantile(columns,[.1,.9])
    chosen=columns[(columns>=a)&(columns<=b)]
    require(len(chosen)>=3, 'task_visual_distal_columns_missing')
    bottom=[int(ys[xs==x].max())+y0 for x in chosen]
    x,y,w,h,area=map(int,stats[selected])
    return dict(centroid=[float(centers[selected,0]+x0),float(centers[selected,1]+y0)],
                distalY=float(np.median(bottom)),bbox=[x+x0,y+y0,w,h],area=area,
                distalSpreadPx=float(max(bottom)-min(bottom)),
                candidateCount=len(choices))


def _subpixel(scores, x, y):
    offsets=[]
    for left,mid,right in ((scores[y,x-1],scores[y,x],scores[y,x+1]),
                           (scores[y-1,x],scores[y,x],scores[y+1,x])):
        denominator=float(left-2*mid+right)
        offset=0. if abs(denominator)<1e-9 else .5*float(left-right)/denominator
        offsets.append(float(np.clip(offset,-.5,.5)))
    return offsets


def _reference(gray, parameter, template):
    x0,y0,x1,y1=parameter['searchRoi']
    scores=cv2.matchTemplate(gray[y0:y1,x0:x1],template,cv2.TM_CCOEFF_NORMED)
    require(np.isfinite(scores).all(), 'task_visual_reference_nonfinite')
    _,score,_,(x,y)=cv2.minMaxLoc(scores)
    require(score>=parameter['minNcc'], 'task_visual_reference_ncc')
    require(0 < x < scores.shape[1]-1 and 0 < y < scores.shape[0]-1,
            'task_visual_reference_clipped')
    # 인접 2px 이내는 같은 상관 봉우리로 취급한다. 그 밖의 경쟁 봉우리는 거절한다.
    other=scores.copy(); radius=2
    other[max(0,y-radius):y+radius+1,max(0,x-radius):x+radius+1]=-1
    gap=float(score-other.max())
    require(gap>=parameter['minPeakGap'], 'task_visual_reference_ambiguous')
    dx,dy=_subpixel(scores,x,y)
    return dict(center=[float(x0+x+dx+template.shape[1]/2),float(y0+y+dy+template.shape[0]/2)],
                ncc=float(score),peakGap=gap)


def _features(external,gripper,parameters,templates):
    obj=_component(external,parameters['externalObject'])
    tool=_component(external,parameters['externalTool'])
    grip=_component(gripper,parameters['gripperObject'])
    gray=cv2.cvtColor(gripper,cv2.COLOR_BGR2GRAY)
    refs=[_reference(gray,p,t) for p,t in zip(parameters['gripperReferences'],templates)]
    reference_y=float(np.mean([r['center'][1] for r in refs]))
    features=dict(objectFootY=obj['distalY'],toolMarkerY=tool['centroid'][1],
                  objectRelativeJawY=grip['centroid'][1]-reference_y)
    return features,dict(externalObject=obj,externalTool=tool,gripperObject=grip,gripperReferences=refs)


def register(external_bytes,gripper_bytes,parameters):
    external,gripper=_image(external_bytes),_image(gripper_bytes)
    shapes=dict(external=list(external.shape),gripper=list(gripper.shape))
    _parameters(parameters,shapes)
    gray=cv2.cvtColor(gripper,cv2.COLOR_BGR2GRAY);templates=[];saved=[]
    for p in parameters['gripperReferences']:
        x0,y0,x1,y1=p['templateRoi'];template=gray[y0:y1,x0:x1].copy()
        require(float(template.std())>=3.,'task_visual_reference_texture')
        templates.append(template)
        saved.append(dict(shape=list(template.shape),pixels=base64.b64encode(template.tobytes()).decode(),
                          sha256=_sha(template.tobytes())))
    baseline,landmarks=_features(external,gripper,parameters,templates)
    return dict(schemaVersion=SCHEMA,stateFields=list(STATE),parameters=copy.deepcopy(parameters),
                parametersSha256=digest(parameters),imageShapes=shapes,templates=saved,baseline=baseline,
                referenceImages=dict(externalSha256=_sha(external_bytes),gripperSha256=_sha(gripper_bytes)),
                referenceLandmarks=landmarks,
                meaning='projected_pixel_motion_not_calibrated_height_contact_or_physical_slip')


def measure(external_bytes,gripper_bytes,registration):
    require(isinstance(registration,dict) and registration.get('schemaVersion')==SCHEMA
            and registration.get('stateFields')==list(STATE), 'task_visual_registration_schema')
    parameters=registration['parameters'];shapes=registration['imageShapes']
    require(digest(parameters)==registration['parametersSha256'],'task_visual_parameters_changed')
    external,gripper=_image(external_bytes),_image(gripper_bytes)
    require(shapes==dict(external=list(external.shape),gripper=list(gripper.shape)),
            'task_visual_image_shape_changed')
    _parameters(parameters,shapes)
    require(isinstance(registration['templates'],list) and len(registration['templates'])==2,
            'task_visual_templates_required')
    templates=[]
    for saved,p in zip(registration['templates'],parameters['gripperReferences']):
        raw=base64.b64decode(saved['pixels'],validate=True)
        x0,y0,x1,y1=p['templateRoi'];shape=[y1-y0,x1-x0]
        require(saved['shape']==shape and len(raw)==shape[0]*shape[1] and _sha(raw)==saved['sha256'],
                'task_visual_template_changed')
        templates.append(np.frombuffer(raw,np.uint8).reshape(shape))
    features,landmarks=_features(external,gripper,parameters,templates)
    baseline=registration['baseline']
    require(set(baseline)==set(features) and all(_number(v) for v in baseline.values()),
            'task_visual_baseline_invalid')
    state=dict(objectRisePx=baseline['objectFootY']-features['objectFootY'],
               toolRisePx=baseline['toolMarkerY']-features['toolMarkerY'],
               slipDyPx=features['objectRelativeJawY']-baseline['objectRelativeJawY'])
    return dict(state=state,landmarks=landmarks,features=features,
                quality=dict(valid=True,method=SCHEMA,registrationSha256=digest(registration),
                             inputImages=dict(externalSha256=_sha(external_bytes),gripperSha256=_sha(gripper_bytes)),
                             actualPhysicalSlipVerified=False,calibratedHeight=False))
