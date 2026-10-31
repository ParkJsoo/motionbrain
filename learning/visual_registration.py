"""Fixed-ID startup registration with bounded, cross-scale background matching.

Camera-free. Never reacquires points after runtime tracking has started.
"""
import cv2
import numpy as np


def locate(source, target, xy, size=41):
    template=cv2.getRectSubPix(source,(size,size),tuple(map(float,xy)))
    search=cv2.getRectSubPix(target,(size+24,size+24),tuple(map(float,xy)))
    if float(template.std())<2 or float(search.std())<2:raise ValueError('texture_missing')
    score=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED)
    _,peak,_,(x,y)=cv2.minMaxLoc(score)
    if not 1<=x<24 or not 1<=y<24:raise ValueError('search_boundary')
    alternative=score.copy();alternative[max(0,y-3):y+4,max(0,x-3):x+4]=-1
    gap=peak-float(alternative.max())
    if peak<.8 or gap<.03:raise ValueError('weak_or_ambiguous_patch')
    offsets=[]
    for a,b,c in [(score[y,x-1],score[y,x],score[y,x+1]),(score[y-1,x],score[y,x],score[y+1,x])]:
        denom=float(a-2*b+c)
        if denom>=-1e-6:raise ValueError('flat_patch_peak')
        offsets.append(float(.5*(a-c)/denom))
    point=np.array(xy)+np.array([x-12+offsets[0],y-12+offsets[1]])
    if np.linalg.norm(point-np.array(xy))>12:raise ValueError('registration_shift')
    return point,dict(score=float(peak),uniquenessGap=gap)


def match_point(seed, current, xy, *, background):
    def match(size):
        q,forward=locate(seed,current,xy,size)
        back,reverse=locate(current,seed,q,size)
        error=float(np.linalg.norm(back-xy))
        if error>1:raise ValueError('roundtrip_registration')
        return q,dict(patchSize=size,forward=forward,reverse=reverse,roundtripPx=error)
    try:
        return match(41)
    except ValueError as exc:
        if not background or str(exc)!='weak_or_ambiguous_patch':raise
    # Broad background patches can contain moving illumination/shadow boundaries.
    # Require agreement of at least two smaller crops, with unchanged score,
    # uniqueness, search radius and roundtrip requirements at each scale.
    matches=[]
    for size in (31,21,15,13,11):
        try:matches.append(match(size))
        except ValueError:continue
    if len(matches)<2:raise ValueError('background_multiscale_support_missing')
    if any(float(np.linalg.norm(a[0]-b[0]))>.5 for i,a in enumerate(matches) for b in matches[i+1:]):
        raise ValueError('background_multiscale_disagreement')
    q,evidence=matches[0]
    evidence['supportingScales']=[dict(x=float(point[0]),y=float(point[1]),**detail) for point,detail in matches]
    return q,evidence


def register(seed_raw,current_raw,points):
    def gray(raw):
        if len(raw)!=1080*1920*4:raise ValueError('registration_raw_size')
        a=np.frombuffer(raw,np.uint8).reshape(1080,1920,4)
        return cv2.GaussianBlur(cv2.cvtColor(a,cv2.COLOR_BGRA2GRAY),(0,0),1.)
    seed,current=gray(seed_raw),gray(current_raw)
    features=[];evidence=[]
    if len({p['id'] for p in points})!=len(points):raise ValueError('duplicate_registration_id')
    for p in points:
        xy=np.array([p['x'],p['y']])
        if type(p.get('reference')) is not bool or not np.isfinite(xy).all() or not 33<=xy[0]<1887 or not 33<=xy[1]<1047:
            raise ValueError('registration_patch_margin_or_role')
        q,detail=match_point(seed,current,xy,background=p['reference'])
        features.append(dict(id=p['id'],x=float(q[0]),y=float(q[1]),visible=True,confidence=1.))
        evidence.append(dict(id=p['id'],**detail))
    return features,evidence
