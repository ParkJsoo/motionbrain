"""고정 1080p 측면 구도의 표시용 부품 추정. 매 영상의 색·윤곽으로 독립 판독한다."""
import cv2
import numpy as np

from learning.open_side_approach import observe_side_tip
from learning.pink_prism_height import open_scene_prism_candidate


COLORS = {
    'fixed_base': (161, 174, 188), 'rotating_base': (62, 142, 250),
    'upper_arm': (41, 215, 205), 'forearm': (255, 173, 65),
    'wrist_body': (181, 113, 247), 'visible_jaws': (156, 233, 60),
    'object': (255, 78, 163),
}


def _components(mask):
    return cv2.connectedComponentsWithStats(mask)[2][1:]


def _base_rings(gold):
    # Two or more wide, horizontally aligned yellow bands anchor the bottom assembly.
    bars = cv2.morphologyEx(gold, cv2.MORPH_OPEN, np.ones((1, 36), np.uint8))
    wide = [b for b in _components(bars) if 88 <= b[2] <= 240]
    bands = [b for b in wide if 2 <= b[3] <= 34]
    groups = []
    for b in wide:
        x, y, w, h, _ = b
        if not .2*w < h < .4*w:
            continue
        patch = bars[y:y+h,x:x+w] != 0
        # A few gold pixels at the ends can join three separate slats in one
        # component. Still require three observed vertical runs in many columns.
        starts = patch & ~np.pad(patch[:-1], ((1,0),(0,0)))
        if (starts.sum(axis=0) >= 3).mean() > .4:
            groups.append([b])
    for b in bands:
        x, y, w, h, _ = b
        below = [k for k in bands if .06*w < k[1]-y < .27*w and
                 abs(k[0]-x) < .2*w and .65 < k[2]/w < 1.5]
        if below:
            groups.append([b, *below])
    if not groups:
        return None
    # More than one candidate base location is ambiguous. Adjacent bands of the
    # same assembly must share a component to merge into the same group.
    group = max(groups, key=len)
    if any(not any(np.array_equal(a, b) for a in g for b in group) for g in groups):
        return None
    merged = np.vstack(groups)
    x, y = merged[:, :2].min(axis=0)
    right, bottom = (merged[:, :2]+merged[:, 2:4]).max(axis=0)
    width = right-x
    if y < .8*width or bottom+.09*width >= gold.shape[0] or x-.38*width <= 0 or right+.24*width >= gold.shape[1]:
        return None
    return int(x), int(y), int(width), int(bottom-y)


def _beam_edges(dark, start, end, width):
    # A centerline alone also runs along the lower pulley and its cables. Require
    # the observed rail to end in background on BOTH sides at several cross-sections.
    axis = (end-start)/np.linalg.norm(end-start)
    normal = np.array([-axis[1], axis[0]])
    offsets = np.arange(-round(.36*width), round(.36*width)+1)
    count = 0
    for t in (.30, .40, .50, .60, .70):
        pts = np.rint(start+t*(end-start)+offsets[:, None]*normal).astype(int)
        if ((pts < 0).any() or (pts[:, 0] >= dark.shape[1]).any() or
                (pts[:, 1] >= dark.shape[0]).any()):
            continue
        outside = offsets[dark[pts[:, 1],pts[:, 0]] == 0]
        left, right = outside[outside<0], outside[outside>0]
        if len(left) and len(right) and -.34*width < left[-1] < -.04*width and .04*width < right[0] < .34*width:
            count += 1
    return count >= 3


def _housing_pair(rgb, gold, rings):
    x, y, w, _ = rings
    components = _components(gold)
    # The near-side gold shoulder housing forms a crescent beside the dark cap.
    shells = [(bx, by, height) for bx, by, width, height, size in components
              if .21*w < width < .51*w and .28*w < height < .58*w and size > .033*w*w
              and abs(bx+.65*height-(x+.50*w)) < .18*w and .27*w < y-by-.52*height < .58*w]
    if len(shells) != 1:
        return None
    bx, by, height = shells[0]
    shoulder = np.array([bx+.65*height, by+.52*height])
    dark = np.uint8(rgb.max(axis=2) < 140)
    cx, cy = np.rint(shoulder).astype(int)
    r = round(.11*w)
    if dark[cy-r:cy+r+1,cx-r:cx+r+1].mean() < .35:
        return None

    # Seen from this camera, two separate yellow flanges enclose the elbow
    # gearbox. Overlapping dark circular screws used to compete as elbow axes.
    # A dark cable can cut one flange into several pieces. Join only small
    # interruptions when proposing flange pairs; all evidence and masks below
    # are checked against original, unclosed pixels.
    closed = cv2.morphologyEx(gold, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
    flanges = [b for b in _components(closed) if .18*w < b[2] < .59*w and .25*w < b[3] < .61*w
               and b[4] > .020*w*w and b[1]+b[3] < cy-.33*w]
    # Thin upper ridges can visually connect both flanges. A single broad
    # component is accepted only with gold on both sides and a dark central cap.
    candidates = [np.array([bx+bw/2,by+.64*bh])
                  for bx,by,bw,bh,area in components
                  if .58*w < bw < .84*w and .32*w < bh < .61*w and area > .055*w*w
                  and by+bh < cy-.33*w]
    for lx, ly, lw, lh, _ in flanges:
        for rx, ry, rw, rh, _ in flanges:
            if (-.09*w < rx-lx-lw < .12*w and abs((ry+rh/2)-(ly+lh/2)) < .25*w
                    and .48*w < rx+rw-lx < .84*w):
                candidates.append(np.array([(lx+rx+rw)/2,(min(ry,ly)+max(ry+rh,ly+lh))/2]))
    elbows = []
    for center in candidates:
        if not (.71*w < np.linalg.norm(center-shoulder) < 1.26*w and
                shoulder[1]-center[1] > .45*w):
            continue
        ex,ey = np.rint(center).astype(int)
        radius, outer = round(.09*w), round(.35*w)
        if ex-outer < 0 or ex+outer >= dark.shape[1] or ey-radius < 0:
            continue
        if dark[ey-radius:ey+radius,ex-radius:ex+radius].mean() < .62:
            continue
        edges = (gold[ey-radius:ey+radius,ex-outer:ex-radius],
                 gold[ey-radius:ey+radius,ex+radius:ex+outer])
        if any(np.mean(e != 0) < .09 for e in edges):
            continue
        rail = np.rint(shoulder+(center-shoulder)*np.linspace(.25,.75,20)[:,None]).astype(int)
        if dark[rail[:,1],rail[:,0]].mean() > .85 and _beam_edges(dark,shoulder,center,w):
            elbows.append(center)
    if elbows and np.linalg.norm(np.ptp(elbows,axis=0)) < .065*w:
        return shoulder,np.mean(elbows,axis=0)
    return shoulder,None


def _polygon(shape, vertices):
    area = np.zeros(shape, np.uint8)
    cv2.fillConvexPoly(area, np.rint(vertices).astype(np.int32), 255)
    return area


def _strip(shape, a, b, radius):
    delta = b-a
    normal = np.array([-delta[1], delta[0]]) * (radius / np.linalg.norm(delta))
    return _polygon(shape, [a+normal, b+normal, b-normal, a-normal])


def _attach(labels, name, region, material, min_area):
    overlap = cv2.bitwise_and(region, material)
    if name in ('fixed_base', 'rotating_base', 'upper_arm', 'forearm'):
        # Thin cables touching a larger link can share its connected component.
        # Opening removes protrusions without inventing missing material.
        overlap = cv2.morphologyEx(overlap, cv2.MORPH_OPEN,
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)))
    # Exclude isolated cables/noise; never fill missing background or occlusion.
    count, ids, sizes, _ = cv2.connectedComponentsWithStats(overlap)
    accepted = np.flatnonzero(sizes[1:, cv2.CC_STAT_AREA] >= min_area)+1
    if count > 1 and len(accepted):
        labels[np.isin(ids, accepted)] = list(COLORS).index(name)+1


def render_side(rgb, *, view_id=None):
    """Return a display RGB image and original-pixel metadata for the same frame."""
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.shape != (1080, 1920, 3):
        raise ValueError('side_1080p_rgb_required')
    small = cv2.resize(rgb, (960, 540), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_RGB2HSV)
    gold = cv2.inRange(hsv, (8, 75, 75), (39, 255, 255))
    # Exclude skin, coloured leads, and pale table/background in the local regions.
    dark = cv2.inRange(hsv, (0, 0, 0), (179, 255, 139))
    colored_lead = (hsv[:, :, 1] > 140) & ((hsv[:, :, 0] > 39) | (hsv[:, :, 0] < 8))
    dark[colored_lead] = 0
    material = cv2.bitwise_or(dark, gold)
    labels = np.zeros(gold.shape, np.uint8)
    landmarks = {}
    rings = _base_rings(gold)
    elbow = None
    if rings:
        x,y,w,h = rings
        if np.mean(dark[y:y+h,x:x+w] != 0) < .16:
            rings = None
    if rings:
        x, y, w, h = rings
        area = _polygon(gold.shape, [(x-.38*w, y-.20*w), (x+1.16*w, y-.20*w),
                                     (x+1.24*w, y+h+.09*w), (x-.38*w, y+h+.09*w)])
        _attach(labels, 'fixed_base', area, material, .07*w*w)
        housings = _housing_pair(small, gold, rings)
        if housings:
            shoulder, elbow = housings
            landmarks['shoulderHousingCenterEstimate'] = (shoulder*2).round().astype(int).tolist()
            pad = np.array([1., 0.])*w*.32
            area = _polygon(gold.shape, [shoulder-pad-[0,.18*w], shoulder+pad-[0,.18*w],
                                         [x+1.13*w,y-3], [x-.04*w,y-3]])
            _attach(labels, 'rotating_base', area, material, .035*w*w)
            if elbow is not None:
                landmarks['elbowHousingCenterEstimate'] = (elbow*2).round().astype(int).tolist()
                _attach(labels, 'upper_arm', _strip(gold.shape, shoulder, elbow, .16*w), material, .06*w*w)
    try:
        tip, _ = observe_side_tip(rgb, view_id=view_id)
        landmarks['visibleJawTip'] = list(tip)
    except ValueError:
        tip = None
    if tip is not None:
        # The compact pink patch physically fixed to the wrist must be visible.
        end = np.array(tip)/2
        violet = cv2.inRange(hsv, (135, 25, 45), (179, 230, 235))
        violet = cv2.morphologyEx(violet, cv2.MORPH_CLOSE, np.ones((2, 2), np.uint8))
        patches = []
        for x, y, w, h, size in _components(violet):
            q = np.array([x+w/2, y+h/2])
            dx, dy = q-end
            if (7 <= w <= 22 and 5 <= h <= 17 and size >= .47*w*h and size >= 26 and
                    75 <= dx <= 113 and abs(dy) <= 57 and 84 <= np.linalg.norm(q-end) <= 116):
                patches.append(q)
        if len(patches) == 1:
            wrist = patches[0]
            landmarks['wristSurfaceMarker'] = (2*wrist).tolist()
            direction = (wrist-end)/np.linalg.norm(wrist-end)
            joint = wrist+direction*42
            if elbow is not None and 145 <= np.linalg.norm(elbow-joint) <= 325:
                _attach(labels,'forearm',_strip(gold.shape,joint,elbow,35),material,750)
            # The surface patch lies below the centerline of the wrist housing.
            normal = np.array([direction[1], -direction[0]])
            root = end+.54*(wrist-end)
            _attach(labels,'wrist_body',_strip(gold.shape,root+normal*10,joint+normal*10,23),material,385)
            _attach(labels,'visible_jaws',_strip(gold.shape,end-direction*2,root,16),dark,155)
    box = open_scene_prism_candidate(rgb)
    if box:
        x,y,w,h=box
        obj = _polygon(gold.shape, np.array([[x,y],[x+w-1,y],[x+w-1,y+h-1],[x,y+h-1]])/2)
        _attach(labels,'object',obj,cv2.inRange(hsv,(147,95,95),(179,255,255)),30)
        landmarks['objectBox'] = list(box)
    out = rgb.copy()
    info = {}
    for i,(name,color) in enumerate(COLORS.items(), 1):
        mask = np.uint8(labels==i)*255
        area = cv2.countNonZero(mask)
        info[name] = 'estimated_visible' if area else 'unresolved'
        if not area:
            continue
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        x,y,w,h = cv2.boundingRect(mask)
        roi = out[2*y:2*(y+h),2*x:2*(x+w)]
        full = cv2.resize(mask[y:y+h,x:x+w], (2*w,2*h), interpolation=cv2.INTER_NEAREST)
        tinted = cv2.addWeighted(roi, .68, np.full_like(roi,color), .32, 0)
        cv2.copyTo(tinted, full, roi)
        cv2.drawContours(out,[c*2 for c in contours],-1,color,2,cv2.LINE_AA)
    if tip is not None:
        cv2.circle(out,tuple(np.rint(tip).astype(int)),6,(255,216,76),-1,cv2.LINE_AA)
    return out, {'parts':info,'landmarks':landmarks,'method':'frame_local_material_and_geometry',
                 'controlEligible':False}
