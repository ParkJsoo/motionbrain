"""집게 카메라의 단일 RGB 원본 위에 보이는 턱·물체와 같은 행의 투영 간격을 그린다."""
import cv2
import numpy as np

from learning.near_gripper_tip_observer import _object, _tip, observer_policy

COLORS = {'leftJaw': (156, 233, 60), 'rightJaw': (48, 205, 125), 'target': (255, 78, 163)}
TIP_COLOR = (255, 216, 76)
POLICY = observer_policy(version=3)


def _selected_pixels(binary, roi, components):
    """Recover exactly the connected pixels accepted by the observer, including holes."""
    x, y, xx, yy = roi
    _, labels, stats, _ = cv2.connectedComponentsWithStats(binary[y:yy, x:xx])
    selected = np.zeros(binary.shape, dtype=np.uint8)
    patch = selected[y:yy, x:xx]
    for part in components:
        bx, by, width, height = part['bbox']
        key = [bx-x, by-y, width, height, part['area']]
        hits = np.flatnonzero(np.all(stats[1:] == key, axis=1))
        if len(hits) != 1:
            raise ValueError('component_pixel_binding_ambiguous')
        patch[labels == hits[0]+1] = 1
    return selected


def _projected_margins(masks, center_y):
    left, obj, right = (masks[k] for k in ('leftJaw', 'target', 'rightJaw'))
    overlap = np.flatnonzero(left.any(axis=1) & obj.any(axis=1) & right.any(axis=1))
    # Require adjacent visible rows, and never draw a measurement through an occluded row.
    usable = overlap[np.isin(overlap-1, overlap) & np.isin(overlap+1, overlap)]
    roi_left, _, roi_right, _ = POLICY['objectRoi']
    for y in sorted(usable.tolist(), key=lambda v: abs(v-center_y)):
        xs = np.flatnonzero(obj[y])
        inner_left = int(np.flatnonzero(left[y])[-1])
        inner_right = int(np.flatnonzero(right[y])[0])
        obj_left, obj_right = int(xs[0]), int(xs[-1])
        if roi_left < obj_left and obj_right < roi_right-1 and inner_left < obj_left <= obj_right < inner_right:
            return dict(row=y, leftEdge=[inner_left, obj_left], rightEdge=[obj_right, inner_right],
                        leftPx=obj_left-inner_left, rightPx=inner_right-obj_right)
    return None


def _far_target(hsv, masks, tips):
    """Estimate visible color only in the upper approach region of the wide setup."""
    pink = cv2.inRange(hsv, np.uint8(POLICY['objectHsvLow']), np.uint8(POLICY['objectHsvHigh']))
    _, labels, stats, centers = cv2.connectedComponentsWithStats(pink)
    # Count even small fragments before selecting a target. A second plausible
    # patch makes a single-prism interpretation ambiguous in this view.
    candidates = [i for i in range(1, len(stats)) if stats[i, cv2.CC_STAT_AREA] >= 32]
    if len(candidates) != 1:
        return dict(status='unresolved'), None
    i = candidates[0]
    x, y, w, h, area = map(int, stats[i])
    left, right = tips['leftJaw'], tips['rightJaw']
    cx, cy = centers[i].tolist()
    if not (left[0] < x and x+w < right[0] and 0 < y and cy < POLICY['objectRoi'][1]
            and y+h < min(left[1], right[1])):
        return dict(status='unresolved'), None
    mask = np.uint8(labels == i)
    measured = _projected_margins(dict(masks, target=mask), cy)
    if measured is None:
        return dict(status='unresolved'), None
    # A short stripe of pink can be all that is visible under the gripper
    # mount. Its center would be the center of a fragment, not of the prism.
    if not (POLICY['objectWidth'][0] <= w <= POLICY['objectWidth'][1]
            and POLICY['objectHeight'][0] <= h <= POLICY['objectHeight'][1]
            and POLICY['objectArea'][0] <= area <= POLICY['objectArea'][1]
            and h >= .65*w and area >= POLICY['minimumObjectFill']*w*h):
        return dict(status='uncertain_fragment'), None
    return dict(status='observed', bbox=[x, y, w, h], pixels=area, center=[cx, cy],
                centerMinusJawMidPx=[cx-(left[0]+right[0])/2, cy-(left[1]+right[1])/2]), mask


def render_near(rgb, *, wide_approach=False):
    """Return an annotated copy and JSON metadata; coordinates refer to the input frame."""
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.shape != (240, 320, 3):
        raise ValueError('near_overlay_qvga_rgb_required')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    dark = np.uint8(hsv[:, :, 2] < POLICY['blackMaximumValue'])
    masks, tips, parts = {}, {}, {}
    for name, roi in [('leftJaw', POLICY['leftRoi']), ('rightJaw', POLICY['rightRoi'])]:
        try:
            component = _tip(hsv, roi, POLICY)
            masks[name] = _selected_pixels(dark, roi, [component])
        except ValueError as exc:
            parts[name] = {'visible': False, 'reason': str(exc)}
            continue
        tips[name] = component['point']
        parts[name] = dict(visible=True, bbox=component['bbox'], pixels=component['area'], tip=component['point'])
    try:
        obj, fragments = _object(hsv, POLICY)
        pink = cv2.inRange(hsv, np.uint8(POLICY['objectHsvLow']), np.uint8(POLICY['objectHsvHigh']))
        masks['target'] = _selected_pixels(pink, POLICY['objectRoi'], fragments['components'])
        parts['target'] = dict(visible=True, bbox=obj['bbox'], pixels=obj['area'], center=obj['center'],
                               fragmentCount=len(fragments['components']))
    except ValueError as exc:
        parts['target'] = {'visible': False, 'reason': str(exc)}

    margins = None
    if len(tips) == 2:
        l, r = tips['leftJaw'], tips['rightJaw']
        paired = r[0]-l[0] >= POLICY['minimumGapPx'] and abs(r[1]-l[1]) <= POLICY['maximumTipYDifferencePx']
        if not paired:
            for name in tips:
                del masks[name]
                parts[name] = dict(visible=False, reason='near_visible_tips_not_separated')
            tips.clear()
        elif 'target' in masks:
            margins = _projected_margins(masks, obj['center'][1])

    canvas = rgb.copy()
    for name, mask in masks.items():
        color = COLORS[name]
        visible = mask.astype(bool)
        canvas[visible] = np.rint(.65*rgb[visible] + .35*np.asarray(color)).astype(np.uint8)
        # Keep the border inside observed pixels; gaps between visible fragments remain transparent.
        border = mask - cv2.erode(mask, np.ones((3, 3), np.uint8))
        canvas[border.astype(bool)] = color
    if margins is not None:
        for edge in ('leftEdge', 'rightEdge'):
            first, last = margins[edge]
            cv2.line(canvas, (first, margins['row']), (last, margins['row']), TIP_COLOR, 1)
    for point in tips.values():
        cv2.circle(canvas, tuple(np.rint(point).astype(int)), 2, TIP_COLOR, -1)
    info = dict(view='gripper', parts=parts, projectedMargins=margins)
    if wide_approach:
        far = dict(status='unresolved')
        if len(tips) == 2 and 'target' not in masks:
            far, mask = _far_target(hsv, masks, tips)
            if mask is not None:
                border = mask - cv2.erode(mask, np.ones((3, 3), np.uint8))
                canvas[border.astype(bool)] = COLORS['target']
        info['farTarget'] = dict(far, controlEligible=False)
    return canvas, info
