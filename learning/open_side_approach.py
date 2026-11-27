"""열린 팔의 측면 1080p RGB 영상에서 집게 끝과 세워진 분홍 물체를 독립 판독한다."""
from itertools import combinations

import cv2
import numpy as np

from learning.pink_prism_height import jaw_occluded_prism_candidate, open_scene_prism_candidate


def _forearm_regions(hsv):
    # 전완에 붙은 녹색 표식 두 개로 탐색 창의 중심·크기를 정한다.
    mask = cv2.inRange(hsv, (45, 24, 50), (90, 180, 230))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    _, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    marks = []
    for (x, y, w, h, area), center in zip(stats[1:], centers[1:]):
        if 16 <= w <= 65 and 15 <= h <= 65 and area >= 220:
            clipped = x == 0 or y == 0 or x + w == mask.shape[1] or y + h == mask.shape[0]
            marks.append((center, clipped))
    pairs = []
    for (left, left_clipped), (right, right_clipped) in combinations(
            sorted(marks, key=lambda item: item[0][0]), 2):
        dx, dy = right - left
        span = float(np.hypot(dx, dy))
        if 55 <= span <= 85 and 30 < dx < 82 and dy < 0:
            if left_clipped or right_clipped:
                raise ValueError('side_forearm_marker_clipped')
            pairs.append((left, right, span))
    # 표식 축의 연장점은 창을 배치하는 용도다. 반환할 집게 끝은 원본 픽셀에서 측정한다.
    out = []
    for left, right, span in pairs:
        cx, cy = left - 2.6 * (right - left) + (0., .22 * span)
        out.append((cx, cy, span))
    return out


def _forearm_region(hsv):
    pairs = _forearm_regions(hsv)
    if len(pairs) > 1:
        raise ValueError('side_forearm_pair_missing_or_ambiguous')
    return pairs[0] if pairs else None


def _paired_marker_tip(rgb, cx, cy, span):
    x0, x1 = round(cx - 4.7 * span), round(cx + .8 * span)
    y0, y1 = round(cy - 1.3 * span), round(cy + 3.4 * span)
    if not (0 <= x0 < x1 <= rgb.shape[1] and 0 <= y0 < y1 <= rgb.shape[0]):
        raise ValueError('side_jaw_search_region_clipped')

    mask = np.uint8(rgb[y0:y1, x0:x1].max(axis=2) < 90)
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = []
    for label, (x, y, w, h, area) in enumerate(stats[1:], 1):
        tip_x = int(x0 + x)
        if w < 1.4 * span or area < .35 * span ** 2 or tip_x + w < cx - .55 * span:
            continue
        # 보이는 왼쪽 끝 5개 열만 사용한다. 창 경계가 끝점을 가르면 보류한다.
        ys = np.nonzero(labels[y:y+h, x:x+5] == label)[0] + y
        if len(ys) < 4:
            continue
        tip_y = float(y0 + np.median(ys))
        if x < 2 and -.9 * span < tip_y - cy < 2.8 * span:
            raise ValueError('side_jaw_tip_clipped')
        # Forearm marker pixels can move the estimated center by a few pixels
        # while the same visible jaw edge stays fixed near this boundary.
        if not 1.85 * span < cx - tip_x < 4.45 * span:
            continue
        if ys.min() == 0 or ys.max() == mask.shape[0] - 1:
            raise ValueError('side_jaw_tip_clipped')
        if -.9 * span < tip_y - cy < 2.8 * span:
            candidates.append((tip_x, tip_y))
    if len(candidates) != 1:
        raise ValueError('side_jaw_contour_missing_or_ambiguous')
    return candidates[0]


def _compact_patches(hsv, lower, upper, limits, *, remove_thin_edges=False):
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((3, 3), np.uint8)
    if remove_thin_edges:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    _, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    min_w, max_w, min_h, max_h, min_area = limits
    for label, ((x, y, w, h, area), center) in enumerate(zip(stats[1:], centers[1:]), 1):
        if (min_w <= w <= max_w and min_h <= h <= max_h and area >= min_area
                and area >= .49*w*h and x > 0 and y > 0
                and x+w < mask.shape[1] and y+h < mask.shape[0]):
            yield center, (x, y, w, h), labels[y:y+h, x:x+w] == label


class _SingleJawMissing(ValueError):
    """단일 전완 표식 경로에서 유효한 후보가 하나도 없는 경우."""


def _has_distal_mark(hsv, component, left, tip_y):
    # 노란 표식이 동일한 검은 턱 내부에 있어야 한다. 가까운 배경색은 제외한다.
    lo, hi = max(0, tip_y-28), min(component.shape[0], tip_y+29)
    roi = np.s_[lo:hi, left:left+40]
    yellow = cv2.inRange(hsv[roi], (15, 55, 60), (40, 255, 255))
    contours, _ = cv2.findContours(component[roi].astype(np.uint8), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    inside = np.zeros(yellow.shape, np.uint8)
    cv2.drawContours(inside, contours, -1, 255, cv2.FILLED)
    stats = cv2.connectedComponentsWithStats(yellow)[2][1:]
    marks = [(x, y, w, h) for x, y, w, h, area in stats if 8 <= w <= 30 and
             8 <= h <= 29 and area >= .45*w*h]
    if len(marks) != 1:
        return False
    x, y, w, h = marks[0]
    patch = np.s_[y:y+h, x:x+w]
    return (0 < x and x+w < yellow.shape[1] and 0 < y and y+h < yellow.shape[0]
            and np.all(inside[patch][yellow[patch] != 0]))


def _single_marker_tip(rgb, hsv, *, marked_wrist=False, green_core=False):
    # 표식 하나만으로 팔 방향을 외삽할 수 없다. 전완의 녹색 표식과
    # 손목에 붙은 작은 분홍 표식을 독립 판독하고 손목에 연결된 턱 윤곽을 찾는다.
    greens = list(_compact_patches(hsv, (45, 23, 50), (95, 170, 230), (16, 58, 15, 58, 220),
                                  remove_thin_edges=green_core))
    wrists = _compact_patches(hsv, (135, 25, 45), (179, 230, 235), (14, 43, 10, 34, 105))
    candidates = []
    for wrist, (x, y, w, h), patch in wrists:
        if marked_wrist:
            neighbours = [g for g, _, _ in greens if 18 <= g[0]-wrist[0] <= 45
                          and abs(g[1]-wrist[1]) < g[0]-wrist[0]]
            if not neighbours:
                continue
        elif not any(130 < g[0]-wrist[0] < 325 and 50 < wrist[1]-g[1] < 240 for g, _, _ in greens):
            continue
        wx, wy = np.rint(wrist).astype(int)
        x0, x1, y0, y1 = wx-252, wx+54, wy-150, wy+150
        if x0 < 0 or y0 < 0 or x1 > rgb.shape[1] or y1 > rgb.shape[0]:
            raise ValueError('side_jaw_search_region_clipped')
        dark = np.uint8(rgb[y0:y1, x0:x1].max(axis=2) < 90)
        _, labels, stats, _ = cv2.connectedComponentsWithStats(dark)
        # 서로 가까이 투영되더라도 별도 물체의 검은 윤곽을 턱으로 쓰지 않는다.
        rim = np.pad(patch.astype(np.uint8), 1)
        rim = cv2.dilate(rim, np.ones((3, 3), np.uint8)) > rim
        around = labels[y-y0-1:y-y0+h+1, x-x0-1:x-x0+w+1][rim]
        for label, (left, top, width, height, area) in enumerate(stats[1:], 1):
            if area < 1500 or width < 150 or np.count_nonzero(around == label) < 8:
                continue
            ys = np.where(labels[:, left:left+5] == label)[0]
            if not ys.size:
                continue
            if left < 2 or ys.min() == 0 or ys.max() == labels.shape[0]-1:
                raise ValueError('side_jaw_tip_clipped')
            tip = (int(x0 + left), float(y0 + np.median(ys)))
            dx, dy = wrist - tip
            if 150 <= dx <= 226 and abs(dy) < 114 and 168 <= np.hypot(dx, dy) <= 232:
                if marked_wrist and not _has_distal_mark(
                        hsv[y0:y1, x0:x1], labels == label, left, round(tip[1])-y0):
                    continue
                if marked_wrist and len(neighbours) != 1:
                    raise ValueError('side_single_marker_jaw_missing_or_ambiguous')
                candidates.append(tip)
    if not candidates:
        raise _SingleJawMissing('side_single_marker_jaw_missing_or_ambiguous')
    if len(candidates) != 1:
        raise ValueError('side_single_marker_jaw_missing_or_ambiguous')
    return candidates[0]


LEGACY_SIDE_VIEW = 'held-resume-external-20261118'
WIDE_SIDE_VIEW = 'wide-side-manual-20261121'


class _WideJawMissing(ValueError):
    """표식과 연결된 유효한 턱끝 후보가 하나도 없는 경우."""


def _wide_tip(rgb, hsv, *, green_core=False):
    # 새 측면 구도의 손목에는 작은 분홍·녹색 표식이 나란히 붙어 있다.
    # 두 표식은 탐색 방향·대략적인 투영 크기를 제공한다. 끝점 자체는
    # 매 프레임 손목에 연결된 검은 턱의 실제 말단 픽셀에서 찾는다.
    pink = _compact_patches(hsv, (128, 20, 30), (179, 230, 235), (9, 45, 7, 36, 75))
    green = list(_compact_patches(hsv, (45, 18, 45), (96, 180, 235), (8, 40, 8, 42, 80),
                                  remove_thin_edges=green_core))
    candidates = []
    for wrist, (x, y, w, h), patch in pink:
        for neighbour, _, _ in green:
            axis = wrist - neighbour
            span = float(np.linalg.norm(axis))
            if not 18 <= span <= 41:
                continue
            axis /= span
            # The annotated jaw is the distal left-facing digit. When the
            # projected wrist axis is steeper than it is leftward or points
            # right, the two tips can exchange order in this view.
            if axis[0] >= -abs(axis[1]):
                continue
            # The aperture includes the complete visible jaw and some surrounding
            # background. Its boundary may never supply an apparent endpoint.
            cx, cy = np.rint(wrist).astype(int)
            radius = int(np.ceil(8 * span))
            x0, x1 = max(0, cx-radius), min(rgb.shape[1], cx+radius+1)
            y0, y1 = max(0, cy-radius), min(rgb.shape[0], cy+radius+1)
            dark = np.uint8(rgb[y0:y1, x0:x1].max(axis=2) < 90)
            _, labels = cv2.connectedComponents(dark)
            rim = np.pad(patch.astype(np.uint8), 1)
            rim = cv2.dilate(rim, np.ones((3, 3), np.uint8)) > rim
            touch = labels[y-y0-1:y-y0+h+1, x-x0-1:x-x0+w+1][rim]
            for label in np.unique(touch):
                if label == 0 or np.count_nonzero(touch == label) < 8:
                    continue
                yy, xx = np.nonzero(labels == label)
                xx += x0
                yy += y0
                dx, dy = xx-wrist[0], yy-wrist[1]
                forward = dx*axis[0] + dy*axis[1]
                across = -dx*axis[1] + dy*axis[0]
                distal = forward > 3.1*span
                if not distal.any():
                    continue
                end = forward[distal].max()
                if not 4.4*span <= end <= 7.1*span:
                    continue
                # Reject a broad background/lead or a clipped end rather than
                # interpreting the calibrated marker distance as visible travel.
                blade = forward > end-1.2*span
                if (np.max(np.abs(across[blade])) > 2.1*span
                        or np.min(xx[blade]) < 2 or np.max(xx[blade]) >= rgb.shape[1]-2
                        or np.min(yy[blade]) < 2 or np.max(yy[blade]) >= rgb.shape[0]-2):
                    continue
                if any(np.count_nonzero((forward >= end-(i+1)*.25*span)
                                        & (forward < end-i*.25*span)) < .045*span**2
                       for i in range(1, 5)):
                    continue
                # Both distal along the wrist and leftmost in the image must
                # identify the same observed tip. Disagreement indicates a
                # second visible digit or an off-axis contour, not evidence
                # for choosing between them.
                tip_x = int(xx[distal].min())
                edge = distal & (xx < tip_x+5)
                far = forward >= end-3
                if np.hypot(np.median(xx[far])-tip_x,
                            np.median(yy[far])-np.median(yy[edge])) > .65*span:
                    continue
                candidates.append((tip_x, float(np.median(yy[edge]))))
    if not candidates:
        raise _WideJawMissing('wide_wrist_jaw_missing_or_ambiguous')
    if len(candidates) != 1:
        raise ValueError('wide_wrist_jaw_missing_or_ambiguous')
    return candidates[0]


def observe_side_tip(rgb, *, view_id=None, diagnose_green_core=False):
    """등록한 측면 구도에서 현재 프레임에 보이는 턱끝과 판독 방식을 반환한다."""
    if not isinstance(rgb, np.ndarray) or rgb.shape != (1080, 1920, 3) or rgb.dtype != np.uint8:
        raise ValueError('side_1080p_rgb_required')
    if view_id not in (None, LEGACY_SIDE_VIEW, WIDE_SIDE_VIEW):
        raise ValueError('unregistered_side_view')
    if diagnose_green_core and view_id != WIDE_SIDE_VIEW:
        raise ValueError('wide_view_required_for_green_core')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    if view_id == WIDE_SIDE_VIEW:
        try:
            return _wide_tip(rgb, hsv), 'wide_wrist_jaw_contour'
        except _WideJawMissing:
            if not diagnose_green_core:
                raise
        # 진단 요청 시에만 녹색 표식의 가는 돌기를 제거해 같은 프레임을 재검사한다.
        # 복수 후보는 재검사하지 않고, 검은 턱 윤곽은 두 경로 모두 원본에서 측정한다.
        return _wide_tip(rgb, hsv, green_core=True), 'wide_wrist_jaw_green_core'
    region = _forearm_region(hsv)
    if region is None:
        return _single_marker_tip(rgb, hsv), 'single_marker_wrist_contour'
    return _paired_marker_tip(rgb, *region), 'paired_markers'


def observe_open_side(rgb, *, view_id=None, diagnose_green_core=False):
    """물체 상자와 측면 집게 끝, 물체 중심 X·윗면 Y 대비 집게의 오프셋(px)."""
    if not isinstance(rgb, np.ndarray) or rgb.shape != (1080, 1920, 3) or rgb.dtype != np.uint8:
        raise ValueError('side_1080p_rgb_required')
    box = open_scene_prism_candidate(rgb)
    if box is None:
        raise ValueError('side_single_upright_prism_required')
    if diagnose_green_core:
        (tip_x, tip_y), mode = observe_side_tip(rgb, view_id=view_id, diagnose_green_core=True)
    else:
        (tip_x, tip_y), mode = observe_side_tip(rgb, view_id=view_id)
    bx, by, bw, _ = box
    return dict(box=list(box), toolPoint=[tip_x, tip_y],
                trackingMode=mode,
                sideOffsetPx=[tip_x - (bx + bw / 2), tip_y - by])


def observe_open_side_height(rgb, *, view_id):
    """등록한 측면 영상 한 장으로 열린 턱 끝이 물체 높이 범위에 있는지 검사한다."""
    if view_id != LEGACY_SIDE_VIEW:
        raise ValueError('height_check_requires_registered_legacy_view')
    try:
        tip, mode = observe_side_tip(rgb, view_id=view_id)
    except _SingleJawMissing:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        try:
            tip = _single_marker_tip(rgb, hsv, marked_wrist=True)
        except _SingleJawMissing:
            tip = _single_marker_tip(rgb, hsv, marked_wrist=True, green_core=True)
        mode = 'wrist_and_distal_mark'
    box = jaw_occluded_prism_candidate(rgb, tip)
    if box is None:
        raise ValueError('side_single_upright_prism_required')
    y_offset = tip[1] - box[1]
    return dict(box=list(box), toolPoint=list(tip), trackingMode=mode,
                withinHeight=0 <= y_offset < box[3], tipOffsetFromTopPx=y_offset)


def observe_open_start_side(rgb):
    """Use a unique original-pixel jaw contour when a second green pair is visible."""
    box = open_scene_prism_candidate(rgb)
    if box is None:
        raise ValueError('side_single_upright_prism_required')
    bx, by, bw, bh = box
    regions = _forearm_regions(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV))
    tips = []
    for region in regions:
        try:
            tips.append(_paired_marker_tip(rgb, *region))
        except ValueError as exc:
            # A search entirely below the table-supported object cannot
            # describe the raised jaw. Other clipped or ambiguous pairs veto.
            _, cy, span = region
            if str(exc) != 'side_jaw_search_region_clipped' or round(cy-1.3*span) < by+bh:
                raise
    if len(tips) != 1 or tips[0][1]-by > -.5*bh:
        raise ValueError('side_forearm_pair_missing_or_ambiguous')
    tip_x, tip_y = tips[0]
    return dict(box=list(box), toolPoint=[tip_x, tip_y], trackingMode='open_start_paired_markers',
                sideOffsetPx=[tip_x-(bx+bw/2),tip_y-by])


if __name__ == '__main__':
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', help='외부 카메라의 1920×1080 정지 영상')
    parser.add_argument('--height-check', action='store_true', help='등록된 기본 측면 구도의 턱 높이 검사')
    args = parser.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        parser.error('측면 영상을 읽을 수 없습니다.')
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    result = observe_open_side_height(rgb, view_id=LEGACY_SIDE_VIEW) if args.height_check else observe_open_side(rgb)
    print(json.dumps(result))
