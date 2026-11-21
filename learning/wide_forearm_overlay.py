"""광각 측면 표시용 전완·손목·보이는 턱 영역. 좌표는 입력 영상의 픽셀 기준이다."""
from itertools import combinations

import cv2
import numpy as np

from learning.open_side_approach import _compact_patches


def _marker_candidates(hsv):
    # 전완의 큰 녹색 점 두 개와 손목 몸체의 작은 분홍 점을 구분한다.
    green = [c for c, _, _ in _compact_patches(
        hsv, (45, 18, 45), (96, 180, 235), (8, 29, 7, 28, 53))]
    pink = [c for c, _, _ in _compact_patches(
        hsv, (128, 20, 30), (179, 230, 235), (4, 22, 4, 20, 18))]
    matches = []
    for a, b in combinations(green, 2):
        span = np.linalg.norm(a-b)
        if not 22 <= span <= 39:
            continue
        for wrist in pink:
            near, far = sorted((a, b), key=lambda v: np.linalg.norm(v-wrist))
            axis = (near-far)/span
            offset = wrist-near
            normal = np.array([-axis[1], axis[0]])
            if 2.5*span < offset@axis < 4.25*span and abs(offset@normal) < 1.25*span:
                matches.append((near, far, wrist, span))
    return matches, green, pink


def _single_marker_wrist(green, pink, full_tip):
    # 한 표식이 가려지면 양 턱 식별을 통과한 실제 끝점만 보조 근거로 쓴다.
    # 끝점과 손목에서 얻은 축으로 전완을 추정하지 않는다.
    tip = np.asarray(full_tip, dtype=float)/2
    candidates = []
    for wrist in pink:
        delta = tip-wrist
        length = np.linalg.norm(delta)
        if not 64 < length < 122:
            continue
        axis = delta/length
        normal = np.array([-axis[1], axis[0]])
        supports = [m for m in green if -1.7*length < (m-wrist)@axis < -.66*length
                    and abs((m-wrist)@normal) < .65*length]
        if len(supports) == 1:
            candidates.append((wrist, axis, length/3.4))
    return candidates[0] if len(candidates) == 1 else None


def wide_distal_parts(small_rgb, elbow, tip=None):
    """Return half-resolution visible material masks and full-resolution marker metadata."""
    hsv = cv2.cvtColor(small_rgb, cv2.COLOR_RGB2HSV)
    matches, green, pink = _marker_candidates(hsv)
    if len(matches) == 1:
        markers = matches[0]
        near, far, wrist, span = markers
        axis = (near-far)/span
    elif not matches and tip is not None:
        single = _single_marker_wrist(green, pink, tip)
        if single is None:
            return {}, {}
        wrist, axis, span = single
        markers = None
    else:
        return {}, {}
    # 표식 기준 탐색 범위를 제한하고 색 재질이 연결된 부분만 남긴다.
    # 전선처럼 얇은 연결은 마스크를 분리한 뒤 제외한다.
    normal = np.array([-axis[1], axis[0]])
    yy, xx = np.ogrid[:hsv.shape[0], :hsv.shape[1]]
    forward = (xx-wrist[0])*axis[0] + (yy-wrist[1])*axis[1]
    lateral = (xx-wrist[0])*normal[0] + (yy-wrist[1])*normal[1]
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    dark = np.uint8(small_rgb.max(axis=2) < 145)*255
    # 집힌 물체의 그늘진 분홍 면도 명도 문턱을 통과할 수 있다.
    dark[cv2.inRange(hsv, (145, 95, 45), (179, 255, 255)) != 0] = 0
    gold = cv2.inRange(hsv, (8, 75, 75), (39, 255, 255))
    material = cv2.bitwise_or(dark, gold)
    distal = (forward > -1.22*span) & (forward < 4.3*span) & (abs(lateral) < 1.65*span)
    silhouette = cv2.morphologyEx(material*distal, cv2.MORPH_OPEN, kernel)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(silhouette)
    touches = (abs(forward) < .35*span) & (abs(lateral) < .35*span)
    candidates = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] > span**2 and
                  np.count_nonzero(touches & (labels == i)) > .1*span**2]
    if len(candidates) != 1:
        return {}, {}
    attached = labels == candidates[0]
    masks = {}
    wrist_pixels = attached & (forward <= .86*span)
    # A dim wall can join the housing and fill the search window. Require
    # actual material edges on both sides before displaying a wrist silhouette.
    rim = wrist_pixels & (np.abs(lateral) >= 1.65*span - 2)
    rim_support = cv2.dilate(np.uint8(rim), np.ones((5, 5), np.uint8)) != 0
    continuation = rim_support & (np.abs(lateral) >= 1.65*span) & (material != 0)
    if np.count_nonzero(continuation) < span:
        masks['wrist_body'] = np.uint8(wrist_pixels)*255
    landmark = {'wristSurfaceMarker': (2*wrist).tolist()}
    # 판별 가능한 검은 말단이 손목에서 이어질 때만 턱 윤곽을 표시한다.
    # 개별 턱끝의 위치나 양 턱의 식별에는 사용하지 않는다.
    blade = attached & (forward > .86*span) & (dark != 0)
    ys, xs = np.where(blade)
    if len(xs) and not (xs.min()==0 or ys.min()==0 or xs.max()==labels.shape[1]-1
                        or ys.max()==labels.shape[0]-1):
        end = forward[blade].max()/span
        terminal = blade & (forward > (end-.65)*span)
        if (2.05 < end < 4.12 and np.ptp(lateral[terminal]) < 1.25*span and
                np.count_nonzero(terminal) > .16*span**2 and
                np.max(np.abs(lateral[blade])) < 1.61*span):
            masks['visible_jaws'] = np.uint8(blade)*255

    if elbow is not None and markers is not None:
        mask = _forearm_mask(material, elbow, markers, forward, kernel)
        if mask is not None:
            masks['forearm'] = mask
    return masks, landmark


def _forearm_mask(material, elbow, markers, forward, kernel):
    # 검출된 팔꿈치에서 손목으로 이어지는 실제 재질에 전완 표식이 모두
    # 놓일 때만 슬롯과 풀리를 포함한 전완 영역을 표시한다.
    near, far, wrist, span = markers
    link = wrist-elbow
    length = np.linalg.norm(link)
    if not 5.0*span < length < 11.5*span:
        return None
    link /= length
    across = np.array([-link[1], link[0]])
    if not all(abs((m-elbow)@across) < 1.45*span and
               0 < (m-elbow)@link < length for m in (near, far)):
        return None
    yy, xx = np.ogrid[:material.shape[0], :material.shape[1]]
    along = (xx-elbow[0])*link[0] + (yy-elbow[1])*link[1]
    side = (xx-elbow[0])*across[0] + (yy-elbow[1])*across[1]
    # 팔꿈치 바로 아래에서는 위팔과 겹친다. 그 접합부 추정은 생략한다.
    region = (along > 1.75*span) & (forward <= -1.22*span) & (abs(side) < 1.55*span)
    observed = cv2.morphologyEx(material*region, cv2.MORPH_OPEN, kernel)
    _, ids, stats, _ = cv2.connectedComponentsWithStats(observed)
    for i in range(1, len(stats)):
        if stats[i, cv2.CC_STAT_AREA] < 2*span**2:
            continue
        if all(np.any(ids[round(y)-3:round(y)+4, round(x)-3:round(x)+4] == i)
               for x, y in (near, far)):
            return np.uint8(ids == i)*255
    return None
