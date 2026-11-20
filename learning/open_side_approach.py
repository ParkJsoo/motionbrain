"""열린 팔의 측면 1080p RGB 영상에서 집게 끝과 세워진 분홍 물체를 독립 판독한다."""
from itertools import combinations

import cv2
import numpy as np

from learning.pink_prism_height import open_scene_prism_candidate


def _forearm_region(rgb):
    # 전완에 붙은 녹색 표식 두 개로 탐색 창의 중심·크기를 정한다.
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
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
    if len(pairs) != 1:
        raise ValueError('side_forearm_pair_missing_or_ambiguous')
    left, right, span = pairs[0]
    # 표식 축의 연장점은 창을 배치하는 용도다. 반환할 집게 끝은 원본 픽셀에서 측정한다.
    cx, cy = left - 2.6 * (right - left) + (0., .22 * span)
    return cx, cy, span


def observe_open_side(rgb):
    """물체 상자와 측면 집게 끝, 물체 중심 X·윗면 Y 대비 집게의 오프셋(px)."""
    if not isinstance(rgb, np.ndarray) or rgb.shape != (1080, 1920, 3) or rgb.dtype != np.uint8:
        raise ValueError('side_1080p_rgb_required')
    box = open_scene_prism_candidate(rgb)
    if box is None:
        raise ValueError('side_single_upright_prism_required')
    cx, cy, span = _forearm_region(rgb)
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
        if not 1.9 * span < cx - tip_x < 4.45 * span:
            continue
        if ys.min() == 0 or ys.max() == mask.shape[0] - 1:
            raise ValueError('side_jaw_tip_clipped')
        if -.9 * span < tip_y - cy < 2.8 * span:
            candidates.append((tip_x, tip_y))
    if len(candidates) != 1:
        raise ValueError('side_jaw_contour_missing_or_ambiguous')
    tip_x, tip_y = candidates[0]
    bx, by, bw, _ = box
    return dict(box=list(box), toolPoint=[tip_x, tip_y],
                sideOffsetPx=[tip_x - (bx + bw / 2), tip_y - by])


if __name__ == '__main__':
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', help='외부 카메라의 1920×1080 정지 영상')
    image = cv2.imread(parser.parse_args().image)
    if image is None:
        parser.error('측면 영상을 읽을 수 없습니다.')
    print(json.dumps(observe_open_side(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))))
