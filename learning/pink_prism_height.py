"""Locate the upright pink prism in the fixed side-camera lift-height setup."""

import cv2
import numpy as np


def open_scene_prism_candidate(rgb):
    """Find a single upright pink silhouette for open-jaw observation only.

    The aspect and fill checks describe an unobstructed view of the known
    prism. The returned envelope is a candidate, not a lift-height reference.
    """
    boxes = _upright_candidates(_pink_mask(rgb))
    return boxes[0] if boxes and len(boxes) == 1 else None


def _pink_mask(rgb):
    return cv2.inRange(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV),
                       (147, 95, 95), (179, 255, 255))


def _upright_candidates(mask):
    height, width = mask.shape
    candidates = []
    for x, y, w, h, area in cv2.connectedComponentsWithStats(mask)[2][1:]:
        if h < 24 or w < 8 or not 0.22 <= w / h <= 0.41 or area < 0.80 * w * h:
            continue
        if x == 0 or y == 0 or x + w == width or y + h == height:
            return None
        candidates.append((int(x), int(y), int(w), int(h)))
    return candidates


def jaw_occluded_prism_candidate(rgb, tip):
    """턱이 가린 내부 간격만 연결한다. 물체의 위·아래 끝은 분홍 픽셀에서 읽는다."""
    mask = _pink_mask(rgb)
    original = _upright_candidates(mask)
    if original is None or len(original) > 1:
        return None
    x, y = map(round, tip)
    x0, y0 = max(0, x-3), max(0, y-80)
    roi = np.s_[y0:y+81, x0:x+100]
    pink = mask[roi]
    dark = np.uint8(rgb[roi].max(axis=2) < 90)
    _, labels = cv2.connectedComponents(dark)
    near_tip = labels[max(0, y-y0-4):y-y0+5, x-x0:x-x0+5]
    ids = np.unique(near_tip[near_tip != 0])
    if len(ids) == 1:
        # 턱 안쪽 표식도 가림 영역에 속한다. 바깥 윤곽의 2px 색 경계만 허용한다.
        contours, _ = cv2.findContours(np.uint8(labels == ids[0]), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        jaw = np.zeros_like(dark)
        cv2.drawContours(jaw, contours, -1, 1, cv2.FILLED)
        support = cv2.dilate(jaw, np.ones((5, 5), np.uint8))
        for col in range(pink.shape[1]):
            seen = np.flatnonzero(pink[:, col])
            for k in np.flatnonzero(np.diff(seen) > 1):
                gap = slice(seen[k]+1, seen[k+1])
                if (gap.stop-gap.start <= 65 and support[gap, col].all()
                        and jaw[gap, col].any()):
                    pink[gap, col] = 255
    repaired = _upright_candidates(mask)
    if not repaired or len(repaired) != 1:
        return None
    if original:
        # 복원 뒤 다른 물체가 유일한 후보가 되어도 원래 물체를 대체하지 못한다.
        ox, oy, ow, oh = original[0]
        rx, ry, rw, rh = repaired[0]
        if not (rx <= ox and ry <= oy and ox+ow <= rx+rw and oy+oh <= ry+rh):
            return None
    return repaired[0]


def pink_prism_box(rgb, reference_height):
    """Return (x, y, w, h), or None for incomplete or ambiguous silhouettes.

    The recorded pre-lift prism height defines the scale. An opaque jaw can
    split the prism into two vertically aligned patches of the same color.
    """
    if reference_height <= 0:
        raise ValueError("reference_height must be positive")
    image_height, image_width = rgb.shape[:2]
    left, right = int(0.24 * image_width), int(0.52 * image_width)
    top = int(0.18 * image_height)
    crop = rgb[top:, left:right]
    mask = cv2.inRange(cv2.cvtColor(crop, cv2.COLOR_RGB2HSV),
                       (147, 95, 95), (179, 255, 255))
    stats = cv2.connectedComponentsWithStats(mask)[2][1:]
    patches = []
    for x, y, width, height, area in stats:
        if area < 0.009 * reference_height ** 2 or width < 0.12 * reference_height:
            continue
        patches.append((int(x), int(y), int(width), int(height)))

    # A patch beyond the jaw gap belongs to another region. Require horizontal
    # overlap even when the held prism tilts, and a full-height envelope.
    groups = []
    for patch in sorted(patches, key=lambda box: box[1]):
        x, y, w, h = patch
        matches = []
        for group in groups:
            gx, gy, gw, gh = group[-1]
            overlap = min(x+w, gx+gw) - max(x, gx)
            gap = y - (gy+gh)
            if (-0.1 * reference_height <= gap <= 0.25 * reference_height
                    and overlap >= 0.35 * min(w, gw)):
                matches.append(group)
        if len(matches) > 1:
            return None
        if matches:
            matches[0].append(patch)
        else:
            groups.append([patch])

    candidates = []
    for group in groups:
        x = min(p[0] for p in group)
        y = min(p[1] for p in group)
        end_x = max(p[0]+p[2] for p in group)
        end_y = max(p[1]+p[3] for p in group)
        w, h = end_x-x, end_y-y
        if x == 0 or y == 0 or end_x == crop.shape[1] or end_y == crop.shape[0]:
            continue
        if (0.94 * reference_height <= h <= 1.08 * reference_height
                and 0.18 * reference_height <= w <= 0.85 * reference_height):
            candidates.append((left+x, top+y, w, h))
    return candidates[0] if len(candidates) == 1 else None
