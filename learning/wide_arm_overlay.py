"""광각 측면 영상에 보이는 팔꿈치 덮개와 위팔 레일의 정합을 확인한다."""
import cv2
import numpy as np


def _shaft_edges(dark, shoulder, elbow, width):
    axis = (elbow - shoulder) / np.linalg.norm(elbow - shoulder)
    normal = np.array([-axis[1], axis[0]])
    offsets = np.arange(-round(.40 * width), round(.40 * width) + 1)
    edges = []
    for t in (.32, .45, .58, .70):
        points = np.rint(shoulder + t * (elbow - shoulder) + offsets[:, None] * normal).astype(int)
        if ((points < 0).any() or (points[:, 0] >= dark.shape[1]).any()
                or (points[:, 1] >= dark.shape[0]).any()):
            return []
        row = dark[points[:, 1], points[:, 0]]
        center = len(row) // 2
        if not row[center]:
            continue
        left = np.flatnonzero(~row[:center])
        right = np.flatnonzero(~row[center:])
        if not left.size or not right.size:
            continue
        span = center + right[0] - left[-1] - 1
        if .14 * width < span < .48 * width:
            edges.append((t, offsets[left[-1]+1], offsets[center+right[0]-1]))
    return edges


def wide_elbow(small, gold, rings, shoulder):
    """Return an observed elbow-housing center in the half-resolution input."""
    width = rings[2]
    size = max(3, round(.04 * width) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    # Break the thin gold bridge between the distal cap and the forearm rails.
    shells = cv2.morphologyEx(gold, cv2.MORPH_OPEN, kernel)
    dark = small.max(axis=2) < 140
    candidates = []
    for x, y, w, h, area in cv2.connectedComponentsWithStats(shells)[2][1:]:
        if not (.25 * width < w < .48 * width and .23 * width < h < .49 * width
                and .022 * width**2 < area < .078 * width**2):
            continue
        # The exposed flange curves around the top/right of the black motor cap.
        elbow = np.array([x + w - .265 * width, y + .27 * width])
        if not (.7 * width < np.linalg.norm(elbow - shoulder) < 1.26 * width
                and elbow[1] < shoulder[1] - .45 * width):
            continue
        cx, cy = np.rint(elbow).astype(int)
        r = round(.08 * width)
        if min(cx, cy) < r or cx+r >= dark.shape[1] or cy+r >= dark.shape[0]:
            continue
        if dark[cy-r:cy+r+1, cx-r:cx+r+1].mean() < .80:
            continue
        if len(_shaft_edges(dark, shoulder, elbow, width)) >= 3:
            candidates.append(elbow)
    return candidates[0] if len(candidates) == 1 else None


def wide_upper_region(small, shoulder, elbow, width):
    """Fit the two shaft edges on the observed rail between the joint housings."""
    region = np.zeros(small.shape[:2], np.uint8)
    delta = elbow - shoulder
    section = np.array(_shaft_edges(small.max(axis=2) < 140, shoulder, elbow, width))
    if len(section) < 3:
        return region
    normal = np.array([-delta[1], delta[0]]) / np.linalg.norm(delta)
    # Cables can intersect one cross-section. Use median pairwise slopes and
    # intercepts so a single longer span does not determine either edge.
    t = section[:, 0]
    sides = section[:, 1:]
    slopes = [(sides[j] - sides[i]) / (t[j] - t[i])
              for j in range(len(t)) for i in range(j)]
    slope = np.median(slopes, axis=0)
    intercept = np.median(sides - t[:, None] * slope, axis=0)
    vertices = []
    for z in (.13, .91):
        a, b = intercept + z * slope
        if not -.4*width < a < 0 < b < .4*width or not .14*width < b-a < .50*width:
            return region
        center = shoulder + z * delta
        vertices.extend((center + normal * a, center + normal * b))
    cv2.fillConvexPoly(region, np.rint([vertices[i] for i in (0, 1, 3, 2)]).astype(int), 255)
    return region
