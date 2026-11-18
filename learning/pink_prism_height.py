"""Locate the upright pink prism in the fixed side-camera lift-height setup."""

import cv2


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
