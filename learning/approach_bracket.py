"""Select a demonstrated first DOWN waypoint between visually supported starts."""
from math import isfinite


def bracket_waypoint(rows, comparisons, angle):
    """Fallback for unmatched starts in the whole-arm M4 approach profile."""
    if not isfinite(angle) or not 187 <= angle <= 224:
        return None
    ordered = sorted(rows, key=lambda row: row["stationary"][0])
    support = {c["episode"]: c["sceneSupported"] for c in comparisons}
    if any(c["angleSupported"] for c in comparisons):
        return None
    for left, right in zip(ordered, ordered[1:]):
        lo, hi = left["stationary"][1], right["stationary"][0]
        if not lo < angle < hi:
            continue
        anchors = [left["episode"], right["episode"]]
        if not all(support.get(episode) is True for episode in anchors):
            return None
        goals = []
        for row, start in ((left, lo), (right, hi)):
            candidates = [v for v in row["candidates"] if 1 <= start-v <= 8]
            if not candidates:
                return None
            goals.append(min(candidates, key=lambda v: abs(row["end"]-v)))
        fraction = (angle-lo)/(hi-lo)
        ideal = goals[0] + fraction*(goals[1]-goals[0])
        available = [v for v in goals if 1 <= angle-v <= 8 and v >= 187]
        if available:
            return dict(targetAngleDeg=min(available, key=lambda v: (abs(v-ideal), -v)),
                        interpolatedTargetDeg=ideal, anchorEpisodes=anchors)
    return None
