"""두 턱 사이 물체에 대한 초기 어깨 하강 제안. 출력 시간과 구동은 호출자가 관리한다."""
from collections import defaultdict
from math import hypot, isfinite


def _coordinates(features):
    if not isinstance(features, dict):
        return None
    names = ('sy', 'ny', 'height', 'gap', 'leftMargin', 'rightMargin')
    if any(type(features.get(k)) not in (int, float) or not isfinite(features[k]) for k in names):
        return None
    if min(features['leftMargin'], features['rightMargin']) < 0 or min(features['gap'], features['height']) <= 0:
        return None
    point = features['sy'] / features['height'], features['ny'] / features['gap']
    return point if all(isfinite(v) for v in point) else None


def propose_centered_descent(features, demonstrations):
    """열린 집게의 두 영상으로 시연별 최근접 행동을 비교한다. 다른 행동은 하강 반례로 남긴다."""
    query = _coordinates(features)
    if query is None:
        return dict(action='hold', reason='centered_observation_required', votes=[])
    by_episode = defaultdict(list)
    for row in demonstrations:
        if not isinstance(row, dict):
            continue
        episode = row.get('episode')
        if type(episode) is not int or episode <= 0 or not isinstance(row.get('action'), str) or not row['action'].strip():
            continue
        point = _coordinates(row.get('features'))
        if point is None:
            continue
        distance = hypot((point[0] - query[0]) / .7, (point[1] - query[1]) / .2)
        if isfinite(distance):
            by_episode[episode].append((distance, row['action']))

    votes = []
    for episode, samples in by_episode.items():
        distance = min(d for d, _ in samples)
        actions = {action for d, action in samples if d == distance}
        votes.append(dict(episode=episode, distance=distance,
                          action=next(iter(actions)) if len(actions) == 1 else 'ambiguous'))
    votes.sort(key=lambda row: row['distance'])
    if len(votes) < 2 or votes[1]['distance'] > 1:
        reason = 'insufficient_independent_demonstrations'
    elif any(vote['action'] != 'shoulder/down' for vote in votes
             if vote['distance'] <= 1.25 * votes[1]['distance']):
        reason = 'competing_demonstration'
    else:
        return dict(action='shoulder/down', reason='independent_descent_agreement', votes=votes)
    return dict(action='hold', reason=reason, votes=votes)
