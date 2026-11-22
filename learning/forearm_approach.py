"""광각 왕복 운동에서 얻은 전완의 평면 변환으로 접근 명령의 결과를 예측한다."""
from pathlib import Path

import cv2
import numpy as np

from learning.action_execution import finite
from learning.artifacts import _decode_json
from learning.dual_view_capture import verified
from learning.near_part_overlay import render_near
from learning.open_side_approach import WIDE_SIDE_VIEW, observe_open_side
from learning.wide_forearm_overlay import _marker_candidates

APPROACH_ACTIONS = {f'{joint}/{direction}' for joint, directions in (
    ('elbow', ('up', 'down')), ('shoulder', ('up', 'down')), ('base', ('left', 'right'))
) for direction in directions}


def marks(rgb):
    """전완 표식 두 개와 손목 표식을 입력 영상의 좌표로 돌려준다."""
    if rgb.shape != (1080, 1920, 3):
        return None
    small = cv2.resize(rgb, (960, 540), interpolation=cv2.INTER_AREA)
    matches, _, _ = _marker_candidates(cv2.cvtColor(small, cv2.COLOR_RGB2HSV))
    if len(matches) == 1:
        near, far, wrist, _ = matches[0]
        return 2 * np.stack((far, near, wrist))
    return None


def xy(z):
    return [z.real, z.imag]


def fit_response(start, end):
    """한 행동 전후의 표식으로 두 전완 축의 회전·크기 변화와 손목 이동을 구한다."""
    axis = complex(*(start[1]-start[0]))
    return dict(wristShift=xy(complex(*(end[2]-start[2]))/axis),
                axisRatio=xy(complex(*(end[1]-end[0]))/axis))


def project(effect, markers, point):
    axis = complex(*(markers[1]-markers[0]))
    wrist = complex(*markers[2])
    return np.array(xy(wrist + complex(*effect['wristShift'])*axis
                       + complex(*effect['axisRatio'])*(complex(*point)-wrist)))


def near_geometry(near):
    # Only the gripper's calibrated target ROI is used for this model.
    _, state = render_near(np.asarray(near.convert('RGB')))
    parts = state['parts']
    if not all(parts[k]['visible'] for k in ('leftJaw', 'rightJaw', 'target')):
        return None
    left = parts['leftJaw']['tip'][0]
    right = parts['rightJaw']['tip'][0]
    x, _, width, _ = parts['target']['bbox']
    return dict(insideJaws=bool(left < x and x+width < right),
                center=parts['target']['center'], jawGap=right-left)


class ForearmApproachObserver:
    """한정된 수동 70% 명령의 결과를 추정한다. 실행기용 명령을 발급하지 않는다."""
    def __init__(self, plan):
        ref = plan['forearmApproachObserver']
        model = _decode_json(verified(Path(ref['artifact']), ref['sha256']))
        scene = plan['scene']
        if (model['schema'] != 'wide_forearm_approach_v1' or model['controlEligible'] is not False
                or model['commandMode'] != 'manual_hold_70_percent_from_press'
                or model['configurationSha256'] != plan['external']['configurationSha256']
                or model['view'] != WIDE_SIDE_VIEW or plan['external']['viewConfigurationId'] != WIDE_SIDE_VIEW
                or scene['physicalView'] != WIDE_SIDE_VIEW or scene['object'] != 'pink-prism'):
            raise ValueError('unsupported_forearm_model_or_scene')
        self.horizon = ref['horizonSeconds']
        self.shoulder = ref['shoulderDeg']
        if not finite(self.shoulder) or not finite(self.horizon) or self.horizon not in (.55, 1.5):
            raise ValueError('invalid_forearm_observation_plan')
        self.effects = model['effects'][str(self.horizon)]
        if not self.effects or not self.effects.keys() <= APPROACH_ACTIONS:
            raise ValueError('invalid_forearm_response_actions')
        for entry in self.effects.values():
            for name in ('wristShift', 'axisRatio', 'trainingShoulderRangeDeg'):
                v = entry[name]
                if not isinstance(v, list) or len(v) != 2 or not all(finite(x) for x in v):
                    raise ValueError('invalid_forearm_response_value')
            if entry['trainingShoulderRangeDeg'][0] > entry['trainingShoulderRangeDeg'][1]:
                raise ValueError('invalid_training_shoulder_range')
        self.profiles = plan['profiles']
        allowed = ref['allowedActions']
        if (not isinstance(allowed, list) or any(type(v) is not str for v in allowed)
                or len(set(allowed)) != len(allowed) or not set(allowed) <= self.profiles.keys()):
            raise ValueError('invalid_forearm_allowed_actions')
        self.allowed = set(allowed)
        self.labels = sorted(self.effects.keys() & self.allowed)
        self.support = {}
        for label, profile in self.profiles.items():
            low, high = profile['shoulderRangeDeg']
            entry = self.effects.get(label)
            train = entry['trainingShoulderRangeDeg'] if entry else None
            self.support[label] = dict(allowed=label in self.allowed, fitted=bool(entry),
                catalogStartPoseInRange=low <= self.shoulder <= high,
                trainingShoulderRangeDeg=train, insideTrainingPose=train[0] <= self.shoulder <= train[1] if train else None)

    def predict(self, side, near):
        rgb = np.asarray(side.convert('RGB'))
        markers = marks(rgb)
        near_state = near_geometry(near)
        result = dict(action='hold', controlEligible=False, horizonSeconds=self.horizon,
                      commandMode='manual_hold_70_percent_from_press', planShoulderDeg=self.shoulder,
                      support=self.support, near=near_state, nearTargetRegion='standard_object_roi')
        if markers is None or near_state is None or not near_state['insideJaws']:
            return dict(result, reason='missing_forearm_or_projected_target')
        try:
            g = observe_open_side(rgb, view_id=WIDE_SIDE_VIEW)
        except ValueError as exc:
            return dict(result, reason=str(exc))
        x, y, width, _ = g['box']
        target = np.array([x+width/2, y])
        tip = target + g['sideOffsetPx']
        if tip[1] >= y-20:
            return dict(result, reason='at_or_beyond_approach_boundary')
        costs = {'hold': float(np.linalg.norm(tip-target))}
        projected = {}
        crossing = []
        for label in self.labels:
            end = project(self.effects[label], markers, tip)
            projected[label] = end.tolist()
            costs[label] = float(np.linalg.norm(end-target))
            if end[1] >= y-20:
                crossing.append(label)
        admissible = [label for label in costs if label not in crossing]
        return dict(result, action=min(admissible,key=costs.get), blockedByApproachBoundary=crossing, reason='predicted_side_tip_distance',
                    costs=costs, projected=projected, sideTip=tip.tolist(), target=target.tolist())
