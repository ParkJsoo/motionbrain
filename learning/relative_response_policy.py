"""Choose a finite command by comparing measured two-view responses to demonstrated grasp geometry."""
import math

import cv2
import numpy as np

from learning.dual_view_policy import ACTIONS
from learning.near_gripper_tip_observer import NearGripperTipObserver, shallow_color_core_height
from learning.open_side_approach import LEGACY_SIDE_VIEW, WIDE_SIDE_VIEW, observe_open_side

SCHEMA = 'motionbrain.relative_response_policy.v1'
COORDS = ('sx', 'sy', 'nx', 'ny', 'gap')
APPROACH = ('wrist/up', 'wrist/down', 'elbow/up', 'elbow/down', 'shoulder/down')


def distal_depth(rgb, box):
    """Measure the distal dark region relative to the object's upper boundary."""
    x, y, _, _ = box
    if y < 60 or x < 25 or y+135 > rgb.shape[0] or x+130 > rgb.shape[1]:
        return None
    patch = rgb[y-60:y+135, x-25:x+130].max(axis=2)
    depths = []
    for threshold in (65, 80, 95):
        _, _, stats, _ = cv2.connectedComponentsWithStats(np.uint8(patch < threshold))
        if len(stats) < 2:
            return None
        _, top, _, height, area = stats[1+np.argmax(stats[1:, 4])]
        if area < 200 or top + height >= patch.shape[0]:
            return None
        depths.append(int(top + height - 61))
    return depths if max(depths)-min(depths) <= 4 else None


def optical_profile(f, *, use_shallow_color_core=False):
    """Reproduce the separate shallow and deep pre-contact geometry windows."""
    sx, sy, nx, ny, gap = (f[k] for k in COORDS)
    depths = f['distalDepth']
    if (not depths or abs(nx) > 8 or f['height'] <= max(depths)+4
            or not 8 <= min(depths)-sy or max(depths)-sy > 35):
        return None
    margin = min(f['leftMargin'], f['rightMargin'])
    shallow_height = f['objectHeight']
    if use_shallow_color_core and 50 < shallow_height <= 53:
        shallow_height = f.get('shallowColorCoreHeight', shallow_height)
    if (42 <= min(depths) <= max(depths) <= 61 and max(depths)-min(depths) <= 2
            and f['height']-max(depths)-4 >= 100 and abs(sx) <= f['width']/2-2
            and 20 <= sy <= 42 and 14 <= ny <= 44 and 110 <= gap <= 135 and margin >= 20
            and -8 <= f['nearTop'] <= 21 and 38 <= f['objectWidth'] <= 52 and 38 <= shallow_height <= 50):
        return 'observed_shallow'
    if (70 <= depths[1] <= 97 and -6 <= sx <= 35 and 22 <= sy <= 82
            and 8 <= ny <= 44 and 110 <= gap <= 145 and margin >= 12):
        return 'observed_deep'
    return None


def close_coordinate_errors(f, delta=(0, 0, 0, 0, 0)):
    """Estimate distance to necessary image coordinates for each observed close profile."""
    sx, sy, nx, ny, gap = feature_vector(f) + delta

    def outside(value, low, high):
        return max(low-value, value-high, 0.)

    def errors(x_range, y_range, near_y_range, gap_range):
        side = outside(sx, *x_range) + outside(sy, *y_range)
        near = outside(nx, -8, 8) + outside(ny, *near_y_range) + outside(gap, *gap_range)
        return float(side), float(near)

    half = f['width']/2-2
    return {
        'observed_shallow': errors((-half, half), (20, 42), (14, 44), (110, 135)),
        'observed_deep': errors((-6, 35), (22, 82), (8, 44), (110, 145)),
    }


def feature_vector(f):
    return np.array([f[k] for k in COORDS], dtype=float)


def normalized(f):
    return np.array([f['sx']/f['width'], f['sy']/f['height'], f['nx']/f['gap'], f['ny']/f['gap']])


def closed_by_controller(receipt):
    if not isinstance(receipt, dict) or receipt.get('executionState') != 'known_terminal':
        return False
    action = receipt.get('action') or {}
    req, rec = action.get('request', {}), action.get('record', {})
    writes = [e.get('pwm') for e in action.get('outputLog', {}).get('entries', [])
              if e.get('actionOrdinal') == rec.get('ordinal') and e.get('motor') == 1]
    return (req.get('kind') == 'pulse' and req.get('motor') == rec.get('motor') == 1
            and req.get('commandId') and req['commandId'] == rec.get('commandId')
            and req.get('bootId') and req['bootId'] == rec.get('bootId')
            and rec.get('outputOff') is True and rec.get('state') in ('completed', 'timed_out', 'cancelled')
            and req.get('pwm', 0) < 0 and len(writes) > 1 and all(type(v) is int and v <= 0 for v in writes)
            and min(writes) < 0 and writes[-1] == 0)


class RelativeResponsePolicy:
    def __init__(self, artifact, *, view_id=LEGACY_SIDE_VIEW, approach_evaluation="demonstrated_goal",
                 check_shallow_color_core=False):
        if approach_evaluation not in ("demonstrated_goal", "close_coordinates"):
            raise ValueError("invalid_approach_evaluation")
        self.approach_evaluation = approach_evaluation
        self.check_shallow_color_core = check_shallow_color_core
        if artifact['schema'] != SCHEMA or set(artifact['responses']) != set(ACTIONS):
            raise ValueError('relative_response_schema')
        self.view_id = view_id
        self.calibrated_view = artifact.get('sideViewId', LEGACY_SIDE_VIEW)
        self.responses = artifact['responses']
        self.labels = sorted(self.responses)
        self.goals = artifact['closeGoals']
        self.held = artifact['heldGoals']
        self.bounds = np.asarray(artifact['observationBounds'], dtype=float)
        self.center_band = float(artifact['centeringBandPx'])
        if (not self.goals or not self.held or self.bounds.shape != (2, 4)
                or not np.isfinite(self.bounds).all() or not (self.bounds[1] > self.bounds[0]).all()
                or not 1 <= self.center_band <= 8):
            raise ValueError('relative_response_artifact_range')
        for goal in [*self.goals, *self.held]:
            point = np.asarray(goal['point'], dtype=float)
            if point.shape != (4,) or not np.isfinite(point).all() or not goal.get('source'):
                raise ValueError('invalid_demonstrated_goal')
        if any(g['profile'] not in ('observed_shallow', 'observed_deep') for g in self.goals):
            raise ValueError('invalid_close_profile')
        if any(not math.isfinite(g['gap']) or g['gap'] <= 0 for g in self.held):
            raise ValueError('invalid_held_gap')
        for r in self.responses.values():
            delta = np.asarray(r['delta'], dtype=float)
            channels = r['progressChannels']
            if (delta.shape != (5,) or not np.isfinite(delta).all() or not r['sources']
                    or not 0 < r['stepFraction'] <= 1 or not isinstance(channels, list)
                    or any(type(i) is not int or not 0 <= i < len(delta) for i in channels)
                    or len(channels) != len(set(channels)) or any(delta[i] == 0 for i in channels)):
                raise ValueError('invalid_measured_response')
        self.near = NearGripperTipObserver(policy_version=3)

    def observe(self, side, near):
        rgb = np.asarray(side.convert('RGB'))
        observed = observe_open_side(rgb, view_id=self.view_id)
        image = cv2.cvtColor(np.asarray(near.convert('RGB')), cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode('.png', image)
        if not ok:
            raise ValueError('gripper_image_conversion')
        g = self.near.observe(encoded.tobytes())
        left, right = (g['components'][k]['bbox'] for k in ('left', 'right'))
        bx, by, bw, bh = g['object']['bbox']
        box = observed['box']
        features = dict(zip(COORDS, [*observed['sideOffsetPx'], *g['state']]),
                    width=box[2], height=box[3], sideBox=box, objectWidth=bw, objectHeight=bh,
                    leftMargin=bx-left[0]-left[2], rightMargin=right[0]-bx-bw,
                    nearTop=by-max(left[1]+left[3], right[1]+right[3]), distalDepth=distal_depth(rgb, box))
        if self.check_shallow_color_core and 50 < bh <= 53:
            height = shallow_color_core_height(image, (bx, by, bw, bh))
            if height is not None:
                features['shallowColorCoreHeight'] = height
        return features

    def predict(self, side, near, *, close_receipt=None, active_action=None, eligible=None):
        try:
            features = self.observe(side, near)
        except (ValueError, cv2.error) as exc:
            return self.result('hold', 'two_view_landmarks_unavailable', None, detail=str(exc))
        return self.decide(features, close_receipt=close_receipt, active_action=active_action, eligible=eligible)

    @staticmethod
    def result(action, reason, features, **details):
        return dict(action=action, reason=reason, features=features,
                    viewVotes=details.pop('viewVotes', [None, None]), viewRmse=[None, None], **details)

    def held_matches(self, f, q):
        if not (f['leftMargin'] <= 2 and f['rightMargin'] <= 2):
            return []
        return [row for row in self.held if abs(f['gap']-row['gap']) <= 5 and
                np.linalg.norm((q-row['point']) / np.array([.35,.18,.12,.20])) <= 1]

    def close_feedback(self, f, start, q):
        """Evaluate the live close using retention geometry and the stationary object."""
        if self.held_matches(f, q):
            return self.result('hold', 'close_stop_held_geometry_observed', f, motionDecision='stop')
        if abs(f['nx']) > 8 or f['gap'] > start['gap']+5:
            return self.result('hold', 'close_stop_alignment_or_direction_changed', f, motionDecision='stop')
        if f['gap'] < min(row['gap'] for row in self.held)-5:
            return self.result('hold', 'close_stop_below_held_gap', f, motionDecision='stop')
        # Closing in the registered side view moves the visible jaw down and
        # toward the prism. Include the admitted deep profile's initial overhang.
        side_half_width = max(f['width']/2, abs(start['sx']))
        if abs(f['sx']) > side_half_width+4 or not start['sy']-4 <= f['sy'] <= f['height']:
            return self.result('hold', 'close_stop_side_overlap_lost', f, motionDecision='stop')
        start_box = start['sideBox']
        box = f['sideBox']
        offsets = np.array([box[0]+box[2]/2, box[1], box[1]+box[3]]) - np.array([
            start_box[0]+start_box[2]/2, start_box[1], start_box[1]+start_box[3]])
        if np.max(np.abs(offsets)) > 4:
            return self.result('hold', 'close_stop_object_moved', f, motionDecision='stop')
        return self.result('gripper/close', 'close_continue_object_aligned', f, motionDecision='continue')

    def decide(self, f, *, close_receipt=None, active_action=None, eligible=None, compare_goal_set=False):
        if not isinstance(self.view_id, str) or not self.view_id or self.calibrated_view != self.view_id:
            return self.result('hold', 'response_model_view_mismatch', f)
        if self.view_id == WIDE_SIDE_VIEW:
            # On steep projections the two physical jaw ends can exchange order.
            # Preserve measured features for replay until identity is validated.
            return self.result('hold', 'wide_jaw_identity_observation_only', f)
        if self.view_id != LEGACY_SIDE_VIEW:
            return self.result('hold', 'response_model_unregistered_view', f)
        q = normalized(f)
        if not np.isfinite(q).all() or (q < self.bounds[0]).any() or (q > self.bounds[1]).any():
            return self.result('hold', 'outside_demonstrated_visual_extent', f)
        if active_action is not None:
            label = active_action['label']
            start = active_action['startFeatures']
            if label == 'gripper/close':
                return self.close_feedback(f, start, q)
            movement = feature_vector(f)-feature_vector(start)
            model = self.responses[label]
            delta = np.array(model['delta'])
            # Only channels with resolvable displacement can measure completion.
            channels = model['progressChannels']
            if channels and all(movement[i]/delta[i] >= model['stepFraction'] for i in channels):
                return self.result('hold', 'learned_step_observed', f, measuredProgress=[
                    float(movement[i]/delta[i]) for i in channels], stepFraction=model['stepFraction'])
        if close_receipt is not None:
            if not closed_by_controller(close_receipt):
                return self.result('hold', 'unverified_close_output', f)
            # Narrowed jaws around a visible object and compatible side overlap, as
            # observed immediately before verified lifts. This cannot establish force.
            matches = self.held_matches(f, q)
            if matches:
                if eligible is not None and 'shoulder/up' not in eligible:
                    return self.result('hold', 'no_executable_lift', f)
                return self.result('shoulder/up', 'post_close_lift_example', f,
                                   source=matches[0]['source'], forceVerified=False)
            return self.result('hold', 'contact_geometry_unsupported', f)
        if not 100 <= f['gap'] <= 150:
            return self.result('hold', 'unsupported_open_jaw_gap', f)
        # Close is eligible only in a separately witnessed optical profile. Keep
        # actual successful targets separate rather than averaging disconnected poses.
        close_profile = optical_profile(f, use_shallow_color_core=self.check_shallow_color_core)
        distances = np.array([np.linalg.norm((q-row['point'])/[.65,.22,.10,.24]) for row in self.goals])
        for i in np.argsort(distances):
            if close_profile == self.goals[i]['profile'] and distances[i] <= 1:
                if eligible is not None and 'gripper/close' not in eligible:
                    return self.result('hold', 'no_executable_close', f)
                return self.result('gripper/close', 'preclose_dual_view_match', f,
                                   source=self.goals[i]['source'], profile=close_profile, forceVerified=False)
        if abs(f['nx']) > self.center_band:
            if f['sy'] >= 0 or min(f['leftMargin'], f['rightMargin']) < 3:
                return self.result('hold', 'lateral_sweep_clearance_unverified', f)
            lateral = [label for label in ('base/left','base/right') if eligible is None or label in eligible]
            if not lateral:
                return self.result('hold', 'no_executable_centering', f)
            label = min(lateral, key=lambda a: abs(f['nx']+self.responses[a]['delta'][2]))
            if abs(f['nx']+self.responses[label]['delta'][2]) < abs(f['nx']):
                return self.result(label, 'predicted_centering_improvement', f)
            return self.result('hold', 'no_centering_gain', f)
        if f['sy'] >= f['height']*.45:
            return self.result('hold', 'approach_depth_limit', f)
        if self.approach_evaluation == 'close_coordinates' and not compare_goal_set:
            return self.close_approach(f, eligible)
        # Score the next *measured* displacement against an individual successful
        # pre-close state. Re-observation resolves accumulated kinematic error.
        target = np.asarray(self.goals[int(np.argmin(distances))]['point'])
        scale = np.array([f['width'], f['height'], f['gap'], f['gap']])
        weights = np.array([.85,.55,.10,.22])
        targets = np.stack([goal['point'] for goal in self.goals]) if compare_goal_set else target[None, :]
        current = (q-targets)/weights
        current_error = float(np.min(np.sum(current**2, axis=1)))
        scores = {}
        for label in APPROACH:
            if eligible is not None and label not in eligible:
                continue
            delta = np.asarray(self.responses[label]['delta'][:4])
            estimate = current+delta/scale/weights
            if f['sy']+delta[1] > .45*f['height']:
                continue
            estimate = estimate[np.argmin(np.sum(estimate**2, axis=1))]
            scores[label] = [float(np.dot(estimate[:2],estimate[:2])), float(np.dot(estimate[2:],estimate[2:]))]
        if not scores:
            return self.result('hold', 'no_executable_approach' if eligible is not None else 'no_supported_approach', f)
        best = min(scores, key=lambda label: sum(scores[label]))
        if current_error-sum(scores[best]) <= .025:
            return self.result('hold', 'no_predicted_two_view_gain', f)
        return self.result(best, 'two_view_response_improvement', f, predictedDelta=self.responses[best]['delta'],
                           viewVotes=[min(scores,key=lambda a:scores[a][i]) for i in (0,1)])

    def close_approach(self, f, eligible):
        profiles = {goal['profile'] for goal in self.goals}
        before = close_coordinate_errors(f)
        initial = min(sum(before[profile]) for profile in profiles)
        scores = {}
        for label in APPROACH:
            if eligible is not None and label not in eligible:
                continue
            delta = self.responses[label]['delta']
            if f['sy']+delta[1] > .45*f['height']:
                continue
            coordinates = close_coordinate_errors(f, delta)
            profile = min(sorted(profiles), key=lambda key: sum(coordinates[key]))
            scores[label] = dict(profile=profile, side=coordinates[profile][0], near=coordinates[profile][1])
        if not scores:
            return self.result('hold', 'no_executable_approach' if eligible is not None else 'no_supported_approach', f)
        ranked = sorted(scores, key=lambda label: (scores[label]['side']+scores[label]['near'],
                                                  max(scores[label]['side'], scores[label]['near'])))
        best = ranked[0]
        evidence = dict(closeCoordinateBefore=before, closeCoordinateCandidates=scores)
        if initial-scores[best]['side']-scores[best]['near'] <= 1.:
            return self.result('hold', 'no_predicted_close_coordinate_gain', f, **evidence)
        return self.result(best, 'predicted_close_coordinate_gain', f, predictedDelta=self.responses[best]['delta'],
                           viewVotes=[min(scores, key=lambda label: scores[label][view]) for view in ('side','near')],
                           **evidence)
