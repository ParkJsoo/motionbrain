"""정확히 같은 유한 명령의 집게 카메라 반응을 광각 수동 전완 모델과 분리해 기록한다."""
from collections import defaultdict
from pathlib import Path

import numpy as np

from learning.action_execution import finite
from learning.artifacts import _decode_json
from learning.dual_view_capture import verified
from learning.open_side_approach import LEGACY_SIDE_VIEW

LABELS = ('shoulder/down', 'elbow/down', 'gripper/close', 'elbow/up')
CHANNELS = ['objectMinusJawMidX', 'objectMinusJawMidY', 'jawGap']


class TimedPickEvidence:
    def __init__(self, plan, reference):
        artifact = _decode_json(verified(Path(reference['artifact']), reference['sha256']))
        if (artifact['schema'] != 'motionbrain.field_near_response.v1'
                or artifact['sourceView'] != LEGACY_SIDE_VIEW or artifact['channels'] != CHANNELS):
            raise ValueError('unknown_timed_response_contract')
        if not set(LABELS) <= plan['profiles'].keys():
            raise ValueError('missing_timed_pick_catalog_profiles')
        self.banks = {}
        by_label = defaultdict(list)
        episodes = set()
        for row in artifact['samples']:
            label = row['label']
            if (label not in LABELS or row['command'] != plan['profiles'][label]['command']
                    or row['sourceEpisode'] in episodes or not row['sourceEpisode']
                    or not row['nearCameraBoot'] or not finite(row['startShoulderDeg'])
                    or row['terminal'] not in ('timed_out', 'completed')
                    or any(not isinstance(row[k], list) or len(row[k]) != len(CHANNELS)
                           or not all(finite(x) for x in row[k]) for k in ('pre', 'delta'))):
                raise ValueError('invalid_timed_response_sample')
            episodes.add(row['sourceEpisode'])
            by_label[label].append(row)
        for label, rows in by_label.items():
            pre = np.array([r['pre'] for r in rows])
            changes = np.array([r['delta'] for r in rows])
            self.banks[label] = dict(count=len(rows), sourceView=LEGACY_SIDE_VIEW,
                command=rows[0]['command'], medianNearDelta=np.median(changes,axis=0).tolist(),
                nearStartMin=pre.min(axis=0).tolist(), nearStartMax=pre.max(axis=0).tolist(),
                observedShoulderDeg=sorted({r['startShoulderDeg'] for r in rows}),
                cameraBootSeen=any(r['nearCameraBoot']==plan['gripperBootId'] for r in rows),
                terminalStates=sorted({r['terminal'] for r in rows}))

    def evaluate(self, observed, close_receipt):
        near = observed['near']
        current = near['state'] if near else None
        # Keep the original signed tip offsets even inside the twenty-pixel stop band.
        side = observed.get('sideGeometry')
        candidates = {}
        for label in LABELS:
            bank = self.banks.get(label)
            support = observed['support'][label]
            # This observer receives no session-bound, post-OFF camera pair with a
            # close receipt. A standalone receipt cannot establish a trial event.
            reasons = ['unbound_close_receipt'] if close_receipt is not None else []
            if not support['allowed']:
                reasons.append('outside_allowed_actions')
            if not support['catalogStartPoseInRange']:
                reasons.append('outside_catalog_start_pose')
            if side is None or near is None:
                reasons.append('missing_current_geometry')
            if bank is None:
                reasons.append('no_exact_command_observations')
            else:
                if observed['planShoulderDeg'] not in bank['observedShoulderDeg']:
                    reasons.append('shoulder_pose_not_sampled')
                if not bank['cameraBootSeen']:
                    reasons.append('near_camera_boot_differs_from_samples')
                if current is not None and (np.any(np.less(current,bank['nearStartMin'])) or
                                            np.any(np.greater(current,bank['nearStartMax']))):
                    reasons.append('outside_observed_near_start_box')
            # The only calibrated timed-response side view in this artifact is legacy.
            # In particular, a loaded object's rise cannot be inferred from an unloaded elbow.
            reasons.append('no_wide_timed_response')
            if label in ('shoulder/down','elbow/down'):
                if side and side['offset'][1] >= -20:
                    reasons.append('approach_stop_band')
            elif label == 'gripper/close':
                if not side or not side['overlap']:
                    reasons.append('close_outside_side_object')
                if not near or not near['closeVisibility']:
                    reasons.append('near_close_visibility_missing')
            else:
                reasons.append('completed_close_required')
                if (not side or not side['overlap'] or not near or not near['closeVisibility']
                        or max(near['margins']) > 2 or abs(near['jawGap']-48.85) > 5):
                    reasons.append('held_geometry_missing')
                reasons.append('no_loaded_object_rise_observed')
            reference = None if bank is None or current is None else np.add(current,bank['medianNearDelta']).tolist()
            candidates[label] = dict(evidence=bank, nearIfMedianTransferred=reference,
                predictedWideSideDelta=None, predictedObjectRise=None, reasons=reasons)
        return dict(action='hold', controlEligible=False, channels=CHANNELS, candidates=candidates)


APPROACH_LABELS = ('shoulder/down', 'elbow/down')
SIDE_RESPONSE_SCHEMA = 'motionbrain.native_timed_approach.v1'


def _native_point(value):
    return isinstance(value, list) and len(value) == 2 and all(finite(v) for v in value)


class NativeTimedApproach:
    """Rank exact approach commands on the native tip-minus-object measurement only."""

    def __init__(self, plan, reference):
        from learning.open_side_approach import WIDE_SIDE_VIEW
        artifact = _decode_json(verified(Path(reference['artifact']), reference['sha256']))
        view = plan['external']['viewConfigurationId']
        if (artifact['schema'] != SIDE_RESPONSE_SCHEMA or artifact['controlEligible'] is not False
                or view not in (LEGACY_SIDE_VIEW, WIDE_SIDE_VIEW)
                or plan['scene']['physicalView'] != view or plan['scene']['object'] != 'pink-prism'
                or artifact['external'] != plan['external'] or artifact['scene'] != plan['scene']
                or artifact['gripperBootId'] != plan['gripperBootId']):
            raise ValueError('native_timed_camera_contract_changed')
        self.samples = defaultdict(list)
        episodes = set()
        for row in artifact['samples']:
            label = row['label']
            posts = row['afterOffsets']
            if (label not in APPROACH_LABELS or label not in plan['profiles']
                    or row['command'] != plan['profiles'][label]['command']
                    or not row['episode'] or row['episode'] in episodes
                    or not finite(row['shoulderDeg']) or not _native_point(row['beforeOffset'])
                    or not isinstance(posts,list) or len(posts)<2 or not all(map(_native_point,posts))
                    or not row['trackingMode'] or not row['sourceRole']
                    or row['terminal'] not in ('completed','timed_out')):
                raise ValueError('invalid_native_timed_sample')
            episodes.add(row['episode'])
            self.samples[label].append(row)

    def evaluate(self, observed, close_receipt=None):
        geometry = observed.get('sideGeometry')
        near = observed.get('near')
        offset = geometry.get('offset') if geometry else None
        current = np.array(offset) if _native_point(offset) else None
        hold_cost = float(np.linalg.norm(current)) if current is not None else None
        costs = {'hold':hold_cost}
        candidates = {}
        choice = 'hold'
        for label in APPROACH_LABELS:
            reasons = []
            rows = self.samples.get(label, [])
            support = observed['support'].get(label, {})
            if not support.get('allowed') or not support.get('catalogStartPoseInRange'):
                reasons.append('outside_allowed_action_or_catalog_pose')
            if close_receipt is not None:
                reasons.append('unbound_close_receipt')
            if not near or near.get('insideJaws') is not True:
                reasons.append('near_target_outside_jaws')
            if current is None or current[1] >= -20:
                reasons.append('missing_side_or_approach_stop_band')
            local = [r for r in rows if r['shoulderDeg']==observed['planShoulderDeg']
                     and r['beforeOffset'][1] < -20 and geometry
                     and r['trackingMode']==geometry.get('trackingMode')]
            prediction = None
            outside_start_box = None
            if not local or current is None:
                reasons.append('no_matching_native_pose_or_tracking_mode')
            else:
                starts = np.array([r['beforeOffset'] for r in local])
                # Camera offset also varies with object placement. Extrapolated translations
                # remain observation-only predictions, even at a matching shoulder angle.
                outside_start_box = bool(np.any(current < starts.min(axis=0))
                                         or np.any(current > starts.max(axis=0)))
                deltas = [np.array(r['afterOffsets'])-r['beforeOffset'] for r in local]
                prediction = current + np.median([np.median(d,axis=0) for d in deltas],axis=0)
                costs[label] = float(np.linalg.norm(prediction))
                if any(np.any(current[1]+d[:,1]>=-20) for d in deltas):
                    reasons.append('predicted_approach_stop_band')
                if not reasons and costs[label] < costs[choice]:
                    choice = label
            candidates[label] = dict(reasons=reasons, episodes=[r['episode'] for r in local],
                outsideObservedSideStartBox=outside_start_box,
                predictedSideOffset=prediction.tolist() if prediction is not None else None)
        return dict(action=choice, reason='native_timed_approach_cost', controlEligible=False,
                    candidates=candidates,costs=costs)
