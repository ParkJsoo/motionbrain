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
