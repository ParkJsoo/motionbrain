"""Development trial for the fixed pink-prism pick task."""
import io

import numpy as np

from learning.near_gripper_tip_observer import NearGripperTipObserver
from learning.open_side_approach import WIDE_SIDE_VIEW, observe_open_side
from learning.relative_response_policy import closed_by_controller


class WidePickDevelopment:
    """Use the frozen RGB proposal only with independent optical and sequence checks."""
    def __init__(self, proposer, plan):
        self.labels = proposer.labels
        if (set(self.labels) != {'shoulder/down', 'elbow/down', 'gripper/close', 'elbow/up'}
                or plan['external']['viewConfigurationId'] != WIDE_SIDE_VIEW
                or plan['scene']['object'] != 'pink-prism' or plan['scene']['physicalView'] != WIDE_SIDE_VIEW
                or plan['developmentPick'] != 'pink-prism-pick-and-lift-v1'):
            raise ValueError('unsupported_development_pick')
        self.proposer = proposer
        self.near = NearGripperTipObserver(policy_version=3)

    def predict(self, side, near, *, close_receipt=None, active_action=None):
        proposal = self.proposer.predict(side, near)
        action = proposal['action']
        vote = dict(action='hold', phaseProposal=proposal, reason='geometry_unavailable', features={})
        try:
            side_state = observe_open_side(np.asarray(side.convert('RGB')), view_id=WIDE_SIDE_VIEW)
            box = side_state['box']
            sx, sy = side_state['sideOffsetPx']
            vote['features'] = dict(sideBox=box, sideOffsetPx=[sx, sy])
            if close_receipt is not None and action != 'elbow/up':
                return dict(vote, reason='await_lift_after_close')
            if action in ('elbow/down', 'shoulder/down'):
                if -sy <= 20:
                    return dict(vote, reason='object_near_on_approach')
            else:
                buf = io.BytesIO()
                near.save(buf, format='PNG')
                optical = self.near.observe(buf.getvalue())
                vote['features']['nearState'] = optical['state']
                if not optical['nearClosingVisibilityGate']['passed']:
                    return dict(vote, reason='object_outside_visible_jaws')
                if action == 'elbow/up':
                    if (not closed_by_controller(close_receipt)
                            or close_receipt['action']['record']['state'] != 'completed'):
                        return dict(vote, reason='lift_requires_completed_close')
                    left = optical['components']['left']['bbox']
                    right = optical['components']['right']['bbox']
                    x, _, width, _ = optical['object']['bbox']
                    # Wide demonstration: 48.85px at lift. The existing held-gap
                    # comparison allows 5px; require both lateral margins as well.
                    if (abs(sx) > box[2]/2 or not 0 <= sy < box[3]
                            or x-left[0]-left[2] > 2 or right[0]-x-width > 2
                            or abs(optical['state'][2]-48.85) > 5):
                        return dict(vote, reason='post_close_geometry_unsupported')
                elif not (abs(sx) <= box[2]/2 and 0 <= sy < box[3]):
                    return dict(vote, reason='close_outside_side_object')
        except ValueError as exc:
            return dict(vote, detail=str(exc))
        return dict(vote, action=action, reason='development_pick_proposal')
