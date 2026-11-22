"""Compare the current wrist height to pre-action wide-view pick demonstrations."""
import math
from pathlib import Path

import cv2
import numpy as np

from learning.artifacts import _decode_json
from learning.dual_view_capture import verified
from learning.open_side_approach import WIDE_SIDE_VIEW
from learning.pink_prism_height import open_scene_prism_candidate
from learning.wide_forearm_overlay import _marker_candidates


class WideWristHeight:
    def __init__(self, plan):
        ref = plan['wristHeightProposal']
        model = _decode_json(verified(Path(ref['artifact']), ref['sha256']))
        rows = model['examples']
        self.labels = [row['action'] for row in rows]
        self.heights = np.array([row['relativeHeight'] for row in rows], dtype=float)
        source = plan['phaseObserver']
        rgb_model = _decode_json(verified(Path(source['artifact']), source['sha256']))
        expected = {e['action']:(e['frameId'],e['nearFrameId']) for e in rgb_model['exemplars']}
        if (model['schema'] != 'wide_wrist_height_pick_v1' or model['view'] != WIDE_SIDE_VIEW
                or plan['external']['viewConfigurationId'] != WIDE_SIDE_VIEW
                or model['sourceModelSha256'] != source['sha256']
                or model['trainingEpisode'] != rgb_model['trainingEpisode']
                or self.labels != ['shoulder/down','elbow/down','gripper/close']
                or any(expected.get(r['action']) != (r['frameId'],r['nearFrameId']) for r in rows)
                or self.heights.shape != (3,)
                or any(type(row['relativeHeight']) not in (int, float) for row in rows)
                or not np.isfinite(self.heights).all() or not np.all(np.diff(self.heights) > 0)):
            raise ValueError('invalid_wide_wrist_demonstrations')

    def predict(self, side):
        rgb = np.asarray(side.convert('RGB'))
        if rgb.shape != (1080,1920,3):
            return dict(action='hold', reason='side_geometry_changed')
        box = open_scene_prism_candidate(rgb)
        small = cv2.resize(rgb,None,fx=.5,fy=.5,interpolation=cv2.INTER_AREA)
        matches, _, _ = _marker_candidates(cv2.cvtColor(small,cv2.COLOR_RGB2HSV))
        if len(matches) != 1 or box is None:
            return dict(action='hold', reason='wrist_or_target_ambiguous')
        wrist_y = 2*float(matches[0][2][1])
        height = (wrist_y-box[1])/box[3]
        distance = np.abs(self.heights-height)
        best = int(distance.argmin())
        action = self.labels[best]
        if (not self.heights[0] <= height <= self.heights[-1]
                or sum(math.isclose(d,distance[best],abs_tol=1e-9) for d in distance) != 1):
            action = 'hold'
        return dict(action=action, relativeHeight=height, wristY=wrist_y, targetTop=box[1],
                    targetHeight=box[3])
