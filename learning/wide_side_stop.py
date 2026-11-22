"""Independent wide side-camera veto for bounded dual-view actions."""
import time
from pathlib import Path

import numpy as np

from learning.dual_view_capture import read, side_image
from learning.open_side_approach import WIDE_SIDE_VIEW, observe_side_tip, open_scene_prism_candidate


class WideSideStop:
    def __init__(self, root, config, max_age, *, clock=time.monotonic):
        if config['viewConfigurationId'] != WIDE_SIDE_VIEW:
            raise ValueError('wide_side_view_required')
        self.root = Path(root)
        self.stream = self.root / 'external-stream'
        self.config = config
        self.max_age = max_age
        self.clock = clock
        self.last_index = None
        self.last_source = None

    def covers(self, frame_id):
        session, _, index = frame_id.rpartition(':')
        return (session == self.root.name and index.isdecimal() and self.last_index is not None
                and int(index) <= self.last_index)

    def check(self, label, boundary, *, admission=False):
        """At admission inspect the latest frame, then consume every subsequent frame in order."""
        try:
            paths = sorted((self.stream / 'host-ingress').glob('[0-9][0-9][0-9][0-9].json'))
            if not paths:
                return dict(reason='side_frame_missing')
            if admission:
                paths = paths[-1:]
            elif self.last_index is None:
                return dict(reason='side_admission_missing')
            else:
                paths = [p for p in paths if int(p.stem) > self.last_index]
            for path in paths:
                index = int(path.stem)
                frame = read(path)
                fid = frame['frameId']
                at = self.clock()
                if (fid != f'{self.root.name}:{index}' or frame['frameIndex'] != index
                        or (not admission and index != self.last_index + 1)
                        or not 0 < frame['sourceEarliest'] <= frame['sourceLatest'] <= frame['stdoutReadAt'] <= at
                        or frame['qualityFlags'] or at-frame['sourceEarliest'] > self.max_age):
                    return dict(reason='side_frame_unusable', frameId=fid)
                self.last_index = index
                self.last_source = frame['sourceEarliest']
                if admission and frame['sourceEarliest'] <= boundary:
                    return dict(reason='side_frame_before_off', frameId=fid)
                rgb = np.asarray(side_image(self.stream, frame, self.config).convert('RGB'))
                box = open_scene_prism_candidate(rgb)
                try:
                    tip, _ = observe_side_tip(rgb, view_id=WIDE_SIDE_VIEW)
                except ValueError:
                    return dict(reason='tip_missing', frameId=fid)
                if box is None:
                    return dict(reason='object_missing', frameId=fid)
                if self.clock()-frame['sourceEarliest'] > self.max_age:
                    return dict(reason='side_frame_expired', frameId=fid)
                if label in ('shoulder/down', 'elbow/down') and box[1]-tip[1] <= 20:
                    return dict(reason='object_near_on_approach', frameId=fid, gapPx=box[1]-tip[1])
        except (OSError, KeyError, TypeError, ValueError):
            return dict(reason='side_frame_unverified')
        return None
