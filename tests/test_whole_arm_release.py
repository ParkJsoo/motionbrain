import unittest

import cv2
import numpy as np

from learning.near_gripper_tip_observer import NearGripperTipObserver
from learning.whole_arm_release import SCHEMA, fresh_pair_after, visual_open_stop


def sample(t, gap, bottom=833):
    pixels = np.full((240, 320, 3), 180, np.uint8)
    for x in (160 - gap // 2, 160 + gap // 2):
        cv2.rectangle(pixels, (x - 7, 20), (x + 7, 95), (30, 30, 30), -1)
    cv2.rectangle(pixels, (137, 110), (182, 165), (100, 20, 240), -1)
    ok, encoded = cv2.imencode(".png", pixels)
    assert ok
    near = NearGripperTipObserver(policy_version=3).observe(encoded.tobytes())
    clock = dict(sourceEarliest=t, sourceLatest=t + 0.05, availableAt=t + 0.1)
    obj = dict(
        center=[690, bottom - 90], bottom=bottom, upright=True, bbox=[660, bottom - 180, 60, 180]
    )
    return dict(
        schema=SCHEMA,
        viewId="side",
        referenceHeight=180,
        external=dict(imageSha256="a" * 64, object=obj),
        near=near,
        nearClock=dict(clock),
        extClock=dict(clock),
    )


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.base = sample(1.0, 100)
        self.before = sample(2.0, 60)

    def stop(self, current, proven=3.0):
        return visual_open_stop(self.before, current, self.base, proven, 4.5)

    def test_requires_visible_opening_and_table_position(self):
        self.assertIsNotNone(self.stop(sample(4.0, 90)))
        self.assertIsNone(self.stop(sample(4.0, 66)))
        self.assertIsNone(self.stop(sample(4.0, 90, bottom=820)))

    def test_requires_output_and_fresh_dual_sources(self):
        current = sample(4.0, 90)
        self.assertIsNone(self.stop(current, None))
        for key in ("extClock", "nearClock"):
            with self.subTest(key=key):
                mixed = dict(current, **{key: self.before[key]})
                self.assertIsNone(self.stop(mixed))
        self.assertFalse(fresh_pair_after(current, 4.0, 4.5))
        self.assertFalse(fresh_pair_after(current, 3.0, 7.1))

    def test_post_off_decision_requires_complete_finite_source_bounds(self):
        for key in ('extClock', 'nearClock'):
            for field, invalid in (('sourceEarliest', 3.), ('sourceEarliest', 2.99),
                                   ('sourceEarliest', True), ('sourceLatest', 3.99),
                                   ('availableAt', 4.01), ('availableAt', 5.), ('sourceLatest', float('nan'))):
                pair = sample(4., 90)
                pair[key][field] = invalid
                with self.subTest(camera=key, field=field, value=invalid):
                    self.assertFalse(fresh_pair_after(pair, 3., 4.5))
            pair = sample(4., 90)
            del pair[key]['sourceLatest']
            self.assertFalse(fresh_pair_after(pair, 3., 4.5))
        for boundary in (None, float('-inf'), float('nan'), True):
            self.assertFalse(fresh_pair_after(sample(4., 90), boundary, 4.5))
        self.assertTrue(fresh_pair_after(sample(4., 90), 3., 4.5))
