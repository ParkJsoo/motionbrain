import unittest
from copy import deepcopy

import numpy as np

from learning.approach_exemplar import exemplar_waypoint, near_object_extent


class ApproachExemplarTests(unittest.TestCase):
    def setUp(self):
        self.obj = dict(upright=True, bottom=834, center=[689, 744])
        self.exemplar = dict(
            anchors=[7, 9],
            startAngle=222.8,
            goal=219.63,
            object=self.obj,
            nearMin=[15, 4, 17, 7],
            nearMax=[15, 5, 17, 8],
            sourceTrial="prior",
        )
        self.decision = dict(
            targetAngleDeg=None,
            action="shoulder/down",
            reason="no_demonstration_matches_angle_and_scene",
            comparisons=[
                dict(episode=7, sceneSupported=True, angleSupported=False),
                dict(episode=9, sceneSupported=False, angleSupported=True),
            ],
        )
        self.rows = [dict(episode=9, stationary=[223.59, 223.59], candidates=[219.63])]

    def result(self, **changes):
        values = dict(
            exemplar=self.exemplar,
            rows=self.rows,
            decision=self.decision,
            angle=223.59,
            scene_mse=0.01,
            max_mse=0.02,
            obj=self.obj,
            extent=[15, 5, 17, 8],
        )
        values.update(changes)
        return exemplar_waypoint(**values)

    def test_only_the_previously_crossed_waypoint(self):
        self.assertEqual(self.result()["targetAngleDeg"], 219.63)
        self.assertEqual(self.result(angle=223.68)["targetAngleDeg"], 219.63)
        for changes in (
            dict(angle=223.79),
            dict(extent=[]),
            dict(obj=dict(self.obj, bottom=float("nan"))),
            dict(scene_mse=0.03),
            dict(extent=None),
            dict(extent=[16, 5, 18, 8]),
            dict(obj=dict(self.obj, bottom=820)),
            dict(obj=dict(self.obj, center=[730, 744])),
            dict(obj=None),
        ):
            self.assertIsNone(self.result(**changes))
        for index, key in [(0, "sceneSupported"), (1, "angleSupported")]:
            d = deepcopy(self.decision)
            d["comparisons"][index][key] = False
            self.assertIsNone(self.result(decision=d))
        self.assertIsNone(self.result(decision=dict(self.decision, targetAngleDeg=220)))
        self.assertIsNone(self.result(decision=dict(self.decision, action="STOP")))

    def test_missing_or_ambiguous_pink_component(self):
        rgb = np.zeros((2, 24, 32, 3), np.float32)
        self.assertIsNone(near_object_extent(rgb))
        rgb[1, 5:8, 15:17] = [1, 0, 0.5]
        self.assertEqual(near_object_extent(rgb), [15, 5, 17, 8])
        rgb[1, 10, 20] = [1, 0, 0.5]
        self.assertIsNone(near_object_extent(rgb))
