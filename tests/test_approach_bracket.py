import unittest

from learning.approach_bracket import bracket_waypoint


class BracketTests(unittest.TestCase):
    def setUp(self):
        self.rows = [dict(episode=i, stationary=[angle, angle], candidates=[goal], end=194)
                     for i, angle, goal in [(6,220.16,215.42),(7,221.57,213.92),(9,223.59,219.63)]]
        self.matches = [dict(episode=i, angleSupported=False, sceneSupported=True) for i in (6,7,9)]

    def test_intermediate_angle_keeps_demonstrated_target(self):
        for angle in (222.80, 222.89):
            result = bracket_waypoint(self.rows, self.matches, angle)
            self.assertEqual(result["anchorEpisodes"], [7,9])
            self.assertEqual(result["targetAngleDeg"], 219.63)

    def test_no_extrapolation_or_skipping_mismatched_neighbor(self):
        for angle in (219,224,225.87):
            self.assertIsNone(bracket_waypoint(self.rows, self.matches, angle))
        self.matches[1]["sceneSupported"] = False
        self.assertIsNone(bracket_waypoint(self.rows, self.matches, 222.8))

    def test_measured_angle_is_left_to_original_selector(self):
        self.matches[1]["angleSupported"] = True
        self.assertIsNone(bracket_waypoint(self.rows, self.matches, 221.60))
