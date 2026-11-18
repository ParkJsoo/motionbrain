import unittest

import numpy as np

from learning.pink_prism_height import pink_prism_box


class PinkPrismHeightTest(unittest.TestCase):
    def setUp(self):
        self.frame = np.zeros((480, 800, 3), np.uint8)
        self.frame[141:322, 245:304] = (235, 23, 106)

    def box(self):
        return pink_prism_box(self.frame, 181)

    def test_intact_and_both_split_area_orderings_have_same_bottom(self):
        for cut in (210, 240):
            with self.subTest(cut=cut):
                self.setUp()
                self.assertEqual(self.box(), (245, 141, 59, 181))
                self.frame[cut:cut+13] = 0
                self.frame[100:117, 345:367] = (235, 23, 106)
                self.assertEqual(self.box(), (245, 141, 59, 181))

    def test_diagonal_jaw_creates_overlapping_component_boxes(self):
        for col in range(245, 304):
            cut = 210 + (col-245)//3
            self.frame[cut:cut+12, col] = 0
        self.assertEqual(self.box(), (245, 141, 59, 181))

    def test_missing_bottom_fifth_cannot_be_used_as_full_object(self):
        self.frame[285:] = 0
        self.assertIsNone(self.box())

    def test_second_object_makes_measurement_ambiguous(self):
        self.frame[121:302, 337:396] = (235, 23, 106)
        self.assertIsNone(self.box())

    def test_bottom_cut_off_at_image_boundary_is_unknown(self):
        self.frame[:] = 0
        self.frame[299:, 245:304] = (235, 23, 106)
        self.assertIsNone(self.box())
