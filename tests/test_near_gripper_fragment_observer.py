"""v3의 명시적 관측 조각 결합; 실제 접촉·가림 픽셀 복원 검증 아님."""
import copy
import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from learning.near_gripper_tip_observer import (
    NearGripperTipObserver, FRAGMENT_SCHEMA, validate_observation)


def pixels(*, tail_x=151, tail_y=117, tail_width=10, tail_height=13, third=False):
    image = np.full((240, 320, 3), 180, np.uint8)
    for x in (90, 230):
        cv2.rectangle(image, (x-7, 20), (x+7, 95), (30, 30, 30), -1)
    cv2.rectangle(image, (143, 70), (177, 110), (100, 20, 240), -1)
    cv2.rectangle(image, (tail_x, tail_y), (tail_x+tail_width-1, tail_y+tail_height-1),
                  (100, 20, 240), -1)
    if third:
        cv2.rectangle(image, (150, 145), (159, 154), (100, 20, 240), -1)
    return image


def encoded(image):
    ok, raw = cv2.imencode('.png', image)
    assert ok
    return raw.tobytes()


class FragmentObserverTests(unittest.TestCase):
    def observe(self, image=None):
        return NearGripperTipObserver(policy_version=3).observe(encoded(pixels() if image is None else image))

    def test_v1_v2_exact_policies_and_rejection_remain_unchanged(self):
        self.assertEqual(NearGripperTipObserver().binding,
                         'ce02359a9178f0357175b9e35f2a1bdc36086e5b5d240151b2e12ff7e9ad3102')
        self.assertEqual(NearGripperTipObserver(policy_version=2).binding,
                         '89e6f0e8f52c683e56370f9c08e2fc3bd162094085bb0794a14540fc2249191d')
        for version in (1, 2):
            with self.assertRaisesRegex(ValueError, 'object_missing_or_ambiguous'):
                NearGripperTipObserver(policy_version=version).observe(encoded(pixels()))

    def test_visible_union_uses_sum_area_weighted_centroid_and_bbox_only(self):
        result = self.observe()
        self.assertEqual(result['schemaVersion'], FRAGMENT_SCHEMA)
        metadata = result['objectFragments']
        self.assertEqual(metadata['mode'], 'observed_fragment_union')
        self.assertFalse(metadata['occludedPixelsEstimated'])
        components = metadata['components']; area = sum(c['area'] for c in components)
        self.assertEqual(result['object']['area'], area)
        expected = [sum(c['area']*c['center'][i] for c in components)/area for i in (0, 1)]
        np.testing.assert_allclose(result['object']['center'], expected)
        self.assertEqual(result['object']['bbox'], [143, 70, 35, 60])
        self.assertLess(area, 35*60)  # 빈 공간을 색 픽셀 면적으로 더하지 않는다.
        self.assertEqual(len(validate_observation(result)), 3)
        self.assertTrue(result['nearClosingVisibilityGate']['passed'])
        for field in ('contactVerified', 'forceVerified', 'depthVerified', 'graspVerified', 'clearanceVerified'):
            self.assertFalse(result[field])

    def test_single_component_preserves_v2_geometry_and_tip_logic(self):
        image = pixels(); image[117:130, 151:161] = 180
        old = NearGripperTipObserver(policy_version=2).observe(encoded(image)); new = self.observe(image)
        for key in ('object', 'state', 'points', 'components', 'nearClosingVisibilityGate'):
            self.assertEqual(old[key], new[key])
        self.assertNotIn('objectFragments', old)
        self.assertEqual(new['objectFragments']['mode'], 'single_visible_component')
        self.assertEqual(NearGripperTipObserver(policy_version=3).policy['minimumGapPx'], 12.)

    def test_three_components_ratio_misalignment_or_gap_reject(self):
        cases = [pixels(third=True), pixels(tail_x=149, tail_width=22, tail_height=30),
                 pixels(tail_x=167), pixels(tail_y=122)]
        for image in cases:
            with self.subTest(image_sha=hash(encoded(image))), self.assertRaises(ValueError):
                self.observe(image)
        self.assertEqual(self.observe(pixels(tail_y=121))['objectFragments']['mode'], 'observed_fragment_union')

    def test_union_envelope_density_is_reported_without_inventing_hidden_pixels(self):
        # 각 관측 성분은 .5를 넘지만 가림 사이 빈 행을 포함한 외곽 밀도는 낮다.
        image = pixels(); image[60:180, 130:190] = 180
        cv2.rectangle(image, (145, 70), (174, 77), (100, 20, 240), -1)
        cv2.rectangle(image, (151, 87), (160, 94), (100, 20, 240), -1)
        result = self.observe(image)
        self.assertLess(result['objectFragments']['unionEnvelopeFill'], .5)
        self.assertEqual(result['object']['area'], 320)
        validate_observation(result)
        bad = copy.deepcopy(result)
        bad['objectFragments']['components'][0]['area'] = 100  # 30*8 bbox 밀도 < .5
        with self.assertRaisesRegex(ValueError, 'fragment_geometry'):
            validate_observation(bad)

    def test_fragment_geometry_union_contract_and_claim_tampering_reject(self):
        original = self.observe()
        for key in ('area', 'center', 'bbox', 'mode', 'estimated', 'contract', 'force', 'fill'):
            bad = copy.deepcopy(original)
            if key == 'area': bad['objectFragments']['components'][0]['area'] -= 1
            if key == 'center': bad['objectFragments']['components'][0]['center'][0] += 1
            if key == 'bbox': bad['object']['bbox'][2] += 1
            if key == 'mode': bad['objectFragments']['mode'] = 'single_visible_component'
            if key == 'estimated': bad['objectFragments']['occludedPixelsEstimated'] = True
            if key == 'contract': bad['observationContractSha256'] = 'f'*64
            if key == 'force': bad['forceVerified'] = True
            if key == 'fill': bad['objectFragments']['unionEnvelopeFill'] = 1.
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_observation(bad)
        for value in (float('nan'), float('inf')):
            bad = copy.deepcopy(original); bad['objectFragments']['components'][0]['center'][0] = value
            with self.assertRaises(ValueError): validate_observation(bad)

    def test_bad_partial01_empty_merged_tips_still_reject_when_raw_available(self):
        root = Path(__file__).resolve().parents[1]/'.local/current-entry-20261114/L-pose-M1-close70-partial-20261116-01'
        paths = list(root.glob('sessions/*/gripper-pair/esp32-after.jpg'))
        if not paths: self.skipTest('private raw unavailable')
        with self.assertRaisesRegex(ValueError, 'tip_missing_or_ambiguous'):
            NearGripperTipObserver(policy_version=3).observe(paths[0].read_bytes())

    def test_actual_partial02_visible_union_when_raw_available(self):
        root = Path(__file__).resolve().parents[1]/'.local/current-entry-20261114/L-pose-M1-close70-partial-20261116-02'
        paths = list(root.glob('sessions/*/gripper-pair/esp32-after.jpg'))
        if not paths: self.skipTest('private raw unavailable')
        raw = paths[0].read_bytes()
        with self.assertRaisesRegex(ValueError, 'object_missing_or_ambiguous'):
            NearGripperTipObserver(policy_version=2).observe(raw)
        result = NearGripperTipObserver(policy_version=3).observe(raw)
        validate_observation(result)
        self.assertEqual(result['object']['bbox'], [137, 60, 34, 46])
        self.assertEqual(result['object']['area'], 764)
        self.assertAlmostEqual(result['objectFragments']['unionEnvelopeFill'], 764/(34*46))
        self.assertAlmostEqual(result['state'][2], 27.5568483364, places=6)
        self.assertFalse(result['objectFragments']['occludedPixelsEstimated'])

    def test_fourteen_raw_frames_only_known_fragment_case_changes_admission(self):
        base = Path(__file__).resolve().parents[1]/'.local/current-entry-20261114'
        frozen = base/'L-pose-M1-near-pose-frozen6-20261116-01'
        audits = sorted(frozen.glob('*-near-source-audit.json'))
        partial = list((base/'L-pose-M1-close70-partial-20261116-02').glob('sessions/*/gripper-pair'))
        if len(audits) != 6 or len(partial) != 1:
            self.skipTest('private fourteen-frame cohort unavailable')
        roots = [Path(json.loads(p.read_bytes())['sourceRoot'])/'gripper-pair' for p in audits]+partial
        old, new = NearGripperTipObserver(policy_version=2), NearGripperTipObserver(policy_version=3)
        unchanged = newly_accepted = 0
        for root in roots:
            for side in ('before', 'after'):
                raw = (root/f'esp32-{side}.jpg').read_bytes()
                current = new.observe(raw); validate_observation(current)
                try:
                    original = old.observe(raw)
                except ValueError:
                    self.assertEqual(root, partial[0]); self.assertEqual(side, 'after')
                    newly_accepted += 1
                else:
                    for key in ('state', 'object', 'points', 'components', 'nearClosingVisibilityGate'):
                        self.assertEqual(current[key], original[key])
                    self.assertEqual(current['objectFragments']['mode'], 'single_visible_component')
                    unchanged += 1
        self.assertEqual((unchanged, newly_accepted), (13, 1))


if __name__ == '__main__':
    unittest.main()
