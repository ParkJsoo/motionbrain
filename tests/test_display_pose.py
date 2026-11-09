"""저장 M4 환산은 표시 전용이고 나머지 관절 가정을 보존하는지 확인한다."""
import importlib.util
from pathlib import Path
import math
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('update_pose',ROOT/'tools/visualization/update_pose.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class DisplayPoseTests(unittest.TestCase):
    def test_anchor_mapping_preserves_other_joint_assumptions(self):
        seed=dict(joints={name:.1 for name in module.JOINTS},controlEligible=False)
        anchor=dict(m4Reference=dict(directionSign=1,sensorReferenceDeg=192.83,jointReferenceDeg=56.44269023807929))
        pose=module.make_pose(seed,anchor,208.12,'saved-output-proof')
        self.assertAlmostEqual(math.degrees(pose['joints']['shoulder_pitch_joint']),71.73269023807929)
        self.assertFalse(pose['controlEligible'])
        self.assertFalse(pose['liveHardwareFollowing'])
        self.assertFalse(pose['feedbackCalibrated'])
        self.assertEqual(len(pose['assumedJoints']),4)
        for name in pose['assumedJoints']:self.assertEqual(pose['joints'][name],seed['joints'][name])
        self.assertEqual(seed['joints']['shoulder_pitch_joint'],.1)
        for value in [float('nan'),-1,361]:
            with self.assertRaises(ValueError):module.make_pose(seed,anchor,value,'proof')
