import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from PIL import Image

from learning.wide_side_stop import WIDE_SIDE_VIEW, WideSideStop


class WideSideStopTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.stop = WideSideStop(self.root, dict(viewConfigurationId=WIDE_SIDE_VIEW), .69, clock=lambda: 10.)
        self.folder = self.root/'external-stream'/'host-ingress'
        self.folder.mkdir(parents=True)
        for name, value in [('side_image', Image.new('RGB', (2, 2))), ('open_scene_prism_candidate', (0, 8, 6, 4))]:
            self.enterContext(patch('learning.wide_side_stop.'+name, return_value=value))
        self.tip = self.enterContext(patch('learning.wide_side_stop.observe_side_tip', return_value=((0, -20), 'wide')))

    def frame(self, i):
        r = dict(frameId=f'{self.root.name}:{i}', frameIndex=i, sourceEarliest=9.7, sourceLatest=9.8,
                 stdoutReadAt=9.9, qualityFlags=[])
        (self.folder/f'{i:04}.json').write_text(json.dumps(r))

    def test_single_transient_missing_before_clear_frame_stops(self):
        self.frame(1)
        self.assertIsNone(self.stop.check('elbow/down', 9., admission=True))
        self.frame(2)
        self.frame(3)
        self.tip.side_effect = [ValueError('missing'), ((0, -20), 'wide')]
        self.assertEqual(self.stop.check('elbow/down', 9.95)['frameId'], f'{self.root.name}:2')

    def test_gap_and_preexisting_approach_hazard(self):
        self.frame(1)
        self.tip.return_value = ((0, 0), 'wide')
        self.assertEqual(self.stop.check('elbow/down', 9., admission=True)['reason'], 'object_near_on_approach')
        self.assertIsNone(self.stop.check('gripper/close', 9., admission=True))
        self.frame(3)
        self.assertEqual(self.stop.check('gripper/close', 9.8)['reason'], 'side_frame_unusable')
