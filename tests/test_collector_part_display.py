import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from tools.visualization import collector_parts as parts


class PartDisplayTest(unittest.TestCase):
    def test_legend_swatches_match_both_renderers(self):
        from learning.side_part_overlay import COLORS as side
        from learning.near_part_overlay import COLORS as near, TIP_COLOR
        legend = parts.legend_image()
        colors = [side[k] for k in ('fixed_base', 'rotating_base', 'upper_arm', 'forearm', 'wrist_body')]
        colors += [near['leftJaw'], near['rightJaw'], near['target'], TIP_COLOR]
        for i, color in enumerate(colors):
            np.testing.assert_array_equal(legend[15+i//5*36, 17+i%5*166], color)

    def test_decision_requires_both_exact_displayed_frames_and_a_complete_append(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'decisions.jsonl'
            ids = {'external': 'side:1', 'gripper': 'near:5'}
            vote = dict(frameId='side:1', nearFrameId='near:5', action='hold', modelAction='wrist/down',
                        reason='test', decidedAt=8.0)
            p.write_bytes(json.dumps(vote).encode() + b'\n{"frameId":')
            self.assertEqual(parts.latest_decision(p, ids), vote)
            self.assertIsNone(parts.latest_decision(p, dict(ids, external='side:2')))
            self.assertIsNone(parts.latest_decision(p, dict(ids, gripper='near:6')))
            self.assertIsNone(parts.latest_decision(p, {}))

    def test_invalid_completed_decision_hides_the_previous_matching_vote(self):
        ids = {'external': 'side:1', 'gripper': 'near:5'}
        vote = dict(frameId='side:1', nearFrameId='near:5', action='hold', modelAction='hold', decidedAt=8.)
        good = json.dumps(vote).encode() + b'\n'
        invalid_fields = [dict(vote, action=''), dict(vote, modelAction='unknown'), dict(vote, reason={}),
                          {k: v for k, v in vote.items() if k != 'decidedAt'}]
        invalid_fields += [dict(vote, decidedAt=v) for v in (None, True, -1, float('nan'), float('inf'))]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'decisions.jsonl'
            malformed = [b'{"broken"', b'\xff', b'null', b'[1]', b'{"action":"hold"}',
                         b'{"frameId":"side:1","nearFrameId":"near:5"}']
            for invalid in malformed + [json.dumps(v).encode() for v in invalid_fields]:
                with self.subTest(invalid=invalid):
                    path.write_bytes(good + invalid + b'\n')
                    self.assertIsNone(parts.latest_decision(path, ids))

    def test_newest_raw_remains_visible_when_older_overlay_finishes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            captured = time.monotonic()
            rgb = np.zeros((10, 12, 3), np.uint8)
            started = threading.Event()
            replaced = threading.Event()
            stop = threading.Event()
            records = []
            def reader(_, camera, single, slots, lock, event, include_rgb):
                self.assertTrue(include_rgb)
                with lock:
                    slots[camera] = (Path(camera+'1'), (captured, camera+':1', b'raw1', rgb))
                if camera == 'external':
                    started.wait(1)
                    with lock:
                        slots[camera] = (Path(camera+'2'), (captured, camera+':2', b'raw2', rgb+2))
                    replaced.set()

            def side(image):
                if image[0,0,0] == 0:
                    started.set()
                    replaced.wait(1)
                return image.copy(), {'pixel': int(image[0,0,0])}

            def record(line, **kwargs):
                event = json.loads(line)
                records.append(event)
                if event.get('kind') == 'overlay' and event['frameId'] == 'external:2':
                    stop.set()

            timer = threading.Timer(2, stop.set)
            timer.start()
            try:
                with patch.object(parts, 'watch', reader), patch.object(parts, 'print', side_effect=record), \
                        patch('learning.side_part_overlay.render_side', side_effect=side), \
                        patch('learning.near_part_overlay.render_near', side_effect=lambda im: (im, {})):
                    parts.monitor_parts(root, MagicMock(), MagicMock(), stop)
            finally:
                timer.cancel()
            overlay = [e for e in records if e['camera']=='external' and e['kind']=='overlay']
            self.assertEqual([e['frameId'] for e in overlay], ['external:2'])
            self.assertEqual(overlay[0]['observation']['pixel'], 2)

    def test_failed_detector_preserves_unannotated_original_until_expiry(self):
        rgb = np.zeros((8, 8, 3), np.uint8)
        slots = {'external': (Path('verified'), (time.monotonic(), 'side:1', b'raw', rgb))}
        stop = threading.Event()
        ready = {}
        def fail(image):
            stop.set()
            raise ValueError('unavailable')
        with patch.object(parts, 'print') as output:
            parts.render_latest('external', slots, ready, threading.Lock(), stop, fail)
        self.assertEqual(json.loads(output.call_args.args[0])['kind'], 'overlay_error')
        self.assertEqual(ready, {})
        self.assertEqual(slots['external'][1][2], b'raw')
