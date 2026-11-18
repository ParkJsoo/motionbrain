import contextlib
import io
import unittest
from unittest.mock import patch

from tools import vision_host_mvp as host


class ManualNudgeTest(unittest.TestCase):
    def test_interrupted_nudge_stops_without_starting_again_in_the_capture_loop(self):
        with patch("sys.argv", ["vision_host_mvp.py", "--camera-url", "http://camera"]):
            args = host.parse_args()
        args.enable_align_action = True
        args.align_mode = "nudge"
        args.cooldown = args.interval = 0
        args.once = False
        state = {"state": "ARMED"}
        snapshot = {"schemaVersion": "motionbrain.manual_control_leases.v1", "bootId": "c" * 32,
                    "motors": [{"motor": 5, "generation": 1, "active": False}]}
        detected = {"detected": True, "alignment": "LEFT"}
        for start, stop in [
            (TimeoutError(), {"success": True}),
            ({"success": False}, {"success": True}),
            ({"success": True}, TimeoutError()),
            ({"success": True}, {"success": False}),
        ]:
            with (
                self.subTest(start=start, stop=stop),
                patch.object(host, "fetch_json", side_effect=[state, snapshot, state, snapshot]),
                patch.object(host, "fetch_bytes_with_retries", return_value=b"frame"),
                patch.object(host, "detect_colored_target", return_value=detected),
                patch.object(host, "post_motionbrain", side_effect=[start]) as send_start,
                patch.object(host, "post_motionbrain_with_retries", side_effect=[stop]) as send_stop,
                patch.object(host.time, "sleep"),
                contextlib.redirect_stdout(io.StringIO()),
                contextlib.redirect_stderr(io.StringIO()),
            ):
                self.assertEqual(host.run(args), 1)
                self.assertEqual(send_start.call_count, 1)
                self.assertIn("lease=start", send_start.call_args.args[1])
                self.assertEqual(send_stop.call_count, 1)
                self.assertEqual(send_stop.call_args.args[1], "/joint?joint=base&action=stop")
