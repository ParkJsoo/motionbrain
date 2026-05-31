import unittest

from motionbrain_ros_bridge.payload_utils import ALIGN_DEADBAND
from motionbrain_ros_bridge.payload_utils import as_bool
from motionbrain_ros_bridge.payload_utils import as_float
from motionbrain_ros_bridge.payload_utils import as_str
from motionbrain_ros_bridge.payload_utils import as_uint
from motionbrain_ros_bridge.payload_utils import classify_alignment
from motionbrain_ros_bridge.payload_utils import command_suggestion_for_alignment
from motionbrain_ros_bridge.payload_utils import compact_json
from motionbrain_ros_bridge.payload_utils import perception_detection_url
from motionbrain_ros_bridge.payload_utils import parse_light_action


class PayloadUtilsPackageTest(unittest.TestCase):
    def test_parse_light_action_accepts_raw_and_json_payloads(self):
        self.assertEqual("toggle", parse_light_action("toggle"))
        self.assertEqual("on", parse_light_action('{"action":"ON"}'))
        self.assertEqual("off", parse_light_action('{"action":" off "}'))

    def test_parse_light_action_rejects_invalid_payloads(self):
        self.assertIsNone(parse_light_action(""))
        self.assertIsNone(parse_light_action("blink"))
        self.assertIsNone(parse_light_action('{"action":"blink"}'))
        self.assertIsNone(parse_light_action('["toggle"]'))

    def test_alignment_classification_matches_dashboard_contract(self):
        self.assertEqual("LOST", classify_alignment(None))
        self.assertEqual("LEFT", classify_alignment(-(ALIGN_DEADBAND + 0.01)))
        self.assertEqual("CENTER", classify_alignment(-ALIGN_DEADBAND))
        self.assertEqual("CENTER", classify_alignment(ALIGN_DEADBAND))
        self.assertEqual("RIGHT", classify_alignment(ALIGN_DEADBAND + 0.01))

    def test_alignment_suggestion_contract(self):
        self.assertEqual("base_left", command_suggestion_for_alignment("LEFT"))
        self.assertEqual("base_right", command_suggestion_for_alignment("RIGHT"))
        self.assertEqual("hold", command_suggestion_for_alignment("CENTER"))
        self.assertEqual("none", command_suggestion_for_alignment("LOST"))

    def test_scalar_coercion_is_stable_for_typed_messages(self):
        self.assertTrue(as_bool("armed"))
        self.assertFalse(as_bool("stopped", default=True))
        self.assertTrue(as_bool(1))
        self.assertFalse(as_bool(0))
        self.assertAlmostEqual(1.25, as_float("1.25"))
        self.assertEqual(0.0, as_float("bad"))
        self.assertEqual(7, as_uint("7"))
        self.assertEqual(0, as_uint("-3"))
        self.assertEqual("UNKNOWN", as_str(None, "UNKNOWN"))

    def test_compact_json_is_sorted_and_minimal(self):
        self.assertEqual('{"a":1,"b":2}', compact_json({"b": 2, "a": 1}))

    def test_perception_detection_url_accepts_base_or_api_url(self):
        self.assertEqual("", perception_detection_url(""))
        self.assertEqual(
            "http://motionbrain-pi.local:8766/api/detection",
            perception_detection_url(" http://motionbrain-pi.local:8766 "),
        )
        self.assertEqual(
            "http://192.168.219.114:8766/api/detection",
            perception_detection_url("http://192.168.219.114:8766/api"),
        )
        self.assertEqual(
            "http://192.168.219.114:8766/api/detection",
            perception_detection_url("http://192.168.219.114:8766/api/detection"),
        )


if __name__ == "__main__":
    unittest.main()
