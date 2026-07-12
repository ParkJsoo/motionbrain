import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MISSION_PACKAGE = ROOT / "ros2_ws" / "src" / "motionbrain_mission"


class PolicyProposalNodeContractTest(unittest.TestCase):
    def test_console_script_is_registered(self):
        setup_text = (MISSION_PACKAGE / "setup.py").read_text(encoding="utf-8")

        self.assertIn(
            "motionbrain_policy_proposal_node = "
            "motionbrain_mission.policy_proposal_node:main",
            setup_text,
        )

    def test_policy_node_has_no_http_or_controller_token_path(self):
        node_text = (
            MISSION_PACKAGE
            / "motionbrain_mission"
            / "policy_proposal_node.py"
        ).read_text(encoding="utf-8")

        forbidden_fragments = [
            "urllib",
            "urlopen",
            "Request(",
            "MOTIONBRAIN_HTTP_TOKEN",
            "X-MotionBrain-Token",
            "post_motionbrain",
            "/joint",
            "/motor",
            "/routine?action=run",
        ]
        for fragment in forbidden_fragments:
            with self.subTest(fragment=fragment):
                self.assertNotIn(fragment, node_text)

    def test_policy_node_publishes_proposal_topics(self):
        node_text = (
            MISSION_PACKAGE
            / "motionbrain_mission"
            / "policy_proposal_node.py"
        ).read_text(encoding="utf-8")

        required_fragments = [
            'self.declare_parameter("proposal_topic", "/motionbrain/policy_proposal_typed")',
            'self.declare_parameter("proposal_json_topic", "/motionbrain/policy_proposal")',
            'self.declare_parameter("instruction_topic", "/motionbrain/policy_instruction")',
            "PolicyProposalMsg",
            "propose_policy_action",
        ]
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, node_text)


if __name__ == "__main__":
    unittest.main()
