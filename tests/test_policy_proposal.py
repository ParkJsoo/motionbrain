import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "motionbrain_mission"))

from motionbrain_mission.policy_proposal import (  # noqa: E402
    ALIGN_LEFT_ACTION,
    ASK_OPERATOR_ACTION,
    CUP_GRASP_PLAN_ACTION,
    LIGHT_TOGGLE_ACTION,
    SAFE_HOLD_ACTION,
    PolicyConfig,
    PolicyDetectionSnapshot,
    PolicyGuardSnapshot,
    PolicyStatusSnapshot,
    propose_policy_action,
)


READY_STATUS = PolicyStatusSnapshot(
    available=True,
    state="ARMED",
    moving=False,
    faulted=False,
    base_active=False,
    safety_blocked=False,
    fault_latched=False,
)

READY_GUARD = PolicyGuardSnapshot(
    ready=True,
    reason="ready",
    status_fresh=True,
    detection_fresh=True,
)


class PolicyProposalTest(unittest.TestCase):
    def test_stale_status_holds_without_motion_candidate(self):
        guard = PolicyGuardSnapshot(
            ready=True,
            reason="ready",
            status_fresh=False,
            detection_fresh=True,
        )
        detection = PolicyDetectionSnapshot(available=True, detected=True, fresh=True)

        proposal = propose_policy_action(status=READY_STATUS, detection=detection, guard=guard)

        self.assertEqual(SAFE_HOLD_ACTION, proposal.action)
        self.assertFalse(proposal.physical_motion_candidate)
        self.assertIn("status_stale", proposal.preconditions)

    def test_unarmed_state_does_not_propose_base_nudge(self):
        status = PolicyStatusSnapshot(available=True, state="IDLE")
        detection = PolicyDetectionSnapshot(
            available=True,
            detected=True,
            fresh=True,
            alignment="LEFT",
            confidence=0.9,
        )

        proposal = propose_policy_action(status=status, detection=detection, guard=READY_GUARD)

        self.assertEqual(SAFE_HOLD_ACTION, proposal.action)
        self.assertFalse(proposal.physical_motion_candidate)
        self.assertIn("state_not_armed", proposal.preconditions)

    def test_held_detection_is_rejected(self):
        detection = PolicyDetectionSnapshot(
            available=True,
            detected=True,
            fresh=True,
            held=True,
            alignment="LEFT",
            confidence=0.9,
        )

        proposal = propose_policy_action(status=READY_STATUS, detection=detection, guard=READY_GUARD)

        self.assertEqual(SAFE_HOLD_ACTION, proposal.action)
        self.assertFalse(proposal.physical_motion_candidate)
        self.assertIn("held_detection", proposal.preconditions)

    def test_left_alignment_proposes_operator_confirmed_nudge(self):
        detection = PolicyDetectionSnapshot(
            available=True,
            detected=True,
            fresh=True,
            alignment="LEFT",
            confidence=0.8,
        )

        proposal = propose_policy_action(status=READY_STATUS, detection=detection, guard=READY_GUARD)

        self.assertEqual(ALIGN_LEFT_ACTION, proposal.action)
        self.assertTrue(proposal.requires_operator_confirm)
        self.assertTrue(proposal.physical_motion_candidate)
        self.assertIn("operator_confirm", proposal.preconditions)

    def test_motion_candidates_can_be_disabled(self):
        detection = PolicyDetectionSnapshot(
            available=True,
            detected=True,
            fresh=True,
            alignment="LEFT",
            confidence=0.8,
        )

        proposal = propose_policy_action(
            status=READY_STATUS,
            detection=detection,
            guard=READY_GUARD,
            config=PolicyConfig(allow_motion_candidates=False),
        )

        self.assertEqual(ASK_OPERATOR_ACTION, proposal.action)
        self.assertFalse(proposal.physical_motion_candidate)

    def test_cup_plan_requires_center_and_confidence(self):
        detection = PolicyDetectionSnapshot(
            available=True,
            detected=True,
            fresh=True,
            alignment="CENTER",
            label="cup",
            confidence=0.88,
        )

        proposal = propose_policy_action(
            instruction="plan cup grasp",
            status=READY_STATUS,
            detection=detection,
            guard=READY_GUARD,
        )

        self.assertEqual(CUP_GRASP_PLAN_ACTION, proposal.action)
        self.assertTrue(proposal.requires_operator_confirm)
        self.assertFalse(proposal.physical_motion_candidate)

    def test_cup_plan_low_confidence_asks_operator(self):
        detection = PolicyDetectionSnapshot(
            available=True,
            detected=True,
            fresh=True,
            alignment="CENTER",
            label="cup",
            confidence=0.2,
        )

        proposal = propose_policy_action(
            instruction="plan cup grasp",
            status=READY_STATUS,
            detection=detection,
            guard=READY_GUARD,
        )

        self.assertEqual(ASK_OPERATOR_ACTION, proposal.action)
        self.assertFalse(proposal.physical_motion_candidate)
        self.assertIn("confidence_below_threshold", proposal.preconditions)

    def test_light_toggle_is_non_motion_operator_confirm(self):
        status = PolicyStatusSnapshot(available=True, state="IDLE")
        guard = PolicyGuardSnapshot(
            ready=False,
            reason="detection_stale",
            status_fresh=True,
            detection_fresh=False,
        )
        detection = PolicyDetectionSnapshot(available=False, detected=False, fresh=False)

        proposal = propose_policy_action(
            instruction="toggle search light",
            status=status,
            detection=detection,
            guard=guard,
        )

        self.assertEqual(LIGHT_TOGGLE_ACTION, proposal.action)
        self.assertTrue(proposal.requires_operator_confirm)
        self.assertFalse(proposal.physical_motion_candidate)

    def test_json_shape_is_dashboard_friendly(self):
        detection = PolicyDetectionSnapshot(
            available=True,
            detected=True,
            fresh=True,
            alignment="LEFT",
            confidence=0.8,
        )

        proposal = propose_policy_action(
            instruction="align target",
            status=READY_STATUS,
            detection=detection,
            guard=READY_GUARD,
        )
        payload = proposal.to_dict()

        self.assertEqual(
            {
                "instruction",
                "action",
                "confidence",
                "reason",
                "requiresOperatorConfirm",
                "physicalMotionCandidate",
                "preconditions",
            },
            set(payload),
        )
        self.assertEqual("align target", payload["instruction"])
        self.assertTrue(payload["requiresOperatorConfirm"])
        self.assertTrue(payload["physicalMotionCandidate"])


if __name__ == "__main__":
    unittest.main()
