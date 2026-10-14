"""Synthetic evidence only: no camera, ROS, robot, Pi, or model training."""
import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.schema import SCHEMA_VERSION, validate_transition


def make_transition():
    """Document the complete recording contract with synthetic clock provenance."""
    def observation(index, time):
        return {
            "observationId": f"obs-{index}", "sourceFrameId": ("a" * 32 + f":{index * 1000000}"),
            "cameraBootId": "a" * 32, "capturedUs": index * 1000000,
            "sourceClockDomain": "camera_boot_us", "sourceTimestampSemantics": "first_dma_buffer",
            "sourceTimeKnown": True, "sourceMetadataValid": True,
            "sourceAgeMs": 40, "sourceFresh": True, "held": False,
            "detectionSourceFrameId": ("a" * 32 + f":{index * 1000000}"), "imageSha256": "a" * 64,
            "image": {"artifactRef": f"images/{index}.jpg", "sha256": "a" * 64},
            "hostClockId": "host-a", "receivedMonotonic": time + .03,
            "inferenceCompletedMonotonic": time + .04,
            "captureHostBoundsBasis": "synthetic_bounded_clock_mapping",
            "captureHostBoundsCalibrated": True, "captureHostBoundsUncertaintySeconds": 0.001,
            "captureHostBoundsEvidenceRef": "synthetic/clock",
            "captureHostBounds": {"clockId": "host-a", "earliest": time, "latest": time + .01},
            "m4": {"motor": 4, "sensor": "AS5600", "measured": True, "angleDegrees": 4.5,
                   "sampleAgeMs": 5, "evidenceRef": "synthetic/m4",
                   "sampledHostBounds": {"clockId": "host-a", "earliest": time + .025, "latest": time + .026,
                                         "method": "synthetic", "evidenceRef": "synthetic/clock"}},
        }
    request = {"bootId": "mcu-a", "commandId": "producer-a:1", "kind": "pulse", "motor": 1,
               "pwm": 30, "durationMs": 100, "targetDegrees": 0, "percent": 0}
    record = {**request, "requestedPwm": 30, "ordinal": 1, "state": "completed",
              "reason": "duration_elapsed", "startedAtMs": 1000, "endedAtMs": 1100,
              "elapsedMs": 100, "overrunMs": 0, "outputOff": True, "targetReached": False}
    del record["pwm"]
    return {
        "schemaVersion": SCHEMA_VERSION, "transitionId": "transition-1", "sessionId": "session-1",
        "episodeId": "episode-1", "metadata": {
            "robotId": "synthetic-robot", "configHash": "b" * 64,
            "calibrationHash": "c" * 64, "firmwareVersion": "synthetic-fw-v1",
            "hostVersion": "synthetic-host-v1", "detectorVersion": "synthetic-detector-v1"},
        "qualityPolicy": {"maxSourceAgeMs": 100, "maxM4AgeMs": 20, "minimumSettlingMs": 50},
        "before": observation(1, 9), "after": observation(2, 11),
        "action": {"request": request, "record": record,
                   "startHostBounds": {"clockId": "host-a", "earliest": 10, "latest": 10.01,
                                       "method": "synthetic", "evidenceRef": "synthetic/clock"},
                   "outputOffHostBounds": {"clockId": "host-a", "earliest": 10.1, "latest": 10.11,
                                           "method": "synthetic", "evidenceRef": "synthetic/clock"},
                   "outputSequenceBounds": {"first": 7, "last": 8, "evidenceRef": "synthetic/trace-window"},
                   "outputLog": {"bootId": "mcu-a", "clockDomain": "controller_boot_millis",
                                 "evidence": "driver_write", "dropped": 0, "entries": [
                                     {"sequence": 7, "atMs": 1000, "actionOrdinal": 1, "motor": 1, "pwm": 30, "reason": "driver_write"},
                                     {"sequence": 8, "atMs": 1100, "actionOrdinal": 1, "motor": 1, "pwm": 0, "reason": "forced_off"}]}},
        "outcome": {"taskSuccess": None},
    }


class ActionTransitionSchemaTest(unittest.TestCase):
    def test_complete_synthetic_contract_is_eligible_and_input_is_unchanged(self):
        value = make_transition()
        original = copy.deepcopy(value)
        report = validate_transition(value)
        self.assertEqual(report, {"valid": True, "schemaErrors": [], "trainingEligible": True, "exclusionReasons": []})
        self.assertEqual(value, original)
        self.assertEqual(validate_transition(json.loads(json.dumps(value))), report)

    def test_real_service_observation_shape_is_quarantined_until_clock_evidence_exists(self):
        from unittest.mock import patch
        from tools import motionbrain_perception_service as service
        from motionbrain_ros_bridge.vision_detection import DetectionConfig
        from tests.test_perception_service import capture_headers

        value = make_transition()
        state = service.PerceptionState("http://synthetic-camera", DetectionConfig(), stale_seconds=1)
        for name, index, start in (("before", 1, 9), ("after", 2, 11)):
            with patch.object(service, "fetch_capture", return_value=(b"synthetic-jpeg", "image/jpeg", capture_headers(index * 1000000))), \
                 patch.object(service, "detect_frame", return_value={"available": True, "detected": True, "label": "cup"}), \
                 patch.object(service.time, "monotonic", side_effect=[start, start + .02, start + .05]):
                state.run_once()
            with patch.object(service.time, "monotonic", return_value=start + .05):
                observed = state.observation_payload("a" * 32 + f":{index * 1000000}")
            observed["image"] = {"artifactRef": f"synthetic/{index}.jpg", "sha256": observed["imageSha256"]}
            observed["m4"] = value[name]["m4"]
            observed["m4"]["sampledHostBounds"].update(clockId=observed["hostClockId"], earliest=start + .015, latest=start + .016)
            value[name] = observed
        for field in ("startHostBounds", "outputOffHostBounds"):
            value["action"][field]["clockId"] = value["before"]["hostClockId"]
        report = validate_transition(value)
        self.assertTrue(report["valid"], report)
        self.assertFalse(report["trainingEligible"])
        self.assertIn("after:source_clock_bounds_unverified", report["exclusionReasons"])
        for name in ("before", "after"):
            value[name].update(captureHostBoundsCalibrated=True,
                               captureHostBoundsUncertaintySeconds=.001,
                               captureHostBoundsEvidenceRef="synthetic/calibration")
        report = validate_transition(value)
        self.assertTrue(report["trainingEligible"], report)

    def test_known_cancellation_or_task_failure_is_useful_dynamics_data(self):
        for state, reason in (("cancelled", "operator_override"), ("failed", "safety_blocked"),
                              ("timed_out", "deadline_exceeded")):
            value = make_transition()
            value["action"]["record"].update(state=state, reason=reason)
            if state == "timed_out":
                value["action"]["record"].update(endedAtMs=1101, elapsedMs=101, overrunMs=1)
                value["action"]["outputLog"]["entries"][-1]["atMs"] = 1101
            value["outcome"] = {"taskSuccess": False, "labelSource": "human_video_review", "evidenceRef": "video/1"}
            self.assertTrue(validate_transition(value)["trainingEligible"], state)

    def test_quality_faults_are_retained_but_excluded(self):
        cases = [
            (("before", "held"), True, "before:stale_or_held"),
            (("after", "sourceFresh"), False, "after:stale_or_held"),
            (("before", "sourceTimeKnown"), False, "before:source_time_unknown"),
            (("before", "sourceAgeMs"), 101, "before:source_age_exceeded"),
            (("before", "m4", "sampleAgeMs"), 21, "before:m4_stale"),
            (("before", "detectionSourceFrameId"), "other", "before:detection_frame_mismatch"),
            (("after", "cameraBootId"), "reboot", "camera_reboot_between_observations"),
            (("after", "sourceFrameId"), ("a" * 32 + ":1000000"), "reused_observation"),
            (("action", "record", "state"), "outcome_unknown", "action_not_known_terminal"),
            (("action", "record", "state"), "running", "action_not_known_terminal"),
            (("action", "record", "outputOff"), False, "output_off_unconfirmed"),
            (("action", "outputLog", "dropped"), 1, "output_history_dropped"),
            (("action", "outputLog", "bootId"), "reboot", "output_boot_mismatch"),
            (("before", "image", "sha256"), "b" * 64, "before:image_hash_mismatch"),
        ]
        for path, replacement, reason in cases:
            with self.subTest(path=path, replacement=replacement):
                value = make_transition()
                parent = value
                for key in path[:-1]:
                    parent = parent[key]
                parent[path[-1]] = replacement
                original = copy.deepcopy(value)
                report = validate_transition(value)
                self.assertFalse(report["trainingEligible"])
                self.assertIn(reason, report["exclusionReasons"])
                self.assertEqual(original, value)

    def test_after_receipt_cannot_substitute_for_after_capture(self):
        value = make_transition()
        value["after"]["captureHostBounds"].update(earliest=9.9, latest=10.2)
        self.assertIn("after_not_proven_after_output_off_and_settling", validate_transition(value)["exclusionReasons"])

    def test_uncertain_before_overlapping_start_is_excluded(self):
        value = make_transition()
        value["before"]["captureHostBounds"]["latest"] = 10
        self.assertIn("before_not_proven_before_action", validate_transition(value)["exclusionReasons"])

    def test_unrelated_host_clock_cannot_be_numerically_compared(self):
        value = make_transition()
        value["action"]["startHostBounds"]["clockId"] = "different-host-boot"
        self.assertIn("host_clock_mismatch", validate_transition(value)["exclusionReasons"])

    def test_clock_mapping_assumption_is_not_training_evidence(self):
        value = make_transition()
        value["after"]["captureHostBoundsCalibrated"] = False
        value["after"]["captureHostBoundsUncertaintySeconds"] = None
        report = validate_transition(value)
        self.assertTrue(report["valid"])
        self.assertIn("after:source_clock_bounds_unverified", report["exclusionReasons"])
        value = make_transition()
        value["after"]["captureHostBoundsUncertaintySeconds"] = 1.0
        self.assertIn("after_not_proven_after_output_off_and_settling", validate_transition(value)["exclusionReasons"])

    def test_source_age_cannot_understate_capture_to_inference_latency(self):
        value = make_transition()
        value["before"]["sourceAgeMs"] = 0
        self.assertIn("before:source_age_understated", validate_transition(value)["exclusionReasons"])
        value = make_transition()
        value["qualityPolicy"]["maxSourceAgeMs"] = 40
        self.assertIn("before:source_age_exceeded", validate_transition(value)["exclusionReasons"])
        value["before"]["captureHostBoundsUncertaintySeconds"] = 0
        value["after"]["captureHostBoundsUncertaintySeconds"] = 0
        self.assertTrue(validate_transition(value)["trainingEligible"])

    def test_millis_wrap_is_supported_only_with_consistent_trace(self):
        value = make_transition()
        value["action"]["record"].update(startedAtMs=2**32 - 50, endedAtMs=50)
        entries = value["action"]["outputLog"]["entries"]
        entries[0]["atMs"], entries[1]["atMs"] = 2**32 - 50, 50
        self.assertTrue(validate_transition(value)["trainingEligible"])
        entries[0]["atMs"] -= 1
        self.assertIn("output_outside_action_interval", validate_transition(value)["exclusionReasons"])

    def test_trace_gaps_and_missing_terminal_zero_are_excluded(self):
        value = make_transition()
        value["action"]["outputLog"]["entries"][1]["sequence"] = 9
        self.assertIn("output_sequence_gap", validate_transition(value)["exclusionReasons"])
        value = make_transition()
        value["action"]["outputLog"]["entries"].pop()
        self.assertIn("terminal_zero_write_missing", validate_transition(value)["exclusionReasons"])
        value = make_transition()
        value["action"]["outputLog"]["entries"][0]["actionOrdinal"] = 2
        self.assertIn("output_action_mismatch", validate_transition(value)["exclusionReasons"])

    def test_source_identity_must_match_boot_and_capture_timestamp(self):
        value = make_transition()
        value["before"]["capturedUs"] = 0
        self.assertFalse(validate_transition(value)["valid"])
        value = make_transition()
        value["before"]["cameraBootId"] = "cam-a"
        self.assertFalse(validate_transition(value)["valid"])
        value = make_transition()
        value["before"]["capturedUs"] += 1
        self.assertIn("before:source_identity_mismatch", validate_transition(value)["exclusionReasons"])

    def test_terminal_overrun_matches_elapsed_and_completed_cannot_overrun(self):
        value = make_transition()
        value["action"]["record"]["overrunMs"] = 1
        self.assertIn("action_overrun_mismatch", validate_transition(value)["exclusionReasons"])
        value = make_transition()
        record = value["action"]["record"]
        record.update(endedAtMs=1110, elapsedMs=110, overrunMs=10)
        self.assertIn("completed_action_overran_deadline", validate_transition(value)["exclusionReasons"])
        record.update(state="timed_out", reason="deadline_exceeded")
        self.assertTrue(validate_transition(value)["trainingEligible"])

    def test_completed_pulse_cannot_end_before_deadline(self):
        value = make_transition()
        value["action"]["record"].update(endedAtMs=1050, elapsedMs=50)
        value["action"]["outputLog"]["entries"][-1]["atMs"] = 1050
        original = copy.deepcopy(value)
        report = validate_transition(value)
        self.assertTrue(report["valid"], report)
        self.assertFalse(report["trainingEligible"])
        self.assertIn("action_terminal_state_inconsistent", report["exclusionReasons"])
        self.assertEqual(value, original)

    def test_terminal_reason_and_target_flag_must_match_executor(self):
        cases = [
            {"reason": "target_reached"},
            {"targetReached": True},
            {"state": "failed", "reason": "operator_override"},
            {"state": "failed", "reason": "controller_failed"},  # only shoulder has a controller
            {"state": "failed", "reason": "start_failed", "targetReached": True},
            {"state": "cancelled", "reason": "duration_elapsed"},
            {"state": "cancelled", "reason": "cancelled", "targetReached": True},
            {"state": "timed_out", "reason": "deadline_exceeded"},  # exact pulse deadline completes
        ]
        for replacement in cases:
            with self.subTest(replacement=replacement):
                value = make_transition()
                value["action"]["record"].update(replacement)
                report = validate_transition(value)
                self.assertTrue(report["valid"], report)
                self.assertIn("action_terminal_state_inconsistent", report["exclusionReasons"])

    def test_shoulder_target_reached_may_timeout_during_stop(self):
        value = make_transition()
        action = value["action"]
        action["request"].update(kind="shoulder", motor=4, pwm=0, percent=30, targetDegrees=5)
        action["record"].update(kind="shoulder", motor=4, requestedPwm=0, percent=30, targetDegrees=5,
                                reason="target_reached", targetReached=True)
        for entry in action["outputLog"]["entries"]:
            entry["motor"] = 4
        self.assertTrue(validate_transition(value)["trainingEligible"])
        action["record"].update(state="timed_out", reason="deadline_exceeded", endedAtMs=1101,
                                elapsedMs=101, overrunMs=1)
        action["outputLog"]["entries"][-1]["atMs"] = 1101
        self.assertTrue(validate_transition(value)["trainingEligible"])
        action["record"].update(endedAtMs=1100, elapsedMs=100, overrunMs=0)
        action["outputLog"]["entries"][-1]["atMs"] = 1100
        self.assertIn("action_terminal_state_inconsistent", validate_transition(value)["exclusionReasons"])
        # The controller may also hit its deadline without reaching the target.
        action["record"]["targetReached"] = False
        self.assertTrue(validate_transition(value)["trainingEligible"])

    def test_nonfinite_nested_data_and_boolean_numbers_are_rejected(self):
        for bad in (float("nan"), float("inf"), -float("inf"), 10**1000):
            value = make_transition()
            value["before"]["extraDetectorData"] = {"untrustedScore": bad}
            self.assertFalse(validate_transition(value)["valid"])
        value = make_transition()
        value["action"]["record"]["motor"] = True
        self.assertFalse(validate_transition(value)["valid"])

    def test_missing_required_sections_do_not_get_approving_defaults(self):
        for key in make_transition():
            value = make_transition()
            del value[key]
            self.assertFalse(validate_transition(value)["valid"], key)
        for malformed in (None, [], 1, {}, {"before": "wrong", "action": []}):
            self.assertFalse(validate_transition(malformed)["valid"])

    def test_m4_sample_must_be_new_and_its_age_conservative(self):
        value = make_transition()
        value["after"]["m4"]["sampledHostBounds"].update(earliest=9, latest=9.01)
        reasons = validate_transition(value)["exclusionReasons"]
        self.assertIn("after:m4_not_proven_after_output_off", reasons)
        self.assertIn("after:m4_sample_age_understated", reasons)

    def test_metadata_and_measured_sensor_provenance_required(self):
        value = make_transition()
        del value["metadata"]["calibrationHash"]
        self.assertFalse(validate_transition(value)["valid"])
        value = make_transition()
        value["before"]["m4"]["measured"] = False
        self.assertFalse(validate_transition(value)["valid"])
        value = make_transition()
        del value["action"]["startHostBounds"]["method"]
        self.assertFalse(validate_transition(value)["valid"])

    def test_requested_action_and_accepted_record_must_match(self):
        value = make_transition()
        value["action"]["request"]["pwm"] = 31
        self.assertFalse(validate_transition(value)["valid"])

    def test_task_success_requires_external_evidence_and_is_not_output_off(self):
        value = make_transition()
        value["outcome"] = {"taskSuccess": True}
        self.assertFalse(validate_transition(value)["valid"])
        value["outcome"].update(labelSource="outputOff", evidenceRef="status/1")
        self.assertFalse(validate_transition(value)["valid"])
        value["outcome"].update(labelSource="human_video_review", evidenceRef="video/1")
        self.assertTrue(validate_transition(value)["trainingEligible"])


if __name__ == "__main__":
    unittest.main()
