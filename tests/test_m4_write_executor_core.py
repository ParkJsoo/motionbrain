import unittest

from tests.test_m4_write_contract import armed_status
from motionbrain_ros_bridge.m4_write_contract import ros_rad_from_sensor_deg
from motionbrain_ros_bridge.m4_write_executor_core import M4WriteExecutorCore


def proposal(command_id="proposal-1", target=250.0):
    return {
        "commandId": command_id,
        "joint": "shoulder_pitch_joint",
        "targetPositionRad": ros_rad_from_sensor_deg(target),
        "timeoutMs": 10000,
        "forwarded": False,
        "operatorConfirmationRequired": True,
    }


class M4WriteExecutorCoreTest(unittest.TestCase):
    def test_confirm_forwards_exactly_once(self):
        core = M4WriteExecutorCore()
        core.accept_proposal(proposal())
        paths = []
        result = core.confirm("proposal-1", armed_status, lambda path: paths.append(path) or {"success": True})
        self.assertTrue(result["success"])
        self.assertTrue(result["forwarded"])
        self.assertEqual(1, len(paths))
        self.assertIn("degrees=250.000000", paths[0])
        duplicate = core.confirm("proposal-1", armed_status, lambda path: paths.append(path) or {"success": True})
        self.assertEqual("proposal_already_consumed", duplicate["reason"])
        self.assertEqual(1, len(paths))

    def test_idle_and_stale_fail_closed_without_post(self):
        for mutation, reason in ((lambda s: s.update(state="IDLE"), "state_not_armed"),
                                 (lambda s: s["shoulderAngle"].update(sensorFresh=False), "sensor_stale")):
            core = M4WriteExecutorCore()
            core.accept_proposal(proposal())
            status = armed_status()
            mutation(status)
            paths = []
            result = core.confirm("proposal-1", lambda: status, lambda path: paths.append(path) or {})
            self.assertEqual(reason, result["reason"])
            self.assertFalse(result["forwarded"])
            self.assertEqual([], paths)

    def test_invalid_boundary_is_rejected(self):
        core = M4WriteExecutorCore()
        bad = proposal()
        bad["forwarded"] = True
        with self.assertRaisesRegex(ValueError, "invalid_proposal_boundary"):
            core.accept_proposal(bad)

# These tests exercise host software with callbacks, virtual time and a local
# journal. They neither contact a controller nor establish physical execution.
class M4WriteExecutorReliabilityTest(unittest.TestCase):
    def make_core(self, **kwargs):
        core = M4WriteExecutorCore(**kwargs)
        self.addCleanup(core.close)
        return core

    def test_same_proposal_reception_does_not_reexecute(self):
        core = self.make_core()
        core.accept_proposal(proposal())
        calls = []
        first = core.confirm("proposal-1", armed_status, lambda path: calls.append(path) or {"success": True})
        replay = core.accept_proposal(proposal())
        self.assertEqual(first, replay)
        duplicate = core.confirm("proposal-1", armed_status, lambda path: calls.append(path) or {})
        self.assertEqual("proposal_already_consumed", duplicate["reason"])
        self.assertEqual("accepted", duplicate["executionState"])
        self.assertEqual(1, len(calls))
        self.assertEqual("accepted", core.get_result("proposal-1")["executionState"])

    def test_duplicate_does_not_refresh_deadline(self):
        now = [0.0]
        core = self.make_core(ttl_seconds=20, clock=lambda: now[0])
        core.accept_proposal(proposal())
        now[0] = 19.0
        core.accept_proposal(proposal())
        now[0] = 20.0
        calls = []
        result = core.confirm("proposal-1", armed_status, lambda path: calls.append(path) or {})
        self.assertEqual("proposal_expired", result["reason"])
        self.assertEqual([], calls)
        core.accept_proposal(proposal())
        self.assertEqual("proposal_expired", core.get_result("proposal-1")["reason"])

    def test_deadline_rechecked_after_status_fetch(self):
        now = [0.0]
        core = self.make_core(ttl_seconds=20, clock=lambda: now[0])
        core.accept_proposal(proposal())
        def fetch():
            now[0] = 20.0
            return armed_status()
        calls = []
        result = core.confirm("proposal-1", fetch, lambda path: calls.append(path) or {})
        self.assertEqual("proposal_expired", result["reason"])
        self.assertEqual([], calls)

    def test_same_id_different_action_rejected_without_overwrite(self):
        core = self.make_core()
        core.accept_proposal(proposal())
        for changed in (proposal(target=249), dict(proposal(), timeoutMs=9999)):
            with self.assertRaisesRegex(ValueError, "command_id_conflict"):
                core.accept_proposal(changed)
        self.assertAlmostEqual(core.journal.get("proposal-1")["request"]["targetPositionRad"],
                               proposal()["targetPositionRad"])

    def test_invalid_values_fail_before_record_or_callback(self):
        core = self.make_core()
        for target in (float("nan"), float("inf"), -float("inf"), True, "250partial", None):
            with self.subTest(target=target), self.assertRaises(ValueError):
                core.accept_proposal(dict(proposal(), targetPositionRad=target))
        for timeout in (True, 500.5, float("inf"), "1000.0", "1000ms", 499, 10001):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                core.accept_proposal(dict(proposal(), timeoutMs=timeout))
        for command_id in ("", "x" * 129, "space id", "\n", 123):
            with self.subTest(command_id=command_id), self.assertRaises(ValueError):
                core.accept_proposal(dict(proposal(), commandId=command_id))
        self.assertIsNone(core.get_result("proposal-1"))

    def test_post_response_loss_is_unknown_and_blocks_new_commands(self):
        core = self.make_core()
        core.accept_proposal(proposal())
        calls = []
        def lost(path):
            calls.append(path)
            raise TimeoutError("request accepted, response lost")
        result = core.confirm("proposal-1", armed_status, lost)
        self.assertTrue(result["forwarded"])
        self.assertTrue(result["dispatchAttempted"])
        self.assertFalse(result["acknowledged"])
        self.assertEqual("outcome_unknown", result["executionState"])
        core.accept_proposal(proposal())
        core.confirm("proposal-1", armed_status, lost)
        core.accept_proposal(proposal(command_id="proposal-2"))
        blocked = core.confirm("proposal-2", armed_status, lost)
        self.assertEqual("unresolved_outcome_unknown", blocked["reason"])
        self.assertEqual(1, len(calls))

    def test_fetch_failure_is_not_sent(self):
        core = self.make_core()
        core.accept_proposal(proposal())
        def unavailable():
            raise TimeoutError("status unavailable")
        calls = []
        result = core.confirm("proposal-1", unavailable, lambda path: calls.append(path) or {})
        self.assertEqual("not_sent", result["executionState"])
        self.assertFalse(result["dispatchAttempted"])
        self.assertEqual([], calls)

    def test_malformed_post_response_is_unknown(self):
        for response in ({}, {"success": "true"}, {"success": 1}, None,
                         {"success": True, "badNumber": float("nan")}):
            with self.subTest(response=response):
                core = self.make_core()
                core.accept_proposal(proposal())
                result = core.confirm("proposal-1", armed_status, lambda path: response)
                self.assertEqual("outcome_unknown", result["executionState"])
                self.assertTrue(result["dispatchAttempted"])

    def test_controller_rejection_is_acknowledged_but_not_success(self):
        import json
        core = self.make_core()
        core.accept_proposal(proposal())
        result = core.confirm("proposal-1", armed_status, lambda path: {"success": False, "error": "busy"})
        self.assertTrue(result["acknowledged"])
        self.assertFalse(result["success"])
        self.assertEqual("failed", result["executionState"])
        self.assertEqual("failed", json.loads(result["rawJson"])["executionState"])

    def test_capacity_fails_closed_without_evicting_results(self):
        core = self.make_core(max_records=1)
        core.accept_proposal(proposal())
        core.confirm("proposal-1", armed_status, lambda path: {"success": True})
        with self.assertRaisesRegex(ValueError, "journal_capacity_reached"):
            core.accept_proposal(proposal(command_id="proposal-2"))
        self.assertEqual("accepted", core.accept_proposal(proposal())["executionState"])

    def test_restart_cancels_pending_and_keeps_result(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "commands.sqlite")
            core = M4WriteExecutorCore(journal_path=path)
            core.accept_proposal(proposal())
            core.accept_proposal(proposal(command_id="done"))
            core.confirm("done", armed_status, lambda _: {"success": True})
            core.close()
            core = M4WriteExecutorCore(journal_path=path)
            try:
                self.assertEqual("canceled", core.get_result("proposal-1")["executionState"])
                self.assertEqual("accepted", core.get_result("done")["executionState"])
                calls = []
                core.accept_proposal(proposal())
                core.confirm("proposal-1", armed_status, lambda path: calls.append(path) or {})
                self.assertEqual([], calls)
            finally:
                core.close()

    def test_restart_after_dispatch_reservation_is_unknown(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "commands.sqlite")
            core = M4WriteExecutorCore(journal_path=path)
            core.accept_proposal(proposal())
            core.journal.reserve("proposal-1")
            core.journal.save("proposal-1", "dispatching", core._result(
                "proposal-1", "dispatch_reserved", "outcome_unknown", attempted=True))
            core.close()
            core = M4WriteExecutorCore(journal_path=path)
            try:
                self.assertEqual("outcome_unknown", core.get_result("proposal-1")["executionState"])
                core.accept_proposal(proposal(command_id="new"))
                result = core.confirm("new", armed_status, lambda _: self.fail("must not POST"))
                self.assertEqual("unresolved_outcome_unknown", result["reason"])
            finally:
                core.close()

    def test_second_live_owner_cannot_steal_pending(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "commands.sqlite")
            core = M4WriteExecutorCore(journal_path=path)
            try:
                core.accept_proposal(proposal())
                with self.assertRaisesRegex(ValueError, "journal_in_use"):
                    M4WriteExecutorCore(journal_path=path)
                self.assertEqual("pending", core.get_result("proposal-1")["ledgerState"])
            finally:
                core.close()

    def test_concurrent_confirm_dispatches_once_and_other_command_is_busy(self):
        import threading
        core = self.make_core()
        core.accept_proposal(proposal())
        core.accept_proposal(proposal(command_id="other"))
        entered = threading.Event()
        release = threading.Event()
        calls = []
        def post(path):
            calls.append(path)
            entered.set()
            if not release.wait(3):
                raise TimeoutError("test synchronization failed")
            return {"success": True}
        thread = threading.Thread(target=lambda: core.confirm("proposal-1", armed_status, post))
        thread.start()
        try:
            self.assertTrue(entered.wait(3))
            duplicate = core.confirm("proposal-1", armed_status, post)
            busy = core.confirm("other", armed_status, post)
            self.assertEqual("proposal_already_consumed", duplicate["reason"])
            self.assertEqual("executor_busy", busy["reason"])
        finally:
            release.set()
            thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(1, len(calls))

    def test_journal_failure_before_dispatch_never_posts(self):
        from unittest.mock import patch
        core = self.make_core()
        core.accept_proposal(proposal())
        with patch.object(core.journal, "save", side_effect=OSError("disk full")):
            result = core.confirm("proposal-1", armed_status, lambda _: self.fail("must not POST"))
        self.assertEqual("not_sent", result["executionState"])
        self.assertEqual("journal_unavailable", result["reason"])
        result = core.confirm("proposal-1", armed_status, lambda _: self.fail("must stay blocked"))
        self.assertEqual("journal_unavailable", result["reason"])

    def test_source_clock_metadata_is_preserved_without_overwriting_on_replay(self):
        core = self.make_core()
        core.accept_proposal(dict(proposal(), createdAt={"sec": 100, "nanosec": 1}, createdClockDomain="ros"))
        core.accept_proposal(dict(proposal(), createdAt={"sec": 200, "nanosec": 2}, createdClockDomain="ros"))
        metadata = core.get_result("proposal-1")["proposalMetadata"]
        self.assertEqual({"sec": 100, "nanosec": 1}, metadata["createdAt"])
        self.assertEqual("ros", metadata["createdClockDomain"])


    def test_post_result_storage_failure_remains_unknown_after_restart(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "commands.sqlite")
            core = M4WriteExecutorCore(journal_path=path)
            core.accept_proposal(proposal())
            original_save = core.journal.save
            def disk_full_after_send(command_id, phase, result):
                if phase != "dispatching":
                    raise OSError("disk full")
                original_save(command_id, phase, result)
            calls = []
            with patch.object(core.journal, "save", side_effect=disk_full_after_send):
                result = core.confirm("proposal-1", armed_status,
                                      lambda path: calls.append(path) or {"success": True})
            self.assertEqual(1, len(calls))
            self.assertEqual("outcome_unknown", result["executionState"])
            self.assertFalse(result["acknowledged"])
            retry = core.confirm("proposal-1", armed_status, lambda _: self.fail("must not retry"))
            self.assertEqual("outcome_unknown", retry["executionState"])
            self.assertTrue(retry["dispatchAttempted"])
            query = core.get_result("proposal-1")
            self.assertEqual("outcome_unknown", query["executionState"])
            self.assertTrue(query["dispatchAttempted"])
            unknown_id = core.confirm("unseen-id", armed_status, lambda _: self.fail("must stay blocked"))
            self.assertEqual("outcome_unknown", unknown_id["executionState"])
            core.close()
            core = M4WriteExecutorCore(journal_path=path)
            try:
                self.assertEqual("outcome_unknown", core.get_result("proposal-1")["executionState"])
                core.accept_proposal(proposal())
                duplicate = core.confirm("proposal-1", armed_status, lambda _: self.fail("must not retry"))
                self.assertEqual("outcome_unknown", duplicate["executionState"])
            finally:
                core.close()

    def test_restart_during_status_validation_cancels_without_dispatch(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "commands.sqlite")
            core = M4WriteExecutorCore(journal_path=path)
            core.accept_proposal(proposal())
            core.journal.reserve("proposal-1")
            core.close()
            core = M4WriteExecutorCore(journal_path=path)
            try:
                result = core.get_result("proposal-1")
                self.assertEqual("canceled", result["executionState"])
                self.assertFalse(result["dispatchAttempted"])
            finally:
                core.close()


    def test_reserve_read_failure_preserves_unknown_for_consumed_command(self):
        from unittest.mock import patch
        core = self.make_core()
        core.accept_proposal(proposal())
        def lost(_):
            raise TimeoutError("response lost")
        core.confirm("proposal-1", armed_status, lost)
        with patch.object(core.journal, "reserve", side_effect=OSError("read failed")):
            result = core.confirm("proposal-1", armed_status, lambda _: self.fail("must not POST"))
        self.assertEqual("outcome_unknown", result["executionState"])
        self.assertTrue(result["dispatchAttempted"])

    def test_unreadable_history_is_unknown_not_not_sent(self):
        from unittest.mock import patch
        core = self.make_core()
        with patch.object(core.journal, "reserve", side_effect=OSError("read failed")):
            result = core.confirm("not-in-local-cache", armed_status, lambda _: self.fail("must not POST"))
        self.assertEqual("outcome_unknown", result["executionState"])
        self.assertFalse(result["dispatchAttempted"])
        self.assertEqual("outcome_unknown", core.get_result("not-in-local-cache")["executionState"])

    def test_result_read_failure_preserves_known_acknowledgment(self):
        from unittest.mock import patch
        core = self.make_core()
        core.accept_proposal(proposal())
        core.confirm("proposal-1", armed_status, lambda _: {"success": True})
        with patch.object(core.journal, "get", side_effect=OSError("read failed")):
            result = core.get_result("proposal-1")
        self.assertEqual("accepted", result["executionState"])
        self.assertTrue(result["acknowledged"])
        self.assertEqual("journal_unavailable", result["reason"])
        self.assertEqual("accepted", core.confirm(
            "proposal-1", armed_status, lambda _: self.fail("must not POST"))["executionState"])

    def test_result_read_failure_without_cache_is_unknown(self):
        from unittest.mock import patch
        core = self.make_core()
        with patch.object(core.journal, "get", side_effect=OSError("read failed")):
            result = core.get_result("not-in-local-cache")
        self.assertEqual("outcome_unknown", result["executionState"])
        self.assertEqual("journal_unavailable", result["reason"])


if __name__ == "__main__":
    unittest.main()
