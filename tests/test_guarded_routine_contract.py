import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class GuardedRoutineContractTest(unittest.TestCase):
    def test_firmware_defines_expected_routine_names(self):
        source = (ROOT / "src" / "control" / "guarded_routine.cpp").read_text()

        for name in {
            "inspect",
            "open_gripper_check",
            "stow",
            "center_target_dry_run",
            "soft_home_reference",
        }:
            with self.subTest(name=name):
                self.assertIn(f'"{name}"', source)
        self.assertIn("isSoftHomeAlias", source)
        self.assertIn("confirm-soft-home-reference", source)
        self.assertIn("software reference", source)
        self.assertIn("no encoder-grade homing", source)

    def test_routine_execute_is_explicitly_dry_run_only(self):
        dispatcher = (ROOT / "src" / "control" / "dispatcher.cpp").read_text()
        executor_header = (ROOT / "src" / "control" / "guarded_routine_executor.h").read_text()
        executor_source = (ROOT / "src" / "control" / "guarded_routine_executor.cpp").read_text()
        feedback_header = (ROOT / "src" / "control" / "hardware_feedback.h").read_text()
        feedback_source = (ROOT / "src" / "control" / "hardware_feedback.cpp").read_text()

        self.assertIn("ROUTINE_DRY_RUN", dispatcher)
        self.assertIn("ROUTINE_CONFIRM_REQ", dispatcher)
        self.assertIn("ROUTINE_PREFLIGHT_BLOCK", dispatcher)
        self.assertIn("GuardedRoutinePreflightResult::FEEDBACK_REQUIRED", dispatcher)
        self.assertIn("HardwareFeedback::baseYawReferenceReadyForRoutineExecution", dispatcher)
        self.assertIn("HardwareFeedback::routineBlockEventCode", dispatcher)
        self.assertIn("ROUTINE_EXECUTE_BLOCKED", dispatcher)
        self.assertIn("p=%s m=%s a=%u s=%u steps=%u", dispatcher)
        self.assertIn("requires operator confirmation code", dispatcher)
        self.assertIn("preflight blocked", dispatcher)
        self.assertIn("GuardedRoutineExecutor::begin", dispatcher)
        self.assertIn("GuardedRoutineExecutor::abort", dispatcher)
        self.assertIn("Routine executor blocked", dispatcher)
        self.assertIn("ROUTINE_ABORT", dispatcher)
        self.assertIn("ROUTINE_ABORT_IDLE", dispatcher)
        self.assertIn("#define MOTIONBRAIN_ROUTINE_EXECUTOR_ENABLED 0", executor_header)
        self.assertIn("routine executor disabled by firmware policy", executor_source)
        self.assertIn("GuardedRoutineExecutorState::RUNNING", executor_source)
        self.assertIn("enum class GuardedRoutineStepResult", executor_header)
        self.assertIn("PENDING = 0", executor_header)
        self.assertIn("SKIPPED", executor_header)
        self.assertIn("BLOCKED", executor_header)
        self.assertIn("buildStepJournal", executor_source)
        self.assertIn("buildPreparedSequence", executor_source)
        self.assertIn("buildMaterializationGate", executor_source)
        self.assertIn("sequence candidate ready; not applied to MotionSequence", executor_source)
        self.assertIn("motion sequence queue apply disabled by executor policy", executor_source)
        self.assertIn("AngleController::MIN_TARGET_DEGREES", executor_source)
        self.assertIn("PREPARE_READY", executor_header)
        self.assertIn("PREPARE_TOO_MANY_STEPS", executor_header)
        self.assertIn("PREPARE_INVALID_STEP", executor_header)
        self.assertIn("enum class GuardedRoutineMaterializeResult", executor_header)
        self.assertIn("MATERIALIZE_QUEUE_APPLY_DISABLED", executor_header)
        self.assertIn("motion step blocked by executor policy", executor_source)
        self.assertIn("motion step skipped before executor", executor_source)
        self.assertIn("routine executor timeout", executor_source)
        self.assertIn("ROUTINE_EXECUTOR_TIMEOUT", executor_source)
        self.assertIn("no active routine executor to abort", executor_source)
        self.assertIn("sequenceStarted(false)", executor_source)
        self.assertIn("executeImplemented() {\n  return false;", executor_source)
        self.assertIn("enum class HardwareFeedbackFault", feedback_header)
        self.assertIn("base_yaw_reference", feedback_source)
        self.assertIn("ROUTINE_FEEDBACK_BLOCK", feedback_source)
        self.assertIn("feedback_required", feedback_source)
        self.assertIn("base_yaw_reference feedback not installed", feedback_source)
        self.assertIn("physicalRoutineExecutionAllowed", feedback_source)
        self.assertIn("readyForRoutineExecution", feedback_source)
        for fault in [
            "NOT_INSTALLED",
            "DISCONNECTED",
            "STALE",
            "UNREFERENCED",
            "FAULTED",
            "NO_PROGRESS",
            "TIMEOUT",
            "OVERSHOOT",
        ]:
            with self.subTest(fault=fault):
                self.assertIn(fault, feedback_header)
                self.assertIn(fault, feedback_source)

    def test_http_and_serial_expose_same_routine_boundary(self):
        web_server = (ROOT / "src" / "network" / "web_server.cpp").read_text()
        serial = (ROOT / "src" / "input" / "serial_command.cpp").read_text()
        dispatcher_header = (ROOT / "src" / "control" / "dispatcher.h").read_text()
        dispatcher_source = (ROOT / "src" / "control" / "dispatcher.cpp").read_text()

        self.assertIn('server_.on("/routine", HTTP_POST', web_server)
        self.assertIn('server_.on("/routine", HTTP_GET', web_server)
        self.assertIn("CommandType::ROUTINE_DRY_RUN", web_server)
        self.assertIn("routineConfirmCode", web_server)
        self.assertIn("GuardedRoutine::evaluateExecutePreflight", web_server)
        self.assertIn("HardwareFeedback::appendStatusJson", web_server)
        self.assertIn("HardwareFeedback::baseYawReferenceReadyForRoutineExecution", web_server)
        self.assertIn("feedbackRequired", web_server)
        self.assertIn("feedbackReady", web_server)
        self.assertIn("feedbackBlockReason", web_server)
        self.assertIn("GuardedRoutineExecutor::appendReportJson", web_server)
        self.assertIn("GuardedRoutineExecutor::appendPolicyJson", web_server)
        self.assertIn("CommandType::ROUTINE_ABORT", web_server)
        self.assertIn("GuardedRoutineExecutor::describe(plan, executorAttempted", web_server)
        self.assertIn("noActiveSequence", web_server)
        self.assertIn('strcasecmp(cmdName, "routine")', serial)
        self.assertIn("routine dry-run <name>", serial)
        self.assertIn("routine status", serial)
        self.assertIn("routine diagnostics", serial)
        self.assertIn("routine abort", serial)
        self.assertIn("=== Routine Diagnostics ===", serial)
        self.assertIn("Last command: id=", serial)
        self.assertIn("confirm=confirm-inspect", serial)
        self.assertIn("soft_home_reference", serial)
        self.assertIn("appendRecoveryJson", web_server)
        self.assertIn("appendRuntimeDiagnosticsJson", web_server)
        self.assertIn("recovery", web_server)
        self.assertIn("diagnostics", web_server)
        self.assertIn("Routine Readiness", web_server)
        self.assertIn("ops-recovery", web_server)
        self.assertIn("ops-feedback", web_server)
        self.assertIn("ops-last-command", web_server)
        self.assertIn("updateRoutinePanel", web_server)
        self.assertIn("feedback.selectedClosureTarget", web_server)
        self.assertIn("feedback.readyForRoutineExecution", web_server)
        self.assertIn("baseYaw.fault", web_server)
        self.assertIn("freshnessThresholdMs", web_server)
        self.assertIn("SafetyMonitor::SENSOR_STALE_MS", web_server)
        self.assertIn("TeleopAdapter::LINK_TIMEOUT_MS", web_server)
        self.assertIn("dispatcher_->appendLastCommandJson", web_server)
        self.assertIn("DispatcherCommandAudit", dispatcher_header)
        self.assertIn("recordCommandResult", dispatcher_source)
        self.assertIn("lastCommand", dispatcher_source)
        self.assertIn("POST /command cmd=stop transitions FAULT toward IDLE", web_server)

    def test_message_interface_documents_dry_run_contract(self):
        doc = (ROOT / "MESSAGE_INTERFACE.md").read_text()

        self.assertIn("POST /routine?action=dry_run&name=inspect", doc)
        self.assertIn("POST /routine?action=run&name=inspect&confirm=confirm-inspect", doc)
        self.assertIn("executeImplemented", doc)
        self.assertIn("operatorConfirmation", doc)
        self.assertIn("executionPolicy", doc)
        self.assertIn('"executor"', doc)
        self.assertIn('"skeleton_disabled_by_default"', doc)
        self.assertIn('"abortSupported": true', doc)
        self.assertIn('"timeoutSupported": true', doc)
        self.assertIn('"stepJournal"', doc)
        self.assertIn('"preparedSequence"', doc)
        self.assertIn('"materialization"', doc)
        self.assertIn('"appliedToMotionSequence": false', doc)
        self.assertIn('"queueApplyAllowed": false', doc)
        self.assertIn('"preparedStepCount"', doc)
        self.assertIn('"preparedMotionCount"', doc)
        self.assertIn('"prepareResult": "ready"', doc)
        self.assertIn('"result": "skipped"', doc)
        self.assertIn("pending|skipped|blocked", doc)
        self.assertIn('"sequenceStarted": false', doc)
        self.assertIn("executor.result=disabled", doc)
        self.assertIn("POST /routine?action=abort", doc)
        self.assertIn("ROUTINE_ABORT_IDLE", doc)
        self.assertIn("state_not_armed", doc)
        self.assertIn("motion_blocked", doc)
        self.assertIn("sequence_active", doc)
        self.assertIn("ROUTINE_CONFIRM_REQ", doc)
        self.assertIn("ROUTINE_PREFLIGHT_BLOCK", doc)
        self.assertIn("ROUTINE_EXECUTE_BLOCKED", doc)
        self.assertIn("ROUTINE_EXECUTOR_TIMEOUT", doc)
        self.assertIn("p=<prepareResult>", doc)
        self.assertIn("m=<materializeResult>", doc)
        self.assertIn("queue_apply_disabled", doc)
        self.assertIn("system|safety|base_angle|teleop|routine", doc)
        self.assertIn("soft_home_reference", doc)
        self.assertIn("motion step", doc)
        self.assertIn("hard-stop seeking", doc)
        self.assertIn("dryRunOnly", doc)
        self.assertIn('"recovery"', doc)
        self.assertIn('"lastCommand"', doc)
        self.assertIn('"action": "stop"', doc)
        self.assertIn("Dispatcher audit", doc)
        self.assertIn("cmd=stop", doc)
        self.assertIn('"diagnostics"', doc)
        self.assertIn('"freshnessThresholdMs"', doc)
        self.assertIn('"sensor"', doc)
        self.assertIn('"teleop"', doc)
        self.assertIn('"safety"', doc)
        self.assertIn("docs/HARDWARE_FEEDBACK_GAP.md", doc)
        self.assertIn("base_yaw_reference", doc)
        self.assertIn("feedback_required", doc)
        self.assertIn("ROUTINE_FEEDBACK_BLOCK", doc)
        self.assertIn('"feedback"', doc)
        self.assertIn('"feedbackRequired": true', doc)
        self.assertIn('"feedbackReady": false', doc)
        self.assertIn('"fault": "not_installed"', doc)

    def test_hardware_feedback_gap_spec_blocks_physical_routine_execution(self):
        spec = (ROOT / "docs" / "HARDWARE_FEEDBACK_GAP.md").read_text()
        readme = (ROOT / "README.md").read_text()
        readme_en = (ROOT / "README.en.md").read_text()
        firmware_evidence = (ROOT / "docs" / "EMBEDDED_FIRMWARE_EVIDENCE.md").read_text()

        required_fragments = [
            "dryRunOnly=true",
            "executeImplemented=false",
            "queueApplyAllowed=false",
            "routine_execute_disabled_by_bridge_policy",
            "soft_home_reference",
            "MotionSequence::addCommand",
            "MotionSequence::addBaseAngleCommand",
            "MotionSequence::run",
            "base_yaw_reference",
            "selectedClosureTarget",
            "physicalRoutineExecutionAllowed",
            "feedback_required",
            "ROUTINE_FEEDBACK_BLOCK",
            "stale",
            "disconnected",
            "unreferenced",
            "no-progress",
            "timeout",
            "overshoot",
            "current sensing",
            "limit switch",
            "encoder",
            "The executor should stay disabled through steps 1-6.",
            "Current Firmware Scaffold",
            "GET /status",
            "GET /routine",
            "routine command responses",
            "not_installed",
        ]
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, spec)

        self.assertIn("docs/HARDWARE_FEEDBACK_GAP.md", readme)
        self.assertIn("docs/HARDWARE_FEEDBACK_GAP.md", readme_en)
        self.assertIn("docs/HARDWARE_FEEDBACK_GAP.md", firmware_evidence)


if __name__ == "__main__":
    unittest.main()
