#!/usr/bin/env python3

import json
import os
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from motionbrain_msgs.action import GuardedRoutine
from motionbrain_msgs.msg import CameraDetection
from motionbrain_msgs.msg import LightCommand
from motionbrain_msgs.msg import LightResult
from motionbrain_msgs.msg import MotionEvent
from motionbrain_msgs.msg import MotionStatus
from motionbrain_msgs.msg import RoutineCommand
from motionbrain_msgs.msg import RoutineResult
from motionbrain_msgs.msg import RoutineStatus
from motionbrain_msgs.srv import GuardedRoutineCommand
from motionbrain_ros_bridge.lifecycle_status import LifecycleStatusPublisher
from motionbrain_ros_bridge.payload_utils import ALIGN_DEADBAND
from motionbrain_ros_bridge.payload_utils import as_bool
from motionbrain_ros_bridge.payload_utils import as_float
from motionbrain_ros_bridge.payload_utils import as_str
from motionbrain_ros_bridge.payload_utils import as_uint
from motionbrain_ros_bridge.payload_utils import compact_json
from motionbrain_ros_bridge.payload_utils import perception_detection_url
from motionbrain_ros_bridge.payload_utils import parse_light_action
from motionbrain_ros_bridge.payload_utils import parse_routine_command
from motionbrain_ros_bridge.vision_detection import detect_colored_target
from rclpy.action import ActionServer
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String


NETWORK_EXCEPTIONS = (urllib.error.URLError, TimeoutError, OSError)
POLL_EXCEPTIONS = NETWORK_EXCEPTIONS + (json.JSONDecodeError,)
PERCEPTION_EXCEPTIONS = POLL_EXCEPTIONS + (ValueError,)


def fetch_json(url: str, timeout: float) -> dict[str, Any]:
    request = urllib.request.Request(url)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def fetch_bytes(url: str, timeout: float) -> bytes:
    request = urllib.request.Request(url)
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


def post_motionbrain(base_url: str, path: str, timeout: float, token: str = "") -> dict[str, Any]:
    headers = {"X-MotionBrain": "1"}
    if token:
        headers["X-MotionBrain-Token"] = token
    request = urllib.request.Request(
        f"{base_url}{path}",
        method="POST",
        headers=headers,
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


class MotionBrainStatusNode(LifecycleNode):
    def __init__(self, autostart: bool | None = None) -> None:
        super().__init__("motionbrain_status_node")

        self.declare_parameter("motion_host", "192.168.4.1")
        self.declare_parameter("motion_port", 80)
        self.declare_parameter("camera_url", "")
        self.declare_parameter("perception_url", os.environ.get("MOTIONBRAIN_PERCEPTION_URL", ""))
        self.declare_parameter("detect_color", "red")
        self.declare_parameter("poll_interval", 1.0)
        self.declare_parameter("http_timeout", 2.0)
        self.declare_parameter("events_limit", 8)
        self.declare_parameter("http_token", os.environ.get("MOTIONBRAIN_HTTP_TOKEN", ""))
        self.declare_parameter("autostart", True if autostart is None else bool(autostart))

        self.motion_base_url = self._motion_base_url()
        self.lifecycle = LifecycleStatusPublisher(
            self,
            detail=f"unconfigured bridge for {self.motion_base_url}",
        )
        self._polling_active = False
        self._configured = False
        self.timer = None
        self.status_pub = None
        self.routine_pub = None
        self.events_pub = None
        self.detection_pub = None
        self.diagnostics_pub = None
        self.light_result_pub = None
        self.routine_result_pub = None
        self.status_typed_pub = None
        self.routine_typed_pub = None
        self.events_typed_pub = None
        self.detection_typed_pub = None
        self.light_result_typed_pub = None
        self.routine_result_typed_pub = None
        self.light_sub = None
        self.light_typed_sub = None
        self.routine_sub = None
        self.routine_typed_sub = None
        self.routine_service = None
        self.routine_action_server = None

        if bool(self.get_parameter("autostart").value):
            self.trigger_configure()
            self.trigger_activate()

    def on_configure(self, state: Any) -> TransitionCallbackReturn:
        del state
        try:
            self.motion_base_url = self._motion_base_url()
            self._create_configured_entities()
            self.lifecycle.mark_inactive(
                f"configured bridge for {self.motion_base_url}; waiting for activation"
            )
            self.get_logger().info(
                f"MotionBrain ROS2 bridge configured for {self.motion_base_url}; "
                "waiting for lifecycle activation"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.lifecycle.mark_error(f"configure failed: {exc}")
            self.get_logger().error(f"MotionBrain ROS2 bridge configure failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: Any) -> TransitionCallbackReturn:
        del state
        try:
            self.motion_base_url = self._motion_base_url()
            if not self._configured:
                self._create_configured_entities()
            self._create_poll_timer()
            self._polling_active = True
            self.lifecycle.mark_active(
                "polling status/routine/camera and serving routine command/action boundaries"
            )
            self._log_bridge_active()
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.lifecycle.mark_error(f"activate failed: {exc}")
            self.get_logger().error(f"MotionBrain ROS2 bridge activate failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._polling_active = False
        self._destroy_poll_timer()
        self.lifecycle.mark_inactive(f"polling stopped for {self.motion_base_url}")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._polling_active = False
        self._destroy_poll_timer()
        self._destroy_configured_entities()
        self.lifecycle.mark_inactive(f"unconfigured bridge for {self.motion_base_url}")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._polling_active = False
        self._destroy_poll_timer()
        self.lifecycle.mark_inactive(f"shutdown requested for {self.motion_base_url}")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: Any) -> TransitionCallbackReturn:
        del state
        self._polling_active = False
        self._destroy_poll_timer()
        self.lifecycle.mark_error(f"lifecycle error for {self.motion_base_url}")
        return TransitionCallbackReturn.SUCCESS

    def _create_configured_entities(self) -> None:
        if self._configured:
            return

        self.status_pub = self.create_publisher(String, "/motionbrain/status", 10)
        self.routine_pub = self.create_publisher(String, "/motionbrain/routine", 10)
        self.events_pub = self.create_publisher(String, "/motionbrain/events", 10)
        self.detection_pub = self.create_publisher(String, "/camera/detection", 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            "/motionbrain/diagnostics",
            10,
        )
        self.light_result_pub = self.create_publisher(String, "/motionbrain/light_result", 10)
        self.routine_result_pub = self.create_publisher(String, "/motionbrain/routine_result", 10)
        self.status_typed_pub = self.create_publisher(MotionStatus, "/motionbrain/status_typed", 10)
        self.routine_typed_pub = self.create_publisher(RoutineStatus, "/motionbrain/routine_typed", 10)
        self.events_typed_pub = self.create_publisher(MotionEvent, "/motionbrain/events_typed", 10)
        self.detection_typed_pub = self.create_publisher(
            CameraDetection,
            "/camera/detection_typed",
            10,
        )
        self.light_result_typed_pub = self.create_publisher(
            LightResult,
            "/motionbrain/light_result_typed",
            10,
        )
        self.routine_result_typed_pub = self.create_publisher(
            RoutineResult,
            "/motionbrain/routine_result_typed",
            10,
        )
        self.light_sub = self.create_subscription(
            String,
            "/motionbrain/light_cmd",
            self.handle_light_cmd,
            10,
        )
        self.light_typed_sub = self.create_subscription(
            LightCommand,
            "/motionbrain/light_cmd_typed",
            self.handle_light_cmd_typed,
            10,
        )
        self.routine_sub = self.create_subscription(
            String,
            "/motionbrain/routine_cmd",
            self.handle_routine_cmd,
            10,
        )
        self.routine_typed_sub = self.create_subscription(
            RoutineCommand,
            "/motionbrain/routine_cmd_typed",
            self.handle_routine_cmd_typed,
            10,
        )
        self.routine_service = self.create_service(
            GuardedRoutineCommand,
            "/motionbrain/routine_command",
            self.handle_routine_service,
        )
        self.routine_action_server = ActionServer(
            self,
            GuardedRoutine,
            "/motionbrain/guarded_routine",
            self.execute_routine_goal,
        )

        self._configured = True

    def _create_poll_timer(self) -> None:
        self._destroy_poll_timer()
        interval = float(self.get_parameter("poll_interval").value)
        self.timer = self.create_timer(max(interval, 0.1), self.poll_once)

    def _destroy_poll_timer(self) -> None:
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None

    def _destroy_configured_entities(self) -> None:
        for publisher_attr in [
            "status_pub",
            "routine_pub",
            "events_pub",
            "detection_pub",
            "diagnostics_pub",
            "light_result_pub",
            "routine_result_pub",
            "status_typed_pub",
            "routine_typed_pub",
            "events_typed_pub",
            "detection_typed_pub",
            "light_result_typed_pub",
            "routine_result_typed_pub",
        ]:
            publisher = getattr(self, publisher_attr)
            if publisher is not None:
                self.destroy_publisher(publisher)
                setattr(self, publisher_attr, None)

        for subscription_attr in [
            "light_sub",
            "light_typed_sub",
            "routine_sub",
            "routine_typed_sub",
        ]:
            subscription = getattr(self, subscription_attr)
            if subscription is not None:
                self.destroy_subscription(subscription)
                setattr(self, subscription_attr, None)

        if self.routine_service is not None:
            self.destroy_service(self.routine_service)
            self.routine_service = None

        if self.routine_action_server is not None:
            self.routine_action_server.destroy()
            self.routine_action_server = None

        self._configured = False

    def _log_bridge_active(self) -> None:
        self.get_logger().info(
            f"MotionBrain ROS2 bridge polling {self.motion_base_url}; "
            "topics: /motionbrain/status /motionbrain/status_typed "
            "/motionbrain/routine /motionbrain/routine_typed "
            "/motionbrain/events /motionbrain/events_typed "
            "/motionbrain/diagnostics "
            "/camera/detection /camera/detection_typed "
            "/motionbrain/light_cmd /motionbrain/light_cmd_typed "
            "/motionbrain/routine_cmd /motionbrain/routine_cmd_typed "
            "service: /motionbrain/routine_command "
            "action: /motionbrain/guarded_routine"
        )

    def _motion_base_url(self) -> str:
        host = str(self.get_parameter("motion_host").value)
        port = int(self.get_parameter("motion_port").value)
        return f"http://{host}:{port}"

    def _timeout(self) -> float:
        return float(self.get_parameter("http_timeout").value)

    def publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = compact_json(payload)
        publisher.publish(message)

    def diagnostic_value(self, key: str, value: Any) -> KeyValue:
        item = KeyValue()
        item.key = key
        item.value = as_str(value)
        return item

    def diagnostic_status(
        self,
        name: str,
        level: int,
        message: str,
        values: dict[str, Any],
        hardware_id: str = "motionbrain",
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = hardware_id
        status.level = level
        status.message = message
        status.values = [self.diagnostic_value(key, value) for key, value in values.items()]
        return status

    def publish_diagnostics(
        self,
        status_payload: dict[str, Any] | None,
        routine_payload: dict[str, Any] | None,
        detection_payload: dict[str, Any] | None,
    ) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [
            self.controller_diagnostic(status_payload),
            self.shoulder_feedback_diagnostic(status_payload),
            self.routine_diagnostic(routine_payload),
            self.feedback_diagnostic(routine_payload),
            self.teleop_sensor_diagnostic(routine_payload),
            self.camera_diagnostic(detection_payload),
        ]
        self.diagnostics_pub.publish(message)

    def controller_diagnostic(self, payload: dict[str, Any] | None) -> DiagnosticStatus:
        if payload is None:
            return self.diagnostic_status(
                "motionbrain/controller",
                DiagnosticStatus.ERROR,
                "status poll unavailable",
                {},
                "esp32_motion_controller",
            )

        sensor = payload.get("sensor")
        sensor = sensor if isinstance(sensor, dict) else {}
        recovery = payload.get("recovery")
        recovery = recovery if isinstance(recovery, dict) else {}
        last_command = payload.get("lastCommand")
        last_command = last_command if isinstance(last_command, dict) else {}

        state = as_str(payload.get("state"), "UNKNOWN")
        fault_latched = as_bool(sensor.get("faultLatched")) or state.upper() == "FAULT"
        motion_blocked = as_bool(sensor.get("blocked"))
        if fault_latched:
            level = DiagnosticStatus.ERROR
            text = "controller fault latched"
        elif motion_blocked:
            level = DiagnosticStatus.WARN
            text = "motion blocked by safety gate"
        else:
            level = DiagnosticStatus.OK
            text = "controller ready"

        return self.diagnostic_status(
            "motionbrain/controller",
            level,
            text,
            {
                "state": state,
                "motor_enabled": as_bool(payload.get("motorEnabled")),
                "block_reason": as_str(sensor.get("blockReason"), "NONE"),
                "fault_reason": as_str(sensor.get("faultReason"), "NONE"),
                "recovery_action": as_str(recovery.get("action"), "none"),
                "last_command_seen": as_bool(last_command.get("seen")),
                "last_command_success": as_bool(last_command.get("success")),
                "last_command_type": as_str(last_command.get("type")),
            },
            "esp32_motion_controller",
        )

    def shoulder_feedback_diagnostic(
        self,
        payload: dict[str, Any] | None,
    ) -> DiagnosticStatus:
        shoulder = payload.get("shoulderAngle") if isinstance(payload, dict) else None
        shoulder = shoulder if isinstance(shoulder, dict) else {}

        available = as_bool(shoulder.get("available"))
        connected = as_bool(shoulder.get("sensorConnected"))
        fresh = as_bool(shoulder.get("sensorFresh"))
        ready = as_bool(shoulder.get("sensorReady"))
        angle = as_float(shoulder.get("angleDeg"))
        soft_min = as_float(shoulder.get("softMinDeg"))
        soft_max = as_float(shoulder.get("softMaxDeg"))
        outside_limits = ready and (angle < soft_min or angle > soft_max)
        stop_reason = as_str(shoulder.get("lastStopReason"), "NONE")

        if not available or not connected:
            level = DiagnosticStatus.ERROR
            text = "M4 shoulder feedback unavailable"
        elif not fresh or not ready:
            level = DiagnosticStatus.ERROR
            text = "M4 shoulder sensor not ready"
        elif outside_limits:
            level = DiagnosticStatus.WARN
            text = "M4 shoulder outside calibrated limits"
        elif stop_reason == "TARGET_MISSED":
            level = DiagnosticStatus.WARN
            text = "M4 shoulder target missed"
        else:
            level = DiagnosticStatus.OK
            text = "M4 shoulder feedback ready"

        return self.diagnostic_status(
            "motionbrain/shoulder_feedback",
            level,
            text,
            {
                "available": available,
                "connected": connected,
                "fresh": fresh,
                "ready": ready,
                "angle_deg": angle,
                "raw_angle_deg": as_float(shoulder.get("rawDeg")),
                "mount_offset_deg": as_float(shoulder.get("mountOffsetDeg")),
                "target_deg": as_float(shoulder.get("targetDeg")),
                "error_deg": as_float(shoulder.get("errorDeg")),
                "soft_min_deg": soft_min,
                "soft_max_deg": soft_max,
                "target_tolerance_deg": as_float(
                    shoulder.get("targetToleranceDeg")
                ),
                "settled_success_tolerance_deg": as_float(
                    shoulder.get("settledSuccessToleranceDeg")
                ),
                "magnet_detected": as_bool(shoulder.get("magnetDetected")),
                "magnet_too_weak": as_bool(shoulder.get("magnetTooWeak")),
                "magnet_too_strong": as_bool(shoulder.get("magnetTooStrong")),
                "agc": as_uint(shoulder.get("agc")),
                "magnitude": as_uint(shoulder.get("magnitude")),
                "age_ms": as_uint(shoulder.get("ageMs")),
                "control_active": as_bool(shoulder.get("active")),
                "correction_active": as_bool(shoulder.get("correctionActive")),
                "correction_attempts": as_uint(shoulder.get("correctionAttempts")),
                "max_correction_attempts": as_uint(
                    shoulder.get("maxCorrectionAttempts")
                ),
                "manual_guard_blocked": as_bool(shoulder.get("manualGuardBlocked")),
                "stop_reason": stop_reason,
            },
            "esp32_m4_as5600",
        )

    def routine_diagnostic(self, payload: dict[str, Any] | None) -> DiagnosticStatus:
        if payload is None:
            return self.diagnostic_status(
                "motionbrain/routine_executor",
                DiagnosticStatus.ERROR,
                "routine poll unavailable",
                {},
                "esp32_motion_controller",
            )

        executor = payload.get("executor")
        executor = executor if isinstance(executor, dict) else {}
        status = executor.get("status")
        status = status if isinstance(status, dict) else {}
        queue_apply_allowed = as_bool(executor.get("queueApplyAllowed"))
        execute_implemented = as_bool(executor.get("executeImplemented"))
        executor_enabled = as_bool(executor.get("enabled"))

        if queue_apply_allowed:
            level = DiagnosticStatus.WARN
            text = "routine queue apply is enabled"
        elif execute_implemented or executor_enabled:
            level = DiagnosticStatus.WARN
            text = "routine executor policy changed"
        else:
            level = DiagnosticStatus.OK
            text = "routine executor disabled by policy"

        routines = payload.get("routines")
        routine_count = len(routines) if isinstance(routines, list) else 0
        return self.diagnostic_status(
            "motionbrain/routine_executor",
            level,
            text,
            {
                "executor_enabled": executor_enabled,
                "execute_implemented": execute_implemented,
                "queue_apply_allowed": queue_apply_allowed,
                "executor_state": as_str(status.get("state")),
                "executor_last_result": as_str(status.get("lastResult")),
                "routine_count": routine_count,
            },
            "esp32_motion_controller",
        )

    def feedback_diagnostic(self, payload: dict[str, Any] | None) -> DiagnosticStatus:
        if payload is None:
            return self.diagnostic_status(
                "motionbrain/feedback",
                DiagnosticStatus.ERROR,
                "feedback poll unavailable",
                {},
                "esp32_motion_controller",
            )

        feedback = payload.get("feedback")
        feedback = feedback if isinstance(feedback, dict) else {}
        base_yaw = feedback.get("baseYaw")
        base_yaw = base_yaw if isinstance(base_yaw, dict) else {}

        ready = as_bool(feedback.get("readyForRoutineExecution"))
        physical_allowed = as_bool(feedback.get("physicalRoutineExecutionAllowed"))
        fault = as_str(base_yaw.get("fault") or feedback.get("blockReason"), "unknown")
        if ready:
            level = DiagnosticStatus.OK
            text = "feedback ready for physical routines"
        elif physical_allowed:
            level = DiagnosticStatus.ERROR
            text = "feedback policy mismatch"
        else:
            level = DiagnosticStatus.WARN
            text = "feedback not ready for physical routines"

        return self.diagnostic_status(
            "motionbrain/feedback",
            level,
            text,
            {
                "selected_target": as_str(
                    feedback.get("selectedClosureTarget"),
                    "base_yaw_reference",
                ),
                "feedback_ready": ready,
                "physical_routine_execution_allowed": physical_allowed,
                "block_reason": as_str(feedback.get("blockReason"), "feedback_required"),
                "base_yaw_installed": as_bool(base_yaw.get("installed")),
                "base_yaw_available": as_bool(base_yaw.get("available")),
                "base_yaw_connected": as_bool(base_yaw.get("connected")),
                "base_yaw_fresh": as_bool(base_yaw.get("fresh")),
                "base_yaw_referenced": as_bool(base_yaw.get("referenced")),
                "base_yaw_faulted": as_bool(base_yaw.get("faulted")),
                "base_yaw_hardware_ready": as_bool(base_yaw.get("hardwareReady")),
                "base_yaw_signal_active": as_bool(base_yaw.get("signalActive")),
                "base_yaw_pin": as_uint(base_yaw.get("pin")),
                "base_yaw_active_low": as_bool(base_yaw.get("activeLow")),
                "base_yaw_fault": fault,
                "base_yaw_age_ms": as_uint(base_yaw.get("ageMs")),
                "base_yaw_last_update_ms": as_uint(base_yaw.get("lastUpdateMs")),
            },
            "esp32_motion_controller",
        )

    def teleop_sensor_diagnostic(self, payload: dict[str, Any] | None) -> DiagnosticStatus:
        diagnostics = payload.get("diagnostics") if isinstance(payload, dict) else None
        diagnostics = diagnostics if isinstance(diagnostics, dict) else {}
        sensor = diagnostics.get("sensor")
        sensor = sensor if isinstance(sensor, dict) else {}
        teleop = diagnostics.get("teleop")
        teleop = teleop if isinstance(teleop, dict) else {}
        safety = diagnostics.get("safety")
        safety = safety if isinstance(safety, dict) else {}

        sensor_connected = as_bool(sensor.get("connected"))
        sensor_fresh = as_bool(sensor.get("fresh"))
        teleop_connected = as_bool(teleop.get("connected"))
        safety_fault = as_bool(safety.get("faultLatched"))
        if safety_fault:
            level = DiagnosticStatus.ERROR
            text = "teleop/sensor safety fault"
        elif not sensor_connected or not sensor_fresh or not teleop_connected:
            level = DiagnosticStatus.WARN
            text = "teleop or sensor stale"
        else:
            level = DiagnosticStatus.OK
            text = "teleop and sensor fresh"

        return self.diagnostic_status(
            "motionbrain/teleop_sensor",
            level,
            text,
            {
                "sensor_connected": sensor_connected,
                "sensor_fresh": sensor_fresh,
                "sensor_age_ms": as_uint(sensor.get("ageMs")),
                "teleop_connected": teleop_connected,
                "teleop_deadman": as_bool(teleop.get("deadman")),
                "teleop_control_active": as_bool(teleop.get("controlActive")),
                "teleop_age_ms": as_uint(teleop.get("ageMs")),
                "safety_block_reason": as_str(safety.get("blockReason"), "NONE"),
                "safety_fault_reason": as_str(safety.get("faultReason"), "NONE"),
            },
            "stm32_teleop_sensor",
        )

    def camera_diagnostic(self, payload: dict[str, Any] | None) -> DiagnosticStatus:
        if payload is None:
            return self.diagnostic_status(
                "motionbrain/camera_perception",
                DiagnosticStatus.ERROR,
                "camera detection poll unavailable",
                {},
                "esp32_cam_or_pi_perception",
            )

        available = as_bool(payload.get("available"))
        detected = as_bool(payload.get("detected"))
        if not available:
            level = DiagnosticStatus.WARN
            text = "camera detection unavailable"
        elif detected:
            level = DiagnosticStatus.OK
            text = "target detected"
        else:
            level = DiagnosticStatus.OK
            text = "camera available, target not found"

        return self.diagnostic_status(
            "motionbrain/camera_perception",
            level,
            text,
            {
                "available": available,
                "detected": detected,
                "target_type": as_str(payload.get("targetType")),
                "label": as_str(payload.get("label") or payload.get("color")),
                "confidence": as_float(payload.get("confidence")),
                "alignment": as_str(payload.get("alignment"), "LOST"),
                "reason": as_str(payload.get("reason")),
                "camera_url": as_str(payload.get("cameraUrl")),
                "perception_url": as_str(payload.get("perceptionUrl")),
            },
            "esp32_cam_or_pi_perception",
        )

    def publish_status_typed(self, payload: dict[str, Any]) -> None:
        message = MotionStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.available = True
        message.state = as_str(payload.get("state"), "UNKNOWN")
        message.armed = message.state.upper() in {"ARMED", "RUNNING", "MOVING"}
        message.moving = as_bool(payload.get("motorEnabled"))
        message.faulted = message.state.upper() == "FAULT"

        sensor = payload.get("sensor")
        if isinstance(sensor, dict):
            message.faulted = message.faulted or as_bool(sensor.get("faultLatched"))

        base_angle = payload.get("baseAngle")
        if isinstance(base_angle, dict):
            message.base_angle_deg = as_float(base_angle.get("currentDeg"))
            message.moving = message.moving or as_bool(base_angle.get("active"))
            message.stop_reason = as_str(base_angle.get("lastStopReason"))

        teleop = payload.get("teleop")
        if isinstance(teleop, dict):
            message.moving = message.moving or as_bool(teleop.get("controlActive"))
            if not message.stop_reason:
                message.stop_reason = as_str(teleop.get("lastStopReason"))

        shoulder = payload.get("shoulderAngle")
        if isinstance(shoulder, dict):
            message.shoulder_feedback_available = as_bool(shoulder.get("available"))
            message.shoulder_sensor_connected = as_bool(shoulder.get("sensorConnected"))
            message.shoulder_sensor_fresh = as_bool(shoulder.get("sensorFresh"))
            message.shoulder_sensor_ready = as_bool(shoulder.get("sensorReady"))
            message.shoulder_magnet_detected = as_bool(shoulder.get("magnetDetected"))
            message.shoulder_magnet_too_weak = as_bool(shoulder.get("magnetTooWeak"))
            message.shoulder_magnet_too_strong = as_bool(shoulder.get("magnetTooStrong"))
            message.shoulder_control_active = as_bool(shoulder.get("active"))
            message.shoulder_correction_active = as_bool(
                shoulder.get("correctionActive")
            )
            message.shoulder_manual_guard_blocked = as_bool(
                shoulder.get("manualGuardBlocked")
            )
            message.shoulder_correction_attempts = as_uint(
                shoulder.get("correctionAttempts")
            )
            message.shoulder_max_correction_attempts = as_uint(
                shoulder.get("maxCorrectionAttempts")
            )
            message.shoulder_sensor_age_ms = as_uint(shoulder.get("ageMs"))
            message.shoulder_agc = as_uint(shoulder.get("agc"))
            message.shoulder_magnitude = as_uint(shoulder.get("magnitude"))
            message.shoulder_raw_angle_deg = as_float(shoulder.get("rawDeg"))
            message.shoulder_angle_deg = as_float(shoulder.get("angleDeg"))
            message.shoulder_mount_offset_deg = as_float(shoulder.get("mountOffsetDeg"))
            message.shoulder_target_deg = as_float(shoulder.get("targetDeg"))
            message.shoulder_error_deg = as_float(shoulder.get("errorDeg"))
            message.shoulder_soft_min_deg = as_float(shoulder.get("softMinDeg"))
            message.shoulder_soft_max_deg = as_float(shoulder.get("softMaxDeg"))
            message.shoulder_target_tolerance_deg = as_float(
                shoulder.get("targetToleranceDeg")
            )
            message.shoulder_manual_down_boundary_deg = as_float(
                shoulder.get("manualDownBoundaryDeg")
            )
            message.shoulder_manual_up_boundary_deg = as_float(
                shoulder.get("manualUpBoundaryDeg")
            )
            message.shoulder_stop_reason = as_str(
                shoulder.get("lastStopReason"),
                "NONE",
            )
            message.tilt_angle_deg = message.shoulder_angle_deg
            message.moving = message.moving or message.shoulder_control_active
            if message.shoulder_control_active or not message.stop_reason:
                message.stop_reason = message.shoulder_stop_reason

        message.raw_json = compact_json(payload)
        self.status_typed_pub.publish(message)

    def publish_events_typed(self, payload: dict[str, Any]) -> None:
        events = payload.get("events")
        if not isinstance(events, list):
            return

        for index, event in enumerate(events):
            if not isinstance(event, dict):
                continue
            message = MotionEvent()
            message.stamp = self.get_clock().now().to_msg()
            message.index = as_uint(event.get("id"), index)
            message.event_type = as_str(event.get("code") or event.get("severity"))
            message.message = as_str(event.get("detail"))
            message.source = as_str(event.get("category"))
            message.raw_json = compact_json(event)
            self.events_typed_pub.publish(message)

    def publish_routine_typed(self, payload: dict[str, Any]) -> None:
        message = RoutineStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.available = True
        message.controller_state = as_str(payload.get("state"), "UNKNOWN")
        message.dry_run_only = as_bool(payload.get("dryRunOnly"))
        message.execute_implemented = as_bool(payload.get("executeImplemented"))

        executor = payload.get("executor")
        if isinstance(executor, dict):
            message.executor_enabled = as_bool(executor.get("enabled"))
            message.executor_mode = as_str(executor.get("mode"))
            message.abort_supported = as_bool(executor.get("abortSupported"))
            message.timeout_supported = as_bool(executor.get("timeoutSupported"))
            message.materialization_gate_supported = as_bool(
                executor.get("materializationGateSupported")
            )
            message.queue_apply_allowed = as_bool(executor.get("queueApplyAllowed"))

            status = executor.get("status")
            if isinstance(status, dict):
                message.executor_state = as_str(status.get("state"))
                message.routine_name = as_str(status.get("routineName"))
                message.current_step = as_uint(status.get("currentStep"))
                message.total_steps = as_uint(status.get("totalSteps"))
                message.motion_step_count = as_uint(status.get("motionStepCount"))
                message.remaining_ms = as_uint(status.get("remainingMs"))
                message.executor_last_result = as_str(status.get("lastResult"))
                message.executor_last_detail = as_str(status.get("lastDetail"))

        diagnostics = payload.get("diagnostics")
        if isinstance(diagnostics, dict):
            sensor = diagnostics.get("sensor")
            if isinstance(sensor, dict):
                message.sensor_connected = as_bool(sensor.get("connected"))
                message.sensor_fresh = as_bool(sensor.get("fresh"))
                message.sensor_age_ms = as_uint(sensor.get("ageMs"))

            teleop = diagnostics.get("teleop")
            if isinstance(teleop, dict):
                message.teleop_connected = as_bool(teleop.get("connected"))
                message.teleop_deadman = as_bool(teleop.get("deadman"))
                message.teleop_control_active = as_bool(teleop.get("controlActive"))
                message.teleop_age_ms = as_uint(teleop.get("ageMs"))

            safety = diagnostics.get("safety")
            if isinstance(safety, dict):
                message.safety_motion_blocked = as_bool(safety.get("motionBlocked"))
                message.safety_block_reason = as_str(safety.get("blockReason"))
                message.safety_fault_latched = as_bool(safety.get("faultLatched"))
                message.safety_fault_reason = as_str(safety.get("faultReason"))

        recovery = payload.get("recovery")
        if isinstance(recovery, dict):
            message.recovery_action = as_str(recovery.get("action"))

        feedback = payload.get("feedback")
        if isinstance(feedback, dict):
            message.feedback_selected_target = as_str(feedback.get("selectedClosureTarget"))
            message.feedback_ready = as_bool(feedback.get("readyForRoutineExecution"))
            message.physical_routine_execution_allowed = as_bool(
                feedback.get("physicalRoutineExecutionAllowed")
            )
            message.feedback_block_reason = as_str(feedback.get("blockReason"))

            base_yaw = feedback.get("baseYaw")
            if isinstance(base_yaw, dict):
                message.base_yaw_feedback_installed = as_bool(base_yaw.get("installed"))
                message.base_yaw_feedback_available = as_bool(base_yaw.get("available"))
                message.base_yaw_feedback_connected = as_bool(base_yaw.get("connected"))
                message.base_yaw_feedback_fresh = as_bool(base_yaw.get("fresh"))
                message.base_yaw_feedback_referenced = as_bool(base_yaw.get("referenced"))
                message.base_yaw_feedback_faulted = as_bool(base_yaw.get("faulted"))
                message.base_yaw_feedback_hardware_ready = as_bool(
                    base_yaw.get("hardwareReady")
                )
                message.base_yaw_feedback_signal_active = as_bool(
                    base_yaw.get("signalActive")
                )
                message.base_yaw_feedback_pin = as_uint(base_yaw.get("pin"))
                message.base_yaw_feedback_active_low = as_bool(base_yaw.get("activeLow"))
                message.base_yaw_feedback_age_ms = as_uint(base_yaw.get("ageMs"))
                message.base_yaw_feedback_last_update_ms = as_uint(
                    base_yaw.get("lastUpdateMs")
                )
                message.base_yaw_feedback_position_deg = as_float(
                    base_yaw.get("positionDeg")
                )
                message.base_yaw_feedback_velocity_dps = as_float(
                    base_yaw.get("velocityDps")
                )
                message.base_yaw_feedback_stop_reason = as_str(
                    base_yaw.get("lastStopReason")
                )
                message.base_yaw_feedback_fault = as_str(base_yaw.get("fault"))

        last_command = payload.get("lastCommand")
        if isinstance(last_command, dict):
            message.last_command_seen = as_bool(last_command.get("seen"))
            message.last_command_success = as_bool(last_command.get("success"))
            message.last_command_type = as_str(last_command.get("type"))
            message.last_command_source = as_str(last_command.get("source"))
            message.last_command_message = as_str(last_command.get("message"))

        routines = payload.get("routines")
        if isinstance(routines, list):
            routine_names = [
                as_str(routine.get("name"))
                for routine in routines
                if isinstance(routine, dict) and as_str(routine.get("name"))
            ]
            message.routine_names = routine_names
            message.routine_count = len(routine_names)

        message.raw_json = compact_json(payload)
        self.routine_typed_pub.publish(message)

    def publish_detection_typed(self, payload: dict[str, Any]) -> None:
        message = CameraDetection()
        message.stamp = self.get_clock().now().to_msg()
        message.available = as_bool(payload.get("available"))
        message.detected = as_bool(payload.get("detected"))
        message.target_type = as_str(payload.get("targetType"))
        message.label = as_str(payload.get("label") or payload.get("color"))
        try:
            message.class_id = int(payload["classId"]) if payload.get("classId") is not None else -1
        except (TypeError, ValueError):
            message.class_id = -1
        message.confidence = as_float(payload.get("confidence"))
        message.color = as_str(payload.get("color"))
        message.alignment = as_str(payload.get("alignment"), "LOST")
        message.command_suggestion = as_str(payload.get("commandSuggestion"), "none")
        message.area_ratio = as_float(payload.get("areaRatio", payload.get("ratio")))
        message.pixels = as_uint(payload.get("pixels"))
        message.width = as_uint(payload.get("width"))
        message.height = as_uint(payload.get("height"))
        message.frame_bytes = as_uint(payload.get("frameBytes"))
        message.center_x = as_float(payload.get("centerX", payload.get("centroidX")))
        message.center_y = as_float(payload.get("centerY", payload.get("centroidY")))
        message.offset_x = as_float(payload.get("offsetX"))
        message.offset_y = as_float(payload.get("offsetY"))
        message.align_deadband = as_float(payload.get("alignDeadband"), ALIGN_DEADBAND)
        message.camera_url = as_str(payload.get("cameraUrl"))
        message.reason = as_str(payload.get("reason"))
        message.raw_json = compact_json(payload)
        self.detection_typed_pub.publish(message)

    def publish_light_result_typed(self, payload: dict[str, Any]) -> None:
        message = LightResult()
        message.stamp = self.get_clock().now().to_msg()
        message.success = as_bool(payload.get("success"))
        message.requested_action = as_str(payload.get("requestedAction"))
        message.state = as_str(payload.get("state"))
        message.error = as_str(payload.get("error") or payload.get("message"))
        message.raw_json = compact_json(payload)
        self.light_result_typed_pub.publish(message)

    def publish_light_result(self, payload: dict[str, Any]) -> None:
        self.publish_json(self.light_result_pub, payload)
        self.publish_light_result_typed(payload)

    def publish_routine_result_typed(self, payload: dict[str, Any]) -> None:
        message = RoutineResult()
        message.stamp = self.get_clock().now().to_msg()
        message.success = as_bool(payload.get("success"))
        message.action = as_str(payload.get("action"))
        message.routine_name = as_str(payload.get("routineName"))
        message.result = as_str(payload.get("result"))
        message.message = as_str(payload.get("message"))
        message.error = as_str(payload.get("error"))
        message.forwarded = as_bool(payload.get("forwarded"))
        message.raw_json = compact_json(payload)
        self.routine_result_typed_pub.publish(message)

    def publish_routine_result(self, payload: dict[str, Any]) -> None:
        self.publish_json(self.routine_result_pub, payload)
        self.publish_routine_result_typed(payload)

    def populate_routine_service_response(
        self,
        response: GuardedRoutineCommand.Response,
        payload: dict[str, Any],
    ) -> GuardedRoutineCommand.Response:
        response.stamp = self.get_clock().now().to_msg()
        response.success = as_bool(payload.get("success"))
        response.action = as_str(payload.get("action"))
        response.routine_name = as_str(payload.get("routineName"))
        response.result = as_str(payload.get("result"))
        response.message = as_str(payload.get("message"))
        response.error = as_str(payload.get("error"))
        response.forwarded = as_bool(payload.get("forwarded"))
        response.raw_json = compact_json(payload)
        return response

    def populate_routine_action_result(
        self,
        payload: dict[str, Any],
    ) -> GuardedRoutine.Result:
        result = GuardedRoutine.Result()
        result.stamp = self.get_clock().now().to_msg()
        result.success = as_bool(payload.get("success"))
        result.action = as_str(payload.get("action"))
        result.routine_name = as_str(payload.get("routineName"))
        result.result = as_str(payload.get("result"))
        result.message = as_str(payload.get("message"))
        result.error = as_str(payload.get("error"))
        result.forwarded = as_bool(payload.get("forwarded"))
        result.raw_json = compact_json(payload)
        return result

    def routine_feedback(
        self,
        state: str,
        current_step: int,
        total_steps: int,
        message: str,
        payload: dict[str, Any] | None = None,
    ) -> GuardedRoutine.Feedback:
        feedback = GuardedRoutine.Feedback()
        feedback.stamp = self.get_clock().now().to_msg()
        feedback.state = state
        feedback.current_step = current_step
        feedback.total_steps = total_steps
        feedback.message = message
        feedback.raw_json = compact_json(payload or {})
        return feedback

    def poll_once(self) -> None:
        if not self._polling_active:
            return

        self.motion_base_url = self._motion_base_url()
        timeout = self._timeout()
        status_payload = None
        routine_payload = None
        detection_payload = None

        try:
            status = fetch_json(f"{self.motion_base_url}/status", timeout)
            self.publish_json(self.status_pub, status)
            self.publish_status_typed(status)
            status_payload = status
        except POLL_EXCEPTIONS as exc:
            self.get_logger().warning(f"status poll failed: {exc}")

        try:
            routine = fetch_json(f"{self.motion_base_url}/routine", timeout)
            self.publish_json(self.routine_pub, routine)
            self.publish_routine_typed(routine)
            routine_payload = routine
        except POLL_EXCEPTIONS as exc:
            self.get_logger().warning(f"routine poll failed: {exc}")

        try:
            limit = int(self.get_parameter("events_limit").value)
            if limit > 0:
                events = fetch_json(f"{self.motion_base_url}/events?limit={limit}", timeout)
                self.publish_json(self.events_pub, events)
                self.publish_events_typed(events)
        except POLL_EXCEPTIONS as exc:
            self.get_logger().warning(f"events poll failed: {exc}")

        perception_url = str(self.get_parameter("perception_url").value).strip().rstrip("/")
        camera_url = str(self.get_parameter("camera_url").value).strip().rstrip("/")
        if perception_url:
            detection_payload = self.poll_perception(perception_url, timeout)
        elif camera_url:
            detection_payload = self.poll_camera(camera_url, timeout)

        self.publish_diagnostics(status_payload, routine_payload, detection_payload)

    def poll_perception(self, perception_url: str, timeout: float) -> dict[str, Any]:
        try:
            detection = fetch_json(perception_detection_url(perception_url), timeout)
            if not isinstance(detection, dict):
                raise ValueError("perception_detection_not_object")
            detection.setdefault("ts", time.time())
            detection.setdefault("perceptionUrl", perception_url)
            self.publish_json(self.detection_pub, detection)
            self.publish_detection_typed(detection)
        except PERCEPTION_EXCEPTIONS as exc:
            detection = {
                "detected": False,
                "available": False,
                "perceptionUrl": perception_url,
                "reason": str(exc),
                "ts": time.time(),
            }
            self.publish_json(self.detection_pub, detection)
            self.publish_detection_typed(detection)
        return detection

    def poll_camera(self, camera_url: str, timeout: float) -> dict[str, Any]:
        color = str(self.get_parameter("detect_color").value)
        try:
            frame = fetch_bytes(f"{camera_url}/capture", timeout)
            detection = detect_colored_target(frame, color)
            detection["ts"] = time.time()
            detection["cameraUrl"] = camera_url
            self.publish_json(self.detection_pub, detection)
            self.publish_detection_typed(detection)
        except NETWORK_EXCEPTIONS as exc:
            detection = {
                "detected": False,
                "available": False,
                "cameraUrl": camera_url,
                "reason": str(exc),
                "ts": time.time(),
            }
            self.publish_json(self.detection_pub, detection)
            self.publish_detection_typed(detection)
        return detection

    def handle_light_cmd(self, message: String) -> None:
        action = parse_light_action(message.data)
        self.handle_light_action(action, message.data)

    def handle_light_cmd_typed(self, message: LightCommand) -> None:
        raw_payload = message.raw_json or message.action
        action = parse_light_action(message.action or raw_payload)
        self.handle_light_action(action, raw_payload)

    def handle_light_action(self, action: str | None, raw_payload: str) -> None:
        if action is None:
            self.publish_light_result(
                {
                    "success": False,
                    "error": "invalid_light_action",
                    "accepted": ["on", "off", "toggle"],
                    "payload": raw_payload,
                },
            )
            return

        try:
            path = f"/light?action={urllib.parse.quote(action)}"
            token = str(self.get_parameter("http_token").value)
            result = post_motionbrain(self.motion_base_url, path, self._timeout(), token)
            result["requestedAction"] = action
            self.publish_light_result(result)
        except POLL_EXCEPTIONS as exc:
            self.publish_light_result(
                {
                    "success": False,
                    "requestedAction": action,
                    "error": str(exc),
                },
            )

    def handle_routine_cmd(self, message: String) -> None:
        command = parse_routine_command(message.data)
        if command is None:
            self.publish_routine_result(
                {
                    "success": False,
                    "action": "",
                    "routineName": "",
                    "result": "invalid_routine_command",
                    "error": "invalid_routine_command",
                    "message": "accepted actions: status, dry_run, abort",
                    "forwarded": False,
                    "payload": message.data,
                },
            )
            return
        self.handle_routine_action(
            command["action"],
            command["routine_name"],
            command["confirm_code"],
            message.data,
        )

    def handle_routine_cmd_typed(self, message: RoutineCommand) -> None:
        if message.raw_json:
            command = parse_routine_command(message.raw_json)
            if command is not None:
                self.handle_routine_action(
                    command["action"],
                    command["routine_name"],
                    command["confirm_code"],
                    message.raw_json,
                )
                return
        command = parse_routine_command(message.action)
        action = command["action"] if command is not None else ""
        routine_name = message.routine_name or (
            command["routine_name"] if command is not None else ""
        )
        confirm_code = message.confirm_code or (
            command["confirm_code"] if command is not None else ""
        )
        self.handle_routine_action(
            action,
            routine_name,
            confirm_code,
            message.raw_json or compact_json(
                {
                    "action": message.action,
                    "routineName": routine_name,
                    "confirmCode": confirm_code,
                },
            ),
        )

    def handle_routine_service(
        self,
        request: GuardedRoutineCommand.Request,
        response: GuardedRoutineCommand.Response,
    ) -> GuardedRoutineCommand.Response:
        if request.raw_json:
            command = parse_routine_command(request.raw_json)
            if command is not None:
                result = self.execute_routine_action(
                    command["action"],
                    command["routine_name"],
                    command["confirm_code"],
                    request.raw_json,
                )
                self.publish_routine_result(result)
                return self.populate_routine_service_response(response, result)

        command = parse_routine_command(request.action)
        action = command["action"] if command is not None else request.action
        routine_name = request.routine_name or (
            command["routine_name"] if command is not None else ""
        )
        confirm_code = request.confirm_code or (
            command["confirm_code"] if command is not None else ""
        )
        raw_payload = compact_json(
            {
                "action": request.action,
                "routineName": routine_name,
                "confirmCode": confirm_code,
            },
        )
        result = self.execute_routine_action(
            action,
            routine_name,
            confirm_code,
            raw_payload,
        )
        self.publish_routine_result(result)
        return self.populate_routine_service_response(response, result)

    def execute_routine_goal(self, goal_handle: Any) -> GuardedRoutine.Result:
        request = goal_handle.request
        raw_payload = request.raw_json or compact_json(
            {
                "action": request.action,
                "routineName": request.routine_name,
                "confirmCode": request.confirm_code,
            },
        )
        goal_handle.publish_feedback(
            self.routine_feedback(
                "accepted",
                0,
                1,
                "guarded routine action accepted",
                {
                    "action": request.action,
                    "routineName": request.routine_name,
                },
            ),
        )

        if request.raw_json:
            command = parse_routine_command(request.raw_json)
            if command is not None:
                action = command["action"]
                routine_name = command["routine_name"]
                confirm_code = command["confirm_code"]
            else:
                action = request.action
                routine_name = request.routine_name
                confirm_code = request.confirm_code
        else:
            command = parse_routine_command(request.action)
            action = command["action"] if command is not None else request.action
            routine_name = request.routine_name or (
                command["routine_name"] if command is not None else ""
            )
            confirm_code = request.confirm_code or (
                command["confirm_code"] if command is not None else ""
            )

        result_payload = self.execute_routine_action(
            action,
            routine_name,
            confirm_code,
            raw_payload,
        )
        self.publish_routine_result(result_payload)
        goal_handle.publish_feedback(
            self.routine_feedback(
                "completed",
                1,
                1,
                as_str(result_payload.get("message"), "guarded routine action completed"),
                result_payload,
            ),
        )

        if as_bool(result_payload.get("success")):
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return self.populate_routine_action_result(result_payload)

    def routine_response_success(self, payload: dict[str, Any]) -> bool:
        if "success" in payload:
            return as_bool(payload.get("success"))
        if "ok" in payload:
            return as_bool(payload.get("ok"))
        return "error" not in payload

    def wrap_routine_response(
        self,
        action: str,
        routine_name: str,
        response: dict[str, Any],
        result: str,
        message: str,
    ) -> dict[str, Any]:
        return {
            "success": self.routine_response_success(response),
            "action": action,
            "routineName": routine_name,
            "result": as_str(response.get("result"), result),
            "message": as_str(response.get("message"), message),
            "error": as_str(response.get("error")),
            "forwarded": True,
            "response": response,
        }

    def handle_routine_action(
        self,
        action: str,
        routine_name: str,
        confirm_code: str,
        raw_payload: str,
    ) -> None:
        result = self.execute_routine_action(action, routine_name, confirm_code, raw_payload)
        self.publish_routine_result(result)

    def execute_routine_action(
        self,
        action: str,
        routine_name: str,
        confirm_code: str,
        raw_payload: str,
    ) -> dict[str, Any]:
        action = action.strip().lower()
        routine_name = routine_name.strip()
        if action in {"run", "execute"}:
            return {
                "success": False,
                "action": "run",
                "routineName": routine_name,
                "result": "routine_execute_disabled_by_bridge_policy",
                "message": "ROS2 routine bridge forwards only status, dry_run, and abort",
                "error": "routine_execute_disabled_by_bridge_policy",
                "forwarded": False,
                "confirmCodePresent": bool(confirm_code),
                "payload": raw_payload,
            }

        if action not in {"status", "dry_run", "abort"}:
            return {
                "success": False,
                "action": action,
                "routineName": routine_name,
                "result": "invalid_routine_action",
                "message": "accepted actions: status, dry_run, abort",
                "error": "invalid_routine_action",
                "forwarded": False,
                "payload": raw_payload,
            }

        if action == "dry_run" and not routine_name:
            return {
                "success": False,
                "action": action,
                "routineName": routine_name,
                "result": "missing_routine_name",
                "message": "dry_run requires routine_name",
                "error": "missing_routine_name",
                "forwarded": False,
                "payload": raw_payload,
            }

        self.motion_base_url = self._motion_base_url()
        timeout = self._timeout()
        try:
            if action == "status":
                response = fetch_json(f"{self.motion_base_url}/routine", timeout)
                self.publish_json(self.routine_pub, response)
                self.publish_routine_typed(response)
                return self.wrap_routine_response(
                    action,
                    routine_name,
                    response,
                    "status",
                    "routine status fetched",
                )

            token = str(self.get_parameter("http_token").value)
            if action == "dry_run":
                path = f"/routine?action=dry_run&name={urllib.parse.quote(routine_name)}"
                response = post_motionbrain(self.motion_base_url, path, timeout, token)
                return self.wrap_routine_response(
                    action,
                    routine_name,
                    response,
                    "dry_run",
                    "routine dry-run requested",
                )

            response = post_motionbrain(self.motion_base_url, "/routine?action=abort", timeout, token)
            return self.wrap_routine_response(
                action,
                routine_name,
                response,
                "abort",
                "routine abort requested",
            )
        except POLL_EXCEPTIONS as exc:
            return {
                "success": False,
                "action": action,
                "routineName": routine_name,
                "result": "http_error",
                "message": "routine HTTP request failed",
                "error": str(exc),
                "forwarded": action in {"status", "dry_run", "abort"},
                "payload": raw_payload,
            }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotionBrainStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
