from __future__ import annotations

import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
controller_manager_srvs = pytest.importorskip("controller_manager_msgs.srv")
motionbrain_msgs = pytest.importorskip("motionbrain_msgs.msg")
sensor_msgs = pytest.importorskip("sensor_msgs.msg")

ListControllers = controller_manager_srvs.ListControllers
ListHardwareInterfaces = controller_manager_srvs.ListHardwareInterfaces
MotionStatus = motionbrain_msgs.MotionStatus
JointState = sensor_msgs.JointState


REQUIRED_RUNTIME_PACKAGES = [
    "controller_manager",
    "joint_state_broadcaster",
    "motionbrain_hardware_interface",
    "robot_state_publisher",
    "ros2_control",
    "ros2controlcli",
]


def isolated_env(offset: int = 0) -> dict[str, str]:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(170 + ((os.getpid() + offset) % 30))
    return env


def require_runtime_packages(env: dict[str, str]) -> None:
    missing = []
    for package in REQUIRED_RUNTIME_PACKAGES:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", package],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5,
            check=False,
        )
        if result.returncode != 0:
            missing.append(package)
    if missing:
        pytest.skip(f"missing ROS2 runtime packages: {', '.join(missing)}")


def start_m4_measured_launch(
    env: dict[str, str],
    *,
    status_topic: str,
    direction_sign: int = 1,
    stale_timeout_sec: float = 2.0,
) -> tuple[subprocess.Popen, Path]:
    log_file = tempfile.NamedTemporaryFile(
        prefix="motionbrain_m4_measured_launch_",
        suffix=".log",
        delete=False,
    )
    log_path = Path(log_file.name)
    log_file.close()
    log_handle = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "motionbrain_hardware_interface",
            "m4_measured_state.launch.py",
            "autostart_controllers:=true",
            f"status_topic:={status_topic}",
            "shoulder_feedback_calibration_enabled:=true",
            "shoulder_sensor_zero_deg:=234.5",
            f"shoulder_direction_sign:={direction_sign}",
            "shoulder_ros_joint_zero_rad:=0.0",
            f"state_stale_timeout_sec:={stale_timeout_sec}",
        ],
        env=env,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    log_handle.close()
    return process, log_path


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=8)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=4)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait(timeout=4)


def publish_status(publisher) -> None:
    message = MotionStatus()
    message.available = True
    message.state = "IDLE"
    message.shoulder_feedback_available = True
    message.shoulder_sensor_connected = True
    message.shoulder_sensor_fresh = True
    message.shoulder_sensor_ready = True
    message.shoulder_magnet_detected = True
    message.shoulder_sensor_age_ms = 10
    message.shoulder_angle_deg = 235.5
    message.shoulder_raw_angle_deg = 235.5
    publisher.publish(message)


def call_service(node, client, request, timeout_sec: float = 5.0):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise AssertionError(f"service call timed out: {client.srv_name}")
    return future.result()


def wait_for_active_joint_state_broadcaster(
    node,
    status_publisher,
    controller_client,
    timeout_sec: float = 35.0,
):
    deadline = time.monotonic() + timeout_sec
    last_states = []
    while time.monotonic() < deadline:
        publish_status(status_publisher)
        rclpy.spin_once(node, timeout_sec=0.05)
        if not controller_client.wait_for_service(timeout_sec=0.1):
            continue
        response = call_service(node, controller_client, ListControllers.Request())
        last_states = [(controller.name, controller.state) for controller in response.controller]
        for controller in response.controller:
            if controller.name == "joint_state_broadcaster" and controller.state == "active":
                return response
        time.sleep(0.2)
    raise AssertionError(f"joint_state_broadcaster did not become active: {last_states}")


def wait_for_finite_joint_state(
    node,
    status_publisher,
    messages: list[JointState],
    timeout_sec: float = 20.0,
) -> JointState:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        publish_status(status_publisher)
        rclpy.spin_once(node, timeout_sec=0.1)
        for message in reversed(messages):
            if "shoulder_pitch_joint" not in message.name:
                continue
            index = list(message.name).index("shoulder_pitch_joint")
            if index < len(message.position) and math.isfinite(message.position[index]):
                return message
    raise AssertionError("finite shoulder_pitch_joint state was not published")


def test_m4_measured_launch_exposes_read_only_joint_state_broadcaster() -> None:
    env = isolated_env()
    require_runtime_packages(env)
    status_topic = "/test/m4_measured_status_typed"
    process, log_path = start_m4_measured_launch(env, status_topic=status_topic)

    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    rclpy.init(args=None)
    try:
        io_node = rclpy.create_node("m4_measured_launch_io")
        status_publisher = io_node.create_publisher(MotionStatus, status_topic, 10)
        joint_states: list[JointState] = []
        io_node.create_subscription(JointState, "/joint_states", joint_states.append, 10)
        controller_client = io_node.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
        )
        hardware_client = io_node.create_client(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces",
        )

        response = wait_for_active_joint_state_broadcaster(
            io_node,
            status_publisher,
            controller_client,
        )
        assert any(
            controller.name == "joint_state_broadcaster" and controller.state == "active"
            for controller in response.controller
        )

        assert hardware_client.wait_for_service(timeout_sec=5.0)
        interfaces = call_service(io_node, hardware_client, ListHardwareInterfaces.Request())
        state_interfaces = {interface.name: interface for interface in interfaces.state_interfaces}
        command_interfaces = {interface.name: interface for interface in interfaces.command_interfaces}
        assert state_interfaces["shoulder_pitch_joint/position"].is_available
        assert state_interfaces["shoulder_pitch_joint/velocity"].is_available
        assert "shoulder_pitch_joint/position" not in command_interfaces
        assert "shoulder_pitch_joint/velocity" not in command_interfaces

        message = wait_for_finite_joint_state(io_node, status_publisher, joint_states)
        index = list(message.name).index("shoulder_pitch_joint")
        assert math.isclose(message.position[index], math.radians(1.0), abs_tol=0.05)

        io_node.destroy_node()
    finally:
        rclpy.shutdown()
        stop_process(process)
        if process.returncode not in {0, -signal.SIGINT}:
            log_tail = log_path.read_text(encoding="utf-8", errors="replace")[-4000:]
            pytest.fail(f"m4 measured launch exited unexpectedly: {process.returncode}\n{log_tail}")


def test_m4_measured_launch_rejects_invalid_direction_sign() -> None:
    env = isolated_env(offset=1)
    require_runtime_packages(env)
    process, log_path = start_m4_measured_launch(
        env,
        status_topic="/test/m4_invalid_status_typed",
        direction_sign=0,
    )
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    rclpy.init(args=None)
    try:
        io_node = rclpy.create_node("m4_invalid_launch_io")
        controller_client = io_node.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
        )
        deadline = time.monotonic() + 18.0
        last_states = []
        while time.monotonic() < deadline:
            if process.poll() is not None and process.returncode != 0:
                return
            rclpy.spin_once(io_node, timeout_sec=0.1)
            if controller_client.wait_for_service(timeout_sec=0.1):
                response = call_service(
                    io_node,
                    controller_client,
                    ListControllers.Request(),
                    timeout_sec=2.0,
                )
                last_states = [
                    (controller.name, controller.state)
                    for controller in response.controller
                ]
                assert not any(
                    controller.name == "joint_state_broadcaster"
                    and controller.state == "active"
                    for controller in response.controller
                ), last_states
            time.sleep(0.5)
        io_node.destroy_node()
    finally:
        rclpy.shutdown()
        stop_process(process)

    log_tail = log_path.read_text(encoding="utf-8", errors="replace")[-4000:]
    assert not any(name == "joint_state_broadcaster" and state == "active" for name, state in last_states), (
        f"{last_states}\n{log_tail}"
    )
