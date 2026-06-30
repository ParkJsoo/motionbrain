from __future__ import annotations

import os
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from lifecycle_msgs.msg import Transition  # noqa: E402
from lifecycle_msgs.srv import ChangeState  # noqa: E402
from motionbrain_msgs.msg import CameraDetection  # noqa: E402
from motionbrain_msgs.msg import ControlGuard  # noqa: E402
from motionbrain_msgs.msg import MotionStatus  # noqa: E402


def spin_for(nodes: list, duration_sec: float) -> None:
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.01)


def wait_for_service(node, client, name: str) -> None:
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.1):
            return
        rclpy.spin_once(node, timeout_sec=0.0)
    raise AssertionError(f"service unavailable: {name}")


def change_state(node, client, transition_id: int) -> None:
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = client.call_async(request)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    assert future.done(), f"transition timed out: {transition_id}"
    assert future.result().success, f"transition failed: {transition_id}"


def wait_for_subscribers(node, topics: list[str], nodes: list) -> None:
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        spin_for(nodes, 0.05)
        if all(node.count_subscribers(topic) > 0 for topic in topics):
            return
    raise AssertionError(f"missing subscribers: {topics}")


def publish_inputs(status_pub, detection_pub) -> None:
    status = MotionStatus()
    status.available = True
    status.armed = True
    status.moving = False
    status.faulted = False
    status.state = "IDLE"
    status_pub.publish(status)

    detection = CameraDetection()
    detection.available = True
    detection.detected = True
    detection.alignment = "CENTER"
    detection.command_suggestion = "hold"
    detection_pub.publish(detection)


def publish_until_guard(status_pub, detection_pub, nodes: list, messages: list) -> None:
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        publish_inputs(status_pub, detection_pub)
        spin_for(nodes, 0.1)
        if messages:
            return


def test_control_guard_lifecycle_blocks_outputs_until_active() -> None:
    domain_id = str(150 + (os.getpid() % 50))
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = domain_id

    status_topic = "/test/control_guard_status"
    detection_topic = "/test/control_guard_detection"
    output_topic = "/test/control_guard_typed"
    json_output_topic = "/test/control_guard"

    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "motionbrain_control",
            "motionbrain_control_guard_node",
            "--ros-args",
            "-p",
            "autostart:=false",
            "-p",
            f"status_topic:={status_topic}",
            "-p",
            f"detection_topic:={detection_topic}",
            "-p",
            f"output_topic:={output_topic}",
            "-p",
            f"json_output_topic:={json_output_topic}",
            "-p",
            "publish_rate_hz:=20.0",
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )

    os.environ["ROS_DOMAIN_ID"] = domain_id
    rclpy.init(args=None)
    try:
        io_node = rclpy.create_node("control_guard_lifecycle_io")
        transition_client = io_node.create_client(
            ChangeState,
            "/motionbrain_control_guard_node/change_state",
        )
        messages: list[ControlGuard] = []
        io_node.create_subscription(ControlGuard, output_topic, messages.append, 10)
        status_pub = io_node.create_publisher(MotionStatus, status_topic, 10)
        detection_pub = io_node.create_publisher(CameraDetection, detection_topic, 10)
        nodes = [io_node]

        wait_for_service(
            io_node,
            transition_client,
            "/motionbrain_control_guard_node/change_state",
        )
        spin_for(nodes, 0.3)
        assert messages == []

        change_state(io_node, transition_client, Transition.TRANSITION_CONFIGURE)
        wait_for_subscribers(io_node, [status_topic, detection_topic], nodes)
        publish_inputs(status_pub, detection_pub)
        spin_for(nodes, 0.4)
        assert messages == []

        change_state(io_node, transition_client, Transition.TRANSITION_ACTIVATE)
        publish_until_guard(status_pub, detection_pub, nodes, messages)
        assert messages
        assert messages[-1].ready
        assert messages[-1].reason == "ready"

        messages.clear()
        change_state(io_node, transition_client, Transition.TRANSITION_DEACTIVATE)
        spin_for(nodes, 0.2)
        messages.clear()
        publish_inputs(status_pub, detection_pub)
        spin_for(nodes, 0.4)
        assert messages == []

        io_node.destroy_node()
    finally:
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)
