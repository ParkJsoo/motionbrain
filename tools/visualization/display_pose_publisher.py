"""격리된 RViz 표시 컨테이너에서 저장 자세만 발행한다. 장치 접근 없음."""
import json
import math
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


class DisplayPosePublisher(Node):
    def __init__(self, path):
        super().__init__('motionbrain_last_measured_display')
        self.path = Path(path)
        self.joints = self.create_publisher(JointState, '/joint_states', 10)
        self.labels = self.create_publisher(Marker, '/motionbrain/display_status', 10)
        self.create_timer(.2, self.publish_pose)

    def publish_pose(self):
        try:
            pose = json.loads(self.path.read_text())
            if pose.get('controlEligible') is not False or not all(type(v) in (int,float) and math.isfinite(v) for v in pose['joints'].values()):
                raise ValueError('display_pose_invalid')
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(pose['joints'])
            msg.position = list(pose['joints'].values())
            self.joints.publish(msg)
            measured = pose['lastMeasuredM4']
            label = Marker()
            label.header.frame_id = 'world'
            label.header.stamp = msg.header.stamp
            label.ns = 'display_status';label.id = 1;label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.z = .30;label.pose.orientation.w = 1.
            label.scale.z = .012;label.color.r = .05;label.color.g = .05;label.color.b = .05;label.color.a = 1.
            label.text = f'LAST M4 {measured["sensorDeg"]:.2f} deg -> approx {measured["displayJointDeg"]:.2f} deg\nOther joints assumed; no live feedback; physics uncalibrated'
            self.labels.publish(label)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning('Stored display pose unavailable: '+str(exc))


if __name__ == '__main__':
    rclpy.init()
    node = DisplayPosePublisher(sys.argv[1])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node();rclpy.shutdown()
