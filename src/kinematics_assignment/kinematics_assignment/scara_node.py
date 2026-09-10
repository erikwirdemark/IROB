#!/usr/bin/env python3
"""
    This node publishes the joint states to make a square trajectory with the SCARA's end-effector
"""

import importlib
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray

from kinematics_assignment.IK_functions import scara_IK
from kinematics_assignment.square_trajectory import SquareTrajectory


class ScaraTrajectoryPub(Node):
    def __init__(self):
        super().__init__('scara_trajectory_pub')

        self.declare_parameter('topic_name', 'joint_states')
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.joint_pub = self.create_publisher(JointState, self.topic_name, 10)

        # the 4 vertices of the square
        vertices = [
            [0.27, -0.15, 0],
            [0.57, -0.15, 0.1],
            [0.57,  0.15, 0.1],
            [0.27,  0.15, 0]
        ]
        self.traj = SquareTrajectory(self, vertices, base_frame='base')  # make sure your ROS2 SquareTrajectory takes (node, ...)

        self.joint_names = ['rotational1', 'rotational2', 'translation']

        # Publish initial joint state at fully stretched configuration
        q = [0.0, 0.0, 0.0]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = list(q)
        self.joint_pub.publish(js)

        # timer @ 10 Hz
        self.timer = self.create_timer(0.1, self._tick)
    
    def _tick(self):
        self.traj.publish_path()

        point = self.traj.get_point()
        if point is None:
            self.get_logger().info("Point is None")
            return

        q = scara_IK(point)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = list(q)
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = ScaraTrajectoryPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
