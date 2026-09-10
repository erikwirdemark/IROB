#!/usr/bin/env python3
"""
    This node publishes the joint states to make a given trajectory with the KUKA's end-effector
"""

import importlib
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from kinematics_assignment.square_trajectory import SquareTrajectory
from kinematics_assignment import IK_functions


class KukaNode(Node):
    def __init__(self):
        super().__init__('kuka_node')

        # define the ros topic where to publish the joints values
        topic = self.declare_parameter('topic_name', 'joint_states') \
                    .get_parameter_value().string_value
        self.pub = self.create_publisher(JointState, topic, 10)

        # the vertices of the square trajectory (in this case it will be a line)-
        vertices = [[-0.217, 0, 0.84], [-0.2, 0, 0.65], [-0.2, 0, 0.65], [-0.217, 0, 0.84]]
        # the name of the robot's base frame
        base_frame = 'lwr_base_link'
        self.traj = SquareTrajectory(self, vertices, base_frame)

        self.desired_R = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]
        self.current_q = [0.0, 1.12, 0.0, 1.71, 0.0, 1.84, 0.0]  # 7 real joints

        # the joint names of kuka:
        self.names = [
            'lwr_a1_joint', 'lwr_a1_joint_stiffness',
            'lwr_a2_joint', 'lwr_a2_joint_stiffness',
            'lwr_e1_joint', 'lwr_e1_joint_stiffness',
            'lwr_a3_joint', 'lwr_a3_joint_stiffness',
            'lwr_a4_joint', 'lwr_a4_joint_stiffness',
            'lwr_a5_joint', 'lwr_a5_joint_stiffness',
            'lwr_a6_joint', 'lwr_a6_joint_stiffness'
        ]

        # define the ros message for publishing the joint positions
        self.msg = JointState()
        self.msg.name = self.names

        # restart service
        self.create_service(Empty, 'restart', self.on_restart)

        self.timer = self.create_timer(0.1, self.tick)
        # A service is used to restart the trajectory execution and reload a new solution to test
        self.on_restart(None, Empty.Response())

    @staticmethod
    def pack_positions(q7):
        out = []
        for qi in q7:
            out += [float(qi), 0.0]
        return out

    def on_restart(self, _req, _res):
        importlib.reload(IK_functions)
        self.current_q = [0.0, 1.12, 0.0, 1.71, 0.0, 1.84, 0.0]

        # publish one message immediately (non-zero stamp!)
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = self.pack_positions(self.current_q)
        self.pub.publish(self.msg)

        self.traj.restart()
        self.traj.publish_path()
        self.get_logger().info('Restarted: published initial JointState and path.')
        return Empty.Response()

    def tick(self):
        # get the current point in the trajectory
        point = self.traj.get_point()
        if point is not None:
            try:
                # get the IK solution for this point
                q = IK_functions.kuka_IK(point, self.desired_R, self.current_q)
                if q is not None and len(q) == 7:
                    self.current_q = list(q)
            except Exception as e:
                self.get_logger().warn(f'IK error: {e}')

        # publish this solution
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = self.pack_positions(self.current_q)
        self.pub.publish(self.msg)

        # publish the path to be visible in RViz
        self.traj.publish_path()


def main(args=None):
    rclpy.init(args=args)
    node = KukaNode()
    # uncomment if you need to give some time for RViz to start up if it is too slow
    # import time
    # time.sleep(30)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
