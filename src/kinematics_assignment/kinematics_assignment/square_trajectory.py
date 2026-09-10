#!/usr/bin/env python3

"""
    This class contains a squared trajectory in 3D given the 4 vertices, and it publishes in RViz
"""

from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import rclpy
from rclpy.node import Node


class SquareTrajectory():
    """docstring for SquareTrajectory"""

    def __init__(self, node: Node, vertices=[[0.27, -0.15, 0], [0.57, -0.15, 0.1], [0.57, 0.15, 0.1], [0.27, 0.15, 0]], base_frame='base'):
        self._node = node
        self._path_publisher = node.create_publisher(PoseArray, 'desired_path', 10)
        self._path = PoseArray()
        self._path.header.frame_id = base_frame
        self._dt = 0.1
        self._v = 0.05
        # the 4 vertices of the square
        self._vertices = vertices

        self._current_segment = 0
        self._current_idx = 0
        self._waypoints = None
        self.compute_waypoints()

    def next_segment(self):
        """This function returns the next segment of the square. -1 if the path ended"""
        if self._current_segment == 3:
            self._current_segment = 0
        else:
            self._current_segment += 1

        self._current_idx = 0

    def return_list_of_waypoints(self, p1, p2, dp, it):
        """This function returns a list of waypoints between a start and a goal point"""
        waypoints = list()
        current_p = np.array(p1)
        waypoints.append(current_p)
        p = Pose()
        p.position.x = current_p[0]
        p.position.y = current_p[1]
        p.position.z = current_p[2]
        p.orientation.w = 0.707
        p.orientation.y = 0.707
        self._path.poses.append(p)

        for i in range(1, int(abs(it))):
            current_p = current_p + dp
            waypoints.append(current_p)
            p = Pose()
            p.position.x = current_p[0]
            p.position.y = current_p[1]
            p.position.z = current_p[2]
            p.orientation.w = 0.707
            p.orientation.y = 0.707
            self._path.poses.append(p)

        waypoints.append(np.array(p2))

        return waypoints

    def compute_waypoints(self):
        """This function computes all the 4 segments of the square, given the initial vertices"""
        ds = self._v * self._dt
        v1, v2, v3, v4 = [np.array(v) for v in self._vertices]

        segments = []
        for start, end in zip([v1, v2, v3, v4], [v2, v3, v4, v1]):
            v = end - start
            v_ds = v / ds
            it = abs(v_ds[np.abs(v_ds).argmax()])
            if abs(float(it)) < 0.000001:
                segment = []
            else:
                dv = v / float(it)
                segment = self.return_list_of_waypoints(start, end, dv, it)
            segments.append(segment)

        self._waypoints = segments

    def publish_path(self):
        """This function publishes the path in RViz"""
        self._path.header.stamp = self._node.get_clock().now().to_msg()
        self._path_publisher.publish(self._path)

    def get_point(self):
        """This function returns the next point in the path. None if the path ended"""
        if self._current_segment == -1:
            return None

        if self._current_idx >= len(self._waypoints[self._current_segment]):
            self.next_segment()

        if self._current_segment == -1:
            return None

        if len(self._waypoints[self._current_segment]) < 1:
            self.next_segment()

        if self._current_segment == -1:
            return None

        desired_point = self._waypoints[self._current_segment][self._current_idx]
        self._current_idx += 1

        return desired_point

    def restart(self):
        """This function resets the current point to go through the path once again"""
        self._current_segment = 0
        self._current_idx = 0
