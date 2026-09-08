#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('kinematics_assignment')

    # URDF (plain XML, no xacro here)
    urdf_file = os.path.join(pkg_share, 'urdf', 'SCARA.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # ── TF broadcaster ───────────────────────────────────────────
        Node(
            package   = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            name      = 'robot_state_publisher',
            parameters = [{'robot_description': robot_description}],
            output    = 'screen'
        ),

        # ── your joint-state & IK publisher ──────────────────────────
        Node(
            package    = 'kinematics_assignment',
            executable = 'scara_node',
            name       = 'scara_node',
            parameters = [{'topic_name': 'joint_states'}],   # keep default
            output     = 'screen'
        ),

        # ── RViz (pre-configured) ────────────────────────────────────
        Node(
            package    = 'rviz2',
            executable = 'rviz2',
            name       = 'rviz2',
            arguments  = [
                '-d',
                os.path.join(pkg_share, 'rviz', 'scara_world.rviz')
            ],
            output     = 'screen'
        ),
    ])
