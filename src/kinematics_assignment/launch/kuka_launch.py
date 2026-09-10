#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_share  = get_package_share_directory('kinematics_assignment')

    robot_name = LaunchConfiguration('robot_name').perform(context)
    remap_name = LaunchConfiguration('remap_name').perform(context)

    # Build the absolute path to the xacro file (Foxy-safe way)
    xacro_path = os.path.join(pkg_share, 'urdf', robot_name + '.urdf.xacro')

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        TextSubstitution(text=xacro_path)
    ])

    nodes = []

    # TF broadcaster
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        )
    )

    # Your single joint-state publisher
    nodes.append(
        Node(
            package='kinematics_assignment',
            executable='kuka_node',
            name='kuka_node',
            output='screen',
            parameters=[{'topic_name': remap_name}]
        )
    )

    # RViz
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                os.path.join(pkg_share, 'rviz', 'kuka_world.rviz')
            ],
            output='screen'
        )
    )

    return nodes


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='single_lwr_robot',
        description='Name of the KUKA xacro in urdf/'
    )

    remap_name_arg = DeclareLaunchArgument(
        'remap_name',
        default_value='joint_states',
        description='Topic where kuka_node publishes joint states'
    )

    return LaunchDescription([
        robot_name_arg,
        remap_name_arg,
        OpaqueFunction(function=launch_setup),
    ])
