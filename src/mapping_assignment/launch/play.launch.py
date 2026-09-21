import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get package share directories
    mapping_pkg_dir = get_package_share_directory('mapping_assignment')
    urdf_pkg_dir = get_package_share_directory('turtlebot3_description')

    # Path to configuration files
    rviz_config_file = os.path.join(mapping_pkg_dir, 'rviz', 'config.rviz')
    urdf_file = os.path.join(urdf_pkg_dir, 'urdf', 'turtlebot3_burger.urdf')

    # Process URDF file (it uses xacro:arg/xacro:property, so it must go through xacro
    # even though it's named .urdf)
    robot_description = xacro.process_file(urdf_file).toxml()

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch description
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
