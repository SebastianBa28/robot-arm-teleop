import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robot_arm_teleop'

    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'ur10e.urdf')
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'viewurdf.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('server_url', default_value='ws://localhost:8000/ws/ros'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),

        # Kinematics Node
        Node(
            package=package_name,
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[{
                'server_url': LaunchConfiguration('server_url'),
            }]
        ),
    ])
