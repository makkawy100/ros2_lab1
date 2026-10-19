import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('lab1_control'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),
        Node(package='lab1_control', executable='go_to_pose',
             output='screen', parameters=[params]),
        Node(package='lab1_control', executable='draw_square',
             output='screen', parameters=[params]),
        Node(package='lab1_control', executable='reset_service',
             output='screen', parameters=[params]),
    ])
