from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2socketcan_bridge',
            executable='ros2socketcan',
            name='can0',
            parameters=[{'can_interface': 'can0', 'log_level': 50}]
        ),
        Node(
            package='ros2socketcan_bridge',
            executable='ros2socketcan',
            name='can1',
            parameters=[{'can_interface': 'can1', 'log_level': 50}]
        ),
    ])

