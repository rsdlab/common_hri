from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='common_hri',
            executable='Mainengine_ros2.py',
            name='Mainengine',
            output='screen'
        ),
        Node(
            package='common_hri',
            executable='Move.py',
            name='Move',
            output='screen'
        ),
        Node(
            package='common_hri',
            executable='Approach.py',
            name='Approach',
            output='screen'
        ),
        Node(
            package='common_hri',
            executable='Leave.py',
            name='Leave',
            output='screen'
        ),
    ])
