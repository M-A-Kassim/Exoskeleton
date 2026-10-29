from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exo_communication',
            executable='test_node',
            name='exo_test_node',
            output='screen'
        )
    ])
