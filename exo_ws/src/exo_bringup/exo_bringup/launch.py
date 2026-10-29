from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=[
                'serial',
                '--dev', '/dev/ttyUSB0',
                '-b', '115200'
            ],
            output='screen',
            emulate_tty=True
        ),
    ])

# usbipd attach --wsl --busid 1-2
# source /opt/ros/jazzy/setup.bash
# source ~/exoskeleton_project/microros_agent_ws/install/local_setup.bash
# source ~/exoskeleton_project/exo_ws/install/local_setup.bash
#ros2 launch exo_bringup launch.py