from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controllers',
            executable='gripper',
            name='gripper',
            output='screen',
            parameters=[{
                'port': '/tmp/ttyUR',
                'baudrate': 115200,
            }],
        )
    ])
