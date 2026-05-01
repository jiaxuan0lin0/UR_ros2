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
        ),
        Node(
            package='controllers',
            executable='ur_joint_control',
            name='ur_joint_control',
            output='screen',
            parameters=[{
                'arm_joint_state_topic': '/joint_states',
                'gripper_joint_state_topic': '/gripper_joint_states',
                'joint_state_topic': '/ur/joint_states',
                'joint_control_action_name': (
                    '/ur/joint_control/follow_joint_trajectory'
                ),
                'arm_action_name': (
                    '/scaled_joint_trajectory_controller/'
                    'follow_joint_trajectory'
                ),
                'gripper_trajectory_topic': '/gripper_trajectory',
            }],
        ),
    ])
