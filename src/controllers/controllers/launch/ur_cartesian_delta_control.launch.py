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
            executable='ur_cartesian_delta_control',
            name='ur_cartesian_delta_control',
            output='screen',
            parameters=[{
                'cartesian_delta_action_name': (
                    '/ur/cartesian_delta_control/follow_cartesian_delta'
                ),
                'servo_pose_topic': '/servo_node/pose_target_cmds',
                'servo_command_type_service': (
                    '/servo_node/switch_command_type'
                ),
                'tcp_pose_topic': '/tcp_pose_broadcaster/pose',
                'tcp_pose_publish_topic': '/ur/tcp_pose',
                'gripper_joint_state_topic': '/gripper_joint_states',
                'gripper_trajectory_topic': '/gripper_trajectory',
                'state_topic': '/ur/cartesian_delta_control/state',
                'target_pose_publish_topic': (
                    '/ur/cartesian_delta_control/target_pose'
                ),
                'command_frame': 'tool0',
                'servo_pose_frame': 'base_link',
                'tool_frame': 'tool0',
                'default_point_duration_sec': 1.0 / 30.0,
                'servo_command_hz': 125.0,
                'set_servo_pose_command_type': True,
            }],
        ),
    ])
