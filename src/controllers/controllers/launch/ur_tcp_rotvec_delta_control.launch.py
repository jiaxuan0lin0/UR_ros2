from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_point_duration_sec = LaunchConfiguration(
        'default_point_duration_sec'
    )
    servo_command_hz = LaunchConfiguration('servo_command_hz')
    wait_for_final_pose_target = LaunchConfiguration(
        'wait_for_final_pose_target'
    )
    command_frame = LaunchConfiguration('command_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'default_point_duration_sec',
            default_value='0.03333333333333333',
        ),
        DeclareLaunchArgument('servo_command_hz', default_value='125.0'),
        DeclareLaunchArgument(
            'wait_for_final_pose_target',
            default_value='false',
        ),
        DeclareLaunchArgument('command_frame', default_value='base'),
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
            executable='ur_tcp_rotvec_delta_control',
            name='ur_tcp_rotvec_delta_control',
            output='screen',
            parameters=[{
                'tcp_rotvec_delta_action_name': (
                    '/ur/tcp_rotvec_delta_control/follow_tcp_rotvec_delta'
                ),
                'servo_pose_topic': '/servo_node/pose_target_cmds',
                'servo_command_type_service': (
                    '/servo_node/switch_command_type'
                ),
                'tcp_pose_topic': '/tcp_pose_broadcaster/pose',
                'tcp_pose_publish_topic': '/ur/tcp_pose',
                'gripper_joint_state_topic': '/gripper_joint_states',
                'gripper_trajectory_topic': '/gripper_trajectory',
                'state_topic': '/ur/tcp_rotvec_delta_control/state',
                'target_pose_publish_topic': (
                    '/ur/tcp_rotvec_delta_control/target_pose'
                ),
                'command_frame': command_frame,
                'servo_pose_frame': 'base_link',
                'tool_frame': 'tool0',
                'default_point_duration_sec': default_point_duration_sec,
                'servo_command_hz': servo_command_hz,
                'set_servo_pose_command_type': True,
                'wait_for_final_pose_target': wait_for_final_pose_target,
            }],
        ),
    ])
