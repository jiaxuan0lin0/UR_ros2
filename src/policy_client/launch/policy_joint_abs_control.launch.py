from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _arg(name: str, default: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default)


def _param(name: str, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    python_executable = LaunchConfiguration('python_executable')

    launch_args = [
        _arg(
            'python_executable',
            PathJoinSubstitution([
                EnvironmentVariable('VIRTUAL_ENV', default_value='/usr'),
                'bin',
                'python3',
            ]),
        ),
        _arg('policy_base_url', 'http://127.0.0.1:8777'),
        _arg('policy_script', ''),
        _arg('policy_timeout_sec', '120.0'),
        _arg('instruction', ''),
        _arg('robot_ip', '192.168.1.102'),
        _arg('primary_camera_serial', ''),
        _arg('wrist_camera_serial', ''),
        _arg('require_wrist_image', 'true'),
        _arg('autostart', 'false'),
        _arg('dry_run', 'false'),
        _arg('rate_hz', '0.6'),
        _arg('use_action_chunk', 'true'),
        _arg('chunk_size', '50'),
        _arg('history_capture_hz', '30.0'),
        _arg('max_pending_history_frames', '8'),
        _arg('joint_point_duration_sec', '0.03333333333333333'),
        _arg('max_joint_delta', '0.00136'),
        _arg('rtde_servoj_time_sec', '0.002'),
        _arg('rtde_servoj_control_frequency', '500.0'),
        _arg('rtde_servoj_max_velocity', '1.05'),
        _arg('rtde_servoj_max_acceleration', '1.4'),
        _arg('rtde_servoj_lookahead_time', '0.06'),
        _arg('rtde_servoj_gain', '500.0'),
        _arg('rtde_servoj_interpolation', 'pchip'),
        _arg('rtde_servoj_stop_acceleration', '2.0'),
        _arg('gripper_pad_value', '0.0'),
        _arg('gripper_velocity', '100.0'),
        _arg('gripper_effort', '100.0'),
        _arg('gripper_command_deadband', '0.01'),
        _arg('async_policy', 'true'),
        _arg('async_policy_prefetch_after_frames', '8'),
        _arg('async_policy_log_wait_sec', '0.05'),
        _arg('rtc_control_enabled', 'true'),
        _arg('rtc_execution_horizon', '10'),
        _arg('rtc_transition_schedule', 'exp'),
        _arg('rtc_exp_decay', '4.0'),
        _arg('direct_gripper', 'true'),
        _arg('gripper_port', '/tmp/ttyUR'),
        _arg('gripper_baudrate', '115200'),
        _arg('gripper_rate_hz', '30.0'),
        _arg('gripper_position_deadband_raw', '4.0'),
        _arg('gripper_init_timeout_sec', '8.0'),
        _arg('gripper_read_hz', '30.0'),
    ]

    policy_node = Node(
        package='policy_client',
        executable='policy_control',
        name='policy_control',
        output='screen',
        prefix=python_executable,
        parameters=[{
            'policy_base_url': _param('policy_base_url', str),
            'policy_script': _param('policy_script', str),
            'policy_timeout_sec': _param('policy_timeout_sec', float),
            'instruction': _param('instruction', str),
            'robot_ip': _param('robot_ip', str),
            'primary_camera_serial': _param('primary_camera_serial', str),
            'wrist_camera_serial': _param('wrist_camera_serial', str),
            'require_wrist_image': _param('require_wrist_image', bool),
            'autostart': _param('autostart', bool),
            'dry_run': _param('dry_run', bool),
            'rate_hz': _param('rate_hz', float),
            'controller_mode': 'rtde_servoj_joint_abs',
            'joint_command_mode': 'absolute',
            'proprio_mode': 'joint_abs',
            'send_initial_zero_action': False,
            'use_action_chunk': _param('use_action_chunk', bool),
            'chunk_size': _param('chunk_size', int),
            'history_capture_hz': _param('history_capture_hz', float),
            'max_pending_history_frames': _param('max_pending_history_frames', int),
            'joint_point_duration_sec': _param('joint_point_duration_sec', float),
            'max_joint_delta': _param('max_joint_delta', float),
            'rtde_servoj_time_sec': _param('rtde_servoj_time_sec', float),
            'rtde_servoj_control_frequency': _param(
                'rtde_servoj_control_frequency',
                float,
            ),
            'rtde_servoj_max_velocity': _param('rtde_servoj_max_velocity', float),
            'rtde_servoj_max_acceleration': _param(
                'rtde_servoj_max_acceleration',
                float,
            ),
            'rtde_servoj_lookahead_time': _param('rtde_servoj_lookahead_time', float),
            'rtde_servoj_gain': _param('rtde_servoj_gain', float),
            'rtde_servoj_interpolation': _param(
                'rtde_servoj_interpolation',
                str,
            ),
            'rtde_servoj_stop_acceleration': _param(
                'rtde_servoj_stop_acceleration',
                float,
            ),
            'gripper_pad_value': _param('gripper_pad_value', float),
            'gripper_velocity': _param('gripper_velocity', float),
            'gripper_effort': _param('gripper_effort', float),
            'gripper_command_deadband': _param('gripper_command_deadband', float),
            'async_policy': _param('async_policy', bool),
            'async_policy_prefetch_after_frames': _param(
                'async_policy_prefetch_after_frames',
                int,
            ),
            'async_policy_log_wait_sec': _param('async_policy_log_wait_sec', float),
            'rtc_control_enabled': _param('rtc_control_enabled', bool),
            'rtc_execution_horizon': _param('rtc_execution_horizon', int),
            'rtc_transition_schedule': _param('rtc_transition_schedule', str),
            'rtc_exp_decay': _param('rtc_exp_decay', float),
            'direct_gripper': _param('direct_gripper', bool),
            'gripper_port': _param('gripper_port', str),
            'gripper_baudrate': _param('gripper_baudrate', int),
            'gripper_rate_hz': _param('gripper_rate_hz', float),
            'gripper_position_deadband_raw': _param(
                'gripper_position_deadband_raw',
                float,
            ),
            'gripper_init_timeout_sec': _param('gripper_init_timeout_sec', float),
            'gripper_read_hz': _param('gripper_read_hz', float),
        }],
    )

    return LaunchDescription([*launch_args, policy_node])
