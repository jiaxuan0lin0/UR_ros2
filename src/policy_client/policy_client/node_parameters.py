"""ROS parameter declaration and loading for the policy control node."""

from __future__ import annotations


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {'1', 'true', 't', 'yes', 'y', 'on'}:
            return True
        if normalized in {'0', 'false', 'f', 'no', 'n', 'off', ''}:
            return False
    return bool(value)


_CONTROLLER_MODE_ALIASES = {
    'joint_trajectory': 'rtde_servoj_joint_abs',
    'joint': 'rtde_servoj_joint_abs',
    'joint_control': 'rtde_servoj_joint_abs',
    'rtde_servoj_joint_abs': 'rtde_servoj_joint_abs',
    'rtde_servoj': 'rtde_servoj_joint_abs',
    'direct_rtde': 'rtde_servoj_joint_abs',
    'direct_joint': 'rtde_servoj_joint_abs',
    'servo_j': 'rtde_servoj_joint_abs',
}


def _normalize_controller_mode(value: str) -> str:
    normalized = str(value).strip().lower()
    if normalized in _CONTROLLER_MODE_ALIASES:
        return _CONTROLLER_MODE_ALIASES[normalized]
    raise ValueError(
        'controller_mode must be "rtde_servoj_joint_abs", '
        f'got {value}'
    )


class NodeParametersMixin:
    """Declare the policy_control ROS parameters and load them onto the node."""

    def _declare_parameters(self) -> None:
        self.declare_parameter('policy_base_url', 'http://127.0.0.1:8777')
        self.declare_parameter('policy_script', '')
        self.declare_parameter('policy_timeout_sec', 30.0)
        self.declare_parameter('instruction', '')
        self.declare_parameter('rate_hz', 0.6)
        self.declare_parameter('autostart', False)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('reset_policy_on_start', True)
        self.declare_parameter('stop_on_error', True)
        self.declare_parameter('history_update_mode', 'last_action')
        self.declare_parameter('send_initial_zero_action', False)
        self.declare_parameter('async_policy', True)
        self.declare_parameter('async_policy_prefetch_after_frames', 8)
        self.declare_parameter('async_policy_log_wait_sec', 0.05)
        self.declare_parameter('rtc_control_enabled', False)
        self.declare_parameter('rtc_execution_horizon', 10)
        self.declare_parameter('rtc_transition_schedule', 'exp')
        self.declare_parameter('rtc_exp_decay', 4.0)

        self.declare_parameter('robot_ip', '192.168.1.102')
        self.declare_parameter('rtde_frequency', -1.0)
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('primary_camera_serial', '')
        self.declare_parameter('wrist_camera_serial', '')
        self.declare_parameter('require_wrist_image', True)
        self.declare_parameter('camera_color_order', 'bgr')
        self.declare_parameter('proprio_mode', 'joint_abs')
        self.declare_parameter('gripper_pad_value', 0.0)

        self.declare_parameter('controller_mode', 'rtde_servoj_joint_abs')
        self.declare_parameter('use_action_chunk', True)
        self.declare_parameter('chunk_size', 50)
        self.declare_parameter('history_capture_hz', 30.0)
        self.declare_parameter('max_pending_history_frames', 8)
        self.declare_parameter('joint_command_mode', 'absolute')
        self.declare_parameter('joint_point_duration_sec', 1.0 / 30.0)
        self.declare_parameter('max_joint_delta', 0.00136)
        self.declare_parameter('rtde_servoj_time_sec', 1.0 / 500.0)
        self.declare_parameter('rtde_servoj_control_frequency', 500.0)
        self.declare_parameter('rtde_servoj_max_velocity', 1.05)
        self.declare_parameter('rtde_servoj_max_acceleration', 1.4)
        self.declare_parameter('rtde_servoj_lookahead_time', 0.06)
        self.declare_parameter('rtde_servoj_gain', 500.0)
        self.declare_parameter('rtde_servoj_interpolation', 'pchip')
        self.declare_parameter('rtde_servoj_stop_acceleration', 2.0)
        self.declare_parameter('gripper_command_deadband', 0.01)
        self.declare_parameter('gripper_input_mode', 'normalized')
        self.declare_parameter('gripper_raw_min', 0.0)
        self.declare_parameter('gripper_raw_max', 1000.0)
        self.declare_parameter('gripper_velocity', 100.0)
        self.declare_parameter('gripper_effort', 100.0)
        self.declare_parameter('direct_gripper', True)
        self.declare_parameter('gripper_port', '/tmp/ttyUR')
        self.declare_parameter('gripper_baudrate', 115200)
        self.declare_parameter('gripper_rate_hz', 30.0)
        self.declare_parameter('gripper_position_deadband_raw', 4.0)
        self.declare_parameter('gripper_init_timeout_sec', 8.0)
        self.declare_parameter('gripper_read_hz', 30.0)
        self.declare_parameter('controller_wait_timeout_sec', 5.0)
        self.declare_parameter('controller_result_timeout_sec', 15.0)
        self.declare_parameter('wait_for_controller_result', True)

    def _load_parameters(self) -> None:
        self.policy_base_url = str(self.get_parameter('policy_base_url').value)
        self.policy_script = str(self.get_parameter('policy_script').value)
        self.policy_timeout_sec = float(
            self.get_parameter('policy_timeout_sec').value
        )
        self.instruction = str(self.get_parameter('instruction').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.autostart = _as_bool(self.get_parameter('autostart').value)
        self.dry_run = _as_bool(self.get_parameter('dry_run').value)
        self.reset_policy_on_start = _as_bool(
            self.get_parameter('reset_policy_on_start').value
        )
        self.stop_on_error = _as_bool(self.get_parameter('stop_on_error').value)
        self.history_update_mode = str(
            self.get_parameter('history_update_mode').value
        ).strip()
        self.send_initial_zero_action = _as_bool(
            self.get_parameter('send_initial_zero_action').value
        )
        self.async_policy = _as_bool(self.get_parameter('async_policy').value)
        self.async_policy_prefetch_after_frames = max(
            1,
            int(self.get_parameter('async_policy_prefetch_after_frames').value),
        )
        self.async_policy_log_wait_sec = max(
            0.0,
            float(self.get_parameter('async_policy_log_wait_sec').value),
        )
        self.rtc_control_enabled = _as_bool(
            self.get_parameter('rtc_control_enabled').value
        )
        self.rtc_execution_horizon = max(
            0,
            int(self.get_parameter('rtc_execution_horizon').value),
        )
        self.rtc_transition_schedule = str(
            self.get_parameter('rtc_transition_schedule').value
        )
        self.rtc_exp_decay = max(
            0.0,
            float(self.get_parameter('rtc_exp_decay').value),
        )

        self.robot_ip = str(self.get_parameter('robot_ip').value)
        self.rtde_frequency = float(self.get_parameter('rtde_frequency').value)
        self.color_width = int(self.get_parameter('color_width').value)
        self.color_height = int(self.get_parameter('color_height').value)
        self.color_fps = int(self.get_parameter('color_fps').value)
        self.primary_camera_serial = str(
            self.get_parameter('primary_camera_serial').value
        )
        self.wrist_camera_serial = str(
            self.get_parameter('wrist_camera_serial').value
        )
        self.require_wrist_image = _as_bool(
            self.get_parameter('require_wrist_image').value
        )
        self.camera_color_order = str(
            self.get_parameter('camera_color_order').value
        )
        self.proprio_mode = str(self.get_parameter('proprio_mode').value)
        self.gripper_pad_value = float(
            self.get_parameter('gripper_pad_value').value
        )

        self.controller_mode = _normalize_controller_mode(
            str(self.get_parameter('controller_mode').value)
        )
        self.use_action_chunk = _as_bool(
            self.get_parameter('use_action_chunk').value
        )
        self.chunk_size = int(self.get_parameter('chunk_size').value)
        self.history_capture_hz = float(
            self.get_parameter('history_capture_hz').value
        )
        self.max_pending_history_frames = int(
            self.get_parameter('max_pending_history_frames').value
        )
        self.joint_command_mode = str(
            self.get_parameter('joint_command_mode').value
        )
        self.joint_point_duration_sec = float(
            self.get_parameter('joint_point_duration_sec').value
        )
        self.max_joint_delta = float(
            self.get_parameter('max_joint_delta').value
        )
        self.rtde_servoj_time_sec = float(
            self.get_parameter('rtde_servoj_time_sec').value
        )
        self.rtde_servoj_control_frequency = float(
            self.get_parameter('rtde_servoj_control_frequency').value
        )
        self.rtde_servoj_max_velocity = float(
            self.get_parameter('rtde_servoj_max_velocity').value
        )
        self.rtde_servoj_max_acceleration = float(
            self.get_parameter('rtde_servoj_max_acceleration').value
        )
        self.rtde_servoj_lookahead_time = float(
            self.get_parameter('rtde_servoj_lookahead_time').value
        )
        self.rtde_servoj_gain = float(
            self.get_parameter('rtde_servoj_gain').value
        )
        self.rtde_servoj_interpolation = str(
            self.get_parameter('rtde_servoj_interpolation').value
        )
        self.rtde_servoj_stop_acceleration = float(
            self.get_parameter('rtde_servoj_stop_acceleration').value
        )
        self.gripper_command_deadband = float(
            self.get_parameter('gripper_command_deadband').value
        )
        self.gripper_input_mode = str(
            self.get_parameter('gripper_input_mode').value
        )
        self.gripper_raw_min = float(
            self.get_parameter('gripper_raw_min').value
        )
        self.gripper_raw_max = float(
            self.get_parameter('gripper_raw_max').value
        )
        self.gripper_velocity = float(
            self.get_parameter('gripper_velocity').value
        )
        self.gripper_effort = float(self.get_parameter('gripper_effort').value)
        self.direct_gripper = _as_bool(self.get_parameter('direct_gripper').value)
        self.gripper_port = str(self.get_parameter('gripper_port').value)
        self.gripper_baudrate = int(self.get_parameter('gripper_baudrate').value)
        self.gripper_rate_hz = float(self.get_parameter('gripper_rate_hz').value)
        self.gripper_position_deadband_raw = float(
            self.get_parameter('gripper_position_deadband_raw').value
        )
        self.gripper_init_timeout_sec = float(
            self.get_parameter('gripper_init_timeout_sec').value
        )
        self.gripper_read_hz = float(self.get_parameter('gripper_read_hz').value)
        self.controller_wait_timeout_sec = float(
            self.get_parameter('controller_wait_timeout_sec').value
        )
        self.controller_result_timeout_sec = float(
            self.get_parameter('controller_result_timeout_sec').value
        )
        self.wait_for_controller_result = _as_bool(
            self.get_parameter('wait_for_controller_result').value
        )
