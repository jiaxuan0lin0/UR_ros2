"""ROS 2 node that calls a remote policy service and drives a UR via RTDE."""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import importlib
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from policy_client.adapters import (
    JointTrajectoryActionAdapter,
    JointTrajectoryAdapterConfig,
)
from policy_client.controller_clients import DirectRTDEJointServoClient
from policy_client.observation import URRealSenseRTDEObservationSource
from policy_client.rtc_control import RTCControl, RTCControlConfig


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


def _observation_gripper(observation) -> Optional[float]:
    raw_proprio = getattr(observation, 'proprio', None)
    if raw_proprio is not None:
        proprio = np.asarray(raw_proprio, dtype=np.float64)
        if proprio.shape == (7,) and np.isfinite(proprio[6]):
            return float(proprio[6])
    robot_state = getattr(observation, 'robot_state', {}) or {}
    value = robot_state.get('gripper_position')
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def _observation_joint_action(observation) -> Optional[np.ndarray]:
    robot_state = getattr(observation, 'robot_state', {}) or {}
    raw_joints = robot_state.get('joint_positions')
    joints = np.asarray(raw_joints, dtype=np.float64)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raw_proprio = getattr(observation, 'proprio', None)
        proprio = np.asarray(raw_proprio, dtype=np.float64)
        if proprio.shape != (7,) or not np.all(np.isfinite(proprio[:6])):
            return None
        joints = proprio[:6]
    gripper = _observation_gripper(observation)
    if gripper is None:
        return None
    return np.concatenate([joints, np.asarray([gripper], dtype=np.float64)])


class PolicyControlNode(Node):
    """Local execution loop for remote policy inference and direct RTDE motion."""

    def __init__(self):
        super().__init__('policy_control')
        self._declare_parameters()
        self._load_parameters()

        self.policy = self._create_policy_client()
        self.observation_source = URRealSenseRTDEObservationSource(
            robot_ip=self.robot_ip,
            rtde_frequency=self.rtde_frequency,
            color_width=self.color_width,
            color_height=self.color_height,
            color_fps=self.color_fps,
            primary_camera_serial=self.primary_camera_serial,
            wrist_camera_serial=self.wrist_camera_serial,
            require_wrist_image=self.require_wrist_image,
            camera_color_order=self.camera_color_order,
            proprio_mode=self.proprio_mode,
        )
        self._create_controller_binding()
        self.rtc_control = RTCControl(
            RTCControlConfig(
                enabled=self.rtc_control_enabled,
                execution_horizon=self.rtc_execution_horizon,
                schedule=self.rtc_transition_schedule,
                exp_decay=self.rtc_exp_decay,
            )
        )

        self._state_lock = threading.Lock()
        self._policy_lock = threading.Lock()
        self._run_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._connected = False
        self._running = False
        self._last_log_time = 0.0
        self._pending_executed_actions: Optional[list[list[float]]] = None
        self._pending_history_images: Optional[list[np.ndarray]] = None
        self._last_history_capture_log_time = 0.0
        self._async_policy_executor: Optional[ThreadPoolExecutor] = None
        self._async_policy_future: Optional[Future] = None
        self._async_policy_generation = 0
        self._async_policy_lock = threading.Lock()
        if self.async_policy:
            self._async_policy_executor = ThreadPoolExecutor(
                max_workers=1,
                thread_name_prefix='policy_async_infer',
            )

        self.start_service = self.create_service(
            Trigger,
            '/policy_control/start',
            self._start_cb,
        )
        self.stop_service = self.create_service(
            Trigger,
            '/policy_control/stop',
            self._stop_cb,
        )
        self.step_service = self.create_service(
            Trigger,
            '/policy_control/step',
            self._step_cb,
        )
        self.reset_service = self.create_service(
            Trigger,
            '/policy_control/reset_policy',
            self._reset_policy_cb,
        )

        self.get_logger().info(
            'Policy control ready: '
            f'policy={self.policy_base_url}, '
            f'controller={self.controller_action_name}, '
            f'mode={self.controller_mode}, '
            f'autostart={self.autostart}, dry_run={self.dry_run}, '
            f'async_policy={self.async_policy}, '
            f'rtc_control={self.rtc_control.enabled}, '
            f'history_capture_hz={self.history_capture_hz}, '
            f'chunk_size={self.chunk_size}, '
            f'joint_point_duration_sec={self.joint_point_duration_sec:.4f}, '
            f'rtde_servoj_time_sec={self.rtde_servoj_time_sec:.4f}, '
            f'rtde_servoj_control_frequency={self.rtde_servoj_control_frequency:.1f}, '
            f'rtde_servoj_interpolation={self.rtde_servoj_interpolation}, '
            f'max_joint_delta={self.max_joint_delta:.6f}'
        )
        if self.autostart:
            ok, message = self._start_control()
            if not ok:
                self.get_logger().error(f'autostart failed: {message}')

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

    def _create_policy_client(self):
        script_name = self.policy_script.strip()
        if not script_name:
            raise ValueError('policy_script parameter must be non-empty')
        if '.' in script_name:
            module_name = script_name
        else:
            module_name = f'policy_client.scripts.{script_name}'

        module = importlib.import_module(module_name)
        factory = getattr(module, 'create_policy_client', None)
        if factory is None:
            raise ValueError(
                f'{module_name} must define create_policy_client(...)'
            )
        return factory(
            base_url=self.policy_base_url,
            timeout_sec=self.policy_timeout_sec,
            gripper_pad_value=self.gripper_pad_value,
        )

    def _create_controller_binding(self) -> None:
        if self.controller_mode != 'rtde_servoj_joint_abs':
            raise ValueError(
                'minimal policy_client only supports '
                f'controller_mode="rtde_servoj_joint_abs", got {self.controller_mode}'
            )
        if self.joint_command_mode.strip().lower() not in {
            'absolute',
            'abs',
            'position',
            'positions',
        }:
            raise ValueError(
                'rtde_servoj_joint_abs requires joint_command_mode="absolute"'
            )

        self.action_adapter = JointTrajectoryActionAdapter(
            JointTrajectoryAdapterConfig(
                joint_command_mode=self.joint_command_mode,
                use_action_chunk=self.use_action_chunk,
                chunk_size=self.chunk_size,
                point_duration_sec=self.joint_point_duration_sec,
                max_joint_delta=self.max_joint_delta,
                gripper_input_mode=self.gripper_input_mode,
                gripper_raw_min=self.gripper_raw_min,
                gripper_raw_max=self.gripper_raw_max,
            )
        )
        self.controller_action_name = 'direct_rtde_servoj'
        self.controller = DirectRTDEJointServoClient(
            self,
            robot_ip=self.robot_ip,
            servo_time=self.rtde_servoj_time_sec,
            control_frequency=self.rtde_servoj_control_frequency,
            max_velocity=self.rtde_servoj_max_velocity,
            max_acceleration=self.rtde_servoj_max_acceleration,
            lookahead_time=self.rtde_servoj_lookahead_time,
            servo_gain=self.rtde_servoj_gain,
            interpolation_mode=self.rtde_servoj_interpolation,
            max_joint_delta=self.max_joint_delta,
            stop_acceleration=self.rtde_servoj_stop_acceleration,
            gripper_raw_min=self.gripper_raw_min,
            gripper_raw_max=self.gripper_raw_max,
            gripper_velocity=self.gripper_velocity,
            gripper_effort=self.gripper_effort,
            gripper_command_deadband=self.gripper_command_deadband,
            direct_gripper=self.direct_gripper,
            gripper_port=self.gripper_port,
            gripper_baudrate=self.gripper_baudrate,
            gripper_rate_hz=self.gripper_rate_hz,
            gripper_position_deadband_raw=self.gripper_position_deadband_raw,
            gripper_init_timeout_sec=self.gripper_init_timeout_sec,
            gripper_read_hz=self.gripper_read_hz,
            wait_timeout_sec=self.controller_wait_timeout_sec,
            result_timeout_sec=self.controller_result_timeout_sec,
        )
        return

    def _start_cb(self, _request, response):
        ok, message = self._start_control()
        response.success = ok
        response.message = message
        return response

    def _stop_cb(self, _request, response):
        ok, message = self._stop_control()
        response.success = ok
        response.message = message
        return response

    def _step_cb(self, _request, response):
        if self._is_running():
            response.success = False
            response.message = 'policy control loop is already running'
            return response

        try:
            ok, message = self._check_controller_ready()
            if not ok:
                response.success = False
                response.message = message
                return response
            self._ensure_connected()
            if self.reset_policy_on_start:
                self._reset_policy_history()
            ok, message = self._execute_step()
        except Exception as exc:
            ok = False
            message = str(exc)
        response.success = ok
        response.message = message
        return response

    def _reset_policy_cb(self, _request, response):
        try:
            self._reset_policy_history()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = 'policy history reset'
        return response

    def _start_control(self) -> tuple[bool, str]:
        instruction = self.instruction.strip()
        if not instruction:
            return False, 'instruction parameter must be non-empty'

        with self._state_lock:
            if self._running:
                return False, 'policy control loop is already running'
            self._running = True
            self._stop_event.clear()
            self._pending_executed_actions = (
                [[0.0] * 7] if self.send_initial_zero_action else None
            )
        self._clear_async_policy_request(increment_generation=True)

        try:
            ok, message = self._check_controller_ready()
            if not ok:
                raise RuntimeError(message)
            self._ensure_connected()
            if self.reset_policy_on_start:
                self._reset_policy_history()
            self.rtc_control.reset()
        except Exception as exc:
            with self._state_lock:
                self._running = False
            return False, f'failed to start policy control: {exc}'

        self._run_thread = threading.Thread(
            target=self._run_loop,
            name='policy_control_loop',
            daemon=True,
        )
        self._run_thread.start()
        return True, 'policy control loop started'

    def _stop_control(self) -> tuple[bool, str]:
        with self._state_lock:
            was_running = self._running
            self._running = False
            self._stop_event.set()

        if self._run_thread is not None and self._run_thread.is_alive():
            self._run_thread.join(timeout=2.0)
        self._clear_async_policy_request(increment_generation=True)
        self._stop_controller_motion()
        if not was_running:
            return False, 'policy control loop was not running'
        return True, 'policy control loop stopped'

    def _stop_controller_motion(self) -> None:
        stop = getattr(self.controller, 'stop', None)
        if not callable(stop):
            return
        try:
            stop()
        except Exception as exc:
            self.get_logger().warn(f'controller stop failed: {exc}')

    def _run_loop(self) -> None:
        period_sec = 1.0 / max(self.rate_hz, 0.01)
        try:
            while not self._stop_event.is_set() and rclpy.ok():
                started = time.monotonic()
                try:
                    ok, message = self._execute_step()
                except Exception as exc:
                    ok = False
                    message = str(exc)

                if not ok:
                    self.get_logger().error(f'policy control step failed: {message}')
                    if self.stop_on_error:
                        break
                else:
                    self._log_step(message)

                remaining = period_sec - (time.monotonic() - started)
                if remaining > 0.0:
                    self._stop_event.wait(timeout=remaining)
        finally:
            with self._state_lock:
                self._running = False

    def _execute_step(self) -> tuple[bool, str]:
        prefetched = self._consume_async_policy_result()
        if prefetched is None:
            observation = self.observation_source.read()
            self._apply_gripper_observation_override(observation)
            history_actions = self._history_actions_for_request()
            history_images = self._history_images_for_request()
            result = self._call_policy(
                observation=observation,
                history_actions=history_actions,
                history_images=history_images,
            )
            self._mark_history_actions_sent(history_actions)
            self._mark_history_images_sent(history_images)
            infer_note = 'sync'
        else:
            observation = prefetched['observation']
            result = prefetched['result']
            inference_latency_sec = prefetched['latency_sec']
            infer_note = (
                'async '
                f"age={time.monotonic() - prefetched['completed_at']:.3f}s "
                f"latency={prefetched['latency_sec']:.3f}s"
            )
        if prefetched is None:
            inference_latency_sec = None

        safe_actions = self.action_adapter.select_actions(result)
        safe_actions, rtc_info = self.rtc_control.align(
            safe_actions,
            action_period_sec=self.joint_point_duration_sec,
            inference_latency_sec=inference_latency_sec,
        )
        action_summary = self._action_summary(
            safe_actions,
            result,
            observation,
        )
        rtc_summary = self._rtc_summary(rtc_info)
        if rtc_summary:
            action_summary = f'{action_summary}, {rtc_summary}'
        self.get_logger().info(f'{action_summary}, inference={infer_note}')
        if self.dry_run:
            return True, f'dry-run {action_summary}'

        goal = self.action_adapter.build_goal(
            safe_actions,
            observation=observation,
        )
        execution_observations: list[object] = []
        last_capture_time = time.monotonic()
        prefetch_started = False

        def capture_execution_observation() -> None:
            nonlocal last_capture_time, prefetch_started
            if self.history_capture_hz <= 0.0:
                return
            now = time.monotonic()
            min_period = 1.0 / max(self.history_capture_hz, 0.01)
            if now - last_capture_time < min_period:
                return
            last_capture_time = now
            captured = self._read_optional_observation()
            if captured is None:
                return
            execution_observations.append(captured)
            if (
                self.async_policy
                and not prefetch_started
                and len(execution_observations)
                >= self.async_policy_prefetch_after_frames
            ):
                prefetch_started = self._start_async_policy_request(
                    start_observation=observation,
                    execution_observations=list(execution_observations),
                )

        ok, message = self.controller.send_goal(
            goal,
            wait_for_result=self.wait_for_controller_result,
            wait_callback=(
                capture_execution_observation
                if self.wait_for_controller_result
                else None
            ),
            wait_poll_sec=1.0 / max(self.history_capture_hz, 1.0),
        )
        if ok and self.wait_for_controller_result:
            final_observation = self._read_optional_observation()
            if final_observation is not None:
                execution_observations.append(final_observation)
            if self.async_policy and not prefetch_started:
                prefetch_started = self._start_async_policy_request(
                    start_observation=observation,
                    execution_observations=list(execution_observations),
                )
            self._store_execution_history(
                start_observation=observation,
                execution_observations=execution_observations,
            )
        if ok:
            self.rtc_control.commit(safe_actions)
        if ok:
            suffix = '; async_prefetch=started' if prefetch_started else ''
            return True, f'{message}; actions={len(safe_actions)}{suffix}'
        return False, message

    def _call_policy(
        self,
        *,
        observation,
        history_actions,
        history_images,
    ):
        with self._policy_lock:
            return self.policy.act(
                instruction=self.instruction,
                observation=observation,
                executed_actions=history_actions,
                history_images=history_images,
            )

    def _start_async_policy_request(
        self,
        *,
        start_observation,
        execution_observations: list[object],
    ) -> bool:
        if not self.async_policy or self._async_policy_executor is None:
            return False
        if not execution_observations:
            return False

        request = self._build_async_policy_request(
            start_observation=start_observation,
            execution_observations=execution_observations,
        )
        if request is None:
            return False

        with self._async_policy_lock:
            if (
                self._async_policy_future is not None
                and not self._async_policy_future.done()
            ):
                return False
            generation = self._async_policy_generation
            future = self._async_policy_executor.submit(
                self._run_async_policy_request,
                generation,
                request,
            )
            self._async_policy_future = future
        return True

    def _build_async_policy_request(
        self,
        *,
        start_observation,
        execution_observations: list[object],
    ) -> Optional[dict]:
        observations = [obs for obs in execution_observations if obs is not None]
        if not observations:
            return None

        actions = self._actual_actions_from_observations(
            start_observation,
            observations,
        )
        images = [
            np.asarray(obs.primary_image).copy()
            for obs in observations
            if getattr(obs, 'primary_image', None) is not None
        ]
        if self.max_pending_history_frames > 0:
            images = images[-self.max_pending_history_frames:]

        return {
            'observation': self._clone_policy_observation(observations[-1]),
            'history_actions': actions or None,
            'history_images': images or None,
        }

    def _run_async_policy_request(self, generation: int, request: dict) -> dict:
        started_at = time.monotonic()
        result = self._call_policy(
            observation=request['observation'],
            history_actions=request['history_actions'],
            history_images=request['history_images'],
        )
        completed_at = time.monotonic()
        return {
            'generation': generation,
            'observation': request['observation'],
            'result': result,
            'started_at': started_at,
            'completed_at': completed_at,
            'latency_sec': completed_at - started_at,
        }

    def _consume_async_policy_result(self) -> Optional[dict]:
        if not self.async_policy:
            return None
        with self._async_policy_lock:
            future = self._async_policy_future
            generation = self._async_policy_generation
        if future is None:
            return None

        started_wait = time.monotonic()
        try:
            payload = future.result()
        except Exception:
            with self._async_policy_lock:
                if self._async_policy_future is future:
                    self._async_policy_future = None
            raise
        finally:
            waited = time.monotonic() - started_wait
            if waited >= self.async_policy_log_wait_sec:
                self.get_logger().info(
                    f'waited {waited:.3f}s for async policy result'
                )

        with self._async_policy_lock:
            if self._async_policy_future is future:
                self._async_policy_future = None
        if payload.get('generation') != generation:
            return None

        with self._state_lock:
            self._pending_executed_actions = None
            self._pending_history_images = None
        return payload

    def _clear_async_policy_request(self, *, increment_generation: bool) -> None:
        with self._async_policy_lock:
            if increment_generation:
                self._async_policy_generation += 1
            future = self._async_policy_future
            self._async_policy_future = None
        if future is not None and not future.done():
            future.cancel()

    @staticmethod
    def _clone_policy_observation(observation):
        from policy_client.types import PolicyObservation

        return PolicyObservation(
            primary_image=np.asarray(observation.primary_image).copy(),
            proprio=np.asarray(observation.proprio, dtype=np.float64).copy(),
            wrist_image=(
                None
                if getattr(observation, 'wrist_image', None) is None
                else np.asarray(observation.wrist_image).copy()
            ),
            camera_serials=tuple(getattr(observation, 'camera_serials', ())),
            robot_state=dict(getattr(observation, 'robot_state', {}) or {}),
        )

    def _action_summary(
        self,
        actions: list[np.ndarray],
        result,
        observation: Optional[object] = None,
    ) -> str:
        raw_count = int(
            getattr(result.action_chunk, 'shape', [len(actions)])[0]
        )
        obs_summary = self._observation_summary(observation)
        if not actions:
            return (
                f'policy action chunk: raw_actions={raw_count}, '
                f'selected_actions=0{obs_summary}'
            )
        first = np.array2string(actions[0], precision=4)
        last = np.array2string(actions[-1], precision=4)
        return (
            f'policy action chunk: raw_actions={raw_count}, '
            f'selected_actions={len(actions)}{obs_summary}, '
            f'first={first}, last={last}'
        )

    @staticmethod
    def _observation_summary(observation: Optional[object]) -> str:
        if observation is None:
            return ''
        proprio = getattr(observation, 'proprio', None)
        if proprio is None:
            return ''
        values = np.asarray(proprio, dtype=np.float64)
        if values.shape[0] < 7:
            return ''
        first3 = np.array2string(values[:3], precision=4)
        return f', proprio0_2={first3}, gripper={values[6]:.3f}'

    def _apply_gripper_observation_override(
        self,
        observation,
    ) -> None:
        current = getattr(self.controller, 'current_gripper_position', None)
        if not callable(current):
            return
        position = current()
        if position is None:
            return

        proprio = np.asarray(observation.proprio, dtype=np.float64).copy()
        if proprio.shape != (7,):
            return
        proprio[6] = float(position)
        observation.proprio = proprio

        robot_state = dict(getattr(observation, 'robot_state', {}) or {})
        robot_state['gripper_position'] = float(position)
        robot_state['gripper_source'] = 'direct_dh_gripper'
        observation.robot_state = robot_state

    def _ensure_connected(self) -> None:
        with self._state_lock:
            if self._connected:
                return
        health = self.policy.health()
        self.get_logger().info(f'policy health: {health}')
        self.observation_source.connect()
        with self._state_lock:
            self._connected = True

    def _check_controller_ready(self) -> tuple[bool, str]:
        if self.dry_run:
            return True, 'dry-run does not require controller action server'
        if not self.controller.wait_ready():
            return (
                False,
                'controller action server not ready: '
                f'{self.controller_action_name}',
            )
        return True, 'controller action server ready'

    def _reset_policy_history(self) -> None:
        self._clear_async_policy_request(increment_generation=True)
        with self._policy_lock:
            self.policy.reset()
        self.rtc_control.reset()
        with self._state_lock:
            self._pending_executed_actions = (
                [[0.0] * 7] if self.send_initial_zero_action else None
            )
            self._pending_history_images = None

    def _read_optional_observation(self):
        try:
            observation = self.observation_source.read()
            self._apply_gripper_observation_override(observation)
            return observation
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_history_capture_log_time >= 2.0:
                self._last_history_capture_log_time = now
                self.get_logger().warn(
                    f'Could not capture execution history observation: {exc}'
                )
        return None

    def _history_actions_for_request(self) -> Optional[list[list[float]]]:
        if self.history_update_mode == 'server_auto':
            return None
        if self.history_update_mode != 'last_action':
            raise ValueError(
                'history_update_mode must be "last_action" or "server_auto"'
            )
        with self._state_lock:
            return self._pending_executed_actions

    def _history_images_for_request(self) -> Optional[list[np.ndarray]]:
        with self._state_lock:
            return self._pending_history_images

    def _mark_history_actions_sent(
        self,
        actions: Optional[list[list[float]]],
    ) -> None:
        if actions is None or self.history_update_mode != 'last_action':
            return
        with self._state_lock:
            if self._pending_executed_actions is actions:
                self._pending_executed_actions = None

    def _mark_history_images_sent(
        self,
        images: Optional[list[np.ndarray]],
    ) -> None:
        if images is None:
            return
        with self._state_lock:
            if self._pending_history_images is images:
                self._pending_history_images = None

    def _store_execution_history(
        self,
        *,
        start_observation,
        execution_observations: list[object],
    ) -> None:
        if self.history_update_mode != 'last_action':
            return

        observations = [obs for obs in execution_observations if obs is not None]
        if not observations:
            return

        actions = self._actual_actions_from_observations(
            start_observation,
            observations,
        )
        images = [
            np.asarray(obs.primary_image).copy()
            for obs in observations
            if getattr(obs, 'primary_image', None) is not None
        ]
        if self.max_pending_history_frames > 0:
            images = images[-self.max_pending_history_frames:]

        with self._state_lock:
            if actions:
                self._pending_executed_actions = actions
            if images:
                self._pending_history_images = images

    def _actual_actions_from_observations(
        self,
        start_observation,
        observations: list[object],
    ) -> list[list[float]]:
        del start_observation
        actions: list[list[float]] = []
        for observation in observations:
            action = _observation_joint_action(observation)
            if action is not None:
                actions.append(action.astype(float).tolist())
        return actions

    def _is_running(self) -> bool:
        with self._state_lock:
            return self._running

    def _log_step(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_log_time < 1.0:
            return
        self._last_log_time = now
        self.get_logger().info(message)

    @staticmethod
    def _rtc_summary(info) -> str:
        if info is None or not getattr(info, 'enabled', False):
            return ''
        return (
            f"rtc_prefix={info.prefix_count}, "
            f"rtc_prev={info.previous_count}, "
            f"rtc_delay_steps={info.inference_delay_steps}, "
            f"rtc_schedule={info.schedule}"
        )

    def destroy_node(self):
        self._stop_control()
        close = getattr(self.controller, 'close', None)
        if callable(close):
            try:
                close()
            except Exception as exc:
                self.get_logger().warn(f'controller close failed: {exc}')
        self._clear_async_policy_request(increment_generation=True)
        if self._async_policy_executor is not None:
            self._async_policy_executor.shutdown(wait=False, cancel_futures=True)
            self._async_policy_executor = None
        self.observation_source.disconnect()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyControlNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
