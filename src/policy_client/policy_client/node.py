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
from policy_client.async_policy import AsyncPolicyMixin
from policy_client.control_loop import ControlLoopMixin
from policy_client.controller_clients import DirectRTDEJointServoClient
from policy_client.execution_history import ExecutionHistoryMixin
from policy_client.node_parameters import NodeParametersMixin
from policy_client.observation import URRealSenseRTDEObservationSource
from policy_client.rtc_control import RTCControl, RTCControlConfig
from policy_client.step_summary import (
    format_action_summary,
    format_rtc_summary,
)


# Mixins come before Node so their methods take precedence over Node's.
class PolicyControlNode(
    NodeParametersMixin,
    ControlLoopMixin,
    AsyncPolicyMixin,
    ExecutionHistoryMixin,
    Node,
):
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
        action_summary = format_action_summary(
            safe_actions,
            result,
            observation,
        )
        rtc_summary = format_rtc_summary(rtc_info)
        if rtc_summary:
            action_summary = f'{action_summary}, {rtc_summary}'
        self.get_logger().info(f'{action_summary}, inference={infer_note}')
        if self.dry_run:
            return True, f'dry-run {action_summary}'
        if self._stop_event.is_set():
            return False, 'stop requested; policy actions not executed'

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
                and not self._stop_event.is_set()
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
            abort_event=self._stop_event,
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
