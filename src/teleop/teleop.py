#!/usr/bin/env python3
import signal
import sys
import time
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.cli import build_arg_parser, config_from_args
from teleop.config import TeleopConfig
from teleop.gripper_direct import DirectDHGripper
from teleop.local_kinematics import LocalURKinematics
from teleop.motion_math import (
    apply_deadband,
    dls_joint_velocity,
    limit_vector_norm,
    nearest_joint_equivalent,
    scale_to_component_limit,
)
from teleop.pose_mapper import PoseMapper
from teleop.recorder_control import RecorderControl
from teleop.replay import ReplayMixin
from teleop.return_home import ReturnHomeMixin
from teleop.rtde_arm import RTDEArm
from teleop.ruckig_smoothing import RuckigPositionSmoother
from teleop.runtime import configure_low_latency_runtime, configure_xr_service_runtime
from teleop.safety import SafetyLimiter
from teleop.state_writer import StateWriter
from teleop.transforms import (
    matrix_to_rotvec,
    parts_to_pose_vector,
    pose_vector_to_parts,
    rotvec_to_matrix,
)
from teleop.xr_input import XrInput


class TeleopRunner(ReplayMixin, ReturnHomeMixin):
    def __init__(self, config: TeleopConfig):
        self.config = config
        self.stop_requested = False
        self.xr = XrInput()
        self.arm = RTDEArm(
            robot_ip=config.arm.robot_ip,
            servo_time=1.0 / max(config.arm.rate_hz, 1.0),
            control_frequency=max(config.arm.rate_hz, 1.0),
            max_velocity=config.arm.max_velocity,
            max_acceleration=config.arm.max_acceleration,
            lookahead_time=config.arm.lookahead_time,
            servo_gain=config.arm.servo_gain,
            speedj_backend=config.arm.speedj_backend,
            speedj_register_start=config.arm.speedj_register_start,
            speedj_velocity_scale=config.arm.speedj_velocity_scale,
            speedj_watchdog_sec=config.arm.speedj_watchdog_sec,
            speedj_stale_stop_sec=config.arm.speedj_stale_stop_sec,
            speedj_upload_port=config.arm.speedj_upload_port,
            speed_slider_fraction=config.arm.speed_slider_fraction,
        )
        self.mapper = PoseMapper(
            scale_factor=config.xr.scale_factor,
            rotation_scale=config.xr.rotation_scale,
        )
        self.tcp_offset = None
        self.local_kinematics = None
        if (
            config.arm.control_mode == "speed_j_dls"
            and config.arm.dls_jacobian_source == "local"
        ):
            self.tcp_offset = self.arm.get_tcp_offset()
            self.local_kinematics = LocalURKinematics(
                ur_type=config.arm.ur_type,
                frame_name=config.arm.dls_frame_name,
                ur_description_path=config.arm.ur_description_path,
            )
            tcp_offset_text = (
                "unavailable"
                if self.tcp_offset is None
                else np.array2string(
                    self.tcp_offset,
                    precision=5,
                    suppress_small=True,
                )
            )
            print(
                "Local DLS kinematics ready: "
                f"ur_type={config.arm.ur_type}, frame={config.arm.dls_frame_name}, "
                f"active_tcp_offset={tcp_offset_text}, "
                f"urdf={self.local_kinematics.urdf_path}"
            )
            self._check_local_kinematics_alignment()
        if config.arm.control_mode == "speed_j_dls":
            if (
                config.arm.speedj_backend == "register_bridge"
                and config.arm.dls_jacobian_source != "local"
            ):
                raise RuntimeError(
                    "speedj register_bridge replaces the RTDE control script, so "
                    "use --dls-jacobian-source local with this backend."
                )
        self.safety = SafetyLimiter(
            max_step_translation=config.safety.max_step_translation,
            max_step_rotation=config.safety.max_step_rotation,
            max_joint_step=config.safety.max_joint_step,
            target_smoothing_sec=config.safety.target_smoothing_sec,
            max_target_lag_translation=config.safety.max_target_lag_translation,
            max_target_lag_rotation=config.safety.max_target_lag_rotation,
            workspace_min=config.safety.workspace_min,
            workspace_max=config.safety.workspace_max,
        )
        self.target_smoothing_mode = str(config.safety.target_smoothing_mode)
        self.ruckig_position_smoother = None
        if self.target_smoothing_mode == "ruckig":
            max_step_speed = 0.0
            if self.arm.servo_time > 0.0 and config.safety.max_step_translation > 0.0:
                max_step_speed = config.safety.max_step_translation / self.arm.servo_time
            max_linear_speed = abs(float(config.arm.max_tcp_linear_speed))
            if max_step_speed > 0.0:
                max_linear_speed = (
                    max_step_speed
                    if max_linear_speed <= 0.0
                    else min(max_linear_speed, max_step_speed)
                )
            self.ruckig_position_smoother = RuckigPositionSmoother(
                cycle_time=self.arm.servo_time,
                max_velocity=max_linear_speed if max_linear_speed > 0.0 else 0.25,
                max_acceleration=config.safety.ruckig_max_linear_acceleration,
                max_jerk=config.safety.ruckig_max_linear_jerk,
                prediction_sec=config.safety.ruckig_target_prediction_sec,
                velocity_hold_sec=config.safety.ruckig_velocity_hold_sec,
            )
            print(
                "Ruckig translation smoothing active: "
                f"cycle={self.arm.servo_time * 1000.0:.2f}ms, "
                f"max_v={self.ruckig_position_smoother.max_velocity:.3f}m/s, "
                f"max_a={self.ruckig_position_smoother.max_acceleration:.3f}m/s^2, "
                f"max_j={self.ruckig_position_smoother.max_jerk:.3f}m/s^3, "
                f"prediction={self.ruckig_position_smoother.prediction_sec * 1000.0:.1f}ms, "
                f"velocity_hold={self.ruckig_position_smoother.velocity_hold_sec * 1000.0:.1f}ms. "
                "Rotation uses max-step limiting, not Ruckig."
            )
        self.gripper = None
        if config.gripper.enabled:
            self.gripper = DirectDHGripper(
                port=config.gripper.port,
                baudrate=config.gripper.baudrate,
                rate_hz=config.gripper.rate_hz,
                speed=config.gripper.speed,
                force=config.gripper.force,
                position_deadband=config.gripper.position_deadband,
                init_timeout_sec=config.gripper.init_timeout_sec,
                read_state_hz=config.gripper.read_state_hz,
            )
            self.gripper.set_target(config.gripper.open_raw)
        self.state_writer = (
            StateWriter(config.state.path, config.state.rate_hz)
            if config.state.enabled
            else None
        )
        self.recorder_control = None
        if config.recorder.enabled:
            self.recorder_control = RecorderControl(
                button_map={
                    "start": config.recorder.start_button,
                    "save": config.recorder.save_button,
                    "discard": config.recorder.discard_button,
                },
                debounce_sec=config.recorder.debounce_sec,
            )
        self.return_home_target_q = np.array(
            config.return_home.joint_positions,
            dtype=np.float64,
        )
        self.return_home_active = False
        self.return_home_started_at = None
        self.return_home_reached_since = None
        self._force_next_state_write = False
        self._stop_button_was_pressed = False
        self._last_stop_button_time = 0.0
        self._start_button_was_pressed = False
        self._last_replay_capture_time = None
        self._replay_capture_started_at = None
        self._replay_episode_armed = False
        self._replay_ready = False
        self._replay_button_was_pressed = False
        self.replay_buffer = []
        self.replay_pending_until = None
        self.replay_active = False
        self.replay_started_at = None
        self._replay_duration = 0.0
        self._replay_index = 0
        self._replay_gripper_index = 0
        self._motion_locked_until_grip_release = False
        self.previous_target_tcp = None
        self.previous_target_q = None
        self.enabled = False
        self._grip_active = False
        self._grip_below_release_since = None
        self._last_motion_grip_value = None
        self._last_motion_enabled = False
        self._last_xr_age_sec = None
        self._xr_sample_updated = True
        self._last_xr_target_interpolated = False
        self._last_raw_target_tcp = None
        self._last_raw_target_time = None
        self._target_interpolation_anchor_tcp = None
        self._target_interpolation_anchor_time = None
        self._target_interpolation_linear_velocity = np.zeros(3, dtype=np.float64)
        self._target_interpolation_angular_velocity = np.zeros(3, dtype=np.float64)
        self._last_xr_target_interpolation_age_sec = None
        self._last_warn_time = 0.0
        self.control_latency_warning_sec = max(
            0.0,
            float(config.arm.control_latency_warning_sec),
        )
        self._last_xr_timestamp = None
        self._last_xr_update_wall = time.monotonic()
        self._previous_joint_velocity = np.zeros(6, dtype=np.float64)
        self._last_speed_cycle_time = None
        self._last_target_cycle_time = None
        self._active_motion_kind = None
        self._latest_actual_q = None
        self._latest_actual_tcp = None
        self._latest_actual_time = 0.0

    def _check_local_kinematics_alignment(self):
        if self.local_kinematics is None:
            return
        current_q = self.arm.get_current_joints()
        rtde_tcp = self.arm.get_current_tcp_pose()
        local_tcp = self.local_kinematics.forward_pose(current_q, self.tcp_offset)

        rtde_position, rtde_rotation = pose_vector_to_parts(rtde_tcp)
        local_position, local_rotation = pose_vector_to_parts(local_tcp)
        position_error = float(np.linalg.norm(local_position - rtde_position))
        rotation_error = float(
            np.linalg.norm(matrix_to_rotvec(local_rotation @ rtde_rotation.T))
        )
        print(
            "Local DLS FK check against RTDE TCP: "
            f"position_error={position_error:.4f}m, "
            f"rotation_error={rotation_error:.4f}rad"
        )
        if position_error > 0.08 or rotation_error > 0.35:
            raise RuntimeError(
                "Local DLS kinematics is not aligned with RTDE TCP pose. "
                "Check UR type, calibration, TCP offset, and URDF base frame before "
                "running speed_j_dls."
            )

    def request_stop(self, *_args):
        self.stop_requested = True

    def _warn_throttled(self, message: str, interval_sec: float = 1.0):
        now = time.monotonic()
        if now - self._last_warn_time >= interval_sec:
            print(message)
            self._last_warn_time = now

    def _update_actual_cache(self, q=None, tcp=None):
        if q is not None:
            self._latest_actual_q = np.array(q, dtype=np.float64).copy()
        if tcp is not None:
            self._latest_actual_tcp = np.array(tcp, dtype=np.float64).copy()
        if q is not None or tcp is not None:
            self._latest_actual_time = time.monotonic()

    def _tcp_from_joints(self, q):
        if self.local_kinematics is not None:
            try:
                return self.local_kinematics.forward_pose(q, self.tcp_offset)
            except Exception:
                return None
        return None

    def _actual_cache_is_fresh(self) -> bool:
        return (
            self._latest_actual_time > 0.0
            and (time.monotonic() - self._latest_actual_time) <= 0.1
        )

    def _trigger_to_raw(self, trigger_value: float) -> int:
        clipped = min(1.0, max(0.0, float(trigger_value)))
        open_raw = self.config.gripper.open_raw
        closed_raw = self.config.gripper.closed_raw
        if self.config.gripper.binary_control:
            threshold = min(
                1.0,
                max(0.0, float(self.config.gripper.trigger_threshold)),
            )
            return closed_raw if clipped >= threshold else open_raw
        return int(round(open_raw + (closed_raw - open_raw) * clipped))

    def _handle_gripper(self):
        if self.gripper is None:
            return None
        trigger = self.xr.get_value(self.config.xr.trigger_name)
        raw = self._trigger_to_raw(trigger)
        self.gripper.set_target(raw)
        return raw

    def _motion_enabled_by_grip(self, grip_value: float) -> bool:
        grip = min(1.0, max(0.0, float(grip_value)))
        self._last_motion_grip_value = grip
        now = time.monotonic()
        if self._grip_active:
            if grip >= self.config.xr.grip_release_threshold:
                self._grip_below_release_since = None
                self._grip_active = True
            else:
                if self._grip_below_release_since is None:
                    self._grip_below_release_since = now
                grace_sec = max(0.0, float(self.config.xr.grip_release_grace_sec))
                self._grip_active = (now - self._grip_below_release_since) <= grace_sec
        else:
            self._grip_below_release_since = None
            self._grip_active = grip >= self.config.xr.grip_threshold
        if not self._grip_active:
            self._grip_below_release_since = None
        self._last_motion_enabled = self._grip_active
        return self._grip_active

    def _handle_recorder_control(self):
        if self.recorder_control is None:
            return
        self._handle_recorder_start_button()
        self._handle_recorder_stop_button()
        self.recorder_control.poll(self.xr.get_button)

    def _handle_recorder_start_button(self):
        button = self.config.recorder.start_button
        pressed = bool(self.xr.get_button(button))
        was_pressed = self._start_button_was_pressed
        self._start_button_was_pressed = pressed
        if not pressed or was_pressed:
            return
        self._clear_replay_buffer("recorder start")
        if self.config.replay.enabled:
            self._replay_episode_armed = True
            print("Replay armed for this recorder episode.")

    def _handle_recorder_stop_button(self):
        button = self.config.recorder.stop_button
        pressed = bool(self.xr.get_button(button))
        was_pressed = self._stop_button_was_pressed
        self._stop_button_was_pressed = pressed
        if not pressed or was_pressed:
            return

        now = time.monotonic()
        if now - self._last_stop_button_time < self.config.recorder.debounce_sec:
            return
        self._last_stop_button_time = now

        if self.config.return_home.enabled:
            self._start_return_home()
        else:
            self.recorder_control.request("stop")

    def _write_state(
        self,
        status: str,
        gripper_raw,
        target_tcp=None,
        target_q=None,
        force: bool = False,
    ):
        force = bool(force or self._force_next_state_write)
        if self.state_writer is None:
            self._force_next_state_write = False
            return
        if not force and not self.state_writer.should_write():
            return
        gripper_current_raw = self.gripper.get_current() if self.gripper is not None else None
        state = {
            "status": status,
            "control_mode": self.config.arm.control_mode,
            "gripper_target_raw": gripper_raw,
            "gripper_current_raw": gripper_current_raw,
        }
        if self._last_motion_grip_value is not None:
            state["motion_grip_value"] = float(self._last_motion_grip_value)
        state["motion_enabled"] = bool(self._last_motion_enabled)
        if self._last_xr_age_sec is not None:
            state["xr_age_sec"] = float(self._last_xr_age_sec)
        if self._last_xr_target_interpolated:
            state["xr_target_interpolated"] = True
            if self._last_xr_target_interpolation_age_sec is not None:
                state["xr_target_interpolation_age_sec"] = float(
                    self._last_xr_target_interpolation_age_sec
                )
        if target_tcp is not None:
            state["target_tcp_pose"] = np.array(target_tcp, dtype=float).tolist()
        if target_q is not None:
            state["target_q"] = np.array(target_q, dtype=float).tolist()
        self._add_actual_robot_state(state)
        self.state_writer.maybe_write(state, force=force)
        self._force_next_state_write = False

    def _current_gripper_target_raw(self):
        if self.gripper is None:
            return None
        try:
            return self.gripper.get_target()
        except Exception as exc:
            self._warn_throttled(f"Failed to read gripper target for state: {exc}")
            return None

    def _add_actual_robot_state(self, state: dict):
        cache_fresh = self._actual_cache_is_fresh()
        if cache_fresh and self._latest_actual_q is not None:
            state["actual_q"] = self._latest_actual_q.tolist()
        else:
            try:
                q = self.arm.get_current_joints()
                state["actual_q"] = q.tolist()
                self._update_actual_cache(q=q)
            except Exception as exc:
                state["actual_q_error"] = str(exc)

        if cache_fresh and self._latest_actual_tcp is not None:
            state["actual_tcp_pose"] = self._latest_actual_tcp.tolist()
        else:
            try:
                tcp = self.arm.get_current_tcp_pose()
                state["actual_tcp_pose"] = tcp.tolist()
                self._update_actual_cache(tcp=tcp)
            except Exception as exc:
                state["actual_tcp_pose_error"] = str(exc)

    def _xr_is_fresh(self) -> bool:
        timestamp = self.xr.timestamp_sec()
        if timestamp <= 0.0:
            self._last_xr_age_sec = None
            self._xr_sample_updated = True
            return True
        now = time.monotonic()
        if self._last_xr_timestamp is None or timestamp != self._last_xr_timestamp:
            self._last_xr_timestamp = timestamp
            self._last_xr_update_wall = now
            self._last_xr_age_sec = 0.0
            self._xr_sample_updated = True
            return True
        self._last_xr_age_sec = now - self._last_xr_update_wall
        self._xr_sample_updated = False
        return self._last_xr_age_sec <= self.config.xr.timeout_sec

    def _deactivate(self):
        if self.enabled:
            print("Teleop disabled.")
            self._stop_active_motion()
        self.enabled = False
        self.mapper.reset()
        self.previous_target_tcp = None
        self.previous_target_q = None
        self._previous_joint_velocity = np.zeros(6, dtype=np.float64)
        self._last_speed_cycle_time = None
        self._last_target_cycle_time = None
        self._last_motion_enabled = False
        self._reset_target_interpolator()
        if self.ruckig_position_smoother is not None:
            self.ruckig_position_smoother.reset()

    def _stop_active_motion(self):
        if self._active_motion_kind == "speed":
            self.arm.stop_speed(self.config.arm.max_joint_acceleration)
        elif self._active_motion_kind == "servo":
            self.arm.stop_servo()
        elif self.config.arm.control_mode == "speed_j_dls":
            self.arm.stop_speed(self.config.arm.max_joint_acceleration)
        else:
            self.arm.stop_servo()
        self._active_motion_kind = None

    def _hold_speed_backend_alive(self):
        if self.config.arm.control_mode == "speed_j_dls":
            self.arm.keep_speedj_backend_alive(self.config.arm.max_joint_acceleration)

    def _reset_target_interpolator(self):
        self._last_xr_target_interpolated = False
        self._last_raw_target_tcp = None
        self._last_raw_target_time = None
        self._target_interpolation_anchor_tcp = None
        self._target_interpolation_anchor_time = None
        self._target_interpolation_linear_velocity = np.zeros(3, dtype=np.float64)
        self._target_interpolation_angular_velocity = np.zeros(3, dtype=np.float64)
        self._last_xr_target_interpolation_age_sec = None

    def _activate(self, xr_pose):
        tcp_pose = self.arm.get_current_tcp_pose()
        current_q = self.arm.get_current_joints()
        self._update_actual_cache(q=current_q, tcp=tcp_pose)
        self.mapper.begin(xr_pose, tcp_pose)
        self.previous_target_tcp = tcp_pose.copy()
        self.previous_target_q = current_q
        self._previous_joint_velocity = np.zeros(6, dtype=np.float64)
        self._last_speed_cycle_time = time.monotonic()
        self._last_target_cycle_time = self._last_speed_cycle_time
        self._reset_target_interpolator()
        if self.ruckig_position_smoother is not None:
            position, _rotation = pose_vector_to_parts(tcp_pose)
            self.ruckig_position_smoother.reset(position)
        self.enabled = True
        print("Teleop enabled.")

    def _interpolate_stale_target(self, raw_target_tcp, now: float) -> np.ndarray:
        raw_target = np.array(raw_target_tcp, dtype=np.float64)
        self._last_xr_target_interpolated = False
        self._last_xr_target_interpolation_age_sec = None
        interpolation_sec = max(0.0, float(self.config.xr.target_interpolation_sec))
        if interpolation_sec <= 0.0:
            self._update_target_interpolator(raw_target, now)
            return raw_target

        if self._xr_sample_updated or self._last_raw_target_tcp is None:
            self._update_target_interpolator(raw_target, now)
            return raw_target

        anchor = self._target_interpolation_anchor_tcp
        anchor_time = self._target_interpolation_anchor_time
        if anchor is None or anchor_time is None:
            return raw_target

        age = self._last_xr_age_sec
        if age is None:
            age = now - float(anchor_time)
        age = max(0.0, float(age))
        interpolation_age = min(age, interpolation_sec)

        anchor_position, anchor_rotation = pose_vector_to_parts(anchor)
        linear_velocity = self._target_interpolation_linear_velocity
        angular_velocity = self._target_interpolation_angular_velocity
        predicted_position = anchor_position + linear_velocity * interpolation_age
        predicted_rotation = (
            rotvec_to_matrix(angular_velocity * interpolation_age) @ anchor_rotation
        )
        self._last_xr_target_interpolated = True
        self._last_xr_target_interpolation_age_sec = interpolation_age
        return parts_to_pose_vector(predicted_position, predicted_rotation)

    def _update_target_interpolator(self, raw_target_tcp: np.ndarray, now: float):
        if self._last_raw_target_tcp is not None and self._last_raw_target_time is not None:
            dt = max(1.0e-4, now - float(self._last_raw_target_time))
            prev_position, prev_rotation = pose_vector_to_parts(self._last_raw_target_tcp)
            position, rotation = pose_vector_to_parts(raw_target_tcp)
            self._target_interpolation_linear_velocity = (position - prev_position) / dt
            delta_rotation = rotation @ prev_rotation.T
            self._target_interpolation_angular_velocity = matrix_to_rotvec(delta_rotation) / dt
        else:
            self._target_interpolation_linear_velocity = np.zeros(3, dtype=np.float64)
            self._target_interpolation_angular_velocity = np.zeros(3, dtype=np.float64)

        self._last_raw_target_tcp = raw_target_tcp.copy()
        self._last_raw_target_time = now
        self._target_interpolation_anchor_tcp = raw_target_tcp.copy()
        self._target_interpolation_anchor_time = now

    def _run_enabled_cycle(self, xr_pose):
        now = time.monotonic()
        raw_target_tcp = self.mapper.target_tcp_pose(xr_pose)
        raw_target_tcp = self._interpolate_stale_target(raw_target_tcp, now)
        previous_target_tcp = (
            None if self.previous_target_tcp is None else self.previous_target_tcp.copy()
        )
        if self._last_target_cycle_time is None:
            target_dt = self.arm.servo_time
        else:
            target_dt = now - self._last_target_cycle_time
        self._last_target_cycle_time = now
        if (
            self.control_latency_warning_sec > 0.0
            and target_dt > self.control_latency_warning_sec
        ):
            self._warn_throttled(
                "Teleop control loop latency gap: "
                f"{target_dt * 1000.0:.1f}ms "
                f"(bridge watchdog={self.config.arm.speedj_watchdog_sec * 1000.0:.0f}ms)",
                interval_sec=0.5,
            )
        target_dt = max(
            self.arm.servo_time * 0.5,
            min(target_dt, self.arm.servo_time * 3.0),
        )
        target_tcp = self._smooth_target_tcp(raw_target_tcp, target_dt, now)

        if self.config.arm.control_mode == "servo_l":
            self.previous_target_tcp = target_tcp.copy()
            self._active_motion_kind = "servo"
            self.arm.servo_linear(target_tcp)
            return target_tcp, None

        if self.config.arm.control_mode == "speed_j_dls":
            return self._run_speed_j_dls_cycle(
                target_tcp,
                previous_target_tcp,
                target_dt,
            )

        return self._run_servo_j_cycle(target_tcp)

    def _smooth_target_tcp(self, raw_target_tcp, target_dt: float, now: float):
        if self.config.safety.disable_teleop_limits:
            return np.array(raw_target_tcp, dtype=np.float64, copy=True)

        if (
            self.target_smoothing_mode == "ruckig"
            and self.ruckig_position_smoother is not None
        ):
            raw_clamped = self.safety.clamp_workspace(raw_target_tcp)
            raw_position, raw_rotation = pose_vector_to_parts(raw_clamped)
            smooth_position = self.ruckig_position_smoother.update(raw_position, now)
            rotation_limited = self.safety.limit_tcp_rotation_step(
                raw_target_tcp,
                self.previous_target_tcp,
                cycle_dt=target_dt,
            )
            _raw_position, smooth_rotation = pose_vector_to_parts(rotation_limited)
            return parts_to_pose_vector(smooth_position, smooth_rotation)

        if self.target_smoothing_mode == "none":
            smoothing_sec = self.safety.target_smoothing_sec
            self.safety.target_smoothing_sec = 0.0
            try:
                return self.safety.limit_tcp_step(raw_target_tcp, self.previous_target_tcp)
            finally:
                self.safety.target_smoothing_sec = smoothing_sec

        return self.safety.limit_tcp_step(
            raw_target_tcp,
            self.previous_target_tcp,
            cycle_dt=target_dt,
        )

    def _run_servo_j_cycle(self, target_tcp):
        q_near = self.previous_target_q
        if q_near is None:
            q_near = self.arm.get_current_joints()
        target_q = self.arm.inverse_kinematics(
            target_tcp,
            q_near=q_near,
            max_position_error=self.config.arm.ik_max_position_error,
            max_orientation_error=self.config.arm.ik_max_orientation_error,
        )
        if target_q is None:
            self._warn_throttled("IK failed or RTDE getInverseKinematics is unavailable.")
            if self.previous_target_q is not None:
                self.arm.servo_joints(self.previous_target_q)
            held_tcp = self.previous_target_tcp if self.previous_target_tcp is not None else target_tcp
            return held_tcp, self.previous_target_q

        target_q = nearest_joint_equivalent(target_q, q_near)
        if self.config.safety.disable_teleop_limits:
            safe_q = np.array(target_q, dtype=np.float64)
        else:
            safe_q = self.safety.limit_joint_step(target_q, self.previous_target_q)
        self.previous_target_tcp = target_tcp.copy()
        self.previous_target_q = safe_q.copy()
        self._active_motion_kind = "servo"
        self.arm.servo_joints(safe_q)
        return target_tcp, safe_q

    def _run_speed_j_dls_cycle(self, target_tcp, previous_target_tcp, target_dt):
        current_q = self.arm.get_current_joints()
        current_tcp = self.arm.get_current_tcp_pose()
        self._update_actual_cache(q=current_q, tcp=current_tcp)
        if not self.config.safety.disable_teleop_limits:
            target_tcp = self.safety.limit_tcp_lag(target_tcp, current_tcp)
        if self.config.arm.dls_jacobian_source == "local":
            if self.local_kinematics is None:
                raise RuntimeError("Local DLS kinematics is not initialized.")
            jacobian = self.local_kinematics.frame_jacobian(
                current_q,
                self.tcp_offset,
            )
        else:
            jacobian = self.arm.get_jacobian(current_q)
            if jacobian is None:
                detail = self.arm.last_jacobian_error
                suffix = f": {detail}" if detail else ""
                raise RuntimeError(f"RTDE getJacobian failed{suffix}")

        twist = self._target_twist(
            target_tcp,
            current_tcp,
            previous_target_tcp,
            target_dt,
        )
        qdot, min_singular, singular_scale, damping = dls_joint_velocity(
            jacobian,
            twist,
            damping=self.config.arm.dls_damping,
            max_damping=self.config.arm.dls_max_damping,
            singularity_threshold=self.config.arm.dls_singularity_threshold,
            hard_stop_threshold=self.config.arm.dls_hard_stop_threshold,
        )
        qdot = self._limit_joint_velocity(qdot)

        target_q = self._speed_action_target_q(current_q, qdot)
        self._active_motion_kind = "speed"
        self.arm.speed_joints(qdot, self.config.arm.max_joint_acceleration)

        self.previous_target_tcp = target_tcp.copy()
        self.previous_target_q = target_q.copy()
        self._previous_joint_velocity = qdot.copy()

        if singular_scale < 0.35:
            self._warn_throttled(
                "DLS singularity damping active: "
                f"min_sv={min_singular:.4f}, scale={singular_scale:.2f}, "
                f"damping={damping:.3f}"
            )
        return target_tcp, target_q

    def _speed_action_target_q(self, current_q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        if self.state_writer is None:
            horizon_sec = 1.0 / 30.0
        else:
            horizon_sec = self.state_writer.period_sec
        horizon_sec = max(self.arm.servo_time, min(float(horizon_sec), 0.1))
        return (
            np.array(current_q, dtype=np.float64)
            + np.array(qdot, dtype=np.float64) * horizon_sec
        )

    def _target_twist(
        self,
        target_tcp,
        current_tcp,
        previous_target_tcp=None,
        target_dt=None,
    ) -> np.ndarray:
        target_position, target_rotation = pose_vector_to_parts(target_tcp)
        current_position, current_rotation = pose_vector_to_parts(current_tcp)

        if self.config.safety.disable_teleop_limits:
            position_error = target_position - current_position
        else:
            position_error = apply_deadband(
                target_position - current_position,
                self.config.arm.dls_position_deadband,
            )
        linear = position_error * self.config.arm.dls_linear_gain
        rotation_error = target_rotation @ current_rotation.T
        if self.config.safety.disable_teleop_limits:
            rotation_vector = matrix_to_rotvec(rotation_error)
        else:
            rotation_vector = apply_deadband(
                matrix_to_rotvec(rotation_error),
                self.config.arm.dls_orientation_deadband,
            )
        angular = rotation_vector * self.config.arm.dls_angular_gain

        if not self.config.safety.disable_teleop_limits:
            linear = limit_vector_norm(linear, self.config.arm.max_tcp_linear_speed)
            angular = limit_vector_norm(angular, self.config.arm.max_tcp_angular_speed)
        return np.concatenate((linear, angular))

    def _limit_joint_velocity(self, qdot: np.ndarray) -> np.ndarray:
        limited = np.array(qdot, dtype=np.float64)
        if self.config.safety.disable_teleop_limits:
            self._last_speed_cycle_time = time.monotonic()
            return limited

        max_speed = abs(float(self.config.arm.max_joint_speed))
        if max_speed > 0.0:
            limited = scale_to_component_limit(limited, max_speed)

        now = time.monotonic()
        if self._last_speed_cycle_time is None:
            dt = self.arm.servo_time
        else:
            dt = now - self._last_speed_cycle_time
        self._last_speed_cycle_time = now
        dt = max(self.arm.servo_time * 0.5, min(dt, self.arm.servo_time * 3.0))

        max_delta = abs(float(self.config.arm.max_joint_acceleration)) * dt
        if max_delta > 0.0:
            delta = limited - self._previous_joint_velocity
            limited = (
                self._previous_joint_velocity
                + scale_to_component_limit(delta, max_delta)
            )
        return limited

    def run(self):
        if self.gripper is not None:
            self.gripper.start()
        if self.config.arm.control_mode == "speed_j_dls":
            self.arm.start_speedj_backend()

        print(
            "Teleop started: "
            f"robot={self.config.arm.robot_ip}, "
            f"mode={self.config.arm.control_mode}, "
            f"speedj_backend={self.config.arm.speedj_backend}, "
            f"rate={self.config.arm.rate_hz}Hz, "
            f"gripper={self.config.gripper.port if self.gripper else 'disabled'}"
        )
        print("Hold right grip to move. Press right stick click for emergency stop.")
        print(
            "Grip enable: "
            f"start>={self.config.xr.grip_threshold:.2f}, "
            f"release<{self.config.xr.grip_release_threshold:.2f}"
        )
        if self.config.safety.disable_teleop_limits:
            print(
                "WARNING: teleop-side safety limits are disabled. "
                "Target smoothing, step/lag limits, workspace clamp, TCP speed caps, "
                "and joint velocity/acceleration limiting are bypassed; emergency "
                "button, UR controller protections, and DLS singularity hard stop remain active."
            )
        if self.recorder_control is not None:
            print(
                "Recorder buttons: "
                f"start={self.config.recorder.start_button}, "
                f"stop={self.config.recorder.stop_button}"
                f"{' -> return home then stop' if self.config.return_home.enabled else ''}, "
                f"save={self.config.recorder.save_button}, "
                f"discard={self.config.recorder.discard_button}"
            )
        if self.config.replay.enabled:
            print(
                "Replay trigger: "
                f"{self.config.replay.trigger_button} starts the ready trajectory."
            )

        while not self.stop_requested:
            gripper_raw = self._handle_gripper()

            if self.xr.get_button(self.config.xr.emergency_button):
                print("Emergency button pressed.")
                break

            if self.return_home_active:
                self._handle_recorder_control()
                try:
                    self._run_return_home_cycle(gripper_raw)
                except Exception as exc:
                    self._warn_throttled(f"Return-home cycle failed: {exc}")
                    self._stop_active_motion()
                    time.sleep(0.02)
                continue

            self._handle_replay_button()
            if self._run_replay_if_needed(gripper_raw):
                continue

            if not self._xr_is_fresh():
                self._warn_throttled("XR input timed out; holding robot.")
                self._deactivate()
                self._hold_speed_backend_alive()
                self._write_state("xr_timeout", gripper_raw)
                time.sleep(0.01)
                continue

            self._handle_recorder_control()
            if self.return_home_active:
                continue

            xr_pose = self.xr.get_pose(self.config.xr.pose_source)
            if not np.all(np.isfinite(np.array(xr_pose, dtype=float))):
                self._warn_throttled("XR pose contains non-finite values; holding robot.")
                self._deactivate()
                self._hold_speed_backend_alive()
                self._write_state("invalid_xr_pose", gripper_raw)
                time.sleep(0.01)
                continue

            grip = self.xr.get_value(self.config.xr.grip_name)
            active = self._motion_enabled_by_grip(grip)

            if not active:
                self._motion_locked_until_grip_release = False
                self._deactivate()
                self._hold_speed_backend_alive()
                self._capture_replay_gripper_only_point(gripper_raw)
                status = "replay_ready" if self._replay_ready else "idle"
                self._write_state(status, gripper_raw)
                time.sleep(0.01)
                continue

            if self._motion_locked_until_grip_release:
                self._deactivate()
                self._hold_speed_backend_alive()
                self._write_state("idle", gripper_raw)
                time.sleep(0.01)
                continue

            if not self.enabled:
                self._activate(xr_pose)

            try:
                target_tcp, target_q = self._run_enabled_cycle(xr_pose)
            except Exception as exc:
                self._warn_throttled(f"Control cycle failed: {exc}")
                self._stop_active_motion()
                time.sleep(0.02)
                continue

            self._capture_replay_point(target_q, target_tcp, gripper_raw)
            self._write_state("running", gripper_raw, target_tcp, target_q)

        self.shutdown()

    def shutdown(self):
        print("Shutting down teleop.")
        self._deactivate()
        if self.gripper is not None:
            self.gripper.stop()
        self.arm.close()
        if self.recorder_control is not None:
            self.recorder_control.close()
        self.xr.close()


def main(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = config_from_args(args)
    configure_xr_service_runtime(config.xr.service_realtime_priority)
    configure_low_latency_runtime(config.arm.realtime_priority, config.arm.lock_memory)
    runner = TeleopRunner(config)
    signal.signal(signal.SIGINT, runner.request_stop)
    signal.signal(signal.SIGTERM, runner.request_stop)
    runner.run()


if __name__ == "__main__":
    main()
