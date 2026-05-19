#!/usr/bin/env python3
import argparse
import ctypes
import json
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.config import (
    ArmConfig,
    GripperConfig,
    RecorderControlConfig,
    ReplayConfig,
    ReturnHomeConfig,
    SafetyConfig,
    StateConfig,
    TeleopConfig,
    XrConfig,
)
from teleop.gripper_direct import DirectDHGripper
from teleop.local_kinematics import LocalURKinematics
from teleop.pose_mapper import PoseMapper
from teleop.recorder_control import RecorderControl
from teleop.rtde_arm import RTDEArm
from teleop.ruckig_smoothing import RuckigPositionSmoother
from teleop.safety import SafetyLimiter
from teleop.state_writer import StateWriter
from teleop.transforms import (
    matrix_to_rotvec,
    parts_to_pose_vector,
    pose_vector_to_parts,
    rotvec_to_matrix,
)
from teleop.xr_input import XrInput


SAMPLED_REPLAY_FILE = Path("/tmp/ur_sampled_replay.json")


def parse_xyz(value: str):
    if value == "":
        return None
    parts = [float(item.strip()) for item in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected x,y,z")
    return tuple(parts)


def parse_joint_positions(value: str):
    parts = [float(item.strip()) for item in value.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("expected q1,q2,q3,q4,q5,q6")
    return tuple(parts)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="High-performance XR RTDE teleoperation.")
    parser.add_argument("--robot-ip", default="192.168.1.102")
    parser.add_argument("--ur-type", default="ur7e")
    parser.add_argument(
        "--control-mode",
        choices=("servo_j", "servo_l", "speed_j_dls"),
        default="servo_j",
    )
    parser.add_argument("--rate-hz", type=float, default=500.0)
    parser.add_argument("--max-velocity", type=float, default=0.5)
    parser.add_argument("--max-acceleration", type=float, default=1.0)
    parser.add_argument("--lookahead-time", type=float, default=0.06)
    parser.add_argument("--servo-gain", type=float, default=500.0)
    parser.add_argument(
        "--speedj-backend",
        choices=("direct", "register_bridge"),
        default="direct",
        help=(
            "speed_j_dls command transport. direct calls RTDE speedJ every Python "
            "cycle; register_bridge uploads a robot-side 2ms speedJ loop and only "
            "updates RTDE input registers from Python."
        ),
    )
    parser.add_argument("--speedj-register-start", type=int, default=42)
    parser.add_argument("--speedj-velocity-scale", type=float, default=10000.0)
    parser.add_argument("--speedj-watchdog-sec", type=float, default=0.06)
    parser.add_argument("--speedj-stale-stop-sec", type=float, default=1.0)
    parser.add_argument("--speedj-upload-port", type=int, default=30001)
    parser.add_argument("--speed-slider-fraction", type=float, default=1.0)
    parser.add_argument("--realtime-priority", type=int, default=0)
    parser.add_argument("--lock-memory", action="store_true")
    parser.add_argument("--control-latency-warning-sec", type=float, default=0.02)
    parser.add_argument("--ik-max-position-error", type=float, default=1.0e-4)
    parser.add_argument("--ik-max-orientation-error", type=float, default=1.0e-3)
    parser.add_argument("--dls-linear-gain", type=float, default=8.0)
    parser.add_argument("--dls-angular-gain", type=float, default=6.0)
    parser.add_argument("--dls-damping", type=float, default=0.03)
    parser.add_argument("--dls-max-damping", type=float, default=0.25)
    parser.add_argument("--dls-position-deadband", type=float, default=0.0)
    parser.add_argument("--dls-orientation-deadband", type=float, default=0.0)
    parser.add_argument("--dls-singularity-threshold", type=float, default=0.05)
    parser.add_argument("--dls-hard-stop-threshold", type=float, default=0.015)
    parser.add_argument("--dls-frame-name", default="tool0")
    parser.add_argument("--dls-jacobian-source", choices=("local", "rtde"), default="local")
    parser.add_argument("--ur-description-path", default=None)
    parser.add_argument("--max-tcp-linear-speed", type=float, default=0.25)
    parser.add_argument("--max-tcp-angular-speed", type=float, default=0.8)
    parser.add_argument("--max-joint-speed", type=float, default=0.8)
    parser.add_argument("--max-joint-acceleration", type=float, default=2.0)

    parser.add_argument("--pose-source", default="right_controller")
    parser.add_argument("--grip-name", default="right_grip")
    parser.add_argument("--trigger-name", default="right_trigger")
    parser.add_argument("--emergency-button", default="right_axis_click")
    parser.add_argument("--xr-service-realtime-priority", type=int, default=0)
    parser.add_argument("--grip-threshold", type=float, default=0.95)
    parser.add_argument("--grip-release-threshold", type=float, default=None)
    parser.add_argument("--grip-release-grace-sec", type=float, default=0.0)
    parser.add_argument("--xr-timeout-sec", type=float, default=0.2)
    parser.add_argument("--xr-target-interpolation-sec", type=float, default=0.0)
    parser.add_argument("--scale-factor", type=float, default=0.6)
    parser.add_argument("--rotation-scale", type=float, default=1.0)

    parser.add_argument("--max-step-translation", type=float, default=0.01)
    parser.add_argument("--max-step-rotation", type=float, default=0.05)
    parser.add_argument("--max-joint-step", type=float, default=0.04)
    parser.add_argument(
        "--unsafe-disable-teleop-safety-limits",
        action="store_true",
        help=(
            "Disable teleop-side target smoothing, workspace clamp, step/lag limits, "
            "TCP speed caps, and joint velocity/acceleration limiting. This does not "
            "disable the emergency button, UR controller protections, or DLS "
            "singularity hard stop."
        ),
    )
    parser.add_argument(
        "--target-smoothing-mode",
        choices=("none", "linear", "ruckig"),
        default="linear",
    )
    parser.add_argument("--target-smoothing-sec", type=float, default=0.0)
    parser.add_argument("--ruckig-max-linear-acceleration", type=float, default=1.5)
    parser.add_argument("--ruckig-max-linear-jerk", type=float, default=30.0)
    parser.add_argument("--ruckig-target-prediction-sec", type=float, default=0.006)
    parser.add_argument("--ruckig-velocity-hold-sec", type=float, default=0.015)
    parser.add_argument("--max-target-lag-translation", type=float, default=0.0)
    parser.add_argument("--max-target-lag-rotation", type=float, default=0.0)
    parser.add_argument("--workspace-min", type=parse_xyz, default=None)
    parser.add_argument("--workspace-max", type=parse_xyz, default=None)

    parser.add_argument("--disable-gripper", action="store_true")
    parser.add_argument("--gripper-port", default="/tmp/ttyUR")
    parser.add_argument("--gripper-baudrate", type=int, default=115200)
    parser.add_argument("--gripper-rate-hz", type=float, default=30.0)
    parser.add_argument("--gripper-speed", type=int, default=100)
    parser.add_argument("--gripper-force", type=int, default=50)
    parser.add_argument("--gripper-open-raw", type=int, default=1000)
    parser.add_argument("--gripper-closed-raw", type=int, default=0)
    parser.add_argument("--gripper-binary-control", action="store_true")
    parser.add_argument("--gripper-trigger-threshold", type=float, default=0.5)
    parser.add_argument("--gripper-deadband", type=int, default=4)
    parser.add_argument("--gripper-init-timeout-sec", type=float, default=8.0)
    parser.add_argument("--gripper-read-hz", type=float, default=30.0)

    parser.add_argument("--disable-state-file", action="store_true")
    parser.add_argument("--state-file", type=Path, default=Path("/tmp/ur_teleop_state.json"))
    parser.add_argument("--state-hz", type=float, default=30.0)

    parser.add_argument("--enable-recorder-control", action="store_true")
    parser.add_argument("--recorder-start-button", default="A")
    parser.add_argument("--recorder-stop-button", default="B")
    parser.add_argument("--recorder-save-button", default="X")
    parser.add_argument("--recorder-discard-button", default="Y")
    parser.add_argument("--recorder-debounce-sec", type=float, default=0.5)

    parser.add_argument("--disable-return-home-before-stop", action="store_true")
    parser.add_argument("--return-home-joints", type=parse_joint_positions, default=None)
    parser.add_argument("--return-home-max-joint-step", type=float, default=0.0006)
    parser.add_argument("--return-home-tolerance", type=float, default=0.01)
    parser.add_argument("--return-home-timeout-sec", type=float, default=20.0)
    parser.add_argument("--return-home-hold-sec", type=float, default=0.3)
    parser.add_argument("--return-home-stop-grace-sec", type=float, default=0.2)
    parser.add_argument("--replay-after-return-home", action="store_true")
    parser.add_argument(
        "--replay-button",
        default="left_grip",
        help=(
            "XR button or analog control that starts a ready replay trajectory. "
            "Use a comma-separated list to accept multiple controls, e.g. "
            "left_grip,left_menu_button,left_axis_click."
        ),
    )
    parser.add_argument("--replay-trigger-threshold", type=float, default=0.8)
    parser.add_argument("--replay-delay-sec", type=float, default=2.0)
    parser.add_argument("--replay-record-hz", type=float, default=30.0)
    parser.add_argument("--replay-speed-scale", type=float, default=1.0)
    parser.add_argument("--replay-joint-gain", type=float, default=10.0)
    parser.add_argument("--replay-tolerance", type=float, default=0.02)
    return parser


def config_from_args(args) -> TeleopConfig:
    arm = ArmConfig(
        robot_ip=args.robot_ip,
        ur_type=args.ur_type,
        control_mode=args.control_mode,
        rate_hz=args.rate_hz,
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
        lookahead_time=args.lookahead_time,
        servo_gain=args.servo_gain,
        speedj_backend=args.speedj_backend,
        speedj_register_start=args.speedj_register_start,
        speedj_velocity_scale=args.speedj_velocity_scale,
        speedj_watchdog_sec=args.speedj_watchdog_sec,
        speedj_stale_stop_sec=args.speedj_stale_stop_sec,
        speedj_upload_port=args.speedj_upload_port,
        speed_slider_fraction=args.speed_slider_fraction,
        realtime_priority=args.realtime_priority,
        lock_memory=args.lock_memory,
        control_latency_warning_sec=args.control_latency_warning_sec,
        ik_max_position_error=args.ik_max_position_error,
        ik_max_orientation_error=args.ik_max_orientation_error,
        dls_linear_gain=args.dls_linear_gain,
        dls_angular_gain=args.dls_angular_gain,
        dls_damping=args.dls_damping,
        dls_max_damping=args.dls_max_damping,
        dls_position_deadband=args.dls_position_deadband,
        dls_orientation_deadband=args.dls_orientation_deadband,
        dls_singularity_threshold=args.dls_singularity_threshold,
        dls_hard_stop_threshold=args.dls_hard_stop_threshold,
        dls_frame_name=args.dls_frame_name,
        dls_jacobian_source=args.dls_jacobian_source,
        ur_description_path=args.ur_description_path,
        max_tcp_linear_speed=args.max_tcp_linear_speed,
        max_tcp_angular_speed=args.max_tcp_angular_speed,
        max_joint_speed=args.max_joint_speed,
        max_joint_acceleration=args.max_joint_acceleration,
    )
    grip_threshold = min(1.0, max(0.0, float(args.grip_threshold)))
    if args.grip_release_threshold is None:
        grip_release_threshold = max(0.0, grip_threshold - 0.15)
    else:
        grip_release_threshold = float(args.grip_release_threshold)
    grip_release_threshold = min(grip_threshold, max(0.0, grip_release_threshold))

    xr = XrConfig(
        pose_source=args.pose_source,
        grip_name=args.grip_name,
        trigger_name=args.trigger_name,
        emergency_button=args.emergency_button,
        service_realtime_priority=args.xr_service_realtime_priority,
        grip_threshold=grip_threshold,
        grip_release_threshold=grip_release_threshold,
        grip_release_grace_sec=max(0.0, float(args.grip_release_grace_sec)),
        timeout_sec=args.xr_timeout_sec,
        target_interpolation_sec=max(0.0, float(args.xr_target_interpolation_sec)),
        scale_factor=args.scale_factor,
        rotation_scale=args.rotation_scale,
    )
    safety = SafetyConfig(
        disable_teleop_limits=args.unsafe_disable_teleop_safety_limits,
        max_step_translation=args.max_step_translation,
        max_step_rotation=args.max_step_rotation,
        max_joint_step=args.max_joint_step,
        target_smoothing_mode=args.target_smoothing_mode,
        target_smoothing_sec=args.target_smoothing_sec,
        ruckig_max_linear_acceleration=args.ruckig_max_linear_acceleration,
        ruckig_max_linear_jerk=args.ruckig_max_linear_jerk,
        ruckig_target_prediction_sec=args.ruckig_target_prediction_sec,
        ruckig_velocity_hold_sec=args.ruckig_velocity_hold_sec,
        max_target_lag_translation=args.max_target_lag_translation,
        max_target_lag_rotation=args.max_target_lag_rotation,
        workspace_min=args.workspace_min,
        workspace_max=args.workspace_max,
    )
    gripper = GripperConfig(
        enabled=not args.disable_gripper,
        port=args.gripper_port,
        baudrate=args.gripper_baudrate,
        rate_hz=args.gripper_rate_hz,
        speed=args.gripper_speed,
        force=args.gripper_force,
        open_raw=args.gripper_open_raw,
        closed_raw=args.gripper_closed_raw,
        binary_control=args.gripper_binary_control,
        trigger_threshold=args.gripper_trigger_threshold,
        position_deadband=args.gripper_deadband,
        init_timeout_sec=args.gripper_init_timeout_sec,
        read_state_hz=args.gripper_read_hz,
    )
    state = StateConfig(
        enabled=not args.disable_state_file,
        path=args.state_file,
        rate_hz=args.state_hz,
    )
    recorder = RecorderControlConfig(
        enabled=args.enable_recorder_control,
        start_button=args.recorder_start_button,
        stop_button=args.recorder_stop_button,
        save_button=args.recorder_save_button,
        discard_button=args.recorder_discard_button,
        debounce_sec=args.recorder_debounce_sec,
    )
    return_home = ReturnHomeConfig(
        enabled=not args.disable_return_home_before_stop,
        joint_positions=(
            args.return_home_joints
            if args.return_home_joints is not None
            else ReturnHomeConfig().joint_positions
        ),
        max_joint_step=args.return_home_max_joint_step,
        tolerance=args.return_home_tolerance,
        timeout_sec=args.return_home_timeout_sec,
        hold_sec=args.return_home_hold_sec,
        stop_grace_sec=args.return_home_stop_grace_sec,
    )
    replay = ReplayConfig(
        enabled=args.replay_after_return_home,
        trigger_button=args.replay_button,
        trigger_threshold=args.replay_trigger_threshold,
        delay_sec=args.replay_delay_sec,
        record_hz=args.replay_record_hz,
        speed_scale=args.replay_speed_scale,
        joint_gain=args.replay_joint_gain,
        tolerance=args.replay_tolerance,
    )
    return TeleopConfig(
        arm=arm,
        xr=xr,
        safety=safety,
        gripper=gripper,
        state=state,
        recorder=recorder,
        return_home=return_home,
        replay=replay,
    )


class TeleopRunner:
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

    def _start_return_home(self):
        if self.return_home_active:
            print("Return-home is already running.")
            return

        if self.enabled:
            self._deactivate()

        current_q = self.arm.get_current_joints()
        current_tcp = self.arm.get_current_tcp_pose()
        self._update_actual_cache(q=current_q, tcp=current_tcp)
        self.previous_target_q = current_q
        self.previous_target_tcp = current_tcp
        self.return_home_active = True
        self.return_home_started_at = time.monotonic()
        self.return_home_reached_since = None
        self._force_next_state_write = True
        gripper_raw = self._current_gripper_target_raw()
        targets = self._write_return_home_state(
            gripper_raw,
            current_q=current_q,
            force=True,
        )
        if targets is not None:
            target_q, target_tcp = targets
            self._capture_replay_point(
                target_q,
                target_tcp,
                gripper_raw,
                force=True,
                phase="return_home",
            )
        print("Return-home started; recorder will stop after home is reached.")

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

    def _return_home_record_target_q(self, current_q, final: bool = False):
        current = np.array(current_q, dtype=np.float64)
        if final:
            return self._nearest_joint_equivalent(self.return_home_target_q, current)

        if self.config.arm.control_mode == "speed_j_dls":
            if self._return_home_joint_error(current) <= self.config.return_home.tolerance:
                return self._nearest_joint_equivalent(self.return_home_target_q, current)
            qdot = self._return_home_joint_velocity(current)
            return self._return_home_speed_target_q(current, qdot)

        previous_q = self.previous_target_q if self.previous_target_q is not None else current
        return self._limit_return_home_joint_step(self.return_home_target_q, previous_q)

    def _write_return_home_state(
        self,
        gripper_raw=None,
        current_q=None,
        final: bool = False,
        force: bool = False,
    ):
        try:
            if current_q is None:
                current_q = self.arm.get_current_joints()
            current_q = np.array(current_q, dtype=np.float64)
            self._update_actual_cache(q=current_q, tcp=self._tcp_from_joints(current_q))
            target_q = self._return_home_record_target_q(current_q, final=final)
            target_tcp = self._return_home_target_tcp(target_q)
            if gripper_raw is None:
                gripper_raw = self._current_gripper_target_raw()
            self._write_state("return_home", gripper_raw, target_tcp, target_q, force=force)
            return target_q, target_tcp
        except Exception as exc:
            self._warn_throttled(f"Failed to write return-home recorder state: {exc}")
            return None

    def _clear_replay_buffer(self, reason: str | None = None):
        self.replay_buffer = []
        self._last_replay_capture_time = None
        self._replay_capture_started_at = None
        self._replay_ready = False
        self.replay_pending_until = None
        self.replay_active = False
        self.replay_started_at = None
        self._replay_duration = 0.0
        self._replay_index = 0
        self._replay_gripper_index = 0
        if self.config.replay.enabled and reason:
            print(f"Replay buffer cleared: {reason}.")

    def _capture_replay_point(
        self,
        target_q,
        target_tcp,
        gripper_raw,
        *,
        force: bool = False,
        phase: str = "running",
    ):
        if (
            not self.config.replay.enabled
            or not self._replay_episode_armed
            or target_q is None
        ):
            return

        now = time.monotonic()
        period = 1.0 / max(float(self.config.replay.record_hz), 1.0)
        if (
            not force
            and self._last_replay_capture_time is not None
            and now - self._last_replay_capture_time < period
        ):
            return

        q = np.array(target_q, dtype=np.float64)
        if q.size != 6 or not np.all(np.isfinite(q)):
            return

        if not self.replay_buffer:
            self._replay_capture_started_at = now
            replay_time = 0.0
        else:
            q = self._nearest_joint_equivalent(q, self.replay_buffer[-1]["q"])
            replay_time = now - self._replay_capture_started_at

        tcp = None
        if target_tcp is not None:
            tcp_array = np.array(target_tcp, dtype=np.float64)
            if tcp_array.size == 6 and np.all(np.isfinite(tcp_array)):
                tcp = tcp_array

        self.replay_buffer.append(
            {
                "time": float(replay_time),
                "q": q.copy(),
                "tcp": None if tcp is None else tcp.copy(),
                "gripper_raw": self._coerce_replay_gripper_raw(gripper_raw),
                "phase": phase,
            }
        )
        self._last_replay_capture_time = now

    def _capture_replay_gripper_only_point(self, gripper_raw):
        if (
            not self.config.replay.enabled
            or not self._replay_episode_armed
            or not self.replay_buffer
        ):
            return

        raw = self._coerce_replay_gripper_raw(gripper_raw)
        if raw is None:
            return

        previous_raw = self._coerce_replay_gripper_raw(
            self.replay_buffer[-1].get("gripper_raw")
        )
        deadband = max(1, int(self.config.gripper.position_deadband))
        if previous_raw is not None and abs(raw - previous_raw) < deadband:
            return

        try:
            current_q = self.arm.get_current_joints()
            current_tcp = self.arm.get_current_tcp_pose()
        except Exception:
            return
        self._update_actual_cache(q=current_q, tcp=current_tcp)
        self._capture_replay_point(current_q, current_tcp, raw)

    def _schedule_replay_after_return_home(self, reason: str):
        if not self.config.replay.enabled:
            return

        was_armed = self._replay_episode_armed
        self._replay_episode_armed = False
        if reason != "reached":
            self._clear_replay_buffer(f"return-home {reason}")
            return

        if not was_armed:
            self._clear_replay_buffer("return-home without recorder start")
            print("Replay skipped: B was pressed without an A-started recorder episode.")
            return

        if not self._load_sampled_replay_buffer():
            self._clear_replay_buffer("sampled replay unavailable")
            print("Replay skipped: sampled recorder trajectory is unavailable.")
            return

        self._replay_ready = True
        self.replay_pending_until = None
        self.replay_active = False
        self._replay_gripper_index = 0
        return_home_points = sum(
            point.get("phase") == "return_home"
            for point in self.replay_buffer
        )
        print(
            "Replay ready: "
            f"{len(self.replay_buffer)} points, "
            f"return_home={return_home_points}. Press "
            f"{self.config.replay.trigger_button} to start."
        )

    def _load_sampled_replay_buffer(self) -> bool:
        try:
            with open(SAMPLED_REPLAY_FILE, "r") as f:
                payload = json.load(f)
        except FileNotFoundError:
            print(f"Replay file is missing: {SAMPLED_REPLAY_FILE}")
            return False
        except (OSError, json.JSONDecodeError) as exc:
            print(f"Replay file is unreadable: {exc}")
            return False

        if isinstance(payload, dict):
            raw_points = payload.get("points")
            episode_dir = payload.get("episode_dir")
        else:
            raw_points = payload
            episode_dir = None

        if not isinstance(raw_points, list):
            print("Replay file does not contain a points list.")
            return False

        loaded_points = []
        previous_q = None
        previous_time = 0.0
        for raw_point in raw_points:
            if not isinstance(raw_point, dict):
                continue

            try:
                q = np.array(raw_point.get("q"), dtype=np.float64)
            except Exception:
                continue
            if q.size != 6 or not np.all(np.isfinite(q)):
                continue
            if previous_q is not None:
                q = self._nearest_joint_equivalent(q, previous_q)

            tcp = None
            try:
                raw_tcp = raw_point.get("tcp")
                if raw_tcp is not None:
                    tcp_array = np.array(raw_tcp, dtype=np.float64)
                    if tcp_array.size == 6 and np.all(np.isfinite(tcp_array)):
                        tcp = tcp_array
            except Exception:
                tcp = None

            try:
                replay_time = float(raw_point.get("time"))
            except (TypeError, ValueError):
                replay_time = previous_time
            if not np.isfinite(replay_time):
                replay_time = previous_time
            replay_time = max(previous_time, replay_time)

            gripper_raw = self._coerce_replay_gripper_raw(
                raw_point.get("gripper_target_raw")
            )
            if gripper_raw is None:
                gripper_raw = self._coerce_replay_gripper_raw(
                    raw_point.get("gripper_raw")
                )
            phase = raw_point.get("phase")
            if not isinstance(phase, str) or not phase:
                phase = "running"

            loaded_points.append(
                {
                    "time": float(replay_time),
                    "q": q.copy(),
                    "tcp": None if tcp is None else tcp.copy(),
                    "gripper_raw": gripper_raw,
                    "phase": phase,
                }
            )
            previous_q = q.copy()
            previous_time = float(replay_time)

        if len(loaded_points) < 2:
            print("Replay file does not contain enough sampled points.")
            return False

        self.replay_buffer = loaded_points
        self._last_replay_capture_time = None
        self._replay_capture_started_at = None
        self._replay_ready = False
        self.replay_pending_until = None
        self.replay_active = False
        self.replay_started_at = None
        self._replay_duration = 0.0
        self._replay_index = 0
        self._replay_gripper_index = 0
        source = f" from {episode_dir}" if isinstance(episode_dir, str) and episode_dir else ""
        print(
            f"Loaded {len(loaded_points)} sampled replay points{source}."
        )
        return True

    def _handle_replay_button(self) -> bool:
        if not self.config.replay.enabled:
            return False

        pressed = self._replay_button_pressed()
        was_pressed = self._replay_button_was_pressed
        self._replay_button_was_pressed = pressed
        if not pressed or was_pressed:
            return False

        if self.replay_active or self.replay_pending_until is not None:
            return True
        if not self._replay_ready:
            print("Replay ignored: no returned-home trajectory is ready.")
            return False
        if len(self.replay_buffer) < 2:
            self._clear_replay_buffer("not enough replay points")
            print("Replay skipped: not enough recorded trajectory points.")
            return False

        self._replay_ready = False
        delay_sec = max(0.0, float(self.config.replay.delay_sec))
        if delay_sec > 0.0:
            self.replay_pending_until = time.monotonic() + delay_sec
            self.replay_active = False
            self._replay_gripper_index = 0
            self._set_replay_gripper_target(self._first_replay_gripper_raw())
            print(f"Replay button accepted; starting after {delay_sec:.2f}s.")
        else:
            self._start_replay()
        return True

    def _replay_button_pressed(self) -> bool:
        pressed_any = False
        for button in self._replay_button_names():
            try:
                pressed_any = pressed_any or bool(self.xr.get_button(button))
                continue
            except Exception:
                pass

            try:
                value = float(self.xr.get_value(button))
                threshold = min(
                    1.0,
                    max(0.0, float(self.config.replay.trigger_threshold)),
                )
                pressed_any = pressed_any or value >= threshold
            except Exception as exc:
                self._warn_throttled(
                    f"Replay trigger {button!r} is unavailable: {exc}",
                    interval_sec=2.0,
                )
        return pressed_any

    def _replay_button_names(self) -> list[str]:
        names = [
            name.strip()
            for name in str(self.config.replay.trigger_button).split(",")
            if name.strip()
        ]
        return names or ["left_grip"]

    def _run_replay_if_needed(self, gripper_raw) -> bool:
        if self.replay_pending_until is None and not self.replay_active:
            return False

        now = time.monotonic()
        if self.replay_pending_until is not None:
            if now < self.replay_pending_until:
                first_gripper_raw = self._first_replay_gripper_raw()
                if first_gripper_raw is not None:
                    gripper_raw = first_gripper_raw
                    self._set_replay_gripper_target(first_gripper_raw)
                self._hold_speed_backend_alive()
                self._write_state("replay_wait", gripper_raw)
                time.sleep(0.01)
                return True
            self._start_replay()

        if not self.replay_active:
            return False

        target_q, target_tcp, qdot_ff, replay_gripper_raw = self._replay_target(now)
        current_q = self.arm.get_current_joints()
        self._update_actual_cache(q=current_q, tcp=self._tcp_from_joints(current_q))
        replay_gripper_raw = self._replay_gripper_for_current_q(
            current_q,
            replay_gripper_raw,
        )
        target_q = self._nearest_joint_equivalent(target_q, current_q)
        joint_error = target_q - current_q

        if replay_gripper_raw is not None and self.gripper is not None:
            gripper_raw = replay_gripper_raw
            self.gripper.set_target(int(replay_gripper_raw))

        if self.config.arm.control_mode == "speed_j_dls":
            qdot = qdot_ff + joint_error * float(self.config.replay.joint_gain)
            qdot = self._limit_joint_velocity(qdot)
            self._active_motion_kind = "speed"
            self.arm.speed_joints(qdot, self.config.arm.max_joint_acceleration)
            self._previous_joint_velocity = qdot.copy()
        else:
            previous_q = self.previous_target_q if self.previous_target_q is not None else current_q
            safe_q = self.safety.limit_joint_step(target_q, previous_q)
            target_q = safe_q.copy()
            self._active_motion_kind = "servo"
            self.arm.servo_joints(safe_q)

        self.previous_target_q = target_q.copy()
        if target_tcp is None:
            target_tcp = self._return_home_target_tcp(target_q)
        self.previous_target_tcp = target_tcp.copy()
        self._write_state("replay", gripper_raw, target_tcp, target_q)

        elapsed = (now - self.replay_started_at) * max(
            float(self.config.replay.speed_scale),
            1.0e-6,
        )
        if elapsed >= self._replay_duration:
            tolerance = max(0.0, float(self.config.replay.tolerance))
            if float(np.max(np.abs(joint_error))) <= tolerance:
                self._finish_replay("finished")
        return True

    def _start_replay(self):
        if len(self.replay_buffer) < 2:
            self.replay_pending_until = None
            return
        self.replay_pending_until = None
        self.replay_active = True
        self.replay_started_at = time.monotonic()
        self._replay_duration = max(0.0, float(self.replay_buffer[-1]["time"]))
        self._replay_index = 0
        self._replay_gripper_index = 0
        self._set_replay_gripper_target(self._first_replay_gripper_raw())
        current_q = self.arm.get_current_joints()
        current_tcp = self.arm.get_current_tcp_pose()
        self._update_actual_cache(q=current_q, tcp=current_tcp)
        self.previous_target_q = current_q
        self.previous_target_tcp = current_tcp
        self._previous_joint_velocity = np.zeros(6, dtype=np.float64)
        self._last_speed_cycle_time = time.monotonic()
        print(f"Replay started: duration={self._replay_duration:.2f}s.")

    def _replay_target(self, now: float):
        speed_scale = max(float(self.config.replay.speed_scale), 1.0e-6)
        elapsed = (now - self.replay_started_at) * speed_scale
        elapsed = max(0.0, min(elapsed, self._replay_duration))

        while (
            self._replay_index + 1 < len(self.replay_buffer)
            and self.replay_buffer[self._replay_index + 1]["time"] < elapsed
        ):
            self._replay_index += 1

        current = self.replay_buffer[self._replay_index]
        if self._replay_index + 1 >= len(self.replay_buffer):
            return (
                current["q"].copy(),
                None if current["tcp"] is None else current["tcp"].copy(),
                np.zeros(6, dtype=np.float64),
                current["gripper_raw"],
            )

        nxt = self.replay_buffer[self._replay_index + 1]
        dt = max(float(nxt["time"] - current["time"]), 1.0e-6)
        alpha = min(1.0, max(0.0, (elapsed - current["time"]) / dt))
        q = current["q"] + (nxt["q"] - current["q"]) * alpha
        qdot_ff = (nxt["q"] - current["q"]) / dt * speed_scale

        tcp = None
        if current["tcp"] is not None and nxt["tcp"] is not None:
            tcp = current["tcp"] + (nxt["tcp"] - current["tcp"]) * alpha
        elif current["tcp"] is not None:
            tcp = current["tcp"].copy()

        gripper_raw = current["gripper_raw"] if alpha < 0.5 else nxt["gripper_raw"]
        return q, tcp, qdot_ff, gripper_raw

    def _coerce_replay_gripper_raw(self, gripper_raw):
        if gripper_raw is None:
            return None
        try:
            raw = int(round(float(gripper_raw)))
        except (TypeError, ValueError):
            return None
        return max(
            DirectDHGripper.MIN_POSITION,
            min(raw, DirectDHGripper.MAX_POSITION),
        )

    def _first_replay_gripper_raw(self):
        if not self.replay_buffer:
            return None
        return self._coerce_replay_gripper_raw(
            self.replay_buffer[0].get("gripper_raw")
        )

    def _set_replay_gripper_target(self, gripper_raw):
        raw = self._coerce_replay_gripper_raw(gripper_raw)
        if raw is None or self.gripper is None:
            return raw
        self.gripper.set_target(raw)
        return raw

    def _replay_gripper_for_current_q(self, current_q, fallback_raw):
        fallback_raw = self._coerce_replay_gripper_raw(fallback_raw)
        if not self.replay_buffer:
            return fallback_raw

        try:
            q = np.array(current_q, dtype=np.float64)
        except (TypeError, ValueError):
            return fallback_raw
        if q.size != 6 or not np.all(np.isfinite(q)):
            return fallback_raw

        last_index = len(self.replay_buffer) - 1
        index = min(max(int(self._replay_gripper_index), 0), last_index)
        max_search_index = min(last_index, max(self._replay_index + 1, index))

        while index < max_search_index:
            current_distance = self._replay_joint_distance(
                q,
                self.replay_buffer[index]["q"],
            )
            next_distance = self._replay_joint_distance(
                q,
                self.replay_buffer[index + 1]["q"],
            )
            if next_distance > current_distance:
                break
            index += 1

        self._replay_gripper_index = index
        gripper_raw = self._coerce_replay_gripper_raw(
            self.replay_buffer[index].get("gripper_raw")
        )
        return fallback_raw if gripper_raw is None else gripper_raw

    def _replay_joint_distance(self, current_q, replay_q) -> float:
        replay_q = np.array(replay_q, dtype=np.float64)
        current_q = self._nearest_joint_equivalent(current_q, replay_q)
        return float(np.linalg.norm(current_q - replay_q))

    def _finish_replay(self, reason: str):
        self._stop_active_motion()
        self.replay_active = False
        self.replay_pending_until = None
        self.replay_started_at = None
        self._replay_duration = 0.0
        self._replay_gripper_index = 0
        self.previous_target_q = None
        self.previous_target_tcp = None
        self._previous_joint_velocity = np.zeros(6, dtype=np.float64)
        self._last_speed_cycle_time = None
        self.mapper.reset()
        self._motion_locked_until_grip_release = True
        self._grip_active = False
        print(f"Replay {reason}. Release grip before teleop resumes.")

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

        target_q = self._nearest_joint_equivalent(target_q, q_near)
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
        qdot, min_singular, singular_scale, damping = self._dls_joint_velocity(
            jacobian,
            twist,
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
            position_error = self._apply_deadband(
                target_position - current_position,
                self.config.arm.dls_position_deadband,
            )
        linear = position_error * self.config.arm.dls_linear_gain
        rotation_error = target_rotation @ current_rotation.T
        if self.config.safety.disable_teleop_limits:
            rotation_vector = matrix_to_rotvec(rotation_error)
        else:
            rotation_vector = self._apply_deadband(
                matrix_to_rotvec(rotation_error),
                self.config.arm.dls_orientation_deadband,
            )
        angular = rotation_vector * self.config.arm.dls_angular_gain

        if not self.config.safety.disable_teleop_limits:
            linear = self._limit_vector_norm(linear, self.config.arm.max_tcp_linear_speed)
            angular = self._limit_vector_norm(angular, self.config.arm.max_tcp_angular_speed)
        return np.concatenate((linear, angular))

    def _dls_joint_velocity(self, jacobian: np.ndarray, twist: np.ndarray):
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        min_singular = float(np.min(singular_values))
        hard_stop = max(0.0, float(self.config.arm.dls_hard_stop_threshold))
        soft_stop = max(hard_stop + 1.0e-6, float(self.config.arm.dls_singularity_threshold))

        if min_singular >= soft_stop:
            singular_scale = 1.0
            singular_ratio = 0.0
        elif min_singular <= hard_stop:
            singular_scale = 0.0
            singular_ratio = 1.0
        else:
            singular_scale = (min_singular - hard_stop) / (soft_stop - hard_stop)
            singular_ratio = 1.0 - singular_scale

        base_damping = max(0.0, float(self.config.arm.dls_damping))
        max_damping = max(base_damping, float(self.config.arm.dls_max_damping))
        damping = base_damping + (max_damping - base_damping) * singular_ratio * singular_ratio

        scaled_twist = np.array(twist, dtype=np.float64) * singular_scale
        regularizer = (damping * damping) * np.eye(6, dtype=np.float64)
        try:
            qdot = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + regularizer,
                scaled_twist,
            )
        except np.linalg.LinAlgError:
            qdot = np.zeros(6, dtype=np.float64)
        if not np.all(np.isfinite(qdot)):
            qdot = np.zeros(6, dtype=np.float64)
        return qdot, min_singular, singular_scale, damping

    def _limit_joint_velocity(self, qdot: np.ndarray) -> np.ndarray:
        limited = np.array(qdot, dtype=np.float64)
        if self.config.safety.disable_teleop_limits:
            self._last_speed_cycle_time = time.monotonic()
            return limited

        max_speed = abs(float(self.config.arm.max_joint_speed))
        if max_speed > 0.0:
            limited = self._scale_to_component_limit(limited, max_speed)

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
                + self._scale_to_component_limit(delta, max_delta)
            )
        return limited

    @staticmethod
    def _scale_to_component_limit(vector: np.ndarray, component_limit: float) -> np.ndarray:
        values = np.array(vector, dtype=np.float64)
        limit = abs(float(component_limit))
        max_abs = float(np.max(np.abs(values))) if values.size else 0.0
        if limit > 0.0 and max_abs > limit:
            values *= limit / max_abs
        return values

    @staticmethod
    def _limit_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
        values = np.array(vector, dtype=np.float64)
        limit = abs(float(max_norm))
        norm = float(np.linalg.norm(values))
        if limit > 0.0 and norm > limit:
            values *= limit / norm
        return values

    @staticmethod
    def _apply_deadband(vector: np.ndarray, deadband: float) -> np.ndarray:
        values = np.array(vector, dtype=np.float64)
        threshold = max(0.0, float(deadband))
        norm = float(np.linalg.norm(values))
        if threshold <= 0.0 or norm <= 0.0:
            return values
        if norm <= threshold:
            return np.zeros_like(values)
        return values * ((norm - threshold) / norm)

    @staticmethod
    def _nearest_joint_equivalent(target_q, reference_q) -> np.ndarray:
        target = np.array(target_q, dtype=np.float64)
        reference = np.array(reference_q, dtype=np.float64)
        delta = (target - reference + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def _run_return_home_cycle(self, gripper_raw):
        if self.config.arm.control_mode == "speed_j_dls":
            return self._run_return_home_speed_cycle(gripper_raw)
        return self._run_return_home_servo_cycle(gripper_raw)

    def _run_return_home_speed_cycle(self, gripper_raw):
        current_q = self.arm.get_current_joints()
        self._update_actual_cache(q=current_q, tcp=self._tcp_from_joints(current_q))
        target_q = self._nearest_joint_equivalent(self.return_home_target_q, current_q)
        if self._return_home_joint_error(current_q) <= self.config.return_home.tolerance:
            self.previous_target_q = target_q.copy()
            self._previous_joint_velocity = np.zeros(6, dtype=np.float64)
            self._active_motion_kind = "speed"
            target_tcp = self._return_home_target_tcp(target_q)
            self._write_state(
                "return_home",
                gripper_raw,
                target_tcp,
                target_q,
            )
            self._capture_replay_point(
                target_q,
                target_tcp,
                gripper_raw,
                phase="return_home",
            )
            self._update_return_home_progress(current_q)
            return

        qdot = self._return_home_joint_velocity(current_q)
        target_q = self._return_home_speed_target_q(current_q, qdot)
        self.previous_target_q = target_q.copy()
        self._previous_joint_velocity = qdot.copy()

        self._active_motion_kind = "speed"
        self.arm.speed_joints(qdot, self.config.arm.max_joint_acceleration)
        target_tcp = self._return_home_target_tcp(target_q)
        self._write_state("return_home", gripper_raw, target_tcp, target_q)
        self._capture_replay_point(
            target_q,
            target_tcp,
            gripper_raw,
            phase="return_home",
        )
        self._update_return_home_progress(current_q)

    def _run_return_home_servo_cycle(self, gripper_raw):
        current_q = self.arm.get_current_joints()
        self._update_actual_cache(q=current_q, tcp=self._tcp_from_joints(current_q))
        previous_q = self.previous_target_q if self.previous_target_q is not None else current_q
        safe_q = self._limit_return_home_joint_step(self.return_home_target_q, previous_q)
        self.previous_target_q = safe_q.copy()

        self._active_motion_kind = "servo"
        self.arm.servo_joints(safe_q)
        target_tcp = self._return_home_target_tcp(safe_q)
        self._write_state("return_home", gripper_raw, target_tcp, safe_q)
        self._capture_replay_point(
            safe_q,
            target_tcp,
            gripper_raw,
            phase="return_home",
        )
        self._update_return_home_progress(current_q)

    def _update_return_home_progress(self, current_q):
        now = time.monotonic()
        joint_error = self._return_home_joint_error(current_q)
        if joint_error <= self.config.return_home.tolerance:
            if self.return_home_reached_since is None:
                self.return_home_reached_since = now
            if now - self.return_home_reached_since >= self.config.return_home.hold_sec:
                self._finish_return_home("reached")
            return

        self.return_home_reached_since = None
        elapsed = now - self.return_home_started_at
        if elapsed >= self.config.return_home.timeout_sec:
            self._finish_return_home(f"timed out at joint_error={joint_error:.4f}rad")

    def _return_home_joint_error(self, current_q) -> float:
        current = np.array(current_q, dtype=np.float64)
        target = self._nearest_joint_equivalent(self.return_home_target_q, current)
        return float(np.max(np.abs(target - current)))

    def _return_home_joint_velocity(self, current_q):
        current = np.array(current_q, dtype=np.float64)
        target = self._nearest_joint_equivalent(self.return_home_target_q, current)
        error = target - current
        abs_error = np.abs(error)

        max_speed = abs(float(self.config.arm.max_joint_speed))
        max_step = abs(float(self.config.return_home.max_joint_step))
        if max_step > 0.0:
            step_speed = max_step / max(self.arm.servo_time, 1.0e-6)
            max_speed = step_speed if max_speed <= 0.0 else min(max_speed, step_speed)

        acceleration = abs(float(self.config.arm.max_joint_acceleration))
        tolerance = max(0.0, float(self.config.return_home.tolerance))
        if acceleration > 0.0:
            braking_speed = np.sqrt(2.0 * acceleration * abs_error) * 0.5
        else:
            braking_speed = np.full(6, max_speed, dtype=np.float64)

        crawl_gain = 2.5
        crawl_speed = crawl_gain * abs_error
        desired_speed = np.minimum(braking_speed, crawl_speed)
        if max_speed > 0.0:
            desired_speed = np.minimum(desired_speed, max_speed)
        desired = np.sign(error) * desired_speed
        desired[abs_error <= tolerance] = 0.0
        limited = self._limit_joint_velocity(desired)

        max_no_overshoot_speed = abs_error / max(self.arm.servo_time, 1.0e-6)
        limited = np.sign(error) * np.minimum(np.abs(limited), max_no_overshoot_speed)
        limited[abs_error <= tolerance] = 0.0
        return limited

    def _return_home_speed_target_q(self, current_q, qdot):
        current = np.array(current_q, dtype=np.float64)
        target = self._nearest_joint_equivalent(self.return_home_target_q, current)
        next_q = current + np.array(qdot, dtype=np.float64) * self.arm.servo_time

        lower = np.minimum(current, target)
        upper = np.maximum(current, target)
        return np.minimum(np.maximum(next_q, lower), upper)

    def _return_home_target_tcp(self, target_q):
        if self.local_kinematics is not None:
            try:
                return self.local_kinematics.forward_pose(target_q, self.tcp_offset)
            except Exception as exc:
                self._warn_throttled(f"Local return-home FK failed: {exc}")

        target_tcp = self.arm.forward_kinematics(target_q)
        if target_tcp is not None:
            return target_tcp

        detail = self.arm.last_forward_kinematics_error
        if detail:
            self._warn_throttled(f"RTDE return-home FK failed; using current TCP: {detail}")
        return self.arm.get_current_tcp_pose()

    def _limit_return_home_joint_step(self, target_q, previous_q):
        target = np.array(target_q, dtype=np.float64)
        previous = np.array(previous_q, dtype=np.float64)
        delta = target - previous
        max_step = float(self.config.return_home.max_joint_step)
        if max_step > 0.0:
            delta = np.clip(delta, -max_step, max_step)
        return previous + delta

    def _finish_return_home(self, reason: str):
        self._stop_active_motion()
        gripper_raw = self._current_gripper_target_raw()
        targets = self._write_return_home_state(
            gripper_raw,
            final=(reason == "reached"),
            force=True,
        )
        if targets is not None:
            target_q, target_tcp = targets
            self._capture_replay_point(
                target_q,
                target_tcp,
                gripper_raw,
                force=True,
                phase="return_home",
            )
        stop_grace_sec = max(0.0, float(self.config.return_home.stop_grace_sec))
        if stop_grace_sec > 0.0:
            time.sleep(stop_grace_sec)
        self.return_home_active = False
        self.return_home_started_at = None
        self.return_home_reached_since = None
        self.previous_target_tcp = None
        self.previous_target_q = None
        self.mapper.reset()
        self._motion_locked_until_grip_release = False
        self._grip_active = False
        print(f"Return-home {reason}; requesting recorder stop.")
        recorder_stopped = True
        if self.recorder_control is not None:
            recorder_stopped = self.recorder_control.request_blocking(
                "stop",
                timeout_sec=3.0,
            )
        if recorder_stopped:
            self._schedule_replay_after_return_home(reason)
        else:
            self._clear_replay_buffer("recorder stop did not complete")
            print("Replay skipped: recorder stop did not complete.")

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


def configure_low_latency_runtime(priority: int, lock_memory: bool):
    if lock_memory:
        try:
            libc = ctypes.CDLL("libc.so.6", use_errno=True)
            mcl_current = 1
            mcl_future = 2
            if libc.mlockall(mcl_current | mcl_future) != 0:
                errno = ctypes.get_errno()
                raise OSError(errno, os.strerror(errno))
            print("Locked teleop process memory with mlockall.")
        except Exception as exc:
            print(f"Warning: failed to lock memory for low latency: {exc}")

    priority = int(priority)
    if priority <= 0:
        return
    try:
        os.sched_setscheduler(
            0,
            os.SCHED_FIFO,
            os.sched_param(priority),
        )
        print(f"Teleop process scheduler set to SCHED_FIFO priority {priority}.")
    except PermissionError as exc:
        print(
            "Warning: failed to set real-time scheduler. "
            "Configure realtime limits for this user first "
            f"(requested SCHED_FIFO priority {priority}): {exc}"
        )
    except Exception as exc:
        print(f"Warning: failed to set real-time scheduler: {exc}")


def configure_xr_service_runtime(priority: int):
    priority = int(priority)
    if priority <= 0:
        return

    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open(f"/proc/{name}/comm", "r") as f:
                comm = f.read().strip()
        except OSError:
            continue
        if comm == "RoboticsService":
            pids.append(int(name))

    if not pids:
        print(
            "Warning: XR service process was not found; "
            "start XRoboToolkit PC Service before teleop."
        )
        return

    for pid in pids:
        try:
            os.sched_setscheduler(
                pid,
                os.SCHED_FIFO,
                os.sched_param(priority),
            )
            print(f"XR service PID {pid} set to SCHED_FIFO priority {priority}.")
        except PermissionError as exc:
            print(
                "Warning: failed to set XR service real-time scheduler "
                f"for PID {pid}: {exc}"
            )
        except Exception as exc:
            print(f"Warning: failed to boost XR service PID {pid}: {exc}")


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
