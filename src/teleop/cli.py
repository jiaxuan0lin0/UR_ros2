import argparse
from pathlib import Path

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
