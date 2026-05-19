from dataclasses import dataclass
from pathlib import Path


@dataclass
class ArmConfig:
    robot_ip: str = "192.168.1.102"
    ur_type: str = "ur7e"
    control_mode: str = "servo_j"
    rate_hz: float = 500.0
    max_velocity: float = 0.5
    max_acceleration: float = 1.0
    lookahead_time: float = 0.06
    servo_gain: float = 500.0
    speedj_backend: str = "direct"
    speedj_register_start: int = 42
    speedj_velocity_scale: float = 10000.0
    speedj_watchdog_sec: float = 0.06
    speedj_stale_stop_sec: float = 1.0
    speedj_upload_port: int = 30001
    speed_slider_fraction: float = 1.0
    realtime_priority: int = 0
    lock_memory: bool = False
    control_latency_warning_sec: float = 0.02
    ik_max_position_error: float = 1.0e-4
    ik_max_orientation_error: float = 1.0e-3
    dls_linear_gain: float = 8.0
    dls_angular_gain: float = 6.0
    dls_damping: float = 0.03
    dls_max_damping: float = 0.25
    dls_position_deadband: float = 0.0
    dls_orientation_deadband: float = 0.0
    dls_singularity_threshold: float = 0.05
    dls_hard_stop_threshold: float = 0.015
    dls_frame_name: str = "tool0"
    dls_jacobian_source: str = "local"
    ur_description_path: str | None = None
    max_tcp_linear_speed: float = 0.25
    max_tcp_angular_speed: float = 0.8
    max_joint_speed: float = 0.8
    max_joint_acceleration: float = 2.0


@dataclass
class XrConfig:
    pose_source: str = "right_controller"
    grip_name: str = "right_grip"
    trigger_name: str = "right_trigger"
    emergency_button: str = "right_axis_click"
    service_realtime_priority: int = 0
    grip_threshold: float = 0.95
    grip_release_threshold: float = 0.8
    grip_release_grace_sec: float = 0.0
    timeout_sec: float = 0.2
    target_interpolation_sec: float = 0.0
    scale_factor: float = 0.6
    rotation_scale: float = 1.0


@dataclass
class SafetyConfig:
    disable_teleop_limits: bool = False
    max_step_translation: float = 0.01
    max_step_rotation: float = 0.05
    max_joint_step: float = 0.04
    target_smoothing_mode: str = "linear"
    target_smoothing_sec: float = 0.0
    ruckig_max_linear_acceleration: float = 1.5
    ruckig_max_linear_jerk: float = 30.0
    ruckig_target_prediction_sec: float = 0.006
    ruckig_velocity_hold_sec: float = 0.015
    max_target_lag_translation: float = 0.0
    max_target_lag_rotation: float = 0.0
    workspace_min: tuple[float, float, float] | None = None
    workspace_max: tuple[float, float, float] | None = None


@dataclass
class GripperConfig:
    enabled: bool = True
    port: str = "/tmp/ttyUR"
    baudrate: int = 115200
    rate_hz: float = 30.0
    speed: int = 100
    force: int = 50
    open_raw: int = 1000
    closed_raw: int = 0
    binary_control: bool = False
    trigger_threshold: float = 0.5
    position_deadband: int = 4
    init_timeout_sec: float = 8.0
    read_state_hz: float = 30.0


@dataclass
class StateConfig:
    enabled: bool = True
    path: Path = Path("/tmp/ur_teleop_state.json")
    rate_hz: float = 30.0


@dataclass
class RecorderControlConfig:
    enabled: bool = False
    start_button: str = "A"
    stop_button: str = "B"
    save_button: str = "X"
    discard_button: str = "Y"
    debounce_sec: float = 0.5


@dataclass
class ReturnHomeConfig:
    enabled: bool = True
    joint_positions: tuple[float, float, float, float, float, float] = (
        -1.5708139578448694,
        -1.5707999668517054,
        -1.570805549621582,
        -1.570810934106344,
        1.5707950592041016,
        -0.02949411073793584,
    )
    max_joint_step: float = 0.0006
    tolerance: float = 0.01
    timeout_sec: float = 20.0
    hold_sec: float = 0.3
    stop_grace_sec: float = 0.2


@dataclass
class ReplayConfig:
    enabled: bool = False
    trigger_button: str = "left_grip"
    trigger_threshold: float = 0.8
    delay_sec: float = 2.0
    record_hz: float = 30.0
    speed_scale: float = 1.0
    joint_gain: float = 10.0
    tolerance: float = 0.02


@dataclass
class TeleopConfig:
    arm: ArmConfig
    xr: XrConfig
    safety: SafetyConfig
    gripper: GripperConfig
    state: StateConfig
    recorder: RecorderControlConfig
    return_home: ReturnHomeConfig
    replay: ReplayConfig
