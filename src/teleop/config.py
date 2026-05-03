from dataclasses import dataclass
from pathlib import Path


@dataclass
class ArmConfig:
    robot_ip: str = "192.168.1.102"
    control_mode: str = "servo_j"
    rate_hz: float = 250.0
    max_velocity: float = 0.5
    max_acceleration: float = 1.0
    lookahead_time: float = 0.06
    servo_gain: float = 500.0


@dataclass
class XrConfig:
    pose_source: str = "right_controller"
    grip_name: str = "right_grip"
    trigger_name: str = "right_trigger"
    emergency_button: str = "right_axis_click"
    grip_threshold: float = 0.95
    grip_release_threshold: float = 0.8
    timeout_sec: float = 0.2
    scale_factor: float = 0.6
    rotation_scale: float = 1.0


@dataclass
class SafetyConfig:
    max_step_translation: float = 0.01
    max_step_rotation: float = 0.05
    max_joint_step: float = 0.04
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
    read_state_hz: float = 5.0


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


@dataclass
class TeleopConfig:
    arm: ArmConfig
    xr: XrConfig
    safety: SafetyConfig
    gripper: GripperConfig
    state: StateConfig
    recorder: RecorderControlConfig
    return_home: ReturnHomeConfig
