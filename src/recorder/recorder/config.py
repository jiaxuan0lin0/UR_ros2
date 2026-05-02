from pathlib import Path

UR_IP = "192.168.1.102"
RECORD_HZ = 30

DATASET_ROOT = Path.home() / "UR" / "dataset"

GRIPPER_OUTPUT_REGISTER = 12
GRIPPER_RAW_MIN = 0
GRIPPER_RAW_MAX = 1000
TELEOP_STATE_FILE = "/tmp/ur_teleop_state.json"
TELEOP_STATE_TIMEOUT_SEC = 1.0

DEFAULT_RTDE_FREQUENCY = -1.0  # use the robot's default frequency

EPISODE_PREFIX = "episode_"
IMAGE_DIR_PREFIX = "cam_"
STATE_FILE_NAME = "robot_states.json"

COLOR_WIDTH = 640
COLOR_HEIGHT = 480
COLOR_FPS = 30
