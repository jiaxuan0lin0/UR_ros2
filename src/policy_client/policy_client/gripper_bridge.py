"""Convert DH gripper raw register values to normalized policy values."""

import numpy as np

from policy_client.config import GRIPPER_RAW_MAX, GRIPPER_RAW_MIN


def raw_to_norm(value: int | float) -> float:
    clipped = float(np.clip(value, GRIPPER_RAW_MIN, GRIPPER_RAW_MAX))
    raw_range = float(GRIPPER_RAW_MAX - GRIPPER_RAW_MIN)
    return (clipped - float(GRIPPER_RAW_MIN)) / raw_range


def norm_to_raw(value: float) -> int:
    clipped = float(np.clip(value, 0.0, 1.0))
    raw_range = float(GRIPPER_RAW_MAX - GRIPPER_RAW_MIN)
    return int(round(clipped * raw_range + float(GRIPPER_RAW_MIN)))
