"""Utilities for converting gripper values between raw register space and normalized space."""

import numpy as np
from recorder.config import GRIPPER_RAW_MIN, GRIPPER_RAW_MAX

def raw_to_norm(value: int | float) -> float:
    clipped = float(np.clip(value, GRIPPER_RAW_MIN, GRIPPER_RAW_MAX))
    return (clipped - GRIPPER_RAW_MIN) / float(GRIPPER_RAW_MAX - GRIPPER_RAW_MIN)

def norm_to_raw(value:float) -> int:
    clipped = float(np.clip(value, 0.0, 1.0))
    return int(round(clipped * (GRIPPER_RAW_MAX - GRIPPER_RAW_MIN) + GRIPPER_RAW_MIN))