"""Read the lightweight state file written by the high-performance teleop process."""

import json
import time
from pathlib import Path

from recorder.config import TELEOP_STATE_FILE, TELEOP_STATE_TIMEOUT_SEC
from recorder.gripper_bridge import raw_to_norm


class TeleopStateReader:
    def __init__(
        self,
        state_file: str = TELEOP_STATE_FILE,
        timeout_sec: float = TELEOP_STATE_TIMEOUT_SEC,
    ):
        self.state_file = Path(state_file)
        self.timeout_sec = float(timeout_sec)

    def read_state(self) -> dict:
        if not self.state_file.exists():
            return {"available": False, "reason": "missing"}

        try:
            with open(self.state_file, "r") as f:
                state = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            return {"available": False, "reason": f"read_error:{exc}"}

        wall_time = state.get("wall_time")
        age_sec = None
        fresh = False
        if isinstance(wall_time, (int, float)):
            age_sec = time.time() - float(wall_time)
            fresh = age_sec <= self.timeout_sec

        target_raw = state.get("gripper_target_raw")
        current_raw = state.get("gripper_current_raw")

        result = {
            "available": True,
            "fresh": fresh,
            "age_sec": age_sec,
            "status": state.get("status"),
            "control_mode": state.get("control_mode"),
            "actual_joint_positions": state.get("actual_q"),
            "actual_tcp_pose": state.get("actual_tcp_pose"),
            "target_joint_positions": state.get("target_q"),
            "target_tcp_pose": state.get("target_tcp_pose"),
            "motion_grip_value": state.get("motion_grip_value"),
            "motion_enabled": state.get("motion_enabled"),
            "xr_age_sec": state.get("xr_age_sec"),
            "gripper_target_position": None,
            "gripper_current_position": None,
        }

        if target_raw is not None:
            result["gripper_target_position"] = raw_to_norm(target_raw)
        if current_raw is not None:
            result["gripper_current_position"] = raw_to_norm(current_raw)
        return result
