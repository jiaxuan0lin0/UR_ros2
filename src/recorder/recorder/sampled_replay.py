"""Build and persist replay trajectories from sampled recorder data."""

import json
import math
from pathlib import Path

from recorder.gripper_bridge import norm_to_raw


def _as_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _vector(record: dict, section: str, field: str):
    if not isinstance(record, dict):
        return None
    payload = record.get(section)
    if not isinstance(payload, dict):
        return None
    value = payload.get(field)
    if not isinstance(value, list) or len(value) != 6:
        return None
    parsed = [_as_float(item) for item in value]
    if any(item is None for item in parsed):
        return None
    return parsed


def _action_gripper_raw(record: dict):
    if not isinstance(record, dict):
        return None
    payload = record.get("action")
    if not isinstance(payload, dict):
        return None
    grip_norm = _as_float(payload.get("gripper_position"))
    if grip_norm is None:
        return None
    return int(norm_to_raw(grip_norm))


def _observation_gripper_raw(record: dict):
    if not isinstance(record, dict):
        return None
    payload = record.get("observation")
    if not isinstance(payload, dict):
        return None
    grip_norm = _as_float(payload.get("gripper_position"))
    if grip_norm is None:
        return None
    return int(norm_to_raw(grip_norm))


def build_sampled_replay_payload(records, episode_dir: Path | None = None) -> dict:
    points = []
    start_timestamp = None
    last_time = 0.0
    for record in records:
        joint_positions = _vector(record, "action", "joint_positions")
        if joint_positions is None:
            continue
        observation_joint_positions = _vector(record, "observation", "joint_positions")
        action_tcp_pose = _vector(record, "action", "tcp_pose")
        observation_tcp_pose = _vector(record, "observation", "tcp_pose")

        timestamp = _as_float(record.get("timestamp"))
        if timestamp is not None and start_timestamp is None:
            start_timestamp = timestamp
        if start_timestamp is not None and timestamp is not None:
            replay_time = max(last_time, timestamp - start_timestamp)
        else:
            replay_time = last_time

        gripper_raw = _observation_gripper_raw(record)
        gripper_target_raw = _action_gripper_raw(record)
        phase = record.get("status")
        if not isinstance(phase, str) or not phase:
            phase = "running"

        points.append(
            {
                "time": float(replay_time),
                "q": joint_positions,
                "tcp": action_tcp_pose,
                "action_q": joint_positions,
                "action_tcp": action_tcp_pose,
                "observation_q": observation_joint_positions,
                "observation_tcp": observation_tcp_pose,
                "gripper_raw": gripper_target_raw,
                "gripper_target_raw": gripper_target_raw,
                "gripper_observation_raw": gripper_raw,
                "phase": phase,
            }
        )
        last_time = float(replay_time)

    return {
        "episode_dir": None if episode_dir is None else str(episode_dir),
        "point_count": len(points),
        "points": points,
    }


def write_sampled_replay_file(path: Path, records, episode_dir: Path | None = None) -> dict:
    payload = build_sampled_replay_payload(records, episode_dir=episode_dir)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(path.suffix + ".tmp")
    with open(temp_path, "w") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    temp_path.replace(path)
    return payload


def clear_sampled_replay_file(path: Path) -> None:
    try:
        Path(path).unlink()
    except FileNotFoundError:
        pass
