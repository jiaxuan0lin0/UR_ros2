"""Policy action adapter for joint-absolute UR RTDE control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from policy_client.types import ACTION_DIM, PolicyObservation, PolicyResult


UR_ARM_JOINT_COUNT = 6

DEFAULT_UR_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "gripper_joint",
)


@dataclass
class JointTrajectoryPointData:
    positions: list[float]
    time_from_start_sec: float


@dataclass
class JointTrajectoryGoal:
    joint_names: list[str]
    points: list[JointTrajectoryPointData]


@dataclass
class JointTrajectoryAdapterConfig:
    joint_command_mode: str = "absolute"
    use_action_chunk: bool = True
    chunk_size: int = 50
    point_duration_sec: float = 1.0 / 30.0
    max_joint_delta: float = 0.00136
    gripper_input_mode: str = "normalized"
    gripper_raw_min: float = 0.0
    gripper_raw_max: float = 1000.0


class JointTrajectoryActionAdapter:
    """Convert 7D joint-space policy outputs into trajectory goals."""

    def __init__(self, cfg: JointTrajectoryAdapterConfig):
        self.cfg = cfg
        self.joint_command_mode = _normalize_joint_command_mode(
            cfg.joint_command_mode
        )

    def select_actions(self, result: PolicyResult) -> list[np.ndarray]:
        if self.cfg.use_action_chunk:
            count = max(1, int(self.cfg.chunk_size))
            actions = result.action_chunk[:count]
        else:
            actions = result.action[None, :]

        selected = []
        for action in actions:
            values = _coerce_action(action).copy()
            if self.joint_command_mode == "delta":
                values[:UR_ARM_JOINT_COUNT] = _limit_vector_norm(
                    values[:UR_ARM_JOINT_COUNT],
                    float(self.cfg.max_joint_delta),
                )
            selected.append(values)
        return selected

    def build_goal(
        self,
        actions: Iterable[Sequence[float]],
        *,
        observation: PolicyObservation | None = None,
    ) -> JointTrajectoryGoal:
        values_list = [_coerce_action(action) for action in actions]
        points: list[JointTrajectoryPointData] = []

        if self.joint_command_mode == "delta":
            running_arm = _current_arm_joints(observation)
        else:
            running_arm = None

        point_duration_sec = max(float(self.cfg.point_duration_sec), 0.0)
        for index, values in enumerate(values_list):
            if self.joint_command_mode == "delta":
                running_arm = running_arm + values[:UR_ARM_JOINT_COUNT]
                arm_positions = running_arm
            else:
                arm_positions = values[:UR_ARM_JOINT_COUNT]

            points.append(
                JointTrajectoryPointData(
                    positions=[
                        *[float(value) for value in arm_positions],
                        gripper_to_normalized_value(
                            values[UR_ARM_JOINT_COUNT],
                            mode=self.cfg.gripper_input_mode,
                            raw_min=self.cfg.gripper_raw_min,
                            raw_max=self.cfg.gripper_raw_max,
                        ),
                    ],
                    time_from_start_sec=point_duration_sec * float(index + 1),
                )
            )

        return JointTrajectoryGoal(
            joint_names=list(DEFAULT_UR_JOINT_NAMES),
            points=points,
        )


def gripper_to_normalized_value(
    value: float,
    *,
    mode: str,
    raw_min: float,
    raw_max: float,
) -> float:
    normalized_mode = mode.lower().strip()
    if normalized_mode in {"normalized", "norm", "unit"}:
        return float(np.clip(value, 0.0, 1.0))
    if normalized_mode == "raw":
        raw_range = float(raw_max) - float(raw_min)
        if raw_range <= 0.0:
            raise ValueError("gripper raw range must be positive")
        normalized = (float(value) - float(raw_min)) / raw_range
        return float(np.clip(normalized, 0.0, 1.0))
    raise ValueError(
        'gripper_input_mode must be "normalized" or "raw", got '
        f"{mode}"
    )


def _limit_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    if max_norm <= 0.0:
        return vector
    norm = float(np.linalg.norm(vector))
    if norm <= max_norm or norm <= 0.0:
        return vector
    return vector * (max_norm / norm)


def _normalize_joint_command_mode(mode: str) -> str:
    normalized = str(mode).strip().lower()
    if normalized in {"absolute", "abs", "position", "positions"}:
        return "absolute"
    if normalized in {"delta", "relative", "rel"}:
        return "delta"
    raise ValueError(
        'joint_command_mode must be "absolute" or "delta", got '
        f"{mode}"
    )


def _current_arm_joints(observation: PolicyObservation | None) -> np.ndarray:
    if observation is None:
        raise ValueError(
            'joint_command_mode="delta" requires current joint_positions '
            "in the policy observation"
        )
    robot_state = getattr(observation, "robot_state", {}) or {}
    expected_shape = (UR_ARM_JOINT_COUNT,)
    joints = np.asarray(robot_state.get("joint_positions"), dtype=np.float64)
    if joints.shape != expected_shape:
        raise ValueError(
            'joint_command_mode="delta" requires robot_state["joint_positions"] '
            f"with shape {expected_shape}, got {joints.shape}"
        )
    if not np.all(np.isfinite(joints)):
        raise ValueError("current joint_positions contains non-finite values")
    return joints.copy()


def _coerce_action(action: Sequence[float]) -> np.ndarray:
    values = np.asarray(action, dtype=np.float64)
    if values.shape != (ACTION_DIM,):
        raise ValueError(f"action must have shape ({ACTION_DIM},), got {values}")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"action contains non-finite values: {values}")
    return values
