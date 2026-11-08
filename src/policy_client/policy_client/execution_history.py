"""Executed-action and image history buffered between policy requests."""

from __future__ import annotations

from typing import Optional

import numpy as np


def _observation_gripper(observation) -> Optional[float]:
    raw_proprio = getattr(observation, 'proprio', None)
    if raw_proprio is not None:
        proprio = np.asarray(raw_proprio, dtype=np.float64)
        if proprio.shape == (7,) and np.isfinite(proprio[6]):
            return float(proprio[6])
    robot_state = getattr(observation, 'robot_state', {}) or {}
    value = robot_state.get('gripper_position')
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def _observation_joint_action(observation) -> Optional[np.ndarray]:
    robot_state = getattr(observation, 'robot_state', {}) or {}
    raw_joints = robot_state.get('joint_positions')
    joints = np.asarray(raw_joints, dtype=np.float64)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raw_proprio = getattr(observation, 'proprio', None)
        proprio = np.asarray(raw_proprio, dtype=np.float64)
        if proprio.shape != (7,) or not np.all(np.isfinite(proprio[:6])):
            return None
        joints = proprio[:6]
    gripper = _observation_gripper(observation)
    if gripper is None:
        return None
    return np.concatenate([joints, np.asarray([gripper], dtype=np.float64)])


class ExecutionHistoryMixin:
    """Pending history buffers; state lives in PolicyControlNode.__init__."""

    def _reset_policy_history(self) -> None:
        self._clear_async_policy_request(increment_generation=True)
        with self._policy_lock:
            self.policy.reset()
        self.rtc_control.reset()
        with self._state_lock:
            self._pending_executed_actions = (
                [[0.0] * 7] if self.send_initial_zero_action else None
            )
            self._pending_history_images = None

    def _history_actions_for_request(self) -> Optional[list[list[float]]]:
        if self.history_update_mode == 'server_auto':
            return None
        if self.history_update_mode != 'last_action':
            raise ValueError(
                'history_update_mode must be "last_action" or "server_auto"'
            )
        with self._state_lock:
            return self._pending_executed_actions

    def _history_images_for_request(self) -> Optional[list[np.ndarray]]:
        with self._state_lock:
            return self._pending_history_images

    def _mark_history_actions_sent(
        self,
        actions: Optional[list[list[float]]],
    ) -> None:
        if actions is None or self.history_update_mode != 'last_action':
            return
        with self._state_lock:
            if self._pending_executed_actions is actions:
                self._pending_executed_actions = None

    def _mark_history_images_sent(
        self,
        images: Optional[list[np.ndarray]],
    ) -> None:
        if images is None:
            return
        with self._state_lock:
            if self._pending_history_images is images:
                self._pending_history_images = None

    def _store_execution_history(
        self,
        *,
        start_observation,
        execution_observations: list[object],
    ) -> None:
        if self.history_update_mode != 'last_action':
            return

        observations = [obs for obs in execution_observations if obs is not None]
        if not observations:
            return

        actions = self._actual_actions_from_observations(
            start_observation,
            observations,
        )
        images = [
            np.asarray(obs.primary_image).copy()
            for obs in observations
            if getattr(obs, 'primary_image', None) is not None
        ]
        if self.max_pending_history_frames > 0:
            images = images[-self.max_pending_history_frames:]

        with self._state_lock:
            if actions:
                self._pending_executed_actions = actions
            if images:
                self._pending_history_images = images

    def _actual_actions_from_observations(
        self,
        start_observation,
        observations: list[object],
    ) -> list[list[float]]:
        del start_observation
        actions: list[list[float]] = []
        for observation in observations:
            action = _observation_joint_action(observation)
            if action is not None:
                actions.append(action.astype(float).tolist())
        return actions
