"""Small data objects shared by policy clients, adapters, and nodes."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np


ACTION_DIM = 7
OBSERVATION_DIM = 7
PROPRIO_DIM = OBSERVATION_DIM


@dataclass
class PolicyObservation:
    """One policy observation in the base 7D real-robot convention."""

    primary_image: np.ndarray
    proprio: np.ndarray
    wrist_image: Optional[np.ndarray] = None
    camera_serials: Tuple[str, ...] = ()
    robot_state: Dict[str, Any] = field(default_factory=dict)

    def validate(self) -> None:
        _validate_image(self.primary_image, 'primary_image')
        if self.wrist_image is not None:
            _validate_image(self.wrist_image, 'wrist_image')
        _validate_vector(self.proprio, OBSERVATION_DIM, 'proprio')


@dataclass
class PolicyResult:
    """A 7D policy action plus the optional action chunk returned by a model."""

    action: np.ndarray
    action_chunk: np.ndarray
    history: Dict[str, Any] = field(default_factory=dict)
    raw_response: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_response(cls, response: Dict[str, Any]) -> 'PolicyResult':
        action = _as_action(response.get('action'), 'action')
        action_chunk_value = response.get('action_chunk')
        if action_chunk_value is None:
            action_chunk = action[None, :]
        else:
            action_chunk = np.asarray(action_chunk_value, dtype=np.float64)
            if action_chunk.ndim != 2 or action_chunk.shape[1] != ACTION_DIM:
                raise ValueError(
                    'action_chunk must have shape (N, 7), got '
                    f'{action_chunk.shape}'
                )
        return cls(
            action=action,
            action_chunk=action_chunk,
            history=dict(response.get('history', {})),
            raw_response=response,
        )


def _validate_image(image: np.ndarray, field_name: str) -> None:
    if not isinstance(image, np.ndarray):
        raise TypeError(f'{field_name} must be a numpy array')
    if image.ndim != 3 or image.shape[-1] != 3:
        raise ValueError(
            f'{field_name} must have shape (H, W, 3), got {image.shape}'
        )


def _validate_vector(
    vector: np.ndarray,
    length: int,
    field_name: str,
) -> None:
    array = np.asarray(vector, dtype=np.float64)
    if array.shape != (length,):
        raise ValueError(
            f'{field_name} must have shape ({length},), got {array.shape}'
        )
    if not np.all(np.isfinite(array)):
        raise ValueError(f'{field_name} contains non-finite values')


def _as_action(value: Any, field_name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (ACTION_DIM,):
        raise ValueError(
            f'{field_name} must have shape ({ACTION_DIM},), got {array.shape}'
        )
    if not np.all(np.isfinite(array)):
        raise ValueError(f'{field_name} contains non-finite values')
    return array


def action_list(action: Sequence[float]) -> list[float]:
    """Return a JSON-friendly validated 7D action list."""

    return _as_action(action, 'action').astype(float).tolist()
