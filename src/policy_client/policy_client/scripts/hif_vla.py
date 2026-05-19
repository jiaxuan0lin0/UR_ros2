"""HiF-VLA-specific HTTP client and 7D-to-8D observation adapter."""

from __future__ import annotations

from typing import Any, Dict, Optional, Sequence

import numpy as np
import requests

from policy_client.http_policy import JsonHttpTransport, PolicyClient
from policy_client.types import (
    ACTION_DIM,
    OBSERVATION_DIM,
    PolicyObservation,
    PolicyResult,
    action_list,
)


HIF_VLA_PROPRIO_DIM = 8


def hif_vla_proprio_from_base(
    proprio: Sequence[float],
    *,
    pad_value: float = 0.0,
) -> np.ndarray:
    """Expand base 7D proprio to HiF-VLA's 8D state convention.

    Base convention:
        TCP model: [x, y, z, rx, ry, rz, gripper]
        joint-abs model: [q0, q1, q2, q3, q4, q5, gripper]

    HiF-VLA convention:
        [six state values, pad, gripper]
    """

    base = np.asarray(proprio, dtype=np.float64)
    if base.shape != (OBSERVATION_DIM,):
        raise ValueError(
            f'base proprio must have shape ({OBSERVATION_DIM},), got '
            f'{base.shape}'
        )
    if not np.all(np.isfinite(base)):
        raise ValueError(f'base proprio contains non-finite values: {base}')

    hif = np.zeros(HIF_VLA_PROPRIO_DIM, dtype=np.float64)
    hif[:6] = base[:6]
    hif[6] = float(pad_value)
    hif[7] = base[6]
    return hif


def build_hif_vla_payload(
    *,
    instruction: str,
    observation: PolicyObservation,
    pad_value: float = 0.0,
    reset: bool = False,
    last_action: Optional[Sequence[float]] = None,
    executed_actions: Optional[Sequence[Sequence[float]]] = None,
    history_images: Optional[Sequence[np.ndarray]] = None,
) -> Dict[str, Any]:
    """Build the JSON payload expected by deploy/server.py."""

    observation.validate()
    payload: Dict[str, Any] = {
        'instruction': str(instruction),
        'full_image': image_to_json(observation.primary_image),
        'state': hif_vla_proprio_from_base(
            observation.proprio,
            pad_value=pad_value,
        ).tolist(),
    }
    if observation.wrist_image is not None:
        payload['wrist_image'] = image_to_json(observation.wrist_image)
    if history_images is not None:
        payload['history_full_images'] = [
            image_to_json(image) for image in history_images
        ]
    if reset:
        payload['reset'] = True
    if last_action is not None:
        payload['last_action'] = action_list(last_action)
    if executed_actions is not None:
        payload['executed_actions'] = [
            action_list(action) for action in executed_actions
        ]
    return payload


class HiFVLAHttpPolicyClient(PolicyClient):
    """Client for the FastAPI server in HiF-VLA deploy/server.py."""

    def __init__(
        self,
        base_url: str,
        timeout_sec: float = 30.0,
        pad_value: float = 0.0,
        session: Optional[requests.Session] = None,
    ):
        self.transport = JsonHttpTransport(
            base_url,
            timeout_sec=timeout_sec,
            session=session,
        )
        self.pad_value = float(pad_value)

    @property
    def base_url(self) -> str:
        return self.transport.base_url

    def health(self) -> Dict[str, Any]:
        return self.transport.health()

    def reset(self) -> Dict[str, Any]:
        return self.transport.reset()

    def act(
        self,
        *,
        instruction: str,
        observation: PolicyObservation,
        reset: bool = False,
        last_action: Optional[Sequence[float]] = None,
        executed_actions: Optional[Sequence[Sequence[float]]] = None,
        history_images: Optional[Sequence[np.ndarray]] = None,
    ) -> PolicyResult:
        payload = build_hif_vla_payload(
            instruction=instruction,
            observation=observation,
            pad_value=self.pad_value,
            reset=reset,
            last_action=last_action,
            executed_actions=executed_actions,
            history_images=history_images,
        )
        response = self.transport.post_json('/act', payload)
        return PolicyResult.from_response(response)

    def record_action(self, actions: Sequence[Sequence[float]]) -> Dict[str, Any]:
        payload = {'actions': [action_list(action) for action in actions]}
        return self.transport.post_json('/record_action', payload)


def image_to_json(image: np.ndarray) -> list:
    array = np.asarray(image)
    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8)
    return array.tolist()


def validate_hif_vla_action(action: Sequence[float]) -> list[float]:
    values = np.asarray(action, dtype=np.float64)
    if values.shape != (ACTION_DIM,):
        raise ValueError(f'action must have shape ({ACTION_DIM},), got {values}')
    if not np.all(np.isfinite(values)):
        raise ValueError(f'action contains non-finite values: {values}')
    return values.astype(float).tolist()


def create_policy_client(
    *,
    base_url: str,
    timeout_sec: float = 30.0,
    gripper_pad_value: float = 0.0,
    **_kwargs,
) -> HiFVLAHttpPolicyClient:
    return HiFVLAHttpPolicyClient(
        base_url,
        timeout_sec=timeout_sec,
        pad_value=gripper_pad_value,
    )
