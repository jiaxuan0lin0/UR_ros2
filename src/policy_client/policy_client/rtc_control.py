"""Deployment-side RTC helpers for action chunk transitions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from policy_client.types import ACTION_DIM


@dataclass
class RTCControlConfig:
    enabled: bool = False
    execution_horizon: int = 10
    schedule: str = "exp"
    exp_decay: float = 4.0


@dataclass
class RTCControlInfo:
    enabled: bool
    prefix_count: int = 0
    previous_count: int = 0
    action_count: int = 0
    schedule: str = "none"
    inference_delay_steps: int = 0


class RTCControl:
    """Keep successive action chunks continuous at execution time.

    This is the robot-side part of RTC: async inference removes waiting, and this
    transition step makes the start of a new chunk agree with the tail of the
    previous executed chunk. Model-side prefix guidance still belongs in the
    Pi0.5/flow-matching policy server.
    """

    def __init__(self, config: RTCControlConfig):
        self.config = config
        self._previous_tail: np.ndarray | None = None
        self._previous_count = 0
        self._weight_cache: dict[tuple[int, str, float], np.ndarray] = {}

    @property
    def enabled(self) -> bool:
        return bool(self.config.enabled)

    def reset(self) -> None:
        self._previous_tail = None
        self._previous_count = 0

    def align(
        self,
        actions: Iterable[Sequence[float]],
        *,
        action_period_sec: float,
        inference_latency_sec: float | None = None,
    ) -> tuple[list[np.ndarray], RTCControlInfo]:
        chunk = _as_action_chunk(actions, copy=False)
        delay_steps = _latency_steps(inference_latency_sec, action_period_sec)
        info = RTCControlInfo(
            enabled=self.enabled,
            previous_count=self._previous_count,
            action_count=len(chunk),
            schedule="disabled" if not self.enabled else self.config.schedule,
            inference_delay_steps=delay_steps,
        )
        if not self.enabled or self._previous_tail is None or len(chunk) == 0:
            return _chunk_to_list(chunk), info

        prefix_count = min(
            max(int(self.config.execution_horizon), 0),
            len(self._previous_tail),
            len(chunk),
        )
        info.prefix_count = prefix_count
        if prefix_count <= 0:
            return _chunk_to_list(chunk), info

        weights = self._transition_weights(prefix_count)
        aligned = chunk.copy()
        previous_tail = self._previous_tail[-prefix_count:]
        aligned[:prefix_count] = (
            weights[:, None] * previous_tail
            + (1.0 - weights[:, None]) * aligned[:prefix_count]
        )
        return _chunk_to_list(aligned), info

    def commit(self, actions: Iterable[Sequence[float]]) -> None:
        chunk = _as_action_chunk(actions, copy=True)
        self._previous_count = len(chunk)
        horizon = max(int(self.config.execution_horizon), 0)
        if len(chunk) == 0 or horizon <= 0:
            self._previous_tail = None
            return
        self._previous_tail = chunk[-horizon:]

    def _transition_weights(self, count: int) -> np.ndarray:
        key = (
            int(count),
            str(self.config.schedule).strip().lower(),
            float(self.config.exp_decay),
        )
        weights = self._weight_cache.get(key)
        if weights is None:
            weights = _transition_weights(
                count,
                schedule=self.config.schedule,
                exp_decay=self.config.exp_decay,
            )
            self._weight_cache[key] = weights
        return weights


def _as_action_chunk(actions: Iterable[Sequence[float]], *, copy: bool) -> np.ndarray:
    values = np.asarray(actions, dtype=np.float64)
    if values.size == 0:
        return np.zeros((0, ACTION_DIM), dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != ACTION_DIM:
        raise ValueError(
            f"action chunk must have shape (N, {ACTION_DIM}), got {values.shape}"
        )
    if not np.all(np.isfinite(values)):
        raise ValueError("action chunk contains non-finite values")
    if copy:
        return np.ascontiguousarray(values).copy()
    return np.ascontiguousarray(values)


def _chunk_to_list(chunk: np.ndarray) -> list[np.ndarray]:
    return [chunk[index] for index in range(len(chunk))]


def _latency_steps(latency_sec: float | None, action_period_sec: float) -> int:
    if latency_sec is None:
        return 0
    try:
        latency = float(latency_sec)
        period = float(action_period_sec)
    except (TypeError, ValueError):
        return 0
    if latency <= 0.0 or period <= 0.0 or not np.isfinite(latency):
        return 0
    return int(np.ceil(latency / period))


def _transition_weights(count: int, *, schedule: str, exp_decay: float) -> np.ndarray:
    if count <= 0:
        return np.zeros(0, dtype=np.float64)
    normalized = str(schedule).strip().lower()
    if normalized in {"zero", "zeros", "none", "off"}:
        return np.zeros(count, dtype=np.float64)
    if normalized in {"one", "ones", "freeze"}:
        return np.ones(count, dtype=np.float64)
    if count == 1:
        return np.ones(1, dtype=np.float64)

    t = np.linspace(0.0, 1.0, count, dtype=np.float64)
    if normalized in {"linear", "lin"}:
        return 1.0 - t
    if normalized in {"exp", "exponential"}:
        decay = max(float(exp_decay), 0.0)
        return np.exp(-decay * t)
    raise ValueError(
        'rtc_transition_schedule must be "exp", "linear", "ones", or "zeros", '
        f"got {schedule!r}"
    )
