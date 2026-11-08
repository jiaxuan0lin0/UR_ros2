"""Background prefetch of the next policy result during chunk execution."""

from __future__ import annotations

import time
from typing import Optional

import numpy as np


class AsyncPolicyMixin:
    """Async policy requests; state lives in PolicyControlNode.__init__."""

    def _start_async_policy_request(
        self,
        *,
        start_observation,
        execution_observations: list[object],
    ) -> bool:
        if not self.async_policy or self._async_policy_executor is None:
            return False
        if not execution_observations:
            return False

        request = self._build_async_policy_request(
            start_observation=start_observation,
            execution_observations=execution_observations,
        )
        if request is None:
            return False

        with self._async_policy_lock:
            if (
                self._async_policy_future is not None
                and not self._async_policy_future.done()
            ):
                return False
            generation = self._async_policy_generation
            future = self._async_policy_executor.submit(
                self._run_async_policy_request,
                generation,
                request,
            )
            self._async_policy_future = future
        return True

    def _build_async_policy_request(
        self,
        *,
        start_observation,
        execution_observations: list[object],
    ) -> Optional[dict]:
        observations = [obs for obs in execution_observations if obs is not None]
        if not observations:
            return None

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

        return {
            'observation': self._clone_policy_observation(observations[-1]),
            'history_actions': actions or None,
            'history_images': images or None,
        }

    def _run_async_policy_request(self, generation: int, request: dict) -> dict:
        started_at = time.monotonic()
        result = self._call_policy(
            observation=request['observation'],
            history_actions=request['history_actions'],
            history_images=request['history_images'],
        )
        completed_at = time.monotonic()
        return {
            'generation': generation,
            'observation': request['observation'],
            'result': result,
            'started_at': started_at,
            'completed_at': completed_at,
            'latency_sec': completed_at - started_at,
        }

    def _consume_async_policy_result(self) -> Optional[dict]:
        if not self.async_policy:
            return None
        with self._async_policy_lock:
            future = self._async_policy_future
            generation = self._async_policy_generation
        if future is None:
            return None

        started_wait = time.monotonic()
        try:
            payload = future.result()
        except Exception:
            with self._async_policy_lock:
                if self._async_policy_future is future:
                    self._async_policy_future = None
            raise
        finally:
            waited = time.monotonic() - started_wait
            if waited >= self.async_policy_log_wait_sec:
                self.get_logger().info(
                    f'waited {waited:.3f}s for async policy result'
                )

        with self._async_policy_lock:
            if self._async_policy_future is future:
                self._async_policy_future = None
        if payload.get('generation') != generation:
            return None

        with self._state_lock:
            self._pending_executed_actions = None
            self._pending_history_images = None
        return payload

    def _clear_async_policy_request(self, *, increment_generation: bool) -> None:
        with self._async_policy_lock:
            if increment_generation:
                self._async_policy_generation += 1
            future = self._async_policy_future
            self._async_policy_future = None
        if future is not None and not future.done():
            future.cancel()

    @staticmethod
    def _clone_policy_observation(observation):
        from policy_client.types import PolicyObservation

        return PolicyObservation(
            primary_image=np.asarray(observation.primary_image).copy(),
            proprio=np.asarray(observation.proprio, dtype=np.float64).copy(),
            wrist_image=(
                None
                if getattr(observation, 'wrist_image', None) is None
                else np.asarray(observation.wrist_image).copy()
            ),
            camera_serials=tuple(getattr(observation, 'camera_serials', ())),
            robot_state=dict(getattr(observation, 'robot_state', {}) or {}),
        )
