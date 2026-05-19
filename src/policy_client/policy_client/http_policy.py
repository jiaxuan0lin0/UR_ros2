"""Generic HTTP utilities and policy client interfaces."""

from __future__ import annotations

from abc import ABC, abstractmethod
from urllib.parse import urlsplit
from typing import Any, Dict, Optional, Sequence

import requests

from policy_client.types import PolicyObservation, PolicyResult


class PolicyClient(ABC):
    """Abstract policy service client.

    New model deployments should implement this interface and keep the rest of
    the local controller stack unchanged.
    """

    @abstractmethod
    def health(self) -> Dict[str, Any]:
        """Return remote service health details."""

    @abstractmethod
    def reset(self) -> Dict[str, Any]:
        """Reset policy-side episode/history state."""

    @abstractmethod
    def act(
        self,
        *,
        instruction: str,
        observation: PolicyObservation,
        reset: bool = False,
        last_action: Optional[Sequence[float]] = None,
        executed_actions: Optional[Sequence[Sequence[float]]] = None,
        history_images: Optional[Sequence[Any]] = None,
    ) -> PolicyResult:
        """Run one policy inference call."""

    @abstractmethod
    def record_action(self, actions: Sequence[Sequence[float]]) -> Dict[str, Any]:
        """Record one or more executed policy-convention actions."""


class JsonHttpTransport:
    """Small JSON transport shared by concrete policy clients."""

    def __init__(
        self,
        base_url: str,
        timeout_sec: float = 30.0,
        session: Optional[requests.Session] = None,
    ):
        self.base_url = self._normalize_base_url(base_url)
        self.timeout_sec = float(timeout_sec)
        self.session = session or requests.Session()
        if session is None:
            # Robot policy calls should go straight to the configured endpoint.
            # Developer shells often export HTTP_PROXY/HTTPS_PROXY, which can
            # turn local SSH tunnels or lab-network URLs into confusing
            # RemoteDisconnected failures.
            self.session.trust_env = False

    def health(self) -> Dict[str, Any]:
        return self.get_json('/health')

    def reset(self) -> Dict[str, Any]:
        return self.post_json('/reset')

    def get_json(self, path: str) -> Dict[str, Any]:
        response = self.session.get(
            self._url(path),
            timeout=self.timeout_sec,
        )
        return self._json_or_raise(response)

    def post_json(
        self,
        path: str,
        payload: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        response = self.session.post(
            self._url(path),
            json=payload,
            timeout=self.timeout_sec,
        )
        return self._json_or_raise(response)

    def _url(self, path: str) -> str:
        return f'{self.base_url}{path}'

    @staticmethod
    def _normalize_base_url(base_url: str) -> str:
        value = str(base_url).strip()
        if not value:
            raise ValueError('policy base_url must be non-empty')
        parsed = urlsplit(value)
        if not parsed.scheme:
            value = f'http://{value}'
            parsed = urlsplit(value)
        if parsed.scheme not in {'http', 'https'}:
            raise ValueError(
                f'policy base_url must use http or https, got {parsed.scheme!r}'
            )
        if not parsed.netloc:
            raise ValueError(f'policy base_url has no host: {base_url!r}')
        return value.rstrip('/')

    @staticmethod
    def _json_or_raise(response: requests.Response) -> Dict[str, Any]:
        try:
            data = response.json()
        except ValueError as exc:
            response.raise_for_status()
            raise RuntimeError('policy server returned non-JSON response') from exc

        if not response.ok:
            error = data.get('error', data)
            raise RuntimeError(
                f'policy server returned HTTP {response.status_code}: {error}'
            )
        if not isinstance(data, dict):
            raise RuntimeError('policy server JSON response must be an object')
        return data
