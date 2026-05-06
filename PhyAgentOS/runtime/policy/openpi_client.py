"""Lazy OpenPI remote policy client wrapper."""

from __future__ import annotations

import time
from typing import Any

from PhyAgentOS.runtime.policy.base_client import BasePolicyClient
from PhyAgentOS.runtime.watchdog.errors import PolicyConnectionError, PolicyProtocolError


class OpenPIClientPolicyWrapper(BasePolicyClient):
    """Wrap OpenPI's official websocket client without importing it at module load."""

    def __init__(self, host: str, port: int, timeout_s: float = 5.0):
        try:
            from openpi_client import websocket_client_policy
        except ImportError as exc:
            raise PolicyConnectionError(
                "openpi-client is not installed. Install it from "
                "$OPENPI_ROOT/packages/openpi-client with `pip install -e .`."
            ) from exc

        self.client = websocket_client_policy.WebsocketClientPolicy(host=host, port=port)
        self.timeout_s = float(timeout_s)

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        started = time.perf_counter()
        out = self.client.infer(observation)
        latency_ms = (time.perf_counter() - started) * 1000
        if not isinstance(out, dict) or "actions" not in out:
            raise PolicyProtocolError("OpenPI response missing `actions`.")
        out.setdefault("policy_meta", {})
        out["policy_meta"]["policy_latency_ms"] = latency_ms
        return out
