"""Deterministic policy client used by tests and dummy simulation."""

from __future__ import annotations

from typing import Any

import numpy as np

from PhyAgentOS.runtime.policy.base_client import BasePolicyClient


class DummyPolicyClient(BasePolicyClient):
    def __init__(self, action_dim: int = 7, chunk_size: int = 4):
        self.action_dim = int(action_dim)
        self.chunk_size = int(chunk_size)

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        return {
            "actions": np.zeros((self.chunk_size, self.action_dim), dtype=np.float32),
            "policy_meta": {
                "backend": "dummy",
                "chunk_size": self.chunk_size,
                "policy_latency_ms": 0.0,
            },
        }
