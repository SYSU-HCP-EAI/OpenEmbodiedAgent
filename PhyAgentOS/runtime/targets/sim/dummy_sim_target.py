"""Dependency-free simulation target used for runtime smoke tests."""

from __future__ import annotations

from typing import Any

import numpy as np

from PhyAgentOS.runtime.targets.sim.base_sim_target import BaseSimTarget
from PhyAgentOS.runtime.watchdog.errors import TargetStepError


class DummySimTarget(BaseSimTarget):
    def __init__(self, config: dict[str, Any] | None = None):
        self.config = dict(config or {})
        self.step_idx = 0
        self._built = False

    def build(self) -> None:
        self._built = True

    def reset(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        self.step_idx = 0
        return self.observe()

    def observe(self) -> dict[str, Any]:
        image_size = self._config_int("image_size", default=self._nested_int("observation", "image_size", 224))
        state_dim = self._config_int("state_dim", default=self._nested_int("observation", "state_dim", 8))
        return {
            "image": np.zeros((image_size, image_size, 3), dtype=np.uint8),
            "wrist_image": np.zeros((image_size, image_size, 3), dtype=np.uint8),
            "state": np.zeros((state_dim,), dtype=np.float32),
        }

    def step(self, action: np.ndarray) -> dict[str, Any]:
        action_array = np.asarray(action, dtype=np.float32)
        action_dim = self._config_int("action_dim", default=self._nested_int("action", "action_dim", 7))
        if action_array.ndim != 1 or action_array.shape[0] < action_dim:
            raise TargetStepError(f"expected action vector with at least {action_dim} dims, got {action_array.shape}")
        self.step_idx += 1
        done = self.step_idx >= self._config_int("success_after_steps", default=5)
        return {
            "obs": self.observe(),
            "reward": float(done),
            "done": done,
            "info": {"success": done, "step_idx": self.step_idx},
        }

    def close(self) -> None:
        self._built = False

    def _nested_int(self, section: str, key: str, default: int) -> int:
        value = self.config.get(section)
        if isinstance(value, dict) and key in value:
            return int(value[key])
        return int(default)

    def _config_int(self, key: str, default: int) -> int:
        return int(self.config.get(key, default))
