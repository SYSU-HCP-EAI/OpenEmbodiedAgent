"""Base adapter for OpenPI-style policy payloads."""

from __future__ import annotations

from abc import abstractmethod
from typing import Any

import numpy as np

from PhyAgentOS.runtime.adapters.base import BaseTargetAdapter
from PhyAgentOS.runtime.watchdog.errors import AdapterError


class BaseOpenPIAdapter(BaseTargetAdapter):
    def on_reset(self, raw_obs: dict[str, Any], target_info: dict[str, Any]) -> dict[str, Any]:
        return self.make_observation(raw_obs, step_idx=0, target_info=target_info)

    @abstractmethod
    def make_observation(
        self,
        raw_obs: dict[str, Any],
        step_idx: int,
        target_info: dict[str, Any],
    ) -> dict[str, Any]:
        """Convert raw target observation to an OpenPI inference dict."""

    def decode_action_chunk(
        self,
        action_payload: dict[str, Any],
        target_info: dict[str, Any],
    ) -> list[np.ndarray]:
        if "actions" not in action_payload:
            raise AdapterError("policy payload missing `actions`")
        actions = np.asarray(action_payload["actions"], dtype=np.float32)
        if actions.ndim == 1:
            actions = actions[None, :]
        if actions.ndim != 2:
            raise AdapterError(f"`actions` must have shape [A] or [T,A], got {actions.shape}")

        action_dim = target_info.get("action_dim")
        if action_dim is None and isinstance(target_info.get("action"), dict):
            action_dim = target_info["action"].get("action_dim")
        if action_dim is not None:
            actions = actions[:, : int(action_dim)]
        return [actions[i] for i in range(actions.shape[0])]

    def should_replan(
        self,
        step_idx: int,
        action_buffer_size: int,
        target_info: dict[str, Any],
    ) -> bool:
        return action_buffer_size == 0
