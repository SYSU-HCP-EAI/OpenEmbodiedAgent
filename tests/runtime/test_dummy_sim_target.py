from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from PhyAgentOS.runtime.targets.base import BaseRolloutTarget
from PhyAgentOS.runtime.targets.local.dummy_sim_target import DummySimTarget


class IncompleteTarget(BaseRolloutTarget):
    def build(self) -> None:
        pass

    def describe(self) -> dict[str, Any]:
        return {}

    def configure_session(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        return {}

    def start_session(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        return {}

    def reset(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        return {}

    def observe(self) -> dict[str, Any]:
        return {}

    def action_chunk(self, executable_action_chunk: dict[str, Any]) -> dict[str, Any]:
        return {}

    def execution_status(self) -> dict[str, Any]:
        return {}

    def cancel(self, reason: str) -> None:
        pass


def test_base_rollout_target_requires_full_contract() -> None:
    with pytest.raises(TypeError):
        IncompleteTarget()


def test_dummy_sim_reset_observe_and_success() -> None:
    target = DummySimTarget({"image_size": 16, "state_dim": 5, "action_dim": 7, "success_after_steps": 2})
    assert isinstance(target, BaseRolloutTarget)

    target.build()

    obs = target.reset({})
    assert obs["image"].shape == (16, 16, 3)
    assert obs["state"].shape == (5,)

    first = target.action_chunk({"actions": np.zeros((1, 7), dtype=np.float32)})
    second = target.action_chunk({"actions": np.zeros((1, 7), dtype=np.float32)})

    assert first["done"] is False
    assert second["done"] is True
    assert second["success"] is True
