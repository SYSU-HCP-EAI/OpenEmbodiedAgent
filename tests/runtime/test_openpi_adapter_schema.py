from __future__ import annotations

import numpy as np
import pytest

from PhyAgentOS.runtime.adapters.openpi.dummy_openpi_adapter import DummyOpenPIAdapter
from PhyAgentOS.runtime.adapters.target_dummy import DummySimTargetAdapter
from PhyAgentOS.runtime.watchdog.errors import AdapterError


def test_dummy_adapter_observation_contract() -> None:
    target_adapter = DummySimTargetAdapter()
    policy_adapter = DummyOpenPIAdapter()
    runtime_observation = target_adapter.to_runtime_observation(
        {
            "image": np.zeros((224, 224, 3), dtype=np.uint8),
            "wrist_image": np.zeros((224, 224, 3), dtype=np.uint8),
            "state": np.zeros((8,), dtype=np.float64),
        },
        {"step_index": 0},
    )
    obs = policy_adapter.to_policy_input(runtime_observation, {"task_description": "move"})

    assert set(obs) == {
        "observation/image",
        "observation/wrist_image",
        "observation/state",
        "prompt",
    }
    assert obs["observation/image"].dtype == np.uint8
    assert obs["observation/state"].dtype == np.float32
    assert obs["prompt"] == "move"


def test_dummy_adapter_missing_key() -> None:
    adapter = DummyOpenPIAdapter()
    with pytest.raises(AdapterError):
        adapter.to_policy_input({"sensors": {}}, {"task_description": "move"})


def test_from_policy_output_vector_and_matrix() -> None:
    adapter = DummyOpenPIAdapter()

    one = adapter.from_policy_output({"actions": np.zeros((7,), dtype=np.float32)}, {"action_dim": 7})
    many = adapter.from_policy_output({"actions": np.zeros((4, 7), dtype=np.float32)}, {"action_dim": 7})

    assert one["actions"].shape == (1, 7)
    assert many["actions"].shape == (4, 7)


def test_from_policy_output_rejects_implicit_truncation() -> None:
    adapter = DummyOpenPIAdapter()
    with pytest.raises(AdapterError):
        adapter.from_policy_output({"actions": np.zeros((4, 8), dtype=np.float32)}, {"action_dim": 7})
