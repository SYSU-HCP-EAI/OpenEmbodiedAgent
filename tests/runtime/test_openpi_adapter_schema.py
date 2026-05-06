from __future__ import annotations

import numpy as np
import pytest

from PhyAgentOS.runtime.adapters.openpi.dummy_openpi_adapter import DummyOpenPIAdapter
from PhyAgentOS.runtime.watchdog.errors import AdapterError


def test_dummy_adapter_observation_contract() -> None:
    adapter = DummyOpenPIAdapter()
    obs = adapter.make_observation(
        {
            "image": np.zeros((224, 224, 3), dtype=np.uint8),
            "wrist_image": np.zeros((224, 224, 3), dtype=np.uint8),
            "state": np.zeros((8,), dtype=np.float64),
        },
        step_idx=0,
        target_info={"task_description": "move"},
    )

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
        adapter.make_observation({}, step_idx=0, target_info={"task_description": "move"})


def test_decode_action_chunk_vector_and_matrix() -> None:
    adapter = DummyOpenPIAdapter()

    one = adapter.decode_action_chunk({"actions": np.zeros((7,), dtype=np.float32)}, {"action_dim": 7})
    many = adapter.decode_action_chunk({"actions": np.zeros((4, 8), dtype=np.float32)}, {"action_dim": 7})

    assert len(one) == 1
    assert one[0].shape == (7,)
    assert len(many) == 4
    assert many[0].shape == (7,)
