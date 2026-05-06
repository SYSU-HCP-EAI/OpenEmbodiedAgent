from __future__ import annotations

import pytest

from PhyAgentOS.runtime.policy.dummy_client import DummyPolicyClient
from PhyAgentOS.runtime.policy.factory import build_policy_client, parse_policy_endpoint
from PhyAgentOS.runtime.policy.openpi_client import OpenPIClientPolicyWrapper
from PhyAgentOS.runtime.watchdog.errors import PolicyConnectionError


def test_dummy_policy_client_action_shape() -> None:
    client = DummyPolicyClient(action_dim=7, chunk_size=4)
    payload = client.infer({"prompt": "move"})

    assert payload["actions"].shape == (4, 7)
    assert payload["actions"].dtype.name == "float32"


def test_parse_policy_endpoint() -> None:
    assert parse_policy_endpoint("dummy://local") == ("dummy", "local", 0)
    assert parse_policy_endpoint("openpi://127.0.0.1:8000") == ("openpi", "127.0.0.1", 8000)


def test_build_dummy_client() -> None:
    client = build_policy_client("dummy://local", action_dim=3, chunk_size=2)
    assert client.infer({})["actions"].shape == (2, 3)


def test_openpi_wrapper_lazy_import_error() -> None:
    with pytest.raises(PolicyConnectionError, match="openpi-client is not installed"):
        OpenPIClientPolicyWrapper(host="127.0.0.1", port=8000)
