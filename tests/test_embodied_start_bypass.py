"""Tests for EmbodiedActionTool HAL warm-up critic bypass."""

from __future__ import annotations

from PhyAgentOS.agent.tools.embodied import EmbodiedActionTool


def test_start_params_deferred_empty() -> None:
    assert EmbodiedActionTool._start_params_deferred_to_driver({}) is True
    assert EmbodiedActionTool._start_params_deferred_to_driver(None) is True


def test_start_params_deferred_optional_flags_only() -> None:
    assert EmbodiedActionTool._start_params_deferred_to_driver({"skip_room_bootstrap": True}) is True
    assert EmbodiedActionTool._start_params_deferred_to_driver({"robot_id": "sim_001"}) is True


def test_start_params_not_deferred_when_overrides() -> None:
    assert EmbodiedActionTool._start_params_deferred_to_driver({"scene_asset_path": "/x.usd"}) is False
    assert EmbodiedActionTool._start_params_deferred_to_driver({"robot_start": [0, 0, 0]}) is False
    assert EmbodiedActionTool._start_params_deferred_to_driver({"api_kwargs": {"force_gui": True}}) is False
