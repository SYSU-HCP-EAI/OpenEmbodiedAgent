from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from PhyAgentOS.runtime.communication import RuntimeEnvelope, decode_msgpack, encode_msgpack
from PhyAgentOS.runtime.adapters.factory import (
    build_action_bridge,
    build_policy_adapter,
    build_target_adapter,
    register_action_bridge,
    register_policy_adapter,
    register_target_adapter,
)
from PhyAgentOS.runtime.preflight import RuntimeCompatibilityPreflight
from PhyAgentOS.runtime.schemas import (
    AdapterPlan,
    SessionSpec,
    SkillSpec,
    TargetRuntimeContractDocument,
    TargetSpec,
)
from PhyAgentOS.runtime.watchdog.scheduler import ScheduledSession
from PhyAgentOS.runtime.targets.remote.proxy import RemoteTargetProxy
from PhyAgentOS.runtime.watchdog.runtime_registry import TargetRuntimeRegistry, register_remote_target


def _write_contract(path: Path, *, shape=None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(
            {
                "version": "runtime_target_contract_v1",
                "target_id": "dummy_sim",
                "target_adapter": "target_adapter://dummy_sim_adapter",
                "action_contract": {
                    "id": "dummy_delta_eef_gripper_v1",
                    "accepted_representations": ["delta_eef_pose_gripper"],
                    "shape": shape or ["T", 7],
                    "dtype": "float32",
                    "normalized": False,
                    "frame": "base",
                    "control_mode": "cartesian_delta_position",
                    "control_hz": 20,
                    "components": [],
                    "chunk": {"max_chunk_size": 4, "switch_policy": "hard_switch"},
                },
                "safety": {"require_target_side_validation": True, "stop_on_nan": True, "stop_on_timeout": True},
                "capabilities": {"available_bridges": ["bridge://safety_clamp"]},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def _scheduled(tmp_path: Path, *, skill_shape=None) -> ScheduledSession:
    _write_contract(tmp_path / "configs/runtime/contracts/dummy_sim.runtime.yaml")
    target = TargetSpec.model_validate(
            {
                "id": "dummy_sim",
                "target_class": "local",
                "target_kind": "simulation",
                "workspace": "workspaces/dummy_sim",
                "supported_skills": ["openpi_sim_vla"],
                "runtime": {
                    "target_runtime": "DummySimTargetRuntime",
                    "target_endpoint": None,
                    "target_adapter": "target_adapter://dummy_sim_adapter",
                    "runtime_contract_ref": "configs/runtime/contracts/dummy_sim.runtime.yaml",
                },
        }
    )
    skill = SkillSpec.model_validate(
            {
                "id": "openpi_sim_vla",
                "runtime": "OpenPISkillRuntime",
                "runtime_kind": "policy",
                "loop_mode": "policy_closed_loop",
                "supported_target_kinds": ["simulation"],
                "policy": {
                    "policy_client": "dummy",
                    "policy_adapter": "policy_adapter://dummy_openpi_adapter",
                },
                "output_contract": {
                "action": {
                    "shape": skill_shape or ["T", 7],
                    "dtype": "float32",
                    "normalized": False,
                    "representation": "delta_eef_pose_gripper",
                    "frame": "base",
                }
            },
            "adapter_requirements": {"allowed_bridges": ["bridge://safety_clamp"]},
        }
    )
    session = SessionSpec.model_validate(
        {
            "session_id": "sess_1",
            "target_ref": "target://dummy_sim",
            "skill_ref": "skill://openpi_sim_vla",
            "task_description": "move",
            "routing": {"policy_endpoint": "dummy://local"},
        }
    )
    return ScheduledSession(session, target, skill, target.id, skill.id)


def test_runtime_envelope_msgpack_round_trip() -> None:
    envelope = RuntimeEnvelope(
        type="target.observe",
        session_id="sess_1",
        target_id="dummy_sim",
        skill_id="openpi_sim_vla",
        seq=1,
        timestamp_ns=123,
        payload={"required_sensors": ["front_rgb"]},
    )

    decoded = decode_msgpack(encode_msgpack(envelope))

    assert decoded == envelope


def test_runtime_envelope_rejects_unknown_type() -> None:
    with pytest.raises(ValidationError):
        RuntimeEnvelope(type="target.step", seq=1, timestamp_ns=123, payload={})


def test_targetws_endpoint_builds_remote_target_proxy(tmp_path: Path) -> None:
    scheduled = _scheduled(tmp_path)
    scheduled.target_spec.target_class = "remote"
    scheduled.target_spec.runtime.target_runtime = "RemoteTargetProxy"
    scheduled.target_spec.runtime.target_endpoint = "targetws://127.0.0.1:9001"
    target = TargetRuntimeRegistry().build(scheduled.target_spec)

    assert isinstance(target, RemoteTargetProxy)
    assert target.client.endpoint == "targetws://127.0.0.1:9001"


def test_remote_target_uses_registered_remote_runtime(tmp_path: Path) -> None:
    class TestFrankaRemoteTarget(RemoteTargetProxy):
        pass

    register_remote_target(
        "FrankaTargetRuntime",
        lambda target_spec, client: TestFrankaRemoteTarget(client, config=target_spec.config),
    )
    scheduled = _scheduled(tmp_path)
    scheduled.target_spec.target_class = "remote"
    scheduled.target_spec.target_kind = "real_robot"
    scheduled.target_spec.runtime.target_runtime = "FrankaTargetRuntime"
    scheduled.target_spec.runtime.target_endpoint = "targetws://192.168.10.31:9001"

    target = TargetRuntimeRegistry().build(scheduled.target_spec)

    assert isinstance(target, TestFrankaRemoteTarget)


def test_runtime_contract_and_adapter_plan_schema() -> None:
    contract = TargetRuntimeContractDocument.model_validate(
        yaml.safe_load(
            """
version: runtime_target_contract_v1
target_id: dummy_sim
target_adapter: target_adapter://dummy_sim_adapter
action_contract:
  id: dummy_delta_eef_gripper_v1
  accepted_representations: [delta_eef_pose_gripper]
  shape: [T, 7]
  dtype: float32
  normalized: false
  frame: base
  control_mode: cartesian_delta_position
  control_hz: 20
  components: []
safety:
  require_target_side_validation: true
  stop_on_nan: true
  stop_on_timeout: true
"""
        )
    )
    plan = AdapterPlan(
        target_adapter=contract.target_adapter,
        policy_adapter="policy_adapter://dummy_openpi_adapter",
        action_bridges=["bridge://safety_clamp"],
    )

    assert plan.validation_mode == "strict"
    assert contract.action_contract.shape == ["T", 7]


def test_adapter_registries_accept_runtime_extensions() -> None:
    class TestTargetAdapter:
        pass

    class TestPolicyAdapter:
        pass

    class TestBridge:
        pass

    register_target_adapter("target_adapter://test_target_adapter", TestTargetAdapter)
    register_policy_adapter("policy_adapter://test_policy_adapter", TestPolicyAdapter)
    register_action_bridge("bridge://test_bridge", TestBridge)

    assert isinstance(build_target_adapter("target_adapter://test_target_adapter"), TestTargetAdapter)
    assert isinstance(build_policy_adapter("policy_adapter://test_policy_adapter"), TestPolicyAdapter)
    assert isinstance(build_action_bridge("bridge://test_bridge"), TestBridge)


def test_preflight_rejects_implicit_action_shape_truncation(tmp_path: Path) -> None:
    result = RuntimeCompatibilityPreflight(tmp_path).check(_scheduled(tmp_path, skill_shape=["T", 8]))

    assert result.verdict == "rejected"
    assert any(item.code == "ACTION_CONTRACT_UNSATISFIED" for item in result.missing_items)
