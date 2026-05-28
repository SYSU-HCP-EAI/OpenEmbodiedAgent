from __future__ import annotations

from PhyAgentOS.runtime.policy.dummy_client import DummyPolicyClient
from PhyAgentOS.runtime.schemas import AdapterPlan, SessionSpec, SkillSpec, TargetSpec
from PhyAgentOS.runtime.sessions.session_runner import SessionRunner
from PhyAgentOS.runtime.skills.policy import OpenPISkillRuntime
from PhyAgentOS.runtime.targets.local.dummy_sim_target import DummySimTarget


def test_dummy_sim_rollout_succeeds() -> None:
    session = SessionSpec.model_validate(
        {
            "session_id": "sess_1",
            "target_ref": "target://dummy_sim",
            "skill_ref": "skill://openpi_sim_vla",
            "task_description": "move",
            "routing": {"policy_endpoint": "dummy://local"},
            "execution": {"max_steps": 10, "replan_every_steps": 4, "action_chunk_mode": "chunk_buffer"},
        }
    )
    target_spec = TargetSpec.model_validate(
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
            "config": {"image_size": 16, "state_dim": 8, "action_dim": 7, "success_after_steps": 3},
        }
    )
    skill_spec = SkillSpec.model_validate(
        {
            "id": "openpi_sim_vla",
            "runtime": "OpenPISkillRuntime",
            "runtime_kind": "policy",
            "loop_mode": "policy_closed_loop",
            "supported_target_kinds": ["simulation"],
            "policy": {"policy_client": "dummy", "policy_adapter": "policy_adapter://dummy_openpi_adapter"},
        }
    )
    target = DummySimTarget({"image_size": 16, "state_dim": 8, "action_dim": 7, "success_after_steps": 3})
    result = SessionRunner(
        session=session,
        target_spec=target_spec,
        skill_spec=skill_spec,
        adapter_plan=AdapterPlan(
            target_adapter="target_adapter://dummy_sim_adapter",
            policy_adapter="policy_adapter://dummy_openpi_adapter",
            action_bridges=["bridge://safety_clamp"],
        ),
        target=target,
        skill_runtime=OpenPISkillRuntime(),
        policy_client=DummyPolicyClient(action_dim=7, chunk_size=4),
        perception_runtime=None,
        perception_plan=None,
    ).start()

    assert result.success is True
    assert result.status == "succeeded"
    assert result.num_steps == 3
