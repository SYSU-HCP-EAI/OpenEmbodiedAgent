from __future__ import annotations

from PhyAgentOS.runtime.adapters.openpi.dummy_openpi_adapter import DummyOpenPIAdapter
from PhyAgentOS.runtime.policy.dummy_client import DummyPolicyClient
from PhyAgentOS.runtime.schemas import SessionSpec
from PhyAgentOS.runtime.skills.vla.openpi_sim_runtime import OpenPISimSkillRuntime
from PhyAgentOS.runtime.targets.sim.dummy_sim_target import DummySimTarget


def test_dummy_sim_rollout_succeeds() -> None:
    session = SessionSpec.model_validate(
        {
            "session_id": "sess_1",
            "target_ref": "target://dummy_sim",
            "skill_ref": "skill://openpi_sim_vla",
            "task_description": "move",
            "routing": {"policy_endpoint": "dummy://local", "adapter": "dummy_openpi_adapter"},
            "execution": {"max_steps": 10, "replan_every": 4, "action_chunk_mode": "open_loop"},
        }
    )
    target = DummySimTarget({"image_size": 16, "state_dim": 8, "action_dim": 7, "success_after_steps": 3})
    result = OpenPISimSkillRuntime().run(
        session,
        target,
        DummyOpenPIAdapter(),
        DummyPolicyClient(action_dim=7, chunk_size=4),
    )

    assert result.success is True
    assert result.status == "succeeded"
    assert result.num_steps == 3
