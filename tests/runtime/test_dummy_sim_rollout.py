from __future__ import annotations

from PhyAgentOS.runtime.adapters.openpi.dummy_openpi_adapter import DummyOpenPIAdapter
from PhyAgentOS.runtime.adapters.target_dummy import DummySimTargetAdapter
from PhyAgentOS.runtime.adapters.bridges import SafetyClampBridge
from PhyAgentOS.runtime.policy.dummy_client import DummyPolicyClient
from PhyAgentOS.runtime.schemas import AdapterPlan, SessionSpec
from PhyAgentOS.runtime.skills.vla.openpi_sim_runtime import OpenPISimSkillRuntime
from PhyAgentOS.runtime.targets.local.dummy_sim_target import DummySimTarget


def test_dummy_sim_rollout_succeeds() -> None:
    session = SessionSpec.model_validate(
        {
            "session_id": "sess_1",
            "target_ref": "target://dummy_sim",
            "skill_ref": "skill://openpi_sim_vla",
            "task_description": "move",
            "routing": {"target_endpoint": "targetws://local/dummy_sim", "policy_endpoint": "dummy://local"},
            "execution": {"max_steps": 10, "replan_every_steps": 4, "action_chunk_mode": "chunk_buffer"},
        }
    )
    target = DummySimTarget({"image_size": 16, "state_dim": 8, "action_dim": 7, "success_after_steps": 3})
    result = OpenPISimSkillRuntime().run(
        session,
        target,
        DummySimTargetAdapter(),
        DummyOpenPIAdapter(),
        [SafetyClampBridge()],
        DummyPolicyClient(action_dim=7, chunk_size=4),
        AdapterPlan(
            target_adapter="target_adapter://dummy_sim_adapter",
            policy_adapter="policy_adapter://dummy_openpi_adapter",
            action_bridges=["bridge://safety_clamp"],
        ),
    )

    assert result.success is True
    assert result.status == "succeeded"
    assert result.num_steps == 3
