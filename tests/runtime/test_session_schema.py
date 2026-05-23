from __future__ import annotations

import pytest
from pydantic import ValidationError

from PhyAgentOS.runtime.schemas import (
    SessionExecution,
    SessionRouting,
    SessionsDocument,
    SessionSpec,
    SessionStatus,
    SkillSpec,
    TargetSpec,
)
from PhyAgentOS.runtime.schemas.session import InvalidStatusTransition, validate_status_transition


def test_session_document_defaults() -> None:
    doc = SessionsDocument.model_validate(
        {
            "sessions": [
                {
                    "session_id": "sess_1",
                    "target_ref": "target://dummy_sim",
                    "skill_ref": "skill://openpi_sim_vla",
                    "task_description": "move object",
                    "routing": {"target_endpoint": "targetws://local/dummy_sim", "policy_endpoint": "dummy://local"},
                }
            ]
        }
    )

    session = doc.sessions[0]
    assert session.status == SessionStatus.PENDING
    assert session.timeouts.policy_timeout_s == 5
    assert session.execution == SessionExecution()


def test_missing_required_session_field_fails() -> None:
    with pytest.raises(ValidationError):
        SessionSpec.model_validate(
            {
                "session_id": "sess_1",
                "target_ref": "target://dummy_sim",
                "skill_ref": "skill://openpi_sim_vla",
                "routing": {"target_endpoint": "targetws://local/dummy_sim", "policy_endpoint": "dummy://local"},
            }
        )


def test_invalid_status_transition_fails() -> None:
    with pytest.raises(InvalidStatusTransition):
        validate_status_transition(SessionStatus.PENDING, SessionStatus.SUCCEEDED)


def test_target_and_skill_schema() -> None:
    target = TargetSpec.model_validate(
        {
            "id": "dummy_sim",
            "type": "sim",
            "backend": "dummy",
            "workspace": "workspaces/dummy_sim",
            "supported_skills": ["openpi_sim_vla"],
            "runtime": {
                "target_runtime": "DummySimTargetRuntime",
                "target_endpoint": "targetws://local/dummy_sim",
                "target_adapter": "target_adapter://dummy_sim_adapter",
                "runtime_contract_ref": "configs/runtime/contracts/dummy_sim.runtime.yaml",
            },
        }
    )
    skill = SkillSpec.model_validate(
        {
            "id": "openpi_sim_vla",
            "category": "vla",
            "runtime": "OpenPISimSkillRuntime",
            "supported_target_types": ["sim"],
        }
    )

    assert target.enabled is True
    assert skill.default_replan_every == 1


def test_routing_required() -> None:
    routing = SessionRouting(target_endpoint="targetws://local/dummy_sim", policy_endpoint="dummy://local")
    assert routing.policy_endpoint == "dummy://local"
