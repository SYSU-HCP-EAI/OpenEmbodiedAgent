from __future__ import annotations

import time

import pytest

from PhyAgentOS.runtime.schemas import SessionsDocument, SkillsDocument, TargetsDocument
from PhyAgentOS.runtime.skills.vla.openpi_sim_runtime import OpenPISimSkillRuntime
from PhyAgentOS.runtime.state_io.markdown_yaml import write_yaml_block
from PhyAgentOS.runtime.state_io.workspace_paths import RuntimeWorkspacePaths
from PhyAgentOS.runtime.targets.local.dummy_sim_target import DummySimTarget
from PhyAgentOS.runtime.watchdog.errors import SchemaValidationError
from PhyAgentOS.runtime.watchdog.failure import FailureEscalator
from PhyAgentOS.runtime.watchdog.health import HealthMonitor
from PhyAgentOS.runtime.watchdog.registry import SessionRegistry
from PhyAgentOS.runtime.watchdog.runtime_registry import (
    SkillRuntimeRegistry,
    TargetRuntimeRegistry,
    register_skill_runtime,
    register_target_runtime,
)
from PhyAgentOS.runtime.watchdog.scheduler import SessionScheduleError, SessionScheduler
from PhyAgentOS.runtime.watchdog.watcher import WorkspaceWatcher


def _targets_doc(*, supported_skills=None, enabled=True, target_type="sim"):
    return TargetsDocument.model_validate(
        {
            "version": "runtime_target_registry_v1",
            "targets": [
                {
                    "id": "dummy_sim",
                    "type": target_type,
                    "backend": "dummy",
                    "enabled": enabled,
                    "workspace": "workspaces/dummy_sim",
                    "supported_skills": supported_skills or ["openpi_sim_vla"],
                    "runtime": {
                        "target_runtime": "DummySimTargetRuntime",
                        "target_endpoint": "targetws://local/dummy_sim",
                        "target_adapter": "target_adapter://dummy_sim_adapter",
                        "runtime_contract_ref": "configs/runtime/contracts/dummy_sim.runtime.yaml",
                    },
                    "config": {"action": {"action_dim": 7, "chunk_size": 4}},
                }
            ],
        }
    )


def _skills_doc(*, supported_target_types=None):
    return SkillsDocument.model_validate(
        {
            "version": "runtime_skill_registry_v1",
            "skills": [
                {
                    "id": "openpi_sim_vla",
                    "category": "vla",
                    "runtime": "OpenPISimSkillRuntime",
                    "supported_target_types": supported_target_types or ["sim"],
                    "policy_adapter": "policy_adapter://dummy_openpi_adapter",
                }
            ],
        }
    )


def _sessions_doc(sessions):
    return SessionsDocument.model_validate({"version": "runtime_sessions_v1", "sessions": sessions})


def _session(session_id, *, priority="normal", status="pending", max_retries=0, attempted=0):
    return {
        "session_id": session_id,
        "target_ref": "target://dummy_sim",
        "skill_ref": "skill://openpi_sim_vla",
        "task_description": "move",
        "status": status,
        "priority": priority,
        "routing": {"target_endpoint": "targetws://local/dummy_sim", "policy_endpoint": "dummy://local"},
        "retry": {"max_retries": max_retries, "attempted": attempted},
    }


def test_workspace_watcher_detects_changes(tmp_path) -> None:
    paths = RuntimeWorkspacePaths.from_path(tmp_path)
    write_yaml_block(paths.sessions, "Runtime Sessions", {"version": "runtime_sessions_v1", "sessions": []})
    watcher = WorkspaceWatcher(paths)

    previous = watcher.snapshot()
    time.sleep(0.001)
    write_yaml_block(
        paths.sessions,
        "Runtime Sessions",
        {"version": "runtime_sessions_v1", "sessions": [_session("sess_1")]},
    )

    assert watcher.has_changed(previous) is True
    assert watcher.wait_for_change(previous, timeout_s=0.1, poll_interval_s=0.001) is not None


def test_session_scheduler_priority_order() -> None:
    sessions = _sessions_doc(
        [
            _session("normal_1", priority="normal"),
            _session("low_1", priority="low"),
            _session("high_1", priority="high"),
            _session("high_2", priority="high"),
        ]
    )

    scheduled = SessionScheduler().select_next(sessions, _targets_doc(), _skills_doc())

    assert scheduled is not None
    assert scheduled.session.session_id == "high_1"


def test_session_scheduler_rejects_incompatible_target_skill() -> None:
    sessions = _sessions_doc([_session("sess_1")])

    with pytest.raises(SessionScheduleError, match="does not support skill"):
        SessionScheduler().select_next(sessions, _targets_doc(supported_skills=["other_skill"]), _skills_doc())

    with pytest.raises(SessionScheduleError, match="does not support target type"):
        SessionScheduler().select_next(sessions, _targets_doc(), _skills_doc(supported_target_types=["real_robot"]))


def test_runtime_registries_build_dummy_runtime() -> None:
    target = TargetRuntimeRegistry().build(_targets_doc().targets[0])
    skill = SkillRuntimeRegistry().build("OpenPISimSkillRuntime")

    assert isinstance(target, DummySimTarget)
    assert isinstance(skill, OpenPISimSkillRuntime)


def test_runtime_registries_accept_runtime_extensions() -> None:
    class TestTarget:
        pass

    class TestSkill(OpenPISimSkillRuntime):
        pass

    register_target_runtime("TestLocalTargetRuntime", lambda target_spec: TestTarget())
    register_skill_runtime("TestSkillRuntime", TestSkill)

    target_doc = _targets_doc()
    target_doc.targets[0].runtime.target_runtime = "TestLocalTargetRuntime"

    assert isinstance(TargetRuntimeRegistry().build(target_doc.targets[0]), TestTarget)
    assert isinstance(SkillRuntimeRegistry().build("TestSkillRuntime"), TestSkill)


def test_health_monitor_preflight_success_for_dummy_session() -> None:
    scheduled = SessionScheduler().select_next(_sessions_doc([_session("sess_1")]), _targets_doc(), _skills_doc())
    assert scheduled is not None

    report = HealthMonitor().preflight(scheduled)

    assert report.ok is True
    assert any(check.name == "policy_connectivity" and check.status == "unknown" for check in report.checks)


def test_failure_escalator_retries_pending_session(tmp_path) -> None:
    sessions_path = tmp_path / "SESSIONS.md"
    write_yaml_block(
        sessions_path,
        "Runtime Sessions",
        {"version": "runtime_sessions_v1", "sessions": [_session("sess_1", max_retries=1)]},
    )
    registry = SessionRegistry(sessions_path)
    assert registry.try_claim("sess_1", "worker-a")

    FailureEscalator().handle("sess_1", RuntimeError("temporary"), registry)

    session = registry.get_session("sess_1")
    assert session.status == "pending"
    assert session.retry.attempted == 1
    assert session.claimed_by is None
    assert session.claim_token is None


def test_failure_escalator_marks_terminal_after_retry_exhausted(tmp_path) -> None:
    sessions_path = tmp_path / "SESSIONS.md"
    write_yaml_block(
        sessions_path,
        "Runtime Sessions",
        {
            "version": "runtime_sessions_v1",
            "sessions": [_session("sess_1", max_retries=1, attempted=1)],
        },
    )
    registry = SessionRegistry(sessions_path)
    assert registry.try_claim("sess_1", "worker-a")

    FailureEscalator().handle("sess_1", RuntimeError("boom"), registry)

    session = registry.get_session("sess_1")
    assert session.status == "failed"
    assert session.result.error_code == "RUNTIME_ERROR"


def test_failure_escalator_rejects_schema_errors_without_retry(tmp_path) -> None:
    sessions_path = tmp_path / "SESSIONS.md"
    write_yaml_block(
        sessions_path,
        "Runtime Sessions",
        {"version": "runtime_sessions_v1", "sessions": [_session("sess_1", max_retries=1)]},
    )
    registry = SessionRegistry(sessions_path)
    assert registry.try_claim("sess_1", "worker-a")

    FailureEscalator().handle("sess_1", SchemaValidationError("bad session"), registry)

    session = registry.get_session("sess_1")
    assert session.status == "rejected"
    assert session.retry.attempted == 0
    assert session.result.error_code == "SCHEMA_VALIDATION"
