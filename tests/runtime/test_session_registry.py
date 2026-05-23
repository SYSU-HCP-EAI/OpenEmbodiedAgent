from __future__ import annotations

from PhyAgentOS.runtime.schemas import SessionResult, SessionStatus
from PhyAgentOS.runtime.state_io.markdown_yaml import write_yaml_block
from PhyAgentOS.runtime.watchdog.registry import SessionRegistry


def _write_sessions(path):
    write_yaml_block(
        path,
        "Runtime Sessions",
        {
            "version": "runtime_sessions_v1",
            "sessions": [
                {
                    "session_id": "sess_1",
                    "target_ref": "target://dummy_sim",
                    "skill_ref": "skill://openpi_sim_vla",
                    "task_description": "move",
                    "status": "pending",
                    "routing": {"target_endpoint": "targetws://local/dummy_sim", "policy_endpoint": "dummy://local"},
                }
            ],
        },
    )


def test_claim_and_double_claim_prevention(tmp_path) -> None:
    sessions = tmp_path / "SESSIONS.md"
    _write_sessions(sessions)
    registry = SessionRegistry(sessions)

    assert registry.first_pending().session_id == "sess_1"
    assert registry.try_claim("sess_1", "worker-a") is True
    assert registry.try_claim("sess_1", "worker-b") is False

    claimed = registry.get_session("sess_1")
    assert claimed.status == SessionStatus.CLAIMED
    assert claimed.claimed_by == "worker-a"
    assert claimed.claim_token


def test_claim_lock_prevents_competing_registry_claims(tmp_path) -> None:
    sessions = tmp_path / "SESSIONS.md"
    _write_sessions(sessions)
    registry_a = SessionRegistry(sessions)
    registry_b = SessionRegistry(sessions)

    assert registry_a.try_claim("sess_1", "worker-a") is True
    assert registry_b.try_claim("sess_1", "worker-b") is False
    assert (tmp_path / "SESSIONS.md.lock").exists()

    claimed = registry_b.get_session("sess_1")
    assert claimed.claimed_by == "worker-a"


def test_running_and_succeeded(tmp_path) -> None:
    sessions = tmp_path / "SESSIONS.md"
    _write_sessions(sessions)
    registry = SessionRegistry(sessions)

    assert registry.try_claim("sess_1", "worker-a")
    registry.mark_preflight_checking("sess_1")
    registry.mark_running("sess_1")
    registry.mark_succeeded("sess_1", SessionResult(num_steps=3, return_value=1.0))

    finished = registry.get_session("sess_1")
    assert finished.status == SessionStatus.SUCCEEDED
    assert finished.result.success is True
    assert finished.result.num_steps == 3


def test_mark_failed(tmp_path) -> None:
    sessions = tmp_path / "SESSIONS.md"
    _write_sessions(sessions)
    registry = SessionRegistry(sessions)

    assert registry.try_claim("sess_1", "worker-a")
    registry.mark_preflight_checking("sess_1")
    registry.mark_running("sess_1")
    registry.mark_failed("sess_1", RuntimeError("boom"))

    failed = registry.get_session("sess_1")
    assert failed.status == SessionStatus.FAILED
    assert failed.result.error_code == "RUNTIME_ERROR"
