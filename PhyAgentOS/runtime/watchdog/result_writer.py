"""Write runtime results to artifacts."""

from __future__ import annotations

from pathlib import Path

from PhyAgentOS.runtime.artifacts.episode_writer import EpisodeWriter
from PhyAgentOS.runtime.schemas import SessionResult, SessionSpec, TargetSpec
from PhyAgentOS.runtime.schemas.common import utc_now
from PhyAgentOS.runtime.state_io.markdown_yaml import read_yaml_block, write_yaml_block


class ResultWriter:
    def __init__(self, workspace: Path):
        self.workspace = workspace
        self.episode_writer = EpisodeWriter(workspace / "artifacts" / "runtime")

    def write_episode(
        self,
        session: SessionSpec,
        target: TargetSpec,
        skill_id: str,
        result: SessionResult,
    ) -> SessionResult:
        artifact_dir = self.episode_writer.write_episode(session, target, skill_id, result)
        result.artifact_dir = str(artifact_dir.relative_to(self.workspace))
        return result

    def write_session_history(
        self,
        session: SessionSpec,
        target: TargetSpec,
        result: SessionResult,
    ) -> None:
        """Write transient runtime session history outside ENVIRONMENT.md."""
        path = self.workspace / "LOG.md"
        history = self._load_session_history(path)
        sessions = history.get("sessions")
        if not isinstance(sessions, dict):
            sessions = {}

        summary = {
            "session_id": session.session_id,
            "target_id": target.id,
            "status": result.status,
            "success": bool(result.success),
            "artifact_dir": result.artifact_dir or "",
            "num_steps": result.num_steps,
            "return_value": result.return_value,
            "mean_policy_latency_ms": result.mean_policy_latency_ms,
            "error_code": result.error_code,
            "error_message": result.error_message,
            "trace_path": result.trace_path,
            "updated_at": utc_now().isoformat(),
        }
        sessions[session.session_id] = {key: value for key, value in summary.items() if value is not None}
        history.update(
            {
                "version": "runtime_session_history_v1",
                "updated_at": utc_now().isoformat(),
                "last_session_id": session.session_id,
                "last_target_id": target.id,
                "last_status": result.status,
                "last_success": bool(result.success),
                "last_artifact_dir": result.artifact_dir or "",
                "sessions": sessions,
            }
        )
        write_yaml_block(path, "Runtime Session History", history)

    def _load_session_history(self, path: Path) -> dict:
        if not path.exists():
            return {"version": "runtime_session_history_v1", "sessions": {}}
        try:
            payload = read_yaml_block(path)
        except Exception:
            return {"version": "runtime_session_history_v1", "sessions": {}}
        if payload.get("version") != "runtime_session_history_v1":
            return {"version": "runtime_session_history_v1", "sessions": {}}
        return payload

    def write_lesson(
        self,
        session: SessionSpec,
        target_id: str,
        skill_id: str,
        phase: str,
        error_code: str | None,
        summary: str,
        metadata: dict,
    ) -> None:
        path = self.workspace / "LESSONS.md"
        payload = self._load_lessons(path)
        lessons = payload.get("lessons")
        if not isinstance(lessons, list):
            lessons = []
        lessons.append(
            {
                "id": f"lesson_{session.session_id}_{len(lessons) + 1}",
                "timestamp": utc_now().isoformat(),
                "session_id": session.session_id,
                "phase": phase,
                "error_code": error_code,
                "target_id": target_id,
                "skill_id": skill_id,
                "summary": summary,
                "metadata": metadata,
            }
        )
        write_yaml_block(
            path,
            "Runtime Lessons",
            {"version": "runtime_lessons_v1", "updated_at": utc_now().isoformat(), "lessons": lessons},
        )

    def _load_lessons(self, path: Path) -> dict:
        if not path.exists():
            return {"version": "runtime_lessons_v1", "lessons": []}
        try:
            payload = read_yaml_block(path)
        except Exception:
            return {"version": "runtime_lessons_v1", "lessons": []}
        if not isinstance(payload.get("lessons"), list):
            payload["lessons"] = []
        return payload
