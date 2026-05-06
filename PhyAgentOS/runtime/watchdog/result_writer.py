"""Write runtime results to artifacts and ENVIRONMENT.md."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

import yaml

from PhyAgentOS.runtime.artifacts.episode_writer import EpisodeWriter
from PhyAgentOS.runtime.schemas import SessionResult, SessionSpec, TargetSpec
from PhyAgentOS.runtime.schemas.common import utc_now
from PhyAgentOS.runtime.state_io.atomic_file import atomic_write_text

_ENV_BLOCK_RE = re.compile(
    r"(?P<fence>`{3,}|~{3,})\s*(?P<lang>json|yaml|yml)\s*\n(?P<body>.*?)(?:\n(?P=fence)\s*)",
    re.DOTALL | re.IGNORECASE,
)


def _load_environment_doc(path: Path) -> dict[str, Any]:
    """Load an ENVIRONMENT.md JSON/YAML fenced block, preserving unknown fields."""
    if not path.exists():
        return {}
    match = _ENV_BLOCK_RE.search(path.read_text(encoding="utf-8"))
    if match is None:
        return {}

    body = match.group("body")
    lang = match.group("lang").lower()
    try:
        if lang == "json":
            payload = json.loads(body)
        else:
            payload = yaml.safe_load(body) or {}
    except (json.JSONDecodeError, yaml.YAMLError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _dump_environment_doc(data: dict[str, Any]) -> str:
    """Serialize an environment document as Markdown with a JSON fenced block."""
    env_json = json.dumps(data, indent=2, ensure_ascii=False, sort_keys=False)
    return (
        "# Environment State\n\n"
        "Auto-updated by PhyAgentOS runtime and perception services.\n"
        "The JSON block below is merged by runtime writers; unrelated sections are preserved.\n\n"
        f"```json\n{env_json}\n```\n"
    )


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

    def write_environment_summary(
        self,
        session: SessionSpec,
        target: TargetSpec,
        result: SessionResult,
    ) -> None:
        environment_path = self.workspace / "ENVIRONMENT.md"
        environment = _load_environment_doc(environment_path)
        runtime = environment.get("runtime")
        if not isinstance(runtime, dict):
            runtime = {}

        session_summary = {
            "session_id": session.session_id,
            "target_id": target.id,
            "status": result.status,
            "success": bool(result.success),
            "artifact_dir": result.artifact_dir or "",
            "num_steps": result.num_steps,
            "return_value": result.return_value,
            "error_code": result.error_code,
            "error_message": result.error_message,
            "updated_at": utc_now().isoformat(),
        }

        sessions = runtime.get("sessions")
        if not isinstance(sessions, dict):
            sessions = {}
        sessions[session.session_id] = session_summary

        runtime.update(
            {
                "last_session_id": session.session_id,
                "last_target_id": target.id,
                "last_status": result.status,
                "last_success": bool(result.success),
                "last_artifact_dir": result.artifact_dir or "",
                "sessions": sessions,
            }
        )
        environment["runtime"] = runtime
        environment["updated_at"] = utc_now().isoformat()
        atomic_write_text(environment_path, _dump_environment_doc(environment))
