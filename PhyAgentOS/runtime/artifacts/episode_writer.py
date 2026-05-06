"""Episode artifact writer."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from PhyAgentOS.runtime.schemas import SessionResult, SessionSpec, TargetSpec
from PhyAgentOS.runtime.state_io.atomic_file import atomic_write_text


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, list | tuple):
        return [_jsonable(v) for v in value]
    return value


class EpisodeWriter:
    """Write episode-level runtime artifacts under artifacts/runtime."""

    def __init__(self, artifacts_root: Path):
        self.artifacts_root = artifacts_root

    def write_episode(
        self,
        session: SessionSpec,
        target: TargetSpec,
        skill_id: str,
        result: SessionResult,
    ) -> Path:
        artifact_dir = self.artifacts_root / session.session_id
        artifact_dir.mkdir(parents=True, exist_ok=True)
        episode_path = artifact_dir / "episode.json"
        payload = {
            "session_id": session.session_id,
            "target_id": target.id,
            "skill_id": skill_id,
            "success": result.success,
            "status": result.status,
            "num_steps": result.num_steps,
            "return_value": result.return_value,
            "policy_latency_ms": {
                "mean": result.mean_policy_latency_ms,
            },
            "error_code": result.error_code,
            "error_message": result.error_message,
            "metadata": result.metadata,
        }
        atomic_write_text(episode_path, json.dumps(_jsonable(payload), indent=2, sort_keys=True) + "\n")
        return artifact_dir
