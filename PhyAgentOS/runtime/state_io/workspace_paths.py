"""Canonical paths for runtime workspace protocol files."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RuntimeWorkspacePaths:
    workspace: Path

    @classmethod
    def from_path(cls, workspace: str | Path) -> "RuntimeWorkspacePaths":
        return cls(Path(workspace).expanduser())

    @property
    def targets(self) -> Path:
        return self.workspace / "TARGETS.md"

    @property
    def skills(self) -> Path:
        return self.workspace / "SKILLS.md"

    @property
    def sessions(self) -> Path:
        return self.workspace / "SESSIONS.md"

    @property
    def environment(self) -> Path:
        return self.workspace / "ENVIRONMENT.md"

    @property
    def artifacts_root(self) -> Path:
        return self.workspace / "artifacts" / "runtime"
