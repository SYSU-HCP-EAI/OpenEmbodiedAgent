"""Workspace file change watcher for runtime protocol files."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

from PhyAgentOS.runtime.state_io.workspace_paths import RuntimeWorkspacePaths


@dataclass(frozen=True)
class FileFingerprint:
    exists: bool
    mtime_ns: int | None = None
    size: int | None = None


@dataclass(frozen=True)
class WorkspaceSnapshot:
    files: dict[str, FileFingerprint]


class WorkspaceWatcher:
    """Poll workspace protocol files for mtime/size changes."""

    def __init__(self, paths: RuntimeWorkspacePaths):
        self.paths = paths

    def snapshot(self) -> WorkspaceSnapshot:
        return WorkspaceSnapshot({name: self._fingerprint(path) for name, path in self._watched_files().items()})

    def has_changed(self, previous: WorkspaceSnapshot) -> bool:
        return self.snapshot() != previous

    def wait_for_change(
        self,
        previous: WorkspaceSnapshot,
        timeout_s: float | None = None,
        poll_interval_s: float = 1.0,
    ) -> WorkspaceSnapshot | None:
        started = time.monotonic()
        interval = max(0.0, float(poll_interval_s))
        while True:
            current = self.snapshot()
            if current != previous:
                return current
            if timeout_s is not None and time.monotonic() - started >= timeout_s:
                return None
            time.sleep(interval)

    def _watched_files(self) -> dict[str, Path]:
        return {
            "targets": self.paths.targets,
            "skills": self.paths.skills,
            "sessions": self.paths.sessions,
            "environment": self.paths.environment,
        }

    def _fingerprint(self, path: Path) -> FileFingerprint:
        if not path.exists():
            return FileFingerprint(exists=False)
        stat = path.stat()
        return FileFingerprint(exists=True, mtime_ns=stat.st_mtime_ns, size=stat.st_size)
