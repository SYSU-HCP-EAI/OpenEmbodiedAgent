"""Thread-backed execution wrapper for a single session runner."""

from __future__ import annotations

import threading
import time

from PhyAgentOS.runtime.schemas import SessionResult
from PhyAgentOS.runtime.sessions.session_runner import SessionRunner


class RunnerThreadHandle:
    """Run SessionRunner.start in a daemon thread and expose non-blocking status."""

    def __init__(self, runner: SessionRunner):
        self.runner = runner
        self.started_at = time.monotonic()
        self.result: SessionResult | None = None
        self.exception: BaseException | None = None
        self._thread = threading.Thread(target=self._run, name=f"runner:{runner.session.session_id}", daemon=True)

    def start(self) -> None:
        self._thread.start()

    @property
    def done(self) -> bool:
        return not self._thread.is_alive()

    def elapsed_s(self) -> float:
        return time.monotonic() - self.started_at

    def snapshot(self) -> dict:
        snapshot = self.runner.snapshot()
        snapshot["thread_alive"] = self._thread.is_alive()
        snapshot["elapsed_s"] = self.elapsed_s()
        return snapshot

    def request_cancel_and_close(self, reason: str) -> None:
        """Request cancellation without letting cleanup block the watchdog thread."""

        def cleanup() -> None:
            try:
                self.runner.cancel(reason)
            except Exception:
                pass
            try:
                self.runner.close()
            except Exception:
                pass

        threading.Thread(
            target=cleanup,
            name=f"runner-cleanup:{self.runner.session.session_id}",
            daemon=True,
        ).start()

    def close(self) -> None:
        self.runner.close()

    def _run(self) -> None:
        try:
            self.result = self.runner.start()
        except BaseException as exc:
            self.exception = exc
