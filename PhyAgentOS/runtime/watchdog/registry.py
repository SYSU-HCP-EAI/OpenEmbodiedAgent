"""Session registry backed by workspace/SESSIONS.md."""

from __future__ import annotations

import fcntl
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator
from uuid import uuid4

from pydantic import ValidationError

from PhyAgentOS.runtime.schemas import SessionResult, SessionsDocument, SessionSpec, SessionStatus
from PhyAgentOS.runtime.schemas.common import utc_now
from PhyAgentOS.runtime.schemas.session import validate_status_transition
from PhyAgentOS.runtime.state_io.markdown_yaml import read_yaml_block, write_yaml_block
from PhyAgentOS.runtime.watchdog.errors import error_code_for, terminal_status_for


class SessionRegistry:
    """Manage runtime session state transitions in a Markdown YAML document."""

    def __init__(self, sessions_path: Path):
        self.sessions_path = sessions_path
        self.lock_path = sessions_path.with_suffix(sessions_path.suffix + ".lock")
        self.last_claim_token: str | None = None

    def load(self) -> SessionsDocument:
        return self._load_unlocked()

    def _load_unlocked(self) -> SessionsDocument:
        try:
            return SessionsDocument.model_validate(read_yaml_block(self.sessions_path))
        except ValidationError as exc:
            raise ValueError(f"invalid sessions document {self.sessions_path}: {exc}") from exc

    def save(self, document: SessionsDocument) -> None:
        with self._exclusive_lock():
            self._save_unlocked(document)

    def _save_unlocked(self, document: SessionsDocument) -> None:
        write_yaml_block(
            self.sessions_path,
            "Runtime Sessions",
            document.model_dump(mode="json", exclude_none=True),
        )

    def first_pending(self, document: SessionsDocument | None = None) -> SessionSpec | None:
        document = document or self.load()
        for session in document.sessions:
            if session.status == SessionStatus.PENDING:
                return session
        return None

    def get_session(self, session_id: str) -> SessionSpec:
        for session in self.load().sessions:
            if session.session_id == session_id:
                return session
        raise KeyError(f"session not found: {session_id}")

    def try_claim(self, session_id: str, worker_id: str) -> bool:
        """Claim a pending session and verify ownership after the write."""
        with self._exclusive_lock():
            document = self._load_unlocked()
            claim_token = uuid4().hex
            changed = False
            for session in document.sessions:
                if session.session_id != session_id:
                    continue
                if session.status != SessionStatus.PENDING:
                    return False
                validate_status_transition(session.status, SessionStatus.CLAIMED)
                session.status = SessionStatus.CLAIMED
                session.claimed_by = worker_id
                session.claim_token = claim_token
                session.updated_at = utc_now()
                changed = True
                break
            if not changed:
                return False

            self._save_unlocked(document)
            owned = self._get_session_from_document(document, session_id)
            verified = (
                owned.status == SessionStatus.CLAIMED
                and owned.claimed_by == worker_id
                and owned.claim_token == claim_token
            )
            self.last_claim_token = claim_token if verified else None
            return verified

    def mark_running(self, session_id: str) -> None:
        self._update_session_status(session_id, SessionStatus.RUNNING)

    def mark_finalizing(self, session_id: str) -> None:
        self._update_session_status(session_id, SessionStatus.FINALIZING)

    def mark_preflight_checking(self, session_id: str) -> None:
        self._update_session_status(session_id, SessionStatus.PREFLIGHT_CHECKING)

    def mark_rejected(self, session_id: str, result: SessionResult) -> None:
        result.status = SessionStatus.REJECTED.value
        result.success = False
        self._update_session_status(session_id, SessionStatus.REJECTED, result=result)

    def mark_succeeded(self, session_id: str, result: SessionResult) -> None:
        result.status = SessionStatus.SUCCEEDED.value
        result.success = True if result.success is None else result.success
        self._update_session_status(session_id, SessionStatus.SUCCEEDED, result=result)

    def mark_failed(self, session_id: str, error: Exception) -> None:
        status = SessionStatus(terminal_status_for(error))
        result = SessionResult(
            status=status.value,
            success=False,
            error_code=error_code_for(error),
            error_message=str(error),
        )
        self._update_session_status(session_id, status, result=result)

    def mark_timed_out(self, session_id: str, result: SessionResult) -> None:
        result.status = SessionStatus.TIMED_OUT.value
        result.success = False
        self._update_session_status(session_id, SessionStatus.TIMED_OUT, result=result)

    def mark_execution_failed(self, session_id: str, error: Exception) -> None:
        result = SessionResult(
            status=SessionStatus.FAILED.value,
            success=False,
            error_code=error_code_for(error),
            error_message=str(error),
        )
        self._update_session_status(session_id, SessionStatus.FAILED, result=result)

    def mark_finished(self, session_id: str, result: SessionResult) -> None:
        status = SessionStatus(result.status or (SessionStatus.SUCCEEDED.value if result.success else SessionStatus.FAILED.value))
        self._update_session_status(session_id, status, result=result)

    def mark_retry_pending(self, session_id: str, error: Exception) -> None:
        """Return a claimed/running session to pending for a configured retry."""
        with self._exclusive_lock():
            document = self._load_unlocked()
            for session in document.sessions:
                if session.session_id != session_id:
                    continue
                session.status = SessionStatus.PENDING
                session.claimed_by = None
                session.claim_token = None
                session.retry.attempted += 1
                session.updated_at = utc_now()
                session.result = SessionResult(
                    status=SessionStatus.PENDING.value,
                    success=False,
                    error_code=error_code_for(error),
                    error_message=str(error),
                )
                self._save_unlocked(document)
                return
        raise KeyError(f"session not found: {session_id}")

    def _update_session_status(
        self,
        session_id: str,
        status: SessionStatus,
        result: SessionResult | None = None,
    ) -> None:
        with self._exclusive_lock():
            document = self._load_unlocked()
            for session in document.sessions:
                if session.session_id != session_id:
                    continue
                validate_status_transition(session.status, status)
                session.status = status
                session.updated_at = utc_now()
                if result is not None:
                    session.result = result
                self._save_unlocked(document)
                return
        raise KeyError(f"session not found: {session_id}")

    def _get_session_from_document(self, document: SessionsDocument, session_id: str) -> SessionSpec:
        for session in document.sessions:
            if session.session_id == session_id:
                return session
        raise KeyError(f"session not found: {session_id}")

    @contextmanager
    def _exclusive_lock(self) -> Iterator[None]:
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock_path.open("a+", encoding="utf-8") as handle:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
