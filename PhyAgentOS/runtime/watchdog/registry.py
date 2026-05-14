"""Session registry backed by workspace/SESSIONS.md."""

from __future__ import annotations

from pathlib import Path
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
        self.last_claim_token: str | None = None

    def load(self) -> SessionsDocument:
        try:
            return SessionsDocument.model_validate(read_yaml_block(self.sessions_path))
        except ValidationError as exc:
            raise ValueError(f"invalid sessions document {self.sessions_path}: {exc}") from exc

    def save(self, document: SessionsDocument) -> None:
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
        document = self.load()
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

        self.save(document)
        owned = self.get_session(session_id)
        verified = (
            owned.status == SessionStatus.CLAIMED
            and owned.claimed_by == worker_id
            and owned.claim_token == claim_token
        )
        self.last_claim_token = claim_token if verified else None
        return verified

    def mark_running(self, session_id: str) -> None:
        self._update_session_status(session_id, SessionStatus.RUNNING)

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

    def mark_finished(self, session_id: str, result: SessionResult) -> None:
        status = SessionStatus(result.status or (SessionStatus.SUCCEEDED.value if result.success else SessionStatus.FAILED.value))
        self._update_session_status(session_id, status, result=result)

    def mark_retry_pending(self, session_id: str, error: Exception) -> None:
        """Return a claimed/running session to pending for a configured retry."""
        document = self.load()
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
            self.save(document)
            return
        raise KeyError(f"session not found: {session_id}")

    def _update_session_status(
        self,
        session_id: str,
        status: SessionStatus,
        result: SessionResult | None = None,
    ) -> None:
        document = self.load()
        for session in document.sessions:
            if session.session_id != session_id:
                continue
            validate_status_transition(session.status, status)
            session.status = status
            session.updated_at = utc_now()
            if result is not None:
                session.result = result
            self.save(document)
            return
        raise KeyError(f"session not found: {session_id}")
