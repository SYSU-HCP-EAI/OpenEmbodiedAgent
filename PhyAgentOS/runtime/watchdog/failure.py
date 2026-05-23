"""Failure escalation and retry handling for runtime sessions."""

from __future__ import annotations

from PhyAgentOS.runtime.schemas import SessionStatus
from PhyAgentOS.runtime.watchdog.errors import SchemaValidationError
from PhyAgentOS.runtime.watchdog.registry import SessionRegistry


class FailureEscalator:
    """Apply retry or terminal failure policy for a session exception."""

    def handle(self, session_id: str, exc: Exception, registry: SessionRegistry) -> None:
        session = registry.get_session(session_id)
        if isinstance(exc, SchemaValidationError):
            if session.status == SessionStatus.RUNNING:
                registry.mark_execution_failed(session_id, exc)
                return
            registry.mark_failed(session_id, exc)
            return
        if session.retry.attempted < session.retry.max_retries:
            registry.mark_retry_pending(session_id, exc)
            return
        registry.mark_failed(session_id, exc)
