"""Execution session schemas and state transition rules."""

from __future__ import annotations

from datetime import datetime
from enum import StrEnum
from typing import Literal

from pydantic import BaseModel, Field

from PhyAgentOS.runtime.schemas.result import SessionResult


class SessionStatus(StrEnum):
    PENDING = "pending"
    CLAIMED = "claimed"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMED_OUT = "timed_out"
    CANCELLING = "cancelling"
    CANCELLED = "cancelled"
    REJECTED = "rejected"


TERMINAL_SESSION_STATUSES = {
    SessionStatus.SUCCEEDED,
    SessionStatus.FAILED,
    SessionStatus.TIMED_OUT,
    SessionStatus.CANCELLED,
    SessionStatus.REJECTED,
}

ALLOWED_STATUS_TRANSITIONS: dict[SessionStatus, set[SessionStatus]] = {
    SessionStatus.PENDING: {SessionStatus.CLAIMED, SessionStatus.REJECTED, SessionStatus.CANCELLED},
    SessionStatus.CLAIMED: {SessionStatus.RUNNING, SessionStatus.FAILED, SessionStatus.REJECTED},
    SessionStatus.RUNNING: {
        SessionStatus.SUCCEEDED,
        SessionStatus.FAILED,
        SessionStatus.TIMED_OUT,
        SessionStatus.CANCELLING,
    },
    SessionStatus.CANCELLING: {SessionStatus.CANCELLED, SessionStatus.FAILED},
}


class InvalidStatusTransition(ValueError):
    """Raised when runtime code attempts an invalid session status transition."""


def validate_status_transition(current: SessionStatus, next_status: SessionStatus) -> None:
    """Validate a session status transition."""
    if current == next_status:
        return
    if next_status not in ALLOWED_STATUS_TRANSITIONS.get(current, set()):
        raise InvalidStatusTransition(f"invalid session transition: {current} -> {next_status}")


class SessionTimeouts(BaseModel):
    queue_timeout_s: float = 30
    execute_timeout_s: float = 300
    policy_timeout_s: float = 5


class SessionRetry(BaseModel):
    max_retries: int = 0
    attempted: int = 0


class SessionRouting(BaseModel):
    policy_endpoint: str
    adapter: str


class SessionExecution(BaseModel):
    max_steps: int = 600
    replan_every: int = 8
    action_chunk_mode: Literal["open_loop", "single_step"] = "open_loop"


class SessionSpec(BaseModel):
    session_id: str
    goal_id: str | None = None
    target_ref: str
    skill_ref: str
    task_description: str
    status: SessionStatus = SessionStatus.PENDING
    priority: Literal["low", "normal", "high"] = "normal"
    created_at: datetime | None = None
    updated_at: datetime | None = None
    claimed_by: str | None = None
    claim_token: str | None = None
    timeouts: SessionTimeouts = Field(default_factory=SessionTimeouts)
    retry: SessionRetry = Field(default_factory=SessionRetry)
    routing: SessionRouting
    execution: SessionExecution = Field(default_factory=SessionExecution)
    result: SessionResult = Field(default_factory=SessionResult)


class SessionsDocument(BaseModel):
    version: Literal["runtime_sessions_v1"] = "runtime_sessions_v1"
    sessions: list[SessionSpec] = Field(default_factory=list)
