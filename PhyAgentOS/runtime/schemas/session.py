"""Execution session schemas and state transition rules."""

from __future__ import annotations

from datetime import datetime
from enum import StrEnum
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from PhyAgentOS.runtime.schemas.result import SessionResult


class SessionStatus(StrEnum):
    PENDING = "pending"
    CLAIMED = "claimed"
    PREFLIGHT_CHECKING = "preflight_checking"
    RUNNING = "running"
    FINALIZING = "finalizing"
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
    SessionStatus.CLAIMED: {SessionStatus.PREFLIGHT_CHECKING, SessionStatus.FAILED, SessionStatus.REJECTED},
    SessionStatus.PREFLIGHT_CHECKING: {SessionStatus.RUNNING, SessionStatus.FAILED, SessionStatus.REJECTED},
    SessionStatus.RUNNING: {
        SessionStatus.FINALIZING,
        SessionStatus.SUCCEEDED,
        SessionStatus.FAILED,
        SessionStatus.TIMED_OUT,
        SessionStatus.CANCELLING,
    },
    SessionStatus.FINALIZING: {
        SessionStatus.SUCCEEDED,
        SessionStatus.FAILED,
        SessionStatus.TIMED_OUT,
        SessionStatus.CANCELLED,
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
    preflight_timeout_s: float = 20
    execute_timeout_s: float = 300
    policy_timeout_s: float = 5


class SessionRetry(BaseModel):
    max_retries: int = 0
    attempted: int = 0


class SessionRouting(BaseModel):
    model_config = ConfigDict(extra="forbid")

    target_endpoint: str | None = None
    policy_endpoint: str | None = None
    adapter_resolution: Literal["strict_auto", "strict_override"] = "strict_auto"
    adapter_overrides: dict[str, Any] | None = None


class SessionExecution(BaseModel):
    max_steps: int = 600
    control_hz: float | None = None
    replan_every: int = 8
    replan_every_steps: int | None = None
    action_chunk_mode: Literal["chunk_buffer", "open_loop", "single_step"] = "chunk_buffer"
    chunk_switch_mode: Literal["soft_blend", "hard_switch"] = "hard_switch"


class SessionRuntimeHints(BaseModel):
    perception_queries: list[dict[str, Any]] = Field(default_factory=list)
    force_environment_refresh: bool = False
    preferred_replan_every_steps: int | None = None


class SessionSafetyProfile(BaseModel):
    profile: str = "default"
    workspace_bounds: str | None = None
    stop_on_policy_timeout: bool = True


class SessionSpec(BaseModel):
    session_id: str
    goal_id: str | None = None
    parent_goal_id: str | None = None
    horizon: Literal["short_term", "long_term"] | None = None
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
    depends_on: list[str] = Field(default_factory=list)
    routing: SessionRouting = Field(default_factory=SessionRouting)
    execution: SessionExecution = Field(default_factory=SessionExecution)
    runtime_hints: SessionRuntimeHints = Field(default_factory=SessionRuntimeHints)
    safety_profile: SessionSafetyProfile = Field(default_factory=SessionSafetyProfile)
    result: SessionResult = Field(default_factory=SessionResult)


class SessionsDocument(BaseModel):
    version: Literal["runtime_sessions_v1"] = "runtime_sessions_v1"
    sessions: list[SessionSpec] = Field(default_factory=list)
