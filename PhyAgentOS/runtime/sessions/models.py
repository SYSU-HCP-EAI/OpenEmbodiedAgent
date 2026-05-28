"""Data models shared by session runner and skill runtimes."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

from PhyAgentOS.runtime.schemas import SessionResult, SessionSpec, SkillSpec, TargetSpec


@dataclass
class RuntimeObservation:
    observation_type: Literal[
        "empty",
        "structured",
        "visual",
        "proprioceptive",
        "multimodal",
        "environment_only",
    ]
    timestamp_ns: int | None
    data: dict[str, Any]
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class EnvironmentRequest:
    requested_outputs: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class EnvironmentSnapshot:
    data: dict[str, Any] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class SessionState:
    session_id: str
    step_index: int = 0
    trace_id: str | None = None
    cancelled: bool = False
    last_status: dict[str, Any] = field(default_factory=dict)


@dataclass
class SkillContext:
    session: SessionSpec
    target: TargetSpec
    skill: SkillSpec
    task_description: str
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class SkillRuntimeResult:
    status: Literal["succeeded", "failed", "timed_out", "cancelled"]
    success: bool
    final_status: dict[str, Any] = field(default_factory=dict)
    error_code: str | None = None
    error_message: str | None = None
    artifacts: dict[str, Any] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_session_result(self) -> SessionResult:
        status = _json_safe_status(self.final_status)
        return SessionResult(
            status=self.status,
            success=self.success,
            num_steps=status.get("target_step_index", status.get("executed_steps")),
            return_value=status.get("reward"),
            mean_policy_latency_ms=self.metadata.get("mean_policy_latency_ms"),
            error_code=self.error_code,
            error_message=self.error_message,
            metadata={**self.metadata, "final_status": status, "artifacts": self.artifacts},
        )


def _json_safe_status(status: dict[str, Any]) -> dict[str, Any]:
    safe = {}
    for key, value in status.items():
        if key == "obs":
            continue
        if hasattr(value, "shape") and hasattr(value, "dtype"):
            safe[key] = {"shape": list(value.shape), "dtype": str(value.dtype)}
        else:
            safe[key] = value
    return safe
