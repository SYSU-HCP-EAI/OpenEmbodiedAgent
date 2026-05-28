"""Lightweight runtime preflight checks."""

from __future__ import annotations

from dataclasses import dataclass

from PhyAgentOS.runtime.policy.factory import parse_policy_endpoint
from PhyAgentOS.runtime.watchdog.scheduler import ScheduledSession


@dataclass(frozen=True)
class HealthCheck:
    name: str
    status: str
    message: str = ""


@dataclass(frozen=True)
class HealthReport:
    checks: list[HealthCheck]

    @property
    def ok(self) -> bool:
        return not any(check.status == "failed" for check in self.checks)

    def summary(self) -> str:
        failed = [check for check in self.checks if check.status == "failed"]
        if not failed:
            return "health preflight passed"
        return "; ".join(f"{check.name}: {check.message}" for check in failed)


class HealthMonitor:
    """Run synchronous preflight checks before session execution."""

    def preflight(self, scheduled: ScheduledSession) -> HealthReport:
        session = scheduled.session
        checks = [
            self._check_target_enabled(scheduled),
            self._check_timeouts(scheduled),
            self._check_execution(scheduled),
            self._check_policy_endpoint(scheduled),
            HealthCheck("policy_connectivity", "unknown", "not implemented in serial MVP"),
            HealthCheck("robot_heartbeat", "unknown", "not implemented in serial MVP"),
            HealthCheck("chunk_buffer", "unknown", "not implemented in serial MVP"),
        ]
        if session.execution.action_chunk_mode not in {"chunk_buffer", "open_loop", "single_step"}:
            checks.append(HealthCheck("action_chunk_mode", "failed", "unsupported action chunk mode"))
        return HealthReport(checks=checks)

    def _check_target_enabled(self, scheduled: ScheduledSession) -> HealthCheck:
        if scheduled.target_spec.enabled:
            return HealthCheck("target_enabled", "passed")
        return HealthCheck("target_enabled", "unknown", "handled by runtime compatibility preflight")

    def _check_timeouts(self, scheduled: ScheduledSession) -> HealthCheck:
        timeouts = scheduled.session.timeouts
        values = {
            "queue_timeout_s": timeouts.queue_timeout_s,
            "preflight_timeout_s": timeouts.preflight_timeout_s,
            "execute_timeout_s": timeouts.execute_timeout_s,
            "policy_timeout_s": timeouts.policy_timeout_s,
        }
        bad = [name for name, value in values.items() if float(value) <= 0]
        if bad:
            return HealthCheck("timeouts", "failed", f"non-positive timeout fields: {', '.join(bad)}")
        return HealthCheck("timeouts", "passed")

    def _check_execution(self, scheduled: ScheduledSession) -> HealthCheck:
        execution = scheduled.session.execution
        if execution.max_steps <= 0:
            return HealthCheck("execution", "failed", "max_steps must be positive")
        replan_every = execution.replan_every_steps or execution.replan_every
        if replan_every <= 0:
            return HealthCheck("execution", "failed", "replan_every must be positive")
        return HealthCheck("execution", "passed")

    def _check_policy_endpoint(self, scheduled: ScheduledSession) -> HealthCheck:
        if scheduled.skill_spec.runtime_kind != "policy":
            return HealthCheck("policy_endpoint", "passed")
        if not scheduled.session.routing.policy_endpoint:
            return HealthCheck("policy_endpoint", "failed", "policy_endpoint is required for policy skills")
        try:
            parse_policy_endpoint(scheduled.session.routing.policy_endpoint)
        except Exception as exc:
            return HealthCheck("policy_endpoint", "failed", str(exc))
        return HealthCheck("policy_endpoint", "passed")
