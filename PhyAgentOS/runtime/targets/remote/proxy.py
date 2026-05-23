"""Generic remote target proxy backed by the runtime RPC protocol."""

from __future__ import annotations

from typing import Any

from PhyAgentOS.runtime.communication.target_ws_client import TargetWSClient
from PhyAgentOS.runtime.targets.base import BaseRolloutTarget


class RemoteTargetProxy(BaseRolloutTarget):
    """Expose a remote TargetRuntime through the local rollout target interface."""

    def __init__(self, client: TargetWSClient, *, config: dict[str, Any] | None = None):
        self.client = client
        self.config = dict(config or {})
        self._session_id: str | None = None
        self._skill_id: str | None = None

    def build(self) -> None:
        self.client.connect()

    def reset(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        self._remember_session(session_ctx)
        return self.client.call(
            "target.reset",
            session_ctx,
            session_id=self._session_id,
            skill_id=self._skill_id,
        )

    def observe(self) -> dict[str, Any]:
        return self.client.call(
            "target.observe",
            {},
            session_id=self._session_id,
            skill_id=self._skill_id,
        )

    def observe_for_environment(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        self._remember_session(session_ctx)
        return self.client.call(
            "target.observe",
            {"environment_refresh": True},
            session_id=self._session_id,
            skill_id=self._skill_id,
        )

    def step(self, action: Any) -> dict[str, Any]:
        return self.client.call(
            "target.action_chunk",
            {"actions": [action]},
            session_id=self._session_id,
            skill_id=self._skill_id,
        )

    def configure_session(self, session_ctx: dict[str, Any]) -> dict[str, Any]:
        self._remember_session(session_ctx)
        return self.client.call(
            "target.configure_session",
            session_ctx,
            session_id=self._session_id,
            skill_id=self._skill_id,
        )

    def action_chunk(self, executable_action_chunk: dict[str, Any]) -> dict[str, Any]:
        return self.client.call(
            "target.action_chunk",
            executable_action_chunk,
            session_id=self._session_id,
            skill_id=self._skill_id,
        )

    def execution_status(self) -> dict[str, Any]:
        return self.client.call(
            "target.execution_status",
            {},
            session_id=self._session_id,
            skill_id=self._skill_id,
        )

    def close(self) -> None:
        try:
            if self.client.connected:
                self.client.call(
                    "target.close",
                    {},
                    session_id=self._session_id,
                    skill_id=self._skill_id,
                )
        finally:
            self.client.close()

    def _remember_session(self, session_ctx: dict[str, Any]) -> None:
        self._session_id = session_ctx.get("session_id", self._session_id)
        skill_ref = session_ctx.get("skill_ref")
        if isinstance(skill_ref, str) and skill_ref.startswith("skill://"):
            self._skill_id = skill_ref.removeprefix("skill://")
