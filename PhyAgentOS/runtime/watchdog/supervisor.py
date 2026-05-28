"""Runtime v2 watchdog supervisor."""

from __future__ import annotations

import socket
from pathlib import Path

from pydantic import ValidationError

from PhyAgentOS.runtime.perception import PerceptionRuntime
from PhyAgentOS.runtime.policy.factory import build_policy_client
from PhyAgentOS.runtime.preflight import RuntimeCompatibilityPreflight
from PhyAgentOS.runtime.schemas import SessionsDocument, SkillsDocument, TargetsDocument
from PhyAgentOS.runtime.schemas.result import SessionResult
from PhyAgentOS.runtime.sessions.session_runner import SessionRunner
from PhyAgentOS.runtime.state_io.markdown_yaml import read_yaml_block
from PhyAgentOS.runtime.state_io.workspace_paths import RuntimeWorkspacePaths
from PhyAgentOS.runtime.watchdog.errors import SchemaValidationError
from PhyAgentOS.runtime.watchdog.failure import FailureEscalator
from PhyAgentOS.runtime.watchdog.health import HealthMonitor
from PhyAgentOS.runtime.watchdog.registry import SessionRegistry
from PhyAgentOS.runtime.watchdog.result_writer import ResultWriter
from PhyAgentOS.runtime.watchdog.runtime_registry import SkillRuntimeRegistry, TargetRuntimeRegistry
from PhyAgentOS.runtime.watchdog.scheduler import SessionScheduleError, SessionScheduler
from PhyAgentOS.runtime.watchdog.watcher import WorkspaceWatcher


class WatchdogSupervisor:
    """Claim and execute runtime sessions from a workspace."""

    def __init__(
        self,
        workspace: str | Path,
        worker_id: str | None = None,
        environment_workspace: str | Path | None = None,
    ):
        self.paths = RuntimeWorkspacePaths.from_path(workspace)
        self.workspace = self.paths.workspace
        self.environment_workspace = (
            Path(environment_workspace).expanduser() if environment_workspace is not None else self.workspace
        )
        self.worker_id = worker_id or f"runtime-watchdog@{socket.gethostname()}"
        self.registry = SessionRegistry(self.paths.sessions)
        self.result_writer = ResultWriter(self.workspace)
        self.watcher = WorkspaceWatcher(self.paths)
        self.scheduler = SessionScheduler()
        self.target_registry = TargetRuntimeRegistry()
        self.skill_registry = SkillRuntimeRegistry()
        self.perception_runtime = PerceptionRuntime(self.workspace, self.environment_workspace)
        self.health_monitor = HealthMonitor()
        self.failure_escalator = FailureEscalator()
        self.preflight = RuntimeCompatibilityPreflight(self.workspace, self.skill_registry)

    def run_once(self) -> bool:
        sessions_doc, targets_doc, skills_doc = self._load_runtime_documents()
        try:
            scheduled = self.scheduler.select_next(sessions_doc, targets_doc, skills_doc)
        except SessionScheduleError as exc:
            self.failure_escalator.handle(exc.session_id, exc, self.registry)
            return True
        if scheduled is None:
            return False
        session_id = scheduled.session.session_id
        if not self.registry.try_claim(session_id, self.worker_id):
            return False

        try:
            session = self.registry.get_session(session_id)
            _, targets_doc, skills_doc = self._load_runtime_documents()
            scheduled = self.scheduler.resolve_session(session, targets_doc, skills_doc)
            self.registry.mark_preflight_checking(session_id)
            session = self.registry.get_session(session_id)
            scheduled = self.scheduler.resolve_session(session, targets_doc, skills_doc)
            health_report = self.health_monitor.preflight(scheduled)
            if not health_report.ok:
                raise SchemaValidationError(health_report.summary())

            perception_plan = self.perception_runtime.resolve_and_check(scheduled)
            preflight_result = self.preflight.check(scheduled, perception_plan)
            if preflight_result.verdict == "rejected":
                result = SessionResult(
                    status="rejected",
                    success=False,
                    error_code="RUNTIME_PREFLIGHT_FAILED",
                    error_message=self._preflight_error_message(preflight_result),
                    metadata={"preflight": preflight_result.model_dump(mode="json", exclude_none=True)},
                )
                self.result_writer.write_lesson(
                    session,
                    scheduled.target_spec.id,
                    scheduled.skill_id,
                    "preflight_checking",
                    result.error_code,
                    result.error_message or "runtime compatibility preflight failed",
                    preflight_result.model_dump(mode="json", exclude_none=True),
                )
                self.registry.mark_rejected(session_id, result)
                return True

            self._write_preflight_metadata(session_id, preflight_result)

            self.registry.mark_running(session_id)
            session = self.registry.get_session(session_id)
            scheduled = self.scheduler.resolve_session(session, targets_doc, skills_doc)
            target_endpoint = session.routing.target_endpoint or scheduled.target_spec.runtime.target_endpoint
            target = self.target_registry.build(scheduled.target_spec, target_endpoint=target_endpoint)
            policy_client = None
            runner = None
            try:
                if scheduled.skill_spec.runtime_kind == "policy":
                    policy_client = self._build_policy_client(session, scheduled.target_spec)
                runtime = self.skill_registry.build(scheduled.skill_spec.runtime)
                runner = SessionRunner(
                    session=session,
                    target_spec=scheduled.target_spec,
                    skill_spec=scheduled.skill_spec,
                    adapter_plan=preflight_result.adapter_plan,
                    target=target,
                    skill_runtime=runtime,
                    policy_client=policy_client,
                    perception_runtime=self.perception_runtime,
                    perception_plan=perception_plan,
                    target_tool_manifest=preflight_result.target_tool_manifest,
                )
                result = runner.start()
            finally:
                if policy_client is not None:
                    policy_client.close()
                if runner is not None:
                    runner.close()
                else:
                    target.close()

            self.registry.mark_finalizing(session_id)
            result = self.result_writer.write_episode(
                session,
                scheduled.target_spec,
                scheduled.skill_id,
                result,
            )
            self.result_writer.write_session_history(session, scheduled.target_spec, result)
            self.registry.mark_finished(session_id, result)
            return True
        except Exception as exc:
            self.failure_escalator.handle(session_id, exc, self.registry)
            return True

    def _load_runtime_documents(self) -> tuple[SessionsDocument, TargetsDocument, SkillsDocument]:
        try:
            sessions_doc = SessionsDocument.model_validate(read_yaml_block(self.paths.sessions))
            targets_doc = TargetsDocument.model_validate(read_yaml_block(self.paths.targets))
            skills_doc = SkillsDocument.model_validate(read_yaml_block(self.paths.skills))
            return sessions_doc, targets_doc, skills_doc
        except ValidationError as exc:
            raise SchemaValidationError(str(exc)) from exc

    def _load_registries(self) -> tuple[TargetsDocument, SkillsDocument]:
        try:
            targets_doc = TargetsDocument.model_validate(read_yaml_block(self.paths.targets))
            skills_doc = SkillsDocument.model_validate(read_yaml_block(self.paths.skills))
            return targets_doc, skills_doc
        except ValidationError as exc:
            raise SchemaValidationError(str(exc)) from exc

    def _build_policy_client(self, session, target_spec):
        action_cfg = target_spec.config.get("action", {})
        action_dim = target_spec.config.get("action_dim", action_cfg.get("action_dim", 7))
        chunk_size = target_spec.config.get("chunk_size", action_cfg.get("chunk_size", 4))
        return build_policy_client(
            session.routing.policy_endpoint or "dummy://local",
            timeout_s=session.timeouts.policy_timeout_s,
            action_dim=int(action_dim),
            chunk_size=int(chunk_size),
        )

    def _preflight_error_message(self, preflight_result) -> str:
        if not preflight_result.missing_items:
            return "runtime compatibility preflight failed"
        return "; ".join(
            f"{item.code}: {item.field} expected {item.expected}"
            + (f", found {item.found}" if item.found is not None else "")
            for item in preflight_result.missing_items
        )

    def _write_preflight_metadata(self, session_id: str, preflight_result) -> None:
        document = self.registry.load()
        for session in document.sessions:
            if session.session_id != session_id:
                continue
            metadata = dict(session.result.metadata)
            metadata["preflight"] = preflight_result.model_dump(mode="json", exclude_none=True)
            session.result.metadata = metadata
            self.registry.save(document)
            return
