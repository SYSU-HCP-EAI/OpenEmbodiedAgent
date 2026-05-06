"""Runtime v2 watchdog supervisor."""

from __future__ import annotations

import socket
from pathlib import Path

from pydantic import ValidationError

from PhyAgentOS.runtime.adapters.factory import build_adapter
from PhyAgentOS.runtime.policy.factory import build_policy_client
from PhyAgentOS.runtime.schemas import SkillsDocument, TargetsDocument
from PhyAgentOS.runtime.schemas.common import strip_ref
from PhyAgentOS.runtime.skills.vla.openpi_sim_runtime import OpenPISimSkillRuntime
from PhyAgentOS.runtime.state_io.markdown_yaml import read_yaml_block
from PhyAgentOS.runtime.state_io.workspace_paths import RuntimeWorkspacePaths
from PhyAgentOS.runtime.targets.factory import build_target
from PhyAgentOS.runtime.watchdog.errors import SchemaValidationError
from PhyAgentOS.runtime.watchdog.registry import SessionRegistry
from PhyAgentOS.runtime.watchdog.result_writer import ResultWriter


class WatchdogSupervisor:
    """Claim and execute runtime sessions from a workspace."""

    def __init__(self, workspace: str | Path, worker_id: str | None = None):
        self.paths = RuntimeWorkspacePaths.from_path(workspace)
        self.workspace = self.paths.workspace
        self.worker_id = worker_id or f"runtime-watchdog@{socket.gethostname()}"
        self.registry = SessionRegistry(self.paths.sessions)
        self.result_writer = ResultWriter(self.workspace)

    def run_once(self) -> bool:
        session = self.registry.first_pending()
        if session is None:
            return False
        if not self.registry.try_claim(session.session_id, self.worker_id):
            return False

        session = self.registry.get_session(session.session_id)
        try:
            targets_doc, skills_doc = self._load_registries()
            target_id = strip_ref(session.target_ref, "target://")
            skill_id = strip_ref(session.skill_ref, "skill://")
            target_spec = self._find_target(targets_doc, target_id)
            skill_spec = self._find_skill(skills_doc, skill_id)
            if skill_id not in target_spec.supported_skills:
                raise SchemaValidationError(f"target {target_id} does not support skill {skill_id}")
            if target_spec.type not in skill_spec.supported_target_types:
                raise SchemaValidationError(f"skill {skill_id} does not support target type {target_spec.type}")

            self.registry.mark_running(session.session_id)
            session = self.registry.get_session(session.session_id)
            target = build_target(target_spec)
            adapter = build_adapter(session.routing.adapter or target_spec.adapter)
            policy_client = self._build_policy_client(session, target_spec)
            runtime = self._build_skill_runtime(skill_spec.runtime)
            try:
                result = runtime.run(session, target, adapter, policy_client)
            finally:
                policy_client.close()
                target.close()

            result = self.result_writer.write_episode(session, target_spec, skill_id, result)
            self.result_writer.write_environment_summary(session, target_spec, result)
            self.registry.mark_finished(session.session_id, result)
            return True
        except Exception as exc:
            self.registry.mark_failed(session.session_id, exc)
            return True

    def _load_registries(self) -> tuple[TargetsDocument, SkillsDocument]:
        try:
            targets_doc = TargetsDocument.model_validate(read_yaml_block(self.paths.targets))
            skills_doc = SkillsDocument.model_validate(read_yaml_block(self.paths.skills))
            return targets_doc, skills_doc
        except ValidationError as exc:
            raise SchemaValidationError(str(exc)) from exc

    def _find_target(self, document: TargetsDocument, target_id: str):
        for target in document.targets:
            if target.id == target_id and target.enabled:
                return target
        raise SchemaValidationError(f"enabled target not found: {target_id}")

    def _find_skill(self, document: SkillsDocument, skill_id: str):
        for skill in document.skills:
            if skill.id == skill_id:
                return skill
        raise SchemaValidationError(f"skill not found: {skill_id}")

    def _build_skill_runtime(self, runtime_name: str):
        if runtime_name == "OpenPISimSkillRuntime":
            return OpenPISimSkillRuntime()
        raise SchemaValidationError(f"unsupported skill runtime: {runtime_name}")

    def _build_policy_client(self, session, target_spec):
        action_cfg = target_spec.config.get("action", {})
        action_dim = target_spec.config.get("action_dim", action_cfg.get("action_dim", 7))
        chunk_size = target_spec.config.get("chunk_size", action_cfg.get("chunk_size", 4))
        return build_policy_client(
            session.routing.policy_endpoint,
            timeout_s=session.timeouts.policy_timeout_s,
            action_dim=int(action_dim),
            chunk_size=int(chunk_size),
        )
