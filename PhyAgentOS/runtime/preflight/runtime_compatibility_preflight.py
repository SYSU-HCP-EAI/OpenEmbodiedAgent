"""Strict runtime compatibility preflight for session-centered runtime."""

from __future__ import annotations

from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import yaml
from pydantic import ValidationError

from PhyAgentOS.runtime.adapters.factory import build_action_bridge, build_policy_adapter, build_target_adapter
from PhyAgentOS.runtime.perception.config_resolver import ResolvedPerceptionPlan, sensor_by_id
from PhyAgentOS.runtime.schemas import (
    AdapterPlan,
    MissingItem,
    RuntimeCompatibilityPreflightResult,
    TargetRuntimeContractDocument,
    TargetToolManifest,
)
from PhyAgentOS.runtime.watchdog.runtime_registry import SkillRuntimeRegistry
from PhyAgentOS.runtime.watchdog.scheduler import ScheduledSession


class RuntimeCompatibilityPreflight:
    def __init__(self, workspace: Path, skill_registry: SkillRuntimeRegistry | None = None):
        self.workspace = workspace
        self.skill_registry = skill_registry or SkillRuntimeRegistry()

    def check(
        self,
        scheduled: ScheduledSession,
        perception_plan: ResolvedPerceptionPlan | None = None,
    ) -> RuntimeCompatibilityPreflightResult:
        missing: list[MissingItem] = []
        warnings: list[str] = []
        session = scheduled.session
        target = scheduled.target_spec
        skill = scheduled.skill_spec

        if not target.enabled:
            missing.append(self._missing("TARGET_DISABLED", "TARGETS.md targets[].enabled", "true", str(target.enabled), session.session_id, "Enable the target."))
        if target.target_class not in {"local", "remote"}:
            missing.append(self._missing("TARGET_CLASS_INVALID", "TARGETS.md targets[].target_class", "local or remote", target.target_class, session.session_id, "Set target_class."))
        if target.target_kind not in {"game", "debug", "simulation", "real_robot"}:
            missing.append(self._missing("TARGET_KIND_INVALID", "TARGETS.md targets[].target_kind", "game/debug/simulation/real_robot", target.target_kind, session.session_id, "Set target_kind."))
        endpoint = session.routing.target_endpoint or target.runtime.target_endpoint
        if target.target_class == "remote":
            if not endpoint or urlparse(endpoint).scheme != "targetws":
                missing.append(self._missing("TARGET_ENDPOINT_INVALID", "TARGETS.md targets[].runtime.target_endpoint", "targetws:// endpoint", endpoint, session.session_id, "Set a targetws:// endpoint for remote targets."))
        if target.target_class == "local" and endpoint:
            warnings.append("local target ignores routing.target_endpoint/runtime.target_endpoint")
        if target.target_kind == "real_robot" and skill.agent_exposure == "target_tools":
            missing.append(self._missing("TARGET_TOOL_POLICY_INVALID", "SKILLS.md skills[].agent_exposure", "constrained_target_tools for real_robot", skill.agent_exposure, session.session_id, "Use constrained_target_tools for real robots."))
        if scheduled.skill_id not in target.supported_skills:
            missing.append(self._missing("SKILL_NOT_SUPPORTED_BY_TARGET", "TARGETS.md targets[].supported_skills", scheduled.skill_id, None, session.session_id, "Choose a supported skill or update the target registry."))
        if target.runtime.target_adapter == "":
            missing.append(self._missing("ACTION_BRIDGE_MISSING", "TARGETS.md targets[].runtime.target_adapter", "target adapter URI", "", session.session_id, "Set runtime.target_adapter."))

        contract = self._load_contract(target.runtime.runtime_contract_ref, missing, session.session_id)
        if contract is not None:
            if contract.target_id != target.id:
                missing.append(self._missing("TARGET_RUNTIME_CONTRACT_INVALID", str(target.runtime.runtime_contract_ref), target.id, contract.target_id, session.session_id, "Use a contract with matching target_id."))
            if contract.target_adapter != target.runtime.target_adapter:
                missing.append(self._missing("TARGET_RUNTIME_CONTRACT_INVALID", f"{target.runtime.runtime_contract_ref} target_adapter", target.runtime.target_adapter, contract.target_adapter, session.session_id, "Align contract target_adapter with TARGETS.md."))

        try:
            self.skill_registry.build(skill.runtime)
        except Exception as exc:
            missing.append(self._missing("SKILL_RUNTIME_MISSING", "SKILLS.md skills[].runtime", "registered runtime", str(exc), session.session_id, "Register the skill runtime."))

        if skill.runtime_kind not in {"policy", "builtin"}:
            missing.append(self._missing("SKILL_RUNTIME_KIND_INVALID", "SKILLS.md skills[].runtime_kind", "policy or builtin", skill.runtime_kind, session.session_id, "Set runtime_kind."))
        if target.target_kind not in skill.supported_target_kinds:
            missing.append(self._missing("SKILL_TARGET_KIND_UNSUPPORTED", "SKILLS.md skills[].supported_target_kinds", target.target_kind, None, session.session_id, "Choose a compatible skill or target."))

        policy_adapter = skill.policy.policy_adapter if skill.runtime_kind == "policy" and skill.policy else None
        if skill.runtime_kind == "policy":
            if not policy_adapter:
                missing.append(self._missing("POLICY_INPUT_CONTRACT_UNSATISFIED", "SKILLS.md skills[].policy.policy_adapter", "policy adapter URI", None, session.session_id, "Set policy.policy_adapter for policy skills."))
            if not skill.policy or not skill.policy.policy_client:
                missing.append(self._missing("POLICY_INPUT_CONTRACT_UNSATISFIED", "SKILLS.md skills[].policy.policy_client", "policy client", None, session.session_id, "Set policy.policy_client."))
            policy_endpoint = session.routing.policy_endpoint
            if not policy_endpoint:
                missing.append(self._missing("POLICY_ENDPOINT_UNREACHABLE", "SESSIONS.md sessions[].routing.policy_endpoint", "policy endpoint", None, session.session_id, "Set routing.policy_endpoint."))
            else:
                scheme = urlparse(policy_endpoint).scheme
                if scheme not in {"dummy", "openpi", "policyws"}:
                    missing.append(self._missing("POLICY_ENDPOINT_UNREACHABLE", "SESSIONS.md sessions[].routing.policy_endpoint", "dummy://, openpi://, or policyws:// endpoint", policy_endpoint, session.session_id, "Use a supported policy endpoint."))
        if skill.runtime_kind == "builtin" and skill.agent_exposure != "none" and skill.target_tool_policy is None:
            missing.append(self._missing("TARGET_TOOL_POLICY_MISSING", "SKILLS.md skills[].target_tool_policy", "target tool policy", None, session.session_id, "Set target_tool_policy for exposed builtin skills."))

        self._check_empty_observation(scheduled, missing)

        if session.routing.adapter_resolution == "strict_override" and session.routing.adapter_overrides:
            overrides = session.routing.adapter_overrides
            target_adapter = str(overrides.get("target_adapter") or target.runtime.target_adapter)
            policy_adapter = str(overrides.get("policy_adapter") or policy_adapter) if policy_adapter else None
            bridges = list(overrides.get("action_bridges") or [])
        else:
            target_adapter = target.runtime.target_adapter
            bridges = self._resolve_bridges(skill, contract)

        adapter_plan = AdapterPlan(
            target_adapter=target_adapter,
            policy_adapter=policy_adapter,
            observation_path=[
                "target.raw_observation",
                "TargetAdapter.to_runtime_observation",
                "PolicyAdapter.to_policy_input" if policy_adapter else "skill.input",
            ],
            action_path=[
                "PolicyAdapter.from_policy_output" if policy_adapter else "skill.output",
                *bridges,
                "TargetAdapter.to_executable_action_chunk",
            ],
            action_bridges=bridges,
            validation_mode="strict",
        )
        self._check_adapter_plan(adapter_plan, missing, session.session_id)
        self._check_sensors(scheduled, perception_plan, missing)
        target_tool_manifest = self._target_tool_manifest(skill)
        if contract is not None:
            self._check_action_contract(scheduled, contract, adapter_plan, missing)

        return RuntimeCompatibilityPreflightResult(
            verdict="rejected" if missing else "accepted",
            session_id=session.session_id,
            target_id=target.id,
            skill_id=skill.id,
            skill_runtime_kind=skill.runtime_kind,
            execution_mode=skill.loop_mode,
            adapter_plan=None if missing else adapter_plan,
            target_tool_manifest=None if missing else target_tool_manifest,
            missing_items=missing,
            warnings=warnings,
        )

    def _load_contract(self, ref: Path, missing: list[MissingItem], session_id: str):
        path = ref if ref.is_absolute() else self.workspace / ref
        if not path.exists():
            missing.append(self._missing("TARGET_RUNTIME_CONTRACT_MISSING", str(ref), "readable runtime contract", None, session_id, "Create configs/runtime/contracts/<target_id>.runtime.yaml."))
            return None
        try:
            payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            return TargetRuntimeContractDocument.model_validate(payload)
        except (OSError, yaml.YAMLError, ValidationError) as exc:
            missing.append(self._missing("TARGET_ACTION_CONTRACT_INVALID", str(ref), "valid runtime_target_contract_v1", str(exc), session_id, "Fix runtime contract YAML."))
            return None

    def _resolve_bridges(self, skill, contract) -> list[str]:
        allowed = list(skill.adapter_requirements.get("allowed_bridges", []))
        available = set((contract.capabilities or {}).get("available_bridges", [])) if contract else set()
        bridges = [bridge for bridge in allowed if not available or bridge in available]
        if "bridge://safety_clamp" not in bridges:
            bridges.append("bridge://safety_clamp")
        return bridges

    def _check_adapter_plan(self, plan: AdapterPlan, missing: list[MissingItem], session_id: str) -> None:
        try:
            build_target_adapter(plan.target_adapter)
        except Exception as exc:
            missing.append(self._missing("ACTION_BRIDGE_MISSING", "adapter_plan.target_adapter", "registered target adapter", str(exc), session_id, "Register the target adapter."))
        if plan.policy_adapter:
            try:
                build_policy_adapter(plan.policy_adapter)
            except Exception as exc:
                missing.append(self._missing("POLICY_INPUT_CONTRACT_UNSATISFIED", "adapter_plan.policy_adapter", "registered policy adapter", str(exc), session_id, "Register the policy adapter."))
        for bridge in plan.action_bridges:
            try:
                build_action_bridge(bridge)
            except Exception as exc:
                missing.append(self._missing("ACTION_BRIDGE_MISSING", "adapter_plan.action_bridges", "registered action bridge", str(exc), session_id, "Register the bridge."))

    def _check_sensors(self, scheduled: ScheduledSession, plan: ResolvedPerceptionPlan | None, missing: list[MissingItem]) -> None:
        required = list(scheduled.skill_spec.requires.sensors)
        if scheduled.target_spec.observation.observation_type == "empty" or scheduled.skill_spec.observation_contract.observation_type == "empty":
            if required:
                missing.append(self._missing("EMPTY_OBSERVATION_INVALID", "SKILLS.md skills[].requires.sensors", "empty list when observation_type=empty", str(required), scheduled.session.session_id, "Do not use empty observation when sensors are required."))
            return
        if not required:
            return
        if plan is None:
            missing.append(self._missing("SENSOR_CONFIG_FILE_MISSING", "TARGETS.md targets[].perception.sensor_config_ref", "sensor config", None, scheduled.session.session_id, "Set sensor_config_ref and enable perception refs for sensor validation."))
            return
        sensors = sensor_by_id(plan.sensor_config)
        for sensor_id in required:
            sensor = sensors.get(sensor_id)
            if sensor is None:
                missing.append(self._missing("REQUIRED_SENSOR_MISSING", f"{plan.sensor_config_ref} sensors[{sensor_id}]", "enabled sensor", None, scheduled.session.session_id, "Add the required sensor."))
                continue
            if not sensor.enabled:
                missing.append(self._missing("REQUIRED_SENSOR_MISSING", f"{plan.sensor_config_ref} sensors[{sensor_id}].enabled", "true", "false", scheduled.session.session_id, "Enable the required sensor."))
            if sensor.observation_key not in plan.sensor_config.observation_schema:
                missing.append(self._missing("OBSERVATION_CHANNEL_MISSING", f"{plan.sensor_config_ref} observation_schema.{sensor.observation_key}", "observation schema entry", None, scheduled.session.session_id, "Add the observation schema entry."))

    def _check_action_contract(
        self,
        scheduled: ScheduledSession,
        contract: TargetRuntimeContractDocument,
        plan: AdapterPlan,
        missing: list[MissingItem],
    ) -> None:
        action = scheduled.skill_spec.output_contract.get("action", {})
        if not action:
            return
        target_action = contract.action_contract
        representation = action.get("representation")
        if representation and representation not in target_action.accepted_representations:
            missing.append(self._missing("ACTION_CONTRACT_UNSATISFIED", "SKILLS.md output_contract.action.representation", f"one of {target_action.accepted_representations}", representation, scheduled.session.session_id, "Add an explicit bridge or choose a compatible target."))
        policy_shape = action.get("shape")
        target_shape = target_action.shape
        if (
            isinstance(policy_shape, list)
            and len(policy_shape) == 2
            and len(target_shape) == 2
            and isinstance(policy_shape[1], int)
            and isinstance(target_shape[1], int)
            and int(policy_shape[1]) != int(target_shape[1])
        ):
            missing.append(self._missing("ACTION_CONTRACT_UNSATISFIED", "SKILLS.md output_contract.action.shape", str(target_shape), str(policy_shape), scheduled.session.session_id, "Declare an explicit component mapping; implicit truncation is forbidden."))
        if action.get("normalized") is True and target_action.normalized is False and "bridge://denormalization" not in plan.action_bridges:
            missing.append(self._missing("ACTION_BRIDGE_MISSING", "adapter_plan.action_bridges", "bridge://denormalization", None, scheduled.session.session_id, "Register denormalization bridge with stats."))

    def _check_empty_observation(self, scheduled: ScheduledSession, missing: list[MissingItem]) -> None:
        target_obs = scheduled.target_spec.observation
        skill_obs = scheduled.skill_spec.observation_contract
        if target_obs.observation_type != "empty" and skill_obs.observation_type != "empty":
            return
        if not (target_obs.observation_type == "empty" and target_obs.empty_observation_allowed):
            missing.append(self._missing("EMPTY_OBSERVATION_INVALID", "TARGETS.md targets[].observation", "explicit empty observation allowed", target_obs.observation_type, scheduled.session.session_id, "Declare empty observation on target."))
        if not (skill_obs.observation_type == "empty" and skill_obs.empty_observation_allowed):
            missing.append(self._missing("EMPTY_OBSERVATION_INVALID", "SKILLS.md skills[].observation_contract", "explicit empty observation allowed", skill_obs.observation_type, scheduled.session.session_id, "Declare empty observation on skill."))
        if scheduled.skill_spec.runtime_kind == "policy":
            missing.append(self._missing("EMPTY_OBSERVATION_INVALID", "SKILLS.md skills[].runtime_kind", "builtin for empty observation", "policy", scheduled.session.session_id, "Policy runtimes require explicit non-empty observation in this implementation."))

    def _target_tool_manifest(self, skill) -> TargetToolManifest | None:
        if skill.runtime_kind != "builtin" or skill.agent_exposure == "none" or skill.target_tool_policy is None:
            return None
        forbidden = {"reset", "close", "raw_sdk_command", "disable_safety"}
        exposed = [tool for tool in skill.target_tool_policy.expose if tool not in forbidden]
        return TargetToolManifest(expose=exposed, agent_exposure=skill.agent_exposure)

    def _missing(self, code: str, field: str, expected: str, found: str | None, session_id: str, fix: str) -> MissingItem:
        return MissingItem(
            code=code,
            field=field,
            expected=expected,
            found=found,
            triggered_by=session_id,
            fix=fix,
        )
