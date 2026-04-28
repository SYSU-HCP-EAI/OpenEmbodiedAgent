"""Embodied action tool for executing robot actions with Critic validation."""

from __future__ import annotations

import json
import logging
import re
from pathlib import Path
from typing import TYPE_CHECKING, Any

try:
    from loguru import logger
except ImportError:  # pragma: no cover - fallback for lightweight test envs
    logger = logging.getLogger(__name__)

from PhyAgentOS.agent.tools.base import Tool
from PhyAgentOS.embodiment_registry import EmbodimentRegistry
from PhyAgentOS.providers.base import LLMProvider
from PhyAgentOS.utils.action_queue import (
    append_action,
    dump_action_document,
    empty_action_document,
    normalize_action_document,
    parse_action_markdown,
    pending_action_type,
)

if TYPE_CHECKING:
    from PhyAgentOS.embodiment_registry import EmbodimentRegistry

# LLMs often put HAL fields next to action_type instead of inside `parameters`.
_TOOL_RESERVED_KEYS = frozenset({"action_type", "parameters", "reasoning"})
_ENV_JSON_BLOCK_RE = re.compile(r"```json\s*\n(.*?)\n```", re.DOTALL)


class EmbodiedActionTool(Tool):
    """Validate embodied actions and route them to the correct robot workspace."""

    @property
    def name(self) -> str:
        return "execute_robot_action"

    @property
    def description(self) -> str:
        return (
            "Execute a physical action on the robot or simulation. "
            "Most actions are validated by a Critic before execution; "
            "`start`/`enter_simulation` with no scene overrides bypass the Critic and rely on "
            "the HAL driver that was started with `--driver-config`. "
            "Requires a running HAL watchdog for the target driver; the watchdog copies "
            "EMBODIED.md from the driver profile into the workspace. "
            "For PiperGo2 manipulation sim, typical action_type values include: "
            "`enter_simulation` or `start` (same meaning; parameters may be `{}` when the "
            "watchdog was started with `--driver-config` — scene path/objects are already loaded). "
            "`navigate_to_named` (parameters: waypoint_key or target, e.g. desk, staging_table). "
            "`describe_visible_scene`, `run_pick_place`, `close`. "
            "Reference JSON for humans: workspace file `configs/pipergo2_manipulation_driver.json` "
            "(not the repo-relative `examples/` path)."
        )

    @property
    def parameters(self) -> dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "action_type": {
                    "type": "string",
                    "description": (
                        "The type of action to execute. Generic HAL examples: "
                        "'move_to', 'pick_up', 'put_down', 'semantic_navigate', 'connect_robot'. "
                        "PiperGo2 manipulation sim: 'enter_simulation'|'start', 'navigate_to_named', "
                        "'navigate_to_waypoint', 'describe_visible_scene', 'run_pick_place', 'close', 'api_call'."
                    ),
                },
                "parameters": {
                    "type": "object",
                    "description": (
                        "Action-specific parameters; use {} when optional. "
                        "For manipulation sim `start`/`enter_simulation`, {} is valid if HAL "
                        "watchdog was started with `--driver-config` (driver already holds scene JSON). "
                        "Include robot_id in fleet mode. "
                        "If the model places keys like waypoint_key at the top level by mistake, "
                        "they are folded into this object automatically."
                    ),
                },
                "reasoning": {
                    "type": "string",
                    "description": "The reasoning behind choosing this action.",
                },
            },
            "required": ["action_type", "parameters", "reasoning"],
        }

    def __init__(
        self,
        workspace: Path,
        provider: LLMProvider,
        model: str,
        registry: EmbodimentRegistry | None = None,
    ):
        self.workspace = workspace
        self.provider = provider
        self.model = model
        self.registry = registry

    def cast_params(self, params: dict[str, Any]) -> dict[str, Any]:
        """Normalize LLM output: missing `parameters`, and fold stray top-level HAL keys."""
        if not isinstance(params, dict):
            return super().cast_params({})
        merged = dict(params)
        if merged.get("parameters") is None:
            merged["parameters"] = {}
        inner: dict[str, Any] = dict(merged.get("parameters") or {})
        for key in list(merged.keys()):
            if key in _TOOL_RESERVED_KEYS:
                continue
            if key not in inner:
                inner[key] = merged[key]
            del merged[key]
        merged["parameters"] = inner
        return super().cast_params(merged)

    async def execute(
        self,
        action_type: str,
        parameters: dict[str, Any],
        reasoning: str,
    ) -> str:
        """Execute the action after Critic validation."""
        parameters = dict(parameters or {})
        robot_id = parameters.get("robot_id")
        if self.registry and self.registry.is_fleet and not robot_id:
            if EmbodiedActionTool._fleet_broadcast_action_allowed(action_type, parameters):
                return self._accept_action_broadcast(action_type, parameters)
            return "Error: robot_id is required for embodied actions in fleet mode."

        try:
            embodied_file = self._resolve_embodied_file(robot_id)
            environment_file = self._resolve_environment_file(robot_id)
            action_file = self._resolve_action_file(robot_id)
            lessons_file = self._resolve_lessons_file()
        except KeyError as exc:
            return f"Error: {exc}"

        if not embodied_file.exists():
            return f"Error: {embodied_file.name} not found for the target robot. Cannot validate action."

        embodied_content = embodied_file.read_text(encoding="utf-8")
        environment_content = environment_file.read_text(encoding="utf-8") if environment_file.exists() else ""
        critic_environment_content = self._critic_environment_scope(environment_content, robot_id)
        params_json = json.dumps(parameters, ensure_ascii=False)

        # LLM Critic often mis-reads HAL warm-up: `start` / `enter_simulation` with no
        # scene overrides is valid when the watchdog was started with --driver-config.
        if action_type in ("start", "enter_simulation") and self._start_params_deferred_to_driver(
            parameters
        ):
            logger.info(
                "Critic bypass (driver-config warm-up): action_type={} parameters={}",
                action_type,
                parameters,
            )
            return self._accept_action(action_type, dict(parameters or {}), action_file)

        # Fleet: many instances share one ENVIRONMENT.md; it reflects whichever HAL last wrote,
        # so the LLM Critic mis-reads "wrong robot" and blocks valid nav. Only skip when params
        # are plain waypoint navigation (no scene overrides).
        if EmbodiedActionTool._fleet_navigation_critic_bypass(
            action_type, parameters, robot_id, self.registry
        ):
            logger.info(
                "Critic bypass (fleet shared ENVIRONMENT): action_type={} parameters={}",
                action_type,
                parameters,
            )
            return self._accept_action(action_type, dict(parameters or {}), action_file)
        if EmbodiedActionTool._fleet_manipulation_critic_bypass(
            action_type, parameters, robot_id, self.registry
        ):
            logger.info(
                "Critic bypass (fleet shared ENVIRONMENT manipulation): action_type={} parameters={}",
                action_type,
                parameters,
            )
            return self._accept_action(action_type, dict(parameters or {}), action_file)

        critic_prompt = (
            "You are the Critic Agent for a robot.\n"
            "Your job is to validate if the proposed action is safe and physically possible "
            "based on the robot's capabilities and the current environment state.\n\n"
            "# Robot Capabilities (EMBODIED.md)\n"
            f"{embodied_content}\n\n"
            "# Current Environment State (ENVIRONMENT.md)\n"
            f"{critic_environment_content}\n\n"
            "# Proposed Action\n"
            f"Action Type: {action_type}\n"
            f"Parameters: {params_json}\n"
            f"Reasoning: {reasoning}\n\n"
            "If ENVIRONMENT contains scene session metadata (for example `scene_sessions` "
            "or per-robot `scene_id`), scope your judgement to the requested `robot_id` "
            "and its scene context instead of treating unrelated robot runtime as conflicts.\n"
            f"{self._critic_guidance(action_type)}\n"
            "If it is safe and valid, respond with exactly 'VALID'.\n"
            "If it is unsafe, out of bounds, or invalid, respond with 'INVALID: <reason>'.\n"
        )

        logger.info("Critic evaluating action: {} {}", action_type, parameters)
        response = await self.provider.chat_with_retry(
            messages=[{"role": "user", "content": critic_prompt}],
            model=self.model,
        )
        critic_result = response.content.strip()

        # Critic verdict parsing. Models are chatty and rarely emit exactly
        # "VALID" / "INVALID: <reason>" — they often append justification (e.g.
        # "VALID\n\nThe action is safe because ..."). Prior strict equality
        # check forced every such reply into the rejection branch, and the
        # rejection message then begins with the word "VALID", which confuses
        # the Planner LLM on the next turn.
        #
        # Rules (in order):
        #   1. explicit "INVALID:" anywhere -> reject (use text after it as reason)
        #   2. first non-empty line starts with "VALID" (case-insensitive) -> accept
        #   3. otherwise -> reject with the raw text as reason (conservative)
        verdict = critic_result
        upper = verdict.upper()
        if "INVALID:" in upper:
            idx = upper.index("INVALID:")
            reason = verdict[idx + len("INVALID:"):].strip() or verdict
            return self._reject_action(action_type, parameters, reasoning, reason, lessons_file)

        first_line = next(
            (line.strip() for line in verdict.splitlines() if line.strip()),
            "",
        )
        if first_line.upper().startswith("VALID"):
            return self._accept_action(action_type, parameters, action_file)

        return self._reject_action(action_type, parameters, reasoning, critic_result, lessons_file)

    def _resolve_environment_file(self, robot_id: str | None) -> Path:
        if self.registry:
            return self.registry.resolve_environment_path(robot_id=robot_id, default_workspace=self.workspace)
        return self.workspace / "ENVIRONMENT.md"

    @staticmethod
    def _extract_environment_doc(environment_content: str) -> dict[str, Any] | None:
        """
        Parse ENVIRONMENT.md content and return embedded JSON document.

        ENVIRONMENT is usually Markdown with a fenced json block; for robustness
        we also accept plain JSON payloads.
        """
        content = (environment_content or "").strip()
        if not content:
            return None
        m = _ENV_JSON_BLOCK_RE.search(content)
        if m:
            try:
                doc = json.loads(m.group(1))
                return doc if isinstance(doc, dict) else None
            except json.JSONDecodeError:
                return None
        try:
            doc = json.loads(content)
            return doc if isinstance(doc, dict) else None
        except json.JSONDecodeError:
            return None

    @staticmethod
    def _critic_environment_scope(environment_content: str, robot_id: str | None) -> str:
        """
        Build robot-scoped environment context for Critic in fleet mode.

        Why:
        - Shared ENVIRONMENT may contain mixed runtime snapshots from different
          robots/drivers. Critic should prioritize the target robot state.
        - Keep backward compatibility: if parsing fails, return original content.
        """
        if not robot_id:
            return environment_content
        doc = EmbodiedActionTool._extract_environment_doc(environment_content)
        if not doc:
            return environment_content

        robots = doc.get("robots")
        robot_state = robots.get(robot_id) if isinstance(robots, dict) else None
        scene_id = robot_state.get("scene_id") if isinstance(robot_state, dict) else None
        sessions = doc.get("scene_sessions") if isinstance(doc.get("scene_sessions"), dict) else {}
        scoped: dict[str, Any] = {
            "schema_version": doc.get("schema_version", "PhyAgentOS.environment.v1"),
            "target_robot_id": robot_id,
            "target_robot_state": robot_state if isinstance(robot_state, dict) else {},
            "target_scene_id": scene_id,
            "target_scene_session": sessions.get(scene_id, {}) if scene_id else {},
            # Keep fallback context for legacy drivers that still publish runtime
            # under objects/manipulation_runtime.
            "objects": doc.get("objects", {}),
            "scene_graph": doc.get("scene_graph", {}),
            "updated_at": doc.get("updated_at"),
        }
        try:
            return json.dumps(scoped, ensure_ascii=False, indent=2)
        except TypeError:
            return environment_content

    def _resolve_embodied_file(self, robot_id: str | None) -> Path:
        if self.registry and robot_id:
            return self.registry.resolve_embodied_path(robot_id=robot_id, default_workspace=self.workspace)
        return self.workspace / "EMBODIED.md"

    def _resolve_action_file(self, robot_id: str | None) -> Path:
        if self.registry and robot_id:
            return self.registry.resolve_action_path(robot_id=robot_id, default_workspace=self.workspace)
        return self.workspace / "ACTION.md"

    def _resolve_lessons_file(self) -> Path:
        if self.registry:
            return self.registry.resolve_lessons_path(default_workspace=self.workspace)
        return self.workspace / "LESSONS.md"

    @staticmethod
    def _fleet_broadcast_action_allowed(action_type: str, parameters: dict[str, Any]) -> bool:
        """
        Allow safe fleet-wide broadcast when user omits robot_id.

        Current scope is intentionally narrow: lifecycle actions only.
        """
        if action_type not in ("start", "enter_simulation", "close"):
            return False
        # For start/enter_simulation we only broadcast when params do not carry
        # per-robot overrides; each watchdog should rely on its own driver-config.
        if action_type in ("start", "enter_simulation"):
            return EmbodiedActionTool._start_params_deferred_to_driver(parameters)
        return True

    def _accept_action_broadcast(self, action_type: str, parameters: dict[str, Any]) -> str:
        """
        Dispatch one lifecycle action to all enabled fleet robots.

        This helps demo workflows where operators use one high-level command
        (e.g. "enter simulation") and expect all robots in the shared fleet
        scene to receive it without manually repeating robot_id.
        """
        if self.registry is None:
            return "Error: broadcast requires fleet registry."
        targets = self.registry.instances(enabled_only=True)
        if not targets:
            return "Error: no enabled robots found for fleet broadcast."
        ok: list[str] = []
        failed: list[str] = []
        for inst in targets:
            rid = inst.robot_id
            action_file = self.registry.resolve_action_path(robot_id=rid, default_workspace=self.workspace)
            params = dict(parameters or {})
            params["robot_id"] = rid
            result = self._accept_action(action_type, params, action_file)
            if result.startswith("Action '"):
                ok.append(rid)
            else:
                failed.append(f"{rid}: {result}")
        if failed:
            return (
                f"Fleet broadcast partial success for '{action_type}'. "
                f"ok={ok}; failed={failed}"
            )
        return f"Fleet broadcast dispatched '{action_type}' to robots: {ok}"

    @staticmethod
    def _accept_action(action_type: str, parameters: dict[str, Any], action_file: Path) -> str:
        """Write validated action to ACTION.md."""
        document = EmbodiedActionTool._load_action_document(action_file)
        if document is None:
            return (
                "Error: ACTION.md contains unreadable content. "
                "Please repair it before dispatching another action."
            )
        existing_action = pending_action_type(document)
        if existing_action is not None:
            return (
                f"Error: ACTION.md already contains pending action '{existing_action}'. "
                "Wait for the watchdog to consume it before dispatching another action."
            )
        action_data = append_action(document, action_type=action_type, parameters=parameters)
        action_file.parent.mkdir(parents=True, exist_ok=True)
        action_content = dump_action_document(action_data)
        action_file.write_text(action_content, encoding="utf-8")

        logger.info("Action validated and written to {}: {}", action_file, action_type)
        return f"Action '{action_type}' validated and dispatched to hardware."

    @staticmethod
    def _load_action_document(action_file: Path) -> dict[str, Any] | None:
        if not action_file.exists():
            return empty_action_document()
        content = action_file.read_text(encoding="utf-8").strip()
        if not content:
            return empty_action_document()
        payload = parse_action_markdown(content)
        if payload is None:
            return None
        return normalize_action_document(payload)

    @staticmethod
    def _fleet_navigation_critic_bypass(
        action_type: str,
        parameters: dict[str, Any],
        robot_id: str | None,
        registry: EmbodimentRegistry | None,
    ) -> bool:
        if registry is None or not registry.is_fleet or not robot_id:
            return False
        if action_type not in ("navigate_to_named", "navigate_to_waypoint"):
            return False
        allowed = frozenset(
            {"robot_id", "waypoint_key", "target", "waypoint_xy", "max_steps", "threshold"}
        )
        for key in parameters:
            if key not in allowed:
                return False
        return True

    @staticmethod
    def _fleet_manipulation_critic_bypass(
        action_type: str,
        parameters: dict[str, Any],
        robot_id: str | None,
        registry: EmbodimentRegistry | None,
    ) -> bool:
        if registry is None or not registry.is_fleet or not robot_id:
            return False
        if action_type not in ("grasp", "pick", "place", "release"):
            return False
        # Shared ENVIRONMENT in fleet mode is often written by a different robot watchdog.
        # Allow standard manipulation commands through when only safe driver-facing keys exist.
        allowed = frozenset({"robot_id", "target", "output_dir", "dump_name"})
        for key in parameters:
            if key not in allowed:
                return False
        target = parameters.get("target")
        if target is None:
            return False
        if not isinstance(target, (str, dict)):
            return False
        return True

    @staticmethod
    def _start_params_deferred_to_driver(parameters: dict[str, Any] | None) -> bool:
        """True when action does not try to override scene loaded by HAL driver-config."""
        p = parameters or {}
        override_keys = (
            "scene_asset_path",
            "objects",
            "robot_start",
            "arm_mass_scale",
            "api_kwargs",
        )
        for key in override_keys:
            val = p.get(key)
            if val is None:
                continue
            if key == "api_kwargs" and isinstance(val, dict) and len(val) == 0:
                continue
            if isinstance(val, str) and not val.strip():
                continue
            if isinstance(val, list) and len(val) == 0:
                continue
            return False
        return True

    @staticmethod
    def _critic_guidance(action_type: str) -> str:
        if action_type in ("start", "enter_simulation"):
            return (
                "For start/enter_simulation on the PiperGo2 manipulation driver: empty parameters {} "
                "are VALID when the robot profile states that the HAL watchdog loads the full "
                "driver JSON via --driver-config (scene path, objects, waypoints already injected). "
                "Do not require scene_asset_path in the action if that contract is documented in EMBODIED.md."
            )
        if action_type == "target_navigation":
            return (
                "When evaluating target navigation, do not require the target to already exist in the scene graph. "
                "Instead verify that lower-level target navigation is supported, the requested visual target or "
                "detection hint is specific enough to pursue safely, connection state allows navigation, and the "
                "current nav state suggests the robot can accept the task."
            )
        return (
            "When evaluating semantic navigation and localization actions, verify target existence, navigation "
            "support, safe approach distance, connection availability, and whether current nav state suggests the "
            "robot can accept the task."
        )

    @staticmethod
    def _reject_action(
        action_type: str,
        parameters: dict[str, Any],
        reasoning: str,
        critic_result: str,
        lessons_file: Path,
    ) -> str:
        """Record a rejected action to LESSONS.md and return an error."""
        error_msg = critic_result.replace("INVALID:", "").strip()
        params_json = json.dumps(parameters, ensure_ascii=False)

        lesson_entry = (
            "\n## Failed Action Attempt\n"
            f"- **Action**: {action_type}\n"
            f"- **Parameters**: {params_json}\n"
            f"- **Reasoning**: {reasoning}\n"
            f"- **Critic Rejection**: {error_msg}\n"
        )

        lessons_file.parent.mkdir(parents=True, exist_ok=True)
        if lessons_file.exists():
            with open(lessons_file, "a", encoding="utf-8") as fh:
                fh.write(lesson_entry)
        else:
            lessons_file.write_text("# Lessons Learned\n" + lesson_entry, encoding="utf-8")

        logger.warning("Action rejected by Critic: {}", error_msg)
        return (
            f"Error: Action rejected by Critic. Reason: {error_msg}. "
            "This failure has been recorded in LESSONS.md. "
            "Please read it and try a different approach."
        )
