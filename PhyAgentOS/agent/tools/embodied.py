"""Embodied action tool for executing robot actions with Critic validation."""

from __future__ import annotations

import json
import logging
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


class EmbodiedActionTool(Tool):
    """Validate embodied actions and route them to the correct robot workspace."""

    @property
    def name(self) -> str:
        return "execute_robot_action"

    @property
    def description(self) -> str:
        return (
            "Execute a physical action on the robot. "
            "The action will be validated by a Critic before execution."
        )

    @property
    def parameters(self) -> dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "action_type": {
                    "type": "string",
                    "description": (
                        "The type of action to execute "
                        "(e.g., 'point_to', 'move_to', 'pick_up', "
                        "'semantic_navigate', 'localize', 'connect_robot')."
                    ),
                },
                "parameters": {
                    "type": "object",
                    "description": "The parameters for the action. Include robot_id in fleet mode.",
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

    async def execute(
        self,
        action_type: str,
        parameters: dict[str, Any],
        reasoning: str,
    ) -> str:
        """Execute the action after Critic validation."""
        robot_id = parameters.get("robot_id")
        if self.registry and self.registry.is_fleet and not robot_id:
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
        params_json = json.dumps(parameters, ensure_ascii=False)

        critic_prompt = (
            "You are the Critic Agent for a robot.\n"
            "Your job is to validate if the proposed action is safe and physically possible "
            "based on the robot's capabilities and the current environment state.\n\n"
            "# Robot Capabilities (EMBODIED.md)\n"
            f"{embodied_content}\n\n"
            "# Current Environment State (ENVIRONMENT.md)\n"
            f"{environment_content}\n\n"
            "# Proposed Action\n"
            f"Action Type: {action_type}\n"
            f"Parameters: {params_json}\n"
            f"Reasoning: {reasoning}\n\n"
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
    def _critic_guidance(action_type: str) -> str:
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
