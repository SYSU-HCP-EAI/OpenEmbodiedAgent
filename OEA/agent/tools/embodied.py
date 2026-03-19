"""Embodied action tool for executing robot actions with Critic validation."""

import json
from pathlib import Path
from typing import Any

from loguru import logger

from OEA.agent.tools.base import Tool
from OEA.providers.base import LLMProvider

# Markdown fences kept outside f-strings to avoid backtick parse issues
_FENCE_OPEN = "```json"
_FENCE_CLOSE = "```"


class EmbodiedActionTool(Tool):
    """
    Tool for executing robot actions.

    This tool acts as the Critic in the Dual-Track Multi-Agent System.
    It intercepts the action draft from the Planner, validates it against
    the physical limits defined in EMBODIED.md, and only writes to ACTION.md
    if the validation passes.
    """

    name = "execute_robot_action"
    description = (
        "Execute a physical action on the robot. "
        "The action will be validated by a Critic before execution."
    )

    def __init__(self, workspace: Path, provider: LLMProvider, model: str):
        self.workspace = workspace
        self.provider = provider
        self.model = model

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
                        "(e.g., 'point_to', 'move_to', 'pick_up')."
                    ),
                },
                "parameters": {
                    "type": "object",
                    "description": (
                        "The parameters for the action "
                        "(e.g., {'x': 10, 'y': 20, 'z': 0})."
                    ),
                },
                "reasoning": {
                    "type": "string",
                    "description": "The reasoning behind choosing this action.",
                },
            },
            "required": ["action_type", "parameters", "reasoning"],
    def to_schema(self) -> dict[str, Any]:
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": {
                        "action_type": {
                            "type": "string",
                            "description": (
                                "The type of action to execute "
                                "(e.g., 'point_to', 'move_to', 'pick_up')."
                            ),
                        },
                        "parameters": {
                            "type": "object",
                            "description": (
                                "The parameters for the action "
                                "(e.g., {'x': 10, 'y': 20, 'z': 0})."
                            ),
                        },
                        "reasoning": {
                            "type": "string",
                            "description": "The reasoning behind choosing this action.",
                        },
                    },
                    "required": ["action_type", "parameters", "reasoning"],
                },
            },
        }

    async def execute(
        self,
        action_type: str,
        parameters: dict[str, Any],
        reasoning: str,
    ) -> str:
        """Execute the action after Critic validation."""
        embodied_file = self.workspace / "EMBODIED.md"
        action_file = self.workspace / "ACTION.md"
        lessons_file = self.workspace / "LESSONS.md"

        if not embodied_file.exists():
            return "Error: EMBODIED.md not found. Cannot validate action."

        embodied_content = embodied_file.read_text(encoding="utf-8")
        params_json = json.dumps(parameters)

        # Construct the Critic prompt
        critic_prompt = (
            "You are the Critic Agent for a robot.\n"
            "Your job is to validate if the proposed action is safe and "
            "physically possible based on the robot's capabilities.\n\n"
            "# Robot Capabilities (EMBODIED.md)\n"
            f"{embodied_content}\n\n"
            "# Proposed Action\n"
            f"Action Type: {action_type}\n"
            f"Parameters: {params_json}\n"
            f"Reasoning: {reasoning}\n\n"
            "Evaluate the action. If it is safe and valid, respond with exactly 'VALID'.\n"
            "If it is unsafe, out of bounds, or invalid, respond with 'INVALID: <reason>'.\n"
        )

        logger.info("Critic evaluating action: {} {}", action_type, parameters)

        # Call the LLM to act as the Critic
        response = await self.provider.chat_with_retry(
            messages=[{"role": "user", "content": critic_prompt}],
            model=self.model,
        )

        critic_result = response.content.strip()

        if critic_result == "VALID":
            return self._accept_action(action_type, parameters, action_file)
        else:
            return self._reject_action(
                action_type, parameters, reasoning, critic_result, lessons_file,
            )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _accept_action(
        action_type: str,
        parameters: dict[str, Any],
        action_file: Path,
    ) -> str:
        """Write validated action to ACTION.md."""
        action_data = {
            "action_type": action_type,
            "parameters": parameters,
            "status": "pending",
        }
        action_content = (
            _FENCE_OPEN + "\n"
            + json.dumps(action_data, indent=2) + "\n"
            + _FENCE_CLOSE + "\n"
        )
        action_file.write_text(action_content, encoding="utf-8")

        logger.info("Action validated and written to ACTION.md: {}", action_type)
        return f"Action '{action_type}' validated and dispatched to hardware."

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
        params_json = json.dumps(parameters)

        lesson_entry = (
            "\n## Failed Action Attempt\n"
            f"- **Action**: {action_type}\n"
            f"- **Parameters**: {params_json}\n"
            f"- **Reasoning**: {reasoning}\n"
            f"- **Critic Rejection**: {error_msg}\n"
        )

        if lessons_file.exists():
            with open(lessons_file, "a", encoding="utf-8") as fh:
                fh.write(lesson_entry)
        else:
            lessons_file.write_text(
                "# Lessons Learned\n" + lesson_entry, encoding="utf-8",
            )

        logger.warning("Action rejected by Critic: {}", error_msg)
        return (
            f"Error: Action rejected by Critic. Reason: {error_msg}. "
            "This failure has been recorded in LESSONS.md. "
            "Please read it and try a different approach."
        )
