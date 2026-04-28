from __future__ import annotations

import asyncio
import json
from pathlib import Path

from PhyAgentOS.agent.context import ContextBuilder
from PhyAgentOS.agent.tools.embodied import EmbodiedActionTool
from PhyAgentOS.config.schema import Config
from PhyAgentOS.embodiment_registry import EmbodimentRegistry


class _FakeResponse:
    def __init__(self, content: str):
        self.content = content


class _FakeProvider:
    async def chat_with_retry(self, messages, model):  # noqa: ANN001
        return _FakeResponse("VALID")


def _fleet_config(tmp_path: Path) -> Config:
    return Config.model_validate(
        {
            "embodiments": {
                "mode": "fleet",
                "sharedWorkspace": str(tmp_path / "workspaces" / "shared"),
                "instances": [
                    {
                        "robotId": "go2_edu_001",
                        "driver": "go2_edu",
                        "workspace": str(tmp_path / "workspaces" / "go2_edu_001"),
                        "enabled": True,
                    },
                    {
                        "robotId": "sim_001",
                        "driver": "simulation",
                        "workspace": str(tmp_path / "workspaces" / "sim_001"),
                        "enabled": False,
                    },
                ],
            }
        }
    )


def test_fleet_config_uses_shared_workspace_path(tmp_path: Path) -> None:
    config = _fleet_config(tmp_path)

    assert config.is_fleet_mode is True
    assert config.workspace_path == tmp_path / "workspaces" / "shared"


def test_registry_sync_layout_creates_shared_and_robot_workspaces(tmp_path: Path) -> None:
    registry = EmbodimentRegistry(_fleet_config(tmp_path))

    registry.sync_layout()

    shared = tmp_path / "workspaces" / "shared"
    robot = tmp_path / "workspaces" / "go2_edu_001"
    disabled = tmp_path / "workspaces" / "sim_001"
    assert (shared / "AGENTS.md").exists()
    assert (shared / "ENVIRONMENT.md").exists()
    assert (shared / "TASK.md").exists()
    assert (shared / "ORCHESTRATOR.md").exists()
    assert (shared / "ROBOTS.md").exists()
    assert not (shared / "ACTION.md").exists()
    assert robot.exists()
    assert (robot / "ACTION.md").exists()
    assert (robot / "EMBODIED.md").exists()
    assert not disabled.exists()


def test_context_builder_loads_robot_registry_file(tmp_path: Path) -> None:
    registry = EmbodimentRegistry(_fleet_config(tmp_path))
    registry.sync_layout()

    prompt = ContextBuilder(registry.resolve_agent_workspace()).build_system_prompt()

    assert "## ROBOTS.md" in prompt
    assert "go2_edu_001" in prompt


def test_embodied_action_tool_routes_action_to_robot_workspace(tmp_path: Path) -> None:
    registry = EmbodimentRegistry(_fleet_config(tmp_path))
    registry.sync_layout()
    shared = registry.resolve_agent_workspace()

    tool = EmbodiedActionTool(
        workspace=shared,
        provider=_FakeProvider(),
        model="fake",
        registry=registry,
    )
    result = asyncio.run(
        tool.execute(
            action_type="connect_robot",
            parameters={"robot_id": "go2_edu_001"},
            reasoning="Need to bring the robot online.",
        )
    )

    assert "validated and dispatched" in result
    assert not (shared / "ACTION.md").exists()
    action_doc = (tmp_path / "workspaces" / "go2_edu_001" / "ACTION.md").read_text(encoding="utf-8")
    assert "connect_robot" in action_doc
    assert "go2_edu_001" in action_doc


def test_embodied_action_tool_requires_robot_id_in_fleet_mode(tmp_path: Path) -> None:
    registry = EmbodimentRegistry(_fleet_config(tmp_path))
    registry.sync_layout()
    tool = EmbodiedActionTool(
        workspace=registry.resolve_agent_workspace(),
        provider=_FakeProvider(),
        model="fake",
        registry=registry,
    )

    result = asyncio.run(
        tool.execute(
            action_type="stop",
            parameters={},
            reasoning="Need to stop the active robot.",
        )
    )

    assert "robot_id is required" in result


def test_embodied_action_tool_rejects_when_action_file_already_pending(tmp_path: Path) -> None:
    registry = EmbodimentRegistry(_fleet_config(tmp_path))
    registry.sync_layout()
    shared = registry.resolve_agent_workspace()
    robot_action = tmp_path / "workspaces" / "go2_edu_001" / "ACTION.md"
    robot_action.write_text(
        "```json\n{\n  \"action_type\": \"connect_robot\",\n  \"parameters\": {\"robot_id\": \"go2_edu_001\"},\n  \"status\": \"pending\"\n}\n```\n",
        encoding="utf-8",
    )

    tool = EmbodiedActionTool(
        workspace=shared,
        provider=_FakeProvider(),
        model="fake",
        registry=registry,
    )

    result = asyncio.run(
        tool.execute(
            action_type="stop",
            parameters={"robot_id": "go2_edu_001"},
            reasoning="Need to stop the robot.",
        )
    )

    assert "already contains pending action 'connect_robot'" in result


def test_embodied_action_tool_allows_append_after_completed_history(tmp_path: Path) -> None:
    registry = EmbodimentRegistry(_fleet_config(tmp_path))
    registry.sync_layout()
    shared = registry.resolve_agent_workspace()
    robot_action = tmp_path / "workspaces" / "go2_edu_001" / "ACTION.md"
    robot_action.write_text(
        "```json\n{\n  \"schema_version\": \"PhyAgentOS.action_queue.v1\",\n  \"actions\": [\n    {\n      \"id\": \"done1\",\n      \"action_type\": \"connect_robot\",\n      \"parameters\": {\"robot_id\": \"go2_edu_001\"},\n      \"status\": \"completed\"\n    }\n  ]\n}\n```\n",
        encoding="utf-8",
    )

    tool = EmbodiedActionTool(
        workspace=shared,
        provider=_FakeProvider(),
        model="fake",
        registry=registry,
    )

    result = asyncio.run(
        tool.execute(
            action_type="stop",
            parameters={"robot_id": "go2_edu_001"},
            reasoning="Need to stop the robot.",
        )
    )

    payload = json.loads(robot_action.read_text(encoding="utf-8").split("```json\n", 1)[1].split("\n```", 1)[0])
    assert "validated and dispatched" in result
    assert len(payload["actions"]) == 2
    assert payload["actions"][1]["status"] == "pending"


def test_embodied_action_tool_rejects_invalid_action_file_content(tmp_path: Path) -> None:
    registry = EmbodimentRegistry(_fleet_config(tmp_path))
    registry.sync_layout()
    shared = registry.resolve_agent_workspace()
    robot_action = tmp_path / "workspaces" / "go2_edu_001" / "ACTION.md"
    robot_action.write_text("broken content", encoding="utf-8")

    tool = EmbodiedActionTool(
        workspace=shared,
        provider=_FakeProvider(),
        model="fake",
        registry=registry,
    )

    result = asyncio.run(
        tool.execute(
            action_type="stop",
            parameters={"robot_id": "go2_edu_001"},
            reasoning="Need to stop the robot.",
        )
    )

    assert "contains unreadable content" in result
    assert robot_action.read_text(encoding="utf-8") == "broken content"


def test_registry_render_robot_index_reads_instance_specific_environment(tmp_path: Path) -> None:
    config = Config.model_validate(
        {
            "embodiments": {
                "mode": "fleet",
                "sharedWorkspace": str(tmp_path / "workspaces" / "shared"),
                "instances": [
                    {
                        "robotId": "go2_edu_001",
                        "driver": "go2_edu",
                        "workspace": str(tmp_path / "workspaces" / "go2_edu_001"),
                        "enabled": True,
                        "sharedEnvironment": str(tmp_path / "workspaces" / "go2_edu_001" / "ENVIRONMENT.md"),
                    }
                ],
            }
        }
    )
    registry = EmbodimentRegistry(config)
    registry.sync_layout()
    robot_env = tmp_path / "workspaces" / "go2_edu_001" / "ENVIRONMENT.md"
    robot_env.write_text(
        "# Environment State\n\n```json\n{\"schema_version\":\"oea.environment.v1\",\"scene_graph\":{\"nodes\":[],\"edges\":[]},\"robots\":{\"go2_edu_001\":{\"connection_state\":{\"status\":\"connected\"},\"nav_state\":{\"status\":\"arrived\"}}},\"objects\":{}}\n```\n",
        encoding="utf-8",
    )

    content = registry.render_robot_index()

    assert "| go2_edu_001 | go2_edu |" in content
    assert "connected | arrived |" in content
