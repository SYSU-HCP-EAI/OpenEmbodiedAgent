from __future__ import annotations

import asyncio
import json
from pathlib import Path

from PhyAgentOS.agent.tools.embodied import EmbodiedActionTool
from PhyAgentOS.agent.tools.scene_graph import SceneGraphQueryTool
from PhyAgentOS.agent.tools.semantic_navigation import SemanticNavigationTool
from hal.perception.environment_writer import EnvironmentWriter
from hal.simulation.scene_io import load_environment_doc, save_environment_doc


class _FakeResponse:
    def __init__(self, content: str):
        self.content = content


class _FakeProvider:
    async def chat_with_retry(self, messages, model):  # noqa: ANN001
        return _FakeResponse("VALID")


def _write_workspace_files(workspace: Path) -> None:
    (workspace / "EMBODIED.md").write_text(
        "# Embodied\n\n- Supports semantic navigation.\n",
        encoding="utf-8",
    )
    (workspace / "LESSONS.md").write_text("# Lessons\n", encoding="utf-8")
    save_environment_doc(
        workspace / "ENVIRONMENT.md",
        {
            "schema_version": "PhyAgentOS.environment.v1",
            "scene_graph": {
                "nodes": [
                    {
                        "id": "fridge_1",
                        "class": "fridge",
                        "center": {"x": 2.0, "y": 1.0, "z": 0.0, "frame": "map"},
                        "size": {"x": 0.8, "y": 0.8, "z": 1.8},
                        "frame": "map",
                    },
                    {
                        "id": "fridge_2",
                        "class": "fridge",
                        "center": {"x": 6.0, "y": 1.0, "z": 0.0, "frame": "map"},
                        "size": {"x": 0.8, "y": 0.8, "z": 1.8},
                        "frame": "map",
                    },
                ],
                "edges": [],
            },
            "robots": {
                "go2_edu_001": {
                    "robot_pose": {
                        "frame": "map",
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "yaw": 0.0,
                        "stamp": "2026-03-18T00:00:00Z",
                    },
                    "nav_state": {"mode": "idle", "status": "idle", "recovery_count": 0},
                }
            },
            "map": {
                "frame": "map",
                "zones": [
                    {
                        "name": "kitchen",
                        "center": {"x": 3.0, "y": 2.0, "z": 0.0},
                        "size": {"x": 1.0, "y": 1.0, "z": 2.0},
                    }
                ],
            },
            "objects": {},
        },
    )


def test_scene_graph_query_find_by_class(tmp_path: Path) -> None:
    _write_workspace_files(tmp_path)
    tool = SceneGraphQueryTool(workspace=tmp_path)
    result = asyncio.run(tool.execute(query_type="find_by_class", target_class="fridge"))
    payload = json.loads(result)
    assert len(payload["matches"]) == 2


def test_scene_graph_query_nearest_to_robot_orders_matches(tmp_path: Path) -> None:
    _write_workspace_files(tmp_path)
    tool = SceneGraphQueryTool(workspace=tmp_path)
    result = asyncio.run(tool.execute(query_type="nearest_to_robot", robot_id="go2_edu_001"))
    payload = json.loads(result)
    assert payload["matches"][0]["id"] == "fridge_1"


def test_semantic_navigation_writes_action_md(tmp_path: Path) -> None:
    _write_workspace_files(tmp_path)
    action_tool = EmbodiedActionTool(workspace=tmp_path, provider=_FakeProvider(), model="fake")
    tool = SemanticNavigationTool(workspace=tmp_path, action_tool=action_tool)

    result = asyncio.run(
        tool.execute(
            robot_id="go2_edu_001",
            target_class="fridge",
            reasoning="Need to inspect the fridge area.",
        )
    )

    assert "validated and dispatched" in result
    action_doc = (tmp_path / "ACTION.md").read_text(encoding="utf-8")
    assert "semantic_navigate" in action_doc
    assert "fridge_1" in action_doc


def test_semantic_navigation_reports_missing_target(tmp_path: Path) -> None:
    _write_workspace_files(tmp_path)
    action_tool = EmbodiedActionTool(workspace=tmp_path, provider=_FakeProvider(), model="fake")
    tool = SemanticNavigationTool(workspace=tmp_path, action_tool=action_tool)

    result = asyncio.run(
        tool.execute(
            robot_id="go2_edu_001",
            target_class="sofa",
            reasoning="Need to inspect the sofa area.",
        )
    )

    assert "target class 'sofa' not found" in result


def test_environment_writer_preserves_existing_objects(tmp_path: Path) -> None:
    _write_workspace_files(tmp_path)
    writer = EnvironmentWriter(workspace=tmp_path)
    updated = writer.write(
        robot_id="go2_edu_001",
        nav_state={"mode": "navigating", "status": "running", "recovery_count": 0},
        scene_graph={"nodes": [], "edges": []},
        tf_data={"map_to_odom": {"available": True}},
    )
    assert "objects" in updated
    doc = load_environment_doc(tmp_path / "ENVIRONMENT.md")
    assert doc["robots"]["go2_edu_001"]["nav_state"]["status"] == "running"
