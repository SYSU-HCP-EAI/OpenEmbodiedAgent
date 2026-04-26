from __future__ import annotations

import asyncio
from pathlib import Path

from PhyAgentOS.agent.tools.embodied_question import EmbodiedQuestionTool
from PhyAgentOS.config.schema import Config
from PhyAgentOS.embodiment_registry import EmbodimentRegistry
from hal.drivers.pipergo2_manipulation_driver import PiperGo2ManipulationDriver
from hal.simulation.scene_io import save_environment_doc


def _write_environment(
    workspace: Path,
    *,
    scene_graph: dict | None = None,
    robots: dict | None = None,
    objects: dict | None = None,
) -> None:
    save_environment_doc(
        workspace / "ENVIRONMENT.md",
        {
            "schema_version": "PhyAgentOS.environment.v1",
            "scene_graph": scene_graph or {"nodes": [], "edges": []},
            "robots": robots or {},
            "objects": objects or {},
        },
    )


def _fleet_config(tmp_path: Path) -> Config:
    shared_env = tmp_path / "workspaces" / "shared" / "ENVIRONMENT.md"
    return Config.model_validate(
        {
            "embodiments": {
                "mode": "fleet",
                "sharedWorkspace": str(tmp_path / "workspaces" / "shared"),
                "instances": [
                    {
                        "robotId": "pipergo2_manip_001",
                        "driver": "pipergo2_manipulation",
                        "workspace": str(tmp_path / "workspaces" / "pipergo2_manip_001"),
                        "enabled": True,
                        "sharedEnvironment": str(shared_env),
                    }
                ],
            }
        }
    )


def test_embodied_question_answers_from_scene_graph(tmp_path: Path) -> None:
    _write_environment(
        tmp_path,
        scene_graph={
            "nodes": [
                {"id": "surface_table", "class": "table", "object_key": "staging_table", "role": "surface"},
                {"id": "obj_pick_cube", "class": "cube", "object_key": "pick_cube", "color_label_cn": "red", "role": "primary_pick"},
                {"id": "obj_nearby_block_tall", "class": "cube", "object_key": "nearby_block_tall", "color_label_cn": "blue", "role": "clutter"},
            ],
            "edges": [
                {"source": "obj_pick_cube", "relation": "ON", "target": "surface_table", "confidence": 0.9},
                {"source": "obj_nearby_block_tall", "relation": "ON", "target": "surface_table", "confidence": 0.9},
            ],
        },
    )

    tool = EmbodiedQuestionTool(workspace=tmp_path)
    result = asyncio.run(tool.execute(question="what is on the table"))

    assert "red cube" in result.lower()
    assert "blue cube" in result.lower()


def test_embodied_question_handles_is_there_anything_on_table_in_front_of_you(tmp_path: Path) -> None:
    _write_environment(
        tmp_path,
        scene_graph={
            "nodes": [
                {"id": "surface_table", "class": "table", "object_key": "staging_table", "role": "surface"},
            ],
            "edges": [],
        },
    )

    tool = EmbodiedQuestionTool(workspace=tmp_path)
    result = asyncio.run(tool.execute(question="is there anything on the table in front of you?"))

    assert "i don't see anything" in result.lower()


def test_embodied_question_answers_generic_visible_scene_question(tmp_path: Path) -> None:
    _write_environment(
        tmp_path,
        scene_graph={
            "nodes": [
                {"id": "surface_table", "class": "table", "object_key": "staging_table", "role": "surface"},
                {"id": "obj_pick_cube", "class": "cube", "object_key": "pick_cube", "color_label_cn": "red", "role": "primary_pick"},
                {"id": "obj_nearby_block_tall", "class": "cube", "object_key": "nearby_block_tall", "color_label_cn": "blue", "role": "clutter"},
                {"id": "obj_nearby_block_flat", "class": "cube", "object_key": "nearby_block_flat", "color_label_cn": "yellow", "role": "clutter"},
                {"id": "obj_nearby_block_small", "class": "cube", "object_key": "nearby_block_small", "color_label_cn": "green", "role": "clutter"},
            ],
            "edges": [
                {"source": "obj_pick_cube", "relation": "ON", "target": "surface_table", "confidence": 0.9},
                {"source": "obj_nearby_block_tall", "relation": "ON", "target": "surface_table", "confidence": 0.9},
                {"source": "obj_nearby_block_flat", "relation": "ON", "target": "surface_table", "confidence": 0.9},
                {"source": "obj_nearby_block_small", "relation": "ON", "target": "surface_table", "confidence": 0.9},
            ],
        },
    )

    tool = EmbodiedQuestionTool(workspace=tmp_path)
    result = asyncio.run(tool.execute(question="你的视角里有什么"))

    assert "red cube" in result.lower()
    assert "blue cube" in result.lower()
    assert "yellow cube" in result.lower()
    assert "green cube" in result.lower()


def test_embodied_question_falls_back_to_runtime_summary(tmp_path: Path) -> None:
    _write_environment(
        tmp_path,
        objects={
            "manipulation_runtime": {
                "table_summary_cn": "桌上有一个红色方块和一个蓝色方块。"
            }
        },
    )

    tool = EmbodiedQuestionTool(workspace=tmp_path)
    result = asyncio.run(tool.execute(question="what is on the table"))

    assert "桌上有一个红色方块" in result


def test_embodied_question_requires_robot_id_in_fleet_mode(tmp_path: Path) -> None:
    registry = EmbodimentRegistry(_fleet_config(tmp_path))
    registry.sync_layout()

    tool = EmbodiedQuestionTool(workspace=registry.resolve_agent_workspace(), registry=registry)
    result = asyncio.run(tool.execute(question="what is on the table"))

    assert "robot_id is required" in result


def test_embodied_question_uses_shared_environment_for_robot(tmp_path: Path) -> None:
    registry = EmbodimentRegistry(_fleet_config(tmp_path))
    registry.sync_layout()
    shared_env = tmp_path / "workspaces" / "shared"
    _write_environment(
        shared_env,
        scene_graph={
            "nodes": [
                {"id": "surface_table", "class": "table", "object_key": "staging_table", "role": "surface"},
                {"id": "obj_pick_cube", "class": "cube", "object_key": "pick_cube", "color_label_cn": "red", "role": "primary_pick"},
            ],
            "edges": [
                {"source": "obj_pick_cube", "relation": "ON", "target": "surface_table", "confidence": 0.9},
            ],
        },
    )

    tool = EmbodiedQuestionTool(workspace=registry.resolve_agent_workspace(), registry=registry)
    result = asyncio.run(tool.execute(question="what is on the table", robot_id="pipergo2_manip_001"))

    assert "red cube" in result.lower()


def test_pipergo2_driver_exposes_robot_scoped_runtime_state() -> None:
    driver = PiperGo2ManipulationDriver(
        gui=False,
        robot_id="arm_a",
        robot_start=[0.58, 7.50704, 0.55],
        visible_objects=[
            {"object_key": "pick_cube", "shape_cn": "cube", "color_label_cn": "red", "role": "primary_pick"},
            {"object_key": "nearby_block_tall", "shape_cn": "cube", "color_label_cn": "blue", "role": "clutter"},
        ],
        objects=[
            {"name": "pick_cube", "position": [3.46, 7.67, 0.41]},
            {"name": "nearby_block_tall", "position": [3.58, 7.73, 0.41]},
        ],
    )
    driver._api = object()
    driver._last_obs = {"position": [1.25, 2.5, 0.55]}

    runtime = driver.get_runtime_state()

    assert runtime["robots"]["arm_a"]["connection_state"]["status"] == "connected"
    assert runtime["robots"]["arm_a"]["robot_pose"]["x"] == 1.25
    assert any(node["id"] == "surface_table" for node in runtime["scene_graph"]["nodes"])
    assert any(edge["relation"] == "ON" and edge["target"] == "surface_table" for edge in runtime["scene_graph"]["edges"])


def test_pipergo2_driver_reports_robot_camera_mounts_in_runtime_state() -> None:
    driver = PiperGo2ManipulationDriver(
        gui=False,
        robot_id="arm_a",
        robot_cameras={
            "dog_view": {
                "parent": "/World/env_0/robots/pipergo2/Head_upper",
                "translation": [0.3, 0.0, 0.33],
                "orientation": [0.9813, 0.0, 0.192, 0.0],
                "purpose": "robot_body_view",
            },
        },
        robot_camera_resolution=[640, 400],
    )

    runtime = driver.get_runtime_state()
    cameras = runtime["robots"]["arm_a"]["cameras"]
    scene = driver.get_scene()

    assert cameras["dog_view"]["status"] == "configured"
    assert cameras["dog_view"]["parent"] == "/World/env_0/robots/pipergo2/Head_upper"
    assert cameras["dog_view"]["prim_path"] == "/World/env_0/robots/pipergo2/Head_upper/paos_dog_view"
    assert scene["manipulation_runtime"]["robot_cameras"] == cameras


def test_pipergo2_driver_uses_live_object_positions_for_scene_and_table_edges() -> None:
    driver = PiperGo2ManipulationDriver(
        gui=False,
        visible_objects=[
            {"object_key": "pick_cube", "shape_cn": "cube", "color_label_cn": "red", "role": "primary_pick"},
            {"object_key": "nearby_block_tall", "shape_cn": "cube", "color_label_cn": "blue", "role": "clutter"},
            {"object_key": "place_pedestal", "shape_cn": "pedestal", "color_label_cn": "gray", "role": "place_surface"},
        ],
        objects=[
            {"kind": "dynamic_cube", "name": "pick_cube", "prim_path": "/World/pick_cube", "position": [3.46, 7.67, 0.41]},
            {"kind": "dynamic_cube", "name": "nearby_block_tall", "prim_path": "/World/nearby_block_tall", "position": [3.58, 7.73, 0.41]},
            {"kind": "dynamic_cube", "name": "place_pedestal", "prim_path": "/World/place_pedestal", "position": [1.02, 7.08, 0.16]},
        ],
    )

    class _Obj:
        def __init__(self, position):
            self._position = position

        def get_pose(self):
            return self._position, (1.0, 0.0, 0.0, 0.0)

    class _Runner:
        def __init__(self):
            self._objects = {
                "pick_cube": _Obj((1.08, 7.10, 0.35)),
                "nearby_block_tall": _Obj((3.58, 7.73, 0.41)),
                "place_pedestal": _Obj((1.02, 7.08, 0.16)),
            }

        def get_obj(self, name: str):
            return self._objects[name]

    class _Env:
        def __init__(self):
            self.runner = _Runner()

    driver._env = _Env()

    runtime = driver.get_runtime_state()
    scene = driver.get_scene()
    narration = driver._describe_visible_scene({})

    on_table_sources = {edge["source"] for edge in runtime["scene_graph"]["edges"] if edge["target"] == "surface_table"}
    assert "obj_pick_cube" not in on_table_sources
    assert "obj_nearby_block_tall" in on_table_sources
    assert scene["pick_cube"]["location"] == "pedestal"
    assert scene["pick_cube"]["position"]["x"] == 1.08
    assert "桌面上现在有" in narration
    assert "red cube@(1.08, 7.10, 0.35)[pedestal]" in narration

    def test_pipergo2_driver_drops_stale_objects_not_in_current_config() -> None:
        driver = PiperGo2ManipulationDriver(gui=False, visible_objects=[], objects=[])
        driver.load_scene(
            {
                "pick_cube": {
                    "type": "dynamic_cube",
                    "location": "table",
                },
                "nearby_block_tall": {
                    "type": "dynamic_cube",
                    "location": "table",
                },
            }
        )

        scene = driver.get_scene()
        runtime = driver.get_runtime_state()

        assert "pick_cube" not in scene
        assert "nearby_block_tall" not in scene
        assert runtime["scene_graph"] == {
            "nodes": [
                {
                    "id": "surface_table",
                    "class": "table",
                    "object_key": "staging_table",
                    "role": "surface",
                }
            ],
            "edges": [],
        }


def test_pipergo2_rule_pick_reports_target_not_on_table() -> None:
    driver = PiperGo2ManipulationDriver(
        gui=False,
        visible_objects=[
            {"object_key": "pick_cube", "shape_cn": "cube", "color_label_cn": "red", "role": "primary_pick"},
            {"object_key": "place_pedestal", "shape_cn": "pedestal", "color_label_cn": "gray", "role": "place_surface"},
        ],
        objects=[
            {"kind": "dynamic_cube", "name": "pick_cube", "prim_path": "/World/pick_cube", "position": [3.46, 7.67, 0.41]},
            {"kind": "dynamic_cube", "name": "place_pedestal", "prim_path": "/World/place_pedestal", "position": [1.02, 7.08, 0.16]},
        ],
        pick_place={
            "pick_target": {
                "position": [3.46, 7.67, 0.41],
                "pre_position": [3.46, 7.67, 0.49],
                "post_position": [3.28, 7.62, 0.59],
                "metadata": {"object_name": "pick_cube"},
            }
        },
    )

    class _Obj:
        def __init__(self, position):
            self._position = position

        def get_pose(self):
            return self._position, (1.0, 0.0, 0.0, 0.0)

    class _Runner:
        def get_obj(self, name: str):
            mapping = {
                "pick_cube": _Obj((1.08, 7.10, 0.35)),
                "place_pedestal": _Obj((1.02, 7.08, 0.16)),
            }
            return mapping[name]

    class _Env:
        def __init__(self):
            self.runner = _Runner()

    driver._env = _Env()
    driver._api = object()

    result = driver._run_pick_place({})

    assert "not on the table anymore" in result
    assert "current_location=pedestal" in result


def test_pipergo2_vla_pick_reports_target_not_on_table() -> None:
    driver = PiperGo2ManipulationDriver(
        gui=False,
        visible_objects=[
            {"object_key": "pick_cube", "shape_cn": "cube", "color_label_cn": "red", "role": "primary_pick"},
            {"object_key": "place_pedestal", "shape_cn": "pedestal", "color_label_cn": "gray", "role": "place_surface"},
        ],
        objects=[
            {"kind": "dynamic_cube", "name": "pick_cube", "prim_path": "/World/pick_cube", "position": [3.46, 7.67, 0.41]},
            {"kind": "dynamic_cube", "name": "place_pedestal", "prim_path": "/World/place_pedestal", "position": [1.02, 7.08, 0.16]},
        ],
        vla={
            "pick_target_prim_path": "/World/pick_cube",
        },
    )

    class _Obj:
        def __init__(self, position):
            self._position = position

        def get_pose(self):
            return self._position, (1.0, 0.0, 0.0, 0.0)

    class _Runner:
        def get_obj(self, name: str):
            mapping = {
                "pick_cube": _Obj((1.08, 7.10, 0.35)),
                "place_pedestal": _Obj((1.02, 7.08, 0.16)),
            }
            return mapping[name]

    class _Env:
        def __init__(self):
            self.runner = _Runner()

    driver._env = _Env()
    driver._api = object()

    result = driver._run_vla_pick_and_return({})

    assert "not on the table anymore" in result
    assert "current_location=pedestal" in result


def test_pipergo2_driver_ignores_implausible_visual_cube_pose() -> None:
    driver = PiperGo2ManipulationDriver(
        gui=False,
        visible_objects=[
            {"object_key": "nearby_block_flat", "shape_cn": "cube", "color_label_cn": "yellow", "role": "clutter"},
            {"object_key": "nearby_block_small", "shape_cn": "cube", "color_label_cn": "green", "role": "clutter"},
        ],
        objects=[
            {"kind": "visual_cube", "name": "nearby_block_flat", "prim_path": "/World/nearby_block_flat", "position": [3.402, 7.744, 0.41172]},
            {"kind": "visual_cube", "name": "nearby_block_small", "prim_path": "/World/nearby_block_small", "position": [3.56, 7.594, 0.41172]},
        ],
    )

    class _Obj:
        def __init__(self, position):
            self._position = position

        def get_pose(self):
            return self._position, (1.0, 0.0, 0.0, 0.0)

    class _Runner:
        def get_obj(self, name: str):
            mapping = {
                "nearby_block_flat": _Obj((3.402, 7.744, -592.81)),
                "nearby_block_small": _Obj((3.56, 7.594, -592.81)),
            }
            return mapping[name]

    class _Env:
        def __init__(self):
            self.runner = _Runner()

    driver._env = _Env()

    scene = driver.get_scene()
    runtime = driver.get_runtime_state()

    assert scene["nearby_block_flat"]["location"] == "table"
    assert scene["nearby_block_small"]["location"] == "table"
    assert scene["nearby_block_flat"]["position"]["z"] == 0.41172
    on_table_sources = {edge["source"] for edge in runtime["scene_graph"]["edges"] if edge["target"] == "surface_table"}
    assert "obj_nearby_block_flat" in on_table_sources
    assert "obj_nearby_block_small" in on_table_sources