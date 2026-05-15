from __future__ import annotations

import json
import re
from pathlib import Path

import pytest
import yaml

from PhyAgentOS.runtime.perception import PerceptionPreflightError
from PhyAgentOS.runtime.perception.config_resolver import PerceptionConfigResolver
from PhyAgentOS.runtime.perception.environment_writer import EnvironmentWriter
from PhyAgentOS.runtime.schemas import SkillSpec, TargetSpec
from PhyAgentOS.runtime.schemas.perception import EnvironmentDelta, EnvironmentObject, EnvironmentObjectSource
from PhyAgentOS.runtime.state_io.markdown_yaml import read_yaml_block, write_yaml_block
from PhyAgentOS.runtime.watchdog.scheduler import ScheduledSession
from PhyAgentOS.runtime.watchdog.supervisor import WatchdogSupervisor


_JSON_BLOCK_RE = re.compile(r"```json\s*\n(.*?)\n```", re.DOTALL)


def _read_environment(path: Path) -> dict:
    match = _JSON_BLOCK_RE.search(path.read_text(encoding="utf-8"))
    assert match is not None
    return json.loads(match.group(1))


def _write_yaml(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _sensor_config(target_id: str = "dummy_sim") -> dict:
    return {
        "version": "runtime_sensor_config_v1",
        "target_id": target_id,
        "sensors": [
            {
                "id": "front_rgb",
                "modality": "rgb",
                "role": "primary_scene_camera",
                "source": "dummy/image",
                "observation_key": "image",
                "frame_id": "camera",
                "enabled": True,
                "resolution": [16, 16],
                "required_fields": ["source", "observation_key", "frame_id", "resolution"],
            },
            {
                "id": "proprio",
                "modality": "proprio",
                "role": "robot_state",
                "source": "dummy/state",
                "observation_key": "state",
                "enabled": True,
            },
        ],
        "observation_schema": {
            "image": {"dtype": "uint8", "shape": [16, 16, 3]},
            "state": {"dtype": "float32", "shape": [8]},
        },
    }


def _front_depth_sensor(*, enabled: bool = True) -> dict:
    return {
        "id": "front_depth",
        "modality": "depth",
        "role": "aligned_depth",
        "source": "dummy/depth",
        "observation_key": "depth",
        "frame_id": "camera",
        "enabled": enabled,
        "resolution": [16, 16],
        "required_fields": ["source", "observation_key", "frame_id", "resolution"],
    }


def _perception_config(target_id: str = "dummy_sim") -> dict:
    return {
        "version": "runtime_perception_config_v1",
        "target_id": target_id,
        "strict_preflight": True,
        "models": [
            {
                "id": "dummy_builtin",
                "type": "dummy",
                "provider": "builtin_dummy",
                "install_hint": "No install needed.",
            }
        ],
        "plugin_candidates": [
            {
                "id": "dummy_segmenter",
                "type": "segmenter_2d",
                "model_ref": "dummy_builtin",
                "requires_sensors": ["front_rgb"],
                "produces": ["boxes_2d", "labels", "confidence"],
                "config": {"object_id": "obj_cube", "label": "cube"},
            },
            {
                "id": "rgbd_object_builder",
                "type": "object_3d_builder",
                "model_ref": "dummy_builtin",
                "requires_plugins": ["dummy_segmenter"],
                "requires_outputs": ["boxes_2d"],
                "requires_sensors": ["front_rgb"],
                "produces": ["objects_3d", "scene_graph"],
            },
        ],
        "pipelines": [
            {
                "id": "dummy_object_environment",
                "required_outputs": ["objects_3d", "scene_graph"],
                "use_plugins": ["dummy_segmenter", "rgbd_object_builder"],
            }
        ],
    }


def _scheduled(
    tmp_path: Path,
    *,
    outputs: list[str] | None = None,
    sensors: list[str] | None = None,
    sensor_config: dict | None = None,
    perception_config: dict | None = None,
):
    _write_yaml(tmp_path / "configs/runtime/sensors/dummy_sim.sensors.yaml", sensor_config or _sensor_config())
    _write_yaml(
        tmp_path / "configs/runtime/perception/dummy_sim.perception.yaml",
        perception_config or _perception_config(),
    )
    target = TargetSpec.model_validate(
        {
            "id": "dummy_sim",
            "type": "sim",
            "backend": "dummy",
            "enabled": True,
            "workspace": "workspaces/dummy_sim",
            "supported_skills": ["perception_skill"],
            "adapter": "dummy_openpi_adapter",
            "perception": {
                "enabled": True,
                "sensor_config_ref": "configs/runtime/sensors/dummy_sim.sensors.yaml",
                "perception_config_ref": "configs/runtime/perception/dummy_sim.perception.yaml",
                "artifact_dir": "artifacts/perception/dummy_sim",
            },
            "config": {"observation": {"image_size": 16, "state_dim": 8}},
        }
    )
    skill = SkillSpec.model_validate(
        {
            "id": "perception_skill",
            "category": "debug",
            "runtime": "OpenPISimSkillRuntime",
            "supported_target_types": ["sim"],
            "requires": {
                "sensors": sensors or ["front_rgb", "proprio"],
                "environment_outputs": outputs or [],
                "strict_environment_contract": True,
            },
        }
    )
    session = {
        "session_id": "sess_perception",
        "target_ref": "target://dummy_sim",
        "skill_ref": "skill://perception_skill",
        "task_description": "inspect",
        "routing": {"policy_endpoint": "dummy://local", "adapter": "dummy_openpi_adapter"},
    }
    from PhyAgentOS.runtime.schemas import SessionSpec

    return ScheduledSession(
        session=SessionSpec.model_validate(session),
        target_spec=target,
        skill_spec=skill,
        target_id=target.id,
        skill_id=skill.id,
    )


def test_perception_schema_defaults_validate() -> None:
    target = TargetSpec.model_validate(
        {
            "id": "dummy_sim",
            "type": "sim",
            "workspace": "workspaces/dummy_sim",
            "supported_skills": [],
            "adapter": "dummy_openpi_adapter",
        }
    )
    skill = SkillSpec.model_validate(
        {
            "id": "noop",
            "category": "debug",
            "runtime": "NoopRuntime",
            "supported_target_types": ["sim"],
        }
    )

    assert target.perception.enabled is False
    assert skill.requires.sensors == []
    assert skill.requires.environment_outputs == []


def test_resolver_and_preflight_accept_dependency_free_dummy_config(tmp_path: Path) -> None:
    scheduled = _scheduled(tmp_path, outputs=["objects_3d", "scene_graph"])
    resolver = PerceptionConfigResolver(tmp_path)

    plan = resolver.resolve(scheduled)
    assert plan is not None
    assert plan.selected_pipeline_id == "dummy_object_environment"
    assert plan.selected_plugins == ["dummy_segmenter", "rgbd_object_builder"]


def test_preflight_rejects_missing_required_sensor(tmp_path: Path) -> None:
    scheduled = _scheduled(tmp_path, sensors=["front_depth"])

    from PhyAgentOS.runtime.perception.preflight import PerceptionPreflightChecker

    resolver = PerceptionConfigResolver(tmp_path)
    plan = resolver.resolve(scheduled)
    assert plan is not None
    with pytest.raises(PerceptionPreflightError, match="sensor_config_missing"):
        PerceptionPreflightChecker(tmp_path, resolver).check(plan)


def test_resolver_includes_selected_plugin_required_sensors(tmp_path: Path) -> None:
    perception_config = _perception_config()
    perception_config["plugin_candidates"][1]["requires_sensors"] = ["front_rgb", "front_depth"]

    plan = PerceptionConfigResolver(tmp_path).resolve(
        _scheduled(
            tmp_path,
            outputs=["objects_3d", "scene_graph"],
            sensors=["front_rgb"],
            perception_config=perception_config,
        )
    )

    assert plan is not None
    assert plan.required_sensors == ["front_rgb", "front_depth"]


def test_environment_writer_converts_legacy_doc_to_clean_v2(tmp_path: Path) -> None:
    (tmp_path / "ENVIRONMENT.md").write_text(
        """# Environment State

```json
{"schema_version":"PhyAgentOS.environment.v1","runtime":{"old":true},"robots":{"r1":{}},"objects":{"old":{}}}
```
""",
        encoding="utf-8",
    )
    delta = EnvironmentDelta(
        objects={
            "obj_cube": EnvironmentObject(
                label="cube",
                confidence=0.9,
                source=EnvironmentObjectSource(target_id="dummy_sim", session_id="sess"),
            )
        },
        generated_outputs=["objects_3d"],
        refresh_scope=["obj_cube"],
    )

    EnvironmentWriter(tmp_path).write(
        target_id="dummy_sim",
        session_id="sess",
        run_id="perc_1",
        sensor_config_ref="configs/runtime/sensors/dummy_sim.sensors.yaml",
        perception_config_ref="configs/runtime/perception/dummy_sim.perception.yaml",
        pipeline_id="dummy",
        pipeline=["dummy_segmenter"],
        requested_outputs=["objects_3d"],
        artifact_dir="artifacts/perception/dummy_sim/sess",
        delta=delta,
    )

    env = _read_environment(tmp_path / "ENVIRONMENT.md")
    assert env["schema_version"] == "PhyAgentOS.environment.v2"
    assert set(env) == {
        "schema_version",
        "updated_at",
        "targets",
        "objects",
        "scene_graph",
        "perception",
        "map",
        "tf",
    }
    assert "runtime" not in env
    assert "robots" not in env
    assert env["objects"]["dummy_sim::obj_cube"]["label"] == "cube"
    run = env["perception"]["runs"]["perc_1"]
    assert run["refresh_scope"]["target_id"] == "dummy_sim"
    assert run["refresh_scope"]["object_ids"] == ["obj_cube"]


def test_environment_writer_merges_duplicate_objects_by_global_position(tmp_path: Path) -> None:
    writer = EnvironmentWriter(tmp_path)
    first = EnvironmentDelta(
        objects={
            "local_cube_a": EnvironmentObject(
                label="cube",
                confidence=0.9,
                source=EnvironmentObjectSource(target_id="target_a", session_id="sess_a"),
                pose={"frame_id": "map", "position_m": [1.0, 2.0, 0.0]},
            )
        },
        generated_outputs=["objects_3d"],
        refresh_scope=["local_cube_a"],
    )
    writer.write(
        target_id="target_a",
        session_id="sess_a",
        run_id="perc_a",
        sensor_config_ref="a.sensors.yaml",
        perception_config_ref="a.perception.yaml",
        pipeline_id="dummy",
        pipeline=["dummy_segmenter"],
        requested_outputs=["objects_3d"],
        artifact_dir="artifacts/perception/target_a/sess_a",
        delta=first,
    )
    second = EnvironmentDelta(
        objects={
            "local_cube_b": EnvironmentObject(
                label="cube",
                confidence=0.95,
                source=EnvironmentObjectSource(target_id="target_b", session_id="sess_b"),
                pose={"frame_id": "map", "position_m": [1.1, 2.0, 0.0]},
            )
        },
        generated_outputs=["objects_3d"],
        refresh_scope=["local_cube_b"],
    )
    writer.write(
        target_id="target_b",
        session_id="sess_b",
        run_id="perc_b",
        sensor_config_ref="b.sensors.yaml",
        perception_config_ref="b.perception.yaml",
        pipeline_id="dummy",
        pipeline=["dummy_segmenter"],
        requested_outputs=["objects_3d"],
        artifact_dir="artifacts/perception/target_b/sess_b",
        delta=second,
    )

    env = _read_environment(tmp_path / "ENVIRONMENT.md")
    assert list(env["objects"]) == ["target_a::local_cube_a"]
    obj = env["objects"]["target_a::local_cube_a"]
    assert obj["source"]["target_id"] == "target_b"
    assert obj["source"]["local_object_id"] == "local_cube_b"


def test_supervisor_rejects_perception_preflight_before_episode(tmp_path: Path) -> None:
    _write_workspace(tmp_path, sensors=["front_depth"], outputs=[])

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    session = read_yaml_block(tmp_path / "SESSIONS.md")["sessions"][0]
    assert session["status"] == "rejected"
    assert "sensor_config_missing" in session["result"]["error_message"]
    assert not (tmp_path / "artifacts" / "runtime" / "sess_perception" / "episode.json").exists()


def test_supervisor_rejects_disabled_plugin_required_sensor_before_episode(tmp_path: Path) -> None:
    sensor_config = _sensor_config()
    sensor_config["sensors"].append(_front_depth_sensor(enabled=False))
    sensor_config["observation_schema"]["depth"] = {"dtype": "float32", "shape": [16, 16]}
    perception_config = _perception_config()
    perception_config["plugin_candidates"][1]["requires_sensors"] = ["front_rgb", "front_depth"]
    _write_workspace(
        tmp_path,
        sensors=["front_rgb"],
        outputs=["objects_3d", "scene_graph"],
        sensor_config=sensor_config,
        perception_config=perception_config,
    )

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    session = read_yaml_block(tmp_path / "SESSIONS.md")["sessions"][0]
    assert session["status"] == "rejected"
    assert "sensor_disabled" in session["result"]["error_message"]
    assert not (tmp_path / "ENVIRONMENT.md").exists()
    assert not (tmp_path / "artifacts" / "runtime" / "sess_perception" / "episode.json").exists()


def test_supervisor_rejects_missing_plugin_required_observation_before_episode(tmp_path: Path) -> None:
    sensor_config = _sensor_config()
    sensor_config["sensors"].append(_front_depth_sensor())
    sensor_config["observation_schema"]["depth"] = {"dtype": "float32", "shape": [16, 16]}
    perception_config = _perception_config()
    perception_config["plugin_candidates"][1]["requires_sensors"] = ["front_rgb", "front_depth"]
    _write_workspace(
        tmp_path,
        sensors=["front_rgb"],
        outputs=["objects_3d", "scene_graph"],
        sensor_config=sensor_config,
        perception_config=perception_config,
    )

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    session = read_yaml_block(tmp_path / "SESSIONS.md")["sessions"][0]
    assert session["status"] == "rejected"
    assert "observation channel missing for sensor front_depth" in session["result"]["error_message"]
    assert not (tmp_path / "ENVIRONMENT.md").exists()
    assert not (tmp_path / "artifacts" / "runtime" / "sess_perception" / "episode.json").exists()


def test_supervisor_rejects_pipeline_that_does_not_generate_requested_outputs(tmp_path: Path) -> None:
    perception_config = _perception_config()
    perception_config["pipelines"][0]["use_plugins"] = ["dummy_segmenter"]
    _write_workspace(
        tmp_path,
        sensors=["front_rgb"],
        outputs=["objects_3d", "scene_graph"],
        perception_config=perception_config,
    )

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    session = read_yaml_block(tmp_path / "SESSIONS.md")["sessions"][0]
    assert session["status"] == "rejected"
    assert "did not generate required outputs" in session["result"]["error_message"]
    assert "objects_3d" in session["result"]["error_message"]
    assert not (tmp_path / "ENVIRONMENT.md").exists()
    assert not (tmp_path / "artifacts" / "runtime" / "sess_perception" / "episode.json").exists()


def test_supervisor_writes_environment_v2_for_environment_output_skill(tmp_path: Path) -> None:
    _write_workspace(tmp_path, sensors=["front_rgb", "proprio"], outputs=["objects_3d", "scene_graph"])

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    session = read_yaml_block(tmp_path / "SESSIONS.md")["sessions"][0]
    assert session["status"] == "succeeded"
    env = _read_environment(tmp_path / "ENVIRONMENT.md")
    assert env["schema_version"] == "PhyAgentOS.environment.v2"
    assert "runtime" not in env
    assert "robots" not in env
    assert env["perception"]["runs"]
    assert env["objects"]["dummy_sim::obj_cube"]["pose"]["frame_id"] == "base_link"


def test_supervisor_writes_environment_to_shared_workspace(tmp_path: Path) -> None:
    runtime_workspace = tmp_path / "robot_runtime"
    shared_workspace = tmp_path / "shared"
    runtime_workspace.mkdir()
    shared_workspace.mkdir()
    _write_workspace(runtime_workspace, sensors=["front_rgb", "proprio"], outputs=["objects_3d", "scene_graph"])

    assert WatchdogSupervisor(
        runtime_workspace,
        worker_id="test-worker",
        environment_workspace=shared_workspace,
    ).run_once() is True

    assert not (runtime_workspace / "ENVIRONMENT.md").exists()
    env = _read_environment(shared_workspace / "ENVIRONMENT.md")
    assert env["schema_version"] == "PhyAgentOS.environment.v2"
    assert "dummy_sim::obj_cube" in env["objects"]


def _write_workspace(
    tmp_path: Path,
    *,
    sensors: list[str],
    outputs: list[str],
    sensor_config: dict | None = None,
    perception_config: dict | None = None,
) -> None:
    _write_yaml(tmp_path / "configs/runtime/sensors/dummy_sim.sensors.yaml", sensor_config or _sensor_config())
    _write_yaml(
        tmp_path / "configs/runtime/perception/dummy_sim.perception.yaml",
        perception_config or _perception_config(),
    )
    write_yaml_block(
        tmp_path / "TARGETS.md",
        "Runtime Targets",
        {
            "version": "runtime_target_registry_v1",
            "targets": [
                {
                    "id": "dummy_sim",
                    "type": "sim",
                    "backend": "dummy",
                    "enabled": True,
                    "workspace": "workspaces/dummy_sim",
                    "supported_skills": ["perception_skill"],
                    "adapter": "dummy_openpi_adapter",
                    "perception": {
                        "enabled": True,
                        "sensor_config_ref": "configs/runtime/sensors/dummy_sim.sensors.yaml",
                        "perception_config_ref": "configs/runtime/perception/dummy_sim.perception.yaml",
                        "artifact_dir": "artifacts/perception/dummy_sim",
                    },
                    "config": {
                        "success_after_steps": 1,
                        "observation": {"image_size": 16, "state_dim": 8},
                        "action": {"action_dim": 7, "chunk_size": 4},
                    },
                }
            ],
        },
    )
    write_yaml_block(
        tmp_path / "SKILLS.md",
        "Runtime Skills",
        {
            "version": "runtime_skill_registry_v1",
            "skills": [
                {
                    "id": "perception_skill",
                    "category": "debug",
                    "runtime": "OpenPISimSkillRuntime",
                    "supported_target_types": ["sim"],
                    "requires": {
                        "sensors": sensors,
                        "environment_outputs": outputs,
                        "strict_environment_contract": True,
                    },
                }
            ],
        },
    )
    write_yaml_block(
        tmp_path / "SESSIONS.md",
        "Runtime Sessions",
        {
            "version": "runtime_sessions_v1",
            "sessions": [
                {
                    "session_id": "sess_perception",
                    "target_ref": "target://dummy_sim",
                    "skill_ref": "skill://perception_skill",
                    "task_description": "inspect",
                    "routing": {"policy_endpoint": "dummy://local", "adapter": "dummy_openpi_adapter"},
                    "execution": {"max_steps": 5, "replan_every": 1, "action_chunk_mode": "open_loop"},
                }
            ],
        },
    )
