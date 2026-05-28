from __future__ import annotations

import json
import re

from PhyAgentOS.runtime.schemas import SessionStatus
from PhyAgentOS.runtime.state_io.markdown_yaml import read_yaml_block, write_yaml_block
from PhyAgentOS.runtime.watchdog.supervisor import WatchdogSupervisor


_JSON_BLOCK_RE = re.compile(r"```json\s*\n(.*?)\n```", re.DOTALL)


def _write_workspace(workspace):
    (workspace / "configs/runtime/contracts").mkdir(parents=True, exist_ok=True)
    (workspace / "configs/runtime/contracts/dummy_sim.runtime.yaml").write_text(
        """version: runtime_target_contract_v1
target_id: dummy_sim
target_adapter: target_adapter://dummy_sim_adapter
action_contract:
  id: dummy_delta_eef_gripper_v1
  accepted_representations: [delta_eef_pose_gripper]
  shape: [T, 7]
  dtype: float32
  normalized: false
  frame: base
  control_mode: cartesian_delta_position
  control_hz: 20
  components: []
  chunk:
    max_chunk_size: 4
    preferred_chunk_size: 4
    preferred_replan_after_steps: 4
    switch_policy: hard_switch
safety:
  require_target_side_validation: true
  stop_on_nan: true
  stop_on_timeout: true
capabilities:
  available_bridges: [bridge://safety_clamp]
""",
        encoding="utf-8",
    )
    write_yaml_block(
        workspace / "TARGETS.md",
        "Runtime Targets",
        {
            "version": "runtime_target_registry_v1",
            "targets": [
                {
                    "id": "dummy_sim",
                    "target_class": "local",
                    "target_kind": "simulation",
                    "enabled": True,
                    "workspace": "workspaces/dummy_sim",
                    "supported_skills": ["openpi_sim_vla"],
                    "runtime": {
                        "target_runtime": "DummySimTargetRuntime",
                        "target_endpoint": None,
                        "target_adapter": "target_adapter://dummy_sim_adapter",
                        "runtime_contract_ref": "configs/runtime/contracts/dummy_sim.runtime.yaml",
                    },
                    "config": {
                        "success_after_steps": 3,
                        "observation": {"image_size": 16, "state_dim": 8},
                        "action": {"action_dim": 7, "chunk_size": 4},
                    },
                }
            ],
        },
    )
    write_yaml_block(
        workspace / "SKILLS.md",
        "Runtime Skills",
        {
            "version": "runtime_skill_registry_v1",
            "skills": [
                {
                    "id": "openpi_sim_vla",
                    "runtime": "OpenPISkillRuntime",
                    "runtime_kind": "policy",
                    "loop_mode": "policy_closed_loop",
                    "supported_target_kinds": ["simulation"],
                    "policy": {
                        "policy_client": "dummy",
                        "policy_adapter": "policy_adapter://dummy_openpi_adapter",
                        "supports_chunk": True,
                    },
                    "supports_chunk": True,
                    "default_replan_every": 4,
                    "output_contract": {
                        "action": {
                            "action_space_id": "dummy_policy_delta_eef_gripper_v1",
                            "shape": ["T", 7],
                            "dtype": "float32",
                            "normalized": False,
                            "representation": "delta_eef_pose_gripper",
                            "frame": "base",
                            "chunk": {"policy_hz": 20},
                        }
                    },
                    "adapter_requirements": {"allowed_bridges": ["bridge://safety_clamp"]},
                }
            ],
        },
    )
    write_yaml_block(
        workspace / "SESSIONS.md",
        "Runtime Sessions",
        {
            "version": "runtime_sessions_v1",
            "sessions": [
                {
                    "session_id": "sess_dummy_001",
                    "goal_id": "goal_smoke",
                    "target_ref": "target://dummy_sim",
                    "skill_ref": "skill://openpi_sim_vla",
                    "task_description": "move the object",
                    "status": "pending",
                    "routing": {"policy_endpoint": "dummy://local"},
                    "execution": {"max_steps": 10, "replan_every_steps": 4, "action_chunk_mode": "chunk_buffer"},
                }
            ],
        },
    )


def _read_environment_json(path):
    match = _JSON_BLOCK_RE.search(path.read_text(encoding="utf-8"))
    assert match is not None
    return json.loads(match.group(1))


def test_supervisor_single_session_succeeds(tmp_path) -> None:
    _write_workspace(tmp_path)

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    sessions = read_yaml_block(tmp_path / "SESSIONS.md")
    session = sessions["sessions"][0]
    assert session["status"] == SessionStatus.SUCCEEDED.value
    assert session["result"]["success"] is True
    assert session["result"]["artifact_dir"] == "artifacts/runtime/sess_dummy_001"

    episode_path = tmp_path / "artifacts" / "runtime" / "sess_dummy_001" / "episode.json"
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    assert episode["status"] == "succeeded"
    assert episode["num_steps"] == 3
    assert not (tmp_path / "ENVIRONMENT.md").exists()
    history = read_yaml_block(tmp_path / "LOG.md")
    assert history["last_session_id"] == "sess_dummy_001"
    assert history["last_status"] == "succeeded"
    assert history["sessions"]["sess_dummy_001"]["artifact_dir"] == "artifacts/runtime/sess_dummy_001"


def test_supervisor_rejects_disabled_target_with_preflight_diagnostic(tmp_path) -> None:
    _write_workspace(tmp_path)
    targets = read_yaml_block(tmp_path / "TARGETS.md")
    targets["targets"][0]["enabled"] = False
    write_yaml_block(tmp_path / "TARGETS.md", "Runtime Targets", targets)

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    session = read_yaml_block(tmp_path / "SESSIONS.md")["sessions"][0]
    assert session["status"] == SessionStatus.REJECTED.value
    assert "TARGET_DISABLED" in session["result"]["error_message"]
    assert "preflight" in session["result"]["metadata"]
    assert not (tmp_path / "artifacts" / "runtime" / "sess_dummy_001" / "episode.json").exists()


def test_supervisor_does_not_write_runtime_summary_to_environment(tmp_path) -> None:
    _write_workspace(tmp_path)
    (tmp_path / "ENVIRONMENT.md").write_text(
        """# Environment State

```json
{
  "schema_version": "PhyAgentOS.environment.v1",
  "scene_graph": {
    "nodes": [{"id": "obj_apple", "class": "apple"}],
    "edges": []
  },
  "objects": {
    "red_apple": {"type": "fruit", "location": "table"}
  },
  "runtime": {
    "perception_status": "fresh"
  }
}
```
""",
        encoding="utf-8",
    )

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    environment = _read_environment_json(tmp_path / "ENVIRONMENT.md")
    assert environment["scene_graph"]["nodes"][0]["id"] == "obj_apple"
    assert environment["objects"]["red_apple"]["location"] == "table"
    assert environment["runtime"] == {"perception_status": "fresh"}
    history = read_yaml_block(tmp_path / "LOG.md")
    assert history["sessions"]["sess_dummy_001"]["success"] is True


def test_supervisor_uses_priority_scheduler(tmp_path) -> None:
    _write_workspace(tmp_path)
    sessions = read_yaml_block(tmp_path / "SESSIONS.md")
    sessions["sessions"] = [
        {
            "session_id": "sess_normal",
            "target_ref": "target://dummy_sim",
            "skill_ref": "skill://openpi_sim_vla",
            "task_description": "normal priority task",
            "status": "pending",
            "priority": "normal",
            "routing": {"policy_endpoint": "dummy://local"},
            "execution": {"max_steps": 10, "replan_every_steps": 4, "action_chunk_mode": "chunk_buffer"},
        },
        {
            "session_id": "sess_high",
            "target_ref": "target://dummy_sim",
            "skill_ref": "skill://openpi_sim_vla",
            "task_description": "high priority task",
            "status": "pending",
            "priority": "high",
            "routing": {"policy_endpoint": "dummy://local"},
            "execution": {"max_steps": 10, "replan_every_steps": 4, "action_chunk_mode": "chunk_buffer"},
        },
    ]
    write_yaml_block(tmp_path / "SESSIONS.md", "Runtime Sessions", sessions)

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    updated = read_yaml_block(tmp_path / "SESSIONS.md")
    by_id = {session["session_id"]: session for session in updated["sessions"]}
    assert by_id["sess_high"]["status"] == SessionStatus.SUCCEEDED.value
    assert by_id["sess_normal"]["status"] == SessionStatus.PENDING.value
