from __future__ import annotations

import json
import re

from PhyAgentOS.runtime.schemas import SessionStatus
from PhyAgentOS.runtime.state_io.markdown_yaml import read_yaml_block, write_yaml_block
from PhyAgentOS.runtime.watchdog.supervisor import WatchdogSupervisor


_JSON_BLOCK_RE = re.compile(r"```json\s*\n(.*?)\n```", re.DOTALL)


def _write_workspace(workspace):
    write_yaml_block(
        workspace / "TARGETS.md",
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
                    "supported_skills": ["openpi_sim_vla"],
                    "adapter": "dummy_openpi_adapter",
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
                    "category": "vla",
                    "runtime": "OpenPISimSkillRuntime",
                    "supported_target_types": ["sim"],
                    "policy_client": "dummy",
                    "supports_chunk": True,
                    "default_replan_every": 4,
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
                    "routing": {"policy_endpoint": "dummy://local", "adapter": "dummy_openpi_adapter"},
                    "execution": {"max_steps": 10, "replan_every": 4, "action_chunk_mode": "open_loop"},
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
    assert (tmp_path / "ENVIRONMENT.md").exists()


def test_supervisor_merges_environment_runtime_summary(tmp_path) -> None:
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
    assert environment["runtime"]["perception_status"] == "fresh"
    assert environment["runtime"]["last_session_id"] == "sess_dummy_001"
    assert environment["runtime"]["last_status"] == "succeeded"
    assert environment["runtime"]["sessions"]["sess_dummy_001"]["success"] is True
