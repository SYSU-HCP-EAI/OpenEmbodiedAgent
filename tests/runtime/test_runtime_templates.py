from __future__ import annotations

import json
import subprocess
import sys
from importlib.resources import files as pkg_files
from pathlib import Path

from PhyAgentOS.runtime.schemas import SessionsDocument, SkillsDocument, TargetsDocument
from PhyAgentOS.runtime.state_io.markdown_yaml import read_yaml_block
from PhyAgentOS.runtime.watchdog.supervisor import WatchdogSupervisor


RUNTIME_TEMPLATE_NAMES = ("TARGETS.md", "SKILLS.md", "SESSIONS.md")


def test_runtime_templates_exist_and_validate() -> None:
    templates = pkg_files("PhyAgentOS") / "templates"

    targets = read_yaml_block(Path(str(templates / "TARGETS.md")))
    skills = read_yaml_block(Path(str(templates / "SKILLS.md")))
    sessions = read_yaml_block(Path(str(templates / "SESSIONS.md")))

    assert TargetsDocument.model_validate(targets).targets[0].id == "dummy_sim"
    assert SkillsDocument.model_validate(skills).skills[0].id == "openpi_sim_vla"
    assert SessionsDocument.model_validate(sessions).sessions[0].status == "pending"


def test_init_runtime_workspace_script_preserves_existing_files(tmp_path) -> None:
    script = Path("scripts/init_runtime_workspace.py")

    subprocess.run(
        [sys.executable, str(script), "--workspace", str(tmp_path)],
        check=True,
        cwd=Path(__file__).resolve().parents[2],
    )

    for name in RUNTIME_TEMPLATE_NAMES:
        assert (tmp_path / name).exists()

    sentinel = "# custom sessions\n"
    (tmp_path / "SESSIONS.md").write_text(sentinel, encoding="utf-8")
    subprocess.run(
        [sys.executable, str(script), "--workspace", str(tmp_path)],
        check=True,
        cwd=Path(__file__).resolve().parents[2],
    )
    assert (tmp_path / "SESSIONS.md").read_text(encoding="utf-8") == sentinel


def test_init_runtime_workspace_script_force_overwrites(tmp_path) -> None:
    script = Path("scripts/init_runtime_workspace.py")
    (tmp_path / "SESSIONS.md").write_text("# custom sessions\n", encoding="utf-8")

    subprocess.run(
        [sys.executable, str(script), "--workspace", str(tmp_path), "--force"],
        check=True,
        cwd=Path(__file__).resolve().parents[2],
    )

    sessions = SessionsDocument.model_validate(read_yaml_block(tmp_path / "SESSIONS.md"))
    assert sessions.sessions[0].session_id == "sess_dummy_smoke"


def test_init_runtime_workspace_supervisor_smoke(tmp_path) -> None:
    script = Path("scripts/init_runtime_workspace.py")
    subprocess.run(
        [sys.executable, str(script), "--workspace", str(tmp_path)],
        check=True,
        cwd=Path(__file__).resolve().parents[2],
    )

    assert WatchdogSupervisor(tmp_path, worker_id="test-worker").run_once() is True

    sessions = SessionsDocument.model_validate(read_yaml_block(tmp_path / "SESSIONS.md"))
    assert sessions.sessions[0].status == "succeeded"

    episode_path = tmp_path / "artifacts" / "runtime" / "sess_dummy_smoke" / "episode.json"
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    assert episode["success"] is True
