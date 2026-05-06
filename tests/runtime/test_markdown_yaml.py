from __future__ import annotations

from PhyAgentOS.runtime.state_io.markdown_yaml import read_yaml_block, write_yaml_block


def test_markdown_yaml_round_trip(tmp_path) -> None:
    path = tmp_path / "SESSIONS.md"
    data = {
        "version": "runtime_sessions_v1",
        "sessions": [{"session_id": "sess_1", "status": "pending"}],
    }

    write_yaml_block(path, "Runtime Sessions", data)

    assert read_yaml_block(path) == data
    assert path.read_text(encoding="utf-8").startswith("# Runtime Sessions")


def test_reads_first_yaml_block(tmp_path) -> None:
    path = tmp_path / "TARGETS.md"
    path.write_text("# T\n\n~~~yaml\nversion: runtime_target_registry_v1\ntargets: []\n~~~\n", encoding="utf-8")

    assert read_yaml_block(path)["version"] == "runtime_target_registry_v1"
