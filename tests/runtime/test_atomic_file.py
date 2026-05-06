from __future__ import annotations

from PhyAgentOS.runtime.state_io.atomic_file import atomic_write_text


def test_atomic_write_text_creates_and_replaces(tmp_path) -> None:
    path = tmp_path / "state.md"

    atomic_write_text(path, "one")
    atomic_write_text(path, "two")

    assert path.read_text(encoding="utf-8") == "two"
    assert not list(tmp_path.glob("*.tmp"))
