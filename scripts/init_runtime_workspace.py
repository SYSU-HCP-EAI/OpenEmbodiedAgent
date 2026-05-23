#!/usr/bin/env python
"""Initialize Runtime v2 protocol files in a workspace."""

from __future__ import annotations

import argparse
import sys
from importlib.resources import files as pkg_files
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

RUNTIME_TEMPLATE_NAMES = ("TARGETS.md", "SKILLS.md", "SESSIONS.md")
RUNTIME_CONFIG_TEMPLATE_NAMES = (
    "configs/runtime/sensors/dummy_sim.sensors.yaml",
    "configs/runtime/perception/dummy_sim.perception.yaml",
    "configs/runtime/contracts/dummy_sim.runtime.yaml",
)


def init_runtime_workspace(workspace: Path, force: bool = False) -> dict[str, list[str]]:
    """Create or refresh Runtime v2 protocol files."""
    workspace = workspace.expanduser()
    workspace.mkdir(parents=True, exist_ok=True)
    templates = pkg_files("PhyAgentOS") / "templates"

    result: dict[str, list[str]] = {"created": [], "skipped": [], "overwritten": []}
    for name in RUNTIME_TEMPLATE_NAMES:
        src = templates / name
        dest = workspace / name
        if dest.exists() and not force:
            result["skipped"].append(name)
            continue

        dest.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
        result["overwritten" if dest.exists() and force else "created"].append(name)
    for name in RUNTIME_CONFIG_TEMPLATE_NAMES:
        src = templates.joinpath(*Path(name).parts)
        dest = workspace / name
        if dest.exists() and not force:
            result["skipped"].append(name)
            continue
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
        result["overwritten" if dest.exists() and force else "created"].append(name)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Initialize PhyAgentOS Runtime v2 workspace files")
    parser.add_argument("--workspace", required=True, help="Workspace directory to initialize")
    parser.add_argument("--force", action="store_true", help="Overwrite existing runtime protocol files")
    args = parser.parse_args()

    result = init_runtime_workspace(Path(args.workspace), force=args.force)
    for key in ("created", "overwritten", "skipped"):
        for name in result[key]:
            print(f"{key}: {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
