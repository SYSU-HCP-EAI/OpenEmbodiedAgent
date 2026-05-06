#!/usr/bin/env python
"""Run the PhyAgentOS runtime v2 watchdog supervisor."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PhyAgentOS.runtime.watchdog.supervisor import WatchdogSupervisor


def main() -> int:
    parser = argparse.ArgumentParser(description="Run PhyAgentOS runtime v2 watchdog")
    parser.add_argument("--workspace", required=True, help="Workspace containing TARGETS/SKILLS/SESSIONS.md")
    parser.add_argument("--once", action="store_true", help="Run one polling pass and exit")
    args = parser.parse_args()

    supervisor = WatchdogSupervisor(args.workspace)
    if args.once:
        return 0 if supervisor.run_once() else 1

    while True:
        supervisor.run_once()


if __name__ == "__main__":
    raise SystemExit(main())
