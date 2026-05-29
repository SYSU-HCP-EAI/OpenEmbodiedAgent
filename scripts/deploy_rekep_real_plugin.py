#!/usr/bin/env python3
"""ReKep real-robot plugin has been migrated to the Session-Centered Runtime.

The old plugin-based HAL driver (hal/drivers/, hal_watchdog.py) has been replaced
by the new TargetAdapter + ActionBridge architecture.

To deploy a ReKep-enabled real-robot target:

1. Define the target in TARGETS.md with target_kind: real_robot
2. Create a FrankaTargetAdapter (or equivalent) in PhyAgentOS/runtime/adapters/
3. Create a ReKepBuiltinSkillRuntime in PhyAgentOS/runtime/skills/builtin/
4. Configure sensor/perception YAML in configs/runtime/
5. Run: python -m PhyAgentOS.runtime.watchdog

See docs/user_development_guide/ for detailed instructions.
"""

from __future__ import annotations

import sys


def main() -> None:
    print(__doc__)
    sys.exit(1)


if __name__ == "__main__":
    main()
