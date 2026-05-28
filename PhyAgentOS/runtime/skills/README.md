# Runtime Skills

Runtime skills are organized by execution mode.

- `policy/` contains `PolicySkillRuntime` implementations. These runtimes own policy-driven loops and access targets only through `TargetSessionHandle`.
- `builtin/` contains `BuiltinSkillRuntime` implementations. These runtimes own builtin or agent-interactive loops and expose only validated target tools.
- `base.py` contains the shared `BaseSkillRuntime` lifecycle contract.

Skill registry entries in `SKILLS.md` select a concrete runtime by name and declare `runtime_kind` as `policy` or `builtin`.
