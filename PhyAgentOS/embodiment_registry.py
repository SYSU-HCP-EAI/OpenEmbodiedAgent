"""Registry helpers for shared/fleet embodiment topologies."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from PhyAgentOS.config.schema import Config, EmbodimentInstanceConfig
from PhyAgentOS.utils.helpers import ensure_dir, load_environment_doc, sync_workspace_templates

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"


@dataclass(frozen=True)
class EmbodimentInstance:
    """Resolved robot instance settings."""

    robot_id: str
    driver: str
    workspace: Path
    enabled: bool = True
    profile_name: str | None = None
    shared_environment: Path | None = None

    @property
    def profile_filename(self) -> str:
        name = self.profile_name or self.driver
        return name if name.endswith(".md") else f"{name}.md"

    @property
    def shared_environment_path(self) -> Path | None:
        return self.shared_environment


class EmbodimentRegistry:
    """Resolve embodiment instances from config and maintain runtime mirrors."""

    def __init__(self, config: Config):
        self.config = config
        self.mode = config.embodiments.mode
        self.shared_workspace = config.workspace_path
        self._instances = [self._resolve_instance(item) for item in config.embodiments.instances]

    @property
    def is_fleet(self) -> bool:
        return self.mode == "fleet"

    def instances(self, enabled_only: bool = False) -> list[EmbodimentInstance]:
        if not enabled_only:
            return list(self._instances)
        return [instance for instance in self._instances if instance.enabled]

    def get_instance(self, robot_id: str) -> EmbodimentInstance | None:
        for instance in self._instances:
            if instance.robot_id == robot_id:
                return instance
        return None

    def require_instance(self, robot_id: str) -> EmbodimentInstance:
        instance = self.get_instance(robot_id)
        if instance is None:
            raise KeyError(f"Unknown robot_id {robot_id!r}")
        return instance

    def resolve_agent_workspace(self) -> Path:
        return self.shared_workspace

    def resolve_environment_path(self, robot_id: str | None = None, default_workspace: Path | None = None) -> Path:
        if not self.is_fleet:
            return (default_workspace or self.shared_workspace) / "ENVIRONMENT.md"
        if robot_id:
            instance = self.require_instance(robot_id)
            if instance.shared_environment_path:
                return instance.shared_environment_path
        return self.shared_workspace / "ENVIRONMENT.md"

    def resolve_lessons_path(self, default_workspace: Path | None = None) -> Path:
        if self.is_fleet:
            return self.shared_workspace / "LESSONS.md"
        return (default_workspace or self.shared_workspace) / "LESSONS.md"

    def resolve_embodied_path(self, robot_id: str, default_workspace: Path | None = None) -> Path:
        if not self.is_fleet:
            return (default_workspace or self.shared_workspace) / "EMBODIED.md"
        return self.require_instance(robot_id).workspace / "EMBODIED.md"

    def resolve_action_path(self, robot_id: str, default_workspace: Path | None = None) -> Path:
        if not self.is_fleet:
            return (default_workspace or self.shared_workspace) / "ACTION.md"
        return self.require_instance(robot_id).workspace / "ACTION.md"

    def sync_layout(self) -> list[str]:
        created: list[str] = []
        if not self.is_fleet:
            sync_workspace_templates(self.shared_workspace)
            return created

        ensure_dir(self.shared_workspace)
        created.extend(sync_workspace_templates(
            self.shared_workspace,
            exclude={"ACTION.md", "EMBODIED.md"},
        ))

        for instance in self.instances(enabled_only=True):
            ensure_dir(instance.workspace)
            action_path = instance.workspace / "ACTION.md"
            if not action_path.exists():
                action_path.write_text("", encoding="utf-8")
                created.append(str(action_path))
            profile_path = self._profile_path_for(instance)
            if profile_path and profile_path.exists():
                embodied_path = instance.workspace / "EMBODIED.md"
                if not embodied_path.exists():
                    embodied_path.write_text(profile_path.read_text(encoding="utf-8"), encoding="utf-8")
                    created.append(str(embodied_path))

        self.write_robot_index()
        return created

    def write_robot_index(self) -> Path:
        path = self.shared_workspace / "ROBOTS.md"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.render_robot_index(), encoding="utf-8")
        return path

    def render_robot_index(self) -> str:
        environment_cache: dict[Path, dict] = {}
        lines = [
            "# Robot Registry",
            "",
            "Auto-generated from config and current shared runtime state.",
            "Use this file as a concise fleet directory: what robots exist, where they run, and what they are roughly good at.",
            "",
            "| Robot ID | Driver | Type | Capabilities | Workspace | Enabled | Profile | Connection | Navigation |",
            "|---|---|---|---|---|---|---|---|---|",
        ]
        for instance in self.instances():
            env_path = self.resolve_environment_path(robot_id=instance.robot_id)
            if env_path not in environment_cache:
                environment_cache[env_path] = load_environment_doc(env_path)
            env = environment_cache[env_path]
            robot_state = env.get("robots", {}) if isinstance(env, dict) else {}
            runtime = robot_state.get(instance.robot_id, {})
            connection = (runtime.get("connection_state") or {}).get("status", "unknown")
            navigation = (runtime.get("nav_state") or {}).get("status", "unknown")
            summary = self._profile_summary(instance)
            lines.append(
                f"| {instance.robot_id} | {instance.driver} | {summary['type']} | {summary['capabilities']} | "
                f"`{instance.workspace}` | {'yes' if instance.enabled else 'no'} | "
                f"{instance.profile_filename} | {connection} | {navigation} |"
            )
        if len(lines) == 6:
            lines.append("| — | — | — | — | — | — | — | — | — |")
        return "\n".join(lines) + "\n"

    @classmethod
    def from_config(cls, config: Config | None) -> EmbodimentRegistry | None:
        if config is None:
            return None
        return cls(config)

    def _resolve_instance(self, item: EmbodimentInstanceConfig) -> EmbodimentInstance:
        shared_env = Path(item.shared_environment).expanduser() if item.shared_environment else None
        return EmbodimentInstance(
            robot_id=item.robot_id,
            driver=item.driver,
            workspace=Path(item.workspace).expanduser(),
            enabled=item.enabled,
            profile_name=item.profile_name,
            shared_environment=shared_env,
        )

    @staticmethod
    def _profile_path_for(instance: EmbodimentInstance) -> Path | None:
        candidate = _PROFILES_DIR / instance.profile_filename
        return candidate if candidate.exists() else None

    def _profile_summary(self, instance: EmbodimentInstance) -> dict[str, str]:
        profile_path = self._profile_path_for(instance)
        if profile_path is None or not profile_path.exists():
            return {"type": "unknown", "capabilities": "unknown"}

        content = profile_path.read_text(encoding="utf-8")
        robot_type = self._extract_bullet_value(content, "Type") or "unknown"
        actions = self._extract_supported_actions(content)
        capabilities = ", ".join(actions[:4]) if actions else "see profile"
        if len(actions) > 4:
            capabilities += ", ..."
        return {"type": robot_type, "capabilities": capabilities}

    @staticmethod
    def _extract_bullet_value(content: str, label: str) -> str | None:
        pattern = rf"- \*\*{re.escape(label)}\*\*: (.+)"
        match = re.search(pattern, content)
        return match.group(1).strip() if match else None

    @staticmethod
    def _extract_supported_actions(content: str) -> list[str]:
        actions: list[str] = []
        in_actions = False
        for line in content.splitlines():
            if line.startswith("## Supported Actions"):
                in_actions = True
                continue
            if in_actions and line.startswith("## "):
                break
            if not in_actions or "`" not in line:
                continue
            match = re.search(r"`([^`]+)`", line)
            if match:
                actions.append(match.group(1))
        return actions
