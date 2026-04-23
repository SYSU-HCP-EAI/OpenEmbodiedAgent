# PhyAgentOS Skills

This directory contains built-in skills that extend PhyAgentOS's capabilities.

## Skill Format

Each skill is a directory containing a `SKILL.md` file with:
- YAML frontmatter (name, description, metadata)
- Markdown instructions for the agent

## Attribution

These skills are adapted from [OpenClaw](https://github.com/openclaw/openclaw)'s skill system.
The skill format and metadata structure follow OpenClaw's conventions to maintain compatibility.

## Available Skills

| Skill | Description |
|-------|-------------|
| `github` | Interact with GitHub using the `gh` CLI |
| `weather` | Get weather info using wttr.in and Open-Meteo |
| `summarize` | Summarize URLs, files, and YouTube videos |
| `tmux` | Remote-control tmux sessions |
| `clawhub` | Search and install skills from ClawHub registry |
| `skill-creator` | Create new skills |
| `robot-management-guideline` | Must-read manual to manage robot projects |
| `rekep-robot-onboarding` | Inspect a robot SDK and wire it into the external ReKep plugin |
| `pipergo2-demo` | Deterministic demo mapping for go-to-desk and red-cube pick then return to spawn |
