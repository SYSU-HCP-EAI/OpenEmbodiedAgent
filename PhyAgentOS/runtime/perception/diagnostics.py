"""Perception diagnostics and actionable rejection messages."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class PreflightIssue:
    code: str
    field: str
    expected: str
    fix: str

    def format(self, index: int) -> str:
        return (
            f"{index}. {self.code}\n"
            f"   Field: {self.field}\n"
            f"   Expected: {self.expected}\n"
            f"   Fix: {self.fix}"
        )


@dataclass
class PreflightResult:
    session_id: str
    issues: list[PreflightIssue] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.issues

    def add(self, code: str, field: str, expected: str, fix: str) -> None:
        self.issues.append(PreflightIssue(code=code, field=field, expected=expected, fix=fix))

    def summary(self) -> str:
        if self.ok:
            return "perception preflight passed"
        items = "\n\n".join(issue.format(idx) for idx, issue in enumerate(self.issues, start=1))
        return (
            f"Session {self.session_id} rejected before running.\n"
            "Reason: required perception preflight failed.\n\n"
            f"Missing items:\n{items}\n\n"
            "No session execution was started."
        )

