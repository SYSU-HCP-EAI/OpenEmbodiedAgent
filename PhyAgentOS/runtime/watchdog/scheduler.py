"""Session scheduling for the serial runtime watchdog."""

from __future__ import annotations

from dataclasses import dataclass

from PhyAgentOS.runtime.schemas import (
    SessionsDocument,
    SessionSpec,
    SessionStatus,
    SkillSpec,
    SkillsDocument,
    TargetSpec,
    TargetsDocument,
)
from PhyAgentOS.runtime.schemas.common import strip_ref
from PhyAgentOS.runtime.watchdog.errors import SchemaValidationError


@dataclass(frozen=True)
class ScheduledSession:
    session: SessionSpec
    target_spec: TargetSpec
    skill_spec: SkillSpec
    target_id: str
    skill_id: str


class SessionScheduleError(SchemaValidationError):
    """Raised when a specific pending session cannot be scheduled."""

    def __init__(self, session_id: str, message: str):
        super().__init__(message)
        self.session_id = session_id


class SessionScheduler:
    """Pick the next executable pending session for a serial worker."""

    _PRIORITY_RANK = {"high": 0, "normal": 1, "low": 2}

    def select_next(
        self,
        sessions_doc: SessionsDocument,
        targets_doc: TargetsDocument,
        skills_doc: SkillsDocument,
    ) -> ScheduledSession | None:
        pending = [
            (idx, session)
            for idx, session in enumerate(sessions_doc.sessions)
            if session.status == SessionStatus.PENDING
        ]
        if not pending:
            return None
        _, session = min(
            pending,
            key=lambda item: (self._PRIORITY_RANK.get(item[1].priority, 1), item[0]),
        )
        try:
            return self.resolve_session(session, targets_doc, skills_doc)
        except SchemaValidationError as exc:
            raise SessionScheduleError(session.session_id, str(exc)) from exc

    def resolve_session(
        self,
        session: SessionSpec,
        targets_doc: TargetsDocument,
        skills_doc: SkillsDocument,
    ) -> ScheduledSession:
        target_id = strip_ref(session.target_ref, "target://")
        skill_id = strip_ref(session.skill_ref, "skill://")
        target_spec = self._find_target(targets_doc, target_id)
        skill_spec = self._find_skill(skills_doc, skill_id)
        if skill_id not in target_spec.supported_skills:
            raise SchemaValidationError(f"target {target_id} does not support skill {skill_id}")
        if target_spec.target_kind not in skill_spec.supported_target_kinds:
            raise SchemaValidationError(
                f"skill {skill_id} does not support target kind {target_spec.target_kind}"
            )
        return ScheduledSession(
            session=session,
            target_spec=target_spec,
            skill_spec=skill_spec,
            target_id=target_id,
            skill_id=skill_id,
        )

    def _find_target(self, document: TargetsDocument, target_id: str) -> TargetSpec:
        for target in document.targets:
            if target.id == target_id:
                return target
        raise SchemaValidationError(f"target not found: {target_id}")

    def _find_skill(self, document: SkillsDocument, skill_id: str) -> SkillSpec:
        for skill in document.skills:
            if skill.id == skill_id:
                return skill
        raise SchemaValidationError(f"skill not found: {skill_id}")
