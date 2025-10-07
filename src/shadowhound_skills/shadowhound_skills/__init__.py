"""ShadowHound Skills Package - Vision and perception capabilities."""

from .vision import (
    SnapshotSkill,
    DescribeSceneSkill,
    LocateObjectSkill,
    DetectObjectsSkill,
    SkillResult,
    get_skill,
    list_skills,
    VISION_SKILLS,
)

__all__ = [
    "SnapshotSkill",
    "DescribeSceneSkill",
    "LocateObjectSkill",
    "DetectObjectsSkill",
    "SkillResult",
    "get_skill",
    "list_skills",
    "VISION_SKILLS",
]

__version__ = "0.1.0"
