"""Base agent interface for ShadowHound mission execution.

Defines the abstract contract that all agent implementations must follow.
This allows easy swapping of different AI backends (OpenAI, local LLMs, etc.)
without changing the mission execution logic.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from enum import Enum


class MissionStatus(Enum):
    """Status of mission execution."""

    SUCCESS = "success"
    FAILURE = "failure"
    PARTIAL = "partial"  # Some skills executed, some failed
    CANCELLED = "cancelled"


@dataclass
class SkillCall:
    """Represents a single skill execution in a mission plan."""

    skill_name: str
    parameters: Dict[str, Any]
    description: Optional[str] = None


@dataclass
class MissionResult:
    """Result of executing a mission.

    Attributes:
        status: Overall status of mission execution
        message: Human-readable description of outcome
        skills_executed: List of skills that were successfully executed
        skills_failed: List of skills that failed
        raw_response: Raw response from the AI agent (for debugging)
        telemetry: Additional telemetry data (execution time, token usage, etc.)
    """

    status: MissionStatus
    message: str
    skills_executed: List[SkillCall] = None
    skills_failed: List[SkillCall] = None
    raw_response: Optional[str] = None
    telemetry: Dict[str, Any] = None

    def __post_init__(self):
        """Initialize default values."""
        if self.skills_executed is None:
            self.skills_executed = []
        if self.skills_failed is None:
            self.skills_failed = []
        if self.telemetry is None:
            self.telemetry = {}

    @property
    def success(self) -> bool:
        """Check if mission was successful."""
        return self.status == MissionStatus.SUCCESS


class BaseAgent(ABC):
    """Abstract base class for all AI agents.

    This defines the contract that all agent implementations must follow.
    Agents take natural language commands and execute them using the robot's
    skill library.

    Example:
        ```python
        agent = AgentFactory.create("openai", skills=skills, robot=robot)
        result = agent.execute_mission("Go to the kitchen")

        if result.success:
            print(f"Mission completed: {result.message}")
        else:
            print(f"Mission failed: {result.message}")
        ```
    """

    def __init__(self, skills=None, robot=None, config: Optional[Dict[str, Any]] = None):
        """Initialize agent with skills and robot interface.

        Args:
            skills: Skill library instance
            robot: Robot interface instance
            config: Optional configuration dict for agent-specific settings
        """
        self.skills = skills
        self.robot = robot
        self.config = config or {}

    @abstractmethod
    def execute_mission(self, command: str) -> MissionResult:
        """Execute a mission from natural language command.

        This is the main entry point for mission execution. The agent should:
        1. Parse the natural language command
        2. Generate a plan (sequence of skill calls)
        3. Execute the skills
        4. Return structured result

        Args:
            command: Natural language command (e.g., "Go to the kitchen and take a photo")

        Returns:
            MissionResult with execution status and details

        Raises:
            AgentError: If agent encounters an error during execution
        """
        pass

    @abstractmethod
    def plan_mission(self, command: str) -> List[SkillCall]:
        """Generate mission plan without executing.

        Useful for:
        - Previewing what agent will do
        - Validating plans before execution
        - Testing agent logic

        Args:
            command: Natural language command

        Returns:
            List of SkillCall objects representing the planned actions

        Raises:
            AgentError: If agent cannot generate a valid plan
        """
        pass

    def get_available_skills(self) -> List[str]:
        """Get list of available skills.

        Returns:
            List of skill names that can be executed
        """
        if self.skills is None:
            return []

        try:
            # Try DIMOS skill library interface
            if hasattr(self.skills, "get"):
                return list(self.skills.get().keys())
            # Try dict-like interface
            elif hasattr(self.skills, "keys"):
                return list(self.skills.keys())
            else:
                return []
        except Exception:
            return []

    def validate_skill(self, skill_name: str) -> bool:
        """Check if a skill is available.

        Args:
            skill_name: Name of skill to validate

        Returns:
            True if skill exists and is available
        """
        return skill_name in self.get_available_skills()


class AgentError(Exception):
    """Base exception for agent errors."""

    pass


class AgentInitializationError(AgentError):
    """Raised when agent fails to initialize."""

    pass


class MissionExecutionError(AgentError):
    """Raised when mission execution fails."""

    pass


class PlanningError(AgentError):
    """Raised when agent cannot generate a valid plan."""

    pass
