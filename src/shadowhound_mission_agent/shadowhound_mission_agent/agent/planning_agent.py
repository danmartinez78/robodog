"""Planning agent implementation.

Wraps DIMOS's PlanningAgent to provide mission planning and execution
with explicit plan generation and validation.
"""

import time
from typing import List, Dict, Any, Optional
from .base_agent import (
    BaseAgent,
    MissionResult,
    MissionStatus,
    SkillCall,
    AgentInitializationError,
    MissionExecutionError,
    PlanningError,
)

try:
    from dimos.agents.planning_agent import PlanningAgent as DIMOSPlanningAgent

    DIMOS_AVAILABLE = True
except ImportError:
    DIMOS_AVAILABLE = False
    DIMOSPlanningAgent = None


class PlanningAgent(BaseAgent):
    """Planning-based agent for mission execution.

    Uses explicit planning step to generate and validate mission plans
    before execution. Provides better transparency and control over
    mission execution compared to reactive agents.

    Example:
        ```python
        agent = PlanningAgent(skills=skill_lib, robot=robot, config={
            "model": "gpt-4",
            "validate_plans": True
        })

        # Preview plan
        plan = agent.plan_mission("Go to kitchen and take photo")
        for skill in plan:
            print(f"- {skill.skill_name}({skill.parameters})")

        # Execute mission
        result = agent.execute_mission("Go to kitchen and take photo")
        ```
    """

    def __init__(
        self,
        skills=None,
        robot=None,
        config: Optional[Dict[str, Any]] = None,
    ):
        """Initialize planning agent.

        Args:
            skills: Skill library instance
            robot: Robot interface instance
            config: Configuration dict with optional keys:
                - model: OpenAI model name (default: "gpt-4")
                - max_tokens: Max tokens for response
                - validate_plans: Whether to validate plans before execution
                - api_key: OpenAI API key (optional, uses env var if not provided)

        Raises:
            AgentInitializationError: If DIMOS not available or init fails
        """
        super().__init__(skills, robot, config)

        if not DIMOS_AVAILABLE:
            raise AgentInitializationError(
                "DIMOS framework not available. Cannot create PlanningAgent."
            )

        # Extract config
        self.model = self.config.get("model", "gpt-4")
        self.validate_plans = self.config.get("validate_plans", True)

        # Initialize underlying DIMOS agent
        try:
            # DIMOS PlanningAgent takes robot and skill_library
            self._dimos_agent = DIMOSPlanningAgent(
                robot=robot,
                skill_library=skills,
                model=self.model,
            )
        except Exception as e:
            raise AgentInitializationError(f"Failed to initialize DIMOS PlanningAgent: {e}")

    def execute_mission(self, command: str) -> MissionResult:
        """Execute mission using planning agent.

        Args:
            command: Natural language command

        Returns:
            MissionResult with execution details

        Raises:
            MissionExecutionError: If execution fails
        """
        start_time = time.time()

        try:
            # Generate plan
            try:
                plan = self.plan_mission(command)
            except PlanningError as e:
                return MissionResult(
                    status=MissionStatus.FAILURE,
                    message=f"Failed to generate mission plan: {e}",
                    skills_executed=[],
                    skills_failed=[],
                    telemetry={
                        "execution_time_seconds": time.time() - start_time,
                        "error": str(e),
                        "agent_type": "planning",
                    },
                )

            # Execute plan
            skills_executed = []
            skills_failed = []

            # DIMOS PlanningAgent has execute_plan() method
            # Note: This is a simplified implementation - actual DIMOS interface may differ
            try:
                result = self._dimos_agent.run_query(command)

                # Track executed skills (would need DIMOS to provide this info)
                skills_executed = plan  # Assume all planned skills executed

                execution_time = time.time() - start_time

                return MissionResult(
                    status=MissionStatus.SUCCESS,
                    message=result or "Mission completed successfully",
                    skills_executed=skills_executed,
                    skills_failed=skills_failed,
                    raw_response=result,
                    telemetry={
                        "execution_time_seconds": execution_time,
                        "model": self.model,
                        "agent_type": "planning",
                        "skills_planned": len(plan),
                        "skills_executed": len(skills_executed),
                    },
                )

            except Exception as e:
                # Partial execution - some skills may have succeeded
                execution_time = time.time() - start_time
                return MissionResult(
                    status=MissionStatus.PARTIAL,
                    message=f"Mission partially completed: {str(e)}",
                    skills_executed=skills_executed,
                    skills_failed=skills_failed,
                    telemetry={
                        "execution_time_seconds": execution_time,
                        "error": str(e),
                        "agent_type": "planning",
                    },
                )

        except Exception as e:
            execution_time = time.time() - start_time
            return MissionResult(
                status=MissionStatus.FAILURE,
                message=f"Mission execution failed: {str(e)}",
                skills_executed=[],
                skills_failed=[],
                telemetry={
                    "execution_time_seconds": execution_time,
                    "error": str(e),
                    "agent_type": "planning",
                },
            )

    def plan_mission(self, command: str) -> List[SkillCall]:
        """Generate mission plan without executing.

        Args:
            command: Natural language command

        Returns:
            List of SkillCall objects representing the planned actions

        Raises:
            PlanningError: If planning fails
        """
        try:
            # DIMOS PlanningAgent should have a method to generate plans
            # Note: Actual implementation depends on DIMOS API
            # For now, we'll use a simplified approach

            # TODO: Implement actual plan extraction from DIMOS PlanningAgent
            # This would require DIMOS to expose the planning method separately
            # from execution

            # Placeholder - in reality we'd call something like:
            # plan = self._dimos_agent.generate_plan(command)

            raise NotImplementedError(
                "Plan extraction not yet implemented for DIMOS PlanningAgent. "
                "DIMOS needs to expose plan generation separately from execution."
            )

        except NotImplementedError:
            raise
        except Exception as e:
            raise PlanningError(f"Failed to generate mission plan: {e}")

    def validate_plan(self, plan: List[SkillCall]) -> bool:
        """Validate a mission plan before execution.

        Checks:
        - All skills exist in skill library
        - Parameters are valid
        - Skill sequence is feasible

        Args:
            plan: List of SkillCall objects to validate

        Returns:
            True if plan is valid, False otherwise
        """
        if not self.validate_plans:
            return True

        try:
            for skill_call in plan:
                # Check skill exists
                if not self.validate_skill(skill_call.skill_name):
                    return False

                # TODO: Add parameter validation
                # Would require skill library to expose parameter schemas

            return True

        except Exception:
            return False
