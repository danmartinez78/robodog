"""OpenAI-based agent implementation.

Wraps DIMOS's OpenAIAgent to provide a clean interface conforming to BaseAgent.
Handles the Observable pattern and provides structured MissionResult outputs.
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
    from dimos.agents.agent import OpenAIAgent as DIMOSOpenAIAgent

    DIMOS_AVAILABLE = True
except ImportError:
    DIMOS_AVAILABLE = False
    DIMOSOpenAIAgent = None


class OpenAIAgent(BaseAgent):
    """OpenAI GPT-based agent for mission execution.

    Uses OpenAI's GPT models (GPT-4, GPT-3.5-turbo) to understand natural language
    commands and generate skill execution plans.

    Example:
        ```python
        agent = OpenAIAgent(skills=skill_lib, robot=robot, config={
            "model": "gpt-4",
            "max_tokens": 4096
        })

        result = agent.execute_mission("Navigate to the kitchen")
        if result.success:
            print("Mission completed!")
        ```
    """

    def __init__(
        self,
        skills=None,
        robot=None,
        config: Optional[Dict[str, Any]] = None,
    ):
        """Initialize OpenAI agent.

        Args:
            skills: Skill library instance
            robot: Robot interface instance
            config: Configuration dict with optional keys:
                - model: OpenAI model name (default: "gpt-4")
                - max_tokens: Max tokens for response
                - temperature: Sampling temperature
                - api_key: OpenAI API key (optional, uses env var if not provided)

        Raises:
            AgentInitializationError: If DIMOS not available or init fails
        """
        super().__init__(skills, robot, config)

        if not DIMOS_AVAILABLE:
            raise AgentInitializationError(
                "DIMOS framework not available. Cannot create OpenAIAgent."
            )

        # Extract config
        self.model = self.config.get("model", "gpt-4")
        self.max_tokens = self.config.get("max_tokens", 4096)

        # Initialize underlying DIMOS agent
        try:
            # DIMOS OpenAIAgent takes skill_library as parameter
            # Note: DIMOS agent handles robot internally through skills
            self._dimos_agent = DIMOSOpenAIAgent(
                skill_library=skills,
                model=self.model,
            )
        except Exception as e:
            raise AgentInitializationError(f"Failed to initialize DIMOS OpenAIAgent: {e}")

    def execute_mission(self, command: str) -> MissionResult:
        """Execute mission using OpenAI agent.

        Args:
            command: Natural language command

        Returns:
            MissionResult with execution details

        Raises:
            MissionExecutionError: If execution fails
        """
        start_time = time.time()

        try:
            # DIMOS OpenAIAgent uses run_observable_query() which returns an Observable
            # We need to subscribe and collect results
            result_data = {"response": None, "error": None}

            def on_next(value):
                """Handle each emission from the observable."""
                result_data["response"] = value

            def on_error(error):
                """Handle errors from the observable."""
                result_data["error"] = str(error)

            def on_completed():
                """Handle completion of the observable."""
                pass

            # Execute query
            observable = self._dimos_agent.run_observable_query(command)
            observable.subscribe(on_next=on_next, on_error=on_error, on_completed=on_completed)

            # Check for errors
            if result_data["error"]:
                raise MissionExecutionError(result_data["error"])

            # Calculate telemetry
            execution_time = time.time() - start_time

            # Parse response to extract executed skills
            # Note: DIMOS agent returns natural language response, not structured skill list
            # We'd need to enhance this to track actual skill executions
            skills_executed = self._extract_skills_from_response(result_data["response"])

            return MissionResult(
                status=MissionStatus.SUCCESS,
                message=result_data["response"] or "Mission completed",
                skills_executed=skills_executed,
                skills_failed=[],
                raw_response=result_data["response"],
                telemetry={
                    "execution_time_seconds": execution_time,
                    "model": self.model,
                    "agent_type": "openai",
                },
            )

        except Exception as e:
            execution_time = time.time() - start_time
            return MissionResult(
                status=MissionStatus.FAILURE,
                message=f"Mission execution failed: {str(e)}",
                skills_executed=[],
                skills_failed=[],
                raw_response=None,
                telemetry={
                    "execution_time_seconds": execution_time,
                    "error": str(e),
                    "model": self.model,
                    "agent_type": "openai",
                },
            )

    def plan_mission(self, command: str) -> List[SkillCall]:
        """Generate mission plan without executing.

        Note: DIMOS OpenAIAgent doesn't provide a separate planning method,
        so this will actually execute the mission and extract the plan.

        Args:
            command: Natural language command

        Returns:
            List of SkillCall objects

        Raises:
            PlanningError: If planning fails
        """
        try:
            # For now, we can't separate planning from execution with DIMOS OpenAIAgent
            # This would require modifying DIMOS or using PlanningAgent instead
            raise NotImplementedError(
                "OpenAIAgent doesn't support planning without execution. "
                "Use PlanningAgent for separate planning capabilities."
            )
        except Exception as e:
            raise PlanningError(f"Failed to generate mission plan: {e}")

    def _extract_skills_from_response(self, response: str) -> List[SkillCall]:
        """Extract executed skills from agent response.

        This is a best-effort extraction since DIMOS OpenAIAgent doesn't
        return structured skill execution data.

        Args:
            response: Agent response text

        Returns:
            List of SkillCall objects (may be empty if parsing fails)
        """
        # TODO: Implement proper skill extraction
        # For now, return empty list - we'd need to enhance DIMOS agent
        # to track skill executions or parse them from logs
        return []
