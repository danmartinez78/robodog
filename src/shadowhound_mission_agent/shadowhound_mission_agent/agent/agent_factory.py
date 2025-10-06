"""Factory for creating agent instances.

Provides a clean interface for instantiating different agent types based on
configuration. Handles dependency injection and initialization.
"""

from typing import Optional, Dict, Any
from .base_agent import BaseAgent, AgentInitializationError
from .openai_agent import OpenAIAgent
from .planning_agent import PlanningAgent


class AgentFactory:
    """Factory for creating agent instances.

    Example:
        ```python
        # Create simple OpenAI agent
        agent = AgentFactory.create(
            agent_type="openai",
            skills=skill_lib,
            robot=robot,
            config={"model": "gpt-4"}
        )

        # Create planning agent
        agent = AgentFactory.create(
            agent_type="planning",
            skills=skill_lib,
            robot=robot,
            config={"model": "gpt-4", "validate_plans": True}
        )
        ```
    """

    # Registry of available agent types
    _AGENT_TYPES = {
        "openai": OpenAIAgent,
        "planning": PlanningAgent,
        # Future: "local": LocalLLMAgent, "mock": MockAgent, etc.
    }

    @classmethod
    def create(
        cls,
        agent_type: str,
        skills=None,
        robot=None,
        config: Optional[Dict[str, Any]] = None,
    ) -> BaseAgent:
        """Create an agent instance.

        Args:
            agent_type: Type of agent to create ("openai", "planning", etc.)
            skills: Skill library instance
            robot: Robot interface instance
            config: Optional configuration dict

        Returns:
            BaseAgent instance

        Raises:
            AgentInitializationError: If agent type unknown or init fails
        """
        agent_type = agent_type.lower()

        if agent_type not in cls._AGENT_TYPES:
            available = ", ".join(cls._AGENT_TYPES.keys())
            raise AgentInitializationError(
                f"Unknown agent type: '{agent_type}'. "
                f"Available types: {available}"
            )

        agent_class = cls._AGENT_TYPES[agent_type]

        try:
            return agent_class(skills=skills, robot=robot, config=config or {})
        except Exception as e:
            raise AgentInitializationError(
                f"Failed to create {agent_type} agent: {e}"
            ) from e

    @classmethod
    def from_config_dict(cls, config: Dict[str, Any]) -> BaseAgent:
        """Create agent from configuration dictionary.

        Convenient method for creating agents from loaded config files.

        Args:
            config: Configuration dict with keys:
                - agent_type: Type of agent ("openai", "planning")
                - skills: Skill library instance
                - robot: Robot interface instance
                - ... other agent-specific config

        Returns:
            BaseAgent instance

        Raises:
            AgentInitializationError: If config invalid or init fails
        """
        if "agent_type" not in config:
            raise AgentInitializationError(
                "Configuration must include 'agent_type' key"
            )

        agent_type = config.pop("agent_type")
        skills = config.pop("skills", None)
        robot = config.pop("robot", None)

        # Remaining config is agent-specific
        return cls.create(
            agent_type=agent_type,
            skills=skills,
            robot=robot,
            config=config,
        )

    @classmethod
    def register_agent_type(cls, name: str, agent_class: type):
        """Register a new agent type.

        Allows external code to add custom agent implementations.

        Args:
            name: Name to register agent under
            agent_class: Agent class (must inherit from BaseAgent)

        Raises:
            ValueError: If agent_class doesn't inherit from BaseAgent
        """
        if not issubclass(agent_class, BaseAgent):
            raise ValueError(
                f"Agent class must inherit from BaseAgent, got {agent_class}"
            )

        cls._AGENT_TYPES[name.lower()] = agent_class

    @classmethod
    def list_agent_types(cls) -> list[str]:
        """Get list of available agent types.

        Returns:
            List of registered agent type names
        """
        return list(cls._AGENT_TYPES.keys())
