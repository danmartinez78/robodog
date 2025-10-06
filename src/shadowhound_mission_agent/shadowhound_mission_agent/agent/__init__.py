"""Agent module for ShadowHound mission execution.

This module provides a clean, decoupled architecture for AI agents that control
the robot. It defines abstract interfaces and provides implementations for
different agent types (OpenAI, local LLMs, planning agents, etc.).

The module is designed to be testable independently of ROS and the robot hardware.
"""

from .base_agent import (
    BaseAgent,
    MissionResult,
    MissionStatus,
    SkillCall,
    AgentError,
    AgentInitializationError,
    MissionExecutionError,
    PlanningError,
)
from .agent_factory import AgentFactory
from .openai_agent import OpenAIAgent
from .planning_agent import PlanningAgent

__all__ = [
    # Base classes
    "BaseAgent",
    "MissionResult",
    "MissionStatus",
    "SkillCall",
    # Exceptions
    "AgentError",
    "AgentInitializationError",
    "MissionExecutionError",
    "PlanningError",
    # Factory
    "AgentFactory",
    # Implementations
    "OpenAIAgent",
    "PlanningAgent",
]

