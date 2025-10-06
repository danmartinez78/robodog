"""Unit tests for agent module - basic tests without DIMOS dependency.

Tests the agent architecture components that don't require DIMOS.
"""

import pytest
from unittest.mock import Mock
from shadowhound_mission_agent.agent import (
    BaseAgent,
    MissionResult,
    MissionStatus,
    SkillCall,
    AgentFactory,
    AgentError,
    AgentInitializationError,
)


# ============================================================================
# Mock Agent for Testing
# ============================================================================


class MockAgent(BaseAgent):
    """Mock agent for testing BaseAgent interface."""

    def __init__(self, skills=None, robot=None, config=None):
        super().__init__(skills, robot, config)
        self.execute_called = False
        self.plan_called = False
        self.should_fail = False

    def execute_mission(self, command: str) -> MissionResult:
        """Mock execution that tracks calls."""
        self.execute_called = True

        if self.should_fail:
            return MissionResult(
                status=MissionStatus.FAILURE,
                message=f"Mock failure: {command}",
                skills_failed=[
                    SkillCall(
                        skill_name="mock_skill", parameters={}, description="failed"
                    )
                ],
            )

        return MissionResult(
            status=MissionStatus.SUCCESS,
            message=f"Mock success: {command}",
            skills_executed=[
                SkillCall(
                    skill_name="mock_skill",
                    parameters={"param": "value"},
                    description="success",
                )
            ],
            telemetry={"execution_time": 0.1},
        )

    def plan_mission(self, command: str) -> list[SkillCall]:
        """Mock planning that tracks calls."""
        self.plan_called = True
        return [
            SkillCall(skill_name="skill1", parameters={"x": 1}),
            SkillCall(skill_name="skill2", parameters={"y": 2}),
        ]


# ============================================================================
# MissionResult Tests
# ============================================================================


def test_mission_result_success():
    """Test successful mission result."""
    result = MissionResult(
        status=MissionStatus.SUCCESS,
        message="All done!",
        skills_executed=[
            SkillCall(skill_name="move", parameters={"x": 1.0}, description="ok")
        ],
        telemetry={"time": 10.0},
    )

    assert result.status == MissionStatus.SUCCESS
    assert result.message == "All done!"
    assert len(result.skills_executed) == 1
    assert len(result.skills_failed) == 0
    assert result.telemetry["time"] == 10.0


def test_mission_result_failure():
    """Test failed mission result."""
    result = MissionResult(
        status=MissionStatus.FAILURE,
        message="Something went wrong",
        skills_failed=[
            SkillCall(skill_name="move", parameters={"x": 1.0}, description="error")
        ],
    )

    assert result.status == MissionStatus.FAILURE
    assert result.message == "Something went wrong"
    assert len(result.skills_executed) == 0
    assert len(result.skills_failed) == 1


def test_mission_result_partial():
    """Test partial success result."""
    result = MissionResult(
        status=MissionStatus.PARTIAL,
        message="Some skills failed",
        skills_executed=[
            SkillCall(skill_name="move", parameters={}, description="ok")
        ],
        skills_failed=[
            SkillCall(skill_name="rotate", parameters={}, description="timeout")
        ],
    )

    assert result.status == MissionStatus.PARTIAL
    assert len(result.skills_executed) == 1
    assert len(result.skills_failed) == 1


# ============================================================================
# SkillCall Tests
# ============================================================================


def test_skill_call_creation():
    """Test creating a SkillCall."""
    skill = SkillCall(
        skill_name="nav.goto",
        parameters={"x": 1.0, "y": 2.0, "yaw": 0.0},
        description="Arrived at destination",
    )

    assert skill.skill_name == "nav.goto"
    assert skill.parameters["x"] == 1.0
    assert skill.description == "Arrived at destination"


def test_skill_call_minimal():
    """Test SkillCall with minimal parameters."""
    skill = SkillCall(skill_name="test_skill", parameters={})

    assert skill.skill_name == "test_skill"
    assert skill.parameters == {}
    assert skill.description is None


# ============================================================================
# BaseAgent Tests
# ============================================================================


def test_base_agent_cannot_instantiate():
    """Test that BaseAgent cannot be instantiated directly."""
    with pytest.raises(TypeError):
        BaseAgent()


def test_base_agent_subclass_must_implement_methods():
    """Test that subclasses must implement abstract methods."""

    class IncompleteAgent(BaseAgent):
        pass

    with pytest.raises(TypeError):
        IncompleteAgent()


def test_mock_agent_execute_success():
    """Test mock agent execution succeeds."""
    agent = MockAgent()
    result = agent.execute_mission("test command")

    assert agent.execute_called
    assert result.status == MissionStatus.SUCCESS
    assert "test command" in result.message
    assert len(result.skills_executed) == 1
    assert result.telemetry is not None


def test_mock_agent_execute_failure():
    """Test mock agent failure."""
    agent = MockAgent()
    agent.should_fail = True

    result = agent.execute_mission("test command")

    assert result.status == MissionStatus.FAILURE
    assert len(result.skills_failed) == 1


def test_mock_agent_plan():
    """Test mock agent planning."""
    agent = MockAgent()

    plan = agent.plan_mission("test command")

    assert agent.plan_called
    assert len(plan) == 2
    assert plan[0].skill_name == "skill1"
    assert plan[1].skill_name == "skill2"


# ============================================================================
# AgentFactory Tests
# ============================================================================


def test_agent_factory_register_custom_agent():
    """Test registering a custom agent type."""
    AgentFactory.register_agent_type("mock", MockAgent)

    # Should be in registry
    assert "mock" in AgentFactory._AGENT_TYPES


def test_agent_factory_create_mock_agent():
    """Test creating a mock agent via factory."""
    AgentFactory.register_agent_type("mock", MockAgent)

    mock_skills = Mock()
    mock_robot = Mock()

    agent = AgentFactory.create(
        agent_type="mock",
        skills=mock_skills,
        robot=mock_robot,
        config={"dev_name": "test"},
    )

    assert isinstance(agent, MockAgent)
    assert agent.skills == mock_skills
    assert agent.robot == mock_robot
    assert agent.config["dev_name"] == "test"


def test_agent_factory_invalid_type():
    """Test creating agent with invalid type."""
    with pytest.raises(AgentInitializationError, match="Unknown agent type"):
        AgentFactory.create(
            agent_type="nonexistent_type", skills=Mock(), robot=Mock(), config={}
        )


def test_agent_factory_from_config_dict():
    """Test creating agent from config dict."""
    AgentFactory.register_agent_type("mock", MockAgent)

    config = {
        "agent_type": "mock",
        "dev_name": "test",
        "model": "gpt-4",
    }

    agent = AgentFactory.from_config_dict(config)

    assert isinstance(agent, MockAgent)
    # Config should be passed through
    assert "dev_name" in agent.config


# ============================================================================
# Integration Tests
# ============================================================================


def test_agent_lifecycle():
    """Test complete agent lifecycle."""
    # Register mock agent
    AgentFactory.register_agent_type("mock", MockAgent)

    # Create agent via factory
    agent = AgentFactory.create(
        agent_type="mock", skills=Mock(), robot=Mock(), config={"dev_name": "test"}
    )

    # Execute mission
    result = agent.execute_mission("Go to waypoint")

    # Verify
    assert isinstance(agent, MockAgent)
    assert agent.execute_called
    assert result.status == MissionStatus.SUCCESS


def test_agent_factory_extensibility():
    """Test that factory can be extended with new agent types."""

    class CustomAgent(BaseAgent):
        def execute_mission(self, command: str) -> MissionResult:
            return MissionResult(
                status=MissionStatus.SUCCESS,
                message="Custom agent executed",
                skills_executed=[
                    SkillCall(skill_name="custom", parameters={}, description="done")
                ],
            )

        def plan_mission(self, command: str) -> list[SkillCall]:
            return []

    # Register custom agent
    AgentFactory.register_agent_type("custom", CustomAgent)

    # Create and use custom agent
    agent = AgentFactory.create(
        agent_type="custom", skills=Mock(), robot=Mock(), config={}
    )

    result = agent.execute_mission("test")

    assert "Custom agent executed" in result.message
    assert len(result.skills_executed) == 1


def test_mission_result_with_telemetry():
    """Test mission result includes telemetry data."""
    agent = MockAgent()
    result = agent.execute_mission("test")

    assert result.telemetry is not None
    assert "execution_time" in result.telemetry


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
