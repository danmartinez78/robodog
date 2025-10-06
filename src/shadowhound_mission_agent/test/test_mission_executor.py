"""Unit tests for MissionExecutor - pure Python business logic tests.

Tests the MissionExecutor class without requiring ROS infrastructure.
This demonstrates the testability benefit of separating ROS concerns
from business logic.

Note: Some tests that require full DIMOS initialization are skipped when
DIMOS is not available. The key tests here validate the MissionExecutor
interface and error handling without needing the full robot stack.
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
from shadowhound_mission_agent.mission_executor import (
    MissionExecutor,
    MissionExecutorConfig,
    DIMOS_AVAILABLE,
)


# ============================================================================
# MissionExecutorConfig Tests
# ============================================================================


def test_config_defaults():
    """Test MissionExecutorConfig default values."""
    config = MissionExecutorConfig()

    assert config.agent_backend == "cloud"
    assert config.use_planning_agent is False
    assert config.robot_ip == "192.168.1.103"
    assert config.webrtc_api_topic == "webrtc_req"
    assert config.agent_model == "gpt-4-turbo"


def test_config_custom_values():
    """Test MissionExecutorConfig with custom values."""
    config = MissionExecutorConfig(
        agent_backend="local",
        use_planning_agent=True,
        robot_ip="192.168.1.200",
        webrtc_api_topic="custom_topic",
        agent_model="gpt-3.5-turbo",
    )

    assert config.agent_backend == "local"
    assert config.use_planning_agent is True
    assert config.robot_ip == "192.168.1.200"
    assert config.webrtc_api_topic == "custom_topic"
    assert config.agent_model == "gpt-3.5-turbo"


# ============================================================================
# MissionExecutor Initialization Tests
# ============================================================================


@patch("shadowhound_mission_agent.mission_executor.DIMOS_AVAILABLE", False)
def test_executor_fails_without_dimos():
    """Test that executor raises error when DIMOS not available."""
    config = MissionExecutorConfig()

    with pytest.raises(RuntimeError, match="DIMOS framework not available"):
        executor = MissionExecutor(config)


@patch("shadowhound_mission_agent.mission_executor.DIMOS_AVAILABLE", True)
def test_executor_creation():
    """Test MissionExecutor can be created with valid config."""
    config = MissionExecutorConfig()
    mock_logger = Mock()

    executor = MissionExecutor(config, logger=mock_logger)

    assert executor.config == config
    assert executor.logger == mock_logger
    assert executor._initialized is False
    assert executor.robot is None
    assert executor.skills is None
    assert executor.agent is None


@patch("shadowhound_mission_agent.mission_executor.DIMOS_AVAILABLE", True)
def test_executor_uses_default_logger():
    """Test that executor creates default logger if none provided."""
    config = MissionExecutorConfig()

    executor = MissionExecutor(config)

    assert executor.logger is not None
    assert hasattr(executor.logger, "info")
    assert hasattr(executor.logger, "error")


# ============================================================================
# MissionExecutor Initialization Flow Tests
# ============================================================================


@pytest.mark.skipif(not DIMOS_AVAILABLE, reason="Requires DIMOS framework")
def test_executor_initialization_requires_dimos():
    """Test that initialization requires DIMOS (integration test placeholder)."""
    # This is a placeholder for integration tests that would run with
    # actual DIMOS available. For unit tests, we test the interface
    # and error handling without full DIMOS mocking.
    config = MissionExecutorConfig()
    mock_logger = Mock()
    
    executor = MissionExecutor(config, logger=mock_logger)
    # With DIMOS available, initialization should work
    # (actual test would call executor.initialize() and verify)
    assert executor is not None


@patch("shadowhound_mission_agent.mission_executor.DIMOS_AVAILABLE", True)
def test_executor_double_initialization_warning(capsys):
    """Test that double initialization logs warning and skips."""
    config = MissionExecutorConfig()
    mock_logger = Mock()

    executor = MissionExecutor(config, logger=mock_logger)

    # Mock the initialization to succeed
    with patch.object(executor, "_init_robot"), patch.object(
        executor, "_init_skills"
    ), patch.object(executor, "_init_agent"):
        executor.initialize()
        executor.initialize()  # Second call

    # Verify warning was logged
    assert mock_logger.warning.called
    warning_msg = mock_logger.warning.call_args[0][0]
    assert "already initialized" in warning_msg.lower()


# ============================================================================
# Mission Execution Tests
# ============================================================================


@patch("shadowhound_mission_agent.mission_executor.DIMOS_AVAILABLE", True)
def test_execute_mission_not_initialized():
    """Test that execute_mission fails if not initialized."""
    config = MissionExecutorConfig()
    executor = MissionExecutor(config)

    with pytest.raises(RuntimeError, match="not initialized"):
        executor.execute_mission("test command")


@pytest.mark.skipif(not DIMOS_AVAILABLE, reason="Requires DIMOS framework")
def test_execute_mission_interface():
    """Test execute_mission interface (integration test placeholder)."""
    # Placeholder for integration test with actual DIMOS
    # Unit test above verifies error handling
    pass


# ============================================================================
# Status and Utility Method Tests  
# ============================================================================


@patch("shadowhound_mission_agent.mission_executor.DIMOS_AVAILABLE", True)
def test_get_robot_status_not_initialized():
    """Test that get_robot_status fails if robot not initialized."""
    config = MissionExecutorConfig()
    executor = MissionExecutor(config)

    with pytest.raises(RuntimeError, match="Robot not initialized"):
        executor.get_robot_status()


@patch("shadowhound_mission_agent.mission_executor.DIMOS_AVAILABLE", True)
def test_get_available_skills_not_initialized():
    """Test that get_available_skills fails if skills not initialized."""
    config = MissionExecutorConfig()
    executor = MissionExecutor(config)

    with pytest.raises(RuntimeError, match="Skills not initialized"):
        executor.get_available_skills()


@pytest.mark.skipif(not DIMOS_AVAILABLE, reason="Requires DIMOS framework")
def test_status_methods_with_initialization():
    """Test status methods work after initialization (integration test placeholder)."""
    # Placeholder for integration tests
    pass


# ============================================================================
# Cleanup Tests
# ============================================================================


@patch("shadowhound_mission_agent.mission_executor.DIMOS_AVAILABLE", True)
def test_cleanup_handles_errors_gracefully():
    """Test that cleanup handles errors gracefully."""
    config = MissionExecutorConfig()
    mock_logger = Mock()

    executor = MissionExecutor(config, logger=mock_logger)

    # Set up agent that will fail on dispose
    executor.agent = Mock()
    executor.agent.dispose_all.side_effect = Exception("Dispose failed!")
    executor._initialized = True

    # Cleanup should not raise
    executor.cleanup()

    # Verify warning was logged
    assert mock_logger.warning.called
    warning_msg = mock_logger.warning.call_args[0][0]
    assert "disposing agent" in warning_msg.lower()


@pytest.mark.skipif(not DIMOS_AVAILABLE, reason="Requires DIMOS framework")
def test_cleanup_full_cycle():
    """Test cleanup after full initialization (integration test placeholder)."""
    # Placeholder for integration test
    pass
