#!/usr/bin/env python3
"""Mission Executor - Pure Python mission execution logic.

This module contains the core business logic for mission execution, separated from
ROS concerns. This separation enables:
- Testing without ROS infrastructure
- Reuse in scripts, notebooks, and web applications
- Clear separation of concerns (ROS vs business logic)

The MissionExecutor handles:
- Robot initialization (DIMOS UnitreeGo2)
- Agent initialization (DIMOS agents)
- Skill library setup
- Mission execution logic

It does NOT handle:
- ROS node initialization
- ROS topic publishers/subscribers
- ROS parameter handling
- ROS logging (uses standard Python logging)
"""

import os
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass

# Import DIMOS components
try:
    from dimos.agents.agent import OpenAIAgent
    from dimos.agents.planning_agent import PlanningAgent
    from dimos.robot.unitree.unitree_go2 import UnitreeGo2
    from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
    from dimos.robot.unitree.unitree_skills import MyUnitreeSkills

    DIMOS_AVAILABLE = True
except ImportError as e:
    DIMOS_AVAILABLE = False
    IMPORT_ERROR = str(e)


@dataclass
class MissionExecutorConfig:
    """Configuration for MissionExecutor."""

    agent_backend: str = "cloud"  # 'cloud' or 'local'
    use_planning_agent: bool = False  # Use PlanningAgent vs OpenAIAgent
    robot_ip: str = "192.168.1.103"  # Robot IP address
    webrtc_api_topic: str = "webrtc_req"  # ROS topic for WebRTC API commands
    agent_model: str = "gpt-4-turbo"  # LLM model to use


class MissionExecutor:
    """Pure Python mission executor using DIMOS framework.

    This class contains the core mission execution logic without ROS dependencies.
    It can be used in ROS nodes, scripts, notebooks, or web applications.

    Example usage:
        # In a ROS node
        config = MissionExecutorConfig(agent_backend="cloud")
        executor = MissionExecutor(config, logger=node.get_logger())
        executor.initialize()
        result = executor.execute_mission("patrol the perimeter")

        # In a script
        executor = MissionExecutor(MissionExecutorConfig())
        executor.initialize()
        result = executor.execute_mission("go to waypoint A")

        # In a notebook
        import logging
        logger = logging.getLogger(__name__)
        executor = MissionExecutor(MissionExecutorConfig(), logger=logger)
        executor.initialize()
    """

    def __init__(
        self, config: MissionExecutorConfig, logger: Optional[logging.Logger] = None
    ):
        """Initialize mission executor.

        Args:
            config: Configuration for the executor
            logger: Optional logger (uses standard logging if not provided)
        """
        self.config = config
        self.logger = logger or logging.getLogger(__name__)

        # DIMOS components (initialized in initialize())
        self.robot: Optional[UnitreeGo2] = None
        self.skills: Optional[MyUnitreeSkills] = None
        self.agent: Optional[OpenAIAgent] = None

        # Initialization state
        self._initialized = False

        # Check DIMOS availability
        if not DIMOS_AVAILABLE:
            raise RuntimeError(f"DIMOS framework not available: {IMPORT_ERROR}")

    def initialize(self) -> None:
        """Initialize robot, skills, and agent.

        This must be called before execute_mission(). Separated from __init__
        to allow for explicit initialization timing (e.g., after ROS node is ready).

        Raises:
            RuntimeError: If initialization fails
        """
        if self._initialized:
            self.logger.warning("MissionExecutor already initialized, skipping")
            return

        self.logger.info("Initializing MissionExecutor...")

        try:
            # 1. Initialize robot interface
            self._init_robot()

            # 2. Initialize skill library
            self._init_skills()

            # 3. Initialize agent
            self._init_agent()

            self._initialized = True
            self.logger.info("MissionExecutor initialization complete")

        except Exception as e:
            self.logger.error(f"Failed to initialize MissionExecutor: {e}")
            raise

    def _init_robot(self) -> None:
        """Initialize DIMOS robot interface."""
        self.logger.info("Initializing robot interface...")

        # Log connection mode for diagnostics
        conn_type = os.getenv("CONN_TYPE", "cyclonedds")
        self.logger.info(f"Connection type: {conn_type}")

        if conn_type == "webrtc":
            self.logger.info(
                "WebRTC mode: High-level API commands (sit, stand, wave) enabled"
            )
        else:
            self.logger.warning(
                "CycloneDDS mode: High-level API commands NOT available"
            )
            self.logger.warning("Set CONN_TYPE=webrtc to enable DIMOS skills")

        # Initialize ROS control bridge
        # Note: This uses ROS topics but doesn't require the caller to be a ROS node
        ros_control = UnitreeROSControl(webrtc_api_topic=self.config.webrtc_api_topic)

        # Initialize robot with ROS provider
        # CONN_TYPE env var controls the underlying communication protocol
        self.robot = UnitreeGo2(
            ros_control=ros_control,
            ip=self.config.robot_ip,
        )

        self.logger.info(f"Robot initialized (ip={self.config.robot_ip})")

    def _init_skills(self) -> None:
        """Initialize DIMOS skill library."""
        if not self.robot:
            raise RuntimeError("Robot must be initialized before skills")

        self.logger.info("Initializing skill library...")
        self.skills = MyUnitreeSkills(robot=self.robot)
        skill_count = len(self.skills.get())
        self.logger.info(f"Loaded {skill_count} skills")

    def _init_agent(self) -> None:
        """Initialize DIMOS agent."""
        if not self.skills:
            raise RuntimeError("Skills must be initialized before agent")

        self.logger.info(
            f"Initializing {self.config.agent_backend} agent "
            f"({'planning' if self.config.use_planning_agent else 'openai'})..."
        )

        # Check for API key if using cloud backend
        if self.config.agent_backend == "cloud":
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                self.logger.warning("OPENAI_API_KEY not set, agent may not function")

        # Create DIMOS agent based on type
        if self.config.use_planning_agent:
            self.agent = PlanningAgent(
                robot=self.robot, dev_name="shadowhound", agent_type="Mission"
            )
            self.logger.info("DIMOS PlanningAgent initialized")
        else:
            self.agent = OpenAIAgent(
                dev_name="shadowhound",
                agent_type="Mission",
                skills=self.skills,
                model_name=self.config.agent_model,
            )
            self.logger.info(f"DIMOS OpenAIAgent initialized (model={self.config.agent_model})")

    def execute_mission(self, command: str) -> str:
        """Execute a mission command.

        Args:
            command: Natural language mission command

        Returns:
            Agent's response as a string

        Raises:
            RuntimeError: If executor not initialized or execution fails
        """
        if not self._initialized:
            raise RuntimeError("MissionExecutor not initialized. Call initialize() first.")

        self.logger.info(f"Executing mission: {command}")

        try:
            # Execute through DIMOS agent
            if self.config.use_planning_agent:
                response = self.agent.plan_and_execute(command)
            else:
                # OpenAIAgent uses run_observable_query() which returns an Observable
                response = self.agent.run_observable_query(command).run()

            self.logger.info(f"Mission completed: {response[:100]}...")
            return response

        except Exception as e:
            self.logger.error(f"Mission execution failed: {e}")
            raise

    def get_robot_status(self) -> Dict[str, Any]:
        """Get current robot status.

        Returns:
            Dictionary with robot status information

        Raises:
            RuntimeError: If robot not initialized
        """
        if not self.robot:
            raise RuntimeError("Robot not initialized")

        # TODO: Implement robot status query
        # This would call robot.get_state() or similar
        return {
            "initialized": self._initialized,
            "robot_ip": self.config.robot_ip,
            "skills_count": len(self.skills.get()) if self.skills else 0,
        }

    def get_available_skills(self) -> list:
        """Get list of available skills.

        Returns:
            List of skill information

        Raises:
            RuntimeError: If skills not initialized
        """
        if not self.skills:
            raise RuntimeError("Skills not initialized")

        return self.skills.get()

    def cleanup(self) -> None:
        """Clean up resources."""
        self.logger.info("Cleaning up MissionExecutor...")

        if self.agent:
            try:
                self.agent.dispose_all()
            except Exception as e:
                self.logger.warning(f"Error disposing agent: {e}")

        # Robot cleanup if needed
        if self.robot:
            try:
                # Add any robot cleanup if needed
                pass
            except Exception as e:
                self.logger.warning(f"Error cleaning up robot: {e}")

        self._initialized = False
        self.logger.info("MissionExecutor cleanup complete")
