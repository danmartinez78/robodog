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
    """Configuration for MissionExecutor.

    Backend Options:
    - 'openai': Use OpenAI cloud API (requires OPENAI_API_KEY)
    - 'ollama': Use Ollama (self-hosted, can be local or remote)

    Deployment Scenarios:
    - Development: agent_backend='ollama', ollama_base_url='http://<gaming-pc-ip>:11434'
    - Thor Production: agent_backend='ollama', ollama_base_url='http://localhost:11434'
    - Cloud Fallback: agent_backend='openai' (slower but higher quality)
    """

    agent_backend: str = "openai"  # 'openai' or 'ollama'
    use_planning_agent: bool = False  # Use PlanningAgent vs OpenAIAgent
    robot_ip: str = "192.168.1.103"  # Robot IP address
    webrtc_api_topic: str = "webrtc_req"  # ROS topic for WebRTC API commands

    # OpenAI backend settings (cloud)
    openai_model: str = "gpt-4-turbo"  # OpenAI model name
    openai_base_url: str = "https://api.openai.com/v1"  # OpenAI API endpoint

    # Ollama backend settings (self-hosted)
    ollama_base_url: str = "http://localhost:11434"  # Ollama server URL
    ollama_model: str = "llama3.1:70b"  # Ollama model name

    # Token limits (apply to both backends)
    max_output_tokens: int = 4096  # Max tokens for model output
    max_input_tokens: int = 128000  # Max tokens for model input


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
        """Initialize DIMOS agent with appropriate backend."""
        if not self.skills:
            raise RuntimeError("Skills must be initialized before agent")

        self.logger.info(
            f"Initializing {self.config.agent_backend} agent "
            f"({'planning' if self.config.use_planning_agent else 'openai'})..."
        )

        # Configure OpenAI client based on backend
        from openai import OpenAI

        if self.config.agent_backend == "ollama":
            # Use Ollama backend (self-hosted LLM)
            self.logger.info(f"Using Ollama backend at {self.config.ollama_base_url}")
            client = OpenAI(
                base_url=f"{self.config.ollama_base_url}/v1",
                api_key="ollama",  # Ollama doesn't validate API keys
            )
            model_name = self.config.ollama_model
            self.logger.info(f"Ollama model: {model_name}")

        elif self.config.agent_backend == "openai":
            # Use OpenAI cloud backend
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                self.logger.warning("OPENAI_API_KEY not set, agent may not function")

            self.logger.info("Using OpenAI cloud backend")
            client = OpenAI(base_url=self.config.openai_base_url, api_key=api_key)
            model_name = self.config.openai_model
            self.logger.info(f"OpenAI model: {model_name}")

        else:
            raise ValueError(
                f"Unknown agent_backend: {self.config.agent_backend}. "
                "Must be 'openai' or 'ollama'"
            )

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
                model_name=model_name,
                openai_client=client,  # Pass custom client for backend flexibility
                max_output_tokens_per_request=self.config.max_output_tokens,
                max_input_tokens_per_request=self.config.max_input_tokens,
            )
            self.logger.info(
                f"DIMOS OpenAIAgent initialized "
                f"(backend={self.config.agent_backend}, model={model_name}, "
                f"max_output={self.config.max_output_tokens})"
            )

    def execute_mission(self, command: str) -> tuple[str, dict]:
        """Execute a mission command.

        Args:
            command: Natural language mission command

        Returns:
            Tuple of (response string, timing dict)
            Timing dict contains: agent_duration, total_duration, overhead_duration

        Raises:
            RuntimeError: If executor not initialized or execution fails
        """
        import time

        if not self._initialized:
            raise RuntimeError(
                "MissionExecutor not initialized. Call initialize() first."
            )

        self.logger.info(f"Executing mission: {command}")
        start_time = time.time()

        try:
            # Execute through DIMOS agent
            agent_start = time.time()
            if self.config.use_planning_agent:
                response = self.agent.plan_and_execute(command)
            else:
                # OpenAIAgent uses run_observable_query() which returns an Observable
                response = self.agent.run_observable_query(command).run()

            agent_duration = time.time() - agent_start
            total_duration = time.time() - start_time
            overhead_duration = total_duration - agent_duration

            timing_info = {
                "agent_duration": agent_duration,
                "total_duration": total_duration,
                "overhead_duration": overhead_duration,
                "agent_percentage": (
                    (agent_duration / total_duration * 100) if total_duration > 0 else 0
                ),
            }

            self.logger.info(f"⏱️  Timing breakdown:")
            self.logger.info(
                f"   Agent call: {agent_duration:.2f}s ({timing_info['agent_percentage']:.0f}%)"
            )
            self.logger.info(f"   Overhead:   {overhead_duration:.3f}s")
            self.logger.info(f"   Total:      {total_duration:.2f}s")
            self.logger.info(f"Mission completed: {response[:100]}...")
            return response, timing_info

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
