#!/usr/bin/env python3
"""ShadowHound Mission Agent - Integrates DIM        try:
            # Initialize ROS control
            # Note: Costmap topic will be provided by go2_robot_sdk launch
            # WORKAROUND: Use mock_connection=True to skip wait_for_server() hang
            # DIMOS bug: wait_for_server() called before executor starts spinning
            ros_control = UnitreeROSControl(
                mock_connection=True,  # Skip Nav2 action server wait (DIMOS bug workaround)
                disable_video_stream=True  # Temporarily disable to debug init hang
            )

            # Get robot IP from environment (required even for mock mode)
            robot_ip = os.getenv("GO2_IP", "192.168.1.103")

            self.robot = UnitreeGo2(
                ros_control=ros_control,
                ip=robot_ip,  # Required for WebRTC connection
                mock_connection=True,  # Skip Nav2 action server wait (DIMOS bug workaround)
                disable_video_stream=True,  # Temporarily disable to debug init hang
            )2.

This node provides a ROS2 interface to DIMOS agents for autonomous mission execution.
It bridges natural language commands with the robot's skill execution system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import logging
from typing import Optional, Dict, Any
from .web_interface import WebInterface

# Import new agent architecture
from .agent import AgentFactory, BaseAgent, MissionResult, MissionStatus

# Import DIMOS components
try:
    from dimos.robot.unitree.unitree_go2 import UnitreeGo2
    from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
    from dimos.robot.unitree.unitree_skills import MyUnitreeSkills

    DIMOS_AVAILABLE = True
except ImportError as e:
    DIMOS_AVAILABLE = False
    IMPORT_ERROR = str(e)


class MissionAgentNode(Node):
    """ROS2 node that executes missions using DIMOS agents."""

    def __init__(self):
        super().__init__("shadowhound_mission_agent")

        # Declare parameters
        self.declare_parameter("agent_backend", "cloud")  # 'cloud' or 'local'
        self.declare_parameter("mock_robot", False)  # Use mock robot interface
        self.declare_parameter(
            "use_planning_agent", False
        )  # Use PlanningAgent instead of OpenAIAgent
        self.declare_parameter("enable_web_interface", True)  # Enable web dashboard
        self.declare_parameter("web_port", 8080)  # Web interface port

        # Get parameters
        self.agent_backend = self.get_parameter("agent_backend").value
        self.mock_robot = self.get_parameter("mock_robot").value
        self.use_planning = self.get_parameter("use_planning_agent").value
        self.enable_web = self.get_parameter("enable_web_interface").value
        self.web_port = self.get_parameter("web_port").value

        self.get_logger().info(f"Configuration:")
        self.get_logger().info(f"  Agent backend: {self.agent_backend}")
        self.get_logger().info(f"  Mock robot: {self.mock_robot}")
        self.get_logger().info(f"  Use planning: {self.use_planning}")
        self.get_logger().info(f"  Web interface: {self.enable_web}")
        if self.enable_web:
            self.get_logger().info(f"  Web port: {self.web_port}")

        # Check DIMOS availability
        if not DIMOS_AVAILABLE:
            self.get_logger().error(f"DIMOS framework not available: {IMPORT_ERROR}")
            self.get_logger().error("Please ensure dimos-unitree is in PYTHONPATH")
            raise RuntimeError("DIMOS not available")

        # Diagnostic: Check what topics are visible to this node
        self._log_topic_diagnostics()

        # Initialize robot interface
        self.get_logger().info("Initializing robot interface...")

        # Log connection mode for diagnostics
        conn_type = os.getenv("CONN_TYPE", "cyclonedds")
        self.get_logger().info(f"Connection type: {conn_type}")
        if conn_type == "webrtc":
            self.get_logger().info(
                "WebRTC mode: High-level API commands (sit, stand, wave) enabled"
            )
        else:
            self.get_logger().warn(
                "CycloneDDS mode: High-level API commands NOT available"
            )
            self.get_logger().warn("Set CONN_TYPE=webrtc to enable DIMOS skills")

        try:
            # Initialize ROS control
            # Costmap topic uses Nav2's default which publishes to /local_costmap/costmap
            # webrtc_api_topic must match what go2_driver_node subscribes to (/webrtc_req)
            #
            # Note: We always use use_ros=True (ROS video/control provider)
            # CONN_TYPE env var controls the underlying communication protocol:
            # - CONN_TYPE=cyclonedds: Ethernet, direct motor control only
            # - CONN_TYPE=webrtc: WiFi, enables high-level API commands (required for DIMOS)
            ros_control = UnitreeROSControl(webrtc_api_topic="webrtc_req")

            # Get robot IP from environment
            robot_ip = os.getenv("GO2_IP", "192.168.1.103")

            # Initialize robot with ROS provider (use_ros=True is default)
            # The CONN_TYPE environment variable determines the underlying communication
            self.robot = UnitreeGo2(
                ros_control=ros_control,
                ip=robot_ip,  # Still needed for some internal checks
                # use_ros=True (default) - Always use ROS bridge for control
                # use_webrtc=False (default) - Don't use direct WebRTC (we use ROS bridge)
            )
            self.get_logger().info(f"Robot initialized (ip={robot_ip})")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize robot: {e}")
            raise

        # Initialize skill library
        self.get_logger().info("Initializing skill library...")
        self.skills = MyUnitreeSkills(robot=self.robot)
        self.get_logger().info(f"Loaded {len(self.skills.get())} skills")

        # Initialize agent
        self.get_logger().info(f"Initializing {self.agent_backend} agent...")
        self.agent: Optional[BaseAgent] = None
        self._init_agent()

        # Initialize web interface (optional)
        self.web: Optional[WebInterface] = None
        if self.enable_web:
            self._init_web_interface()

        # Create subscribers
        self.mission_sub = self.create_subscription(
            String, "mission_command", self.mission_callback, 10
        )

        # Create publishers
        self.status_pub = self.create_publisher(String, "mission_status", 10)

        self.get_logger().info("ShadowHound Mission Agent ready!")

    def _log_topic_diagnostics(self):
        """Log diagnostic information about visible topics and actions."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔍 TOPIC DIAGNOSTICS")
        self.get_logger().info("=" * 60)

        # Get all topics
        topic_names_and_types = self.get_topic_names_and_types()

        # Filter for robot-related topics
        robot_topics = [
            (name, types)
            for name, types in topic_names_and_types
            if any(
                keyword in name
                for keyword in [
                    "go2",
                    "camera",
                    "imu",
                    "odom",
                    "costmap",
                    "cmd_vel",
                    "webrtc",
                ]
            )
        ]

        if robot_topics:
            self.get_logger().info(f"Found {len(robot_topics)} robot-related topics:")
            for name, types in sorted(robot_topics):
                self.get_logger().info(f"  • {name} [{', '.join(types)}]")
        else:
            self.get_logger().warn(
                "⚠️  No robot topics found! Is the robot driver running?"
            )

        # Check for specific critical topics
        critical_topics = {
            "/go2_states": "Robot state data",
            "/camera/image_raw": "Camera feed",
            "/imu": "IMU data",
            "/odom": "Odometry",
            "/local_costmap/costmap": "Local costmap",
            "/cmd_vel_out": "Velocity commands",
        }

        self.get_logger().info("\nCritical topic status:")
        for topic, description in critical_topics.items():
            found = any(name == topic for name, _ in topic_names_and_types)
            status = "✅" if found else "❌"
            self.get_logger().info(f"  {status} {topic} ({description})")

        # List all action servers
        try:
            import time

            time.sleep(0.5)  # Give action servers time to advertise

            from rclpy.action import ActionClient
            from nav2_msgs.action import Spin

            # Check if /spin action exists
            spin_client = ActionClient(self, Spin, "spin")
            spin_available = spin_client.wait_for_server(timeout_sec=1.0)
            spin_client.destroy()

            self.get_logger().info("\nNav2 action server status:")
            status = "✅" if spin_available else "❌"
            self.get_logger().info(f"  {status} /spin action server")

        except Exception as e:
            self.get_logger().warn(f"Could not check action servers: {e}")

        self.get_logger().info("=" * 60)

    def _init_agent(self):
        """Initialize the DIMOS agent using AgentFactory."""
        try:
            # Determine agent type
            if self.use_planning:
                agent_type = "planning"
            else:
                agent_type = "openai"

            # Check for API key if using cloud backend
            if self.agent_backend == "cloud":
                api_key = os.getenv("OPENAI_API_KEY")
                if not api_key:
                    self.get_logger().warn(
                        "OPENAI_API_KEY not set, agent may not function"
                    )

            # Create agent using factory
            self.agent = AgentFactory.create(
                agent_type=agent_type,
                skills=self.skills,
                robot=self.robot,
                config={
                    "dev_name": "shadowhound",
                    "agent_type": "Mission",
                    "model": "gpt-4-turbo",  # Can be overridden via config
                },
            )

            self.get_logger().info(f"Agent initialized: {agent_type}")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize agent: {e}")
            raise

    def _init_web_interface(self):
        """Initialize web interface."""
        try:
            self.web = WebInterface(
                command_callback=self._execute_mission_from_web,
                port=self.web_port,
                logger=self.get_logger(),
            )
            self.web.start()

            self.get_logger().info("=" * 60)
            self.get_logger().info("🌐 Web Interface Ready")
            self.get_logger().info(f"   Dashboard: http://localhost:{self.web_port}/")
            self.get_logger().info(f"   Open in browser to control the robot!")
            self.get_logger().info("=" * 60)
        except Exception as e:
            self.get_logger().error(f"Failed to start web interface: {e}")
            self.web = None

    def _execute_mission_from_web(self, command: str) -> Dict[str, Any]:
        """Execute mission from web interface.

        This is called by the web interface when a command is submitted.
        Returns a dict with 'success' and 'message' keys.
        """
        self.get_logger().info(f"🌐 Web command: {command}")

        try:
            # Execute through agent using unified interface
            result: MissionResult = self.agent.execute_mission(command)

            # Broadcast the actual response to all web clients
            if self.web:
                if result.status == MissionStatus.SUCCESS:
                    msg = result.message or "Mission completed successfully"
                    response_preview = msg[:200] if len(msg) > 200 else msg
                    self.web.broadcast_sync(f"✅ {response_preview}")

                    # Log telemetry if available
                    if result.telemetry:
                        self.get_logger().info(f"Telemetry: {result.telemetry}")
                else:
                    self.web.broadcast_sync(f"FAILED: {command} - {result.message}")

            return {
                "success": result.status == MissionStatus.SUCCESS,
                "message": result.message or "No response",
                "skills_executed": len(result.skills_executed),
                "skills_failed": len(result.skills_failed),
            }

        except Exception as e:
            self.get_logger().error(f"Web mission failed: {e}")

            # Broadcast error to web clients
            if self.web:
                self.web.broadcast_sync(f"FAILED: {command} - {str(e)}")

            return {"success": False, "message": f"Mission failed: {str(e)}"}

    def mission_callback(self, msg: String):
        """Handle incoming mission commands."""
        command = msg.data
        self.get_logger().info(f"Received mission command: {command}")

        # Publish status
        status = String()
        status.data = f"EXECUTING: {command}"
        self.status_pub.publish(status)

        try:
            # Execute mission through agent using unified interface
            result: MissionResult = self.agent.execute_mission(command)

            # Publish result
            if result.status == MissionStatus.SUCCESS:
                msg_text = result.message or "Mission completed successfully"
                status.data = f"COMPLETED: {command} | Result: {msg_text}"
                self.get_logger().info(
                    f"Mission completed: {result.skills_executed} skills executed"
                )

                # Log telemetry if available
                if result.telemetry:
                    self.get_logger().info(f"Telemetry: {result.telemetry}")
            else:
                status.data = f"FAILED: {command} | Error: {result.message}"
                self.get_logger().error(
                    f"Mission failed: {result.message} "
                    f"({len(result.skills_failed)} skills failed)"
                )

            self.status_pub.publish(status)

            # Broadcast to web interface with actual response
            if self.web:
                if result.status == MissionStatus.SUCCESS:
                    msg_text = result.message or "Mission completed successfully"
                    response_preview = msg_text[:200] if len(msg_text) > 200 else msg_text
                    self.web.broadcast_sync(f"✅ {response_preview}")
                else:
                    self.web.broadcast_sync(f"FAILED: {command} - {result.message}")

        except Exception as e:
            self.get_logger().error(f"Mission failed: {e}")
            status.data = f"FAILED: {command} | Error: {str(e)}"
            self.status_pub.publish(status)

            # Broadcast to web interface
            if self.web:
                self.web.broadcast_sync(f"FAILED: {command} - {str(e)}")

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info("Shutting down mission agent...")

        # Stop web interface
        if self.web:
            try:
                self.web.stop()
            except Exception as e:
                self.get_logger().warn(f"Error stopping web interface: {e}")

        if self.agent:
            try:
                self.agent.dispose_all()
            except Exception as e:
                self.get_logger().warn(f"Error disposing agent: {e}")

        if self.robot:
            try:
                # Add any robot cleanup if needed
                pass
            except Exception as e:
                self.get_logger().warn(f"Error cleaning up robot: {e}")

        super().destroy_node()


def main(args=None):
    """Main entry point for the mission agent node."""
    rclpy.init(args=args)

    try:
        node = MissionAgentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
