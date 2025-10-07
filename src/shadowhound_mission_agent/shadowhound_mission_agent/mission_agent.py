#!/usr/bin/env python3
"""ShadowHound Mission Agent - ROS2 wrapper for mission execution.

This node provides a ROS2 interface to the MissionExecutor, handling:
- ROS node lifecycle and parameter management  
- Topic subscriptions (mission commands) and publications (status)
- Web interface integration
- ROS logging bridge

The actual mission execution logic is in MissionExecutor (pure Python).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import os
from typing import Optional, Dict, Any
from .web_interface import WebInterface
from .mission_executor import MissionExecutor, MissionExecutorConfig


class MissionAgentNode(Node):
    """ROS2 node providing interface to MissionExecutor.
    
    This is a thin wrapper that handles ROS-specific concerns while
    delegating all business logic to MissionExecutor.
    """

    def __init__(self):
        super().__init__("shadowhound_mission_agent")

        # Declare ROS parameters
        self.declare_parameter("agent_backend", "cloud")  # 'cloud' or 'local'
        self.declare_parameter(
            "use_planning_agent", False
        )  # Use PlanningAgent instead of OpenAIAgent
        self.declare_parameter("enable_web_interface", True)  # Enable web dashboard
        self.declare_parameter("web_port", 8080)  # Web interface port
        self.declare_parameter("robot_ip", "192.168.1.103")  # Robot IP address
        self.declare_parameter("webrtc_api_topic", "webrtc_req")  # WebRTC API topic
        self.declare_parameter("agent_model", "gpt-4-turbo")  # LLM model

        # Get parameters
        agent_backend = self.get_parameter("agent_backend").value
        use_planning = self.get_parameter("use_planning_agent").value
        enable_web = self.get_parameter("enable_web_interface").value
        web_port = self.get_parameter("web_port").value
        robot_ip = self.get_parameter("robot_ip").value
        webrtc_api_topic = self.get_parameter("webrtc_api_topic").value
        agent_model = self.get_parameter("agent_model").value

        self.get_logger().info("Configuration:")
        self.get_logger().info(f"  Agent backend: {agent_backend}")
        self.get_logger().info(f"  Use planning: {use_planning}")
        self.get_logger().info(f"  Web interface: {enable_web}")
        if enable_web:
            self.get_logger().info(f"  Web port: {web_port}")
        self.get_logger().info(f"  Robot IP: {robot_ip}")
        self.get_logger().info(f"  Agent model: {agent_model}")

        # Log connection diagnostics
        self._log_connection_diagnostics()
        self._log_topic_diagnostics()

        # Create MissionExecutor with configuration
        self.get_logger().info("Creating MissionExecutor...")
        config = MissionExecutorConfig(
            agent_backend=agent_backend,
            use_planning_agent=use_planning,
            robot_ip=robot_ip,
            webrtc_api_topic=webrtc_api_topic,
            agent_model=agent_model,
        )

        # Create mission executor (uses ROS logging via this node's logger)
        # Note: Named mission_executor to avoid conflict with ROS node's executor attribute
        self.mission_executor = MissionExecutor(config, logger=self.get_logger())

        # Initialize mission executor
        self.get_logger().info("Initializing MissionExecutor...")
        self.mission_executor.initialize()
        self.get_logger().info("MissionExecutor ready!")

        # Initialize web interface (optional)
        self.web: Optional[WebInterface] = None
        if enable_web:
            self._init_web_interface(web_port)

        # Create ROS subscribers
        self.mission_sub = self.create_subscription(
            String, "mission_command", self.mission_callback, 10
        )

        # Subscribe to camera feed if web interface is enabled
        self.camera_sub = None
        if enable_web:
            try:
                from sensor_msgs.msg import Image
                from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
                
                # Use BEST_EFFORT QoS to match camera publisher
                qos_profile = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1
                )
                
                self.camera_sub = self.create_subscription(
                    Image,
                    "/camera/image_raw",
                    self._camera_callback,
                    qos_profile
                )
                self.get_logger().info("📷 Subscribed to camera feed: /camera/image_raw")
            except Exception as e:
                self.get_logger().warn(f"Failed to subscribe to camera: {e}")

        # Create ROS publishers
        self.status_pub = self.create_publisher(String, "mission_status", 10)

        # Create timer for diagnostics updates (2 Hz)
        self.diagnostics_timer = self.create_timer(0.5, self.update_diagnostics)

        self.get_logger().info("ShadowHound Mission Agent ready!")

    def _log_connection_diagnostics(self):
        """Log connection type diagnostics."""
        conn_type = os.getenv("CONN_TYPE", "cyclonedds")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔌 CONNECTION DIAGNOSTICS")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Connection type: {conn_type}")

        if conn_type == "webrtc":
            self.get_logger().info(
                "✅ WebRTC mode: High-level API commands (sit, stand, wave) enabled"
            )
        else:
            self.get_logger().warn(
                "⚠️  CycloneDDS mode: High-level API commands NOT available"
            )
            self.get_logger().warn("   Set CONN_TYPE=webrtc to enable DIMOS skills")
        self.get_logger().info("=" * 60)

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

        self.get_logger().info("=" * 60)

    def _init_web_interface(self, port: int):
        """Initialize web interface."""
        try:
            self.web = WebInterface(
                command_callback=self._execute_mission_from_web,
                port=port,
                logger=self.get_logger(),
            )
            self.web.start()

            self.get_logger().info("=" * 60)
            self.get_logger().info("🌐 Web Interface Ready")
            self.get_logger().info(f"   Dashboard: http://localhost:{port}/")
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
            # Delegate to MissionExecutor
            response = self.mission_executor.execute_mission(command)

            # Broadcast the response to all web clients
            if self.web:
                response_preview = response[:200] if len(response) > 200 else response
                self.web.broadcast_sync(f"✅ {response_preview}")

            return {"success": True, "message": response}

        except Exception as e:
            self.get_logger().error(f"Web mission failed: {e}")

            # Broadcast error to web clients
            if self.web:
                self.web.broadcast_sync(f"❌ FAILED: {command} - {str(e)}")
                self.web.add_terminal_line(f"❌ FAILED: {command} - {str(e)}")

            return {"success": False, "message": f"Mission failed: {str(e)}"}

    def mission_callback(self, msg: String):
        """Handle incoming mission commands from ROS topic."""
        command = msg.data
        self.get_logger().info(f"📨 Received mission command: {command}")

        # Publish status
        status = String()
        status.data = f"EXECUTING: {command}"
        self.status_pub.publish(status)

        try:
            # Delegate to MissionExecutor
            response = self.mission_executor.execute_mission(command)

            # Publish result
            status.data = f"COMPLETED: {command} | Result: {response[:100]}"
            self.status_pub.publish(status)
            self.get_logger().info(f"✅ Mission completed: {response[:100]}...")

            # Broadcast to web interface
            if self.web:
                response_preview = response[:200] if len(response) > 200 else response
                self.web.broadcast_sync(f"✅ {response_preview}")

        except Exception as e:
            self.get_logger().error(f"❌ Mission failed: {e}")
            status.data = f"FAILED: {command} | Error: {str(e)}"
            self.status_pub.publish(status)

            # Broadcast to web interface
            if self.web:
                self.web.broadcast_sync(f"❌ FAILED: {command} - {str(e)}")
                self.web.add_terminal_line(f"❌ FAILED: {command} - {str(e)}")

    def _camera_callback(self, msg):
        """Handle incoming camera images and forward to web interface."""
        if not self.web:
            return
        
        try:
            import base64
            import cv2
            import numpy as np
            
            # Convert ROS Image message to numpy array
            # Assuming RGB8 or BGR8 encoding (common for cameras)
            if msg.encoding == 'rgb8':
                image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'bgr8':
                image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            else:
                # Unsupported encoding - skip
                if not hasattr(self, '_encoding_warned'):
                    self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
                    self._encoding_warned = True
                return
            
            # Encode to JPEG
            success, buffer = cv2.imencode('.jpg', cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR), 
                                           [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not success:
                return
            
            # Convert to base64
            image_base64 = base64.b64encode(buffer.tobytes()).decode('utf-8')
            
            # Send to web interface via WebSocket
            # Format: CAMERA_FRAME:<base64_data>
            self.web.broadcast_sync(f"CAMERA_FRAME:{image_base64}")
            
        except Exception as e:
            # Only log errors occasionally to avoid spam
            if not hasattr(self, '_camera_error_count'):
                self._camera_error_count = 0
            self._camera_error_count += 1
            if self._camera_error_count % 100 == 0:
                self.get_logger().warn(f"Camera callback error (x{self._camera_error_count}): {e}")
    
    def update_diagnostics(self):
        """Update diagnostics information for web UI."""
        if not self.web:
            return
        
        try:
            # Get available topics
            topic_names_and_types = self.get_topic_names_and_types()
            robot_topics = [
                name for name, types in topic_names_and_types
                if any(keyword in name for keyword in [
                    "go2", "camera", "imu", "odom", "costmap", "cmd_vel", "webrtc"
                ])
            ]
            
            # Get action servers (simplified - just check if nav2 actions exist)
            action_servers = []
            if any("/navigate_to_pose" in name for name, _ in topic_names_and_types):
                action_servers.append("navigate_to_pose")
            if any("/spin" in name for name, _ in topic_names_and_types):
                action_servers.append("spin")
            
            # Update web interface
            self.web.update_diagnostics({
                "robot_mode": "operational",  # TODO: Get actual mode from robot state
                "topics_available": robot_topics,
                "action_servers": action_servers
            })
            
        except Exception as e:
            self.get_logger().debug(f"Diagnostics update error: {e}")
            if self._camera_error_count % 100 == 0:
                self.get_logger().warn(f"Camera callback error (x{self._camera_error_count}): {e}")

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info("Shutting down mission agent...")

        # Stop web interface
        if self.web:
            try:
                self.web.stop()
            except Exception as e:
                self.get_logger().warn(f"Error stopping web interface: {e}")

        # Clean up mission executor
        if self.mission_executor:
            try:
                self.mission_executor.cleanup()
            except Exception as e:
                self.get_logger().warn(f"Error cleaning up mission executor: {e}")

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
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
