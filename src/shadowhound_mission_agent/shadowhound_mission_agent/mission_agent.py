#!/usr/bin/env python3
"""ShadowHound Mission Agent - Integrates DIMOS agents with ROS2.

This node provides a ROS2 interface to DIMOS agents for autonomous mission execution.
It bridges natural language commands with the robot's skill execution system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import logging
from typing import Optional

# Import DIMOS components
try:
    from dimos.agents.agent import OpenAIAgent
    from dimos.agents.planning_agent import PlanningAgent
    from dimos.robot.unitree import UnitreeGo2, UnitreeROSControl
    from dimos.robot.unitree.unitree_skills import UnitreeSkills
    from dimos.skills.skills import SkillLibrary
    DIMOS_AVAILABLE = True
except ImportError as e:
    DIMOS_AVAILABLE = False
    IMPORT_ERROR = str(e)


class MissionAgentNode(Node):
    """ROS2 node that executes missions using DIMOS agents."""

    def __init__(self):
        super().__init__('shadowhound_mission_agent')
        
        # Declare parameters
        self.declare_parameter('agent_backend', 'cloud')  # 'cloud' or 'local'
        self.declare_parameter('mock_robot', True)
        self.declare_parameter('use_planning_agent', False)
        
        # Get parameters
        self.agent_backend = self.get_parameter('agent_backend').value
        self.mock_robot = self.get_parameter('mock_robot').value
        self.use_planning = self.get_parameter('use_planning_agent').value
        
        # Check DIMOS availability
        if not DIMOS_AVAILABLE:
            self.get_logger().error(
                f'DIMOS framework not available: {IMPORT_ERROR}'
            )
            self.get_logger().error(
                'Please ensure dimos-unitree is in PYTHONPATH'
            )
            raise RuntimeError('DIMOS not available')
        
        # Initialize robot interface
        self.get_logger().info('Initializing robot interface...')
        try:
            ros_control = UnitreeROSControl(mock_connection=self.mock_robot)
            self.robot = UnitreeGo2(
                ros_control=ros_control,
                disable_video_stream=True  # Can be enabled when needed
            )
            self.get_logger().info(
                f'Robot initialized (mock={self.mock_robot})'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to initialize robot: {e}')
            raise
        
        # Initialize skill library
        self.get_logger().info('Initializing skill library...')
        self.skills = UnitreeSkills(robot=self.robot)
        self.get_logger().info(
            f'Loaded {len(self.skills.get())} skills'
        )
        
        # Initialize agent
        self.get_logger().info(
            f'Initializing {self.agent_backend} agent...'
        )
        self.agent: Optional[OpenAIAgent] = None
        self._init_agent()
        
        # Create subscribers
        self.mission_sub = self.create_subscription(
            String,
            'mission_command',
            self.mission_callback,
            10
        )
        
        # Create publishers
        self.status_pub = self.create_publisher(
            String,
            'mission_status',
            10
        )
        
        self.get_logger().info('ShadowHound Mission Agent ready!')
    
    def _init_agent(self):
        """Initialize the DIMOS agent based on configuration."""
        try:
            if self.use_planning:
                self.agent = PlanningAgent(
                    robot=self.robot,
                    dev_name="shadowhound",
                    agent_type="Planning"
                )
                self.get_logger().info('PlanningAgent initialized')
            else:
                # Basic OpenAI agent
                api_key = os.getenv('OPENAI_API_KEY')
                if not api_key:
                    self.get_logger().warn(
                        'OPENAI_API_KEY not set, agent may not function'
                    )
                
                self.agent = OpenAIAgent(
                    robot=self.robot,
                    dev_name="shadowhound",
                    agent_type="Mission",
                    skills=self.skills
                )
                self.get_logger().info('OpenAIAgent initialized')
        
        except Exception as e:
            self.get_logger().error(f'Failed to initialize agent: {e}')
            raise
    
    def mission_callback(self, msg: String):
        """Handle incoming mission commands."""
        command = msg.data
        self.get_logger().info(f'Received mission command: {command}')
        
        # Publish status
        status = String()
        status.data = f'EXECUTING: {command}'
        self.status_pub.publish(status)
        
        try:
            # Execute mission through agent
            if self.use_planning:
                # Planning agent creates and executes a plan
                result = self.agent.plan_and_execute(command)
            else:
                # Basic agent processes command directly
                result = self.agent.process_text(command)
            
            # Publish result
            status.data = f'COMPLETED: {command} | Result: {result}'
            self.status_pub.publish(status)
            self.get_logger().info(f'Mission completed: {result}')
        
        except Exception as e:
            self.get_logger().error(f'Mission failed: {e}')
            status.data = f'FAILED: {command} | Error: {str(e)}'
            self.status_pub.publish(status)
    
    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down mission agent...')
        
        if self.agent:
            try:
                self.agent.dispose_all()
            except Exception as e:
                self.get_logger().warn(f'Error disposing agent: {e}')
        
        if self.robot:
            try:
                # Add any robot cleanup if needed
                pass
            except Exception as e:
                self.get_logger().warn(f'Error cleaning up robot: {e}')
        
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
        print(f'Fatal error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
