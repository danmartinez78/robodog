#!/usr/bin/env python3
"""
ShadowHound Full System Launch
Launches both Go2 ROS SDK and Mission Agent in proper sequence.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for full ShadowHound system."""

    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value=os.getenv("GO2_IP", "192.168.1.103"),
        description="IP address of the Go2 robot",
    )

    mock_robot_arg = DeclareLaunchArgument(
        "mock_robot",
        default_value="true",
        description="Use mock robot connection (skips Go2 SDK launch)",
    )

    agent_backend_arg = DeclareLaunchArgument(
        "agent_backend",
        default_value="cloud",
        description="Agent backend: cloud (OpenAI) or local",
    )

    use_planning_arg = DeclareLaunchArgument(
        "use_planning_agent",
        default_value="false",
        description="Use planning agent for multi-step missions",
    )

    # Get launch configurations
    robot_ip = LaunchConfiguration("robot_ip")
    mock_robot = LaunchConfiguration("mock_robot")
    agent_backend = LaunchConfiguration("agent_backend")
    use_planning = LaunchConfiguration("use_planning_agent")

    # Launch Go2 SDK (only if NOT in mock mode)
    go2_sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("go2_robot_sdk"),
                    "launch",
                    "robot_dimos.launch.py",
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "conn_type": "webrtc",
            "use_optimized_lidar": "true",
        }.items(),
        condition=UnlessCondition(mock_robot),
    )

    # Launch Mission Agent
    mission_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("shadowhound_mission_agent"),
                    "launch",
                    "mission_agent.launch.py",
                )
            ]
        ),
        launch_arguments={
            "agent_backend": agent_backend,
            "mock_robot": mock_robot,
            "use_planning_agent": use_planning,
        }.items(),
    )

    return LaunchDescription(
        [
            # Arguments
            robot_ip_arg,
            mock_robot_arg,
            agent_backend_arg,
            use_planning_arg,
            # Launch files
            go2_sdk_launch,  # Only launches if mock_robot=false
            mission_agent_launch,
        ]
    )
