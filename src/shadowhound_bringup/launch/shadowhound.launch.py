"""Main bringup launch file for ShadowHound system."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for complete ShadowHound system."""

    # Declare launch arguments
    mock_robot_arg = DeclareLaunchArgument(
        "mock_robot",
        default_value="true",
        description="Use mock robot connection (true/false)",
    )

    agent_backend_arg = DeclareLaunchArgument(
        "agent_backend",
        default_value="cloud",
        description="Agent backend: cloud (OpenAI) or local",
    )

    use_planning_arg = DeclareLaunchArgument(
        "use_planning_agent",
        default_value="true",  # Changed to true for multi-step sequential execution
        description="Use planning agent for multi-step missions (recommended)",
    )

    # Include mission agent launch
    mission_agent_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("shadowhound_mission_agent"),
                "launch",
                "mission_agent.launch.py",
            ]
        ),
        launch_arguments={
            "mock_robot": LaunchConfiguration("mock_robot"),
            "agent_backend": LaunchConfiguration("agent_backend"),
            "use_planning_agent": LaunchConfiguration("use_planning_agent"),
        }.items(),
    )

    return LaunchDescription(
        [
            mock_robot_arg,
            agent_backend_arg,
            use_planning_arg,
            mission_agent_launch,
        ]
    )
