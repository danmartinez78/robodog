"""Launch file for ShadowHound Mission Agent."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for mission agent."""

    # Declare launch arguments
    agent_backend_arg = DeclareLaunchArgument(
        "agent_backend",
        default_value="cloud",
        description="Agent backend: cloud (OpenAI) or local",
    )

    mock_robot_arg = DeclareLaunchArgument(
        "mock_robot",
        default_value="true",
        description="Use mock robot connection (true/false)",
    )

    use_planning_arg = DeclareLaunchArgument(
        "use_planning_agent",
        default_value="false",
        description="Use planning agent for multi-step missions (true/false)",
    )

    # Mission agent node
    mission_agent_node = Node(
        package="shadowhound_mission_agent",
        executable="mission_agent",
        name="mission_agent",
        output="screen",
        parameters=[
            {
                "agent_backend": LaunchConfiguration("agent_backend"),
                "mock_robot": LaunchConfiguration("mock_robot"),
                "use_planning_agent": LaunchConfiguration("use_planning_agent"),
            }
        ],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            agent_backend_arg,
            mock_robot_arg,
            use_planning_arg,
            mission_agent_node,
        ]
    )
