#!/usr/bin/env python3
"""
Minimal Go2 SDK launch file - just the core robot driver.
Use this for basic robot connection without lidar/perception components.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate minimal launch description for Go2 SDK."""

    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value=os.getenv("ROBOT_IP", "192.168.1.103"),
        description="IP address of the Go2 robot",
    )

    conn_type_arg = DeclareLaunchArgument(
        "conn_type",
        default_value="webrtc",
        description="Connection type: webrtc or wifi",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    # Configuration
    robot_ip = LaunchConfiguration("robot_ip")
    conn_type = LaunchConfiguration("conn_type")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Core robot driver node
    go2_driver_node = Node(
        package="go2_robot_sdk",
        executable="go2_driver_node",
        name="go2_driver_node",
        output="screen",
        parameters=[
            {"robot_ip": robot_ip},
            {"conn_type": conn_type},
            {"use_sim_time": use_sim_time},
            # DimOS-compatible topic mappings
            {"odom_topic": "/odom"},
            {"cmd_vel_topic": "/cmd_vel_out"},
            {"pose_cmd_topic": "/pose_cmd"},
            {"go2_states_topic": "/go2_states"},
            {"imu_topic": "/imu"},
            {"webrtc_topic": "/webrtc_req"},
            {"camera_topic": "/camera/image_raw"},
            {"camera_compressed_topic": "/camera/compressed"},
            {"camera_info_topic": "/camera/camera_info"},
        ],
    )

    # Static transforms for sensor frames
    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=["0.1", "0", "0.35", "0", "0.1", "0", "base_link", "camera_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Map to odom transform
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            conn_type_arg,
            use_sim_time_arg,
            go2_driver_node,
            base_to_camera_tf,
            map_to_odom_tf,
        ]
    )
