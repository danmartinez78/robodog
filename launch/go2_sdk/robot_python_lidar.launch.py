#!/usr/bin/env python3
"""
Go2 SDK launch with Python lidar processor (no C++ dependency).
Suitable for systems where lidar_processor_cpp is not available.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with Python lidar processor."""

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

    # Core robot driver
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

    # Python lidar processor (no C++ dependencies)
    lidar_processor_node = Node(
        package="lidar_processor",
        executable="lidar_processor_node",
        name="lidar_processor_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_cloud_topic": "/lidar/points"},
            {"output_cloud_topic": "/lidar/filtered"},
            {"costmap_topic": "/local_costmap/costmap"},
        ],
    )

    # Static transforms
    base_to_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_tf",
        arguments=["0", "0", "0.44", "0", "0", "0", "base_link", "lidar_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=["0.1", "0", "0.35", "0", "0.1", "0", "base_link", "camera_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

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
            LogInfo(msg="Starting Go2 SDK with Python lidar processor"),
            LogInfo(msg=["Robot IP: ", robot_ip, ", Connection: ", conn_type]),
            go2_driver_node,
            lidar_processor_node,
            base_to_lidar_tf,
            base_to_camera_tf,
            map_to_odom_tf,
        ]
    )
