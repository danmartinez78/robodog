#!/bin/bash
# Test script to check topic visibility for mission agent

set -e

echo "========================================"
echo "Topic Visibility Test"
echo "========================================"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source /workspaces/shadowhound/install/setup.bash

# Set environment
export PYTHONPATH="/workspaces/shadowhound/src/dimos-unitree:$PYTHONPATH"
export ROBOT_IP="${ROBOT_IP:-192.168.10.167}"

echo "Environment:"
echo "  ROBOT_IP: $ROBOT_IP"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo ""

echo "Checking if robot driver is running..."
if ros2 topic list | grep -q "go2_states"; then
    echo "✅ Robot driver is running"
    echo ""
    echo "Robot topics:"
    ros2 topic list | grep -E "(go2|camera|imu|odom|costmap|cmd_vel|webrtc)" || echo "  (none found)"
    echo ""
    echo "Action servers:"
    ros2 action list | grep -E "(spin|navigate|backup)" || echo "  (none found)"
else
    echo "❌ Robot driver NOT running!"
    echo ""
    echo "Please start the robot driver first:"
    echo "  ros2 launch go2_robot_sdk robot.launch.py"
    echo ""
    exit 1
fi

echo ""
echo "========================================"
echo "Starting Mission Agent (diagnostics mode)"
echo "========================================"
echo ""

# Launch mission agent - it will show diagnostics and then attempt to initialize
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    mock_robot:=false \
    enable_web_interface:=false
