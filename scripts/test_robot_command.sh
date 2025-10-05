#!/bin/bash
# Test script to send robot commands directly via ROS2 CLI
# Usage: ./test_robot_command.sh [command_name]

set -e

# Source ROS2 environment if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

# Map command names to API IDs
declare -A COMMANDS=(
    ["sit"]=1009
    ["stand"]=1001
    ["hello"]=1016  # Wave
    ["stretch"]=1011
    ["dance1"]=1021
    ["dance2"]=1022
)

# Get command from argument or default to "sit"
COMMAND_NAME=${1:-sit}
API_ID=${COMMANDS[$COMMAND_NAME]}

if [ -z "$API_ID" ]; then
    echo "❌ Unknown command: $COMMAND_NAME"
    echo "Available commands:"
    for cmd in "${!COMMANDS[@]}"; do
        echo "  - $cmd (API ID: ${COMMANDS[$cmd]})"
    done
    exit 1
fi

echo "🐕 Sending command: $COMMAND_NAME (API ID: $API_ID)"
echo "   Topic: /webrtc_req"
echo ""

# Construct the message
# WebRtcReq fields: id, topic, api_id, parameter, priority
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq "{
  id: 0,
  topic: 'rt/api/sport/request',
  api_id: $API_ID,
  parameter: '',
  priority: 0
}"

echo ""
echo "✅ Command sent! Watch the robot to see if it responds."
echo ""
echo "To debug, monitor the topic:"
echo "  ros2 topic echo /rt/api/sport/request"
echo ""
echo "To check if robot driver is running:"
echo "  ros2 node list | grep go2"
