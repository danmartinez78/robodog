#!/bin/bash
# ============================================================================
# Kill All ROS2 Nodes
# ============================================================================
#
# Emergency cleanup script to kill all ROS2 nodes and processes.
# Use this if the start script's cleanup didn't work or you need
# to forcefully clean up a stuck system.
#
# Usage:
#   ./scripts/kill_all_ros.sh
#
# ============================================================================

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}  🧹 ROS2 Emergency Cleanup${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Show what's running before cleanup
echo -e "${BLUE}Current ROS2 processes:${NC}"
ps aux | grep -E "ros2|go2|nav2|slam|rviz2|foxglove" | grep -v grep | head -10
echo ""

read -p "Kill all ROS2 processes? [y/N]: " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo "Cancelled."
    exit 0
fi

echo ""
echo -e "${YELLOW}Killing processes...${NC}"

# ShadowHound specific
pkill -f "shadowhound_mission_agent" 2>/dev/null && echo "  ✓ shadowhound_mission_agent"
pkill -f "mission_agent.launch" 2>/dev/null && echo "  ✓ mission_agent.launch"

# Go2 robot driver
pkill -f "go2_driver_node" 2>/dev/null && echo "  ✓ go2_driver_node"
pkill -f "robot.launch" 2>/dev/null && echo "  ✓ robot.launch"
pkill -f "go2_rviz2" 2>/dev/null && echo "  ✓ rviz2"

# Nav2 nodes
pkill -f "behavior_server" 2>/dev/null && echo "  ✓ behavior_server"
pkill -f "controller_server" 2>/dev/null && echo "  ✓ controller_server"
pkill -f "planner_server" 2>/dev/null && echo "  ✓ planner_server"
pkill -f "bt_navigator" 2>/dev/null && echo "  ✓ bt_navigator"
pkill -f "waypoint_follower" 2>/dev/null && echo "  ✓ waypoint_follower"
pkill -f "velocity_smoother" 2>/dev/null && echo "  ✓ velocity_smoother"
pkill -f "smoother_server" 2>/dev/null && echo "  ✓ smoother_server"

# SLAM
pkill -f "slam_toolbox" 2>/dev/null && echo "  ✓ slam_toolbox"

# Visualization
pkill -f "foxglove_bridge" 2>/dev/null && echo "  ✓ foxglove_bridge"
pkill -f "rviz2" 2>/dev/null && echo "  ✓ rviz2"

# DIMOS-specific nodes
pkill -f "pointcloud_aggregator" 2>/dev/null && echo "  ✓ pointcloud_aggregator"
pkill -f "tts_node" 2>/dev/null && echo "  ✓ tts_node"

# Generic ROS2 launch processes
pkill -f "ros2 launch" 2>/dev/null && echo "  ✓ ros2 launch processes"

# Wait for processes to die
sleep 2

# Final aggressive cleanup - SIGKILL any stragglers
echo ""
echo -e "${YELLOW}Force killing any remaining processes...${NC}"
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "go2" 2>/dev/null || true
pkill -9 -f "nav2" 2>/dev/null || true

sleep 1

echo ""
echo -e "${BLUE}Remaining ROS2 processes:${NC}"
remaining=$(ps aux | grep -E "ros2|go2|nav2|slam|rviz2|foxglove|pointcloud|tts_node" | grep -v grep | grep -v "kill_all_ros")
if [ -z "$remaining" ]; then
    echo -e "${GREEN}  ✓ All clean!${NC}"
else
    echo "$remaining"
    echo ""
    echo -e "${YELLOW}  ⚠ Some processes remain. You may need to kill them manually.${NC}"
fi

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
