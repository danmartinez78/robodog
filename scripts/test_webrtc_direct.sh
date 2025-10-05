#!/bin/bash
# ============================================================================
# Direct WebRTC Test Script
# ============================================================================
# 
# This script launches ONLY the go2_ros2_sdk driver in WebRTC mode
# and tests direct API commands to validate robot connectivity.
#
# Prerequisites:
#   - Robot connected to WiFi network
#   - GO2_IP set to robot's WiFi IP address
#   - CONN_TYPE=webrtc
#
# ============================================================================

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Direct WebRTC Test - go2_ros2_sdk Only                       ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Load test-specific environment file
TEST_ENV=".env.webrtc_test"
if [ ! -f "$TEST_ENV" ]; then
    echo -e "${RED}✗ Test environment file not found: $TEST_ENV${NC}"
    echo ""
    echo "Creating default .env.webrtc_test file..."
    cat > "$TEST_ENV" << 'EOF'
# WebRTC Test Environment
# UPDATE GO2_IP with your robot's WiFi IP address!

GO2_IP=192.168.1.103
CONN_TYPE=webrtc
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
EOF
    echo -e "${GREEN}✓ Created $TEST_ENV${NC}"
    echo ""
    echo -e "${YELLOW}Please edit $TEST_ENV and set your robot's WiFi IP address:${NC}"
    echo "  nano $TEST_ENV"
    echo ""
    echo "Find your robot's WiFi IP in the Unitree app:"
    echo "  Settings → Network Settings → WiFi IP"
    echo ""
    exit 1
fi

echo -e "${BLUE}Loading environment from: $TEST_ENV${NC}"
set -a
source "$TEST_ENV"
set +a
echo -e "${GREEN}✓ Environment loaded${NC}"
echo ""

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Set WebRTC mode
export CONN_TYPE=webrtc
export GO2_IP=${GO2_IP:-192.168.1.103}
# SDK uses ROBOT_IP internally, so export it from GO2_IP
export ROBOT_IP=$GO2_IP

echo -e "${YELLOW}Configuration:${NC}"
echo "  • CONN_TYPE: $CONN_TYPE"
echo "  • GO2_IP: $GO2_IP (robot WiFi address)"
echo "  • ROBOT_IP: $ROBOT_IP (for SDK compatibility)"
echo ""

# Verify robot connectivity
echo -e "${BLUE}[1/4] Verifying robot connectivity...${NC}"
if ping -c 2 -W 2 "$GO2_IP" &> /dev/null; then
    echo -e "${GREEN}✓ Robot reachable at $GO2_IP${NC}"
else
    echo -e "${RED}✗ Cannot reach robot at $GO2_IP${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Is robot powered on?"
    echo "  2. Is robot connected to WiFi?"
    echo "  3. Is GO2_IP correct? (check in Unitree app)"
    echo ""
    read -p "Continue anyway? [y/N]: " choice
    if [[ "$choice" != "y" && "$choice" != "Y" ]]; then
        exit 1
    fi
fi
echo ""

# Find launch file
echo -e "${BLUE}[2/4] Locating go2_ros2_sdk launch file...${NC}"
LAUNCH_FILE=""
if [ -f "launch/go2_sdk/robot.launch.py" ]; then
    LAUNCH_FILE="launch/go2_sdk/robot.launch.py"
elif [ -f "src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk/launch/robot.launch.py" ]; then
    LAUNCH_FILE="src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk/launch/robot.launch.py"
fi

if [ -z "$LAUNCH_FILE" ]; then
    echo -e "${RED}✗ Robot launch file not found${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Found: $LAUNCH_FILE${NC}"
echo ""

# Launch robot driver
echo -e "${BLUE}[3/4] Launching go2_ros2_sdk driver (WebRTC mode)...${NC}"
echo "  Press Ctrl+C to stop when ready to test"
echo ""
echo -e "${YELLOW}Launch command:${NC}"
echo "  ros2 launch $LAUNCH_FILE nav2:=false rviz2:=false"
echo ""

# Create log file
LOG_FILE="/tmp/webrtc_test_$(date +%Y%m%d_%H%M%S).log"
echo "  Log file: $LOG_FILE"
echo ""

read -p "Press Enter to launch driver..."
echo ""

# Launch in background
ros2 launch "$LAUNCH_FILE" nav2:=false rviz2:=false > "$LOG_FILE" 2>&1 &
DRIVER_PID=$!

echo -e "${GREEN}✓ Driver launched (PID: $DRIVER_PID)${NC}"
echo ""

# Wait for topics
echo -e "${BLUE}[4/4] Waiting for topics to appear...${NC}"
MAX_WAIT=30
WAITED=0

while [ $WAITED -lt $MAX_WAIT ]; do
    if ros2 topic list 2>/dev/null | grep -q "/go2_states"; then
        echo -e "${GREEN}✓ Robot topics detected!${NC}"
        break
    fi
    
    # Check if process died
    if ! kill -0 $DRIVER_PID 2>/dev/null; then
        echo -e "${RED}✗ Driver process died${NC}"
        echo "Last 30 lines of log:"
        tail -30 "$LOG_FILE"
        exit 1
    fi
    
    echo -n "."
    sleep 1
    WAITED=$((WAITED + 1))
done
echo ""

if [ $WAITED -ge $MAX_WAIT ]; then
    echo -e "${RED}✗ Timeout waiting for topics${NC}"
    echo "Check logs: $LOG_FILE"
    kill $DRIVER_PID 2>/dev/null || true
    exit 1
fi

# Show available topics
echo ""
echo -e "${YELLOW}Available topics:${NC}"
ros2 topic list | grep -E "(go2_|cmd_vel|webrtc)" | sed 's/^/  /'
echo ""

# Check for WebRTC-specific indicators
echo -e "${YELLOW}Checking for WebRTC connection indicators...${NC}"
if grep -q "webrtc\|WebRTC\|LocalSTA\|LocalAP" "$LOG_FILE"; then
    echo -e "${GREEN}✓ WebRTC-related messages found in logs${NC}"
    echo "Sample:"
    grep -i "webrtc\|LocalSTA\|LocalAP" "$LOG_FILE" | head -5 | sed 's/^/  /'
else
    echo -e "${YELLOW}⚠ No explicit WebRTC messages found${NC}"
    echo "This might be OK - check if CONN_TYPE was respected"
fi
echo ""

# Interactive test menu
echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Driver Running - Ready to Test Commands                      ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "Test commands (run in another terminal with: source .shadowhound_env):"
echo ""
echo -e "${YELLOW}1. Check robot state:${NC}"
echo "   ros2 topic echo /go2_states --once"
echo ""
echo -e "${YELLOW}2. Sit command (API 1009):${NC}"
echo '   ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \'
echo '     "{id: 0, topic: '"'"'rt/api/sport/request'"'"', api_id: 1009, parameter: '"'"''"'"', priority: 0}"'
echo ""
echo -e "${YELLOW}3. Stand command (API 1001):${NC}"
echo '   ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \'
echo '     "{id: 0, topic: '"'"'rt/api/sport/request'"'"', api_id: 1001, parameter: '"'"''"'"', priority: 0}"'
echo ""
echo -e "${YELLOW}4. Wave command (API 1021):${NC}"
echo '   ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \'
echo '     "{id: 0, topic: '"'"'rt/api/sport/request'"'"', api_id: 1021, parameter: '"'"''"'"', priority: 0}"'
echo ""
echo -e "${YELLOW}5. Monitor WebRTC requests:${NC}"
echo "   ros2 topic echo /webrtc_req"
echo ""
echo -e "${YELLOW}6. Check for bare DDS subscribers (WebRTC endpoint):${NC}"
echo "   ros2 topic info /webrtcreq --verbose"
echo "   (Should show _CREATED_BY_BARE_DDS_APP_ if WebRTC active)"
echo ""
echo -e "${GREEN}Driver is running in foreground. Press Ctrl+C to stop.${NC}"
echo ""

# Keep running and show logs
echo "=== Live Logs (last 50 lines) ==="
echo ""
tail -f -n 50 "$LOG_FILE"
