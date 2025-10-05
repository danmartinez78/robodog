#!/bin/bash
# ============================================================================
# WebRTC Test Setup Helper
# ============================================================================
#
# This script helps you configure .env.webrtc_test for testing
#
# ============================================================================

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

ENV_FILE=".env.webrtc_test"

echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  WebRTC Test Configuration Helper                             ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if file exists
if [ -f "$ENV_FILE" ]; then
    echo -e "${BLUE}Current configuration in $ENV_FILE:${NC}"
    echo ""
    grep -E "^GO2_IP=|^CONN_TYPE=|^ROS_DOMAIN_ID=" "$ENV_FILE" | sed 's/^/  /'
    echo ""
    read -p "Reconfigure? [y/N]: " reconfigure
    if [[ "$reconfigure" != "y" && "$reconfigure" != "Y" ]]; then
        echo "Configuration unchanged."
        exit 0
    fi
fi

echo -e "${YELLOW}Let's configure the WebRTC test environment!${NC}"
echo ""

# Get robot WiFi IP
echo -e "${BLUE}Step 1: Robot WiFi IP Address${NC}"
echo ""
echo "Find your robot's WiFi IP in the Unitree app:"
echo "  1. Open Unitree app"
echo "  2. Connect to robot (WiFi or hotspot)"
echo "  3. Go to: Settings → Network Settings"
echo "  4. Look for WiFi IP address"
echo ""
echo "Common ranges:"
echo "  • Home WiFi: 192.168.1.x or 192.168.0.x"
echo "  • Robot hotspot: 192.168.12.x"
echo ""
echo "⚠️  Do NOT use Ethernet IP (usually 192.168.123.161 or 192.168.10.167)"
echo ""

# Try to detect current IP from main .env
CURRENT_IP=""
if [ -f ".env" ]; then
    CURRENT_IP=$(grep "^GO2_IP=" .env 2>/dev/null | cut -d= -f2)
fi

if [ -n "$CURRENT_IP" ]; then
    read -p "Robot WiFi IP [$CURRENT_IP]: " robot_ip
    robot_ip=${robot_ip:-$CURRENT_IP}
else
    read -p "Robot WiFi IP [192.168.1.103]: " robot_ip
    robot_ip=${robot_ip:-192.168.1.103}
fi

# Validate IP format
if ! echo "$robot_ip" | grep -qE '^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$'; then
    echo -e "${RED}Invalid IP format. Please use format: xxx.xxx.xxx.xxx${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Using IP: $robot_ip${NC}"
echo ""

# Test connectivity
echo -e "${BLUE}Step 2: Testing Robot Connectivity${NC}"
echo ""
echo -n "Pinging $robot_ip... "
if ping -c 2 -W 2 "$robot_ip" &> /dev/null; then
    echo -e "${GREEN}✓ Robot is reachable!${NC}"
else
    echo -e "${RED}✗ Cannot reach robot${NC}"
    echo ""
    echo -e "${YELLOW}Troubleshooting:${NC}"
    echo "  1. Is robot powered on?"
    echo "  2. Is robot connected to WiFi?"
    echo "  3. Are you on the same network as robot?"
    echo "  4. Is the IP address correct?"
    echo ""
    read -p "Continue anyway? [y/N]: " continue_choice
    if [[ "$continue_choice" != "y" && "$continue_choice" != "Y" ]]; then
        exit 1
    fi
fi
echo ""

# ROS Domain ID
echo -e "${BLUE}Step 3: ROS Domain ID${NC}"
echo ""
echo "ROS_DOMAIN_ID isolates ROS2 networks."
echo "Use 0 unless you have multiple ROS2 systems on your network."
echo ""
read -p "ROS Domain ID [0]: " ros_domain
ros_domain=${ros_domain:-0}
echo -e "${GREEN}✓ Using ROS_DOMAIN_ID: $ros_domain${NC}"
echo ""

# Write configuration
echo -e "${BLUE}Step 4: Writing Configuration${NC}"
echo ""

cat > "$ENV_FILE" << EOF
# ============================================================================
# WebRTC Direct Test Environment
# ============================================================================
#
# Auto-configured by setup_webrtc_test.sh on $(date)
#
# ============================================================================

# ----------------------------------------------------------------------------
# Robot Configuration
# ----------------------------------------------------------------------------

# Robot WiFi IP address
GO2_IP=$robot_ip

# Connection type - MUST be webrtc for this test
CONN_TYPE=webrtc

# ----------------------------------------------------------------------------
# ROS2 Configuration
# ----------------------------------------------------------------------------

# ROS Domain ID
ROS_DOMAIN_ID=$ros_domain

# ROS2 Middleware (CycloneDDS recommended for performance)
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ----------------------------------------------------------------------------
# Optional: Robot Token (if your robot requires authentication)
# ----------------------------------------------------------------------------

# Robot authentication token (usually not needed for local network)
# ROBOT_TOKEN=

# ----------------------------------------------------------------------------
# Notes
# ----------------------------------------------------------------------------
#
# To reconfigure: ./scripts/setup_webrtc_test.sh
# To test: ./scripts/test_webrtc_direct.sh
#
# ============================================================================
EOF

echo -e "${GREEN}✓ Configuration saved to: $ENV_FILE${NC}"
echo ""

# Show summary
echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  Configuration Summary                                         ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "  • Robot IP: $robot_ip"
echo "  • Connection: webrtc (WiFi)"
echo "  • ROS Domain: $ros_domain"
echo "  • Config file: $ENV_FILE"
echo ""

# Next steps
echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  ✓ Setup Complete! Ready to Test                              ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "Next steps:"
echo ""
echo "  1. Verify robot is on WiFi network"
echo "  2. Run test: ${YELLOW}./scripts/test_webrtc_direct.sh${NC}"
echo "  3. In another terminal:"
echo "     ${YELLOW}source .shadowhound_env${NC}"
echo "     ${YELLOW}./scripts/test_commands.sh sit${NC}"
echo ""
echo "Documentation: ${BLUE}docs/WEBRTC_DIRECT_TEST.md${NC}"
echo ""
