#!/bin/bash
# Quick command tester for WebRTC API

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}════════════════════════════════════════════${NC}"
echo -e "${BLUE}  WebRTC API Command Tester${NC}"
echo -e "${BLUE}════════════════════════════════════════════${NC}"
echo ""

if [ -z "$1" ]; then
    echo "Usage: $0 <command>"
    echo ""
    echo "Available commands:"
    echo "  sit        - Sit down (API 1009)"
    echo "  stand      - Stand up (API 1001)"
    echo "  wave       - Wave paw (API 1021)"
    echo "  stretch    - Stretch pose (API 1023)"
    echo "  damping    - Damping mode (API 1004)"
    echo "  recovery   - Recovery stand (API 1005)"
    echo "  state      - Check robot state"
    echo "  monitor    - Monitor WebRTC requests"
    echo "  info       - Check WebRTC endpoint"
    echo ""
    exit 1
fi

CMD=$1

case $CMD in
    sit)
        echo -e "${YELLOW}Sending SIT command (API 1009)...${NC}"
        ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
          "{id: 0, topic: 'rt/api/sport/request', api_id: 1009, parameter: '', priority: 0}"
        echo -e "${GREEN}✓ Command sent! Robot should sit down.${NC}"
        ;;
    stand)
        echo -e "${YELLOW}Sending STAND command (API 1001)...${NC}"
        ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
          "{id: 0, topic: 'rt/api/sport/request', api_id: 1001, parameter: '', priority: 0}"
        echo -e "${GREEN}✓ Command sent! Robot should stand up.${NC}"
        ;;
    wave)
        echo -e "${YELLOW}Sending WAVE command (API 1021)...${NC}"
        ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
          "{id: 0, topic: 'rt/api/sport/request', api_id: 1021, parameter: '', priority: 0}"
        echo -e "${GREEN}✓ Command sent! Robot should wave paw.${NC}"
        ;;
    stretch)
        echo -e "${YELLOW}Sending STRETCH command (API 1023)...${NC}"
        ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
          "{id: 0, topic: 'rt/api/sport/request', api_id: 1023, parameter: '', priority: 0}"
        echo -e "${GREEN}✓ Command sent! Robot should stretch.${NC}"
        ;;
    damping)
        echo -e "${YELLOW}Sending DAMPING command (API 1004)...${NC}"
        ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
          "{id: 0, topic: 'rt/api/sport/request', api_id: 1004, parameter: '', priority: 0}"
        echo -e "${GREEN}✓ Command sent! Robot entering damping mode.${NC}"
        ;;
    recovery)
        echo -e "${YELLOW}Sending RECOVERY STAND command (API 1005)...${NC}"
        ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
          "{id: 0, topic: 'rt/api/sport/request', api_id: 1005, parameter: '', priority: 0}"
        echo -e "${GREEN}✓ Command sent! Robot should stand up from lying down.${NC}"
        ;;
    state)
        echo -e "${YELLOW}Checking robot state...${NC}"
        ros2 topic echo /go2_states --once
        ;;
    monitor)
        echo -e "${YELLOW}Monitoring WebRTC requests (Ctrl+C to stop)...${NC}"
        ros2 topic echo /webrtc_req
        ;;
    info)
        echo -e "${YELLOW}Checking WebRTC endpoint info...${NC}"
        echo ""
        echo "Topic: /webrtc_req (ROS2 publisher → driver)"
        ros2 topic info /webrtc_req --verbose
        echo ""
        echo "Topic: /webrtcreq (bare DDS subscriber on robot)"
        ros2 topic info /webrtcreq --verbose 2>&1 || echo "  (Not found - WebRTC may not be active)"
        ;;
    *)
        echo -e "${RED}Unknown command: $CMD${NC}"
        echo "Run without arguments to see available commands"
        exit 1
        ;;
esac
