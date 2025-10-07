#!/bin/bash
# Diagnostic script for camera feed troubleshooting

echo "=========================================="
echo "ShadowHound Camera Feed Diagnostics"
echo "=========================================="
echo ""

# Source workspace
cd /home/daniel/shadowhound
source install/setup.bash

echo "1. Checking ROS environment..."
echo "   ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "   RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo ""

echo "2. Listing all camera-related topics..."
ros2 topic list | grep -i camera
echo ""

echo "3. Checking if /camera/compressed is publishing..."
timeout 2 ros2 topic hz /camera/compressed 2>&1 || echo "   ⚠️  Topic not publishing or timed out"
echo ""

echo "4. Checking topic info and publishers..."
ros2 topic info /camera/compressed 2>&1 || echo "   ⚠️  Topic does not exist"
echo ""

echo "5. Checking if mission_agent is running..."
if pgrep -f "mission_agent" > /dev/null; then
    echo "   ✅ mission_agent is running"
    echo "   Process: $(pgrep -f mission_agent)"
else
    echo "   ❌ mission_agent is NOT running"
fi
echo ""

echo "6. Checking web interface on port 8080..."
if curl -s http://localhost:8080/api/health > /dev/null; then
    echo "   ✅ Web interface is responding"
    curl -s http://localhost:8080/api/health | python3 -m json.tool
else
    echo "   ❌ Web interface is NOT responding on port 8080"
fi
echo ""

echo "7. Checking if camera topic has BEST_EFFORT QoS..."
ros2 topic info /camera/compressed -v 2>&1 | grep -A 10 "QoS profile:" || echo "   Could not determine QoS"
echo ""

echo "=========================================="
echo "Diagnostic complete!"
echo "=========================================="
