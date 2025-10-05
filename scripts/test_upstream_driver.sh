#!/bin/bash
# Script to temporarily test with upstream go2_ros2_sdk driver
# Usage: ./test_upstream_driver.sh [restore]

set -e

DIMOS_SDK="$HOME/shadowhound/src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk"
BACKUP_DIR="$HOME/shadowhound/src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk.dimos_backup"
UPSTREAM_DIR="/tmp/upstream_go2_sdk"

if [ "$1" = "restore" ]; then
    echo "🔄 Restoring DIMOS fork..."
    if [ -d "$BACKUP_DIR" ]; then
        rm -rf "$DIMOS_SDK"
        mv "$BACKUP_DIR" "$DIMOS_SDK"
        echo "✅ DIMOS fork restored"
    else
        echo "❌ No backup found at $BACKUP_DIR"
        exit 1
    fi
    exit 0
fi

echo "========================================="
echo "Testing with Upstream go2_ros2_sdk"
echo "========================================="
echo ""

# Clone upstream if needed
if [ ! -d "$UPSTREAM_DIR" ]; then
    echo "📥 Cloning upstream go2_ros2_sdk..."
    git clone https://github.com/dimensionalOS/go2_ros2_sdk.git "$UPSTREAM_DIR"
fi

# Backup DIMOS fork
if [ -d "$BACKUP_DIR" ]; then
    echo "⚠️  Backup already exists, removing old backup..."
    rm -rf "$BACKUP_DIR"
fi

echo "💾 Backing up DIMOS fork..."
mv "$DIMOS_SDK" "$BACKUP_DIR"

# Symlink upstream
echo "🔗 Symlinking upstream driver..."
ln -s "$UPSTREAM_DIR" "$DIMOS_SDK"

echo ""
echo "✅ Upstream driver activated!"
echo ""
echo "📦 Rebuilding packages..."
cd ~/shadowhound
colcon build --packages-select go2_interfaces unitree_go go2_robot_sdk lidar_processor --symlink-install

echo ""
echo "========================================="
echo "Ready to Test"
echo "========================================="
echo ""
echo "🚀 Terminal 1 - Launch robot driver:"
echo "   source ~/shadowhound/install/setup.bash"
echo "   ros2 launch go2_robot_sdk robot.launch.py"
echo ""
echo "🤖 Terminal 2 - Launch mission agent:"
echo "   export PYTHONPATH=\"\$HOME/shadowhound/src/dimos-unitree:\$PYTHONPATH\""
echo "   source ~/shadowhound/install/setup.bash"
echo "   ros2 launch shadowhound_mission_agent mission_agent.launch.py mock_robot:=false"
echo ""
echo "📊 Terminal 3 - Monitor topics:"
echo "   ros2 topic list | grep -E 'camera|go2'"
echo "   ros2 topic hz /go2_states"
echo "   ros2 topic hz /camera/image_raw"
echo ""
echo "========================================="
echo "To restore DIMOS fork:"
echo "   $0 restore"
echo "========================================="
