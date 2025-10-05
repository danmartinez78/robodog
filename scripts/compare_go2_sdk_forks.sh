#!/bin/bash
# Script to compare go2_ros2_sdk upstream vs DIMOS fork
# Helps diagnose if driver changes are causing mission agent hang

set -e

UPSTREAM_URL="https://github.com/dimensionalOS/go2_ros2_sdk.git"
UPSTREAM_DIR="/tmp/upstream_go2_sdk"
DIMOS_SDK_DIR="$HOME/shadowhound/src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk"

echo "========================================="
echo "go2_ros2_sdk Fork Comparison Tool"
echo "========================================="
echo ""

# Clone upstream if not exists
if [ ! -d "$UPSTREAM_DIR" ]; then
    echo "📥 Cloning upstream go2_ros2_sdk..."
    git clone "$UPSTREAM_URL" "$UPSTREAM_DIR"
    echo "✅ Cloned to $UPSTREAM_DIR"
else
    echo "ℹ️  Upstream already cloned at $UPSTREAM_DIR"
    echo "   To refresh: rm -rf $UPSTREAM_DIR && $0"
fi

echo ""
echo "========================================="
echo "File Structure Comparison"
echo "========================================="
echo ""

# Compare directory structure
echo "📂 Files only in DIMOS fork:"
diff -qr "$UPSTREAM_DIR" "$DIMOS_SDK_DIR" | grep "Only in $DIMOS_SDK_DIR" | head -20

echo ""
echo "📂 Files only in upstream:"
diff -qr "$UPSTREAM_DIR" "$DIMOS_SDK_DIR" | grep "Only in $UPSTREAM_DIR" | head -20

echo ""
echo "========================================="
echo "Modified Python Files"
echo "========================================="
echo ""

# Find all differing Python files
DIFF_FILES=$(diff -qr "$UPSTREAM_DIR" "$DIMOS_SDK_DIR" | grep "\.py differ" | awk '{print $2}' || true)

if [ -z "$DIFF_FILES" ]; then
    echo "✅ No Python files differ between forks"
else
    echo "⚠️  Modified Python files:"
    echo "$DIFF_FILES" | while read -r file; do
        rel_path=${file#$UPSTREAM_DIR/}
        echo "  - $rel_path"
    done
fi

echo ""
echo "========================================="
echo "Key File Comparisons"
echo "========================================="
echo ""

# Compare critical files
CRITICAL_FILES=(
    "go2_robot_sdk/go2_robot_sdk/go2_driver_node.py"
    "go2_robot_sdk/launch/robot.launch.py"
    "go2_robot_sdk/launch/webrtc_web.launch.py"
    "go2_interfaces/msg/Go2State.msg"
)

for file in "${CRITICAL_FILES[@]}"; do
    if [ -f "$UPSTREAM_DIR/$file" ] && [ -f "$DIMOS_SDK_DIR/$file" ]; then
        echo "📄 Comparing: $file"
        diff_output=$(diff -u "$UPSTREAM_DIR/$file" "$DIMOS_SDK_DIR/$file" | head -50 || true)
        if [ -z "$diff_output" ]; then
            echo "   ✅ Files identical"
        else
            echo "   ⚠️  Files differ (showing first 50 lines):"
            echo "$diff_output" | sed 's/^/   /'
        fi
        echo ""
    else
        echo "📄 $file: ⚠️  Missing in one fork"
        echo ""
    fi
done

echo ""
echo "========================================="
echo "Launch File Topic Configuration"
echo "========================================="
echo ""

# Extract topic names from launch files
echo "📡 Topics configured in UPSTREAM robot.launch.py:"
grep -E "topic|Topic|camera|image" "$UPSTREAM_DIR/go2_robot_sdk/launch/robot.launch.py" 2>/dev/null | head -10 || echo "  (none found)"

echo ""
echo "📡 Topics configured in DIMOS robot.launch.py:"
grep -E "topic|Topic|camera|image" "$DIMOS_SDK_DIR/go2_robot_sdk/launch/robot.launch.py" 2>/dev/null | head -10 || echo "  (none found)"

echo ""
echo "========================================="
echo "Package Dependencies"
echo "========================================="
echo ""

# Compare package.xml dependencies
echo "📦 UPSTREAM dependencies:"
grep -E "<depend>|<exec_depend>" "$UPSTREAM_DIR/go2_robot_sdk/package.xml" 2>/dev/null | head -15 || echo "  (none found)"

echo ""
echo "📦 DIMOS fork dependencies:"
grep -E "<depend>|<exec_depend>" "$DIMOS_SDK_DIR/go2_robot_sdk/package.xml" 2>/dev/null | head -15 || echo "  (none found)"

echo ""
echo "========================================="
echo "Git History Comparison"
echo "========================================="
echo ""

# Check git commits
echo "📊 UPSTREAM last 5 commits:"
(cd "$UPSTREAM_DIR" && git log --oneline -5) || echo "  (not a git repo)"

echo ""
echo "📊 DIMOS fork last 5 commits:"
(cd "$DIMOS_SDK_DIR" && git log --oneline -5 2>/dev/null) || echo "  (not a git repo or no history)"

echo ""
echo "========================================="
echo "Summary & Recommendations"
echo "========================================="
echo ""

if [ -n "$DIFF_FILES" ]; then
    echo "⚠️  DIMOS fork has modifications to upstream"
    echo ""
    echo "🔍 Next steps:"
    echo "   1. Review modified Python files listed above"
    echo "   2. Test with upstream driver: scripts/test_upstream_driver.sh"
    echo "   3. Check if upstream has newer fixes: cd $UPSTREAM_DIR && git pull"
    echo "   4. Document DIMOS-specific changes in troubleshooting doc"
else
    echo "✅ DIMOS fork appears identical to upstream"
    echo ""
    echo "🔍 This suggests the issue is NOT in the ROS driver"
    echo "   Focus investigation on DIMOS robot interface code:"
    echo "   - dimos/robot/ros_control.py"
    echo "   - dimos/robot/unitree/unitree_ros_control.py"
    echo "   - dimos/robot/unitree/unitree_go2.py"
fi

echo ""
echo "========================================="
echo "Comparison complete!"
echo "Full results saved to: /tmp/go2_sdk_comparison.log"
echo "========================================="
