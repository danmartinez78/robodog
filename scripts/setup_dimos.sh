#!/bin/bash
# Setup DIMOS with custom go2_ros2_sdk fork

set -e

echo "🔧 Configuring DIMOS to use danmartinez78/go2_ros2_sdk fork..."

cd /workspaces/shadowhound/src/dimos-unitree

# Update submodule URL to point to your fork
git config --file=.gitmodules submodule.dimos/robot/unitree/external/go2_ros2_sdk.url https://github.com/danmartinez78/go2_ros2_sdk.git
git config --file=.gitmodules submodule.dimos/robot/unitree/external/go2_ros2_sdk.branch robodan_dev

# Sync the changes
git submodule sync

# Initialize and update submodules
echo "📦 Initializing DIMOS submodules..."
git submodule update --init --recursive --remote

echo "✅ DIMOS setup complete!"
echo ""
echo "Submodule status:"
git submodule status
