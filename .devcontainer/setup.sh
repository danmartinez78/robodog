#!/bin/bash
set -euo pipefail

echo "Starting devcontainer setup..."

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ROS_DISTRO not set; defaulting to humble"
  export ROS_DISTRO=humble
fi

# Prevent ROS setup scripts from tripping `set -u` when AMENT_TRACE_SETUP_FILES is unset
: "${AMENT_TRACE_SETUP_FILES:=0}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

echo "ROS ${ROS_DISTRO} environment loaded"

# Ensure rosdep database is up to date
if [ ! -f "$HOME/.ros/rosdep_updated" ]; then
  echo "Updating rosdep database..."
  rosdep update
  touch "$HOME/.ros/rosdep_updated"
fi

# Refresh apt cache once to improve rosdep installations (if apt-get is available)
if command -v apt-get >/dev/null 2>&1; then
  if [ ! -f "$HOME/.ros/apt_cache_updated" ]; then
    echo "Refreshing apt package index..."
    sudo apt-get update
    touch "$HOME/.ros/apt_cache_updated"
  fi
fi

# Initialize workspace structure if it doesn't exist
if [ ! -d "/workspaces/shadowhound/src" ]; then
  echo "Creating ROS2 workspace structure..."
  mkdir -p /workspaces/shadowhound/src
fi

# Install workspace dependencies (best effort)
if [ -d /workspaces/shadowhound ]; then
  cd /workspaces/shadowhound
  echo "Resolving ROS dependencies (best effort)..."
  rosdep install --from-paths src --ignore-src -r -y || true
fi

# Initialize colcon mixins metadata (if not already present)
echo "Setting up colcon build system..."
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml 2>/dev/null || true
colcon mixin update default 2>/dev/null || true

# Set up git configuration if not already configured
if [ -z "$(git config --global user.name 2>/dev/null || true)" ]; then
  echo "Setting up default git configuration..."
  git config --global user.name "Developer"
  git config --global user.email "developer@shadowhound.local"
  git config --global core.autocrlf input
  git config --global pull.rebase false
fi

# Ensure proper permissions
echo "Setting up workspace permissions..."
# Get current user, fallback to 'vscode' if USER is unset
CURRENT_USER="${USER:-$(whoami)}"
sudo chown -R ${CURRENT_USER}:${CURRENT_USER} /workspaces/shadowhound 2>/dev/null || true

echo "Devcontainer setup complete."
echo "Use 'cb' to build the workspace with colcon"
echo "Use 'source-ws' to source the workspace setup after building"
