# 🚀 ShadowHound Quick Reference

## Common Commands

### Check for Updates
```bash
# Check and prompt
./scripts/update_repos.sh

# Auto-update everything
./scripts/update_repos.sh --auto

# Just fetch, don't pull
./scripts/update_repos.sh --fetch-only
```

### Check Robot Topics
```bash
# Quick diagnostic
python3 scripts/check_topics.py

# Full test
./test_topic_visibility.sh
```

### Launch System
```bash
# Development mode with update check
./start.sh --dev

# Auto-update and launch
./start.sh --dev --auto-update

# Skip update check (testing local changes)
./start.sh --dev --skip-update

# Production mode
./start.sh --prod
```

### Build & Source
```bash
# Build specific packages
colcon build --packages-select shadowhound_mission_agent --symlink-install

# Build SDK packages
colcon build --packages-select go2_interfaces unitree_go go2_robot_sdk --symlink-install

# Source workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$HOME/shadowhound/src/dimos-unitree:$PYTHONPATH"
```

### Git Operations
```bash
# See current status
git status
git log --oneline --graph -5

# Update submodules
git submodule update --init --remote

# Work on go2_ros2_sdk
cd src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk
git checkout -b feature/my-improvement
# ... make changes ...
git commit -am "Description"
git push origin feature/my-improvement
```

## Environment Variables

```bash
# Robot connection
export GO2_IP=192.168.10.167

# ROS2 configuration  
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# DIMOS
export PYTHONPATH="$HOME/shadowhound/src/dimos-unitree:$PYTHONPATH"

# OpenAI
export OPENAI_API_KEY=sk-your-key-here
```

## Troubleshooting

### No Robot Topics?
```bash
# 1. Check driver is running
ros2 topic list | grep go2

# 2. Check diagnostics
python3 scripts/check_topics.py

# 3. Launch robot driver
ros2 launch go2_robot_sdk robot.launch.py
```

### Submodule Issues?
```bash
# Reset to tracking branch
cd src/dimos-unitree
git checkout main
git pull origin main

# Or update all submodules
cd ~/shadowhound
git submodule update --remote --merge
```

### Build Errors?
```bash
# Clean build
rm -rf build install log
colcon build --symlink-install

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Update Conflicts?
```bash
# See what changed
git status
git diff

# Stash your changes
git stash push -m "My local changes"

# Pull updates
git pull origin feature/dimos-integration

# Restore your changes
git stash pop
```

## File Locations

| Path | Description |
|------|-------------|
| `start.sh` | Main launch script with auto-update |
| `scripts/check_topics.py` | Topic diagnostic tool |
| `scripts/update_repos.sh` | Standalone repo updater |
| `test_topic_visibility.sh` | Full topic test |
| `src/shadowhound_mission_agent/` | Mission agent package |
| `src/dimos-unitree/` | DIMOS framework (submodule) |
| `src/dimos-unitree/.../go2_ros2_sdk/` | Robot SDK (nested submodule) |
| `launch/go2_sdk/` | Robot launch files |
| `docs/` | Documentation |
| `.env` | Environment configuration |

## Useful Aliases (Add to ~/.bashrc)

```bash
# ShadowHound shortcuts
alias sh-update='cd ~/shadowhound && ./scripts/update_repos.sh'
alias sh-topics='cd ~/shadowhound && python3 scripts/check_topics.py'
alias sh-dev='cd ~/shadowhound && ./start.sh --dev'
alias sh-build='cd ~/shadowhound && colcon build --symlink-install'
alias sh-source='source /opt/ros/humble/setup.bash && source ~/shadowhound/install/setup.bash && export PYTHONPATH="$HOME/shadowhound/src/dimos-unitree:$PYTHONPATH"'
```

## Documentation Links

- **Auto-Update**: `docs/auto_update.md`
- **Topic Diagnostics**: `docs/topic_diagnostics.md`
- **QoL Summary**: `docs/qol_improvements.md`
- **Main README**: `README.md`

## Emergency Commands

```bash
# Reset everything to remote
git fetch origin
git reset --hard origin/feature/dimos-integration
git submodule update --init --remote

# Nuclear option - clean rebuild
rm -rf build install log
git clean -fdx
./start.sh --dev --auto-update
```

## Status Check One-Liner

```bash
# See everything at once
echo "=== GIT STATUS ===" && git status -s && \
echo "=== TOPICS ===" && ros2 topic list | grep -E "(go2|camera)" | head -5 && \
echo "=== PACKAGES ===" && ls install/ 2>/dev/null | grep shadowhound || echo "Not built"
```
