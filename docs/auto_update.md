# Repository Auto-Update Features

## Overview

The start script now automatically checks for updates in both the main repository and all submodules (including `dimos-unitree` and `go2_ros2_sdk`) before launching.

## Features Added

### 1. **Automatic Update Detection**
- Checks main repo against remote
- Checks all submodules for updates
- Shows how many commits behind/ahead
- Displays recent commit messages

### 2. **Smart Update Prompts**
- Interactive: Prompts to pull if updates available
- Auto mode: Pulls automatically without prompting
- Skip mode: Bypasses update check entirely

### 3. **Safe Handling of Local Changes**
- Detects uncommitted changes
- Offers to stash before pulling
- Optionally restores stashed changes after update

### 4. **Automatic Rebuild Trigger**
- Sets `FORCE_REBUILD=true` when code is pulled
- Ensures packages are rebuilt after updates

## Usage

### Start Script with Update Check

```bash
# Interactive mode (default) - prompts if updates available
./start.sh --dev

# Auto-update mode - pulls automatically
./start.sh --dev --auto-update

# Skip update check - faster startup
./start.sh --dev --skip-update
```

### Standalone Update Script

For when you just want to update repos without launching:

```bash
# Interactive update check
./scripts/update_repos.sh

# Automatic update (no prompts)
./scripts/update_repos.sh --auto

# Only fetch, don't pull
./scripts/update_repos.sh --fetch-only

# Only update submodules
./scripts/update_repos.sh --submodules
```

## Command-Line Options

### start.sh Options

| Option | Description |
|--------|-------------|
| `--skip-update` | Skip git repository update check |
| `--auto-update` | Automatically pull updates without prompting |
| `--dev` | Use development configuration |
| `--prod` | Use production configuration |
| `--mock` | Force mock robot mode |
| `--no-web` | Disable web interface |
| `--web-port N` | Set web port (default: 8080) |

### update_repos.sh Options

| Option | Description |
|--------|-------------|
| `--auto` | Automatically update without prompting |
| `--fetch-only` | Only fetch, don't pull changes |
| `--submodules` | Only update submodules, skip main repo |
| `--help` | Show help message |

## Workflow Examples

### Daily Development Workflow

```bash
# Morning: Pull latest changes from team
./scripts/update_repos.sh --auto

# Work on your changes...

# Afternoon: Test with latest code
./start.sh --dev --auto-update
```

### Working on go2_ros2_sdk Improvements

```bash
# 1. Make changes to go2_ros2_sdk
cd src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk
# ... edit files ...

# 2. Test locally
cd ~/shadowhound
./start.sh --dev --skip-update  # Don't pull over your changes

# 3. Commit your improvements
cd src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk
git add .
git commit -m "Improve lidar processing"
git push origin your-branch

# 4. Later, pull team's updates to other parts
./scripts/update_repos.sh  # Will show your go2_ros2_sdk is ahead
```

### CI/CD Pipeline

```bash
# Always pull latest before testing
./start.sh --auto-update --dev --no-web
```

## What Gets Updated

### Main Repository (`shadowhound`)
- Your mission agent code
- Launch files
- Configuration files
- Documentation
- Start scripts

### Submodule: `dimos-unitree`
- DIMOS framework
- Unitree robot interface
- Skills library
- Agent implementations

### Nested Submodule: `go2_ros2_sdk`
- Go2 driver node
- Robot interfaces (go2_interfaces)
- Lidar processor
- WebRTC communication

## Update Check Output

### Example: No Updates

```
── Repository Update Check ──────────────────────────────────
ℹ Current branch: feature/dimos-integration
ℹ Checking for updates from remote...
✓ Main repo is up to date
ℹ Checking submodules...
  ✓ dimos-unitree: up to date
  ✓ go2_ros2_sdk: up to date
```

### Example: Updates Available

```
── Repository Update Check ──────────────────────────────────
ℹ Current branch: feature/dimos-integration
ℹ Checking for updates from remote...
⚠ Main repo is 2 commit(s) behind remote
  Latest changes:
    d5ae2ba Format diagnostic code with black/isort
    ad48e43 Add topic visibility diagnostics

ℹ Checking submodules...
  ⚠️  dimos-unitree: 1 commit(s) behind
      a3f4b7d Fix costmap subscription QoS
  ✓ go2_ros2_sdk: up to date

⚠ Updates are available!

Pull latest changes from remote? [Y/n]:
```

### Example: Local Changes

```
⚠ You have uncommitted changes

 M src/shadowhound_mission_agent/mission_agent.py
 M scripts/check_topics.py

Stash changes before pulling? [Y/n]: y
✓ Changes stashed
ℹ Pulling latest changes...
✓ Main repo updated
✓ Submodules updated

Restore your stashed changes? [Y/n]: y
✓ Changes restored
```

## Troubleshooting

### Merge Conflicts After Pull

```bash
# If auto-update causes conflicts
git status  # See conflicted files

# Resolve conflicts manually, then:
git add <resolved-files>
git commit -m "Resolve merge conflicts"

# Or abort and go back to before update:
git merge --abort
```

### Submodule Detached HEAD

```bash
# If submodule shows "HEAD detached"
cd src/dimos-unitree
git checkout main  # or your branch
git pull origin main

cd ../..
git add src/dimos-unitree
git commit -m "Update submodule pointer"
```

### Update Script Hangs

```bash
# If update check takes too long (network issues)
# Use --skip-update flag:
./start.sh --dev --skip-update

# Or set short timeout in git config:
git config --global http.timeout 10
```

## Best Practices

1. **Start of Day**: Run `./scripts/update_repos.sh` to sync with team
2. **Before Committing**: Check you're not behind with `git fetch origin`
3. **CI/CD**: Use `--auto-update` for automated environments
4. **Development**: Use `--skip-update` when testing local changes
5. **Submodule Work**: Always commit and push submodule changes separately

## Integration with Other Tools

### With ROS2 Build

```bash
# Update and rebuild in one go
./start.sh --auto-update --dev
# Automatically rebuilds if code changed
```

### With Git Hooks

Create `.git/hooks/post-merge`:
```bash
#!/bin/bash
# Remind to rebuild after pulling
echo "⚠️  Code was updated - consider rebuilding:"
echo "   colcon build --symlink-install"
```

```bash
chmod +x .git/hooks/post-merge
```

## Related Documentation

- `docs/topic_diagnostics.md` - Topic visibility checking
- `README.md` - General setup
- `.env.development` - Development configuration
- `.env.production` - Production configuration
