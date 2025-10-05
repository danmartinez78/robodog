# Orchestrated Launch System

## Overview

The start script now orchestrates the complete system launch in three validated stages:

1. **Launch Robot Driver** (go2_ros2_sdk) in background
2. **Verify Topics** are publishing correctly
3. **Launch Mission Agent** (DIMOS) once robot is confirmed ready

This ensures the mission agent never starts before the robot is fully initialized.

## Three-Stage Launch

### Stage 1: Robot Driver

```
── Stage 1: Launching Robot Driver ──
ℹ Verifying robot connectivity at 192.168.10.167...
✓ Robot is reachable
ℹ Launching robot driver in background...
✓ Robot driver launched (PID: 12345)
ℹ Logs: /tmp/shadowhound_robot_driver.log
ℹ Waiting for robot topics to appear...
✓ Robot topics detected!
```

**What happens:**
- Pings robot to verify network connectivity
- Checks if driver already running (offers to reuse)
- Launches `launch/go2_sdk/robot.launch.py` in background
- Waits up to 30 seconds for `/go2_states` topic
- Saves PID for cleanup

### Stage 2: Topic Verification

```
── Stage 2: Verifying Robot Topics ──
ℹ Running topic diagnostics...

🔍 ROS2 TOPIC & ACTION DIAGNOSTICS
📡 Found 15 robot-related topics:
  ✓ /go2_states
  ✓ /camera/image_raw
  ✓ /imu
  ✓ /odom
  ✓ /local_costmap/costmap
  ...

🎯 Critical topic status:
  ✅ /go2_states (Robot state data)
  ✅ /camera/image_raw (Camera feed)
  ✅ /imu (IMU data)
  ✅ /odom (Odometry)
  ...

✅ ALL CRITICAL TOPICS FOUND - Ready for mission agent!

Topics look good? Continue to launch mission agent? [Y/n]:
```

**What happens:**
- Runs `scripts/check_topics.py` diagnostic
- Shows all robot-related topics
- Verifies critical topics exist
- Checks Nav2 action servers
- Prompts before continuing

### Stage 3: Mission Agent

```
── Stage 3: Launching Mission Agent ──
ℹ Launch command:
  ros2 launch shadowhound_mission_agent mission_agent.launch.py mock_robot:=false

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🌐 Web Dashboard will be available at: http://localhost:8080
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✓ Starting Mission Agent...

[mission_agent]: Configuration:
[mission_agent]:   Agent backend: cloud
[mission_agent]:   Mock robot: False
[mission_agent]: 🔍 TOPIC DIAGNOSTICS (built-in check)
[mission_agent]:   ✅ /go2_states found
[mission_agent]: Initializing robot interface...
```

**What happens:**
- Sources ROS2 and workspace
- Sets PYTHONPATH for DIMOS
- Launches mission agent with configured parameters
- Agent runs its own diagnostics (double-check)
- Initializes DIMOS framework

## Usage

### Full Orchestrated Launch (Recommended)

```bash
# Default: Launches everything with verification
./start.sh --dev

# Production mode with full orchestration
./start.sh --prod
```

**Process:**
1. Pings robot
2. Launches robot driver
3. Waits for topics
4. Verifies with diagnostics
5. Launches mission agent

### Skip Driver Launch (Use Existing)

```bash
# If driver already running in another terminal
./start.sh --dev --skip-driver
```

**Process:**
1. Skips driver launch
2. Verifies existing topics
3. Launches mission agent

### Agent-Only Mode (Fast Development)

```bash
# When testing agent changes only
./start.sh --dev --agent-only
```

**Process:**
1. Skips driver launch
2. Skips topic verification
3. Directly launches mission agent

Good for:
- Testing agent code changes
- When you manually control the driver
- Rapid iteration on mission agent

### Mock Robot Mode

```bash
# No hardware needed
./start.sh --dev --mock
```

**Process:**
1. Skips driver launch (mock mode)
2. Skips topic verification (not needed)
3. Launches agent with `mock_robot:=true`

## Command-Line Options

| Option | Description |
|--------|-------------|
| `--dev` | Development mode (default config) |
| `--prod` | Production mode (real robot) |
| `--mock` | Mock robot (no hardware) |
| `--skip-driver` | Don't launch driver, use existing |
| `--agent-only` | Only launch agent (skip all checks) |
| `--skip-update` | Skip git update check |
| `--auto-update` | Auto-pull updates |
| `--no-web` | Disable web interface |
| `--web-port N` | Set web port |

## Examples

### Daily Development

```bash
# Morning: Full orchestrated launch with auto-update
./start.sh --dev --auto-update
```

### Rapid Agent Iteration

```bash
# Terminal 1: Launch driver once
ros2 launch launch/go2_sdk/robot.launch.py

# Terminal 2: Iterate on agent (skip driver)
./start.sh --dev --skip-driver

# Kill agent (Ctrl+C), make changes, relaunch:
./start.sh --dev --skip-driver
```

### Testing Without Robot

```bash
# Mock mode for development without hardware
./start.sh --dev --mock
```

### Production Deployment

```bash
# Full validation before launch
./start.sh --prod --auto-update
```

## Log Files

### Robot Driver Logs
**Location**: `/tmp/shadowhound_robot_driver.log`

Check if driver fails to start:
```bash
tail -f /tmp/shadowhound_robot_driver.log
```

### Mission Agent Logs
**Location**: Terminal output (stdout/stderr)

Or capture to file:
```bash
./start.sh --dev 2>&1 | tee /tmp/shadowhound_mission_agent.log
```

## Troubleshooting

### Driver Launch Fails

**Symptom:**
```
❌ Robot not reachable at 192.168.10.167
```

**Solutions:**
1. Check robot is powered on: `ping 192.168.10.167`
2. Verify network connection: `ifconfig` or `ip addr`
3. Check GO2_IP in .env matches robot's actual IP
4. Use mock mode if no robot: `./start.sh --dev --mock`

### Topics Don't Appear

**Symptom:**
```
ℹ Waiting for robot topics to appear...
❌ Timeout waiting for robot topics
```

**Solutions:**
1. Check driver logs: `tail /tmp/shadowhound_robot_driver.log`
2. Verify robot is connected: `ros2 topic list`
3. Check ROS_DOMAIN_ID matches: `echo $ROS_DOMAIN_ID`
4. Try launching driver manually to see errors:
   ```bash
   ros2 launch launch/go2_sdk/robot.launch.py
   ```

### Driver Already Running

**Symptom:**
```
⚠ Robot driver already running (topics detected)
Use existing driver? [Y/n]:
```

**Options:**
- **Press Y**: Use existing driver (recommended)
- **Press N**: Kill and restart driver
- **Use flag**: `./start.sh --dev --skip-driver` to always use existing

### Topics Present But Wrong

**Symptom:**
```
⚠️  SOME CRITICAL TOPICS MISSING
   /local_costmap/costmap (missing)
```

**Solutions:**
1. Check which launch file is being used:
   - `robot.launch.py` = Full stack (Nav2, SLAM, lidar)
   - `robot_minimal.launch.py` = Basic driver only
2. Use full launch file:
   ```bash
   ros2 launch launch/go2_sdk/robot.launch.py
   ```

### Cleanup Not Working

**Symptom:** Processes remain after Ctrl+C

**Solution:**
```bash
# Manual cleanup
pkill -f go2_driver_node
pkill -f shadowhound_mission_agent
rm -f /tmp/shadowhound_driver.pid
```

## Architecture

### Process Tree

```
start.sh (PID 1000)
├── Robot Driver (PID 1234, background, logged)
│   ├── go2_driver_node
│   ├── lidar_processor
│   ├── Nav2 nodes
│   └── RViz2
│
└── Mission Agent (PID 2345, foreground)
    ├── DIMOS framework
    ├── Skills library
    ├── OpenAI agent
    └── Web interface
```

### Cleanup on Exit

When you press Ctrl+C:
1. **Trap signal caught** by start.sh
2. **Mission agent terminated** (SIGTERM → SIGKILL)
3. **Robot driver terminated** using saved PID
4. **Temp files cleaned** (/tmp/shadowhound_*.pid)
5. **Exit gracefully**

Both processes are fully cleaned up automatically.

## Benefits

### vs. Manual Launch

**Before:**
```bash
# Terminal 1
ros2 launch launch/go2_sdk/robot.launch.py
# Wait... is it ready?

# Terminal 2
ros2 topic list | grep go2_states  # Check manually
ros2 launch shadowhound_mission_agent mission_agent.launch.py
# Oops, driver wasn't ready yet, agent hangs
```

**After:**
```bash
# Single command
./start.sh --dev
# Automatically:
# ✓ Launches driver
# ✓ Waits for ready
# ✓ Verifies topics
# ✓ Launches agent when safe
```

### Development Benefits

✅ **No more guessing** if driver is ready
✅ **Automatic validation** before agent starts
✅ **Single terminal** for common workflows
✅ **Clean shutdown** of all processes
✅ **Flexible modes** for different scenarios

### Debugging Benefits

✅ **Clear failure points** (which stage failed?)
✅ **Topic diagnostics** built-in
✅ **Log files** for driver issues
✅ **Interactive prompts** to decide next action

## Integration with Other Features

### With Auto-Update

```bash
# Pull latest code, then orchestrated launch
./start.sh --dev --auto-update

# Process:
# 1. Check/pull git updates
# 2. Rebuild if code changed
# 3. Orchestrated three-stage launch
```

### With Topic Diagnostics

```bash
# Standalone check before launch
python3 scripts/check_topics.py

# Or let start.sh handle it (Stage 2)
./start.sh --dev
```

### With Go2 SDK Improvements

When working on go2_ros2_sdk:
```bash
# Terminal 1: Test driver changes
cd src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk
# ... edit files ...
cd ~/shadowhound
ros2 launch launch/go2_sdk/robot.launch.py

# Terminal 2: Test agent with your driver
./start.sh --dev --skip-driver
```

## Related Documentation

- `docs/topic_diagnostics.md` - Topic checking details
- `docs/auto_update.md` - Auto-update feature
- `docs/qol_improvements.md` - All QoL features
- `QUICK_REFERENCE.md` - Command cheat sheet
