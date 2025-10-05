# Topic Visibility Diagnostics

## Purpose

These tools help diagnose whether the mission agent can see the required ROS2 topics and action servers before attempting to initialize DIMOS.

## Files Added

1. **`scripts/check_topics.py`** - Standalone diagnostic tool
2. **`test_topic_visibility.sh`** - Full test with robot driver check
3. **`mission_agent.py`** - Now includes `_log_topic_diagnostics()` method

## Usage

### Quick Check (Recommended First Step)

Run this standalone script to see what's visible:

```bash
cd ~/shadowhound  # On your laptop, not devcontainer
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/check_topics.py
```

**Expected Output (when robot driver is running):**
```
🔍 ROS2 TOPIC & ACTION DIAGNOSTICS
==================================================

📡 Found 15 robot-related topics:
  ✓ /go2_states
      └─ go2_interfaces/msg/Go2State
  ✓ /camera/image_raw
      └─ sensor_msgs/msg/Image
  ✓ /imu
      └─ sensor_msgs/msg/Imu
  ... etc ...

🎯 Critical topic status:
  ✅ /go2_states                    Robot state data (30Hz)
  ✅ /camera/image_raw              Camera feed (15Hz)
  ✅ /imu                           IMU data
  ✅ /odom                          Odometry
  ✅ /local_costmap/costmap         Local costmap (Nav2)
  ✅ /cmd_vel_out                   Velocity commands

🎮 Checking Nav2 action servers...
  ✅ /spin action server

==================================================
✅ ALL CRITICAL TOPICS FOUND - Ready for mission agent!
==================================================
```

### Full Test (With Robot Driver Check)

This script checks if the robot driver is running and then launches the mission agent:

```bash
cd ~/shadowhound
./test_topic_visibility.sh
```

This will:
1. Check if robot driver topics exist
2. List all robot-related topics and actions
3. Launch mission agent with diagnostics enabled

### Mission Agent Built-in Diagnostics

The mission agent now automatically logs topic diagnostics during initialization:

```bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py
```

Look for this section in the logs:
```
[mission_agent]: ============================================================
[mission_agent]: 🔍 TOPIC DIAGNOSTICS
[mission_agent]: ============================================================
[mission_agent]: Found 15 robot-related topics:
[mission_agent]:   • /go2_states [go2_interfaces/msg/Go2State]
[mission_agent]:   • /camera/image_raw [sensor_msgs/msg/Image]
...
```

## Troubleshooting

### No Topics Found

**Symptom:**
```
❌ No robot topics found!
⚠️  Is the robot driver running?
```

**Solution:**
1. Start the robot driver first:
   ```bash
   ros2 launch go2_robot_sdk robot.launch.py
   ```
2. Wait 5-10 seconds for all topics to publish
3. Run diagnostics again

### Wrong ROS_DOMAIN_ID

**Symptom:** Topics exist but don't show up in diagnostics

**Solution:**
```bash
# Check current domain
echo $ROS_DOMAIN_ID

# Match it to robot driver's domain
export ROS_DOMAIN_ID=0  # or whatever your robot uses
```

### Nav2 Action Server Missing

**Symptom:**
```
❌ /spin action server
⚠️  Nav2 /spin action server not found
```

**Possible Causes:**
1. Nav2 not included in robot launch file (use `robot.launch.py`, not `robot_minimal.launch.py`)
2. Nav2 still starting up (wait 10-15 seconds)
3. Nav2 crashed (check: `ros2 node list | grep nav`)

## What This Tells Us

### If Topics ARE Visible
- ✅ Robot driver is running correctly
- ✅ ROS2 network configuration is correct
- ✅ QoS settings are compatible
- ➡️ Problem is likely in DIMOS initialization logic

### If Topics are NOT Visible
- ❌ Robot driver not running or crashed
- ❌ ROS_DOMAIN_ID mismatch
- ❌ Network configuration issue
- ➡️ Fix topic visibility before debugging DIMOS

## Next Steps

1. **Run `scripts/check_topics.py` on your laptop** (where robot driver runs)
2. **Verify all critical topics show ✅**
3. If topics missing: Fix robot driver setup
4. If topics present: Continue debugging DIMOS initialization hang

The diagnostics will help us understand whether the mission agent initialization hang is:
- **Pre-DIMOS**: Topics not visible to the node
- **In-DIMOS**: DIMOS can't subscribe/connect even though topics exist
