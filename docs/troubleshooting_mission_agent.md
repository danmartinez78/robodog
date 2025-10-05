# Mission Agent Initialization Troubleshooting Log

## Problem
Mission agent hangs during initialization, getting stuck after subscribing to topics.

## Symptoms
```
[INFO] [mission_agent]: Initializing robot interface...
2025-10-04 21:06:07,858 - dimos.robot.unitree.unitree_ros_control - INFO - Initializing Unitree ROS control interface
INFO:unitree_hardware_interface_video:ROSVideoProvider initialized
2025-10-04 21:06:07,863 - dimos.robot.ros_control - INFO - Subscribing to camera/compressed with BEST_EFFORT QoS using message type CompressedImage
2025-10-04 21:06:07,864 - dimos.robot.ros_control - INFO - Subscribing to go2_states with BEST_EFFORT QoS
[HANGS HERE - never proceeds to "Initializing skill library"]
```

## Root Cause Analysis

### Confirmed Working
- ✅ `/go2_states` topic publishing at ~30-50 Hz with BEST_EFFORT QoS
- ✅ `/camera/image_raw` topic publishing at ~13-16 Hz with BEST_EFFORT QoS
- ✅ `robot.launch.py` launches successfully
- ✅ Go2 driver node connected and publishing
- ✅ DIMOS imports successfully when PYTHONPATH set
- ✅ Mission agent package builds without errors

### Identified Issues

#### Issue 1: Missing Camera Topic ❌ TRIED
**Problem**: DIMOS expected `/camera/compressed` but robot only publishes `/camera/image_raw`

**Solution Attempted**: Added `image_transport republish` node to convert raw to compressed

**Result**: Republisher runs but publishes with RELIABLE QoS, incompatible with DIMOS's BEST_EFFORT subscription

#### Issue 2: QoS Mismatch ❌ TRIED
**Problem**: `image_transport republish` publishes `/camera/compressed` with RELIABLE QoS, but DIMOS subscribes with BEST_EFFORT

**Solution Attempted 1**: Configure `use_raw=True` in mission_agent.py to subscribe to `/camera/image_raw` directly

**Result**: Still hangs (tried earlier in session)

**Solution Attempted 2**: Add QoS overrides to republisher node
```python
parameters=[{
    'qos_overrides./camera/compressed.publisher.reliability': 'best_effort',
}]
```

**Result**: QoS override didn't work, republisher still uses RELIABLE

### Current State
- Robot publishing: `/camera/image_raw` (BEST_EFFORT, ~15 Hz) ✅
- Robot publishing: `/go2_states` (BEST_EFFORT, ~30 Hz) ✅
- Mission agent config: `use_raw=True` (subscribes to `/camera/image_raw`)
- Status: **STILL HANGS** - unclear why

## Theories to Investigate

### Theory 1: DIMOS waiting for initial message but not receiving due to timing
- DIMOS might subscribe after first message already published
- Could be checking message buffer that's empty

**Test**: Check if DIMOS has timeout logic, increase if needed

### Theory 2: DIMOS initializing other components that we can't see
- Might be waiting on costmap, transforms, or other topics
- Logs might not show all subscription attempts

**Test**: Enable debug logging in DIMOS, check what it's actually waiting for

### Theory 3: Node spinning/executor issue
- DIMOS might not be spinning the ROS node properly during init
- Callbacks not being processed

**Test**: Check DIMOS ros_control initialization, verify executor is running

### Theory 4: Multiple node name collision
- Earlier we saw "duplicate node name" warnings
- Might be causing subscription issues

**Test**: Verify only one mission_agent node running, check for conflicts

## Next Steps
1. **Try disabling video stream completely** - Set `disable_video_stream=True` to rule out video init issues
2. Add debug logging to see exactly where DIMOS hangs
3. Check if DIMOS waits for first message on subscribed topics (costmap?)
4. Verify ROS executor is spinning during initialization
5. Check for any hidden topic subscriptions (transforms, etc.)

## Latest Finding (2025-10-04 21:35)
**ROOT CAUSE IDENTIFIED**: Mission agent hangs at `self._spin_client.wait_for_server()` in ROSControl.__init__

The initialization is blocking while waiting for Nav2's `/spin` action server to become available.
With `mock_robot:=false`, DIMOS tries to connect to Nav2 action servers.

Location: `src/dimos-unitree/dimos/robot/ros_control.py:236`
```python
if not mock_connection:
    self._spin_client.wait_for_server()  # <-- BLOCKS HERE
```

**Quick Fix Options**:
1. Ensure Nav2 spin action server is running: `ros2 action list | grep spin`
2. Temporarily use `mock_connection=True` in ROSControl (skips Nav2 connection)
3. Remove/timeout the `wait_for_server()` call

## Commands for Testing

### Verify topics publishing
```bash
ros2 topic hz /go2_states
ros2 topic hz /camera/image_raw
ros2 topic list | grep camera
```

### Check QoS profiles
```bash
ros2 topic info /camera/image_raw -v
ros2 topic info /go2_states -v
```

### Verify node running
```bash
ros2 node list | grep mission
ros2 node info /mission_agent
```

### Launch with PYTHONPATH
```bash
export PYTHONPATH="$HOME/shadowhound/src/dimos-unitree:$PYTHONPATH"
source ~/shadowhound/install/setup.bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py mock_robot:=false
```
