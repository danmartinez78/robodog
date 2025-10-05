# Mission Agent Initialization Troubleshooting Log

## Problem
Mission agent hangs during initialization, getting stuck after subscribing to topics.

## Symptoms
```
# Mission Agent Initialization Troubleshooting Log

## Problem Statement
Mission agent hangs during initialization after subscribing to ROS topics. Never proceeds to skill library initialization or web interface startup.

## Current Status: UNRESOLVED (2025-10-04 21:45)
Despite identifying root cause and applying workarounds, mission agent still hangs at same point.

---

## Environment Details

### Hardware & Software
- **Laptop**: Ubuntu 22.04 bare metal (not container)
- **Robot**: Unitree Go2 at IP 192.168.10.167
- **ROS2**: Humble
- **Workspace**: `~/shadowhound` on laptop
- **Branch**: `feature/dimos-integration`

### Confirmed Working Components
- ✅ Robot network connectivity (ping works)
- ✅ Go2 driver node publishing data
- ✅ `/go2_states` topic publishing at ~30-50 Hz (BEST_EFFORT QoS)
- ✅ `/camera/image_raw` topic publishing at ~13-16 Hz (BEST_EFFORT QoS)
- ✅ Nav2 `/spin` action server available and running
- ✅ DIMOS imports successfully when PYTHONPATH set
- ✅ Mission agent package builds without errors
- ✅ robot.launch.py launches successfully

---

## Symptoms & Log Analysis

### Last Visible Log Output
```
[INFO] [mission_agent]: Initializing robot interface...
2025-10-04 21:39:02,085 - dimos.robot.unitree.unitree_ros_control - INFO - Initializing Unitree ROS control interface
[WARN] [rcl.logging_rosout]: Publisher already registered for provided node name...
2025-10-04 21:39:02,089 - dimos.robot.ros_control - INFO - Subscribing to go2_states with BEST_EFFORT QoS
[HANGS HERE - never proceeds]
```

### Expected Next Log (never appears)
```
2025-10-04 XX:XX:XX - dimos.robot.ros_control - INFO - unitree_hardware_interface initialized with multi-threaded executor
[INFO] [mission_agent]: Robot initialized (mock=False, ip=192.168.10.167)
[INFO] [mission_agent]: Initializing skill library...
```

---

## Root Cause Analysis

### Discovery Process

#### Phase 1: Topic Issues (Resolved)
**Issue**: DIMOS expected `/camera/compressed` but only `/camera/image_raw` was published.

**Investigation**:
- Verified topics: `ros2 topic list | grep camera` showed only `/camera/image_raw`
- DIMOS subscribes to `/camera/compressed` by default

**Solution Attempted**: Added `image_transport republish` node to `robot.launch.py`
```python
Node(
    package='image_transport',
    executable='republish',
    name='image_republisher',
    arguments=['raw', 'compressed'],
    remappings=[
        ('in', 'camera/image_raw'),
        ('out/compressed', 'camera/compressed'),
    ],
)
```

**Result**: Republisher uses RELIABLE QoS (incompatible with DIMOS's BEST_EFFORT subscription)

**Workaround**: Configured DIMOS to use `use_raw=True` - still hung

**Final Approach**: Disabled video entirely with `disable_video_stream=True` - still hung

#### Phase 2: Video Stream Issues (Ruled Out)
**Hypothesis**: Video/tracking streams causing hang during UnitreeGo2.__init__

**Test**: Set `disable_video_stream=True` in both ROSControl and UnitreeGo2

**Result**: Still hangs at same point - video not the issue

#### Phase 3: Nav2 Action Server Blocking (ROOT CAUSE IDENTIFIED)
**Discovery**: Found blocking call in DIMOS code at `src/dimos-unitree/dimos/robot/ros_control.py:236`

```python
# Nav2 Action Clients
self._spin_client = ActionClient(self._node, Spin, "spin")

# Wait for action servers
if not mock_connection:
    self._spin_client.wait_for_server()  # <-- BLOCKS INDEFINITELY HERE
```

**Critical Bug**: `wait_for_server()` is called BEFORE the ROS executor starts spinning (line 267):
```python
# Start ROS spin in a background thread via the executor
self._spin_thread = threading.Thread(target=self._ros_spin, daemon=True)
self._spin_thread.start()
```

**Why It Hangs**: Without the executor spinning, the node cannot receive the action server advertisement, so `wait_for_server()` waits forever even though the action server exists.

**Verification**:
```bash
ros2 action list | grep spin  # Shows: /spin ✅
ros2 action info /spin -v     # Server exists and available ✅
ros2 node list | grep nav     # Nav2 nodes running ✅
```

**Workaround Applied**: Set `mock_connection=True` to skip `wait_for_server()` call
```python
ros_control = UnitreeROSControl(
    mock_connection=True,  # Skip Nav2 action server wait
    disable_video_stream=True
)
```

**Result**: **STILL HANGS AT SAME POINT** 😞

---

## Workarounds & Code Changes Applied

### Changes Made During Troubleshooting

#### 1. Added image_transport republisher to robot.launch.py
**File**: `launch/go2_sdk/robot.launch.py`
**Change**: Added republisher node in `create_core_nodes()`
**Status**: May not be needed (QoS incompatible anyway)
**Revert Instructions**:
```bash
git diff HEAD~5 launch/go2_sdk/robot.launch.py
# Remove the image_republisher Node() block
```

#### 2. Modified mission_agent.py multiple times
**File**: `src/shadowhound_mission_agent/shadowhound_mission_agent/mission_agent.py`

**Current State** (as of commit fa3cf45):
```python
ros_control = UnitreeROSControl(
    mock_connection=True,  # Skip Nav2 action server wait (DIMOS bug workaround)
    disable_video_stream=True  # Temporarily disable to debug init hang
)

self.robot = UnitreeGo2(
    ros_control=ros_control,
    ip=robot_ip,
    mock_connection=True,  # Skip Nav2 action server wait (DIMOS bug workaround)
    disable_video_stream=True,  # Temporarily disable to debug init hang
)
```

**Original State** (before troubleshooting):
```python
ros_control = UnitreeROSControl(mock_connection=self.mock_robot)

self.robot = UnitreeGo2(
    ros_control=ros_control,
    ip=robot_ip,
    mock_connection=self.mock_robot,
    disable_video_stream=True,  # Was already disabled
)
```

**Revert Instructions**:
```bash
cd ~/shadowhound
git log --oneline --graph | head -20  # Find commit before troubleshooting started
git diff <commit-before-troubleshooting> src/shadowhound_mission_agent/
git checkout <commit-before-troubleshooting> -- src/shadowhound_mission_agent/shadowhound_mission_agent/mission_agent.py
```

#### 3. Created troubleshooting documentation
**File**: `docs/troubleshooting_mission_agent.md`
**Status**: Keep for reference
**Action**: Update with final resolution when found

---

## Theories & Next Steps

### Why Workaround Didn't Work

**Mystery**: Even with `mock_connection=True`, initialization still hangs at exact same point.

**Possible Explanations**:
1. **Incorrect parameter passing**: `mock_connection=True` not being honored due to Python import caching or parameter override
2. **Different blocking call**: Another `wait_for_server()` or similar blocking call we haven't found
3. **Executor not starting**: ROS executor thread not actually starting, preventing any callbacks
4. **Hidden dependency**: DIMOS waiting for something else (transform, another topic, etc.)
5. **File not updating**: Despite git pull, Python file not reloading due to symlink/cache issue

### Investigation Steps for Tomorrow

#### Quick Comparison (Automated)
```bash
cd ~/shadowhound
./scripts/compare_go2_sdk_forks.sh
# Review differences and decide if driver testing needed
```

#### 1. Verify mock_connection is actually True at runtime
Add debug logging to DIMOS code:
```python
# In src/dimos-unitree/dimos/robot/ros_control.py line 235
logger.info(f"DEBUG: mock_connection={mock_connection}")
if not mock_connection:
    logger.info("DEBUG: About to call wait_for_server()")
    self._spin_client.wait_for_server()
    logger.info("DEBUG: wait_for_server() returned")
else:
    logger.info("DEBUG: Skipping wait_for_server() due to mock_connection=True")
```

#### 2. Add timeout to wait_for_server()
If mock_connection isn't working, add timeout:
```python
if not mock_connection:
    logger.info("Waiting for /spin action server (10s timeout)...")
    if not self._spin_client.wait_for_server(timeout_sec=10.0):
        logger.warning("/spin action server not available, continuing anyway")
```

#### 3. Check if executor actually starts
Add debug in `_ros_spin()`:
```python
def _ros_spin(self):
    """Background thread for spinning the multi-threaded executor."""
    logger.info("DEBUG: _ros_spin thread started")
    self._executor.add_node(self._node)
    logger.info("DEBUG: Node added to executor, starting spin")
    try:
        self._executor.spin()
    finally:
        logger.info("DEBUG: Executor shutdown")
        self._executor.shutdown()
```

#### 4. Try completely bypassing ROSControl initialization
Create minimal test:
```python
# In mission_agent.py
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
import rclpy
rclpy.init()
ros_control = UnitreeROSControl(mock_connection=True, disable_video_stream=True)
print("SUCCESS: ROSControl initialized!")
```

#### 5. Check for multiple node instances
```bash
ros2 node list  # Look for duplicate "unitree_hardware_interface" nodes
ros2 node info /unitree_hardware_interface  # Check subscriptions/publishers
```

#### 6. Monitor Python process
```bash
# In separate terminal while mission agent hangs
ps aux | grep mission_agent
strace -p <PID>  # See what system calls it's blocked on
```

#### 7. Compare go2_ros2_sdk forks
Investigate differences between upstream and DIMOS fork to see if driver changes cause the hang.

**Upstream**: https://github.com/dimensionalOS/go2_ros2_sdk  
**DIMOS Fork**: `src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk`

**Comparison Steps**:
```bash
# Clone upstream for comparison
cd /tmp
git clone https://github.com/dimensionalOS/go2_ros2_sdk.git upstream_go2_sdk

# Compare key files
cd ~/shadowhound

# Check go2_driver_node implementation
diff -u /tmp/upstream_go2_sdk/go2_robot_sdk/go2_robot_sdk/go2_driver_node.py \
        src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk/go2_robot_sdk/go2_robot_sdk/go2_driver_node.py

# Check launch files
diff -u /tmp/upstream_go2_sdk/go2_robot_sdk/launch/robot.launch.py \
        src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk/go2_robot_sdk/launch/robot.launch.py

# List all Python files that differ
diff -qr /tmp/upstream_go2_sdk/go2_robot_sdk/go2_robot_sdk/ \
         src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk/go2_robot_sdk/go2_robot_sdk/ \
    | grep "\.py"
```

**Key Areas to Investigate**:
1. **Topic names/QoS**: Does upstream publish different topics or use different QoS?
2. **Initialization order**: Does upstream driver initialize differently?
3. **Node lifecycle**: Does DIMOS fork change how nodes start/stop?
4. **Action servers**: Does upstream provide Nav2 action servers differently?
5. **Camera handling**: Different camera topic configuration?

**Test with Upstream Driver**:
```bash
# Temporarily switch to upstream driver
cd ~/shadowhound
mv src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk.dimos_fork
ln -s /tmp/upstream_go2_sdk src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk

# Rebuild and test
colcon build --packages-select go2_robot_sdk go2_interfaces unitree_go lidar_processor
source install/setup.bash
ros2 launch go2_robot_sdk robot.launch.py

# In another terminal, try mission agent
export PYTHONPATH="$HOME/shadowhound/src/dimos-unitree:$PYTHONPATH"
source install/setup.bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py mock_robot:=false

# Revert when done
rm src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk
mv src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk.dimos_fork \
   src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk
```

**Expected Findings**:
- If mission agent works with upstream → DIMOS fork has breaking changes
- If mission agent still hangs → Issue is in DIMOS robot interface code, not driver
- Compare topic outputs: `ros2 topic list` with both versions

---

## Files Modified (Summary)

### To Keep (Intentional Changes)
- `docs/troubleshooting_mission_agent.md` - This documentation

### To Review/Revert (Debug Changes)
- `launch/go2_sdk/robot.launch.py` - Added image_republisher (may not be needed)
- `src/shadowhound_mission_agent/shadowhound_mission_agent/mission_agent.py` - Multiple debug workarounds

### Commits to Review
```bash
git log --oneline --since="2025-10-04" feature/dimos-integration
# Should show:
# fa3cf45 Workaround: Use mock_connection=True to bypass DIMOS init bug
# 06dae60 Debug: Disable video stream to isolate initialization hang
# 7cad277 Configure image_republisher to use BEST_EFFORT QoS
# 90b7a5c Add image_transport republisher to robot.launch.py
# 3f30bc4 Fix mission agent camera topic configuration
```

---

## Quick Reference Commands

### Launch Mission Agent (Current State)
```bash
cd ~/shadowhound
export PYTHONPATH="$HOME/shadowhound/src/dimos-unitree:$PYTHONPATH"
source install/setup.bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py mock_robot:=false
# Ctrl+C to kill after ~30s when it hangs
```

### Launch Robot Driver
```bash
cd ~/shadowhound
source install/setup.bash
ros2 launch ~/shadowhound/launch/go2_sdk/robot.launch.py
# Or from package:
ros2 launch go2_robot_sdk robot.launch.py
```

### Verify Topics & Nodes
```bash
ros2 topic hz /go2_states
ros2 topic hz /camera/image_raw
ros2 topic list | grep -E "camera|go2"
ros2 node list | grep -E "mission|unitree|nav"
ros2 action list | grep spin
```

### Check Git State
```bash
cd ~/shadowhound
git status
git log --oneline --graph -10
git diff origin/main
```

### Python Environment
```bash
export PYTHONPATH="$HOME/shadowhound/src/dimos-unitree:$PYTHONPATH"
python3 -c "import dimos; print('DIMOS imported successfully')"
python3 -c "import sys; sys.path.insert(0, '$HOME/shadowhound/src/dimos-unitree'); from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl; print('UnitreeROSControl imported')"
```

---

## Known Issues & Notes

### DIMOS Bug Confirmed
**Location**: `src/dimos-unitree/dimos/robot/ros_control.py:236`
**Issue**: `wait_for_server()` called before executor starts
**Impact**: Blocks initialization even when action server exists
**Proper Fix**: Move `wait_for_server()` after executor start, or add timeout

### QoS Incompatibility
**Issue**: `image_transport republish` uses RELIABLE QoS
**Impact**: DIMOS can't subscribe with BEST_EFFORT
**Current State**: Video disabled entirely
**Proper Fix**: Use `/camera/image_raw` directly with `use_raw=True`

### Duplicate Node Warning
**Warning**: "Publisher already registered for provided node name"
**Impact**: Unknown (may cause subscription issues)
**Location**: Multiple nodes named "unitree_hardware_interface"
**Investigation**: Check if multiple ROSControl instances being created

---

## Success Criteria

Mission agent successfully initializes when we see:
```
[INFO] [mission_agent]: Initializing robot interface...
[INFO] unitree_hardware_interface initialized with multi-threaded executor
[INFO] [mission_agent]: Robot initialized (mock=True/False, ip=192.168.10.167)
[INFO] [mission_agent]: Initializing skill library...
[INFO] [mission_agent]: Loaded 15 skills
[INFO] [mission_agent]: Initializing cloud agent...
[INFO] [mission_agent]: Starting web interface on port 8080...
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8080
```

Then verify:
```bash
curl http://localhost:8080  # Should return web interface
```

---

## Contact & Handoff

**Session Date**: 2025-10-04  
**Duration**: ~3 hours  
**Status**: UNRESOLVED - Needs fresh investigation with debug logging  
**Branch**: `feature/dimos-integration` (commit fa3cf45)  
**Next Session**: Add debug logging to DIMOS code to see actual execution flow
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
