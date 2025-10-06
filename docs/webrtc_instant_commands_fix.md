# WebRTC Instant Commands Fix - Session Summary

**Date:** October 5-6, 2025  
**Branch:** `fix/webrtc-instant-commands-and-progress` in dimos-unitree fork  
**Status:** 🔧 Fix implemented, awaiting validation

---

## Problem Statement

WebRTC API commands worked perfectly via **CLI** (direct publish to `/webrtc_req`) but **hung indefinitely** when called through the **web UI** (via mission agent's ROSCommandQueue).

### Symptoms
- ✅ CLI commands (1001, 1004, 1005, 1009, 1016, 1017) executed instantly
- ❌ Web UI commands never completed - queue waited forever
- ✅ Nav2 skills (Move, SpinLeft, SpinRight) worked fine via web UI

### Commands Tested
- **1001** - Damp (walk mode)
- **1004** - StandUp
- **1005** - StandDown  
- **1009** - Sit ✨ (re-enabled)
- **1016** - Hello (wave)
- **1017** - Stretch

---

## Root Cause

The `ROSCommandQueue` in `ros_command_queue.py` had **backwards logic** for instant commands:

```python
# OLD BROKEN CODE - only waited WHILE busy
while self._is_busy_func() and (time.time() - start_time) < timeout:
    time.sleep(0.1)
```

**The Problem:**  
Instant commands execute so fast (0→1→0 progress in ~100ms) that by the time the queue checked `_is_busy_func()`, the robot was already back to idle. The queue thought the command never started and waited forever.

### Progress State Transitions
```
Command sent → Robot state changes: 0 → 1 → 0 (in milliseconds)
                                     ↑   ↑   ↑
                                  IDLE BUSY IDLE
```

The queue checked after the transition completed, saw `progress=0`, and thought "not busy yet, keep waiting."

---

## The Fix

Modified `ros_command_queue.py` to **wait FOR the robot to become busy first**, then wait for completion:

```python
# NEW WORKING CODE - wait FOR busy state, then wait for completion
wait_for_busy_timeout = 2.0
became_busy = False

# Phase 1: Wait up to 2s for robot to become busy
while not self._is_busy_func() and (time.time() - start_time) < wait_for_busy_timeout:
    time.sleep(0.05)  # Check every 50ms
    if self._is_busy_func():
        became_busy = True
        break

if not became_busy:
    # Instant command - already complete by the time we checked
    self._logger.info("Command appears to be instant or already complete")
    time.sleep(0.5)  # Brief stabilization delay
    return True

# Phase 2: Now wait for completion (progress returns to 0)
while self._is_busy_func() and (time.time() - start_time) < timeout:
    time.sleep(0.1)
    
return True  # Success
```

### Additional Changes

**`unitree_ros_control.py`** - Simplified state detection:
```python
# OLD: Complex mode+progress logic
if progress == 0 and mode == 1:
    self._mode = RobotMode.IDLE
elif progress == 1 or mode != 1:
    self._mode = RobotMode.MOVING

# NEW: Use progress only (more reliable)
if progress == 0:
    self._mode = RobotMode.IDLE
elif progress == 1:
    self._mode = RobotMode.MOVING
```

**`unitree_skills.py`**:
- Re-enabled Sit command (ID 1009)
- Minor formatting improvements

---

## Changes Made

### Commits in `dimos-unitree` fork (fix/webrtc-instant-commands-and-progress):

1. **7538e0f** - Fix WebRTC command queue for instant commands  
   - Modified: `ros_command_queue.py`, `unitree_ros_control.py`, `unitree_skills.py`
   - **The Core Fix** ✨

2. **27d2ea2** - Fix: Use go2_interfaces.msg instead of unitree_go.msg  
   - Fixed import error in `unitree_ros_control.py`

3. **3c0bc01** - Revert go2_ros2_sdk to 6c0551b (restore working state)  
   - Reverted unnecessary submodule update that crossed a rebase

4. **4addfbc** - Skip models directory during build (requires PyTorch/CUDA)  
   - Added `COLCON_IGNORE` to `dimos/models/` directory
   - Avoids build failures from Detic vision models that need ML dependencies

### Updated in `shadowhound` repo:

**`shadowhound.repos`**:
```yaml
repositories:
  dimos-unitree:
    type: git
    url: https://github.com/danmartinez78/dimos-unitree.git
    version: fix/webrtc-instant-commands-and-progress
```

---

## Current State

### ✅ Completed
- Queue fix logic implemented in `ros_command_queue.py`
- State detection simplified in `unitree_ros_control.py`
- Import errors resolved
- Build issues fixed (submodule reverted, models skipped)
- All changes pushed to `fix/webrtc-instant-commands-and-progress` branch
- `shadowhound.repos` updated to point to fix branch
- Clean build verified in devcontainer

### ⚠️ NOT YET TESTED
- **The fix has NOT been validated via web UI** - this is critical!
- We only confirmed:
  - ✅ Code compiles and builds cleanly
  - ✅ Commands work via CLI (but CLI never had the issue)
  - ❌ **Web UI command execution NOT tested yet**

### 🎯 Critical Next Step
**Test via Web UI on laptop** - This is the ONLY way to know if the fix actually works!

### 🎯 Test Commands (via Web UI)
```
"sit"
"stand up"
"wave hello"
"stretch"
"damp"
```

---

## Deployment Instructions

### On Laptop (or any machine with the robot):

```bash
cd ~/shadowhound/src/dimos-unitree
git pull
git submodule update --recursive

cd ~/shadowhound
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

# Launch the stack
ros2 launch shadowhound_bringup shadowhound_bringup.launch.py
```

### Expected Behavior After Fix (IF IT WORKS)
1. **Web UI commands should execute immediately** (no more hanging)
2. **Agent should receive success/failure responses** within 2-3 seconds
3. **Progress state transitions visible** in logs:
   ```
   [INFO] Executing WebRTC command: 1009
   [INFO] Waiting for robot to become busy...
   [INFO] Robot became busy (progress=1)
   [INFO] Waiting for completion...
   [INFO] Command complete (progress=0)
   ```

### What Could Still Go Wrong
- The 2-second busy detection timeout might be too short
- State transitions might happen even faster than 50ms polling
- Progress state might not be reliable indicator
- There could be race conditions we didn't account for
- The fix might only work for some commands, not all

---

## Technical Details

### Robot State Topics
- **`/go2_states`** - Published by `go2_driver_node`
  - `progress`: 0 (idle), 1 (executing)
  - `mode`: 1 (normal), others (unknown)
  
### Command Flow
```
Web UI → Mission Agent → ROSCommandQueue → /webrtc_req → go2_driver_node → Robot
         ↑                     ↓
         └─── success/failure ─┘
```

### Why CLI Works But Web UI Didn't
- **CLI**: Direct publish to `/webrtc_req`, no waiting
- **Web UI**: Goes through queue which needs to monitor completion
- **Queue bug**: Missed the instant state transition

---

## Known Issues / Limitations

### Current Limitations
1. **No VLM integration yet** - Commands are still hardcoded skill IDs
2. **No pose feedback** - Can't verify if commands achieved desired result
3. **2-second busy detection timeout** - Might be too long for some use cases

### Not Tested Yet ⚠️
- ❌ **Actual web UI execution** (CRITICAL - this is the whole point!)
- ❌ The fix might not work at all
- ❌ Error handling for failed commands
- ❌ Multiple rapid commands in sequence
- ❌ Command interruption/cancellation

**Bottom line:** We have a theory and an implementation, but zero proof it works.

---

## Next Steps

### Immediate (Priority 1) - CRITICAL ⚡
1. **Test via Web UI on laptop** 🔴 **MUST DO FIRST**
   - Deploy the fix to laptop
   - Try "sit", "hello", "stretch" via web interface
   - **This will tell us if the fix actually works**
   - Check logs for proper state transitions
   - If it hangs: back to the drawing board
   - If it works: celebrate and move to validation

2. **IF the fix works, validate edge cases**
   - Test invalid command IDs
   - Test commands during motion (busy state)
   - Verify timeout behavior
   - Try rapid command sequences

### Short Term (Priority 2)
3. **Add telemetry/logging**
   - Log execution time per command
   - Track success/failure rates
   - Monitor state transitions

4. **Create PR to upstream dimos-unitree**
   - Clean up commit history (squash if needed)
   - Write proper PR description
   - Include before/after behavior

### Medium Term (Priority 3)
5. **VLM Integration**
   - Use vision to verify command completion
   - "Did the robot actually sit?"
   - Retry logic based on visual feedback

6. **Pose Estimation**
   - Parse actual robot pose from state messages
   - Validate position after navigation commands
   - Detect falls or stuck states

7. **Mission Planning**
   - Chain multiple commands
   - Handle dependencies (must stand before walking)
   - Recovery behaviors

---

## Files Modified

```
dimos-unitree/
├── dimos/robot/unitree/
│   ├── external/go2_ros2_sdk/           (submodule at 6c0551b)
│   └── ros_control/
│       ├── ros_command_queue.py         ✏️ Core fix
│       ├── unitree_ros_control.py       ✏️ State detection
│       └── unitree_skills.py            ✏️ Re-enabled Sit
└── dimos/models/COLCON_IGNORE           ➕ New file

shadowhound/
└── shadowhound.repos                     ✏️ Point to fix branch
```

---

## Environment Details

- **Robot:** Unitree Go2 (Firmware 1.1.3)
- **Network:** WebRTC mode via hotspot (192.168.12.1)
- **ROS2:** Humble
- **Development:** VS Code devcontainer
- **Repository Structure:** vcstool-managed (not git submodules)

---

## Lessons Learned

1. **Always check timing assumptions** - "Instant" means microseconds, not zero time
2. **State machines need proper phase detection** - Wait FOR state change, not just DURING
3. **Submodule updates are risky** - Only update when necessary, verify compatibility
4. **COLCON_IGNORE is your friend** - Skip unneeded packages to avoid dependency hell
5. **CLI vs Queue behavior differs** - Direct publish != queue-mediated execution

---

## Questions for Next Session

1. **Does the fix work via web UI?** 🔴 (Critical - we don't know yet!)
2. **IF it works:** Should we add a "became_busy" flag to telemetry?
3. **IF it works:** What's the right busy detection timeout? (currently 2s)
4. **IF it works:** Should instant commands have a different timeout than motion commands?
5. **IF it doesn't work:** What did we miss in the state machine logic?
6. **IF it doesn't work:** Do we need to monitor a different state topic?

---

**Status:** Fix implemented but UNVALIDATED. We have a hypothesis (instant commands transition too fast for the queue to detect) and a proposed solution (wait FOR busy state with timeout), but we haven't tested if it actually solves the problem. Next session MUST start with web UI testing. 🧪
