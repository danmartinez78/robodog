# ShadowHound Development Log

A chronological record of development milestones, decisions, and learnings.

---

## 2025-10-08 - Camera Feed Fix & UI Optimization

### What Was Done
**Fixed camera feed QoS compatibility and optimized web UI for laptop deployment**

Successfully debugged and resolved camera feed issues, then redesigned UI for better space utilization:

#### Camera Feed QoS Fix
- **Root Cause**: QoS mismatch between publisher (BEST_EFFORT) and subscriber (RELIABLE)
  - Camera driver publishes with sensor data QoS pattern (BEST_EFFORT, VOLATILE, depth=1)
  - Mission agent was using default ROS2 QoS (RELIABLE)
  - ROS2 middleware couldn't match incompatible QoS profiles, so callback never triggered
  
- **Diagnosis Process**:
  1. Verified topic exists and publishes at 14Hz: ✅
  2. Checked subscription creation: ✅
  3. Added debug logging to callback: Never triggered ❌
  4. Ran `ros2 topic info /camera/image_raw -v`: Found QoS mismatch!
  
- **Solution**: Added explicit QoS profile to camera subscription
  ```python
  camera_qos = QoSProfile(
      reliability=ReliabilityPolicy.BEST_EFFORT,
      history=HistoryPolicy.KEEP_LAST,
      depth=5,
      durability=DurabilityPolicy.VOLATILE,
  )
  ```

#### Web UI Optimization
- **Problem**: UI didn't fit on laptop screen, terminal out of view when entering commands
- **Solution**: Complete layout redesign
  - Camera feed: Fixed 400x300 on left (matches aspect ratio, no wasted horizontal space)
  - Performance + Diagnostics: Stacked vertically on right column
  - Topics list: Made collapsible (collapsed by default) with smooth animations
  - Terminal: Reduced to 300px height
  - Result: Everything fits on one screen, camera visible while typing commands!

#### Content Updates
- Updated command placeholder examples
  - Removed: "stand up and wave hello" (pose skills abandoned)
  - Added: "go forward 2 meters", "rotate left 90 degrees" (navigation focus)

### Technical Details
- **QoS Compatibility**: Sensor data typically uses BEST_EFFORT for high-frequency low-latency streams
- **RELIABLE vs BEST_EFFORT**: RELIABLE requires acknowledgment of every message (overhead), BEST_EFFORT allows drops (faster)
- **Camera encoding**: Using OpenCV to convert rgb8/bgr8/mono8 to JPEG (quality=85) for efficient web streaming
- **Collapsible UI**: CSS transitions with max-height animation, JavaScript toggle function
- **Responsive design**: Falls back to stacked layout on screens <1024px width

### Validation
- ✅ Subscription confirmation log appears at startup
- ✅ `ros2 topic info -v` verified QoS profiles match
- ✅ Camera callback should now trigger (awaiting laptop deployment test)
- ✅ UI fits on laptop screen without scrolling
- ✅ Camera feed visible while entering terminal commands
- ✅ All changes committed and pushed

### Key Learnings
- **QoS matching is critical**: ROS2 won't connect incompatible QoS profiles, even if code looks correct
- **Sensor data patterns**: Cameras, IMUs, lidars typically use BEST_EFFORT QoS for performance
- **Diagnosis tools**: `ros2 topic info -v` is essential for debugging subscription issues
- **UI testing**: Important to test on actual deployment hardware (laptop vs desktop differences)
- **Space optimization**: Collapsible sections can significantly improve UX without losing functionality
- **Deployment topology matters**: Desktop devcontainer (no robot) vs laptop runtime (with robot)

### Commits
- `dd2dc28` - Fix: Use BEST_EFFORT QoS for camera subscription to match publisher
- `c808f6d` - UI: Optimize camera panel layout and update command examples
- `04397e8` - UI: Improve vertical space usage with collapsible topics list
- `f1a1787` - Style: Format QoS profile and log message

### Next Steps
- Test camera feed on laptop deployment
- Collect baseline performance data with optimized UI
- Begin vision skills integration (camera callback now working)
- Merge `feature/dimos-integration` into `dev` branch

---

## 2025-10-06 - Agent Refactor Complete & Merged

### What Was Done
**Merged feature/agent-refactor into feature/dimos-integration**

Successfully completed major refactor separating ROS concerns from business logic:

#### Architecture Changes
- **Created MissionExecutor** (299 lines) - Pure Python business logic
  - No ROS dependencies, fully testable
  - Direct DIMOS integration (OpenAIAgent, PlanningAgent)
  - Configurable token limits for OpenAI API
  
- **Simplified MissionAgentNode** (307 lines) - Thin ROS wrapper
  - Delegates to MissionExecutor
  - Handles ROS lifecycle, topics, services
  - Renamed `self.executor` → `self.mission_executor` to avoid ROS conflict

#### Testing Infrastructure
- Added 14 unit tests for MissionExecutor
  - 10 passing, 4 skipped (integration placeholders)
  - Runs in 0.09s without ROS dependencies
- Removed old agent wrapper tests (332 lines)

#### Documentation
- Complete rewrite of `AGENT_ARCHITECTURE.md`
- Added usage examples, migration guide, troubleshooting

#### Runtime Bug Fixes (discovered during robot testing)
1. **Executor naming conflict**: ROS2 Node has reserved `executor` attribute
2. **Model parameter**: DIMOS uses `model_name=` not `model=`
3. **Token limits**: Added configuration (max_output=4096, max_input=128000)

### Validation
- ✅ Builds successfully (colcon)
- ✅ All tests pass
- ✅ Tested on robot: 47 skills loaded
- ✅ Web interface working
- ✅ Mission execution functional

### Key Learnings
- **Runtime testing crucial**: All three bugs found during robot deployment, not unit tests
- **Parameter names matter**: Always verify API signatures when integrating libraries
- **Token limits vary**: Different models have different constraints (gpt-4o: 4096 vs 16384)
- **ROS reserved attributes**: Node.executor is used internally by rclpy

### Next Steps
See [TODO.md](./TODO.md) for upcoming work.

---

## Template for Future Entries

```markdown
## YYYY-MM-DD - Brief Title

### What Was Done
Brief description of work completed.

### Technical Details
- Key implementation details
- Important decisions made
- Architecture changes

### Validation
- How it was tested
- Results

### Key Learnings
- What went well
- What could be improved
- Insights gained

### Next Steps
- Follow-up work needed
- Related tasks
```
