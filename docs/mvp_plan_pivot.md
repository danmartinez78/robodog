# MVP Development Plan - Pivot Decision

**Date:** October 6, 2025  
**Decision:** Abandon WebRTC skills debugging, focus on working Nav2 stack for MVP

---

## Context: Why We're Pivoting

### What We Discovered
1. ✅ **Nav2 works reliably** - Navigation, move commands, spin actions all functional
2. ❌ **WebRTC skills are problematic** - Queue hangs, mode conflicts, breaks Nav2
3. ⏰ **Time investment** - Days spent on debugging with unclear path to solution
4. 🎯 **MVP goal** - Demo autonomous navigation with vision, not skill tricks

### The Pragmatic Choice
**Use what works.** Nav2 provides everything needed for MVP:
- ✅ Go to waypoint (position + orientation)
- ✅ Rotate in place
- ✅ Stop/cancel
- ✅ Obstacle avoidance
- ✅ Path planning

WebRTC skills (sit, stand, wave) are nice-to-have, not critical for MVP.

---

## MVP Scope - What We're Building

### Core Demo Flow
```
User: "Go explore the kitchen and tell me what you see"
  ↓
Agent: Plans mission using Nav2 + Vision
  ↓
1. Navigate to kitchen waypoint (Nav2)
2. Rotate to scan area (Nav2 spin)
3. Capture images from multiple angles (Vision)
4. Process images with VLM (AI Agent)
5. Generate natural language report
6. Return to start position (Nav2)
```

### Required Components
1. **Nav2 Skills** (WORKING) ✅
   - `nav.goto(x, y, yaw)` - Navigate to pose
   - `nav.spin(angle)` - Rotate in place
   - `nav.stop()` - Cancel current action

2. **Vision Skills** (NEED TO IMPLEMENT) 🔨
   - `vision.snapshot()` - Capture single image
   - `vision.scan(num_angles)` - Capture panorama
   - `vision.detect_objects()` - Run object detection

3. **Lidar Skills** (NEED TO IMPLEMENT) 🔨
   - `lidar.get_obstacles()` - Get nearby obstacles
   - `lidar.is_path_clear(direction)` - Check if path blocked
   - `lidar.get_map()` - Get current occupancy grid

4. **AI Agent** (PARTIAL) 🔨
   - Mission planner (basic version exists)
   - VLM integration for image understanding
   - Natural language report generation
   - Mission execution with Skills API

---

## Implementation Plan

### Phase 1: Vision Integration (Priority 1) 🔴

**Goal:** Capture and process camera images

**Tasks:**
1. Create `vision.py` in shadowhound_skills
   ```python
   @register_skill("vision.snapshot")
   class SnapshotSkill(Skill):
       """Capture single image from front camera"""
       
   @register_skill("vision.scan")
   class ScanSkill(Skill):
       """Capture panorama by rotating and snapshotting"""
   ```

2. Subscribe to camera topics
   - Find camera topics: `ros2 topic list | grep camera`
   - Likely: `/front_camera/image_raw` or similar
   - Handle compressed images if needed

3. Save images with metadata
   - Timestamp
   - Robot pose (x, y, yaw)
   - Image filename/path

4. Test vision skills
   ```bash
   # Via CLI
   python3 -m shadowhound_skills.cli vision.snapshot
   
   # Via mission agent
   curl -X POST http://localhost:8000/execute \
     -d '{"skill": "vision.snapshot"}'
   ```

**Acceptance:**
- ✅ Can capture image on demand
- ✅ Image saved to disk with correct format
- ✅ Metadata recorded (pose, timestamp)
- ✅ Works through Skills API

---

### Phase 2: Lidar Integration (Priority 2) 🟠

**Goal:** Use lidar for spatial awareness

**Tasks:**
1. Create `lidar.py` in shadowhound_skills
   ```python
   @register_skill("lidar.get_obstacles")
   class GetObstaclesSkill(Skill):
       """Return list of nearby obstacles with distances"""
       
   @register_skill("lidar.is_path_clear")
   class PathClearSkill(Skill):
       """Check if path is clear in given direction"""
   ```

2. Subscribe to lidar topics
   - Find: `ros2 topic list | grep -i lidar`
   - Likely: `/scan` or `/lidar/scan`
   - Parse LaserScan or PointCloud2

3. Process lidar data
   - Convert to obstacle list (distance, angle)
   - Implement path clearance check
   - Handle min/max range filtering

4. Test lidar skills
   ```bash
   python3 -m shadowhound_skills.cli lidar.get_obstacles
   ```

**Acceptance:**
- ✅ Can detect obstacles in front of robot
- ✅ Can determine if path is clear
- ✅ Returns structured data (not just raw scan)

---

### Phase 3: VLM Integration (Priority 3) 🟡

**Goal:** AI understands what camera sees

**Tasks:**
1. Add VLM client to mission agent
   ```python
   # In shadowhound_mission_agent/
   class VLMClient:
       def analyze_image(self, image_path: str, prompt: str) -> str:
           """Send image to GPT-4V/Claude and get description"""
   ```

2. Create vision understanding skills
   ```python
   @register_skill("vision.describe")
   class DescribeSkill(Skill):
       """Use VLM to describe what's in latest image"""
       
   @register_skill("vision.answer")
   class AnswerSkill(Skill):
       """Ask VLM a question about latest image"""
   ```

3. Integrate with OpenAI/Anthropic APIs
   - Use GPT-4 Vision API
   - Or Claude 3 with vision
   - Handle base64 encoding, API calls
   - Parse responses

4. Test VLM understanding
   ```bash
   # Capture image
   python3 -m shadowhound_skills.cli vision.snapshot
   
   # Ask VLM about it
   python3 -m shadowhound_skills.cli vision.describe
   # Should return: "I see a kitchen with a table..."
   ```

**Acceptance:**
- ✅ Can send image to VLM API
- ✅ Receives natural language description
- ✅ Can answer specific questions about image
- ✅ Handles API errors gracefully

---

### Phase 4: Mission Planning (Priority 4) 🟢

**Goal:** Chain skills into coherent missions

**Tasks:**
1. Enhance mission planner
   ```python
   # In shadowhound_mission_agent/
   class MissionPlanner:
       def plan_exploration(self, target: str) -> List[SkillCall]:
           """Convert 'explore kitchen' to skill sequence"""
           return [
               {"skill": "nav.goto", "args": {"x": 5.0, "y": 2.0}},
               {"skill": "nav.spin", "args": {"angle": 6.28}},  # Full rotation
               {"skill": "vision.scan", "args": {"num_angles": 8}},
               {"skill": "vision.describe", "args": {}},
               {"skill": "nav.goto", "args": {"x": 0.0, "y": 0.0}},
           ]
   ```

2. Add waypoint database
   ```yaml
   # In configs/waypoints.yaml
   waypoints:
     kitchen:
       x: 5.0
       y: 2.0
       yaw: 1.57
     living_room:
       x: -3.0
       y: 1.0
       yaw: 0.0
   ```

3. Implement error recovery
   - If navigation fails → retry or abort
   - If vision fails → retry capture
   - If VLM fails → fallback to simple description

4. Test full mission
   ```bash
   # Launch stack
   ros2 launch shadowhound_bringup shadowhound_bringup.launch.py
   
   # Send mission via web UI
   "Go to the kitchen and tell me what you see"
   ```

**Acceptance:**
- ✅ Can parse natural language mission
- ✅ Generates valid skill sequence
- ✅ Executes sequence with error handling
- ✅ Returns mission report to user

---

### Phase 5: Polish & Demo (Priority 5) ⭐

**Goal:** Make it demo-ready

**Tasks:**
1. **Web UI improvements**
   - Show robot position on map
   - Display captured images
   - Show mission progress
   - Stream VLM responses

2. **Logging & telemetry**
   - Mission timeline
   - Skill execution times
   - Success/failure tracking
   - Error logs

3. **Demo script**
   ```markdown
   1. Show robot in home position
   2. Give mission: "Explore the kitchen"
   3. Watch robot navigate
   4. See images captured
   5. Read VLM description
   6. Robot returns home
   ```

4. **Documentation**
   - User guide for running demo
   - Architecture diagram
   - Skills API reference
   - Troubleshooting guide

**Acceptance:**
- ✅ Demo runs reliably (90%+ success rate)
- ✅ Looks impressive (smooth nav, good images, smart descriptions)
- ✅ Can explain architecture clearly
- ✅ Recovery from common failures

---

## What We're NOT Building (For Now)

### Deferred to Post-MVP
- ❌ WebRTC skills (sit, stand, wave) - debugging rabbit hole
- ❌ Multi-robot coordination - single robot is enough
- ❌ SLAM/mapping - use pre-made map
- ❌ Dynamic obstacle avoidance - lidar-based only
- ❌ Voice control - text commands are fine
- ❌ Real-time streaming - batch image processing ok
- ❌ ML on robot - cloud APIs are fine

### Why These Are Deferred
Not because they're unimportant, but because:
1. **Time to value** - MVP can work without them
2. **Complexity** - Each adds significant dev time
3. **Demo impact** - Won't make demo more impressive
4. **Working alternatives** - Nav2 replaces WebRTC skills

---

## Success Metrics

### MVP is successful if:
1. ✅ **Robot navigates autonomously** to commanded waypoints
2. ✅ **Captures useful images** from camera
3. ✅ **VLM understands images** and generates descriptions
4. ✅ **Natural language interface** works (user → mission → execution → report)
5. ✅ **90%+ mission success rate** in controlled environment
6. ✅ **Demo runs smoothly** in front of audience

### Bonus points for:
- Lidar-based obstacle detection working
- Multi-waypoint missions
- Recovery from navigation failures
- Beautiful web UI

---

## Timeline Estimate

Assuming focused development time:

| Phase | Effort | Status |
|-------|--------|--------|
| Phase 1: Vision | 1-2 days | 🔨 Next |
| Phase 2: Lidar | 1 day | ⏳ After vision |
| Phase 3: VLM | 1-2 days | ⏳ After lidar |
| Phase 4: Planning | 1-2 days | ⏳ After VLM |
| Phase 5: Polish | 1-2 days | ⏳ Final |
| **Total** | **5-9 days** | 🎯 MVP complete |

**Key assumption:** No major blocking issues with camera/lidar topics

---

## Risk Mitigation

### High Risk Items
1. **Camera topic unavailable** 
   - Mitigation: Test camera topics first day
   - Fallback: Use pre-captured images for demo

2. **VLM API costs high**
   - Mitigation: Cache responses, limit calls
   - Fallback: Use cheaper model (GPT-4V mini)

3. **Nav2 reliability issues**
   - Mitigation: Test thoroughly, tune parameters
   - Fallback: Pre-scripted waypoints that work

4. **Integration complexity**
   - Mitigation: Test each skill independently first
   - Fallback: Simplify mission (fewer skills)

### Medium Risk Items
- Lidar processing performance
- Image quality in different lighting
- Network latency to VLM APIs
- ROS topic discovery/naming

---

## WebRTC Skills - Future Investigation

**Not abandoned forever, just deprioritized.**

If time permits after MVP, or if mode conflict fix is obvious:
- Document the mode switching issue fully
- Test mode field monitoring
- Implement mode restore in queue
- Re-enable WebRTC skills

But: **MVP success doesn't depend on this.**

---

## Next Immediate Steps

### Tomorrow's Work (Start of Phase 1)
1. ✅ Find camera topic
   ```bash
   ros2 topic list | grep -i camera
   ros2 topic info /camera_topic
   ros2 topic echo /camera_topic --once
   ```

2. ✅ Create vision.py skeleton
   ```bash
   cd src/shadowhound_skills/shadowhound_skills/skills/
   touch vision.py
   ```

3. ✅ Implement basic snapshot skill
   - Subscribe to camera topic
   - Save one image to disk
   - Return success/failure

4. ✅ Test via CLI
   ```bash
   python3 -m shadowhound_skills.cli vision.snapshot
   ls /tmp/shadowhound_images/  # Should see image
   ```

**Goal for day 1:** Capture at least one image successfully.

---

## Questions Before We Start

1. **Camera resolution preference?** 
   - Higher res = better VLM accuracy but slower
   - Lower res = faster but less detail
   - Recommendation: 640x480 or 1280x720

2. **VLM provider?**
   - OpenAI GPT-4 Vision (most capable, expensive)
   - Claude 3 Sonnet with vision (fast, mid-cost)
   - Claude 3 Haiku with vision (cheapest, still good)
   - Recommendation: Start with GPT-4V, optimize later

3. **Image storage?**
   - Save all images permanently?
   - Or just latest N images?
   - Recommendation: Keep latest 100, older ones deleted

4. **Demo environment?**
   - Indoor office/lab?
   - Outdoor?
   - Recommendation: Indoor, controlled lighting

---

**Status:** Plan documented. Ready to pivot from WebRTC debugging to vision-first MVP development. Next session starts with camera topic discovery. 🚀

**Philosophy:** Perfect is the enemy of done. Ship the MVP with Nav2, add tricks later. 📦
