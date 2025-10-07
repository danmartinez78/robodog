# ShadowHound Performance Analysis & Optimization Plan

**Date**: October 7, 2025  
**Branch**: `feature/dimos-integration`  
**Status**: Pre-VLM Integration Analysis

---

## 🎯 Objective

Identify and fix latency issues in the DIMOS-integrated mission execution pipeline **before** adding VLM integration, which will add additional 1-3 seconds per query.

## 📊 Current State

### What's Working
- ✅ Vision skills package implemented and tested
- ✅ Web UI with camera feed
- ✅ Nav2 and teleop: Very responsive
- ✅ Basic infrastructure solid

### Observed Issues
- ⏱️ **Agent response latency**: Noticeable delay between command and response
- 🤔 **Unclear bottleneck**: Cloud API vs DIMOS skill execution

### Key Question
> Is the delay from OpenAI cloud API calls or DIMOS skill execution?

## 🔍 Investigation Plan

### Phase 1: Instrumentation ✅ DONE
**Commit**: `f248557`

Added timing breakdown to `mission_executor.py`:
```python
⏱️  Timing breakdown:
   Agent call: X.XXs    # LLM + skill execution
   Total:      X.XXs    # End-to-end
```

### Phase 2: Data Collection (NEXT)

#### Test Scenarios

1. **Simple Commands** (Baseline)
   ```bash
   # Test these commands and record timing
   > take one step forward
   > rotate 90 degrees
   > stop
   ```
   **Expected**: 1-2s (mostly cloud API)

2. **Navigation Commands** (Nav2 skills)
   ```bash
   > go to the kitchen
   > navigate to waypoint A
   ```
   **Expected**: 2-5s (API + skill execution + robot movement)

3. **Complex Commands** (Multi-step)
   ```bash
   > patrol the hallway
   > explore the room and return
   ```
   **Expected**: Variable (depends on planning)

#### Metrics to Collect

For each command:
- **Agent call duration**: `agent_duration` (from logs)
- **Total duration**: `total_duration` (from logs)
- **User-perceived latency**: Time from pressing enter to seeing response
- **Robot response time**: Time from response to robot starting movement

#### Data Collection Template

```markdown
| Command | Agent Call | Total | User Perceived | Robot Action | Notes |
|---------|-----------|-------|----------------|--------------|-------|
| "forward" | 1.2s | 1.3s | ~1.5s | Immediate | Quick |
| "kitchen" | 3.5s | 3.6s | ~4s | 0.5s delay | Nav2 plan |
```

### Phase 3: Analysis

#### Expected Breakdown

```
User Command → Mission Executor → DIMOS Agent → OpenAI API → Skill Execution → Robot
   0ms            +50ms              +100ms        +1-2s          +50-500ms      Response
```

**Hypothesis**:
- **1-2s**: OpenAI API call (GPT-4-turbo inference)
- **50-500ms**: DIMOS skill execution (varies by skill)
- **50-100ms**: ROS/Python overhead

### Phase 4: Optimization Strategies

Based on findings, choose approach:

#### If Bottleneck = Cloud API (Most Likely)

**Options**:
1. **Switch to faster model**
   - gpt-4-turbo → gpt-3.5-turbo (3-5x faster, cheaper)
   - Trade-off: Less capable reasoning

2. **Optimize prompts**
   - Shorter system prompts
   - Remove unnecessary context
   - Stream responses (show progress)

3. **Local LLM** (future)
   - Ollama with llama3 or mistral
   - 100-500ms inference on good hardware
   - Trade-off: Setup complexity, need GPU

4. **Hybrid approach**
   - Simple commands → local/fast model
   - Complex commands → GPT-4
   - Best of both worlds

#### If Bottleneck = DIMOS Skill Execution

**Options**:
1. **Profile individual skills**
   - Add timing to each skill
   - Identify slow skills

2. **Optimize skill implementations**
   - Remove unnecessary waits
   - Parallelize where possible
   - Cache expensive operations

3. **Skill execution feedback**
   - Show "Executing..." message immediately
   - Stream progress updates
   - Better UX even if not faster

#### If Bottleneck = ROS/Communication

**Options**:
1. **Optimize topic communication**
   - Use compressed messages
   - Reduce message size
   - Batch operations

2. **Reduce unnecessary callbacks**
   - Profile callback timing
   - Combine multiple callbacks

---

## 🎬 VLM Integration Impact

### Current Vision Skills Timing

From testing:
- `vision.snapshot`: <50ms (local, no network)
- `vision.describe_scene`: 1-3s (VLM API call)
- `vision.locate_object`: 1-3s (VLM API call)
- `vision.detect_objects`: 1-3s (VLM API call)

### Total Mission Timing with VLM

**Example**: "describe what you see and move forward"

```
User Command
  ↓ 50ms (ROS overhead)
DIMOS Agent
  ↓ 1-2s (OpenAI LLM: understand command + plan)
Vision Skill (describe_scene)
  ↓ 1-3s (Qwen VLM API: analyze image)
Navigation Skill (move forward)
  ↓ 50-500ms (execute movement)
Response
  ↓
Total: 2.5-6s
```

**Implication**: Need to optimize base latency first!

---

## 🚀 Recommended Path Forward

### Option A: Optimize First, Then Integrate VLM ⭐ RECOMMENDED
**Timeline**: 2-3 hours
**Benefit**: Fast baseline + optimized VLM experience

1. ✅ Add timing instrumentation (DONE)
2. ⏳ Collect timing data (15 min)
3. ⏳ Analyze bottlenecks (15 min)
4. ⏳ Implement optimizations (1-2 hours)
   - Likely: Switch to gpt-3.5-turbo for simple commands
   - Add streaming/progress feedback
5. ⏳ Test and verify improvements
6. ⏳ Then integrate VLM with optimized base

**Pro**: Clean, fast system. VLM adds to solid foundation.
**Con**: Delays VLM integration by 2-3 hours.

### Option B: Integrate VLM Now, Optimize Later
**Timeline**: 1-2 hours for VLM, unknown for optimization
**Benefit**: VLM working quickly

1. ⏳ Wire vision skills to mission_agent
2. ⏳ Register with DIMOS MyUnitreeSkills
3. ⏳ Test end-to-end vision missions
4. ⏳ Deal with 4-8s total latency
5. ⏳ Optimize later (harder with more complexity)

**Pro**: VLM features available sooner.
**Con**: Slower system, harder to optimize with VLM complexity added.

### Option C: Parallel Development
**Timeline**: 2-3 hours total
**Benefit**: Both done simultaneously

1. ⏳ You: Test missions, collect timing data, analyze
2. ⏳ Me: Integrate VLM skills with mission_agent
3. ⏳ Merge both when ready

**Pro**: Fastest total time.
**Con**: Requires coordination, potential merge conflicts.

---

## 📋 Action Items

### Immediate (Option A - Recommended)

```bash
# 1. Rebuild with timing instrumentation
cd /workspaces/shadowhound
colcon build --packages-select shadowhound_mission_agent
source install/setup.bash

# 2. Launch agent
ros2 launch shadowhound_bringup shadowhound_bringup.launch.py

# 3. Test commands and watch logs for timing
# In web UI or via:
ros2 topic pub /mission_command std_msgs/String "data: 'take one step forward'"

# 4. Collect timing from logs:
# Look for: "⏱️  Timing breakdown:"
```

### Data to Collect

| Command Type | Example | Expected Agent Call | Notes |
|--------------|---------|-------------------|-------|
| Simple motion | "forward" | 1-2s | Baseline |
| Simple motion | "rotate 90" | 1-2s | Baseline |
| Navigation | "go to kitchen" | 2-4s | Nav2 planning |
| Complex | "patrol" | 3-6s | Multi-step |

### Analysis Questions

1. Is agent_duration consistently 1-2s? → **Cloud API bottleneck**
2. Does agent_duration vary widely? → **Skill execution varies**
3. Is total_duration much > agent_duration? → **Overhead issue**

---

## 🎯 Success Criteria

### Before VLM Integration
- ✅ Understand where latency comes from
- ✅ Optimize bottlenecks (target: <2s for simple commands)
- ✅ Add progress feedback for better UX
- ✅ Document baseline performance

### After VLM Integration
- ✅ Vision missions work end-to-end
- ✅ Total latency <5s for vision+action commands
- ✅ Good UX with progress indicators
- ✅ System feels responsive

---

## 📚 References

- **Timing code**: `mission_executor.py` line 214
- **Agent call**: `run_observable_query().run()` (DIMOS OpenAIAgent)
- **Vision skills**: `src/shadowhound_skills/shadowhound_skills/vision.py`
- **DIMOS docs**: `docs/DIMOS_VISION_CAPABILITIES.md`

---

## 🔮 Future Optimizations

### Short-term (After VLM)
- [ ] Stream LLM responses (show thinking in real-time)
- [ ] Parallel skill execution where possible
- [ ] Cache common queries/responses
- [ ] Compressed image topic (reduce bandwidth)

### Long-term
- [ ] Local LLM for simple commands
- [ ] Hybrid cloud/local routing
- [ ] Predictive skill loading
- [ ] Advanced caching with embeddings

---

## 💡 Recommendation

**Start with Option A**: Optimize base system first (2-3 hours), then add VLM.

**Why**: VLM will add 1-3s per query. If base system is already slow, VLM missions will be 4-8s total - poor UX. Fix the foundation first, then build on it.

**Next Step**: Run test missions and collect timing data. Should take ~15 minutes to understand the bottleneck clearly.
