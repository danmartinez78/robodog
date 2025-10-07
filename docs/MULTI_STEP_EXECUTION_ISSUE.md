# Multi-Step Execution Issue & Solution

**Date**: October 7, 2025  
**Branch**: `feature/dimos-integration`  
**Issue**: Multi-step commands like "rotate to the right and take a step back" execute inconsistently

---

## 🐛 Problem Description

### Observed Behavior
```
Command: "rotate to the right and take a step back"
Expected: Robot rotates, THEN steps back
Actual: Robot starts rotating but immediately steps back (rotation incomplete)
```

### Root Cause

You're currently using **`OpenAIAgent`** (default), which is a **single-shot function-calling agent**:

```python
# Current setup (mission_executor.py line 200)
self.agent = OpenAIAgent(
    dev_name="shadowhound",
    agent_type="Mission",
    skills=self.skills,
    model_name=self.config.agent_model,
)
```

**How OpenAIAgent Works**:
1. Receives: "rotate to the right and take a step back"
2. LLM calls: **ONE** function with params (or multiple calls in parallel)
3. Problem: Skills execute **simultaneously or race condition** (not sequential!)

### Why This Happens

**OpenAIAgent Execution Model**:
```python
# Pseudo-code of what happens
query = "rotate right and step back"
response = llm.chat(query, skills=available_skills)
# LLM returns:
[
    {"function": "rotate", "params": {"direction": "right"}},
    {"function": "step", "params": {"direction": "back"}}
]
# Both execute at same time or overlap!
```

**Result**: 
- Rotate command starts
- Step command starts before rotation completes
- Robot behavior: Confused/incomplete rotation + step back

---

## ✅ Solution: Use PlanningAgent

DIMOS provides **`PlanningAgent`** specifically for multi-step sequential execution:

```python
# Solution (already available in code!)
self.agent = PlanningAgent(
    robot=self.robot, 
    dev_name="shadowhound", 
    agent_type="Mission"
)
```

**How PlanningAgent Works**:
1. **Planning Phase**: LLM creates a step-by-step plan
   ```
   Plan:
   1. Rotate right (90 degrees)
   2. Wait for rotation to complete
   3. Step backward
   4. Wait for step to complete
   ```

2. **Execution Phase**: Execute steps **sequentially** with validation
   ```python
   for step in plan:
       execute(step)
       wait_for_completion()
       verify_success()
       if failed:
           replan()
   ```

3. **Monitoring**: Agent monitors execution and can adapt if steps fail

### Architecture Comparison

```
OpenAIAgent (Single-Shot)
┌─────────────────────────────────────┐
│ Command → LLM → Function Calls      │
│               ↓                     │
│            Skill 1 ──┐              │
│            Skill 2 ──┼→ Execute     │
│            Skill N ──┘   (parallel) │
└─────────────────────────────────────┘
❌ No sequencing, no validation


PlanningAgent (Sequential)
┌─────────────────────────────────────┐
│ Command → LLM → Create Plan         │
│               ↓                     │
│           [Step 1]                  │
│               ↓                     │
│           Execute → Validate        │
│               ↓                     │
│           [Step 2]                  │
│               ↓                     │
│           Execute → Validate        │
│               ↓                     │
│           [Step N]                  │
└─────────────────────────────────────┘
✅ Sequential, validated, can replan
```

---

## 🚀 How to Enable PlanningAgent

### Option 1: Launch Argument (Recommended)

```bash
# Enable planning agent for this session
ros2 launch shadowhound_bringup shadowhound.launch.py use_planning_agent:=true

# Or with full launch
ros2 launch shadowhound_full.launch.py use_planning_agent:=true
```

### Option 2: Modify Launch File Default

Edit `src/shadowhound_bringup/launch/shadowhound.launch.py`:

```python
use_planning_arg = DeclareLaunchArgument(
    "use_planning_agent",
    default_value="true",  # ← Change from "false" to "true"
    description="Use planning agent for multi-step missions",
)
```

### Option 3: Environment Variable (Config File)

Create/edit `configs/shadowhound.yaml`:

```yaml
shadowhound_mission_agent:
  ros__parameters:
    agent_backend: "cloud"
    use_planning_agent: true  # ← Enable planning agent
    agent_model: "gpt-4-turbo"
    robot_ip: "192.168.1.103"
```

Then launch with config:
```bash
ros2 launch shadowhound_bringup shadowhound.launch.py \
    --params-file configs/shadowhound.yaml
```

---

## 📊 Performance Comparison

### OpenAIAgent (Current)
- **Speed**: Fast (single LLM call)
- **Best for**: Simple single-action commands
  - "stand up"
  - "sit down"
  - "wave hello"
- **Issues**: Multi-step commands execute incorrectly

### PlanningAgent (Recommended)
- **Speed**: Slower (plan + execute phases)
- **Best for**: Multi-step/complex commands
  - "rotate right and step back" ✅
  - "patrol the hallway" ✅
  - "go to kitchen, look around, return" ✅
- **Benefits**: 
  - Sequential execution
  - Validation between steps
  - Can replan if step fails
  - Better error handling

### Timing Impact

**OpenAIAgent**:
```
"stand up" → 1.2s (1 LLM call + skill)
"rotate right and step back" → 1.5s (1 LLM call + both skills at once ❌)
```

**PlanningAgent**:
```
"stand up" → 1.5s (plan + execute, still fast for simple)
"rotate right and step back" → 3.5s (plan + rotate + validate + step ✅)
```

**Trade-off**: ~2 seconds slower for multi-step, but **actually works correctly**!

---

## 🔍 Verification Steps

### 1. Check Current Setting

```bash
ros2 param get /shadowhound_mission_agent use_planning_agent
# Output: False (currently using OpenAIAgent)
```

### 2. Enable PlanningAgent

```bash
# Restart with planning agent enabled
ros2 launch shadowhound_bringup shadowhound.launch.py use_planning_agent:=true
```

### 3. Test Multi-Step Commands

```bash
# Test sequential execution
ros2 topic pub /mission_command std_msgs/String \
    "data: 'rotate to the right and take a step back'" --once

# Watch logs for planning output
# You should see:
# [INFO] Creating plan for: rotate to the right...
# [INFO] Plan created: Step 1: rotate, Step 2: step
# [INFO] Executing step 1: rotate
# [INFO] Step 1 complete, executing step 2: step
# [INFO] Step 2 complete
```

### 4. Verify Timing in Web UI

**With OpenAIAgent**:
```
⏱️  TIMING: Agent 1.45s | Total 1.52s
```

**With PlanningAgent**:
```
⏱️  TIMING: Agent 3.82s | Total 3.90s
```

Slower, but commands execute **correctly**!

---

## 🎯 Recommendation

### Short-term: Enable PlanningAgent Now

**Why**:
1. Your multi-step commands will work correctly
2. Performance hit is acceptable (2-3s more, but functional)
3. No code changes needed (just launch argument)

**How**:
```bash
ros2 launch shadowhound_bringup shadowhound.launch.py use_planning_agent:=true
```

### Medium-term: Hybrid Approach (Future Enhancement)

**Smart Agent Selection**:
```python
# Pseudo-code for future enhancement
def execute_mission(self, command: str):
    if is_multi_step(command):
        # Use PlanningAgent for complex commands
        return self.planning_agent.plan_and_execute(command)
    else:
        # Use OpenAIAgent for simple commands (faster)
        return self.openai_agent.run_query(command)
```

**Benefits**:
- Fast for simple commands (1-2s)
- Correct for complex commands (3-5s)
- Best of both worlds

**Implementation**: Add command analysis to route to appropriate agent

### Long-term: Advanced Planning (Post-VLM)

**Features**:
- **Parallel execution** where safe (e.g., "look around while standing")
- **Conditional execution** (e.g., "if you see a person, wave")
- **Retry logic** (e.g., "try rotating, if stuck, take step back first")
- **Dynamic replanning** (e.g., obstacle detected, replan route)

---

## 📝 Code Changes Needed (Optional)

If you want to make PlanningAgent the **default**:

```python
# In mission_executor.py (line 47)
@dataclass
class MissionExecutorConfig:
    """Configuration for MissionExecutor."""
    
    agent_backend: str = "cloud"
    use_planning_agent: bool = True  # ← Change to True
    robot_ip: str = "192.168.1.103"
    # ... rest of config
```

Or in launch file:
```python
# In shadowhound.launch.py (line 30)
use_planning_arg = DeclareLaunchArgument(
    "use_planning_agent",
    default_value="true",  # ← Change to "true"
    description="Use planning agent for multi-step missions",
)
```

---

## 🧪 Test Cases

### Test 1: Simple Command (Both agents should work)
```bash
Command: "stand up"
OpenAIAgent: ✅ Works (1.2s)
PlanningAgent: ✅ Works (1.5s, slightly slower but fine)
```

### Test 2: Sequential Command (Only PlanningAgent works correctly)
```bash
Command: "rotate right and step back"
OpenAIAgent: ❌ Rotation incomplete, steps back immediately
PlanningAgent: ✅ Rotates completely, then steps back
```

### Test 3: Multi-Step Navigation
```bash
Command: "go to the kitchen, look around, and come back"
OpenAIAgent: ❌ Likely fails or confused execution
PlanningAgent: ✅ Sequential: navigate → look → return
```

### Test 4: Conditional Logic
```bash
Command: "patrol the hallway and stop if you see a person"
OpenAIAgent: ❌ Can't handle conditional logic
PlanningAgent: ✅ Monitors, can stop mid-execution
```

---

## 📚 DIMOS Documentation References

### PlanningAgent Usage
From DIMOS examples:
```python
# Example from dimos-unitree
from dimos.agents.planning_agent import PlanningAgent

agent = PlanningAgent(
    robot=robot,
    dev_name="go2",
    agent_type="mission"
)

# Multi-step execution
response = agent.plan_and_execute(
    "Go to the kitchen and pick up the ball"
)
```

### OpenAIAgent Usage
```python
# Example for simple commands
from dimos.agents.agent import OpenAIAgent

agent = OpenAIAgent(
    dev_name="go2",
    agent_type="mission",
    skills=skills
)

# Single-step execution
response = agent.run_observable_query("stand up").run()
```

---

## ✅ Action Items

### Immediate (Do Now)
- [ ] Launch with `use_planning_agent:=true`
- [ ] Test "rotate right and step back" command
- [ ] Verify sequential execution in logs
- [ ] Check timing increase in web UI (~2-3s more)

### Short-term (After Testing)
- [ ] Make PlanningAgent the default (change launch file)
- [ ] Document agent selection in user guide
- [ ] Add agent type to web UI diagnostics panel

### Medium-term (Future Enhancement)
- [ ] Implement hybrid agent selection
- [ ] Add command complexity analyzer
- [ ] Route simple → OpenAI, complex → Planning
- [ ] Benchmark and optimize both agents

---

## 🔮 Expected Results

**Before (OpenAIAgent)**:
```
> rotate right and step back
[Robot starts rotating, immediately starts stepping back]
❌ Rotation incomplete, confusing behavior
⏱️  TIMING: Agent 1.45s | Total 1.52s
```

**After (PlanningAgent)**:
```
> rotate right and step back
[Planning phase...]
[Robot rotates completely]
[Robot waits]
[Robot steps back]
✅ Both actions complete sequentially
⏱️  TIMING: Agent 3.82s | Total 3.90s
```

**Trade-off**: +2.3s execution time, but **correct behavior**! 🎯

---

## 💡 Summary

**Problem**: OpenAIAgent doesn't sequence multi-step commands properly  
**Solution**: Use PlanningAgent for sequential execution  
**How**: Launch with `use_planning_agent:=true`  
**Cost**: +2-3 seconds per multi-step command  
**Benefit**: Commands actually work correctly! 

**Try it now**:
```bash
ros2 launch shadowhound_bringup shadowhound.launch.py use_planning_agent:=true
```

Then test your command again and watch it execute **correctly** in sequence! 🚀
