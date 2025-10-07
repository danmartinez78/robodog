# Web UI Performance Metrics

**Date**: October 7, 2025  
**Branch**: `feature/dimos-integration`  
**Commit**: `0790578`

---

## 🎯 Overview

Added comprehensive performance timing instrumentation throughout the mission execution pipeline with real-time visualization in the web UI.

## 📊 What's New

### 1. **Performance Metrics Panel** (New Dashboard Section)

Located in the top-right of the dashboard, shows:

```
⚡ PERFORMANCE METRICS
┌─────────────────────────────────────┐
│ AGENT CALL      OVERHEAD            │
│ 1.23s           0.045s              │
│                                     │
│ TOTAL TIME      COMMANDS            │
│ 1.28s           42                  │
├─────────────────────────────────────┤
│ Averages (Last 50 Commands)        │
│ AVG AGENT       AVG TOTAL           │
│ 1.45s           1.52s               │
└─────────────────────────────────────┘
```

**Color Coding**:
- 🟢 **Green**: Good performance (Agent <2s, Total <3s)
- 🟡 **Yellow**: Warning (Agent 2-4s, Total 3-5s)
- 🔴 **Red**: Needs optimization (Agent >4s, Total >5s)

### 2. **Terminal Timing Output**

After every command execution, timing info appears in terminal:

```
[14:32:15] > take one step forward
[14:32:17] ✅ Robot moved forward successfully
[14:32:17] ⏱️  TIMING: Agent 1.23s | Overhead 0.045s | Total 1.28s
```

### 3. **Backend Timing Instrumentation**

**mission_executor.py**:
```python
def execute_mission(self, command: str) -> tuple[str, dict]:
    """Returns (response, timing_info)"""
    # Measures:
    # - agent_duration: LLM call + skill execution
    # - overhead_duration: ROS/Python overhead
    # - total_duration: End-to-end
    # - agent_percentage: % of time in agent
```

**Timing Dict Structure**:
```python
{
    "agent_duration": 1.23,      # Seconds in DIMOS agent
    "total_duration": 1.28,      # Total execution time
    "overhead_duration": 0.045,  # ROS/Python overhead
    "agent_percentage": 96.1,    # % time in agent
    "avg_agent_duration": 1.45,  # Running average
    "avg_total_duration": 1.52,  # Running average
    "command_count": 42          # Total commands executed
}
```

---

## 🔍 What to Look For

### Baseline Metrics (Expected)

#### Simple Commands
```bash
> "take one step forward"
Agent:    1.0-2.0s  (OpenAI LLM inference)
Overhead: 0.03-0.1s (ROS messaging, Python)
Total:    1.1-2.1s
```

#### Navigation Commands
```bash
> "go to the kitchen"
Agent:    2.0-4.0s  (LLM + Nav2 planning)
Overhead: 0.05-0.2s
Total:    2.1-4.2s
```

#### Complex/Multi-step
```bash
> "patrol the hallway"
Agent:    3.0-6.0s  (Multi-step planning)
Overhead: 0.1-0.3s
Total:    3.2-6.3s
```

### Performance Bottleneck Analysis

**If Agent Duration >> Overhead:**
- **Bottleneck**: Cloud API (OpenAI LLM)
- **Solution**: 
  - Switch to gpt-3.5-turbo (3-5x faster)
  - Optimize prompts (shorter, fewer tokens)
  - Use streaming responses
  - Consider local LLM for simple commands

**If Overhead is Significant (>10% of total):**
- **Bottleneck**: ROS/Python communication
- **Solution**:
  - Profile callback timing
  - Optimize message passing
  - Reduce unnecessary processing

**If Agent Duration varies wildly:**
- **Bottleneck**: DIMOS skill execution
- **Solution**:
  - Profile individual skills
  - Optimize slow skills
  - Parallelize where possible

---

## 🚀 Usage Guide

### 1. **Start the System**

```bash
# Terminal 1: Build and source
cd /workspaces/shadowhound
colcon build --packages-select shadowhound_mission_agent --symlink-install
source install/setup.bash

# Terminal 2: Launch mission agent
ros2 launch shadowhound_bringup shadowhound_bringup.launch.py

# Terminal 3: Open web browser
$BROWSER http://localhost:8080
```

### 2. **Send Test Commands**

Via Web UI:
- Use the command input box at the bottom
- Click quick command buttons
- Watch metrics update in real-time

Via ROS Topic:
```bash
ros2 topic pub /mission_command std_msgs/String "data: 'stand up'" --once
```

### 3. **Monitor Performance**

**Watch the Dashboard**:
- Performance panel updates every 1 second
- Color changes indicate performance trends
- Averages smooth out outliers

**Check ROS Logs**:
```bash
# See detailed timing breakdown
ros2 topic echo /mission_status

# Or view node logs directly
ros2 run shadowhound_mission_agent mission_agent
```

### 4. **Collect Baseline Data**

Create a test script to systematically measure performance:

```bash
# test_performance.sh
#!/bin/bash

COMMANDS=(
    "stand up"
    "sit down"
    "wave hello"
    "take one step forward"
    "rotate 90 degrees"
    "go to the kitchen"
    "patrol the hallway"
)

for cmd in "${COMMANDS[@]}"; do
    echo "Testing: $cmd"
    ros2 topic pub /mission_command std_msgs/String "data: '$cmd'" --once
    sleep 5  # Wait for completion
done

echo "Performance data collected. Check web UI for averages."
```

---

## 📈 Data Collection Template

| Command Type | Example | Agent (s) | Overhead (ms) | Total (s) | Notes |
|--------------|---------|-----------|---------------|-----------|-------|
| Simple motion | "stand" | 1.2 | 45 | 1.25 | Baseline |
| Simple motion | "sit" | 1.3 | 42 | 1.34 | Baseline |
| Simple motion | "wave" | 1.4 | 50 | 1.45 | Baseline |
| Navigation | "kitchen" | 3.5 | 78 | 3.58 | Nav2 planning |
| Navigation | "waypoint" | 2.8 | 65 | 2.87 | Direct nav |
| Complex | "patrol" | 5.2 | 120 | 5.32 | Multi-step |
| Complex | "explore" | 6.1 | 150 | 6.25 | Planning heavy |

**Analysis**:
- Average Agent: ? s
- Average Total: ? s
- Agent % of Total: ?%
- Bottleneck: Cloud API / DIMOS / Overhead

---

## 🎯 Optimization Targets

Based on user expectations and VLM impact:

### Short-term (Before VLM Integration)
- **Target**: Agent <2s for simple commands
- **Why**: VLM will add 1-3s, need fast baseline

### Medium-term (With VLM)
- **Target**: Total <5s for vision+action commands
- **Why**: Acceptable UX for robot control

### Long-term (Production)
- **Target**: Agent <1s for 80% of commands
- **Why**: Snappy, responsive feel
- **How**: Local LLM for simple commands, cloud for complex

---

## 🔧 Technical Details

### API Endpoints

**GET /api/performance**
```json
{
  "latest": {
    "agent_duration": 1.23,
    "total_duration": 1.28,
    "overhead_duration": 0.045,
    "agent_percentage": 96.1,
    "avg_agent_duration": 1.45,
    "avg_total_duration": 1.52,
    "command_count": 42
  },
  "history": [
    {
      "agent_duration": 1.15,
      "total_duration": 1.20,
      "overhead_duration": 0.050,
      "agent_percentage": 95.8,
      "timestamp": "2025-10-07T14:32:15.123456"
    },
    // ... last 20 measurements for graphing
  ]
}
```

### WebSocket Messages

**Timing Broadcast** (after each command):
```
⏱️  TIMING: Agent 1.23s | Overhead 0.045s | Total 1.28s
```

### Storage Limits

- **Performance History**: Last 50 measurements
- **Terminal Buffer**: Last 1000 lines
- **API History Response**: Last 20 measurements (for graphing)

---

## 🐛 Troubleshooting

### Metrics Not Updating

**Check**:
1. Web UI connected? (Status indicator at top)
2. Commands being executed? (Check terminal output)
3. Browser console errors? (F12 Developer Tools)

**Fix**:
```bash
# Restart mission agent
ros2 run shadowhound_mission_agent mission_agent

# Clear browser cache and reload
```

### Metrics Show "0.00s" or "--"

**Cause**: No commands executed yet

**Fix**: Send a test command

### Performance Seems Slow

**Expected**:
- First command after launch: Slower (DIMOS initialization)
- Subsequent commands: Faster (cached connections)

**Check**:
1. Network latency to OpenAI? (ping openai.com)
2. DIMOS agent initialized? (check logs)
3. Robot connected? (check diagnostics panel)

---

## 📚 Related Files

- **Backend**: `src/shadowhound_mission_agent/shadowhound_mission_agent/mission_executor.py`
- **Agent**: `src/shadowhound_mission_agent/shadowhound_mission_agent/mission_agent.py`
- **Web Interface**: `src/shadowhound_mission_agent/shadowhound_mission_agent/web_interface.py`
- **Dashboard**: `src/shadowhound_mission_agent/shadowhound_mission_agent/dashboard_template.html`
- **Analysis Plan**: `docs/PERFORMANCE_ANALYSIS_PLAN.md`

---

## 🔮 Future Enhancements

### Phase 1: Enhanced Visualization
- [ ] Line graph of timing over time
- [ ] Histogram of agent duration distribution
- [ ] Performance trend indicators (improving/degrading)
- [ ] Skill-level breakdown (which skills are slow?)

### Phase 2: Advanced Analysis
- [ ] Automatic bottleneck detection
- [ ] Performance regression alerts
- [ ] Comparison: cloud vs local LLM
- [ ] Export performance data to CSV

### Phase 3: Optimization Recommendations
- [ ] AI-powered optimization suggestions
- [ ] Model selection recommendations
- [ ] Skill execution parallelization hints
- [ ] Cache effectiveness metrics

---

## ✅ Success Criteria

Before proceeding to VLM integration, achieve:

- [x] Timing instrumentation in place ✅
- [x] Web UI metrics panel working ✅
- [x] Terminal timing output showing ✅
- [ ] Baseline data collected (15 min task)
- [ ] Bottleneck identified (cloud/DIMOS/overhead)
- [ ] Optimization strategy defined
- [ ] Target: Agent <2s for simple commands

**Next Step**: Run test missions and collect timing data! 🚀
