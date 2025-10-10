# Quick Start: Robot Testing with phi4:14b

**Date**: 2025-10-11  
**Goal**: Test Ollama local LLM (phi4:14b) with GO2 robot  
**Time Estimate**: 2-3 hours total

---

## ☕ Morning Setup (30 min)

### Step 1: Reboot Thor for Clean GPU State
```bash
# On Thor
sudo reboot
```
**Wait ~2 minutes for system to come back up**

---

### Step 2: Install jtop for GPU Monitoring
```bash
# SSH back into Thor after reboot
ssh thor

# Pull latest code
cd ~/shadowhound
git pull origin feature/local-llm-support

# Install jtop (requires sudo, ~10 min)
sudo ./scripts/install_jtop_thor.sh

# Verify installation
systemctl status jtop.service  # Should be active (running)
sudo jtop  # Should show GPU metrics, not N/A
# Press 'q' to exit
```

**Success Check**: 
- ✅ jtop shows GPU memory (MiB/GiB)
- ✅ jtop shows GPU utilization %
- ✅ No "N/A" values

---

### Step 3: Verify Ollama Container Running
```bash
# On Thor
docker ps | grep ollama
# Should show: ghcr.io/nvidia-ai-iot/ollama:r38.2.arm64-sbsa-cu130-24.04

# If not running, start it
./scripts/setup_ollama_thor.sh
```

---

### Step 4: Test phi4:14b Baseline Performance
```bash
# On Thor
docker exec ollama ollama run --verbose phi4:14b "Count from 1 to 10, one number per line."

# Watch the output for speed metrics:
# eval rate: should be 15-20 tok/s ✅
# If <10 tok/s, something is wrong ❌
```

**Example Good Output**:
```
1
2
3
...
10

total duration: 2.5s
load duration: 800ms
prompt eval rate: 50 tok/s
eval rate: 18.5 tok/s  ← CHECK THIS!
```

**If slow (<10 tok/s)**: GPU degradation issue present, see troubleshooting below

---

### Step 5: Setup Monitoring Terminals

**On Thor, open 3 terminal windows:**

**Terminal 1 (jtop - GPU monitoring)**:
```bash
sudo jtop
```
Leave this open to watch GPU memory and utilization

**Terminal 2 (Docker stats - container monitoring)**:
```bash
watch -n 5 'docker stats ollama --no-stream'
```

**Terminal 3 (Ollama logs - if needed)**:
```bash
docker logs -f ollama
```

---

## 🤖 Robot Testing (2 hours)

### Pre-Flight Checklist
- [ ] Thor rebooted (clean GPU state)
- [ ] jtop installed and showing metrics
- [ ] phi4:14b baseline verified (15-20 tok/s)
- [ ] GO2 robot powered on
- [ ] ROS2 bridge running (go2_ros2_sdk on Thor)
- [ ] Monitoring terminals open

---

### Test Phase 1: Launch Mission Agent (10 min)

**On laptop in devcontainer**:
```bash
# Terminal 1: Launch mission agent
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.50.10:11434 \
    ollama_model:=phi4:14b \
    web_host:=0.0.0.0 \
    web_port:=8080
```

**Watch for**:
```
============================================================
🔍 VALIDATING LLM BACKEND CONNECTION
============================================================
Testing ollama backend...
  ✅ Ollama service responding
  ✅ Model 'phi4:14b' available
  ✅ Test prompt succeeded
============================================================
✅ Ollama backend validation PASSED
============================================================
```

**Check jtop on Thor**: GPU memory should jump to ~7-8GB when model loads

**Open browser**: http://localhost:8080
- Should show: Backend: OLLAMA, Model: phi4:14b

✅ **Success**: Mission agent running, backend validated, dashboard accessible

---

### Test Phase 2: Text-Only Tests (15 min)

**Test 1: Simple Response**
```
Mission: "Describe what you are in one sentence"
```
Expected: Fast response (<3s), coherent answer

---

**Test 2: JSON Generation**
```
Mission: "Generate JSON with these keys: action='test', status='ok', value=42"
```
Expected: Valid JSON output, <5s response

---

**Test 3: Navigation Plan**
```
Mission: "Create a navigation plan to move forward 2 meters then rotate 90 degrees left"
```
Expected JSON:
```json
{
  "steps": [
    {"action": "nav.goto", "params": {"x": 2.0, "y": 0.0}},
    {"action": "nav.rotate", "params": {"yaw": 1.57}}
  ]
}
```

✅ **Success Criteria**: 
- All 3 responses coherent
- Response times <5 seconds
- JSON valid and correct
- No errors in logs

---

### Test Phase 3: Robot Motion (30 min)

**⚠️ Safety First**:
- Clear area around robot (3m radius)
- Emergency stop ready
- Start with small movements

---

**Test 4: Simple Forward Motion**
```
Mission: "Move forward 0.5 meters"
```

**Watch for**:
1. LLM generates plan (JSON with nav.goto)
2. `/cmd_vel` topic starts publishing
3. Robot moves forward ~0.5m
4. Mission reports success

**On laptop Terminal 2**:
```bash
ros2 topic echo /cmd_vel
```

✅ **Success**: Robot moves approximately correct distance

---

**Test 5: Rotation**
```
Mission: "Rotate 45 degrees to the left"
```
Expected: Robot turns left ~45 degrees

---

**Test 6: Multi-Step Mission**
```
Mission: "Move forward 1 meter, rotate 90 degrees right, then move forward 1 meter"
```
Expected: 
- LLM generates 3-step plan
- Robot executes each step in sequence
- Final position ~1m forward, ~1m right of start

---

**Test 7: Error Handling**
```
Mission: "Move forward 100 meters"
```
(Assuming you don't have 100m of clear space)

Expected:
- LLM generates plan
- Navigation attempts execution
- Nav2 reports error (obstacle/timeout)
- System doesn't crash
- Clear error message to user

✅ **Success Criteria**:
- 4+ robot motions successful
- Movements approximately correct (±20% tolerance)
- System stable, no crashes
- Error handling works

---

### Test Phase 4: Performance Check (15 min)

**Run 5 missions consecutively**:
1. "Move forward 0.5m"
2. "Rotate left 45 degrees"
3. "Move forward 0.5m"
4. "Rotate right 45 degrees"  
5. "Return to starting position"

**Monitor on Thor (jtop)**:
- GPU memory: Should stay ~7-8GB (stable)
- GPU utilization: Spikes during LLM responses
- Temperature: <80°C (typical for Orin)

**Time each mission**: Response time should stay <5 seconds

✅ **Success Criteria**:
- All 5 missions complete successfully
- Response times consistent (<5s, low variance)
- GPU memory stable (no leaks)
- No performance degradation

**⚠️ If Degradation Occurs**:
- Note the pattern (after which mission?)
- Check jtop for memory/thermal issues
- This is the known GPU degradation issue
- Continue testing but document behavior

---

## 📊 Results Documentation (15 min)

### Update Deployment Checklist

Open `docs/OLLAMA_DEPLOYMENT_CHECKLIST.md` and fill in:

**Test Results Summary**:
```
Date Tested: 2025-10-11
Tested By: [Your Name]
System: Thor + GO2 Robot

Overall Results: [ ] PASS / [ ] PASS WITH ISSUES / [ ] FAIL

Performance Summary:
| Metric              | Target | Actual | Status |
|---------------------|--------|--------|--------|
| Mission Success     | >95%   | ____%  | ✅/❌  |
| Avg Response Time   | <5s    | ___s   | ✅/❌  |
| Memory Stability    | Stable | Stable/Leak | ✅/❌ |
| Robot Motion Accuracy | ±20% | ____%  | ✅/❌  |
```

**Issues Encountered**:
```
1. Issue: [Description]
   Severity: Critical / High / Medium / Low
   Workaround: [If any]
```

---

## 🎯 Decision Time

### If All Tests PASS ✅

**You're ready to merge!**

1. Update main README.md with Ollama setup instructions
2. Merge feature/local-llm-support → dev
3. Tag release: `v1.1.0-ollama-support`
4. Document production model: phi4:14b
5. 🎉 Celebrate!

```bash
cd /workspaces/shadowhound
git checkout dev
git merge feature/local-llm-support
git push
git tag -a v1.1.0 -m "Add Ollama local LLM support with phi4:14b"
git push --tags
```

---

### If Tests FAIL ❌

**Common issues and fixes**:

**Issue: Slow responses (>10s)**
- Check baseline: `docker exec ollama ollama run --verbose phi4:14b "test"`
- If baseline also slow: GPU degradation, reboot Thor
- If only during robot test: Check CPU/memory on Thor (`htop`)

**Issue: Invalid JSON**
- Try simpler prompt: "Generate JSON: {action: test}"
- May need to switch to qwen2.5-coder:32b (better JSON)
- Check LLM prompt templates in mission_agent.py

**Issue: Robot doesn't move**
- Check ROS2 topics: `ros2 topic list | grep cmd_vel`
- Check nav2 stack: `ros2 node list | grep nav`
- Check robot bridge: `ros2 topic echo /robot_state`

**Issue: Mission agent crashes**
- Check logs for specific error
- Verify backend validation passed
- Check Thor network connectivity
- Try restarting mission agent

---

### If GPU Degradation Occurs 🟡

**This is a known issue, document it**:

1. Note when it occurred (after how many missions?)
2. Measure the degradation (baseline speed vs current)
3. Check jtop for:
   - Memory fragmentation
   - Clock speed reduction
   - Thermal throttling
4. Document in `docs/OLLAMA_STATUS_AND_TODOS.md`
5. **Decision**: 
   - If degradation is gradual (>20 missions): ACCEPTABLE, merge with known issue
   - If degradation is immediate (<5 missions): INVESTIGATE before merge

---

## 🚨 Emergency Procedures

### Robot Behaving Unsafely
```bash
# STOP EVERYTHING
Ctrl+C in mission agent terminal

# Or emergency stop on robot (physical button)
```

### Ollama Container Crashes
```bash
# On Thor
docker stop ollama
docker rm ollama
./scripts/setup_ollama_thor.sh
```

### Mission Agent Won't Start
```bash
# Check backend manually
curl http://192.168.50.10:11434/api/tags

# If no response, check Thor:
ssh thor
docker ps | grep ollama
```

### Forgot to Reboot Thor?
If baseline performance is slow:
```bash
# Stop testing
# On Thor
sudo reboot
# Wait and restart from Step 1
```

---

## 📚 Reference Documents

- **Full deployment checklist**: `docs/OLLAMA_DEPLOYMENT_CHECKLIST.md`
- **Status and TODOs**: `docs/OLLAMA_STATUS_AND_TODOS.md`
- **Performance notes**: `docs/THOR_PERFORMANCE_NOTES.md`
- **Backend validation**: `docs/LLM_BACKEND_VALIDATION.md`
- **jtop security analysis**: `docs/SECURITY_ANALYSIS_JTOP.md`

---

## 🎯 Quick Success Checklist

### Before Testing
- [ ] Thor rebooted
- [ ] jtop installed and showing GPU metrics
- [ ] phi4:14b baseline verified (15-20 tok/s)
- [ ] GO2 powered on
- [ ] Monitoring terminals open

### During Testing
- [ ] Mission agent starts successfully
- [ ] Backend validation passes
- [ ] 3+ text-only tests pass
- [ ] 3+ robot motion tests pass
- [ ] Performance stable over 5+ missions

### After Testing
- [ ] Results documented in checklist
- [ ] Issues documented (if any)
- [ ] Decision made: merge or investigate
- [ ] Celebrate or plan next steps! 🚀

---

**Good luck! You've got this! 💪**

**Remember**: The goal is to validate phi4:14b works well enough for robot testing. Perfect performance not required - just stable, safe, and functional.

If you hit any blockers, see:
- `docs/OLLAMA_STATUS_AND_TODOS.md` - Known issues and TODOs
- `docs/THOR_PERFORMANCE_NOTES.md` - GPU degradation workarounds

---

**Last Updated**: 2025-10-10 23:30  
**Next Update**: After testing 2025-10-11
