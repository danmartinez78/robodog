# Ollama Deployment Test Checklist

**Purpose**: Validate Ollama local LLM backend works correctly with ShadowHound robot before merging to dev/main.

**Date Created**: 2025-10-10  
**Branch**: feature/local-llm-support  
**Target System**: Thor + GO2 Robot

---

## Pre-Deployment Validation

### ✅ Prerequisites (Updated 2025-10-10 EOD)

- [x] Ollama container running on Thor (Jetson-optimized)
- [x] phi4:14b model pulled (PRIMARY for testing, ~7.7GB)
- [x] qwen2.5-coder:32b model pulled (BACKUP, 98/100 quality, slower)
- [x] Models available and working (basic validation done)
- [x] Backend validation system implemented (two-layer)
- [x] Container memory stable
- [x] Benchmark infrastructure complete
- [ ] ⚠️ **jtop NOT YET INSTALLED** - Need GPU monitoring before robot test
- [ ] ⚠️ **GPU degradation issue unresolved** - Workaround: reboot Thor before testing
- [ ] ⚠️ **Benchmark incomplete** - GPU degradation invalidated results (~5 tok/s post-test)

**STATUS**: Ready for robot testing with phi4:14b after Thor reboot and jtop installation

---

## Test Plan: End-to-End Robot Integration

**⚠️ IMPORTANT PRE-TEST STEPS** (Added 2025-10-10):

1. **Reboot Thor for clean GPU state**
   ```bash
   # On Thor
   sudo reboot
   # Wait for system to come back up (~2 min)
   ```

2. **Install jtop for GPU monitoring**
   ```bash
   cd ~/shadowhound
   git pull origin feature/local-llm-support
   sudo ./scripts/install_jtop_thor.sh
   # Verify: systemctl status jtop.service
   # Verify: sudo jtop (should show GPU memory, not N/A)
   ```

3. **Verify phi4:14b baseline performance**
   ```bash
   docker exec ollama ollama run --verbose phi4:14b "Count to 10"
   # Expected: 15-20 tok/s (eval rate)
   # If much slower, investigate GPU degradation
   ```

4. **Setup monitoring terminals**
   - Terminal 1: `sudo jtop` (watch GPU memory/utilization)
   - Terminal 2: Mission agent logs
   - Terminal 3: `ros2 topic echo /cmd_vel` (robot commands)

### Phase 1: Mission Agent Startup (15 min)

#### Test 1.1: Launch Mission Agent with Ollama Backend

**On laptop** (in devcontainer):
```bash
# Launch mission agent with phi4:14b (PRIMARY TEST MODEL)
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.50.10:11434 \
    ollama_model:=phi4:14b \
    web_host:=0.0.0.0 \
    web_port:=8080

# Alternative: qwen2.5-coder:32b (if phi4 has issues)
# ollama_model:=qwen2.5-coder:32b
```

**Expected Output**:
```
[INFO] [mission_agent]: Starting Mission Agent with Ollama backend
[INFO] [mission_agent]: Ollama URL: http://192.168.50.10:11434
[INFO] [mission_agent]: Model: phi4:14b
============================================================
🔍 VALIDATING LLM BACKEND CONNECTION
============================================================
Testing ollama backend...
  URL: http://192.168.50.10:11434
  Model: phi4:14b
  Checking Ollama service...
  ✅ Ollama service responding
  ✅ Model 'phi4:14b' available
  Sending test prompt...
  ✅ Test prompt succeeded (response: 'OK')
============================================================
✅ Ollama backend validation PASSED
============================================================
[INFO] [mission_agent]: MissionExecutor ready!
[INFO] [mission_agent]: Web dashboard: http://0.0.0.0:8080
[INFO] [mission_agent]: Ready to accept missions
```

**New Feature**: The mission agent now **automatically validates** the LLM backend connection on startup. If validation fails, the node exits immediately with a clear error message. See `docs/LLM_BACKEND_VALIDATION.md` for details.

**⚠️ Watch jtop Terminal**: Should see GPU memory jump to ~7-8GB when phi4:14b loads on first request

**Verify**:
- [ ] Mission agent starts without errors
- [ ] **Backend validation PASSED** (automatic on startup)
- [ ] Ollama connection successful
- [ ] Model loads correctly (watch jtop: ~7-8GB GPU memory for phi4:14b)
- [ ] First response <5 seconds (phi4 is fast: 15-20 tok/s expected)
- [ ] No timeout errors
- [ ] jtop shows GPU memory stable (not increasing rapidly)

**Troubleshooting**:
- If **validation fails**: See error message for specific issue (service unreachable, model not found, etc.)
- If connection fails: Check `OLLAMA_BASE_URL` matches Thor IP
- If model not found: Verify model pulled on Thor (`docker exec ollama ollama list`)
- If timeout: Check Thor firewall allows port 11434
- **See detailed troubleshooting**: `docs/LLM_BACKEND_VALIDATION.md`

---

#### Test 1.2: Web Dashboard Access

**On laptop browser**: Navigate to `http://localhost:8080`

**Verify**:
- [ ] Dashboard loads successfully
- [ ] Backend indicator shows "OLLAMA" (not "OPENAI")
- [ ] Model name displays: "phi4:14b" (or qwen2.5-coder:32b if using backup)
- [ ] Connection status: GREEN
- [ ] No JavaScript console errors

**Screenshot**: Take screenshot of dashboard showing Ollama backend active

---

### Phase 2: Simple Mission Tests (30 min)

#### Test 2.1: Text Response (No Robot Motion)

**Mission**: "Describe what you are"

**Expected LLM Response**:
- Should identify as robot assistant
- Concise response (phi4 is direct, not overly verbose)
- Fast response (<3 seconds for phi4:14b at 15-20 tok/s expected)

**⚠️ Watch for GPU Degradation**: If response is slow (>10s), check jtop for issues

**Verify**:
- [ ] Response received within 10 seconds
- [ ] Response is coherent and relevant
- [ ] No errors in mission agent logs
- [ ] Dashboard shows response correctly

**Logs to Check**:
```bash
# In separate terminal
ros2 topic echo /shadowhound/mission/status
```

---

#### Test 2.2: Navigation Plan Generation (JSON Output)

**Mission**: "Create a navigation plan to explore 3 meters forward, then rotate right"

**Expected LLM Response** (JSON structure):
```json
{
  "steps": [
    {"action": "nav.goto", "params": {"x": 3.0, "y": 0.0}},
    {"action": "nav.rotate", "params": {"yaw": -1.57}}
  ]
}
```

**Verify**:
- [ ] JSON structure is valid
- [ ] Skills are correctly identified (nav.goto, nav.rotate)
- [ ] Parameters have reasonable values
- [ ] phi4:14b produces valid JSON (good structured output capability)

**Compare**: Try same mission with OpenAI backend later to validate quality difference

---

#### Test 2.3: Complex Reasoning

**Mission**: "If the robot is 0.6m wide and needs to pass through a 0.8m doorway with an obstacle 0.3m to the left, should it go right or left?"

**Expected Response**:
- Logical reasoning explaining choice
- Correct answer: "Go RIGHT" (0.4m clearance vs 0.1m)
- Clear explanation

**Verify**:
- [ ] Correct reasoning
- [ ] Not overly verbose (phi4 should be concise)
- [ ] Fast response (<5 seconds)

---

### Phase 3: Robot Hardware Integration (45 min)

**Prerequisites**:
- GO2 robot powered on
- ROS2 bridge active (go2_ros2_sdk running on Thor)
- Nav2 stack running
- Map loaded

#### Test 3.1: Simple Navigation Command

**Mission**: "Move forward 1 meter"

**Expected Behavior**:
1. LLM generates navigation plan: `{"steps": [{"action": "nav.goto", "params": {"x": 1.0, "y": 0.0}}]}`
2. Mission agent executes skill: `nav.goto`
3. Robot moves forward ~1 meter
4. Success reported back to dashboard

**Verify**:
- [ ] LLM generates correct JSON plan
- [ ] Skill execution starts (check `/cmd_vel` topic)
- [ ] Robot physically moves
- [ ] Distance approximately correct (±0.2m tolerance)
- [ ] Mission completes successfully
- [ ] Dashboard shows "SUCCESS" status
- [ ] **jtop shows stable GPU memory** (no sudden spikes or leaks)

**Logs to Check**:
```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor skill execution
ros2 topic echo /shadowhound/skill/status
```

---

#### Test 3.2: Multi-Step Mission

**Mission**: "Rotate 90 degrees left, move forward 2 meters, then rotate back to original heading"

**Expected Plan**:
```json
{
  "steps": [
    {"action": "nav.rotate", "params": {"yaw": 1.57}},
    {"action": "nav.goto", "params": {"x": 2.0, "y": 0.0}},
    {"action": "nav.rotate", "params": {"yaw": -1.57}}
  ]
}
```

**Verify**:
- [ ] LLM generates multi-step plan
- [ ] Each step executes in sequence
- [ ] Robot completes full mission
- [ ] Final heading approximately correct (±15° tolerance)
- [ ] No skill execution failures

---

#### Test 3.3: Perception Integration

**Mission**: "Take a photo"

**Expected Plan**:
```json
{
  "steps": [
    {"action": "perception.snapshot", "params": {}}
  ]
}
```

**Verify**:
- [ ] LLM identifies perception skill
- [ ] Camera image captured
- [ ] Image displayed in dashboard (if implemented)
- [ ] Mission reports success

---

#### Test 3.4: Error Handling

**Mission**: "Move to an unreachable location behind a wall"

**Expected Behavior**:
- LLM generates plan (may not know about wall)
- Navigation skill attempts execution
- Nav2 reports failure (obstacle/timeout)
- Mission agent reports error to LLM
- LLM suggests alternative or acknowledges failure

**Verify**:
- [ ] System doesn't crash on navigation failure
- [ ] Error properly reported to user
- [ ] LLM provides helpful error message
- [ ] System ready for next mission

---

### Phase 4: Performance Validation (30 min)

#### Test 4.1: Response Time Benchmarks

Execute 10 simple missions and measure:

**Metrics**:
- Time to first token (TTFT)
- Total completion time
- End-to-end mission time (LLM + skill execution)

**Target Performance** (based on preliminary observations):
- TTFT: <2 seconds (phi4 is fast)
- Simple text response: <3 seconds
- JSON navigation plan: <5 seconds
- Quality: Good reasoning and JSON generation

**⚠️ Known Issue**: GPU degradation may occur after multiple model unload/reload cycles. If speeds drop significantly, note in results and plan investigation.

**Record Results**:
```bash
# Use benchmark script
cd scripts
./benchmark_ollama_models.sh  # Re-run with production config

# Or manual timing
time echo "Mission: move forward 1m" | ros2 topic pub --once /shadowhound/mission ...
```

**Verify**:
- [ ] Performance meets benchmarked expectations
- [ ] No degradation from Thor system load
- [ ] Consistent response times (low variance)

---

#### Test 4.2: Memory Stability

Run 20+ missions consecutively and monitor:

**On Thor**:
```bash
# Monitor container memory (watch for GPU memory specifically with jtop)
watch -n 10 'docker stats ollama --no-stream'

# Also monitor GPU memory with jtop
sudo jtop  # Watch GPU section
```

**Verify**:
- [ ] GPU memory usage stable (~7-8GB for phi4:14b)
- [ ] No memory leaks (gradual increase in GPU VRAM)
- [ ] No OOM errors
- [ ] Model stays loaded between missions (jtop GPU memory stays elevated)
- [ ] **No performance degradation** (if speeds drop >20%, investigate)

**Expected Memory**:
- Initial: ~500MB (empty container)
- After first mission: ~7-8GB GPU VRAM (model loaded)
- After 20 missions: Still ~7-8GB GPU VRAM (stable)
- Container RAM: ~2-3GB (stable)

---

#### Test 4.3: Concurrent Operation

With robot operating:
- Launch Nav2 stack
- Launch mission agent with Ollama
- Execute navigation mission
- Monitor system resources on Thor

**On Thor**:
```bash
# Check CPU, memory, GPU
htop
docker stats
nvidia-smi
```

**Verify**:
- [ ] Thor CPU usage <80%
- [ ] Memory usage comfortable (<100GB of 128GB)
- [ ] No resource contention
- [ ] All systems responsive

---

### Phase 5: Backup Model Testing (15 min)

#### Test 5.1: Fallback to phi4:14b

**Reconfigure mission agent** to use backup model:
```bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_model:=phi4:14b
```

**Run same test missions**:
1. Simple text response
2. Navigation plan generation
3. Multi-step mission

**Verify**:
- [ ] phi4:14b works correctly
- [ ] Much faster responses (20.2 tok/s vs 4.4)
- [ ] Quality still acceptable (86.7/100 benchmark)
- [ ] Valid backup option if qwen2.5-coder has issues

---

### Phase 6: Comparison with OpenAI (15 min)

**Optional but Recommended**: Compare with cloud backend

#### Test 6.1: OpenAI Backend Baseline

**Launch with OpenAI**:
```bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=openai \
    openai_model:=gpt-4-turbo
```

**Run same test missions** and compare:

| Metric | qwen2.5-coder:32b | phi4:14b | gpt-4-turbo |
|--------|-------------------|----------|-------------|
| Response Time | ~5s | ~2s | ~3s |
| JSON Quality | | | |
| Reasoning Quality | | | |
| Cost per Mission | $0 | $0 | ~$0.03 |
| Privacy | Local | Local | Cloud |

**Verify**:
- [ ] Ollama quality competitive with OpenAI
- [ ] Local response times acceptable
- [ ] No degradation in mission success rate

---

## Post-Testing: Documentation

### Test Results Summary

**Date Tested**: ___________  
**Tested By**: ___________  
**System**: Thor + GO2 Robot  
**Branch**: feature/local-llm-support

#### Overall Results

- [ ] **PASS**: All critical tests passed
- [ ] **PASS WITH ISSUES**: Some non-critical failures (document below)
- [ ] **FAIL**: Critical issues blocking deployment

#### Performance Summary

| Metric | Target | Actual | Status |
|--------|--------|--------|--------|
| Mission Success Rate | >95% | ___% | |
| Avg Response Time | <5s | ___s | |
| Memory Stability | Stable | | |
| JSON Quality | >90/100 | | |

#### Issues Encountered

1. **Issue**: ___________
   - **Severity**: Critical / High / Medium / Low
   - **Workaround**: ___________
   - **Resolution**: ___________

2. *(Add more as needed)*

#### Recommendations

- [ ] **APPROVED FOR MERGE**: All tests passed, ready for dev branch
- [ ] **CONDITIONAL APPROVAL**: Minor issues, document and merge
- [ ] **NOT READY**: Critical issues must be fixed first

---

## Deployment Checklist (Pre-Merge)

### Code Quality

- [ ] All commits have descriptive messages
- [ ] No debug code or commented-out sections
- [ ] Configuration files updated with production values
- [ ] Documentation complete and accurate

### Testing

- [ ] All phases completed successfully
- [ ] Test results documented above
- [ ] Edge cases considered (failures, timeouts, etc.)
- [ ] Performance meets expectations

### Documentation

- [ ] README.md updated with Ollama instructions
- [ ] Configuration examples provided
- [ ] Troubleshooting guide complete
- [ ] Benchmark results documented

### Integration

- [ ] No breaking changes to existing code
- [ ] Backwards compatible (OpenAI backend still works)
- [ ] Launch files updated
- [ ] Dependencies documented

---

## Merge Process

### 1. Final Review

```bash
# On laptop, review all changes
cd /workspaces/shadowhound
git diff dev...feature/local-llm-support

# Check for unintended changes
git status
```

### 2. Update Documentation

- [ ] Update main README.md with Ollama setup
- [ ] Add deployment notes
- [ ] Update CHANGELOG.md (if exists)

### 3. Merge to dev

```bash
# Ensure feature branch is up to date
git checkout feature/local-llm-support
git pull

# Switch to dev and merge
git checkout dev
git pull
git merge feature/local-llm-support

# Resolve any conflicts
# Test one more time on dev branch
# Push to remote
git push
```

### 4. Verify dev Branch

- [ ] CI/CD passes (if configured)
- [ ] Quick smoke test on dev branch
- [ ] No unexpected issues

### 5. Merge to main (Production)

**Only after dev branch validated**:
```bash
git checkout main
git pull
git merge dev
git tag -a v1.1.0 -m "Add Ollama local LLM support with qwen2.5-coder:32b"
git push
git push --tags
```

---

## Rollback Plan

If issues discovered after merge:

### Quick Rollback
```bash
# Revert merge commit
git revert -m 1 <merge-commit-hash>
git push

# Or reset to before merge (if not pushed)
git reset --hard HEAD~1
```

### Fallback Configuration
```bash
# Temporarily switch back to OpenAI
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=openai
```

---

## Success Criteria

✅ **Minimum Requirements for Merge**:
1. Mission agent starts successfully with Ollama backend
2. At least 3 simple missions execute correctly
3. At least 1 hardware navigation mission succeeds
4. No memory leaks or stability issues
5. Performance meets benchmarked expectations
6. Backup model (phi4:14b) works as fallback
7. All documentation complete

---

## Notes & Observations

*(Use this section during testing to record observations, unexpected behavior, or insights)*

---

**Last Updated**: 2025-10-10  
**Next Review**: After robot testing complete
