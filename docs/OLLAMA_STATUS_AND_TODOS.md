# Ollama Local LLM Support - Current Status and TODOs

**Date**: 2025-10-10 (End of Day)  
**Branch**: feature/local-llm-support  
**Status**: Ready for Robot Testing  
**Next Test Date**: 2025-10-11

---

## 🎯 Current State Summary

### ✅ What's Working

1. **Ollama Container on Thor**
   - Jetson-optimized container running: `ghcr.io/nvidia-ai-iot/ollama:r38.2.arm64-sbsa-cu130-24.04`
   - HTTP API accessible at `http://192.168.50.10:11434`
   - Container properly configured with GPU access
   - Service stable and responding to requests

2. **Models Pulled and Available**
   - ✅ **phi4:14b** - Primary candidate (~7.7GB model size)
   - ✅ qwen2.5-coder:32b (~20GB)
   - ✅ qwq:32b
   - ✅ llama3.3:70b
   - ✅ deepseek-r1:7b
   - ✅ gpt-oss:20b (testing only)

3. **Backend Validation System**
   - Two-layer validation implemented:
     * Early check in `start.sh` (warns if backend unreachable)
     * Authoritative check in `mission_agent.py` (exits if validation fails)
   - Automatic validation on mission agent startup
   - Clear error messages for troubleshooting
   - See `docs/LLM_BACKEND_VALIDATION.md` for details

4. **Benchmark Infrastructure**
   - Quality scoring system implemented (`scripts/quality_scorer.py`)
   - Automated benchmark script (`scripts/benchmark_ollama_models.sh`)
   - GPU validation tests (baseline + final verification)
   - Auto-start container if not running
   - API-only operations (no docker restart issues)

5. **Documentation Complete**
   - Setup guides for Thor (`scripts/setup_ollama_thor.sh`)
   - Performance notes (`docs/THOR_PERFORMANCE_NOTES.md`)
   - Deployment checklist (`docs/OLLAMA_DEPLOYMENT_CHECKLIST.md`)
   - Backend validation guide (`docs/LLM_BACKEND_VALIDATION.md`)
   - Benchmark results template (`docs/OLLAMA_BENCHMARK_RESULTS.md`)

6. **GPU Monitoring Tools Ready**
   - jtop installation scripts created and committed:
     * `scripts/install_jtop_thor.sh` (main installer)
     * `scripts/patch_thor_jp7_in_repo.sh` (Thor GPU patches)
   - Security analysis completed (`docs/SECURITY_ANALYSIS_JTOP.md`)
   - NOT YET INSTALLED on Thor

---

## ⚠️ Known Issues

### 1. **GPU Performance Degradation (CRITICAL)**

**Symptom**: After running benchmark or unloading/reloading models, GPU performance drops 50-86%

**Evidence from Tonight's Testing**:
```
BASELINE (Fresh):  gpt-oss:20b at 37.00 tok/s  ✅
POST-BENCHMARK:    ~5 tok/s                     ❌ (86% slower!)

FINAL VERIFICATION: ~5 tok/s                    ❌ (FAILED)
```

**Impact**:
- Benchmark results unreliable after first model test
- Production use may suffer degradation over time
- Affects ALL models, not model-specific

**Current Workaround**:
- Reboot Thor before production use
- Avoid unnecessary model unload/reload cycles
- Keep models loaded with `keep_alive` parameter

**Root Cause**: UNKNOWN
- Not fixed by API-only operations
- Not fixed by avoiding container restart
- Possibly Jetson container issue
- Possibly CUDA context persistence issue
- Possibly GPU memory fragmentation

**Priority**: HIGH - Must investigate before production deployment

---

### 2. **Memory Monitoring Disabled**

**Issue**: `nvidia-smi` on Thor returns `N/A` for GPU memory metrics

**Current State**:
- Memory monitoring in benchmark script commented out (lines 165-195)
- Cannot track per-model VRAM usage accurately
- Cannot verify models fit in available memory

**Solution Available**: Install jtop with Thor patches
- Scripts ready: `install_jtop_thor.sh` + `patch_thor_jp7_in_repo.sh`
- Provides accurate GPU memory, utilization, power, temperature
- NOT YET INSTALLED

**Priority**: MEDIUM - Should install before robot testing

---

## 📊 Benchmark Results (Partial)

### Test Conditions
- **System**: Thor (Jetson AGX Orin 64GB)
- **Container**: Jetson-optimized Ollama
- **Date**: 2025-10-10 evening
- **Status**: **INCOMPLETE** - GPU degradation invalidated results

### Observed Performance (Before Degradation)

**phi4:14b** - Most promising candidate
- Speed: ~15-20 tok/s estimated (not formally measured)
- Quality: Strong for reasoning tasks
- Size: Reasonable memory footprint (~7.7GB)
- JSON: Good structured output capability
- **Recommendation**: Primary model for robot testing

**qwen2.5-coder:32b** - Previous benchmark winner
- Speed: ~4-5 tok/s (previous benchmark)
- Quality: 98/100 (JSON specialist)
- Size: Large memory footprint (~20GB)
- JSON: Excellent structured output
- **Status**: Backup option, may be too slow for real-time robot

### GPU Validation Results
```
✅ BASELINE TEST: 36.67 tok/s (gpt-oss:20b)
❌ FINAL TEST:    ~5 tok/s (FAILED - degraded)
```

**Conclusion**: Benchmark results unreliable, must retest after fixing GPU issue

---

## 📋 TODOs Before Robot Testing

### Priority 1: CRITICAL (Must Fix)

#### [ ] 1.1: Install jtop on Thor for GPU Monitoring
```bash
# On Thor
cd ~/shadowhound
git pull origin feature/local-llm-support
sudo ./scripts/install_jtop_thor.sh

# Verify installation
systemctl status jtop.service
sudo jtop  # Should show GPU memory, not N/A
```

**Why Critical**: Need accurate GPU monitoring during robot testing to:
- Verify models fit in VRAM
- Monitor for memory pressure
- Track GPU utilization during missions
- Debug performance issues

**Estimated Time**: 15 minutes  
**Blockers**: None, scripts ready

---

#### [ ] 1.2: Investigate GPU Performance Degradation

**Hypothesis Testing Plan**:

1. **Test CUDA Persistence Mode**
   ```bash
   # On Thor, before benchmark
   sudo nvidia-smi -pm 1  # Enable persistence mode
   
   # Run single model test twice
   # Compare speeds - should be consistent
   ```

2. **Test Standard Container vs Jetson Container**
   ```bash
   # Try standard ollama/ollama container
   docker run -d --gpus=all -p 11435:11434 ollama/ollama
   
   # Test same model multiple times
   # Check if degradation still occurs
   ```

3. **Test Model Keep-Alive Behavior**
   ```bash
   # Load model and keep in memory
   curl http://localhost:11434/api/generate -d '{
     "model": "phi4:14b",
     "prompt": "test",
     "keep_alive": -1  # Keep forever
   }'
   
   # Run multiple queries
   # Check if keeping model loaded prevents degradation
   ```

4. **Monitor with jtop During Benchmark**
   ```bash
   # Terminal 1: Run jtop
   sudo jtop
   
   # Terminal 2: Run benchmark
   ./scripts/benchmark_ollama_models.sh
   
   # Watch for:
   # - GPU memory fragmentation
   # - Clock speed changes
   # - Thermal throttling
   # - Power state changes
   ```

5. **Check NVIDIA/Ollama Known Issues**
   - Search Jetson forums for similar reports
   - Check Ollama GitHub issues for Jetson-specific bugs
   - Test with different Ollama versions

**Success Criteria**: Identify root cause and implement fix OR document reliable workaround

**Estimated Time**: 2-4 hours  
**Priority**: CRITICAL - Blocks production deployment

---

### Priority 2: HIGH (Should Do Before Robot Test)

#### [ ] 2.1: Reboot Thor and Verify Clean State
```bash
# On Thor
sudo reboot

# After reboot, verify GPU baseline
docker exec ollama ollama run --verbose gpt-oss:20b "Count to 10"
# Should see: ~37 tok/s (eval rate)
```

**Why**: Ensure clean starting state for robot testing

**Estimated Time**: 5 minutes (+ reboot time)

---

#### [ ] 2.2: Verify phi4:14b Performance in Clean State
```bash
# On Thor, after reboot
docker exec ollama ollama run --verbose phi4:14b "Explain what a quadruped robot is in one sentence."

# Expected:
# - Speed: 15-20 tok/s
# - Quality: Coherent, concise response
# - JSON capable: Test with structured output prompt
```

**Test JSON Output**:
```bash
docker exec ollama ollama run phi4:14b "Return JSON with keys: name (value: GO2), type (value: quadruped), speed (value: fast)"

# Expected output:
# {"name": "GO2", "type": "quadruped", "speed": "fast"}
```

**Success Criteria**: 
- Speed >15 tok/s in clean state
- Valid JSON generation
- Coherent reasoning

**Estimated Time**: 10 minutes

---

#### [ ] 2.3: Document phi4:14b as Primary Test Model
Update `docs/OLLAMA_BENCHMARK_RESULTS.md` with decision rationale:
- Speed: Good balance (15-20 tok/s)
- Size: Reasonable VRAM (~7.7GB)
- Quality: Strong reasoning capability
- JSON: Capable of structured output
- Availability: Already pulled on Thor

**Estimated Time**: 10 minutes

---

### Priority 3: MEDIUM (Good to Have)

#### [ ] 3.1: Update Deployment Checklist with Current Status
Mark Phase 1 and Phase 2 tests as ready:
- Mission agent backend validation ✅ (implemented)
- Ollama connection working ✅
- Models available ✅
- GPU monitoring tools ready ⏳ (need to install)

**Estimated Time**: 5 minutes

---

#### [ ] 3.2: Create Quick Start Guide for Tomorrow
Write `docs/ROBOT_TEST_QUICKSTART.md` with:
1. Pre-flight checks (reboot Thor, verify GPU)
2. Launch commands (mission agent with phi4:14b)
3. Test missions (3-5 simple commands)
4. What to look for (speed, quality, errors)
5. Troubleshooting steps

**Estimated Time**: 15 minutes

---

#### [ ] 3.3: Prepare Test Mission Scripts
Create `scripts/test_missions.txt` with pre-written missions:
```
# Simple missions for robot testing
1. "Move forward 1 meter"
2. "Rotate 90 degrees to the left"
3. "Move to coordinates x=2.0, y=1.0"
4. "Take a photo"
5. "Return to starting position"
```

**Estimated Time**: 10 minutes

---

## 🧪 Robot Testing Plan (2025-10-11)

### Test Setup

1. **Hardware Prerequisites**
   - [ ] GO2 robot powered on and responsive
   - [ ] Thor rebooted (clean GPU state)
   - [ ] ROS2 bridge running (go2_ros2_sdk)
   - [ ] Network connectivity verified

2. **Software Prerequisites**
   - [ ] jtop installed and working
   - [ ] phi4:14b verified in clean state
   - [ ] Mission agent tested with backend validation
   - [ ] Dashboard accessible

3. **Monitoring Setup**
   - [ ] Terminal 1: `sudo jtop` (GPU monitoring)
   - [ ] Terminal 2: Mission agent logs
   - [ ] Terminal 3: ROS2 topic monitoring (`/cmd_vel`, `/odom`)
   - [ ] Browser: Web dashboard (http://localhost:8080)

### Test Sequence

#### Phase 1: Startup Validation (10 min)
1. Launch mission agent with phi4:14b
2. Verify backend validation passes
3. Verify first response time (<5 seconds)
4. Check jtop shows model loaded (~7-8GB GPU memory)

#### Phase 2: Text-Only Tests (15 min)
1. "Describe what you are"
2. "Create a navigation plan to move forward 2 meters"
3. "Generate JSON: {action: nav.goto, x: 1.0, y: 0.0}"

**Success Criteria**:
- All responses coherent and fast (<5s)
- JSON output valid
- No errors in logs

#### Phase 3: Robot Motion Tests (30 min)
1. "Move forward 0.5 meters" (conservative first test)
2. "Rotate 45 degrees to the left"
3. "Move forward 1 meter then stop"
4. "Navigate to x=2.0, y=1.0"

**Success Criteria**:
- Robot executes commands
- Movements approximately correct (±20% tolerance)
- No crashes or safety issues
- Mission agent reports success

#### Phase 4: Performance Validation (15 min)
1. Monitor GPU memory during 5 consecutive missions
2. Check for memory leaks or degradation
3. Time 5 missions, average response time
4. Verify speeds stay consistent (>15 tok/s)

**Success Criteria**:
- GPU memory stable (<10GB)
- No performance degradation
- Response times consistent (<5s variance)

### Abort Conditions
- Robot behaves unsafely (stop immediately)
- GPU performance degrades (note pattern, may need to investigate)
- Mission agent crashes repeatedly
- Response times >30 seconds

---

## 📝 Decision Log

### Model Selection: phi4:14b (Primary)

**Date**: 2025-10-10  
**Rationale**:
- Best speed/quality tradeoff observed
- Reasonable memory footprint
- Good reasoning capability
- JSON generation capable
- Already available on Thor

**Alternatives**:
- qwen2.5-coder:32b (backup if more accuracy needed, slower)
- OpenAI fallback (always available)

### Benchmark Status: INCOMPLETE

**Date**: 2025-10-10  
**Reason**: GPU performance degradation invalidated results  
**Action**: Must retest after fixing degradation issue  
**Impact**: Using phi4:14b based on preliminary observations, not formal benchmark

### jtop Installation: DEFERRED

**Date**: 2025-10-10  
**Reason**: End of day, scripts complete but not yet run  
**Action**: Install before robot testing tomorrow  
**Impact**: None (scripts ready, just need to execute)

---

## 🚀 Tomorrow's Workflow (2025-10-11)

### Morning Setup (30 min)
1. ☕ Coffee first
2. Reboot Thor → wait for clean state
3. Install jtop → verify GPU monitoring works
4. Verify phi4:14b performance in clean state
5. Review test plan

### Robot Testing (2 hours)
1. Launch mission agent with phi4:14b
2. Run Phase 1: Startup validation
3. Run Phase 2: Text-only tests
4. Run Phase 3: Robot motion tests
5. Run Phase 4: Performance validation
6. Document results in deployment checklist

### Afternoon Analysis (1 hour)
1. Review test results
2. Investigate any issues found
3. Document decision: PASS → merge OR FAIL → fix
4. Update documentation with findings

### If PASS: Merge to dev (30 min)
1. Final documentation updates
2. Merge feature/local-llm-support → dev
3. Tag release
4. Celebrate! 🎉

### If FAIL: Debug (variable)
1. Investigate GPU degradation issue
2. Test alternative models/configurations
3. Document blockers
4. Plan next steps

---

## 📞 Need Help? Reference These Docs

- **Backend validation issues**: `docs/LLM_BACKEND_VALIDATION.md`
- **GPU performance problems**: `docs/THOR_PERFORMANCE_NOTES.md`
- **Benchmark procedures**: `docs/OLLAMA_BENCHMARK_RESULTS.md`
- **Deployment steps**: `docs/OLLAMA_DEPLOYMENT_CHECKLIST.md`
- **jtop installation**: `scripts/install_jtop_thor.sh` (run with `sudo`)
- **Security concerns**: `docs/SECURITY_ANALYSIS_JTOP.md`

---

## 🎯 Success Definition

**Minimum criteria for merge**:
- ✅ Mission agent starts with Ollama backend
- ✅ phi4:14b generates valid responses
- ✅ Robot executes 3+ navigation commands successfully
- ✅ Performance stable over 10+ missions
- ✅ GPU monitoring working (jtop installed)
- ✅ No critical bugs or safety issues
- ⚠️ GPU degradation issue documented (workaround: reboot)

**Nice to have**:
- Root cause of GPU degradation identified and fixed
- Benchmark completed with reliable results
- Multiple models tested and compared

---

## 💭 Final Notes from 2025-10-10 Testing

**What went well**:
- Ollama infrastructure solid and stable
- Backend validation system working perfectly
- phi4:14b shows promise (fast, capable)
- Documentation comprehensive and helpful
- jtop scripts ready for deployment

**What needs work**:
- GPU degradation mystery needs solving
- Need accurate memory monitoring (install jtop)
- Benchmark incomplete due to degradation
- Production deployment blocked until degradation understood

**Confidence level**: 
- 🟢 **HIGH** for phi4:14b basic functionality
- 🟡 **MEDIUM** for production deployment (need to solve degradation)
- 🟢 **HIGH** for tools and infrastructure

**Overall assessment**: Ready for robot testing tomorrow with phi4:14b. If testing goes well, can merge with known issue documented (GPU degradation workaround = reboot). If degradation occurs during robot operation, must investigate further before production use.

---

**Last Updated**: 2025-10-10 23:00  
**Next Update**: After robot testing 2025-10-11  
**Status**: READY FOR TOMORROW'S TEST 🚀
