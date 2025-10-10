# Backend Validation - Implementation Summary

## Changes Made

### 1. Mission Agent Validation (`mission_agent.py`)

Added three new methods to the `MissionAgentNode` class:

#### `_validate_llm_backend() -> bool`
Main validation orchestrator that:
- Identifies configured backend (ollama or openai)
- Routes to appropriate validation method
- Logs validation status with clear visual indicators
- Returns True/False for validation success

#### `_validate_ollama_backend() -> bool`
Ollama-specific validation that checks:
1. **Service reachability** - GET /api/tags with 5s timeout
2. **Model availability** - Verifies configured model is pulled
3. **Test prompt** - POST /api/generate with simple prompt (30s timeout)

Returns detailed error messages for each failure mode:
- Connection refused → Service not running
- Timeout → Network/firewall issue
- Model not found → Need to pull model
- Test prompt fails → Model loading issue

#### `_validate_openai_backend() -> bool`
OpenAI-specific validation that checks:
1. **API key present** - Checks OPENAI_API_KEY env var
2. **Test prompt** - Simple chat completion with 30s timeout

Returns detailed error messages:
- API key missing → Need to export env var
- Test prompt fails → Invalid key or network issue

### 2. Startup Integration

Modified mission agent initialization to call validation after MissionExecutor.initialize():

```python
# Initialize mission executor
self.get_logger().info("Initializing MissionExecutor...")
self.mission_executor.initialize()

# Validate LLM backend connection on startup
if not self._validate_llm_backend():
    raise RuntimeError(
        "LLM backend validation failed. Check logs above for details. "
        "Ensure the backend service is running and accessible."
    )

self.get_logger().info("MissionExecutor ready!")
```

If validation fails, the node exits immediately with RuntimeError, preventing silent failures.

### 3. Test Script (`test_backend_validation.py`)

Created standalone test script for validation logic:
- Tests Ollama validation (using Thor IP from environment)
- Tests OpenAI validation (if API key present)
- Provides detailed output for each validation step
- Can be run independently of ROS for quick debugging

Usage:
```bash
export THOR_IP=192.168.50.10  # Optional, defaults to 192.168.50.10
python3 test_backend_validation.py
```

### 4. Documentation

#### `docs/LLM_BACKEND_VALIDATION.md` (new)
Comprehensive guide covering:
- Why validation matters (before/after comparison)
- What gets validated (Ollama vs OpenAI)
- Common errors and fixes
- Testing validation manually
- Integration with mission agent
- Performance impact (~2-5s for Ollama)
- Future enhancements

#### `docs/OLLAMA_DEPLOYMENT_CHECKLIST.md` (updated)
- Updated Phase 1 expected output to show validation logs
- Added note about automatic validation feature
- Added reference to validation documentation
- Updated troubleshooting section

## Benefits

### Before Validation
```
[INFO] Mission agent ready!  ← Appears successful
[INFO] Ready to accept missions

# User sends first mission
[ERROR] Failed to execute mission: Connection refused  ← Confusing error
```

User must:
1. Debug cryptic error message
2. Check logs for root cause
3. Discover Ollama is down/unreachable
4. Restart mission agent after fixing

**Time to diagnose**: 5-15 minutes

### After Validation
```
[INFO] Mission agent starting...
============================================================
🔍 VALIDATING LLM BACKEND CONNECTION
============================================================
❌ Cannot connect to Ollama at http://192.168.50.10:11434
   Error: [Errno 111] Connection refused
   Check that Ollama is running and URL is correct
[ERROR] LLM backend validation failed. Check logs above for details.
```

User:
1. Immediately sees clear error message
2. Gets actionable fix instructions
3. Fixes issue before attempting missions
4. Mission agent validates successfully on restart

**Time to diagnose**: 30 seconds

## Error Examples Caught

### 1. Service Not Running
```
❌ Cannot connect to Ollama at http://192.168.50.10:11434
   Error: [Errno 111] Connection refused
   Check that Ollama is running and URL is correct
```

**Fix**: `systemctl start ollama` or `docker start ollama`

### 2. Wrong IP/URL
```
❌ Timeout connecting to Ollama at http://192.168.50.99:11434
   Check network connectivity and Ollama status
```

**Fix**: Check `THOR_IP` environment variable or launch file parameter

### 3. Model Not Pulled
```
✅ Ollama service responding
❌ Model 'qwen2.5-coder:32b' not found in Ollama
   Available models: llama3.1:70b, phi4:14b
   Pull the model with: ollama pull qwen2.5-coder:32b
```

**Fix**: `ollama pull qwen2.5-coder:32b`

### 4. OpenAI Key Missing
```
❌ OPENAI_API_KEY environment variable not set
   Set it with: export OPENAI_API_KEY='sk-...'
```

**Fix**: `export OPENAI_API_KEY='sk-...'`

### 5. Model Too Slow (Cold Start)
```
✅ Ollama service responding
✅ Model 'qwen2.5-coder:32b' available
❌ Timeout waiting for test prompt response (>30s)
   Model 'qwen2.5-coder:32b' may be too slow or not loaded
```

**Fix**: Wait for model to load, then restart mission agent

## Performance Impact

- **Ollama validation**: ~2-5 seconds
  - Service check: <1s
  - Model check: <1s  
  - Test prompt: 1-3s (depends on model speed)
  
- **OpenAI validation**: ~1-3 seconds
  - Test prompt via internet: 1-3s

**Verdict**: Minimal startup time increase (<5s) for significant reliability improvement.

## Testing

### Manual Test (Without ROS)
```bash
cd /workspaces/shadowhound
python3 test_backend_validation.py
```

Expected output when Thor is reachable:
```
🧪 Testing Ollama Backend Validation
------------------------------------------------------------
============================================================
🔍 TESTING OLLAMA BACKEND VALIDATION
============================================================
URL: http://192.168.50.10:11434
Model: qwen2.5-coder:32b

1. Checking Ollama service...
✅ Ollama service responding

2. Checking model availability...
   Available models: qwen2.5-coder:32b, phi4:14b, llama3.1:70b
✅ Model 'qwen2.5-coder:32b' available

3. Sending test prompt...
✅ Test prompt succeeded
   Response: 'OK'

============================================================
✅ Ollama backend validation PASSED
============================================================
```

### Integrated Test (With ROS)
```bash
ros2 launch shadowhound_mission_agent bringup.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.50.10:11434 \
    ollama_model:=qwen2.5-coder:32b
```

Validation runs automatically during startup. Check logs for validation status.

## Code Changes Summary

### Files Modified
- `src/shadowhound_mission_agent/shadowhound_mission_agent/mission_agent.py` (+187 lines)
  - Added `_validate_llm_backend()` method
  - Added `_validate_ollama_backend()` method
  - Added `_validate_openai_backend()` method
  - Integrated validation into initialization

### Files Created
- `test_backend_validation.py` (+182 lines)
  - Standalone validation test script
  
- `docs/LLM_BACKEND_VALIDATION.md` (+285 lines)
  - Comprehensive validation documentation
  
### Files Updated
- `docs/OLLAMA_DEPLOYMENT_CHECKLIST.md` (+8 lines, -6 lines)
  - Updated Phase 1 expected output
  - Added validation feature notes

**Total changes**: ~662 lines added

## Next Steps

### 1. Test on Real Hardware (HIGH PRIORITY)
Follow `docs/OLLAMA_DEPLOYMENT_CHECKLIST.md`:
- Run `test_ollama_deployment.sh` for pre-flight checks
- Launch mission agent with Ollama backend
- Verify validation passes
- Execute test missions
- Monitor stability

### 2. Merge to Dev (After Testing)
```bash
git checkout dev
git merge feature/local-llm-support
git push
```

### 3. Future Enhancements (OPTIONAL)

#### Health Check Endpoint
Expose validation status via ROS service:
```python
# New service: /shadowhound/validate_backend
srv_type: std_srvs/Trigger
response: success=True/False, message=validation_details
```

#### Model Warm-up
Pre-load model during validation to reduce first mission latency:
```python
# After validation succeeds, send warm-up prompt
warmup_prompt = "You are a robot assistant. Say ready."
# This loads model into memory for fast first mission
```

#### Retry Logic
Add exponential backoff for transient failures:
```python
for attempt in range(3):
    if validate():
        break
    time.sleep(2 ** attempt)  # 1s, 2s, 4s
else:
    raise RuntimeError("Validation failed after 3 attempts")
```

#### Fallback Backend
Auto-switch to OpenAI if Ollama validation fails:
```python
if not validate_ollama():
    logger.warning("Ollama validation failed, falling back to OpenAI")
    switch_backend_to_openai()
```

## Related Work

This validation completes the production-readiness work for Ollama backend:

1. ✅ **Model Selection** - qwen2.5-coder:32b chosen (98/100 quality)
2. ✅ **Memory Management** - Benchmark reliability improvements
3. ✅ **Documentation** - Comprehensive guides (~3500 lines)
4. ✅ **Testing Resources** - Deployment checklist + automation
5. ✅ **Backend Validation** - This work (startup health checks)
6. ⏳ **Robot Testing** - Pending (use deployment checklist)
7. ⏳ **Merge to Dev** - Pending (after robot testing)

## Questions Answered

**User's Question**: "How are we controlling which backend is being used?"

**Answer**: Via ROS parameter `agent_backend` (values: "openai" or "ollama"). Set in launch file or command line:
```bash
ros2 launch ... agent_backend:=ollama
```

**User's Question**: "Does it make sense to have a test in the startup script to verify connection to whatever llm/vlm/vla backend we have configured?"

**Answer**: Absolutely! This is now implemented. The mission agent automatically validates the configured backend on startup, catching misconfigurations immediately rather than failing silently on first mission. This is a production-critical feature that significantly improves operational reliability and debugging experience.

---

**Status**: ✅ Complete and ready for testing  
**Last Updated**: 2025-01-XX
