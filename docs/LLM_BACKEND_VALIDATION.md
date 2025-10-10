# LLM Backend Validation

## Overview

The ShadowHound mission agent now includes **startup validation** for LLM backends. This catches configuration errors immediately when the node starts, rather than failing silently when the first mission is attempted.

## Why This Matters

**Before validation:**
- Mission agent appears to start successfully
- First mission command fails with cryptic errors
- User must debug to find root cause (wrong URL, model not loaded, service down, etc.)
- Poor debugging experience

**After validation:**
- Mission agent validates backend connection on startup
- Clear error messages if backend is unreachable
- Fails fast with actionable diagnostics
- Excellent debugging experience

## What Gets Validated

### Ollama Backend (`agent_backend=ollama`)

1. ✅ **Service reachability** - Ollama API responding at configured URL
2. ✅ **Model availability** - Configured model is pulled and ready
3. ✅ **Test prompt** - Send simple prompt, verify response received

Example validation output:
```
============================================================
🔍 VALIDATING LLM BACKEND CONNECTION
============================================================
Testing ollama backend...
  URL: http://192.168.50.10:11434
  Model: qwen2.5-coder:32b
  Checking Ollama service...
  ✅ Ollama service responding
  ✅ Model 'qwen2.5-coder:32b' available
  Sending test prompt...
  ✅ Test prompt succeeded (response: 'OK')
============================================================
✅ Ollama backend validation PASSED
============================================================
```

### OpenAI Backend (`agent_backend=openai`)

1. ✅ **API key present** - `OPENAI_API_KEY` environment variable is set
2. ✅ **Test prompt** - Send simple prompt to OpenAI, verify response

Example validation output:
```
============================================================
🔍 VALIDATING LLM BACKEND CONNECTION
============================================================
Testing openai backend...
  ✅ OPENAI_API_KEY found
  Base URL: https://api.openai.com/v1
  Model: gpt-4-turbo
  Sending test prompt...
  ✅ Test prompt succeeded (response: 'OK')
============================================================
✅ OpenAI backend validation PASSED
============================================================
```

## Common Errors Caught

### Ollama Service Not Running

```
❌ Cannot connect to Ollama at http://192.168.50.10:11434
   Error: [Errno 111] Connection refused
   Check that Ollama is running and URL is correct
```

**Fix:**
```bash
# On Thor or gaming PC
systemctl status ollama  # Check if running
docker ps | grep ollama  # If containerized
```

### Model Not Pulled

```
❌ Model 'qwen2.5-coder:32b' not found in Ollama
   Available models: llama3.1:70b, phi4:14b, qwen2.5:14b
   Pull the model with: ollama pull qwen2.5-coder:32b
```

**Fix:**
```bash
# On Thor or gaming PC
ollama pull qwen2.5-coder:32b
```

### Wrong URL / Network Issues

```
❌ Timeout connecting to Ollama at http://192.168.50.10:11434
   Check network connectivity and Ollama status
```

**Fix:**
```bash
# Check Thor is reachable
ping 192.168.50.10

# Check correct port
curl http://192.168.50.10:11434/api/tags

# Verify parameter in launch file
grep ollama_base_url launch/shadowhound_bringup.launch.py
```

### OpenAI API Key Missing

```
❌ OPENAI_API_KEY environment variable not set
   Set it with: export OPENAI_API_KEY='sk-...'
```

**Fix:**
```bash
export OPENAI_API_KEY='sk-...'
# Or add to ~/.bashrc for persistence
```

### Slow Model Response

```
❌ Timeout waiting for test prompt response (>30s)
   Model 'qwen2.5-coder:32b' may be too slow or not loaded
```

**Fix:**
- Model is cold-loading (first request after pull)
- Wait for model to fully load, then restart mission agent
- Consider using faster model (e.g., phi4:14b for testing)

## Testing Validation Manually

Use the standalone test script:

```bash
cd /workspaces/shadowhound

# Test Ollama validation (adjust THOR_IP if needed)
export THOR_IP=192.168.50.10
python3 test_backend_validation.py

# Test OpenAI validation (requires API key)
export OPENAI_API_KEY='sk-...'
python3 test_backend_validation.py
```

Expected output:
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

============================================================
📊 VALIDATION TEST SUMMARY
============================================================
Ollama: ✅ PASS
OpenAI: ⏭️  SKIPPED
============================================================
```

## Integration with Mission Agent

The validation is integrated into the mission agent startup sequence:

```python
# In mission_agent.py
self.mission_executor.initialize()

# Validate LLM backend connection on startup
if not self._validate_llm_backend():
    raise RuntimeError(
        "LLM backend validation failed. Check logs above for details. "
        "Ensure the backend service is running and accessible."
    )

self.get_logger().info("MissionExecutor ready!")
```

If validation fails, the node exits immediately with a clear error message:

```
[ERROR] [shadowhound_mission_agent]: LLM backend validation failed. Check logs above for details. Ensure the backend service is running and accessible.
```

## Disabling Validation (Not Recommended)

If you need to bypass validation for testing (e.g., no network access), you can comment out the validation check in `mission_agent.py`:

```python
# Validate LLM backend connection on startup
# if not self._validate_llm_backend():
#     raise RuntimeError(...)
```

**⚠️ Warning:** This is NOT recommended for production deployments. The validation catches real issues that will cause mission failures.

## Performance Impact

- **Ollama validation:** ~2-5 seconds (service check + model check + test prompt)
- **OpenAI validation:** ~1-3 seconds (test prompt via internet)

The validation adds minimal startup time but provides significant value in catching misconfigurations early.

## Future Enhancements

Potential improvements:

1. **VLM validation** - Add validation for vision-language models when implemented
2. **Model warm-up** - Pre-load model during validation to reduce first mission latency
3. **Health check endpoint** - Expose validation status via ROS service or web API
4. **Retry logic** - Automatic retry with exponential backoff for transient failures
5. **Fallback backend** - Auto-switch to OpenAI if Ollama validation fails

## Related Documentation

- **Ollama Deployment:** `docs/OLLAMA_DEPLOYMENT_CHECKLIST.md`
- **Benchmark System:** `docs/OLLAMA_BENCHMARK_MEMORY_MANAGEMENT.md`
- **Architecture:** `docs/project_context.md`
- **Test Script:** `test_backend_validation.py`

## Troubleshooting Tips

### "Connection refused" error

The most common issue. Check:
1. Is Ollama running? `systemctl status ollama` or `docker ps`
2. Is the URL correct? Check launch file parameters
3. Is Thor reachable? `ping 192.168.50.10`
4. Is the port correct? Ollama uses 11434 by default

### "Model not found" error

Check available models:
```bash
curl http://192.168.50.10:11434/api/tags | jq '.models[].name'
```

Pull the missing model:
```bash
ollama pull qwen2.5-coder:32b
```

### Validation passes but missions still fail

This suggests an issue with DIMOS integration or robot communication, not the LLM backend. Check:
1. Robot connectivity (CycloneDDS or WebRTC)
2. ROS topic diagnostics in startup logs
3. DIMOS skill library initialization

---

**Last Updated:** 2025-01-XX  
**Status:** ✅ Implemented and tested
