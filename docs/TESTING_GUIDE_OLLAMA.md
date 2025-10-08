# Testing Guide: Ollama Backend on Laptop

**Branch**: `feature/local-llm-support`  
**Date**: 2025-10-08  
**Status**: Ready for testing

## Pre-Flight Checklist

### On Laptop (daniel@9510)
- [ ] Workspace pulled latest changes
- [ ] On `feature/local-llm-support` branch
- [ ] Workspace built (`cb` or colcon build)
- [ ] Workspace sourced (`source-ws`)

### On Gaming PC
- [ ] Ollama installed and running
- [ ] Model downloaded (llama3.1:70b recommended)
- [ ] Network configured (OLLAMA_HOST=0.0.0.0:11434)
- [ ] Firewall allows port 11434
- [ ] Accessible from laptop (curl test passes)

## Pull Latest Changes on Laptop

```bash
# On laptop devcontainer
cd /workspaces/shadowhound
git fetch origin
git checkout feature/local-llm-support
git pull origin feature/local-llm-support

# Rebuild workspace
cb
source-ws
```

## Setup Gaming PC Ollama (If Not Already Done)

```bash
# SSH to gaming PC
ssh gaming-pc

# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull model (choose based on RAM)
ollama pull llama3.1:70b  # 48GB+ RAM
# OR
ollama pull llama3.1:13b  # 16GB+ RAM

# Configure network access
sudo systemctl edit ollama
# Add this line:
[Service]
Environment="OLLAMA_HOST=0.0.0.0:11434"

# Restart service
sudo systemctl restart ollama

# Configure firewall
sudo ufw allow 11434/tcp

# Verify it's running
curl http://localhost:11434/api/tags
```

## Test Connectivity from Laptop

```bash
# On laptop, test gaming PC Ollama
# Replace 192.168.1.100 with your gaming PC IP
curl http://192.168.1.100:11434/api/tags

# Should return JSON with installed models
# Example:
# {"models":[{"name":"llama3.1:70b",...}]}
```

## Test 1: Launch with Ollama Backend

```bash
# On laptop devcontainer
# Replace 192.168.1.100 with your gaming PC IP

ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.1.100:11434 \
    ollama_model:=llama3.1:70b
```

**Expected Output:**
```
[INFO] [mission_agent]: Configuration:
[INFO] [mission_agent]:   Agent backend: ollama
[INFO] [mission_agent]:   Ollama URL: http://192.168.1.100:11434
[INFO] [mission_agent]:   Ollama model: llama3.1:70b
[INFO] [mission_agent]: MissionExecutor ready!
[INFO] [mission_agent]: Using Ollama backend at http://192.168.1.100:11434
[INFO] [mission_agent]: Ollama model: llama3.1:70b
[INFO] [mission_agent]: Web interface started at http://localhost:8080
```

## Test 2: Verify Web UI Shows Ollama Backend

1. Open browser: http://localhost:8080
2. Check diagnostics panel (right side, below performance)
3. Verify:
   - **LLM BACKEND**: Shows "OLLAMA" in **green**
   - **MODEL**: Shows "LLAMA3.1:70B"

## Test 3: Simple Command Performance Test

### With Ollama (Expected: <2s)

```bash
# In web UI or terminal:
ros2 topic pub -1 /shadowhound/mission std_msgs/msg/String "data: 'stand up'"

# Watch logs for timing:
# [INFO] [mission_agent]: Mission complete in X.Xs (agent: X.Xs, overhead: X.Xs)
```

**Expected Results:**
- Total duration: **0.5-2 seconds** ⚡
- Agent duration: **0.5-1.5 seconds**
- Overhead: <0.5 seconds
- Web UI backend indicator: **OLLAMA (green)**

### With OpenAI (Expected: 10-15s)

Stop the agent and restart with OpenAI:

```bash
# Ctrl+C to stop

# Set API key
export OPENAI_API_KEY="sk-..."

# Launch with OpenAI backend
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=openai \
    openai_model:=gpt-4-turbo
```

Send same command:
```bash
ros2 topic pub -1 /shadowhound/mission std_msgs/msg/String "data: 'stand up'"
```

**Expected Results:**
- Total duration: **10-15 seconds** 🐌
- Agent duration: **10-14 seconds**
- Overhead: <1 second
- Web UI backend indicator: **OPENAI (orange)**

## Test 4: Multi-Step Command

```bash
# Complex mission
ros2 topic pub -1 /shadowhound/mission std_msgs/msg/String \
    "data: 'rotate to the right and take a step back'"
```

**Expected with Ollama:**
- **1-3 seconds** total

**Expected with OpenAI:**
- **20-30 seconds** total

## Test 5: Web UI Camera Feed

1. Verify camera feed shows in web UI (left panel)
2. Should update at ~1 Hz
3. Backend indicator still shows correct backend

## Test 6: Config File Test

Edit config file:
```bash
nano configs/laptop_dev_ollama.yaml

# Update ollama_base_url with your gaming PC IP:
ollama_base_url: "http://192.168.1.100:11434"
```

Launch with config:
```bash
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/laptop_dev_ollama.yaml
```

Verify all settings loaded correctly in logs.

## Performance Targets

| Metric | Ollama (Gaming PC) | OpenAI Cloud | Status |
|--------|-------------------|--------------|--------|
| Simple command | 0.5-2s | 10-15s | Pass if <3s |
| Multi-step | 1-3s | 20-30s | Pass if <5s |
| Backend indicator | Green "OLLAMA" | Orange "OPENAI" | Visual check |

## Troubleshooting

### "Connection refused" error

**Problem**: Can't connect to Ollama
```
Error: Connection refused to http://192.168.1.100:11434
```

**Solutions**:
1. Check Ollama is running: `ssh gaming-pc "systemctl status ollama"`
2. Test connectivity: `curl http://192.168.1.100:11434/api/tags`
3. Check firewall: `ssh gaming-pc "sudo ufw status | grep 11434"`
4. Verify OLLAMA_HOST: `ssh gaming-pc "systemctl show ollama | grep Environment"`

### Slow responses even with Ollama

**Problem**: Still taking 5-10 seconds

**Possible causes**:
1. **Wrong model**: Check model size matches RAM
2. **Network latency**: Try ping to gaming PC
3. **First request**: First call is slower (model loading)
4. **Wrong backend**: Check web UI shows OLLAMA not OPENAI

**Debug**:
```bash
# Check model is loaded
curl http://192.168.1.100:11434/api/tags

# Test direct inference
curl http://192.168.1.100:11434/api/generate -d '{
  "model": "llama3.1:70b",
  "prompt": "Hello world",
  "stream": false
}'
```

### Backend shows UNKNOWN in web UI

**Problem**: Diagnostics show UNKNOWN backend

**Solutions**:
1. Wait 5-10 seconds for diagnostics to update
2. Refresh browser page
3. Check logs for agent initialization messages
4. Verify mission_agent started without errors

### Wrong backend shown

**Problem**: Web UI shows OPENAI but you launched with ollama

**Solutions**:
1. Verify launch command used `agent_backend:=ollama`
2. Check logs for "Using Ollama backend" message
3. Restart mission agent
4. Clear browser cache and refresh

## Success Criteria

✅ **PASS** if:
- Ollama backend connects successfully
- Simple commands complete in <2 seconds
- Web UI shows green "OLLAMA" indicator
- Performance is 10-20x faster than OpenAI

❌ **FAIL** if:
- Can't connect to Ollama server
- Responses take >5 seconds consistently
- Backend indicator doesn't update
- Errors in logs about model or connection

## Recording Results

After testing, note:
1. Gaming PC specs (CPU, RAM, GPU)
2. Model used (70B/13B/8B)
3. Actual response times:
   - Simple command: _____ seconds
   - Multi-step command: _____ seconds
4. Comparison with OpenAI:
   - Speedup factor: _____x
5. Any issues encountered: _____________

## Next Steps After Successful Test

1. Update DEVLOG.md with test results
2. Add actual performance numbers to documentation
3. Consider merging to dev branch
4. Test on Thor with local Ollama (future)

---

**Questions?** Check:
- `docs/OLLAMA_SETUP.md` - Detailed setup guide
- `docs/BACKEND_QUICK_REFERENCE.md` - Quick commands
- `docs/OLLAMA_BACKEND_INTEGRATION.md` - Architecture details

Good luck with testing! 🚀
