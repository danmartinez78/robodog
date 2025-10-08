# Testing Guide: Ollama Backend on Laptop

**Branch**: `feature/local-llm-support`  
**Date**: 2025-10-08  
**Status**: Ready for testing

## Setup Overview

```
┌─────────────────────────────────────────────┐
│ Laptop (daniel@9510) - Native Ubuntu       │
│  - ROS2 Humble + ShadowHound workspace      │
│  - Mission agent                            │
│  - Web UI (http://localhost:8080)           │
│  - Connects to Thor's Ollama via network    │
└─────────────────────────────────────────────┘
                    ↓ network
┌─────────────────────────────────────────────┐
│ Thor (on desk)                              │
│  - Ollama container                         │
│  - LLM inference (llama3.1:70b/13b)         │
│  - Port 11434 exposed to network            │
└─────────────────────────────────────────────┘
```

## Pre-Flight Checklist

### On Laptop (daniel@9510) - Native Ubuntu
- [ ] Workspace pulled latest changes
- [ ] On `feature/local-llm-support` branch
- [ ] Workspace built (`cb` or colcon build)
- [ ] Workspace sourced (`source-ws`)

### On Thor (Desk Unit)
- [ ] Ollama container running
- [ ] Model downloaded (llama3.1:70b or 13b recommended)
- [ ] Port 11434 accessible from network
- [ ] Firewall allows connections from laptop
- [ ] Thor's IP address known (check with `ip addr`)

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

## Setup Ollama Container on Thor (If Not Already Done)

```bash
# SSH to Thor
ssh thor  # or use your Thor's hostname/IP

# Option 1: Run Ollama with Docker
docker run -d \
  --name ollama \
  --gpus all \
  -p 11434:11434 \
  -v ollama_data:/root/.ollama \
  --restart unless-stopped \
  ollama/ollama

# Option 2: Using docker-compose (if you have a compose file)
cd /path/to/ollama
docker-compose up -d

# Pull model (choose based on Thor's RAM)
docker exec -it ollama ollama pull llama3.1:13b  # Recommended for Thor (32GB RAM)
# OR
docker exec -it ollama ollama pull llama3.1:8b   # If memory constrained

# Verify it's running
curl http://localhost:11434/api/tags

# Get Thor's IP address (for laptop connection)
ip addr show | grep "inet " | grep -v 127.0.0.1
# Note the IP (e.g., 192.168.1.50)
```

## Test Connectivity from Laptop

```bash
# On laptop (native Ubuntu), test Thor's Ollama
# Replace 192.168.1.50 with your Thor's actual IP
curl http://192.168.1.50:11434/api/tags

# Should return JSON with installed models
# Example:
# {"models":[{"name":"llama3.1:13b",...}]}

# If connection fails, check:
# 1. Thor's IP is correct: ssh thor "ip addr show | grep inet"
# 2. Ollama container is running: ssh thor "docker ps | grep ollama"
# 3. Port is exposed: ssh thor "docker port ollama"
# 4. Firewall allows it: ssh thor "sudo ufw status"
```

## Test 1: Launch with Ollama Backend

```bash
# On laptop (native Ubuntu)
# Replace 192.168.1.50 with your Thor's actual IP

ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.1.50:11434 \
    ollama_model:=llama3.1:13b
```

**Expected Output:**
```
[INFO] [mission_agent]: Configuration:
[INFO] [mission_agent]:   Agent backend: ollama
[INFO] [mission_agent]:   Ollama URL: http://192.168.1.50:11434
[INFO] [mission_agent]:   Ollama model: llama3.1:13b
[INFO] [mission_agent]: MissionExecutor ready!
[INFO] [mission_agent]: Using Ollama backend at http://192.168.1.50:11434
[INFO] [mission_agent]: Ollama model: llama3.1:13b
[INFO] [mission_agent]: Web interface started at http://localhost:8080
```

## Test 2: Verify Web UI Shows Ollama Backend

1. Open browser: http://localhost:8080
2. Check diagnostics panel (right side, below performance)
3. Verify:
   - **LLM BACKEND**: Shows "OLLAMA" in **green**
   - **MODEL**: Shows "LLAMA3.1:13B" (or 8B/70B depending on what you pulled)

## Test 3: Simple Command Performance Test

### With Ollama (Expected: <2s)

```bash
# In web UI or terminal:
ros2 topic pub -1 /shadowhound/mission std_msgs/msg/String "data: 'stand up'"

# Watch logs for timing:
# [INFO] [mission_agent]: Mission complete in X.Xs (agent: X.Xs, overhead: X.Xs)
```

**Expected Results (Thor 13B model):**
- Total duration: **1-3 seconds** ⚡
- Agent duration: **1-2 seconds**
- Overhead: <0.5 seconds
- Web UI backend indicator: **OLLAMA (green)**
- Note: Network adds ~0.1-0.3s latency vs localhost

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

**Expected with Ollama (Thor 13B):**
- **2-4 seconds** total (slightly slower for planning)

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

# Update ollama_base_url with your Thor's IP:
ollama_base_url: "http://192.168.1.50:11434"  # Replace with Thor's actual IP
ollama_model: "llama3.1:13b"  # Match what you pulled on Thor
```

Launch with config:
```bash
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/laptop_dev_ollama.yaml
```

Verify all settings loaded correctly in logs.

## Performance Targets

| Metric | Ollama (Thor 13B) | OpenAI Cloud | Status |
|--------|------------------|--------------|--------|
| Simple command | 1-3s | 10-15s | Pass if <4s |
| Multi-step | 2-4s | 20-30s | Pass if <6s |
| Backend indicator | Green "OLLAMA" | Orange "OPENAI" | Visual check |
| Network latency | +0.1-0.3s | N/A | Laptop→Thor overhead |

## Troubleshooting

### "Connection refused" error

**Problem**: Can't connect to Ollama
```
Error: Connection refused to http://192.168.1.100:11434
```

**Solutions**:
1. Check Ollama container is running: `ssh thor "docker ps | grep ollama"`
2. Test connectivity from laptop: `curl http://192.168.1.50:11434/api/tags`
3. Check container port: `ssh thor "docker port ollama"`
4. Restart container if needed: `ssh thor "docker restart ollama"`
5. Check Thor's firewall: `ssh thor "sudo ufw status"`
6. Verify network connectivity: `ping 192.168.1.50` (from laptop)

### Slow responses even with Ollama

**Problem**: Still taking 5-10 seconds

**Possible causes**:
1. **Wrong model**: Check model size matches RAM
2. **Network latency**: Try ping to gaming PC
3. **First request**: First call is slower (model loading)
4. **Wrong backend**: Check web UI shows OLLAMA not OPENAI

**Debug**:
```bash
# Check model is loaded on Thor
curl http://192.168.1.50:11434/api/tags

# Test direct inference from laptop
curl http://192.168.1.50:11434/api/generate -d '{
  "model": "llama3.1:13b",
  "prompt": "Hello world",
  "stream": false
}'

# Check container logs on Thor
ssh thor "docker logs ollama --tail 50"

# Monitor Thor's resource usage
ssh thor "docker stats ollama --no-stream"
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
- Ollama backend connects to Thor successfully
- Simple commands complete in <4 seconds
- Web UI shows green "OLLAMA" indicator
- Performance is 5-10x faster than OpenAI (accounting for network)

❌ **FAIL** if:
- Can't connect to Ollama server
- Responses take >5 seconds consistently
- Backend indicator doesn't update
- Errors in logs about model or connection

## Recording Results

After testing, note:
1. Thor specs (AGX Orin, RAM, GPU)
2. Model used (13B/8B recommended for Thor)
3. Network setup (laptop native Ubuntu → Thor container via LAN)
4. Actual response times:
   - Simple command: _____ seconds
   - Multi-step command: _____ seconds
   - Network latency (ping): _____ ms
5. Comparison with OpenAI:
   - Speedup factor: _____x
6. Any issues encountered: _____________

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
