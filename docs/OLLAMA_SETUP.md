# Ollama Backend Setup Guide

This guide explains how to set up and use Ollama as a local LLM backend for ShadowHound, providing **24x faster** mission planning compared to cloud APIs.

## Performance Comparison

| Backend | Typical Response Time | Use Case |
|---------|----------------------|----------|
| **Ollama (Gaming PC)** | 0.5-1s | Development, fastest iteration |
| **Ollama (Thor Local)** | 1-2s | Production, fully autonomous |
| **OpenAI Cloud** | 10-15s | Fallback, highest quality |

## Architecture Overview

ShadowHound supports three deployment scenarios:

```
┌─────────────────────────────────────────────────────────┐
│ Development: Laptop → Gaming PC Ollama                  │
│  - Best for development iteration                       │
│  - Requires network connection to gaming PC             │
│  - Uses agent_backend='ollama'                          │
│  - Config: configs/laptop_dev_ollama.yaml               │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│ Production: Thor → Thor Local Ollama                    │
│  - Fully autonomous (no cloud dependency)               │
│  - Requires Ollama installed on Thor                    │
│  - Uses agent_backend='ollama'                          │
│  - Config: configs/thor_onboard_ollama.yaml             │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│ Fallback: Any → OpenAI Cloud                            │
│  - Slower but always available                          │
│  - Requires internet + OPENAI_API_KEY                   │
│  - Uses agent_backend='openai'                          │
│  - Config: configs/cloud_openai.yaml                    │
└─────────────────────────────────────────────────────────┘
```

## Installation

### Gaming PC Setup (Windows/Linux)

1. **Install Ollama**:
   ```bash
   # Linux
   curl -fsSL https://ollama.com/install.sh | sh
   
   # Windows: Download from https://ollama.com/download
   ```

2. **Pull a model** (one-time, ~40GB for 70B):
   ```bash
   ollama pull llama3.1:70b  # Best quality, requires ~40GB RAM
   # OR
   ollama pull llama3.1:13b  # Faster, requires ~8GB RAM
   # OR
   ollama pull llama3.1:8b   # Fastest, requires ~5GB RAM
   ```

3. **Configure network access**:
   
   **Linux** (systemd):
   ```bash
   sudo systemctl edit ollama
   ```
   Add:
   ```
   [Service]
   Environment="OLLAMA_HOST=0.0.0.0:11434"
   ```
   Then:
   ```bash
   sudo systemctl restart ollama
   ```
   
   **Windows** (environment variable):
   - Open System Properties → Environment Variables
   - Add system variable: `OLLAMA_HOST=0.0.0.0:11434`
   - Restart Ollama service

4. **Configure firewall**:
   ```bash
   # Linux (ufw)
   sudo ufw allow 11434/tcp
   
   # Windows: Add inbound rule for port 11434 in Windows Firewall
   ```

5. **Verify server is accessible**:
   ```bash
   # From laptop (replace with your gaming PC IP)
   curl http://192.168.1.100:11434/api/tags
   ```
   Should return JSON with installed models.

### Thor Setup (Jetson AGX Orin)

1. **Install Ollama on Thor**:
   ```bash
   curl -fsSL https://ollama.com/install.sh | sh
   ```

2. **Pull a model** (choose based on memory):
   ```bash
   # Recommended for Orin (32GB RAM)
   ollama pull llama3.1:13b
   
   # If tight on memory
   ollama pull llama3.1:8b
   ```

3. **Verify installation**:
   ```bash
   ollama list  # Should show downloaded model
   ollama run llama3.1:13b "Hello"  # Quick test
   ```

## Configuration

### Option 1: Launch Arguments (Quick Testing)

```bash
# Development: Use gaming PC Ollama
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.1.100:11434 \
    ollama_model:=llama3.1:70b

# Thor: Use local Ollama
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://localhost:11434 \
    ollama_model:=llama3.1:13b

# Fallback: Use OpenAI cloud
export OPENAI_API_KEY="sk-..."
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=openai \
    openai_model:=gpt-4-turbo
```

### Option 2: Config Files (Recommended)

```bash
# Development
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/laptop_dev_ollama.yaml

# Thor
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/thor_onboard_ollama.yaml

# Cloud fallback
export OPENAI_API_KEY="sk-..."
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/cloud_openai.yaml
```

**Before using laptop config**, edit `configs/laptop_dev_ollama.yaml`:
```yaml
ollama_base_url: "http://192.168.1.100:11434"  # <-- YOUR gaming PC IP
```

## Model Selection Guide

| Model | Size | RAM Required | Speed | Quality | Best For |
|-------|------|--------------|-------|---------|----------|
| llama3.1:70b | ~40GB | 48GB+ | Slow | Excellent | Gaming PC development |
| llama3.1:13b | ~7GB | 16GB+ | Medium | Good | Thor production |
| llama3.1:8b | ~4.5GB | 8GB+ | Fast | Decent | Quick testing |
| mistral:7b | ~4GB | 8GB+ | Fast | Good | Alternative |
| codellama:13b | ~7GB | 16GB+ | Medium | Excellent (code) | Code-heavy tasks |

**Recommendations**:
- **Gaming PC** (64GB+ RAM): llama3.1:70b for best quality
- **Thor** (32GB RAM): llama3.1:13b balances speed and quality
- **Testing**: llama3.1:8b for quick iteration

## Troubleshooting

### "Connection refused" error

**Symptoms**: 
```
Failed to initialize MissionExecutor: Connection refused
```

**Solutions**:
1. **Check Ollama is running**:
   ```bash
   # On gaming PC/Thor
   curl http://localhost:11434/api/tags
   ```

2. **Check network connectivity** (for gaming PC):
   ```bash
   # From laptop
   ping 192.168.1.100
   curl http://192.168.1.100:11434/api/tags
   ```

3. **Verify OLLAMA_HOST setting**:
   ```bash
   # Linux
   sudo systemctl status ollama
   # Should show: Environment="OLLAMA_HOST=0.0.0.0:11434"
   ```

4. **Check firewall**:
   ```bash
   # Linux
   sudo ufw status | grep 11434
   
   # Windows: Check Windows Firewall settings
   ```

### "Model not found" error

**Symptoms**:
```
Error: model 'llama3.1:70b' not found
```

**Solution**:
```bash
# On the Ollama server
ollama pull llama3.1:70b
ollama list  # Verify it appears
```

### Slow response times on Thor

**Symptoms**: Ollama on Thor slower than expected (>5s)

**Solutions**:
1. **Use smaller model**:
   ```bash
   ollama pull llama3.1:8b  # Faster than 13b
   ```

2. **Check CPU/GPU utilization**:
   ```bash
   htop  # Check CPU
   nvidia-smi  # Check GPU (if using GPU acceleration)
   ```

3. **Verify not swapping**:
   ```bash
   free -h  # Check available RAM
   # If swap is being used heavily, model is too large
   ```

### Out of memory on Thor

**Symptoms**:
```
Error: failed to load model: insufficient memory
```

**Solutions**:
1. **Switch to smaller model**:
   ```yaml
   ollama_model: "llama3.1:8b"  # Instead of 13b
   ```

2. **Close other applications**:
   ```bash
   # Kill memory-hungry processes
   sudo systemctl stop docker  # If not needed
   ```

3. **Increase swap** (temporary workaround):
   ```bash
   sudo fallocate -l 8G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

## Performance Testing

After setup, test performance:

```bash
# Start mission agent
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.1.100:11434

# In web UI or separate terminal, send test command:
ros2 topic pub -1 /shadowhound/mission std_msgs/msg/String \
    "data: 'stand up'"

# Check logs for timing:
[INFO] [mission_agent]: Mission complete in 0.8s (agent: 0.7s, overhead: 0.1s)
```

**Expected timings**:
- **Gaming PC 70B**: 0.5-1.5s
- **Thor 13B**: 1-3s
- **Thor 8B**: 0.5-2s
- **OpenAI Cloud**: 10-15s

## Advanced: Custom Models

You can use other Ollama-compatible models:

```bash
# Try different models
ollama pull mistral:7b
ollama pull codellama:13b
ollama pull deepseek-coder:6.7b

# Use in launch:
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_model:=mistral:7b
```

## Security Considerations

⚠️ **Important**: Ollama has no authentication by default!

- **Gaming PC**: Only expose on trusted local network
- **Thor**: Localhost-only is secure (no remote access needed)
- **Production**: Consider adding reverse proxy with auth if exposing externally

## Additional Resources

- [Ollama Documentation](https://github.com/ollama/ollama/blob/main/docs/README.md)
- [Ollama Model Library](https://ollama.com/library)
- [OpenAI-Compatible API](https://github.com/ollama/ollama/blob/main/docs/openai.md)

## Summary: Quick Start

```bash
# 1. Install Ollama on gaming PC
curl -fsSL https://ollama.com/install.sh | sh

# 2. Pull model
ollama pull llama3.1:70b

# 3. Configure network access
sudo systemctl edit ollama
# Add: Environment="OLLAMA_HOST=0.0.0.0:11434"
sudo systemctl restart ollama

# 4. Test from laptop
curl http://<gaming-pc-ip>:11434/api/tags

# 5. Update config
# Edit configs/laptop_dev_ollama.yaml with your gaming PC IP

# 6. Launch ShadowHound
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/laptop_dev_ollama.yaml

# 7. Enjoy 24x faster responses! 🚀
```
