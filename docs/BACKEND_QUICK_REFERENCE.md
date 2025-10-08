# LLM Backend Quick Reference

Quick reference for switching between OpenAI and Ollama backends.

## TL;DR

```bash
# Ollama (Fast - 0.5-2s) ⚡
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.1.100:11434

# OpenAI (Slow - 10-15s) 
export OPENAI_API_KEY="sk-..."
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=openai
```

## Backend Comparison

| Feature | Ollama | OpenAI Cloud |
|---------|--------|--------------|
| **Speed** | 0.5-2s ⚡ | 10-15s 🐌 |
| **Cost** | Free | $0.01-0.03/request |
| **Setup** | Install + Model Download | Just API key |
| **Internet** | Not required (offline) | Required |
| **Quality** | Good (llama3.1) | Excellent (gpt-4) |
| **Best For** | Development, Production | Fallback, High Quality |

## Quick Commands

### Using Ollama (Recommended)

```bash
# Development (Gaming PC)
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.1.100:11434 \
    ollama_model:=llama3.1:70b

# Thor (Local)
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://localhost:11434 \
    ollama_model:=llama3.1:13b

# With Config File (Easiest)
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/laptop_dev_ollama.yaml
```

### Using OpenAI Cloud

```bash
# Set API key first
export OPENAI_API_KEY="sk-proj-..."

# Launch with OpenAI
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=openai \
    openai_model:=gpt-4-turbo

# Or with config file
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/cloud_openai.yaml
```

## Configuration Files

| Config File | Backend | Use Case |
|-------------|---------|----------|
| `configs/laptop_dev_ollama.yaml` | Ollama | Development on laptop |
| `configs/thor_onboard_ollama.yaml` | Ollama | Production on Thor |
| `configs/cloud_openai.yaml` | OpenAI | Cloud fallback |

## Model Selection

### Ollama Models

| Model | Speed | Quality | RAM | Best For |
|-------|-------|---------|-----|----------|
| llama3.1:70b | Slow | Excellent | 48GB+ | Gaming PC, best quality |
| llama3.1:13b | Medium | Good | 16GB+ | Thor, balanced |
| llama3.1:8b | Fast | Decent | 8GB+ | Quick testing |

### OpenAI Models

| Model | Speed | Quality | Cost/1K tokens | Best For |
|-------|-------|---------|----------------|----------|
| gpt-4-turbo | Medium | Excellent | $0.01/$0.03 | Complex missions |
| gpt-3.5-turbo | Fast | Good | $0.0005/$0.0015 | Simple commands |

## Testing Backend Performance

```bash
# 1. Start mission agent
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_base_url:=http://192.168.1.100:11434

# 2. Send test command
ros2 topic pub -1 /shadowhound/mission std_msgs/msg/String \
    "data: 'stand up'"

# 3. Check logs for timing
# Look for: "Mission complete in X.Xs (agent: X.Xs, overhead: X.Xs)"
```

**Expected Results**:
- Ollama (Gaming PC 70B): 0.5-1.5s
- Ollama (Thor 13B): 1-3s
- OpenAI (gpt-4-turbo): 10-15s

## Troubleshooting

### "Connection refused" (Ollama)
```bash
# Check Ollama is running
curl http://192.168.1.100:11434/api/tags

# Restart Ollama
sudo systemctl restart ollama  # Linux
```

### "Invalid API key" (OpenAI)
```bash
# Check API key is set
echo $OPENAI_API_KEY

# Set it
export OPENAI_API_KEY="sk-proj-..."
```

### Slow responses
```bash
# Use smaller model
ollama_model:=llama3.1:8b  # Instead of 70b

# Or switch to faster OpenAI model
openai_model:=gpt-3.5-turbo  # Instead of gpt-4-turbo
```

## Environment Variables

```bash
# OpenAI
export OPENAI_API_KEY="sk-proj-..."  # Required for OpenAI backend

# No env vars needed for Ollama!
```

## Complete Setup Examples

### Gaming PC Development Setup

```bash
# 1. On gaming PC: Install Ollama
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.1:70b
sudo systemctl edit ollama  # Add: OLLAMA_HOST=0.0.0.0:11434
sudo systemctl restart ollama

# 2. On laptop: Test connectivity
curl http://192.168.1.100:11434/api/tags

# 3. Edit config with gaming PC IP
# Edit configs/laptop_dev_ollama.yaml

# 4. Launch!
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/laptop_dev_ollama.yaml
```

### Thor Production Setup

```bash
# 1. Install Ollama on Thor
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.1:13b

# 2. Launch with local Ollama
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/thor_onboard_ollama.yaml
```

## All Launch Arguments

```bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=<openai|ollama> \
    openai_model:=<model-name> \
    openai_base_url:=<url> \
    ollama_base_url:=<url> \
    ollama_model:=<model-name> \
    use_planning_agent:=<true|false> \
    enable_web_interface:=<true|false> \
    web_port:=<port>
```

## More Information

- **Full Setup Guide**: [docs/OLLAMA_SETUP.md](OLLAMA_SETUP.md)
- **Architecture Details**: [docs/OLLAMA_BACKEND_INTEGRATION.md](OLLAMA_BACKEND_INTEGRATION.md)
- **Ollama Documentation**: https://github.com/ollama/ollama

---

**Recommendation**: Start with Ollama for development (24x faster!), keep OpenAI as fallback for when you need highest quality or Ollama is unavailable.
