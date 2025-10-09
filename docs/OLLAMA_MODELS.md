# Ollama Model Recommendations for ShadowHound

**Last Updated**: 2025-10-09  
**Target Hardware**: NVIDIA Jetson AGX Thor (128GB RAM)

---

## Model Selection Guide

> **IMPORTANT**: llama3.1 is only available in **8B, 70B, and 405B** sizes. There is NO 13B variant!  
> See: https://ollama.com/library/llama3.1/tags

### Primary Recommendation: **llama3.1:70b**

**Why this model:**
- **Best model Thor can run** - 128GB RAM is perfect for 70B!
- **70B parameters** - Near GPT-4 level quality
- **~60-70 GB RAM** during inference (plenty of headroom for ROS + navigation)
- **Response time**: 2-5 seconds for typical mission commands
- **Quality**: Exceptional instruction following, reasoning, and planning

**Download size**: ~43 GB  
**Runtime RAM**: ~60-70 GB  
**Recommended for**: Primary mission agent operation

```bash
docker exec ollama ollama pull llama3.1:70b
```

---

## Alternative Models

### Backup: **llama3.1:8b**

**When to use:**
- Faster responses needed (0.5-1.5s typical)
- Running alongside heavy perception workloads
- Testing/development with quick iteration
- When you need snappy responses over maximum quality

**Download size**: ~4.9 GB  
**Runtime RAM**: ~10-12 GB  
**Trade-off**: Lower quality reasoning compared to 70B, but still very capable

```bash
docker exec ollama ollama pull llama3.1:8b
```

---

### For Experimentation: **llama3.2:3b**

**When to use:**
- Minimal resource footprint
- Testing/development only (not for production missions)
- Very fast responses (<0.5s)

**Download size**: ~2 GB  
**Runtime RAM**: ~3-4 GB  
**Trade-off**: Reduced instruction following, simpler reasoning

```bash
docker exec ollama ollama pull llama3.2:3b
```

---

### High-End Option: **llama3.1:405b** ⚠️

**Status**: NOT recommended for Thor

**Why avoid:**
- Requires **~250+ GB RAM** - exceeds Thor's 128GB
- Model size: ~243GB just to download
- Would cause out-of-memory errors
- Better suited for high-end server with 512GB+ RAM

---

## Model Comparison

| Model | Size (GB) | RAM (GB) | Speed | Quality | Thor Compatible |
|-------|-----------|----------|-------|---------|-----------------|
| llama3.1:70b | 43 | 60-70 | ★★★☆☆ | ★★★★★ | ✅ **Recommended** |
| llama3.1:8b | 4.9 | 10-12 | ★★★★★ | ★★★★☆ | ✅ Faster backup |
| llama3.2:3b | 2.0 | 3-4 | ★★★★★ | ★★★☆☆ | ✅ Testing only |
| llama3.1:405b | 243+ | 250+ | ★☆☆☆☆ | ★★★★★ | ❌ Too large |

---

## Performance Expectations

### llama3.1:70b on Thor (128GB RAM)

| Task Type | Expected Time | vs OpenAI (gpt-4-turbo) |
|-----------|---------------|-------------------------|
| Simple command ("rotate 90 degrees") | 2-4s | **5x faster** (was 12s) |
| Multi-step plan ("explore the lab") | 3-6s | **4x faster** (was 25s) |
| Complex reasoning | 4-8s | **3x faster** (was 20s) |

**Quality**: Near GPT-4 level - significantly better than 8B model  
**Network overhead**: +0.1-0.3s (laptop → Thor via LAN)

### llama3.1:8b on Thor (for comparison)

| Task Type | Expected Time | vs 70B Quality |
|-----------|---------------|----------------|
| Simple command | 0.5-1.5s | Good enough |
| Multi-step plan | 1-3s | Noticeably simpler |
| Complex reasoning | 2-4s | May miss nuance |

---

## Model Management Commands

### Pull a model
```bash
docker exec ollama ollama pull <model-name>
```

### List installed models
```bash
docker exec ollama ollama list
```

### Remove a model
```bash
docker exec ollama ollama rm <model-name>
```

### Test a model interactively
```bash
docker exec -it ollama ollama run llama3.1:13b
```

### Check model info
```bash
docker exec ollama ollama show llama3.1:13b
```

---

## Switching Models at Runtime

### Option 1: Launch Parameter
```bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=ollama \
    ollama_model:=llama3.1:8b  # Change model here
```

### Option 2: Config File
Edit `configs/laptop_dev_ollama.yaml`:
```yaml
ollama_model: "llama3.1:8b"  # Change from 13b to 8b
```

Then launch:
```bash
ros2 launch shadowhound_bringup shadowhound.launch.py \
    config:=configs/laptop_dev_ollama.yaml
```

---

## Advanced: Quantization Variants

Ollama uses **Q4_0 quantization** by default (good balance).

For more control, you can specify variants:

```bash
# Higher quality, more RAM
docker exec ollama ollama pull llama3.1:13b-q8_0

# Lower RAM, faster
docker exec ollama ollama pull llama3.1:13b-q4_K_M

# Ultra-low RAM
docker exec ollama ollama pull llama3.1:13b-q3_K_S
```

**Default Q4_0 is recommended** - good balance without manual tuning.

---

## Multi-Model Setup

You can have multiple models installed and switch between them:

```bash
# Pull both models
docker exec ollama ollama pull llama3.1:13b
docker exec ollama ollama pull llama3.1:8b

# Use 13b for missions
ros2 launch ... ollama_model:=llama3.1:13b

# Use 8b for testing/development
ros2 launch ... ollama_model:=llama3.1:8b
```

**Disk usage**: Models stored in `~/ollama-data/` on Thor
- 13B: ~7.4 GB
- 8B: ~4.7 GB
- Total: ~12 GB for both

---

## Troubleshooting

### Model download fails
```bash
# Check Thor's internet connection
ping google.com

# Check disk space
df -h ~/ollama-data

# Retry download
docker exec ollama ollama pull llama3.1:13b
```

### Out of memory during inference
```bash
# Check memory usage
docker exec ollama free -h

# Switch to smaller model
# Use llama3.1:8b instead of 13b

# Or close other applications on Thor
```

### Slow inference
```bash
# Check GPU utilization on Thor
nvidia-smi

# Verify GPU is being used by container
docker exec ollama nvidia-smi

# Check if model is loaded (first request is slower)
# Subsequent requests should be faster
```

---

## Recommended Setup

For **ShadowHound development** with Thor's 128GB RAM:

```bash
# Primary for production - BEST quality
docker exec ollama ollama pull llama3.1:70b

# Backup for fast iteration during development
docker exec ollama ollama pull llama3.1:8b
```

This gives you **flexibility** to switch based on needs:
- **70B**: Best quality for actual missions, complex planning, near GPT-4 performance
- **8B**: Faster responses during development/testing

**Total disk**: ~48 GB (totally fine on Thor)  
**Peak RAM**: ~70 GB when running 70B (still leaves ~58GB free for ROS/Nav/Perception)

---

## Future: Specialized Models

As Ollama ecosystem grows, consider these for specific tasks:

- **codellama:13b** - For code generation skills
- **mistral:7b** - Alternative to llama, good reasoning
- **phi-2** - Tiny (2.7B) but surprisingly capable
- **neural-chat:7b** - Optimized for dialogue

**For now**: Stick with **llama3.1:13b** - it's well-tested and reliable.

---

*This guide is based on NVIDIA Jetson AGX Thor specifications (32GB RAM, integrated GPU). Performance may vary with workload and concurrent processes.*
