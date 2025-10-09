# Ollama Model Recommendations for ShadowHound

**Last Updated**: 2025-10-09  
**Target Hardware**: NVIDIA Jetson AGX Thor (32GB RAM)

---

## Model Selection Guide

### Primary Recommendation: **llama3.1:13b**

**Why this model:**
- **Best balance** of performance and quality for Thor's 32GB RAM
- **13B parameters** - "Goldilocks zone" for Jetson Thor
- **~10-12 GB RAM** during inference (leaves headroom for ROS + navigation)
- **Response time**: 1-3 seconds for typical mission commands
- **Quality**: Excellent instruction following and reasoning

**Download size**: ~7.4 GB  
**Runtime RAM**: ~10-12 GB  
**Recommended for**: Primary mission agent operation

```bash
docker exec ollama ollama pull llama3.1:13b
```

---

## Alternative Models

### Backup: **llama3.1:8b**

**When to use:**
- Testing with lower resource usage
- Running alongside heavy perception workloads
- Faster responses needed (0.5-1.5s typical)
- Thor RAM is constrained by other processes

**Download size**: ~4.7 GB  
**Runtime RAM**: ~6-8 GB  
**Trade-off**: Slightly less sophisticated reasoning

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

### High-End Option: **llama3.1:70b** ⚠️

**Status**: NOT recommended for Thor

**Why avoid:**
- Requires **40-45 GB RAM** - exceeds Thor's 32GB
- Will cause out-of-memory errors
- Better suited for desktop with 64GB+ RAM

**Alternative**: Use this on gaming PC if available, otherwise stick with 13B

---

## Model Comparison

| Model | Size (GB) | RAM (GB) | Speed | Quality | Thor Compatible |
|-------|-----------|----------|-------|---------|-----------------|
| llama3.1:13b | 7.4 | 10-12 | ★★★☆☆ | ★★★★★ | ✅ **Recommended** |
| llama3.1:8b | 4.7 | 6-8 | ★★★★☆ | ★★★★☆ | ✅ Good backup |
| llama3.2:3b | 2.0 | 3-4 | ★★★★★ | ★★★☆☆ | ✅ Testing only |
| llama3.1:70b | 40+ | 40-45 | ★★☆☆☆ | ★★★★★ | ❌ Too large |

---

## Performance Expectations

### llama3.1:13b on Thor

| Task Type | Expected Time | vs OpenAI (gpt-4-turbo) |
|-----------|---------------|-------------------------|
| Simple command ("rotate 90 degrees") | 1-3s | **10x faster** (was 12s) |
| Multi-step plan ("explore the lab") | 2-4s | **6x faster** (was 25s) |
| Complex reasoning | 3-5s | **5x faster** (was 20s) |

**Network overhead**: +0.1-0.3s (laptop → Thor via LAN)

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

For **ShadowHound development**, pull both models:

```bash
# Primary for production
docker exec ollama ollama pull llama3.1:13b

# Backup for testing/light workloads
docker exec ollama ollama pull llama3.1:8b
```

This gives you **flexibility** to switch based on needs:
- **13B**: Best quality for actual missions
- **8B**: Faster iteration during development

**Total disk**: ~12 GB (acceptable on Thor)

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
