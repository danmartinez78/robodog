# Ollama Model Benchmarking Guide

**Last Updated**: 2025-10-09  
**Purpose**: Test and compare Ollama models before deployment

---

## Overview

The `benchmark_ollama_models.sh` script helps you **objectively compare** different Ollama models on your actual hardware (Thor) before committing to one for production use.

### What It Tests

1. **Speed**: Tokens per second generation rate
2. **Latency**: Time to first token (responsiveness)
3. **Quality**: Response accuracy, completeness, instruction-following (see [Quality Scoring Guide](OLLAMA_QUALITY_SCORING.md))
4. **Resource Usage**: Model size and memory footprint

### Test Scenarios

The benchmark runs three types of prompts to simulate real mission agent tasks:

- **Simple**: Basic acknowledgment (tests baseline speed + instruction following)
- **Navigation**: JSON plan generation (tests structured output + validity)
- **Reasoning**: Problem-solving task (tests logic + explanation quality)

**New in v2.0**: Automated quality scoring inspired by OpenAI Evals and IFEval. Each response gets a 0-100 quality score based on accuracy, completeness, and task-specific criteria.

---

## Quick Start

### 1. Make Sure Ollama is Running

```bash
# On Thor - check container status
docker ps | grep ollama

# Should show ollama container running on port 11434
```

If not running, start it:
```bash
./scripts/setup_ollama_thor.sh
```

### 2. Run Benchmark

```bash
# On Thor
cd ~/shadowhound
./scripts/benchmark_ollama_models.sh
```

### 3. Review Results

The script will:
1. Pull models if not already downloaded
2. Warm up each model (first inference is always slower)
3. Run 3 test prompts per model
4. Generate JSON results + summary report
5. Provide recommendation

**Runtime**: ~10-20 minutes depending on models tested

---

## Understanding the Output

### During Execution

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Testing: llama3.1:8b
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Model Info:
Size: 4.9GB | Parameters: 8B | Family: llama

Warming up model...
✓ Model loaded

  Testing: simple
    Duration: 1.23s | Tokens: 15 | Speed: 12.2 tok/s | TTFT: 0.15s
  Testing: navigation
    Duration: 3.45s | Tokens: 85 | Speed: 24.6 tok/s | TTFT: 0.18s
  Testing: reasoning
    Duration: 4.12s | Tokens: 120 | Speed: 29.1 tok/s | TTFT: 0.21s
```

### Summary Report

```
============================================================
BENCHMARK SUMMARY
============================================================

llama3.1:8b
------------------------------------------------------------
  Total Duration:      8.80s
  Total Tokens:        220
  Avg Speed:           21.9 tokens/sec
  Avg Time to First:   0.180s

  Performance by Task:
    simple           1.23s  |  12.2 tok/s  |  Q: 100/100
    navigation       3.45s  |  24.6 tok/s  |  Q: 85/100
    reasoning        4.12s  |  29.1 tok/s  |  Q: 72/100

llama3.1:70b
------------------------------------------------------------
  Total Duration:      24.50s
  Total Tokens:        265
  Avg Speed:           10.8 tokens/sec
  Avg Time to First:   0.850s

  Performance by Task:
    simple           3.20s  |   4.7 tok/s  |  Q: 100/100
    navigation       8.80s  |  10.5 tok/s  |  Q: 100/100
    reasoning       12.50s  |  16.2 tok/s  |  Q: 96/100

============================================================
RECOMMENDATIONS
============================================================

🚀 Fastest Model:       llama3.1:8b (21.9 tok/s)
🎯 Best Quality:        llama3.1:70b (98.7/100)

📊 Speed vs Quality Tradeoff:
   llama3.1:8b          Speed: 21.9 tok/s  |  Quality: 85.7/100
   llama3.1:70b         Speed: 10.8 tok/s  |  Quality: 98.7/100

💡 Recommendation:
   🌟 Use llama3.1:70b - 13pts better quality, only 2.0x slower!
```

**Note**: Quality scores (Q: X/100) measure response accuracy and instruction-following. See [Quality Scoring Guide](OLLAMA_QUALITY_SCORING.md) for details.

---

## Metrics Explained

### Speed Metrics

#### Tokens Per Second (tok/s)
- **Higher is better**
- How fast the model generates text
- **8B**: Typically 15-30 tok/s
- **70B**: Typically 5-15 tok/s

#### Time to First Token (TTFT)
- **Lower is better**
- How quickly the model starts responding
- Important for perceived responsiveness
- **8B**: ~0.1-0.3s
- **70B**: ~0.5-1.5s

#### Duration
- Total time to complete the response

### Quality Metrics

#### Quality Score (0-100)
- **Higher is better**
- Automated evaluation of response quality
- Measures:
  - **Simple**: Instruction following (word count, format)
  - **Navigation**: JSON validity, structure, completeness
  - **Reasoning**: Answer presence, logic, explanation quality
- **90-100**: Excellent (production-ready)
- **75-89**: Good (usable)
- **60-74**: Fair (consider for non-critical)
- **<60**: Poor (significant issues)

See [OLLAMA_QUALITY_SCORING.md](OLLAMA_QUALITY_SCORING.md) for complete scoring methodology.
- Depends on both speed and response length

---

## Customizing Tests

### Add More Models

Edit `scripts/benchmark_ollama_models.sh`:

```bash
# Models to test (in order of size)
declare -a MODELS=(
    "llama3.1:8b"
    "llama3.1:70b"
    "mistral:7b"           # Add alternative models
    "phi-2:latest"         # Add smaller models
)
```

### Add Custom Prompts

```bash
declare -A TEST_PROMPTS=(
    ["simple"]="Say hello in exactly 3 words"
    ["navigation"]="Generate a JSON plan..."
    ["reasoning"]="A robot needs to..."
    ["custom"]="Your custom test prompt here"  # Add your own
)
```

### Change Test Parameters

```bash
# In the benchmark_prompt function, adjust:
"options": {
    "temperature": 0.7,      # Randomness (0-1)
    "num_predict": 200       # Max tokens to generate
}
```

---

## Interpreting Results for ShadowHound

### Speed vs Quality Tradeoff

| Model | Speed | Quality | Use Case |
|-------|-------|---------|----------|
| **llama3.1:8b** | ⚡⚡⚡⚡⚡ | ⭐⭐⭐⭐ | Development, testing, simple tasks |
| **llama3.1:70b** | ⚡⚡⚡ | ⭐⭐⭐⭐⭐ | Production missions, complex planning |

### Mission Agent Requirements

**Ideal characteristics for robot missions:**
- ✅ **Latency**: <2s total response time for simple commands
- ✅ **Throughput**: >10 tok/s for plan generation
- ✅ **Quality**: Reliable JSON output, good reasoning
- ✅ **Consistency**: Repeatable results with temp=0.7

### Decision Matrix

**Choose 8B if:**
- You need fast iteration during development
- Simple navigation tasks are primary use case
- You want snappy responses
- Running many inference requests in parallel

**Choose 70B if:**
- Complex mission planning is needed
- Quality/reliability is critical
- You can accept 2-4s response times
- You need better reasoning and understanding

**Recommendation**: Run both! Use 8B for development, switch to 70B for actual missions.

---

## Advanced: Automated Testing

### Run Benchmark on Schedule

Add to Thor's crontab:
```bash
# Benchmark weekly to track performance
0 2 * * 0 /home/user/shadowhound/scripts/benchmark_ollama_models.sh > /tmp/ollama_benchmark.log 2>&1
```

### Compare Models Over Time

```bash
# View historical results
ls -lh ~/ollama_benchmarks/

# Compare two benchmark runs
diff <(jq '.[0].tests' benchmark1.json) <(jq '.[0].tests' benchmark2.json)
```

### Automated Model Selection

Use benchmark results in launch files:

```python
# In launch file
import json
from pathlib import Path

# Load latest benchmark
benchmark_file = Path.home() / "ollama_benchmarks" / "latest.json"
with open(benchmark_file) as f:
    results = json.load(f)

# Pick fastest model that meets threshold
for model in results:
    avg_speed = sum(t['tokens_per_second'] for t in model['tests']) / len(model['tests'])
    if avg_speed > 15.0:  # Minimum acceptable speed
        selected_model = model['model']
        break
```

---

## Troubleshooting

### Benchmark Fails to Connect

```bash
# Check Ollama is running
curl http://localhost:11434/api/tags

# Check container
docker ps | grep ollama

# View logs
docker logs ollama
```

### Model Download Fails

```bash
# Check disk space
df -h ~/ollama-data

# Check internet
ping ollama.com

# Manually pull model
docker exec ollama ollama pull llama3.1:8b
```

### Slow Performance

**First run is always slower** - models need to be loaded into memory.

```bash
# Warm up model manually
docker exec ollama ollama run llama3.1:8b "hi"

# Then run benchmark
```

### Out of Memory

Thor has 128GB, should handle 70B model. If OOM occurs:

```bash
# Check memory usage
free -h

# Check what's using memory
docker stats

# Stop other containers
docker stop <other-containers>
```

---

## Example Results (Reference)

### NVIDIA Jetson AGX Thor (128GB RAM)

**llama3.1:8b**
- Avg Speed: 22 tok/s
- Avg TTFT: 0.18s
- Total for 3 tests: ~8s
- Quality: Good for most tasks

**llama3.1:70b**
- Avg Speed: 11 tok/s
- Avg TTFT: 0.85s
- Total for 3 tests: ~24s
- Quality: Excellent, near GPT-4

**Conclusion**: 70B is ~2x slower but significantly better quality. Worth it for production missions!

---

## Integration with Setup Script

The benchmark results inform your choice in `setup_ollama_thor.sh`:

```bash
# Based on benchmark results:
PRIMARY_MODEL="llama3.1:70b"    # If quality is priority
# OR
PRIMARY_MODEL="llama3.1:8b"     # If speed is priority
```

You can also use env var to switch at runtime:

```bash
export OLLAMA_MODEL="llama3.1:8b"  # Fast for development
# OR
export OLLAMA_MODEL="llama3.1:70b" # Quality for missions
```

---

## Best Practices

1. **Benchmark on Thor** - Don't trust specs, test on your actual hardware
2. **Test with real prompts** - Add your actual mission commands to TEST_PROMPTS
3. **Consider task variety** - Balance of simple/complex tasks
4. **Warm up matters** - First inference is slower, benchmark accounts for this
5. **Track over time** - Re-benchmark after Ollama updates

---

## Related Documentation

- [OLLAMA_MODELS.md](OLLAMA_MODELS.md) - Model recommendations
- [OLLAMA_SETUP.md](OLLAMA_SETUP.md) - Installation guide
- Setup script: `scripts/setup_ollama_thor.sh`
- Test script: `scripts/test_ollama_laptop.sh`

---

*Benchmark before you deploy! Objective data beats guessing.* 🎯
