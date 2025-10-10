# Quality Scoring - Quick Reference

**TL;DR**: Benchmark now measures both **speed** AND **quality** to help you choose between fast (8B) vs accurate (70B) models.

---

## What's New

### Before (Performance Only)
```
llama3.1:8b:  21.9 tok/s  ← Fast!
llama3.1:70b: 10.8 tok/s  ← Slow...

Decision: Use 8B? 🤷
```

### After (Performance + Quality)
```
llama3.1:8b:  21.9 tok/s  |  Quality: 85.7/100  ← Fast but mistakes
llama3.1:70b: 10.8 tok/s  |  Quality: 98.7/100  ← Slower but reliable

Decision: Use 70B! 13pts quality gain worth 2x slowdown ✅
```

---

## How It Works

### Automated Checks (No Human Required)

| Prompt Type | Checks |
|-------------|--------|
| **Simple** | ✅ Word count<br>✅ Format compliance<br>✅ No errors |
| **Navigation** | ✅ Valid JSON<br>✅ Required fields<br>✅ Correct structure |
| **Reasoning** | ✅ Answer present<br>✅ Explanation quality<br>✅ Logic markers |

### Score Interpretation

| Score | Meaning |
|-------|---------|
| 90-100 | ✅ Production ready |
| 75-89 | ⚠️ Usable with caution |
| 60-74 | ⚠️ Non-critical only |
| <60 | ❌ Not recommended |

---

## Example Output

```bash
llama3.1:8b
  Performance by Task:
    simple      1.23s  |  12.2 tok/s  |  Q: 100/100  ← Perfect
    navigation  3.45s  |  24.6 tok/s  |  Q: 85/100   ← Some JSON issues
    reasoning   4.12s  |  29.1 tok/s  |  Q: 72/100   ← Weak logic

llama3.1:70b
  Performance by Task:
    simple      3.20s  |   4.7 tok/s  |  Q: 100/100  ← Perfect
    navigation  8.80s  |  10.5 tok/s  |  Q: 100/100  ← Perfect JSON
    reasoning  12.50s  |  16.2 tok/s  |  Q: 96/100   ← Strong reasoning
```

---

## When to Use Which Model

### Use 8B When:
- 🚀 Speed critical (real-time responses)
- 🧪 Development/testing (fast iteration)
- 📝 Simple tasks (word count = 100 for both models)

### Use 70B When:
- 🎯 Accuracy critical (mission planning)
- 🧠 Complex reasoning needed
- 🏭 Production deployment
- 🔒 Safety-critical tasks

---

## Quick Start

```bash
# On Thor - run benchmark (auto-scores quality)
./scripts/benchmark_ollama_models.sh

# Takes 10-20 minutes, outputs:
# - Performance metrics (speed, latency)
# - Quality scores (0-100 per task)
# - Intelligent recommendation
```

---

## Learn More

- **Full Guide**: [OLLAMA_QUALITY_SCORING.md](OLLAMA_QUALITY_SCORING.md) - Complete methodology
- **Benchmarking**: [OLLAMA_BENCHMARKING.md](OLLAMA_BENCHMARKING.md) - How to run tests
- **Academic Background**: 
  - [IFEval Paper](https://arxiv.org/abs/2311.07911) - Instruction-following evaluation
  - [OpenAI Evals](https://github.com/openai/evals) - Evaluation framework

---

**Added**: 2025-10-09  
**Status**: Production Ready  
**Dependencies**: None (Python 3 stdlib only)
