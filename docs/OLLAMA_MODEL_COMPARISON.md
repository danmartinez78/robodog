# Ollama Model Comparison for ShadowHound

**Last Updated**: 2025-01-10  
**Purpose**: Document model selection rationale for robot control workloads  
**Hardware Target**: Thor (128GB RAM, ~58GB available for models)

---

## Executive Summary

After comprehensive research of the Ollama model ecosystem (100+ available models), we've identified **5 primary candidates** and 2 optional models for ShadowHound testing. These models were selected based on:

1. **Memory constraints**: Must fit in 58GB available RAM
2. **Use case alignment**: Excel at structured output (JSON), planning, and reasoning
3. **Popularity/maturity**: High pull counts indicate community validation
4. **Specialization**: Task-specific models (coding, reasoning) vs general-purpose
5. **Performance diversity**: Range from 7B (fast) to 70B (quality) parameters

**Expected Winner**: `qwen2.5-coder:32b` for production, `phi4:14b` for development

---

## Model Selection Matrix

| Model | Size | RAM | Pulls | Specialization | Use Case Fit | Speed Estimate |
|-------|------|-----|-------|----------------|--------------|----------------|
| **phi4:14b** | 9.1GB | ~10GB | 5.3M | SOTA efficiency | Fast dev/test | ~80 tok/s ⚡⚡⚡ |
| **qwen2.5-coder:32b** | 20GB | ~20GB | 7.5M | Code/JSON specialist | **Navigation plans** | ~35 tok/s ⚡⚡ |
| **qwq:32b** | 20GB | ~20GB | 1.7M | Reasoning specialist | Complex planning | ~35 tok/s ⚡⚡ |
| **llama3.3:70b** | 43GB | ~43GB | 2.6M | General purpose | Baseline upgrade | ~18 tok/s ⚡ |
| **deepseek-r1:7b** | 4.7GB | ~5GB | 65.2M* | Reasoning chains | Fast reasoning | ~120 tok/s ⚡⚡⚡⚡ |
| *hermes3:70b* | 50GB | ~50GB | 339K | Function calling | Future tools API | ~18 tok/s ⚡ |
| *gemma2:27b* | 17GB | ~17GB | 8.1M | Efficient | Middle ground | ~45 tok/s ⚡⚡ |

*Combined pulls across all deepseek-r1 sizes

---

## Tier 1: Primary Testing Candidates

### 🥇 **qwen2.5-coder:32b** - JSON/Coding Specialist

**Why This is Likely the Winner**:
- **Built for structured output**: Trained specifically for code generation
- **JSON expertise**: Should achieve 95-100/100 on navigation prompt quality
- **Proven popularity**: 7.5M pulls = battle-tested in production
- **Optimal size**: 32B parameters = sweet spot for quality/speed
- **Robot control fit**: Navigation plans are essentially code (structured instructions)

**Expected Performance**:
- Simple prompts: 95-100/100 quality
- Navigation (JSON): **98-100/100** ← Best in class
- Reasoning: 85-90/100 (not specialized, but capable)
- Speed: ~35 tok/s (good enough for missions)

**Use Case**: **Primary production model** for mission planning and navigation

---

### 🥈 **qwq:32b** - Reasoning Specialist

**Why This Matters**:
- **Purpose-built reasoning**: From Qwen's reasoning-focused series
- **Complex planning**: Excels at multi-step logical decisions
- **Emerging but proven**: 1.7M pulls, newer but gaining traction
- **Same size as qwen-coder**: Direct comparison at 32B parameter level

**Expected Performance**:
- Simple prompts: 95-100/100 quality
- Navigation (JSON): 85-95/100 (formatting secondary to logic)
- Reasoning: **95-100/100** ← Best in class
- Speed: ~35 tok/s (comparable to qwen-coder)

**Use Case**: **Alternative primary** if reasoning > JSON formatting, or for complex obstacle scenarios

---

### 🥉 **phi4:14b** - Speed Champion

**Why This is Essential**:
- **Microsoft SOTA**: State-of-the-art small model from Microsoft Research
- **Efficiency leader**: 5.3M pulls, highly regarded in community
- **Development speed**: 2.5x faster than 32B models, near-instant responses
- **Surprising quality**: Small models have dramatically improved in 2024-2025

**Expected Performance**:
- Simple prompts: 95-100/100 quality
- Navigation (JSON): 90-95/100 (strong, but not specialist level)
- Reasoning: 85-95/100 (impressive for size)
- Speed: **~80 tok/s** ← 2.5x faster than 32B models

**Use Case**: **Development/testing model**, backup for production when speed critical

---

## Tier 2: Validation & Comparison

### **llama3.3:70b** - Latest Meta Release

**Why Test This**:
- **Direct upgrade**: Meta claims "similar to llama3.1:405b performance"
- **Current baseline successor**: Natural evolution from llama3.1:70b
- **Proven architecture**: Meta's LLaMA series is industry standard
- **Largest in our range**: 70B = maximum quality we can fit

**Expected Performance**:
- All tasks: 90-100/100 (generalist strength)
- Speed: ~18 tok/s (slowest, but acceptable)

**Use Case**: **Validation** that specialized models beat generalists

---

### **deepseek-r1:7b** - Experimental Reasoning

**Why This is Interesting**:
- **"Reasoning approaching O3"**: Cutting-edge reasoning architecture
- **Massive popularity**: 65.2M combined pulls (all sizes) = most popular
- **Ultra-fast**: Smallest model = fastest responses
- **Curiosity test**: Can 7B compete with 32B on reasoning?

**Expected Performance**:
- Simple prompts: 90-95/100
- Navigation (JSON): 80-90/100 (size limitation)
- Reasoning: 90-95/100 (specialized architecture compensates for size)
- Speed: **~120 tok/s** ← Fastest option

**Use Case**: **CI/testing**, curiosity about reasoning vs size tradeoff

---

## Tier 3: Optional Models

### **hermes3:70b** - Tool-Use Specialist (Optional)

**Why Consider**:
- **Function calling**: Built for tool-based use cases
- **Skills API alignment**: Future expansion of skills API with tool definitions
- **70B quality**: Large model = high capability

**When to Test**:
- If planning to expand skills API with function calling
- If need tool-use capabilities (structured API calls)
- If have extra time for benchmarking

**Downside**: 50GB RAM = 86% memory utilization (risky)

---

### **gemma2:27b** - Google's Efficient Model (Optional)

**Why Consider**:
- **Middle ground**: Between 14B and 32B
- **Google quality**: 8.1M pulls, proven
- **Efficiency focus**: Optimized for resource usage

**When to Test**:
- If 32B models are too slow
- If 14B models have insufficient quality
- If need compromise option

**Downside**: Likely beaten by phi4 (speed) and qwen-coder (quality)

---

## Memory Usage Analysis

Thor has **128GB total RAM**:
- ROS2 + Nav2 + Perception: ~30GB
- System overhead: ~20GB
- OS/buffers: ~20GB
- **Available for models**: ~58GB

### Models by Memory Footprint

**Small (can run 5+ simultaneously)**:
- deepseek-r1:7b: ~5GB ✅
- phi4:14b: ~9GB ✅

**Medium (can run 2 simultaneously)**:
- gemma2:27b: ~17GB ✅
- qwen2.5-coder:32b: ~20GB ✅
- qwq:32b: ~20GB ✅

**Large (run one at a time)**:
- llama3.3:70b: ~43GB ✅ (74% utilization)
- hermes3:70b: ~50GB ⚠️ (86% utilization - risky)

**Cannot Fit**:
- llama4:maverick: 245GB ❌
- qwen3:235b: 147GB ❌
- deepseek-r1:671b: 404GB ❌

---

## Expected Benchmark Results

### Prediction Matrix

Based on model architectures and specializations:

| Model | Simple (3 words) | Navigation (JSON) | Reasoning (obstacle) | Avg Speed |
|-------|------------------|-------------------|----------------------|-----------|
| **phi4:14b** | 100 | 92 | 88 | ~80 tok/s |
| **qwen2.5-coder:32b** | 100 | **98** ⭐ | 87 | ~35 tok/s |
| **qwq:32b** | 100 | 90 | **96** ⭐ | ~35 tok/s |
| **llama3.3:70b** | 100 | 95 | 92 | ~18 tok/s |
| **deepseek-r1:7b** | 95 | 85 | 92 | ~120 tok/s |

**Key Insights**:
- **Simple prompts**: All models should score 95-100 (trivial task)
- **Navigation**: qwen2.5-coder should dominate (JSON specialist)
- **Reasoning**: qwq should lead (reasoning specialist)
- **Speed**: Size is king (7B > 14B > 32B > 70B)

---

## Decision Framework

### Use Case: Navigation-Heavy Missions (Most Likely)

**Recommendation**: `qwen2.5-coder:32b` primary, `phi4:14b` backup

**Rationale**:
- Navigation plans are the primary workload
- JSON generation quality is critical (invalid JSON = failed mission)
- qwen2.5-coder is purpose-built for structured output
- phi4 provides fast fallback (2.5x speed) with acceptable quality

**Configuration**:
```bash
PRIMARY_MODEL="qwen2.5-coder:32b"
BACKUP_MODEL="phi4:14b"
```

---

### Use Case: Complex Reasoning Missions

**Recommendation**: `qwq:32b` primary, `deepseek-r1:7b` backup

**Rationale**:
- Multi-step logical planning is primary challenge
- Obstacle avoidance requires spatial reasoning
- qwq is reasoning-focused, should excel at "which side of doorway" scenarios
- deepseek-r1 offers ultra-fast reasoning for iterative planning

**Configuration**:
```bash
PRIMARY_MODEL="qwq:32b"
BACKUP_MODEL="deepseek-r1:7b"
```

---

### Use Case: Development/Testing

**Recommendation**: `phi4:14b` primary, `deepseek-r1:7b` optional

**Rationale**:
- Dev iterations need fast responses
- Quality is "good enough" for testing
- Can iterate 2-3x faster than production models
- deepseek-r1 for ultra-fast reasoning tests

**Configuration**:
```bash
PRIMARY_MODEL="phi4:14b"
BACKUP_MODEL="deepseek-r1:7b"
```

---

### Use Case: Production Quality (Regardless of Speed)

**Recommendation**: `llama3.3:70b` primary, `qwen2.5-coder:32b` backup

**Rationale**:
- Largest model = highest quality (generalist)
- Speed is acceptable for mission planning (not real-time)
- qwen-coder as backup reduces RAM to 20GB if needed

**Configuration**:
```bash
PRIMARY_MODEL="llama3.3:70b"
BACKUP_MODEL="qwen2.5-coder:32b"
```

---

## What Makes These Different from llama3.1?

### Current Baseline (llama3.1:8b / llama3.1:70b)
- **Type**: General-purpose instruction-following models
- **Strengths**: Broad capabilities, proven in production
- **Weaknesses**: Not specialized, older architecture (released 2024)

### Why Alternatives May Be Better

#### **Specialization**
- **qwen2.5-coder**: Trained on code/structured data → better JSON
- **qwq**: Trained with reasoning chains → better planning
- **hermes3**: Trained for function calling → better tool use

#### **Efficiency**
- **phi4**: 1/5 the size of 70B, similar quality (new architecture)
- **deepseek-r1**: 1/10 the size, specialized reasoning

#### **Modern Architecture**
- **llama3.3**: Improved over 3.1 (released 2025)
- **qwen2.5/qwq**: 2025 releases with 128K context, 18T tokens training
- **deepseek-r1**: Chain-of-thought prompting, O3-level reasoning

#### **Training Data**
- **qwen2.5-coder**: Massive code corpus (better at JSON/structure)
- **phi4**: Textbook-quality data (better at reasoning despite size)

---

## Next Steps: Running the Benchmark

### 1. Pull Models

On Thor, pull the Tier 1 + Tier 2 models (Tier 3 is optional):

```bash
# SSH to Thor
ssh thor

# Pull models (will take 15-30 minutes total)
docker exec ollama ollama pull phi4:14b          # ~9GB download
docker exec ollama ollama pull qwen2.5-coder:32b # ~20GB download
docker exec ollama ollama pull qwq:32b           # ~20GB download
docker exec ollama ollama pull llama3.3:70b      # ~43GB download
docker exec ollama ollama pull deepseek-r1:7b    # ~5GB download

# Optional Tier 3
# docker exec ollama ollama pull hermes3:70b     # ~50GB download
# docker exec ollama ollama pull gemma2:27b      # ~17GB download
```

### 2. Run Benchmark

```bash
cd ~/shadowhound
./scripts/benchmark_ollama_models.sh
```

**Expected Duration**: 15-25 minutes (5 models × 3 prompts × ~60s each)

### 3. Analyze Results

The script will output:
- Performance summary (speed, tokens/sec, TTFT)
- Quality scores (0-100 per task)
- Recommendations based on speed vs quality tradeoffs

Look for:
- ✅ Quality scores >90 on navigation prompts (critical)
- ✅ Quality scores >85 on reasoning prompts (important)
- ⚖️ Speed tradeoffs (2x slower for 10% better quality = good deal)

### 4. Make Data-Driven Decision

Based on actual results:

```python
# Pseudo-logic for decision
if qwen_coder_nav_quality > 95 and qwen_coder_speed > 30:
    PRIMARY = "qwen2.5-coder:32b"  # JSON specialist wins
elif qwq_reasoning > 95 and missions_are_reasoning_heavy:
    PRIMARY = "qwq:32b"  # Reasoning specialist wins
elif phi4_quality > 90 and speed_is_critical:
    PRIMARY = "phi4:14b"  # Speed champion wins
else:
    PRIMARY = "llama3.3:70b"  # Safe generalist choice

if PRIMARY == "qwen2.5-coder:32b" or PRIMARY == "qwq:32b":
    BACKUP = "phi4:14b"  # Fast backup for 32B primary
else:
    BACKUP = "qwen2.5-coder:32b"  # Quality backup for speed primary
```

### 5. Update Configuration

Edit `scripts/setup_ollama_thor.sh`:

```bash
# Before (baseline)
PRIMARY_MODEL="llama3.1:70b"
BACKUP_MODEL="llama3.1:8b"

# After (data-driven choice)
PRIMARY_MODEL="qwen2.5-coder:32b"  # Or winner from benchmark
BACKUP_MODEL="phi4:14b"
```

---

## Research Sources

### Model Information
- **Ollama Library**: https://ollama.com/library (100+ models with specs)
- **Ollama GitHub**: https://github.com/ollama/ollama (154k stars, active)
- **Model Cards**: Individual model pages on Ollama library

### Key Models Investigated

**Reasoning Specialists**:
- deepseek-r1 (1.5b-671b): "Reasoning approaching O3", 65.2M pulls
- qwq (32b): "Reasoning model of Qwen series", 1.7M pulls
- openthinker (7b-32b): "Distilled from DeepSeek-R1", 601K pulls

**Coding Specialists**:
- qwen2.5-coder (0.5b-32b): "Code generation, reasoning, fixing", 7.5M pulls
- qwen3-coder (30b-480b): "Agentic and coding tasks", 471K pulls
- deepseek-coder-v2 (16b-236b): "GPT4-Turbo comparable", 1.1M pulls

**General Purpose**:
- llama3.3 (70b): "Similar to llama3.1:405b", 2.6M pulls
- qwen2.5 (0.5b-72b): "18T tokens, 128K context", 14.8M pulls
- phi4 (14b): "Microsoft state-of-the-art", 5.3M pulls
- gemma2 (2b-27b): "High-performing, efficient", 8.1M pulls

**Function Calling**:
- hermes3 (3b-405b): "Tool-based use cases", 339K pulls
- granite3.1-dense (2b-8b): "RAG and tool support", 121K pulls

### Selection Methodology
1. **Memory filtering**: Eliminated models >60GB (Thor constraint)
2. **Popularity filtering**: Prioritized models with >1M pulls (proven)
3. **Specialization matching**: Selected coding/reasoning specialists for robot control
4. **Size diversity**: Covered 7B-70B range for speed vs quality comparison
5. **Community validation**: Checked GitHub stars, pull counts, recent activity

---

## Appendix: Full Model Landscape

For reference, here are other notable models that **didn't make the cut** and why:

### Too Large for Thor (>60GB)
- llama4:maverick (400B): 245GB RAM required ❌
- deepseek-r1:671b: 404GB RAM required ❌
- qwen3:235b: 147GB RAM required ❌
- deepseek-v3:671b: 404GB RAM required ❌

### Too Small (Insufficient Quality)
- smollm2 (135m-1.7b): Compact but limited capability ⚠️
- gemma3:1b: Tiny, good for edge but not robot control ⚠️

### Not Specialized for Use Case
- mistral (7b): Good general model, but beaten by phi4 ⚠️
- llava (7b-34b): Vision-focused, but we don't need VLM yet ⚠️
- codellama (7b): Older coding model, beaten by qwen2.5-coder ⚠️

### Redundant with Better Options
- qwen3 (various): Newer, but qwen2.5-coder more specialized ⚠️
- llama3.1:8b: Original baseline, but phi4 likely better ⚠️
- gemma3 (various): Good, but covered by gemma2:27b ⚠️

### Experimental/Unproven
- cogito (3b-70b): Interesting hybrid, but only 548K pulls ⚠️
- openthinker (7b-32b): Promising, but beaten by qwq ⚠️

---

## Changelog

### 2025-01-10 - Initial Selection
- Researched 100+ Ollama models
- Selected 5 primary + 2 optional candidates
- Documented selection rationale
- Created testing matrix and decision framework

---

**Next Update**: After benchmark results are available, update with actual performance data and final recommendation.
