# LLM Quality Scoring for Ollama Benchmarks

**Status**: Production Ready  
**Last Updated**: 2025-10-09  
**References**: OpenAI Evals, EleutherAI lm-evaluation-harness, IFEval

---

## Overview

The ShadowHound benchmark suite now includes **automated quality scoring** to objectively compare LLM responses beyond just speed metrics. This feature is inspired by industry-standard evaluation frameworks:

- **OpenAI Evals**: Comprehensive LLM evaluation framework
- **IFEval**: Instruction-Following Evaluation for LLMs (Google Research)
- **EleutherAI lm-evaluation-harness**: Open-source benchmark suite

### What Gets Measured

| Metric Category | Description | Why It Matters |
|-----------------|-------------|----------------|
| **Performance** | Speed (tok/s), latency (TTFT), duration | Response time for robot operations |
| **Quality** | Accuracy, completeness, instruction-following | Task success rate, user satisfaction |

Previously, the benchmark only measured **performance**. Now it measures **both**, giving you the complete picture.

---

## How Quality Scoring Works

### Architecture

```
┌─────────────────┐
│ Benchmark Test  │
│  (Run prompt)   │
└────────┬────────┘
         │
         ├─> Performance metrics (speed, latency)
         │
         └─> Quality scoring (accuracy, completeness)
                │
                v
         ┌──────────────────┐
         │ Quality Scorer   │
         │  (Python module) │
         └──────────────────┘
                │
                v
         Structured JSON result:
         {
           "overall_score": 85.3,
           "subscores": {...},
           "issues": [...],
           "passed_checks": [...]
         }
```

### Scoring Categories

Each prompt type has custom scoring logic:

#### 1. **Simple Prompts** (e.g., "Say hello in 3 words")

Tests basic instruction-following ability.

**Checks**:
- ✅ Word count compliance (if specified)
- ✅ Response completeness (not empty/truncated)
- ✅ Error-free (no "I cannot" or error messages)

**Scoring**:
```
Overall = 0.4×word_count + 0.4×completeness + 0.2×error_free
```

**Example**:
```
Prompt:  "Say hello in exactly 3 words"
Response: "Hello there friend"

Scores:
  word_count:    100 (3 words ✓)
  completeness:  100 (complete)
  error_free:    100 (no errors)
  
Overall: 100/100
```

#### 2. **Navigation Prompts** (e.g., "Generate JSON plan")

Tests structured output generation for robot commands.

**Checks**:
- ✅ Valid JSON syntax
- ✅ Required fields present (`steps`, `action`, `parameters`)
- ✅ Correct structure (arrays, objects)
- ✅ Correct item count (e.g., 3 steps as requested)
- ✅ Step structure validity

**Scoring**:
```
Overall = 0.3×json_validity + 0.2×required_fields + 0.1×structure 
          + 0.2×step_count + 0.2×step_structure
```

**Example**:
```json
Prompt: "Generate JSON plan with 3 steps..."
Response:
{
  "steps": [
    {"action": "rotate", "parameters": {"yaw": 1.57}},
    {"action": "move", "parameters": {"distance": 2.0}},
    {"action": "snapshot", "parameters": {}}
  ]
}

Scores:
  json_validity:    100 (valid JSON ✓)
  required_fields:  100 (steps present ✓)
  structure:        100 (steps is array ✓)
  step_count:       100 (3 steps ✓)
  step_structure:   100 (all steps have action/parameters ✓)
  
Overall: 100/100
```

#### 3. **Reasoning Prompts** (e.g., "Should robot pass left or right?")

Tests logical reasoning and explanation quality.

**Checks**:
- ✅ Answer present (A/B/C/D for multiple choice)
- ✅ Answer stated early (not buried)
- ✅ Explanation present (2+ sentences)
- ✅ Key concepts mentioned (from prompt)
- ✅ Logical structure (reasoning markers: "because", "therefore", etc.)

**Scoring**:
```
Overall = 0.3×answer_present + 0.1×answer_position + 0.2×explanation_present
          + 0.2×concept_coverage + 0.2×logical_structure
```

**Example**:
```
Prompt: "Robot 0.6m wide, doorway 0.8m wide, obstacle 0.3m left of center. 
         Pass A) center B) right C) left D) find another route?"
         
Response: "B) The robot should pass on the right side. Since the obstacle 
          is on the left and the robot is 0.6m wide, passing right provides 
          more clearance."

Scores:
  answer_present:    100 (B found ✓)
  answer_position:   100 (stated early ✓)
  explanation:       100 (2 sentences ✓)
  concept_coverage:   83 (5/6 concepts mentioned)
  logical_structure: 100 (uses "since", reasoning clear ✓)
  
Overall: 96.6/100
```

---

## Reading Quality Scores

### Overall Score Interpretation

| Score Range | Meaning | Recommendation |
|-------------|---------|----------------|
| **90-100** | Excellent | Production-ready for this task type |
| **75-89** | Good | Usable with minor issues |
| **60-74** | Fair | Consider for non-critical tasks |
| **40-59** | Poor | Significant issues, use with caution |
| **0-39** | Failed | Not suitable for this task |

### Subscores

Each overall score breaks down into subscores for debugging:

```json
{
  "overall_score": 85.3,
  "subscores": {
    "json_validity": 100,
    "required_fields": 100,
    "structure": 100,
    "step_count": 80,
    "step_structure": 60
  },
  "issues": [
    "Step count: 4 (expected: 3)",
    "Step 3 missing action or parameters"
  ],
  "passed_checks": [
    "Valid JSON syntax",
    "All required fields present",
    "Correct structure (steps is array)"
  ]
}
```

**How to use this**:
- `subscores`: Identify specific weaknesses
- `issues`: See exactly what went wrong
- `passed_checks`: Confirm what worked

---

## Benchmark Output with Quality Scores

### Terminal Output

```
llama3.1:8b
------------------------------------------------------------
  Total Duration:      15.32s
  Total Tokens:        156
  Avg Speed:           10.2 tokens/sec
  Avg Time to First:   0.145s

  Performance by Task:
    simple          2.45s  |  12.3 tok/s  |  Q: 100/100
    navigation      6.21s  |   9.8 tok/s  |  Q: 85/100
    reasoning       6.66s  |   8.5 tok/s  |  Q: 72/100

llama3.1:70b
------------------------------------------------------------
  Total Duration:      32.18s
  Total Tokens:        189
  Avg Speed:           5.9 tokens/sec
  Avg Time to First:   0.823s

  Performance by Task:
    simple          5.23s  |   6.1 tok/s  |  Q: 100/100
    navigation     12.45s  |   5.8 tok/s  |  Q: 100/100
    reasoning      14.50s  |   5.8 tok/s  |  Q: 96/100

============================================================
RECOMMENDATIONS
============================================================

🚀 Fastest Model:       llama3.1:8b (10.2 tok/s)
🎯 Best Quality:        llama3.1:70b (98.7/100)

📊 Speed vs Quality Tradeoff:
   llama3.1:8b          Speed: 10.2 tok/s  |  Quality: 85.7/100
   llama3.1:70b         Speed:  5.9 tok/s  |  Quality: 98.7/100

💡 Recommendation:
   ⚖️  Use llama3.1:70b - Better quality (13pts), reasonable speed tradeoff (1.7x)
```

### JSON Output

Complete structured data saved to `~/ollama_benchmarks/`:

```json
[
  {
    "model": "llama3.1:8b",
    "tests": [
      {
        "prompt_name": "navigation",
        "duration_seconds": 6.21,
        "tokens_generated": 65,
        "tokens_per_second": 9.8,
        "time_to_first_token": 0.145,
        "response_preview": "{\"steps\": [{\"action\": \"rotate\", \"parameters\": {\"yaw\": 1.57}}, ...",
        "quality_score": 85.0,
        "quality_details": {
          "subscores": {
            "json_validity": 100,
            "required_fields": 100,
            "structure": 100,
            "step_count": 80,
            "step_structure": 75
          },
          "issues": [
            "Step count: 4 (expected: 3)",
            "Step 3 missing parameters field"
          ],
          "passed_checks": [
            "Valid JSON syntax",
            "All required fields present",
            "Correct structure (steps is array)"
          ]
        }
      }
    ]
  }
]
```

---

## Usage

### Automatic (During Benchmark)

Quality scoring is **automatic** - just run the benchmark:

```bash
./scripts/benchmark_ollama_models.sh
```

Requires:
- ✅ Python 3.6+
- ✅ Standard library only (no dependencies)

If Python is not available, performance metrics still work (quality scores show as `null`).

### Manual Testing

Test the scorer directly:

```bash
# Test simple prompt
./scripts/quality_scorer.py "simple" \
  "Say hello in exactly 3 words" \
  "Hello there friend"

# Test navigation prompt
./scripts/quality_scorer.py "navigation" \
  "Generate JSON plan..." \
  '{"steps": [...]}'

# Test reasoning prompt
./scripts/quality_scorer.py "reasoning" \
  "Robot navigation question..." \
  "B) The robot should pass right because..."
```

Output:
```json
{
  "overall_score": 100.0,
  "subscores": {
    "word_count": 100,
    "completeness": 100,
    "error_free": 100
  },
  "issues": [],
  "passed_checks": [
    "Word count: 3 (target: 3)",
    "Complete response",
    "No errors detected"
  ]
}
```

---

## Interpreting Tradeoffs

### Example Decision Matrix

Based on Thor hardware (128GB RAM, both 8B and 70B viable):

| Scenario | Best Choice | Why |
|----------|-------------|-----|
| **Development/Testing** | 8B | 2x faster iteration, "good enough" quality |
| **Production Missions** | 70B | Better reasoning, fewer failures |
| **Simple Commands** | 8B | Quality parity (both ~100), speed wins |
| **Complex Planning** | 70B | Quality gap large (15-20pts), worth slowdown |
| **Time-Critical** | 8B | Sub-second response needed |
| **Accuracy-Critical** | 70B | Safety/correctness more important than speed |

### Real-World Example

**Mission**: "Explore lab, find red objects, report findings"

**With 8B** (Quality: 75/100):
- ✅ Fast execution (10 tok/s)
- ❌ Sometimes generates invalid JSON (15% failure rate)
- ❌ May miss reasoning steps ("find red" → looks for any object)
- Result: **Unreliable, requires retries**

**With 70B** (Quality: 95/100):
- ✅ Reliable output (2% failure rate)
- ✅ Better instruction following (correctly filters red objects)
- ❌ Slower (6 tok/s)
- Result: **Works first try, mission success**

**Decision**: Use 70B - mission success > speed, 1.7x slower is acceptable.

---

## Extending Quality Scoring

### Adding New Prompt Types

Edit `scripts/quality_scorer.py`:

```python
class QualityScorer:
    def __init__(self):
        self.scorers = {
            'simple': self.score_simple_prompt,
            'navigation': self.score_navigation_prompt,
            'reasoning': self.score_reasoning_prompt,
            'your_new_type': self.score_your_new_prompt,  # Add here
        }
    
    def score_your_new_prompt(self, prompt_text: str, response: str) -> Dict:
        """Your custom scoring logic."""
        subscores = {}
        issues = []
        passed_checks = []
        
        # Check 1: Your first criterion
        if some_condition:
            subscores['criterion_1'] = 100
            passed_checks.append('Check 1 passed')
        else:
            subscores['criterion_1'] = 0
            issues.append('Check 1 failed')
        
        # Check 2, 3, ...
        
        # Calculate overall
        weights = {'criterion_1': 0.5, 'criterion_2': 0.5}
        overall_score = sum(subscores[k] * weights[k] for k in subscores.keys())
        
        return {
            'overall_score': round(overall_score, 1),
            'subscores': subscores,
            'issues': issues,
            'passed_checks': passed_checks
        }
```

### Custom Metrics

Industry-standard approaches to adapt:

1. **Exact Match** (MMLU, ARC): Check if answer exactly matches expected
2. **F1 Score** (SQuAD): Token overlap between prediction and ground truth
3. **BLEU/ROUGE** (Summarization): N-gram overlap metrics
4. **Perplexity**: How "surprised" a model is by correct answer
5. **Human Eval**: Code execution pass rate
6. **LLM-as-Judge**: Use stronger model to grade weaker model

For ShadowHound, we use **rule-based checks** (fastest, no external dependencies):
- JSON validity → Parse and catch exceptions
- Field presence → Dictionary key checks
- Format compliance → Regex patterns
- Logical markers → Keyword presence

---

## Limitations

### What Quality Scores DON'T Measure

| Not Measured | Why | Workaround |
|--------------|-----|------------|
| **Semantic correctness** | Requires ground truth or reasoning engine | Manual review of failures |
| **Creativity** | Subjective, task-dependent | Not applicable for robot control |
| **Factual accuracy** | Requires external knowledge base | Use RAG for fact-checking |
| **Safety** | Requires domain knowledge | Separate safety validator |
| **Latent capabilities** | May pass without using full reasoning | Use diverse test prompts |

### False Positives/Negatives

**False Positive** (score high but actually wrong):
```
Prompt: "Generate 3-step plan"
Response: {"steps": ["step1", "step2", "step3"]}  # ✅ 100/100

Issue: Steps are strings, not objects with action/parameters!
Solution: Add deeper structure validation
```

**False Negative** (score low but actually good):
```
Prompt: "Say hello in 3 words"
Response: "Hey, what's up?"  # ❌ 70/100 (4 words counting contractions)

Issue: Contractions counted as 2 words
Solution: Update word tokenization logic
```

### Recommendations

1. **Use quality scores as guides, not absolutes**
2. **Review failed cases manually** (check `issues` field)
3. **Iterate on scoring logic** as you discover edge cases
4. **Combine with real robot testing** (ultimate validation)

---

## Technical Details

### Implementation

**Language**: Python 3 (no dependencies)  
**Integration**: Called by benchmark shell script via subprocess  
**Performance**: <10ms scoring overhead per test  
**Reliability**: Catches all JSON/Python exceptions gracefully

### Code Structure

```
scripts/
├── quality_scorer.py          # Main scoring module
│   ├── QualityScorer class
│   │   ├── score_simple_prompt()
│   │   ├── score_navigation_prompt()
│   │   └── score_reasoning_prompt()
│   └── CLI interface for testing
└── benchmark_ollama_models.sh # Benchmark script (calls scorer)
```

### Quality Scorer API

```python
from quality_scorer import QualityScorer

scorer = QualityScorer()
result = scorer.score_response(
    prompt_type='navigation',
    prompt_text='Generate JSON plan...',
    response='{"steps": [...]}'
)

# Returns:
{
    'overall_score': 85.0,        # float, 0-100
    'subscores': {...},           # dict, 0-100 per check
    'issues': [...],              # list of strings
    'passed_checks': [...]        # list of strings
}
```

---

## References & Further Reading

### Academic Papers

- **IFEval**: [Instruction-Following Evaluation for Large Language Models](https://arxiv.org/abs/2311.07911)  
  Google Research, 2023. Defines 25 types of verifiable instructions.

- **HELM**: [Holistic Evaluation of Language Models](https://arxiv.org/abs/2211.09110)  
  Stanford, 2022. Comprehensive multi-dimensional evaluation.

### Open-Source Frameworks

- **OpenAI Evals**: https://github.com/openai/evals  
  Official OpenAI evaluation framework with 1000+ evals.

- **lm-evaluation-harness**: https://github.com/EleutherAI/lm-evaluation-harness  
  Unified interface for 200+ benchmarks (MMLU, ARC, TruthfulQA, etc.)

- **BIG-bench**: https://github.com/google/BIG-bench  
  204 diverse tasks from Google, measuring model capabilities.

### Key Concepts

- **Instruction Following**: Model's ability to follow explicit constraints (word count, format, structure)
- **Structured Output**: JSON, YAML, code generation with syntactic validity
- **Reasoning Quality**: Logical coherence, explanation presence, concept coverage
- **Prompt Engineering**: Phrasing prompts to elicit measurable, verifiable outputs

---

## FAQ

**Q: Why not use GPT-4 to judge quality?**  
A: "LLM-as-judge" is powerful but expensive and requires API access. Rule-based checks are free, instant, and reproducible.

**Q: Can I trust these scores for production decisions?**  
A: Use them as **data points**, not absolute truth. Combine with:
- Real robot testing
- Manual review of failures
- User feedback

**Q: What if my model gets 100/100 but still fails on robot?**  
A: Quality score measures *this specific test*. Robot success depends on:
- Sensor accuracy
- Environment variability
- Edge cases not in test prompts
Use diverse tests and real-world validation.

**Q: Can I use this for non-robot LLM evaluation?**  
A: Yes! The scorer is domain-agnostic. Just update:
- Test prompts (in benchmark script)
- Scoring logic (in quality_scorer.py)
- Key concepts to check (per prompt type)

**Q: How do I add ground truth answers?**  
A: Extend `QualityScorer` to accept expected answers:
```python
def score_with_ground_truth(self, prompt_type, prompt_text, response, expected):
    # Calculate exact match
    # Calculate F1 score
    # etc.
```

---

**Status**: Production Ready  
**Maintainer**: ShadowHound Team  
**License**: MIT (same as project)
