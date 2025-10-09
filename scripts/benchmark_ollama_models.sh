#!/bin/bash
# Benchmark Ollama Models for ShadowHound
# Tests multiple model sizes to find the best quality/speed tradeoff
# Run this ON THOR before committing to a model for production

set -e

echo "=========================================="
echo "ShadowHound Ollama Model Benchmarking"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

# Configuration
OLLAMA_HOST="${OLLAMA_HOST:-http://localhost:11434}"
RESULTS_FILE="ollama_benchmark_results_$(date +%Y%m%d_%H%M%S).json"
RESULTS_DIR="${HOME}/ollama_benchmarks"

# Models to test (in order of size)
declare -a MODELS=(
    "llama3.1:8b"
    "llama3.1:70b"
)

# Test prompts of varying complexity
declare -A TEST_PROMPTS=(
    ["simple"]="Say hello in exactly 3 words"
    ["navigation"]="Generate a JSON plan with 3 steps to explore a laboratory: step 1 should rotate the robot, step 2 should move forward 2 meters, step 3 should take a photo. Return only valid JSON with fields: steps (array of {action, parameters})"
    ["reasoning"]="A robot needs to navigate through a doorway that is 0.8m wide. The robot is 0.6m wide. The robot detects an obstacle 0.3m to the left of the doorway center. Should the robot: A) Pass through center, B) Pass on the right side, C) Pass on the left side, or D) Find another route? Explain reasoning in 2-3 sentences."
)

mkdir -p "$RESULTS_DIR"

echo -e "${BLUE}Configuration:${NC}"
echo "  Ollama Host: $OLLAMA_HOST"
echo "  Results Dir: $RESULTS_DIR"
echo "  Models to test: ${MODELS[@]}"
echo ""

# Check if Ollama is running
echo -e "${YELLOW}Checking Ollama connection...${NC}"
if ! curl -s --max-time 5 "$OLLAMA_HOST/api/tags" > /dev/null; then
    echo -e "${RED}Error: Cannot connect to Ollama at $OLLAMA_HOST${NC}"
    echo "Make sure Ollama container is running:"
    echo "  docker ps | grep ollama"
    exit 1
fi
echo -e "${GREEN}✓ Connected to Ollama${NC}"
echo ""

# Function to format bytes to human readable
format_bytes() {
    local bytes=$1
    if [ $bytes -lt 1024 ]; then
        echo "${bytes}B"
    elif [ $bytes -lt 1048576 ]; then
        echo "$(( bytes / 1024 ))KB"
    elif [ $bytes -lt 1073741824 ]; then
        echo "$(( bytes / 1048576 ))MB"
    else
        echo "$(( bytes / 1073741824 ))GB"
    fi
}

# Function to test a single model with a prompt
benchmark_prompt() {
    local model=$1
    local prompt_name=$2
    local prompt_text=$3
    
    echo -e "${BLUE}  Testing: $prompt_name${NC}"
    
    # Prepare request
    local request_json=$(cat <<EOF
{
  "model": "$model",
  "prompt": "$prompt_text",
  "stream": false,
  "options": {
    "temperature": 0.7,
    "num_predict": 200
  }
}
EOF
)
    
    # Make request and measure time
    local start_time=$(date +%s.%N)
    local response=$(curl -s --max-time 120 -X POST "$OLLAMA_HOST/api/generate" \
        -H "Content-Type: application/json" \
        -d "$request_json")
    local end_time=$(date +%s.%N)
    
    # Calculate duration
    local duration=$(echo "$end_time - $start_time" | bc)
    
    # Parse response
    if echo "$response" | jq -e . >/dev/null 2>&1; then
        local response_text=$(echo "$response" | jq -r '.response // empty')
        local eval_count=$(echo "$response" | jq -r '.eval_count // 0')
        local eval_duration=$(echo "$response" | jq -r '.eval_duration // 0')
        local prompt_eval_count=$(echo "$response" | jq -r '.prompt_eval_count // 0')
        local prompt_eval_duration=$(echo "$response" | jq -r '.prompt_eval_duration // 0')
        
        # Calculate tokens per second
        local tokens_per_sec=0
        if [ "$eval_duration" -gt 0 ]; then
            tokens_per_sec=$(echo "scale=2; $eval_count / ($eval_duration / 1000000000)" | bc)
        fi
        
        # Calculate time to first token
        local ttft=0
        if [ "$prompt_eval_duration" -gt 0 ]; then
            ttft=$(echo "scale=3; $prompt_eval_duration / 1000000000" | bc)
        fi
        
        echo -e "    Duration: ${duration}s | Tokens: $eval_count | Speed: ${tokens_per_sec} tok/s | TTFT: ${ttft}s"
        
        # Return JSON result
        cat <<EOF
{
  "prompt_name": "$prompt_name",
  "duration_seconds": $duration,
  "tokens_generated": $eval_count,
  "tokens_per_second": $tokens_per_sec,
  "time_to_first_token": $ttft,
  "response_preview": "$(echo "$response_text" | head -c 100 | tr '\n' ' ')"
}
EOF
    else
        echo -e "    ${RED}Error: Invalid response${NC}"
        echo "{\"prompt_name\": \"$prompt_name\", \"error\": \"Invalid response\"}"
    fi
}

# Function to get model info
get_model_info() {
    local model=$1
    
    # Get model details from Ollama
    local model_info=$(curl -s "$OLLAMA_HOST/api/show" -d "{\"name\": \"$model\"}")
    
    if echo "$model_info" | jq -e . >/dev/null 2>&1; then
        local size=$(echo "$model_info" | jq -r '.size // 0')
        local param_size=$(echo "$model_info" | jq -r '.details.parameter_size // "unknown"')
        local family=$(echo "$model_info" | jq -r '.details.family // "unknown"')
        
        echo "Size: $(format_bytes $size) | Parameters: $param_size | Family: $family"
        echo "$size"  # Return size for JSON
    else
        echo "unknown"
        echo "0"
    fi
}

# Start benchmarking
echo -e "${YELLOW}Starting benchmark...${NC}"
echo ""

# Initialize results JSON
echo "[" > "$RESULTS_DIR/$RESULTS_FILE"
first_model=true

for model in "${MODELS[@]}"; do
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Testing: $model${NC}"
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Check if model exists, pull if not
    if ! curl -s "$OLLAMA_HOST/api/tags" | jq -r '.models[].name' | grep -q "^$model$"; then
        echo -e "${YELLOW}Model not found locally. Pulling $model...${NC}"
        echo "This may take several minutes..."
        
        docker exec ollama ollama pull "$model"
        
        if [ $? -ne 0 ]; then
            echo -e "${RED}Failed to pull $model. Skipping...${NC}"
            continue
        fi
    else
        echo -e "${GREEN}✓ Model already available${NC}"
    fi
    
    # Get model info
    echo -e "${BLUE}Model Info:${NC}"
    model_size=$(get_model_info "$model")
    echo ""
    
    # Warm up model (first request is always slower)
    echo -e "${YELLOW}Warming up model...${NC}"
    curl -s "$OLLAMA_HOST/api/generate" -d "{\"model\": \"$model\", \"prompt\": \"Hi\", \"stream\": false}" > /dev/null
    echo -e "${GREEN}✓ Model loaded${NC}"
    echo ""
    
    # Add model entry to results (comma separator for JSON array)
    if [ "$first_model" = false ]; then
        echo "," >> "$RESULTS_DIR/$RESULTS_FILE"
    fi
    first_model=false
    
    echo "  {" >> "$RESULTS_DIR/$RESULTS_FILE"
    echo "    \"model\": \"$model\"," >> "$RESULTS_DIR/$RESULTS_FILE"
    echo "    \"timestamp\": \"$(date -u +%Y-%m-%dT%H:%M:%SZ)\"," >> "$RESULTS_DIR/$RESULTS_FILE"
    echo "    \"model_size_bytes\": $model_size," >> "$RESULTS_DIR/$RESULTS_FILE"
    echo "    \"tests\": [" >> "$RESULTS_DIR/$RESULTS_FILE"
    
    # Run tests for each prompt
    first_test=true
    for prompt_name in "${!TEST_PROMPTS[@]}"; do
        prompt_text="${TEST_PROMPTS[$prompt_name]}"
        
        if [ "$first_test" = false ]; then
            echo "," >> "$RESULTS_DIR/$RESULTS_FILE"
        fi
        first_test=false
        
        result=$(benchmark_prompt "$model" "$prompt_name" "$prompt_text")
        echo "      $result" >> "$RESULTS_DIR/$RESULTS_FILE"
    done
    
    echo "" >> "$RESULTS_DIR/$RESULTS_FILE"
    echo "    ]" >> "$RESULTS_DIR/$RESULTS_FILE"
    echo -n "  }" >> "$RESULTS_DIR/$RESULTS_FILE"
    
    echo ""
done

# Close JSON array
echo "" >> "$RESULTS_DIR/$RESULTS_FILE"
echo "]" >> "$RESULTS_DIR/$RESULTS_FILE"

echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}Benchmark Complete!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Generate summary report
echo -e "${BLUE}Generating summary report...${NC}"

python3 - <<PYTHON_SCRIPT
import json
import sys
from pathlib import Path

results_file = "$RESULTS_DIR/$RESULTS_FILE"

with open(results_file, 'r') as f:
    data = json.load(f)

print("\n" + "="*60)
print("BENCHMARK SUMMARY")
print("="*60)

for model_data in data:
    model = model_data['model']
    tests = model_data['tests']
    
    # Calculate averages
    total_duration = sum(t.get('duration_seconds', 0) for t in tests if 'duration_seconds' in t)
    total_tokens = sum(t.get('tokens_generated', 0) for t in tests if 'tokens_generated' in t)
    avg_speed = sum(t.get('tokens_per_second', 0) for t in tests if 'tokens_per_second' in t) / len(tests)
    avg_ttft = sum(t.get('time_to_first_token', 0) for t in tests if 'time_to_first_token' in t) / len(tests)
    
    print(f"\n{model}")
    print("-" * 60)
    print(f"  Total Duration:      {total_duration:.2f}s")
    print(f"  Total Tokens:        {total_tokens}")
    print(f"  Avg Speed:           {avg_speed:.1f} tokens/sec")
    print(f"  Avg Time to First:   {avg_ttft:.3f}s")
    
    print(f"\n  Performance by Task:")
    for test in tests:
        name = test.get('prompt_name', 'unknown')
        duration = test.get('duration_seconds', 0)
        speed = test.get('tokens_per_second', 0)
        print(f"    {name:15} {duration:6.2f}s  |  {speed:5.1f} tok/s")

# Recommendations
print("\n" + "="*60)
print("RECOMMENDATIONS")
print("="*60)

# Find fastest and best quality
speeds = [(m['model'], sum(t.get('tokens_per_second', 0) for t in m['tests']) / len(m['tests'])) 
          for m in data if len(m['tests']) > 0]
speeds.sort(key=lambda x: x[1], reverse=True)

if len(speeds) >= 2:
    print(f"\n🚀 Fastest Model:       {speeds[0][0]} ({speeds[0][1]:.1f} tok/s)")
    print(f"🎯 Quality Model:       {speeds[-1][0]} (slower but more capable)")
    print(f"\n💡 Recommendation:")
    
    # Simple heuristic: if 70B is <3x slower than 8B, recommend it
    if '70b' in speeds[-1][0] and '8b' in speeds[0][0]:
        ratio = speeds[0][1] / speeds[-1][1]
        if ratio < 3:
            print(f"   Use {speeds[-1][0]} - Only {ratio:.1f}x slower but much better quality!")
        else:
            print(f"   Use {speeds[0][0]} for development, {speeds[-1][0]} for production")

print("\n" + "="*60)
print(f"Full results saved to: {results_file}")
print("="*60 + "\n")

PYTHON_SCRIPT

echo ""
echo -e "${GREEN}✓ Benchmark results saved to: $RESULTS_DIR/$RESULTS_FILE${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "  1. Review the summary above"
echo "  2. Choose model based on your speed/quality requirements"
echo "  3. Update PRIMARY_MODEL in setup_ollama_thor.sh"
echo "  4. Or set OLLAMA_MODEL environment variable when launching"
echo ""
