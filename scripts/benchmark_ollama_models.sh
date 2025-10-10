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
UNLOAD_BETWEEN_MODELS="${UNLOAD_BETWEEN_MODELS:-true}"  # Unload models to prevent memory buildup
RESTART_ON_LARGE_MODELS="${RESTART_ON_LARGE_MODELS:-true}"  # Restart container before testing large models (>40GB)

# Models to test (in order of size)
# Based on research for robot control: JSON generation, planning, reasoning
# See docs/OLLAMA_MODEL_COMPARISON.md for selection rationale
declare -a MODELS=(
    # Tier 1: Primary Testing Candidates
    "phi4:14b"              # Speed champion - Microsoft SOTA, ~9GB RAM
    "qwen2.5-coder:32b"     # JSON/coding specialist - Best structured output, ~20GB RAM
    "qwq:32b"               # Reasoning specialist - Purpose-built for complex planning, ~20GB RAM
    
    # Tier 2: Validation & Comparison
    "llama3.3:70b"          # Latest Meta release - Upgrade from 3.1, ~43GB RAM
    "deepseek-r1:7b"        # Experimental reasoning - Fast, cutting-edge, ~5GB RAM
    
    # Tier 3: Optional (comment out to speed up benchmarking)
    # "hermes3:70b"         # Tool-use specialist - Future skills API expansion, ~50GB RAM
    # "gemma2:27b"          # Google efficient model - Middle ground, ~17GB RAM
    # "llama3.1:8b"         # Original baseline - For comparison, ~5GB RAM
    # "llama3.1:70b"        # Original baseline - For comparison, ~43GB RAM
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
echo "  Unload between models: $UNLOAD_BETWEEN_MODELS"
echo "  Restart for large models: $RESTART_ON_LARGE_MODELS"
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

# Function to check container memory usage
check_memory_usage() {
    if command -v docker &> /dev/null; then
        local mem_usage=$(docker stats ollama --no-stream --format "{{.MemUsage}}" 2>/dev/null | cut -d'/' -f1 | tr -d 'GiB' | tr -d ' ' || echo "0")
        echo "$mem_usage"
    else
        echo "0"
    fi
}

# Function to unload a specific model from memory
unload_model() {
    local model=$1
    echo -e "${YELLOW}  Unloading $model from memory...${NC}" >&2
    
    # Send an empty prompt to explicitly stop the model
    curl -s --max-time 10 -X POST "$OLLAMA_HOST/api/generate" \
        -H "Content-Type: application/json" \
        -d "{\"model\": \"$model\", \"prompt\": \"\", \"keep_alive\": 0}" > /dev/null 2>&1
    
    sleep 2
    echo -e "${GREEN}  ✓ Model unloaded${NC}" >&2
}

# Function to restart Ollama container (for large models or memory issues)
restart_ollama_container() {
    if ! command -v docker &> /dev/null; then
        echo -e "${YELLOW}  Docker not available, skipping restart${NC}" >&2
        return 1
    fi
    
    echo -e "${YELLOW}  Restarting Ollama container to clear memory...${NC}" >&2
    
    docker restart ollama > /dev/null 2>&1
    
    if [ $? -eq 0 ]; then
        echo -e "${YELLOW}  Waiting 15s for Ollama to stabilize...${NC}" >&2
        sleep 15
        
        # Wait for API to be responsive
        local max_attempts=6
        local attempt=0
        while [ $attempt -lt $max_attempts ]; do
            if curl -s --max-time 5 "$OLLAMA_HOST/api/tags" > /dev/null 2>&1; then
                echo -e "${GREEN}  ✓ Container restarted and ready${NC}" >&2
                return 0
            fi
            attempt=$((attempt + 1))
            sleep 5
        done
        
        echo -e "${RED}  ⚠ Container restarted but API not responsive${NC}" >&2
        return 1
    else
        echo -e "${RED}  ⚠ Failed to restart container${NC}" >&2
        return 1
    fi
}

# Function to estimate model size from name
estimate_model_size_gb() {
    local model=$1
    
    # Extract parameter size from model name (e.g., "llama3.3:70b" -> 70)
    if [[ $model =~ :([0-9]+)b ]]; then
        local params="${BASH_REMATCH[1]}"
        # Rough estimate: 0.6-0.7 GB per billion parameters for Q4 quantization
        local size_gb=$(echo "$params * 0.65" | bc)
        echo "${size_gb%.*}"  # Return integer
    else
        echo "0"
    fi
}

# Function to test a single model with a prompt
benchmark_prompt() {
    local model=$1
    local prompt_name=$2
    local prompt_text=$3
    
    echo -e "${BLUE}  Testing: $prompt_name${NC}" >&2
    
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
        
        # Calculate tokens per second (default to 0 if division fails)
        local tokens_per_sec="0"
        if [ "$eval_duration" -gt 0 ] 2>/dev/null; then
            tokens_per_sec=$(echo "scale=2; $eval_count / ($eval_duration / 1000000000)" | bc 2>/dev/null || echo "0")
        fi
        
        # Calculate time to first token (default to 0 if division fails)
        local ttft="0"
        if [ "$prompt_eval_duration" -gt 0 ] 2>/dev/null; then
            ttft=$(echo "scale=3; $prompt_eval_duration / 1000000000" | bc 2>/dev/null || echo "0")
        fi
        
        echo -e "    Duration: ${duration}s | Tokens: $eval_count | Speed: ${tokens_per_sec} tok/s | TTFT: ${ttft}s" >&2
        
        # Calculate quality score (if Python available)
        local quality_score="null"
        local quality_details="null"
        if command -v python3 &> /dev/null; then
            local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
            local quality_json=$(python3 "$script_dir/quality_scorer.py" "$prompt_name" "$prompt_text" "$response_text" 2>/dev/null || echo '{}')
            
            if echo "$quality_json" | jq -e '.overall_score' >/dev/null 2>&1; then
                quality_score=$(echo "$quality_json" | jq -r '.overall_score')
                quality_details=$(echo "$quality_json" | jq -c '{subscores, issues, passed_checks}')
                echo -e "    ${GREEN}Quality Score: ${quality_score}/100${NC}" >&2
            fi
        fi
        
        # Return JSON result using jq for proper escaping (output to stdout only)
        local preview=$(echo "$response_text" | head -c 100 | tr '\n' ' ' | tr -d '\r')
        jq -n \
            --arg name "$prompt_name" \
            --arg duration "$duration" \
            --arg tokens "$eval_count" \
            --arg speed "$tokens_per_sec" \
            --arg ttft "$ttft" \
            --arg preview "$preview" \
            --arg quality_score "$quality_score" \
            --argjson quality_details "$quality_details" \
            '{
                prompt_name: $name,
                duration_seconds: ($duration | tonumber),
                tokens_generated: ($tokens | tonumber),
                tokens_per_second: ($speed | tonumber),
                time_to_first_token: ($ttft | tonumber),
                response_preview: $preview,
                quality_score: (if $quality_score == "null" then null else ($quality_score | tonumber) end),
                quality_details: $quality_details
            }'
    else
        echo -e "    ${RED}Error: Invalid response${NC}" >&2
        jq -n --arg name "$prompt_name" '{prompt_name: $name, error: "Invalid response"}'
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
        
        echo "Size: $(format_bytes $size) | Parameters: $param_size | Family: $family" >&2
        echo "$size"  # Return size for JSON
    else
        echo "unknown" >&2
        echo "0"
    fi
}

# Start benchmarking
echo -e "${YELLOW}Starting benchmark...${NC}"
echo ""

# Create temp directory for test results
TEMP_DIR=$(mktemp -d)
trap "rm -rf $TEMP_DIR" EXIT

for model in "${MODELS[@]}"; do
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Testing: $model${NC}"
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Check memory usage before loading new model
    mem_before=$(check_memory_usage)
    if [ "$mem_before" != "0" ]; then
        echo -e "${BLUE}Container memory before: ${mem_before}GiB${NC}"
    fi
    
    # Estimate model size and decide if restart needed
    estimated_size=$(estimate_model_size_gb "$model")
    if [ "$RESTART_ON_LARGE_MODELS" = "true" ] && [ "$estimated_size" -gt 40 ]; then
        echo -e "${YELLOW}Large model detected (~${estimated_size}GB). Restarting container to ensure clean state...${NC}"
        restart_ollama_container
    fi
    
    # Check if model exists, pull if not
    if ! curl -s "$OLLAMA_HOST/api/tags" | jq -r '.models[].name' | grep -q "^$model$"; then
        echo -e "${YELLOW}Model not found locally. Pulling $model...${NC}"
        echo "This may take several minutes..."
        
        docker exec ollama ollama pull "$model"
        
        if [ $? -ne 0 ]; then
            echo -e "${RED}Failed to pull $model. Skipping...${NC}"
            echo ""
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
    warmup_response=$(curl -s --max-time 60 "$OLLAMA_HOST/api/generate" \
        -d "{\"model\": \"$model\", \"prompt\": \"Hi\", \"stream\": false}" 2>&1)
    
    if echo "$warmup_response" | grep -q "500 Internal Server Error\|EOF"; then
        echo -e "${RED}✗ Failed to load model (memory issue or crash)${NC}"
        echo -e "${RED}  Error: $warmup_response${NC}"
        echo -e "${YELLOW}  Attempting container restart...${NC}"
        
        restart_ollama_container
        
        # Retry warmup after restart
        echo -e "${YELLOW}  Retrying model load...${NC}"
        warmup_response=$(curl -s --max-time 60 "$OLLAMA_HOST/api/generate" \
            -d "{\"model\": \"$model\", \"prompt\": \"Hi\", \"stream\": false}" 2>&1)
        
        if echo "$warmup_response" | grep -q "500 Internal Server Error\|EOF"; then
            echo -e "${RED}✗ Model still fails to load after restart. Skipping...${NC}"
            echo -e "${RED}  This model may be too large for available memory.${NC}"
            echo ""
            continue
        fi
    fi
    
    echo -e "${GREEN}✓ Model loaded${NC}"
    
    # Check memory usage after loading
    mem_after=$(check_memory_usage)
    if [ "$mem_after" != "0" ] && [ "$mem_before" != "0" ]; then
        mem_delta=$(echo "$mem_after - $mem_before" | bc)
        echo -e "${BLUE}Container memory after: ${mem_after}GiB (+${mem_delta}GiB)${NC}"
    fi
    echo ""
    
    # Create file for this model's results
    model_file="$TEMP_DIR/$(echo $model | tr ':/' '__').json"
    
    # Write model metadata
    jq -n \
        --arg model "$model" \
        --arg timestamp "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
        --argjson size "$model_size" \
        --arg mem_before "$mem_before" \
        --arg mem_after "$mem_after" \
        '{model: $model, timestamp: $timestamp, model_size_bytes: $size, memory_before_gib: $mem_before, memory_after_gib: $mem_after, tests: []}' > "$model_file"
    
    # Run tests for each prompt and collect results
    for prompt_name in "${!TEST_PROMPTS[@]}"; do
        prompt_text="${TEST_PROMPTS[$prompt_name]}"
        
        result=$(benchmark_prompt "$model" "$prompt_name" "$prompt_text")
        
        # Add result to model's tests array
        jq --argjson test "$result" '.tests += [$test]' "$model_file" > "$model_file.tmp" && mv "$model_file.tmp" "$model_file"
    done
    
    # Unload model to prevent memory buildup for next test
    if [ "$UNLOAD_BETWEEN_MODELS" = "true" ]; then
        unload_model "$model"
        
        # Check memory after unload
        mem_unload=$(check_memory_usage)
        if [ "$mem_unload" != "0" ]; then
            echo -e "${BLUE}Container memory after unload: ${mem_unload}GiB${NC}"
        fi
    fi
    
    echo ""
done

# Combine all model results into final JSON array
echo -e "${BLUE}Combining results...${NC}"
jq -s '.' $TEMP_DIR/*.json > "$RESULTS_DIR/$RESULTS_FILE" 2>/dev/null || echo "[]" > "$RESULTS_DIR/$RESULTS_FILE"

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
        quality = test.get('quality_score')
        
        if quality is not None:
            print(f"    {name:15} {duration:6.2f}s  |  {speed:5.1f} tok/s  |  Q: {quality:.0f}/100")
        else:
            print(f"    {name:15} {duration:6.2f}s  |  {speed:5.1f} tok/s")

# Recommendations
print("\n" + "="*60)
print("RECOMMENDATIONS")
print("="*60)

# Find fastest and best quality
speeds = [(m['model'], sum(t.get('tokens_per_second', 0) for t in m['tests']) / len(m['tests'])) 
          for m in data if len(m['tests']) > 0]
speeds.sort(key=lambda x: x[1], reverse=True)

# Calculate average quality scores per model
qualities = []
for m in data:
    quality_scores = [t.get('quality_score') for t in m['tests'] if t.get('quality_score') is not None]
    if quality_scores:
        avg_quality = sum(quality_scores) / len(quality_scores)
        qualities.append((m['model'], avg_quality))
qualities.sort(key=lambda x: x[1], reverse=True)

if len(speeds) >= 2:
    print(f"\n🚀 Fastest Model:       {speeds[0][0]} ({speeds[0][1]:.1f} tok/s)")
    
    if qualities:
        print(f"🎯 Best Quality:        {qualities[0][0]} ({qualities[0][1]:.1f}/100)")
        print(f"\n📊 Speed vs Quality Tradeoff:")
        for model, speed in speeds:
            quality_match = next((q for m, q in qualities if m == model), None)
            if quality_match:
                print(f"   {model:20} Speed: {speed:5.1f} tok/s  |  Quality: {quality_match:5.1f}/100")
    else:
        print(f"🎯 Quality Model:       {speeds[-1][0]} (slower but more capable)")
    
    print(f"\n💡 Recommendation:")
    
    # Intelligent recommendation based on speed and quality
    if '70b' in speeds[-1][0] and '8b' in speeds[0][0]:
        ratio = speeds[0][1] / speeds[-1][1]
        
        if qualities:
            quality_8b = next((q for m, q in qualities if '8b' in m), 0)
            quality_70b = next((q for m, q in qualities if '70b' in m), 0)
            quality_diff = quality_70b - quality_8b
            
            if quality_diff > 20 and ratio < 3:
                print(f"   🌟 Use {speeds[-1][0]} - {quality_diff:.0f}pts better quality, only {ratio:.1f}x slower!")
            elif quality_diff > 10 and ratio < 2:
                print(f"   ⚖️  Use {speeds[-1][0]} - Better quality ({quality_diff:.0f}pts), reasonable speed tradeoff ({ratio:.1f}x)")
            elif quality_diff < 10:
                print(f"   ⚡ Use {speeds[0][0]} - Quality difference small ({quality_diff:.0f}pts), much faster")
            else:
                print(f"   🔄 Use {speeds[0][0]} for dev, {speeds[-1][0]} for production")
        else:
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
