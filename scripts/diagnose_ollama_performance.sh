#!/bin/bash
# Quick Ollama Performance Diagnostic
# Checks why speeds might be slow

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "============================================================"
echo "Ollama Performance Diagnostic"
echo "============================================================"
echo ""

OLLAMA_HOST="${OLLAMA_HOST:-http://localhost:11434}"

# 1. Check if Ollama is running
echo -e "${BLUE}1. Checking Ollama service...${NC}"
if curl -s --max-time 5 "$OLLAMA_HOST/api/tags" > /dev/null; then
    echo -e "${GREEN}✓ Ollama responding at $OLLAMA_HOST${NC}"
else
    echo -e "${RED}✗ Cannot reach Ollama at $OLLAMA_HOST${NC}"
    exit 1
fi
echo ""

# 2. Check GPU availability
echo -e "${BLUE}2. Checking GPU acceleration...${NC}"
GPU_AVAILABLE=false

if command -v nvidia-smi &> /dev/null; then
    if nvidia-smi -L 2>/dev/null | grep -q "GPU"; then
        GPU_AVAILABLE=true
        echo -e "${GREEN}✓ NVIDIA GPU detected${NC}"
        nvidia-smi --query-gpu=index,name,memory.total,driver_version,compute_mode --format=csv,noheader | while IFS=, read -r idx name memory driver compute; do
            echo "  GPU $idx: $name"
            echo "    Memory: $memory"
            echo "    Driver: $driver"
            echo "    Compute Mode: $compute"
        done
        echo ""
        
        # Check GPU utilization
        echo "  Current GPU utilization:"
        nvidia-smi --query-gpu=utilization.gpu,utilization.memory,temperature.gpu --format=csv,noheader,nounits | while IFS=, read -r gpu_util mem_util temp; do
            echo "    GPU Load: ${gpu_util}%  |  Memory: ${mem_util}%  |  Temp: ${temp}°C"
        done
    else
        echo -e "${YELLOW}⚠️  nvidia-smi found but no GPU detected${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  nvidia-smi not found${NC}"
fi

if ! $GPU_AVAILABLE; then
    echo -e "${RED}✗ No GPU acceleration available${NC}"
    echo -e "${YELLOW}  Running on CPU - expect 5-10x slower speeds${NC}"
    echo ""
    echo "  Typical speeds:"
    echo "    GPU: qwen2.5-coder:32b ~4-5 tok/s"
    echo "    CPU: qwen2.5-coder:32b ~0.5-1 tok/s"
fi
echo ""

# 3. Check if Docker container has GPU access (if using Docker)
if command -v docker &> /dev/null; then
    echo -e "${BLUE}3. Checking Docker GPU access...${NC}"
    
    if docker ps 2>/dev/null | grep -q ollama; then
        CONTAINER_NAME=$(docker ps --format "{{.Names}}" | grep ollama | head -1)
        echo "  Container: $CONTAINER_NAME"
        
        # Check if container has GPU devices
        if docker inspect "$CONTAINER_NAME" 2>/dev/null | grep -q "DeviceRequests"; then
            echo -e "${GREEN}✓ Container has GPU device requests${NC}"
            
            # Check nvidia runtime
            if docker inspect "$CONTAINER_NAME" 2>/dev/null | grep -q "nvidia"; then
                echo -e "${GREEN}✓ NVIDIA runtime configured${NC}"
            fi
        else
            echo -e "${RED}✗ Container does NOT have GPU access${NC}"
            echo ""
            echo "  To fix, restart container with GPU:"
            echo "    docker stop $CONTAINER_NAME"
            echo "    docker run -d --gpus all -v ollama:/root/.ollama -p 11434:11434 --name ollama ollama/ollama"
        fi
        
        # Check container resource usage
        echo ""
        echo "  Container resources:"
        docker stats "$CONTAINER_NAME" --no-stream --format "table {{.MemUsage}}\t{{.CPUPerc}}"
    else
        echo "  Ollama not running in Docker (native install or remote)"
    fi
else
    echo -e "${BLUE}3. Docker not available${NC}"
fi
echo ""

# 4. Test actual inference speed
echo -e "${BLUE}4. Running speed test...${NC}"
TEST_MODEL="${TEST_MODEL:-qwen2.5-coder:32b}"
echo "  Model: $TEST_MODEL"
echo "  Testing with simple prompt..."
echo ""

START_TIME=$(date +%s%N)
RESPONSE=$(curl -s --max-time 30 "$OLLAMA_HOST/api/generate" \
    -d "{\"model\": \"$TEST_MODEL\", \"prompt\": \"Count to 10\", \"stream\": false}" 2>/dev/null)
END_TIME=$(date +%s%N)

if echo "$RESPONSE" | jq -e . >/dev/null 2>&1; then
    EVAL_COUNT=$(echo "$RESPONSE" | jq -r '.eval_count // 0')
    EVAL_DURATION=$(echo "$RESPONSE" | jq -r '.eval_duration // 0')
    PROMPT_EVAL_DURATION=$(echo "$RESPONSE" | jq -r '.prompt_eval_duration // 0')
    
    WALL_TIME=$(echo "scale=2; ($END_TIME - $START_TIME) / 1000000000" | bc)
    
    if [ "$EVAL_DURATION" -gt 0 ] 2>/dev/null; then
        TOK_PER_SEC=$(echo "scale=2; $EVAL_COUNT / ($EVAL_DURATION / 1000000000)" | bc 2>/dev/null || echo "0")
        
        echo "  Results:"
        echo "    Tokens generated: $EVAL_COUNT"
        echo "    Speed: $TOK_PER_SEC tok/s"
        echo "    Total time: ${WALL_TIME}s"
        
        if [ -n "$PROMPT_EVAL_DURATION" ] && [ "$PROMPT_EVAL_DURATION" -gt 0 ] 2>/dev/null; then
            TTFT=$(echo "scale=3; $PROMPT_EVAL_DURATION / 1000000000" | bc)
            echo "    Time to first token: ${TTFT}s"
        fi
        echo ""
        
        # Analyze speed
        SPEED_FLOAT=$(echo "$TOK_PER_SEC" | bc -l)
        if (( $(echo "$SPEED_FLOAT < 1.0" | bc -l) )); then
            echo -e "${RED}⚠️  VERY SLOW - likely CPU-only inference${NC}"
            echo "    Expected GPU speed for $TEST_MODEL: ~4-5 tok/s"
            echo "    Your speed: $TOK_PER_SEC tok/s"
            echo "    Slowdown: ~$(echo "scale=0; 4.0 / $SPEED_FLOAT" | bc)x"
        elif (( $(echo "$SPEED_FLOAT < 2.0" | bc -l) )); then
            echo -e "${YELLOW}⚠️  Slower than expected${NC}"
            echo "    Expected GPU speed for $TEST_MODEL: ~4-5 tok/s"
            echo "    Your speed: $TOK_PER_SEC tok/s"
        elif (( $(echo "$SPEED_FLOAT < 3.5" | bc -l) )); then
            echo -e "${YELLOW}ℹ️  Below optimal but acceptable${NC}"
            echo "    Expected GPU speed for $TEST_MODEL: ~4-5 tok/s"
        else
            echo -e "${GREEN}✓ Good speed - GPU acceleration working${NC}"
        fi
    else
        echo -e "${RED}✗ Invalid response or timeout${NC}"
    fi
else
    echo -e "${RED}✗ Failed to get valid response${NC}"
    echo "  Check if model is loaded: curl $OLLAMA_HOST/api/tags"
fi
echo ""

# 5. Summary and recommendations
echo "============================================================"
echo "SUMMARY"
echo "============================================================"
echo ""

if $GPU_AVAILABLE; then
    if command -v docker &> /dev/null && docker ps 2>/dev/null | grep -q ollama; then
        if docker inspect "$(docker ps --format "{{.Names}}" | grep ollama | head -1)" 2>/dev/null | grep -q "DeviceRequests"; then
            echo -e "${GREEN}✓ System configured correctly for GPU acceleration${NC}"
        else
            echo -e "${RED}✗ Docker container needs GPU access${NC}"
            echo ""
            echo "Fix:"
            echo "  docker stop ollama"
            echo "  docker run -d --gpus all -v ollama:/root/.ollama -p 11434:11434 --name ollama ollama/ollama"
        fi
    else
        echo -e "${GREEN}✓ GPU available (native Ollama install)${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Running on CPU - speeds will be 5-10x slower${NC}"
    echo ""
    echo "To add GPU support:"
    echo "  1. Install NVIDIA drivers"
    echo "  2. Install NVIDIA Container Toolkit (if using Docker)"
    echo "  3. Restart Ollama with GPU access"
fi

echo ""
echo "Expected speeds for common models (GPU):"
echo "  phi4:14b           ~15-20 tok/s"
echo "  qwen2.5-coder:32b  ~4-5 tok/s"
echo "  llama3.3:70b       ~1-2 tok/s"
echo ""
echo "If speeds are much lower, check:"
echo "  1. GPU is being used (nvidia-smi)"
echo "  2. Docker has GPU access (--gpus all)"
echo "  3. No other processes using GPU heavily"
echo "  4. Thermal throttling (GPU temp)"
echo ""
