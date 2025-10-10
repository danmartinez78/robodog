#!/bin/bash
# Setup Ollama on Thor for ShadowHound Testing
# Run this script ON THOR (the robot on your desk)

set -e  # Exit on error

echo "=========================================="
echo "ShadowHound Ollama Setup for Thor"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Configuration
OLLAMA_IMAGE="ghcr.io/nvidia-ai-iot/ollama:r38.2.arm64-sbsa-cu130-24.04"
CONTAINER_NAME="ollama"
DATA_DIR="${HOME}/ollama-data"
OLLAMA_PORT=11434

# Model recommendations based on Thor's 128GB RAM and benchmark results
# Benchmarked 2025-01-10: qwen2.5-coder:32b WINNER (98/100 quality, JSON specialist)
# See docs/OLLAMA_BENCHMARK_RESULTS.md for full analysis
PRIMARY_MODEL="qwen2.5-coder:32b"  # ~20GB download, ~20GB RAM (JSON specialist, 98/100 quality)
BACKUP_MODEL="phi4:14b"            # ~9GB download, ~9GB RAM (fast fallback, 86.7/100 quality, 20 tok/s)

echo -e "${YELLOW}Step 1: Checking prerequisites...${NC}"

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed!${NC}"
    echo "Please install Docker first:"
    echo "  sudo apt-get update"
    echo "  sudo apt-get install docker.io"
    exit 1
fi

# Check if user is in docker group
if ! groups | grep -q docker; then
    echo -e "${YELLOW}Warning: User not in docker group. You may need sudo.${NC}"
    echo "To fix: sudo usermod -aG docker $USER && newgrp docker"
fi

echo -e "${GREEN}✓ Docker installed${NC}"

# Check for NVIDIA GPU
if command -v nvidia-smi &> /dev/null; then
    echo -e "${GREEN}✓ NVIDIA GPU detected${NC}"
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
else
    echo -e "${RED}Warning: nvidia-smi not found. GPU acceleration may not work!${NC}"
fi

echo ""
echo -e "${YELLOW}Step 2: Creating data directory...${NC}"
mkdir -p "$DATA_DIR"
echo -e "${GREEN}✓ Created $DATA_DIR${NC}"

echo ""
echo -e "${YELLOW}Step 3: Stopping any existing Ollama container...${NC}"
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
    echo -e "${GREEN}✓ Removed existing container${NC}"
else
    echo "No existing container found"
fi

echo ""
echo -e "${YELLOW}Step 4: Pulling Ollama image...${NC}"
echo "This may take several minutes..."
docker pull "$OLLAMA_IMAGE"
echo -e "${GREEN}✓ Image pulled${NC}"

echo ""
echo -e "${YELLOW}Step 5: Starting Ollama container...${NC}"

# The image needs a TTY to keep running (similar to -it flag)
# Use --tty flag with detached mode
docker run -d \
  --name "$CONTAINER_NAME" \
  --gpus all \
  --tty \
  -p ${OLLAMA_PORT}:11434 \
  -v "${DATA_DIR}:/data" \
  --restart unless-stopped \
  "$OLLAMA_IMAGE"

CONTAINER_ID=$(docker ps -lq)
echo -e "${GREEN}✓ Container started: $CONTAINER_ID${NC}"

# Give server time to start
sleep 10

# Wait for Ollama to be ready
echo ""
echo -e "${YELLOW}Step 6: Waiting for Ollama to start...${NC}"
echo "Checking container status..."
sleep 5

# Check if container is still running
if ! docker ps | grep -q "$CONTAINER_NAME"; then
    echo -e "${RED}Error: Container exited${NC}"
    echo "Container logs:"
    docker logs "$CONTAINER_NAME"
    exit 1
fi

# Check if Ollama API is responding
MAX_RETRIES=30
RETRY_COUNT=0
echo -n "Waiting for API"
while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    if curl -s --max-time 2 http://localhost:${OLLAMA_PORT}/api/tags > /dev/null 2>&1; then
        echo ""
        echo -e "${GREEN}✓ Ollama is ready!${NC}"
        break
    fi
    echo -n "."
    sleep 2
    RETRY_COUNT=$((RETRY_COUNT + 1))
done

if [ $RETRY_COUNT -eq $MAX_RETRIES ]; then
    echo ""
    echo -e "${RED}Error: Ollama API not responding${NC}"
    docker logs --tail 20 "$CONTAINER_NAME"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 7: Pulling model...${NC}"
echo ""
echo "Based on comprehensive benchmarking (2025-01-10):"
echo "  - PRIMARY: qwen2.5-coder:32b (~20GB) - 98/100 quality, JSON specialist"
echo "  - BACKUP: phi4:14b (~9GB) - 86.7/100 quality, 4.6x faster"
echo ""
echo "See docs/OLLAMA_BENCHMARK_RESULTS.md for full analysis"
echo ""

# Pull the primary model
echo -e "${YELLOW}Pulling $PRIMARY_MODEL...${NC}"
echo "Expected size: ~20 GB | RAM usage: ~20 GB"
echo "This will take 5-15 minutes depending on connection..."
if docker exec "$CONTAINER_NAME" ollama pull "$PRIMARY_MODEL" 2>&1; then
    echo -e "${GREEN}✓ $PRIMARY_MODEL ready${NC}"
else
    echo -e "${RED}Failed to pull $PRIMARY_MODEL${NC}"
    echo ""
    echo "Available models can be checked at: https://ollama.com/library"
    echo "Or run: docker exec ollama ollama list"
    exit 1
fi

# Ask about backup model
echo ""
echo -e "${YELLOW}Optional: Pull fast backup model for development?${NC}"
read -p "Pull backup model $BACKUP_MODEL (~9GB, 4.6x faster than primary)? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Pulling $BACKUP_MODEL...${NC}"
    docker exec "$CONTAINER_NAME" ollama pull "$BACKUP_MODEL"
    echo -e "${GREEN}✓ $BACKUP_MODEL ready${NC}"
fi

echo ""
echo -e "${YELLOW}Step 8: Getting Thor's network information...${NC}"
THOR_IP=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v 127.0.0.1 | head -n1)
echo -e "${GREEN}✓ Thor's IP address: $THOR_IP${NC}"

echo ""
echo -e "${YELLOW}Step 9: Testing local connectivity...${NC}"
curl -s http://localhost:${OLLAMA_PORT}/api/tags | python3 -m json.tool
echo -e "${GREEN}✓ Local test successful${NC}"

echo ""
echo -e "${YELLOW}Step 10: Testing inference...${NC}"
echo "Running quick test with $PRIMARY_MODEL..."
INFERENCE_TEST=$(curl -s http://localhost:${OLLAMA_PORT}/api/generate -d "{
  \"model\": \"$PRIMARY_MODEL\",
  \"prompt\": \"Say 'Ollama is operational' in 5 words or less\",
  \"stream\": false
}")

if echo "$INFERENCE_TEST" | grep -q "response"; then
    echo -e "${GREEN}✓ Inference test successful${NC}"
    echo "Model response:"
    echo "$INFERENCE_TEST" | python3 -c "import sys, json; print(json.load(sys.stdin)['response'])"
else
    echo -e "${RED}Warning: Inference test failed${NC}"
    echo "$INFERENCE_TEST"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}✓ Setup Complete!${NC}"
echo "=========================================="
echo ""
echo "Container Status:"
docker ps | grep "$CONTAINER_NAME"
echo ""
echo "Installed Models:"
docker exec "$CONTAINER_NAME" ollama list
echo ""
echo "=========================================="
echo "Next Steps on LAPTOP:"
echo "=========================================="
echo ""
echo "1. Switch to feature branch:"
echo "   cd /workspaces/shadowhound"
echo "   git checkout feature/local-llm-support"
echo ""
echo "2. Test connectivity from laptop:"
echo "   curl http://${THOR_IP}:${OLLAMA_PORT}/api/tags"
echo ""
echo "3. Launch mission agent with Ollama:"
echo "   ros2 launch shadowhound_mission_agent mission_agent.launch.py \\"
echo "       agent_backend:=ollama \\"
echo "       ollama_base_url:=http://${THOR_IP}:${OLLAMA_PORT} \\"
echo "       ollama_model:=${PRIMARY_MODEL}"
echo ""
echo "4. Open web UI and check for green 'OLLAMA' indicator:"
echo "   http://localhost:8080"
echo ""
echo "=========================================="
echo "Useful Commands:"
echo "=========================================="
echo ""
echo "# View container logs"
echo "docker logs -f $CONTAINER_NAME"
echo ""
echo "# Restart container"
echo "docker restart $CONTAINER_NAME"
echo ""
echo "# Stop container"
echo "docker stop $CONTAINER_NAME"
echo ""
echo "# Pull additional models"
echo "docker exec $CONTAINER_NAME ollama pull <model-name>"
echo ""
echo "# List available models"
echo "docker exec $CONTAINER_NAME ollama list"
echo ""
echo "# Test inference"
echo "docker exec -it $CONTAINER_NAME ollama run $PRIMARY_MODEL"
echo ""
echo "=========================================="
echo -e "${GREEN}Thor is ready for Ollama testing!${NC}"
echo "=========================================="
