#!/bin/bash
# Test Ollama Backend from Laptop
# Run this script ON LAPTOP after Thor setup is complete

set -e

echo "=========================================="
echo "ShadowHound Ollama Testing (Laptop)"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Get Thor's IP from user
echo -e "${YELLOW}Enter Thor's IP address:${NC}"
read -p "Thor IP: " THOR_IP

if [ -z "$THOR_IP" ]; then
    echo -e "${RED}Error: Thor IP cannot be empty${NC}"
    exit 1
fi

OLLAMA_PORT=11434
OLLAMA_URL="http://${THOR_IP}:${OLLAMA_PORT}"
MODEL="llama3.1:13b"

echo ""
echo -e "${YELLOW}Step 1: Testing network connectivity...${NC}"
if ping -c 3 "$THOR_IP" > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Thor is reachable via ping${NC}"
else
    echo -e "${RED}✗ Cannot ping Thor${NC}"
    echo "Check network connection and Thor's IP address"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 2: Testing Ollama API endpoint...${NC}"
if curl -s --connect-timeout 5 "${OLLAMA_URL}/api/tags" > /dev/null; then
    echo -e "${GREEN}✓ Ollama API is accessible${NC}"
else
    echo -e "${RED}✗ Cannot connect to Ollama API${NC}"
    echo "Check if:"
    echo "  - Ollama container is running on Thor"
    echo "  - Port $OLLAMA_PORT is open"
    echo "  - Firewall allows connections"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 3: Listing available models...${NC}"
MODELS=$(curl -s "${OLLAMA_URL}/api/tags")
echo "$MODELS" | python3 -m json.tool
echo ""

if echo "$MODELS" | grep -q "$MODEL"; then
    echo -e "${GREEN}✓ Model $MODEL is available${NC}"
else
    echo -e "${RED}✗ Model $MODEL not found${NC}"
    echo "Available models:"
    echo "$MODELS" | python3 -c "import sys, json; [print(m['name']) for m in json.load(sys.stdin).get('models', [])]"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 4: Testing inference (this may take a few seconds)...${NC}"
START_TIME=$(date +%s.%N)

RESPONSE=$(curl -s "${OLLAMA_URL}/api/generate" -d "{
  \"model\": \"$MODEL\",
  \"prompt\": \"You are TARS from Interstellar. Say hello in 10 words or less.\",
  \"stream\": false
}")

END_TIME=$(date +%s.%N)
DURATION=$(echo "$END_TIME - $START_TIME" | bc)

if echo "$RESPONSE" | grep -q "response"; then
    echo -e "${GREEN}✓ Inference successful${NC}"
    echo ""
    echo "Response:"
    echo "$RESPONSE" | python3 -c "import sys, json; print(json.load(sys.stdin)['response'])"
    echo ""
    echo -e "${GREEN}Time taken: ${DURATION}s${NC}"
    
    # Performance evaluation
    DURATION_INT=$(echo "$DURATION" | cut -d. -f1)
    if [ "$DURATION_INT" -lt 3 ]; then
        echo -e "${GREEN}🚀 Excellent! Sub-3s response time${NC}"
    elif [ "$DURATION_INT" -lt 5 ]; then
        echo -e "${YELLOW}👍 Good! Response time within target${NC}"
    else
        echo -e "${YELLOW}⚠️  Slower than expected. Check GPU utilization on Thor.${NC}"
    fi
else
    echo -e "${RED}✗ Inference failed${NC}"
    echo "$RESPONSE"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 5: Measuring latency (5 quick requests)...${NC}"
TOTAL=0
for i in {1..5}; do
    START=$(date +%s.%N)
    curl -s "${OLLAMA_URL}/api/generate" -d "{
      \"model\": \"$MODEL\",
      \"prompt\": \"Hi\",
      \"stream\": false,
      \"options\": {\"num_predict\": 10}
    }" > /dev/null
    END=$(date +%s.%N)
    LATENCY=$(echo "$END - $START" | bc)
    TOTAL=$(echo "$TOTAL + $LATENCY" | bc)
    echo "  Request $i: ${LATENCY}s"
done

AVG=$(echo "scale=2; $TOTAL / 5" | bc)
echo -e "${GREEN}Average latency: ${AVG}s${NC}"

echo ""
echo "=========================================="
echo -e "${GREEN}✓ All Tests Passed!${NC}"
echo "=========================================="
echo ""
echo "Configuration for mission agent:"
echo "  agent_backend: ollama"
echo "  ollama_base_url: ${OLLAMA_URL}"
echo "  ollama_model: ${MODEL}"
echo ""
echo "=========================================="
echo "Next Steps:"
echo "=========================================="
echo ""
echo "1. Ensure you're on the feature branch:"
echo "   git checkout feature/local-llm-support"
echo ""
echo "2. Update config file (optional):"
echo "   nano configs/laptop_dev_ollama.yaml"
echo "   # Change ollama_base_url to: ${OLLAMA_URL}"
echo ""
echo "3. Launch mission agent:"
echo "   ros2 launch shadowhound_mission_agent mission_agent.launch.py \\"
echo "       agent_backend:=ollama \\"
echo "       ollama_base_url:=${OLLAMA_URL} \\"
echo "       ollama_model:=${MODEL}"
echo ""
echo "4. Open web UI and verify:"
echo "   - Browser: http://localhost:8080"
echo "   - Check diagnostics shows: LLM BACKEND = OLLAMA (green)"
echo "   - Send test command: 'rotate 90 degrees'"
echo "   - Verify response time < 3s"
echo ""
echo "5. Compare with OpenAI (optional):"
echo "   # Stop current agent (Ctrl+C)"
echo "   ros2 launch shadowhound_mission_agent mission_agent.launch.py \\"
echo "       agent_backend:=openai"
echo "   # Send same command and compare times"
echo ""
echo "=========================================="
echo -e "${GREEN}Ready to test Ollama backend!${NC}"
echo "=========================================="
