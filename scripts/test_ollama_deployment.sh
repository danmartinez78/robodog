#!/bin/bash
# Quick Test Script for Ollama Deployment Validation
# Run this on LAPTOP after Thor + GO2 are ready
#
# Usage: ./test_ollama_deployment.sh [phase]
#   phase: 1-6 (default: all)

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

THOR_IP="${THOR_IP:-192.168.50.10}"
OLLAMA_MODEL="${OLLAMA_MODEL:-qwen2.5-coder:32b}"
BACKUP_MODEL="${BACKUP_MODEL:-phi4:14b}"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Ollama Deployment Test Suite${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "Configuration:"
echo "  Thor IP: $THOR_IP"
echo "  Primary Model: $OLLAMA_MODEL"
echo "  Backup Model: $BACKUP_MODEL"
echo ""

# Function to check if Thor is reachable
check_thor() {
    echo -e "${YELLOW}Checking Thor connectivity...${NC}"
    if ping -c 1 -W 2 $THOR_IP > /dev/null 2>&1; then
        echo -e "${GREEN}✓ Thor reachable at $THOR_IP${NC}"
    else
        echo -e "${RED}✗ Cannot reach Thor at $THOR_IP${NC}"
        echo "  Update THOR_IP environment variable if needed"
        exit 1
    fi
}

# Function to check Ollama service
check_ollama() {
    echo -e "${YELLOW}Checking Ollama service...${NC}"
    if curl -s --max-time 5 "http://$THOR_IP:11434/api/tags" > /dev/null; then
        echo -e "${GREEN}✓ Ollama service responding${NC}"
        
        # Check if model is available
        if curl -s "http://$THOR_IP:11434/api/tags" | grep -q "$OLLAMA_MODEL"; then
            echo -e "${GREEN}✓ Model $OLLAMA_MODEL available${NC}"
        else
            echo -e "${RED}✗ Model $OLLAMA_MODEL not found${NC}"
            echo "  Available models:"
            curl -s "http://$THOR_IP:11434/api/tags" | jq -r '.models[].name' | sed 's/^/    /'
            exit 1
        fi
    else
        echo -e "${RED}✗ Ollama service not responding${NC}"
        echo "  Check: docker ps | grep ollama (on Thor)"
        exit 1
    fi
}

# Function to test LLM directly
test_llm_direct() {
    echo ""
    echo -e "${BLUE}=== Direct LLM Test ===${NC}"
    echo -e "${YELLOW}Sending test prompt to $OLLAMA_MODEL...${NC}"
    
    local start_time=$(date +%s.%N)
    local response=$(curl -s --max-time 30 "http://$THOR_IP:11434/api/generate" \
        -d "{\"model\": \"$OLLAMA_MODEL\", \"prompt\": \"Say 'OK' if you can hear me.\", \"stream\": false}")
    local end_time=$(date +%s.%N)
    
    local duration=$(echo "$end_time - $start_time" | bc)
    
    if echo "$response" | jq -e '.response' > /dev/null 2>&1; then
        local text=$(echo "$response" | jq -r '.response')
        local tokens=$(echo "$response" | jq -r '.eval_count')
        local tok_per_sec=$(echo "$response" | jq -r '.eval_count / (.eval_duration / 1000000000)' | bc -l)
        
        echo -e "${GREEN}✓ Response received in ${duration}s${NC}"
        echo "  Response: $text"
        echo "  Tokens: $tokens"
        echo "  Speed: $(printf '%.1f' $tok_per_sec) tok/s"
        echo ""
        return 0
    else
        echo -e "${RED}✗ Invalid response${NC}"
        echo "$response"
        return 1
    fi
}

# Function to show launch command
show_launch_command() {
    echo ""
    echo -e "${BLUE}=== Mission Agent Launch ===${NC}"
    echo ""
    echo -e "${YELLOW}Run this command to launch mission agent with Ollama:${NC}"
    echo ""
    echo "ros2 launch shadowhound_mission_agent mission_agent.launch.py \\"
    echo "    agent_backend:=ollama \\"
    echo "    ollama_base_url:=http://$THOR_IP:11434 \\"
    echo "    ollama_model:=$OLLAMA_MODEL \\"
    echo "    web_host:=0.0.0.0 \\"
    echo "    web_port:=8080"
    echo ""
    echo -e "${YELLOW}Then open browser to: http://localhost:8080${NC}"
    echo ""
}

# Function to show test missions
show_test_missions() {
    echo ""
    echo -e "${BLUE}=== Test Missions ===${NC}"
    echo ""
    echo -e "${YELLOW}Phase 2: Simple Tests${NC}"
    echo "  1. \"Describe what you are\""
    echo "  2. \"Create a navigation plan to move forward 3 meters then rotate right\""
    echo "  3. \"A robot is 0.6m wide. A doorway is 0.8m wide with an obstacle 0.3m to the left. Should the robot go left or right?\""
    echo ""
    echo -e "${YELLOW}Phase 3: Robot Hardware${NC}"
    echo "  4. \"Move forward 1 meter\""
    echo "  5. \"Rotate 90 degrees left, move forward 2 meters, then rotate back\""
    echo "  6. \"Take a photo\""
    echo ""
    echo -e "${YELLOW}Phase 4: Performance${NC}"
    echo "  Run 10+ simple missions, monitor response times"
    echo ""
}

# Function to show monitoring commands
show_monitoring() {
    echo ""
    echo -e "${BLUE}=== Monitoring Commands ===${NC}"
    echo ""
    echo -e "${YELLOW}On Thor (in separate terminals):${NC}"
    echo ""
    echo "# Monitor Ollama memory"
    echo "watch -n 5 'docker stats ollama --no-stream'"
    echo ""
    echo "# Monitor Ollama logs"
    echo "docker logs -f ollama"
    echo ""
    echo "# Monitor system resources"
    echo "htop"
    echo ""
    echo -e "${YELLOW}On Laptop:${NC}"
    echo ""
    echo "# Monitor mission status"
    echo "ros2 topic echo /shadowhound/mission/status"
    echo ""
    echo "# Monitor velocity commands"
    echo "ros2 topic echo /cmd_vel"
    echo ""
    echo "# Monitor skill execution"
    echo "ros2 topic echo /shadowhound/skill/status"
    echo ""
}

# Function to test backup model
test_backup_model() {
    echo ""
    echo -e "${BLUE}=== Backup Model Test ===${NC}"
    echo -e "${YELLOW}Testing $BACKUP_MODEL...${NC}"
    
    if curl -s "http://$THOR_IP:11434/api/tags" | grep -q "$BACKUP_MODEL"; then
        echo -e "${GREEN}✓ Backup model $BACKUP_MODEL available${NC}"
        
        echo -e "${YELLOW}Sending test prompt...${NC}"
        local response=$(curl -s --max-time 30 "http://$THOR_IP:11434/api/generate" \
            -d "{\"model\": \"$BACKUP_MODEL\", \"prompt\": \"Say OK.\", \"stream\": false}")
        
        if echo "$response" | jq -e '.response' > /dev/null 2>&1; then
            local tok_per_sec=$(echo "$response" | jq -r '.eval_count / (.eval_duration / 1000000000)' | bc -l)
            echo -e "${GREEN}✓ Backup model working${NC}"
            echo "  Speed: $(printf '%.1f' $tok_per_sec) tok/s (should be ~20 tok/s)"
        else
            echo -e "${RED}✗ Backup model failed${NC}"
        fi
    else
        echo -e "${RED}✗ Backup model $BACKUP_MODEL not found${NC}"
        echo "  Pull it on Thor: docker exec ollama ollama pull $BACKUP_MODEL"
    fi
}

# Function to show results checklist
show_results_checklist() {
    echo ""
    echo -e "${BLUE}=== Results Checklist ===${NC}"
    echo ""
    echo "After completing tests, fill in docs/OLLAMA_DEPLOYMENT_CHECKLIST.md:"
    echo ""
    echo "[ ] All Phase 1 tests passed (mission agent startup)"
    echo "[ ] All Phase 2 tests passed (simple missions)"
    echo "[ ] All Phase 3 tests passed (robot hardware)"
    echo "[ ] All Phase 4 tests passed (performance)"
    echo "[ ] All Phase 5 tests passed (backup model)"
    echo "[ ] Optional Phase 6 completed (OpenAI comparison)"
    echo ""
    echo "[ ] Mission success rate: >95%"
    echo "[ ] Average response time: <5s"
    echo "[ ] Memory stable: ~20GB for primary model"
    echo "[ ] JSON quality: >90/100"
    echo ""
    echo "If all pass: Ready to merge to dev branch!"
    echo ""
}

# Main execution
main() {
    check_thor
    check_ollama
    test_llm_direct
    test_backup_model
    show_launch_command
    show_test_missions
    show_monitoring
    show_results_checklist
    
    echo -e "${GREEN}Pre-flight checks complete!${NC}"
    echo -e "${YELLOW}Follow the test plan in docs/OLLAMA_DEPLOYMENT_CHECKLIST.md${NC}"
    echo ""
}

main "$@"
