#!/bin/bash
# Check all dependencies without building or launching

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "Checking ShadowHound Dependencies..."
echo ""

# ROS2
echo -n "ROS2: "
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}✓ Installed${NC}"
else
    echo -e "${RED}✗ Not found${NC}"
fi

# Python
echo -n "Python 3: "
if command -v python3 &> /dev/null; then
    version=$(python3 --version | cut -d' ' -f2)
    echo -e "${GREEN}✓ $version${NC}"
else
    echo -e "${RED}✗ Not found${NC}"
fi

# Colcon
echo -n "colcon: "
if command -v colcon &> /dev/null; then
    echo -e "${GREEN}✓ Installed${NC}"
else
    echo -e "${RED}✗ Not found${NC}"
fi

# Python packages
echo ""
echo "Python Packages:"

check_package() {
    local package=$1
    local import_name=${2:-$1}
    echo -n "  $package: "
    if python3 -c "import $import_name" 2>/dev/null; then
        echo -e "${GREEN}✓${NC}"
    else
        echo -e "${RED}✗${NC}"
    fi
}

check_package "rclpy" "rclpy"
check_package "fastapi" "fastapi"
check_package "uvicorn" "uvicorn"
check_package "websockets" "websockets"
check_package "openai" "openai"
check_package "pydantic" "pydantic"
check_package "python-dotenv" "dotenv"

# Git submodules
echo ""
echo "Git Submodules:"
echo -n "  dimos-unitree: "
if [ -d "src/dimos-unitree/.git" ] && [ -f "src/dimos-unitree/setup.py" ]; then
    echo -e "${GREEN}✓ Initialized${NC}"
else
    echo -e "${RED}✗ Not initialized${NC}"
fi

# Configuration
echo ""
echo "Configuration:"
echo -n "  .env file: "
if [ -f ".env" ]; then
    echo -e "${GREEN}✓ Exists${NC}"
    
    # Check if API key is set
    if grep -q "OPENAI_API_KEY=sk-" .env 2>/dev/null && ! grep -q "sk-proj-your-api-key-here" .env; then
        echo -e "  OpenAI API Key: ${GREEN}✓ Configured${NC}"
    else
        echo -e "  OpenAI API Key: ${YELLOW}⚠ Not configured${NC}"
    fi
else
    echo -e "${RED}✗ Not found${NC}"
    echo -e "  ${YELLOW}⚠ Run ./start.sh to create .env${NC}"
fi

# Workspace
echo ""
echo "Workspace:"
echo -n "  Built: "
if [ -d "install" ] && [ -f "install/setup.bash" ]; then
    echo -e "${GREEN}✓ Yes${NC}"
else
    echo -e "${YELLOW}⚠ Not yet${NC}"
fi

echo ""
echo "Ready to run: ./start.sh"
