#!/bin/bash
# ============================================================================
# ShadowHound Start Script
# ============================================================================
#
# This script handles complete setup and launch of the ShadowHound system.
# It checks dependencies, validates configuration, and provides helpful
# guidance for first-time setup.
#
# Usage:
#   ./start.sh [OPTIONS]
#
# Options:
#   --dev          Use development configuration (mock robot)
#   --prod         Use production configuration (real robot)
#   --mock         Force mock robot mode
#   --no-web       Disable web interface
#   --web-port N   Set web port (default: 8080)
#   --skip-update  Skip git repository update check
#   --auto-update  Automatically pull updates without prompting
#   --skip-driver  Skip launching robot driver (use existing)
#   --agent-only   Only launch mission agent (skip driver + verification)
#   --help         Show this help message
#
# Examples:
#   ./start.sh                    # Interactive mode
#   ./start.sh --dev              # Development mode
#   ./start.sh --prod --no-web    # Production without web UI
#   ./start.sh --mock             # Mock robot mode
#   ./start.sh --auto-update      # Auto-pull latest changes
#   ./start.sh --skip-update      # Don't check for updates
#
# ============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Emojis for better UX
CHECK="✓"
CROSS="✗"
WARN="⚠"
INFO="ℹ"
ROBOT="🐕"
WEB="🌐"
ROCKET="🚀"

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Load .env file if it exists (do this early to get ROS_DOMAIN_ID)
if [ -f ".env" ]; then
    set -a  # automatically export all variables
    source .env
    set +a
fi

# Default options
MOCK_ROBOT=""
WEB_INTERFACE=""
WEB_PORT=""
CONFIG_MODE=""
SKIP_UPDATE=false
AUTO_UPDATE=false
SKIP_DRIVER=false
AGENT_ONLY=false

# ============================================================================
# Helper Functions
# ============================================================================

print_header() {
    echo ""
    echo -e "${CYAN}============================================================================${NC}"
    echo -e "${CYAN}  $ROBOT  ShadowHound - Autonomous Robot Control System${NC}"
    echo -e "${CYAN}============================================================================${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}${CHECK} $1${NC}"
}

print_error() {
    echo -e "${RED}${CROSS} $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}${WARN} $1${NC}"
}

print_info() {
    echo -e "${BLUE}${INFO} $1${NC}"
}

print_section() {
    echo ""
    echo -e "${CYAN}── $1 ──────────────────────────────────────────────────────${NC}"
}

# ============================================================================
# ROS Environment Setup
# ============================================================================

# Set ROS_DOMAIN_ID if not already set (for topic/node visibility)
if [ -z "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=0
fi

# ============================================================================
# Parse Command Line Arguments
# ============================================================================

parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --dev)
                CONFIG_MODE="development"
                shift
                ;;
            --prod)
                CONFIG_MODE="production"
                shift
                ;;
            --mock)
                MOCK_ROBOT="true"
                shift
                ;;
            --no-web)
                WEB_INTERFACE="false"
                shift
                ;;
            --web-port)
                WEB_PORT="$2"
                shift 2
                ;;
            --skip-update)
                SKIP_UPDATE=true
                shift
                ;;
            --auto-update)
                AUTO_UPDATE=true
                shift
                ;;
            --skip-driver)
                SKIP_DRIVER=true
                shift
                ;;
            --agent-only)
                AGENT_ONLY=true
                shift
                ;;
            --help|-h)
                grep '^#' "$0" | grep -v '#!/bin/bash' | sed 's/^# //' | sed 's/^#//'
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done
}

# ============================================================================
# System Checks
# ============================================================================

check_system() {
    print_section "System Check"
    
    local all_ok=true
    
    # Source ROS2 if not already sourced
    if [ -z "$ROS_DISTRO" ] && [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_info "Sourced ROS2 Humble environment"
    fi
    
    # Check ROS2
    if command -v ros2 &> /dev/null; then
        print_success "ROS2 installed"
    else
        print_error "ROS2 not found"
        print_info "Install ROS2 Humble: https://docs.ros.org/en/humble/Installation.html"
        all_ok=false
    fi
    
    # Check CycloneDDS if RMW_IMPLEMENTATION is set to it
    if [ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ]; then
        if dpkg -l | grep -q ros-humble-rmw-cyclonedds-cpp; then
            print_success "CycloneDDS middleware installed"
        else
            print_error "CycloneDDS not installed but RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
            print_info "Install: sudo apt install ros-humble-rmw-cyclonedds-cpp"
            all_ok=false
        fi
    fi
    
    # Check Python
    if command -v python3 &> /dev/null; then
        python_version=$(python3 --version | cut -d' ' -f2)
        print_success "Python $python_version installed"
    else
        print_error "Python 3 not found"
        all_ok=false
    fi
    
    # Check colcon
    if command -v colcon &> /dev/null; then
        print_success "colcon build tool installed"
    else
        print_error "colcon not found"
        print_info "Install: sudo apt install python3-colcon-common-extensions"
        all_ok=false
    fi
    
    # Check workspace
    if [ -d "src/shadowhound_mission_agent" ] && [ -d "src/dimos-unitree" ]; then
        print_success "Workspace structure valid"
    else
        print_error "Invalid workspace structure"
        print_info "Make sure you're in the workspace root and submodules are initialized"
        all_ok=false
    fi
    
    if [ "$all_ok" = false ]; then
        print_error "System checks failed. Please fix the issues above."
        exit 1
    fi
}

# ============================================================================
# Git Repository Updates
# ============================================================================

check_git_updates() {
    # Skip if requested
    if [ "$SKIP_UPDATE" = true ]; then
        print_info "Skipping repository update check (--skip-update flag)"
        return 0
    fi
    
    print_section "Repository Update Check"
    
    local updates_available=false
    local current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
    
    print_info "Current branch: $current_branch"
    
    # Fetch latest from remote (without pulling)
    print_info "Checking for updates from remote..."
    if git fetch origin 2>&1 | grep -q "error\|fatal"; then
        print_warning "Could not fetch from remote (network issue or no remote)"
        return 0
    fi
    
    # Check main repo for updates
    local local_commit=$(git rev-parse HEAD 2>/dev/null)
    local remote_commit=$(git rev-parse origin/$current_branch 2>/dev/null)
    
    if [ "$local_commit" != "$remote_commit" ]; then
        updates_available=true
        local ahead=$(git rev-list --count origin/$current_branch..HEAD 2>/dev/null || echo "0")
        local behind=$(git rev-list --count HEAD..origin/$current_branch 2>/dev/null || echo "0")
        
        if [ "$behind" -gt 0 ]; then
            print_warning "Main repo is $behind commit(s) behind remote"
            echo "  Latest changes:"
            git log --oneline HEAD..origin/$current_branch | head -3 | sed 's/^/    /'
        fi
        
        if [ "$ahead" -gt 0 ]; then
            print_info "Main repo has $ahead unpushed commit(s)"
        fi
    else
        print_success "Main repo is up to date"
    fi
    
    # Check submodules for updates
    print_info "Checking submodules..."
    
    if [ -f ".gitmodules" ]; then
        # Update submodule references
        git submodule update --init --remote 2>/dev/null || true
        
        # Check each submodule
        git submodule foreach --quiet '
            submodule_name=$(basename "$sm_path")
            local_commit=$(git rev-parse HEAD 2>/dev/null)
            
            # Fetch updates
            git fetch origin 2>/dev/null || exit 0
            
            current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
            remote_commit=$(git rev-parse origin/$current_branch 2>/dev/null)
            
            if [ "$local_commit" != "$remote_commit" ]; then
                behind=$(git rev-list --count HEAD..origin/$current_branch 2>/dev/null || echo "0")
                if [ "$behind" -gt 0 ]; then
                    echo "⚠️  $submodule_name: $behind commit(s) behind"
                    git log --oneline HEAD..origin/$current_branch | head -2 | sed "s/^/      /"
                fi
            else
                echo "✓ $submodule_name: up to date"
            fi
        ' | sed 's/^/  /'
    fi
    
    echo ""
    
    # Prompt to update if needed
    if [ "$updates_available" = true ]; then
        echo ""
        print_warning "Updates are available!"
        echo ""
        
        if [ "$AUTO_UPDATE" = true ]; then
            print_info "Auto-updating (--auto-update flag)..."
            pull_updates
        else
            read -p "Pull latest changes from remote? [Y/n]: " pull_choice
            
            if [[ "$pull_choice" != "n" && "$pull_choice" != "N" ]]; then
                pull_updates
            else
                print_info "Skipping updates (you can run 'git pull' manually later)"
            fi
        fi
    fi
}

pull_updates() {
    print_info "Pulling latest changes..."
    
    # Check for uncommitted changes
    if ! git diff-index --quiet HEAD -- 2>/dev/null; then
        print_warning "You have uncommitted changes"
        echo ""
        git status --short
        echo ""
        read -p "Stash changes before pulling? [Y/n]: " stash_choice
        
        if [[ "$stash_choice" != "n" && "$stash_choice" != "N" ]]; then
            git stash push -m "Auto-stash by start.sh at $(date)"
            print_success "Changes stashed"
            local stashed=true
        else
            print_error "Cannot pull with uncommitted changes"
            print_info "Either commit, stash, or discard your changes first"
            exit 1
        fi
    fi
    
    # Pull main repo
    if git pull origin $current_branch; then
        print_success "Main repo updated"
    else
        print_error "Failed to pull main repo"
        if [ "$stashed" = true ]; then
            print_info "Your changes are stashed. Run 'git stash pop' to restore them."
        fi
        exit 1
    fi
    
    # Update submodules
    print_info "Updating submodules..."
    if git submodule update --remote --merge; then
        print_success "Submodules updated"
    else
        print_warning "Some submodules may not have updated successfully"
    fi
    
    # Pop stash if we stashed
    if [ "$stashed" = true ]; then
        echo ""
        read -p "Restore your stashed changes? [Y/n]: " pop_choice
        if [[ "$pop_choice" != "n" && "$pop_choice" != "N" ]]; then
            if git stash pop; then
                print_success "Changes restored"
            else
                print_warning "Conflicts restoring changes - run 'git stash pop' manually"
            fi
        else
            print_info "Changes remain stashed - run 'git stash pop' when ready"
        fi
    fi
    
    # Suggest rebuild if code changed
    echo ""
    print_warning "Code was updated - rebuild recommended"
    FORCE_REBUILD=true
}

# ============================================================================
# Configuration Setup
# ============================================================================

setup_config() {
    print_section "Configuration Setup"
    
    # Check if .env exists
    if [ ! -f ".env" ]; then
        print_warning ".env file not found"
        
        # Interactive mode if no config specified
        if [ -z "$CONFIG_MODE" ]; then
            echo ""
            echo "Choose configuration mode:"
            echo "  1) Development (mock robot, cheap model, free embeddings)"
            echo "  2) Production (real robot, best model, security-focused)"
            echo "  3) Custom (start from .env.example)"
            echo ""
            read -p "Enter choice [1-3]: " choice
            
            case $choice in
                1)
                    CONFIG_MODE="development"
                    ;;
                2)
                    CONFIG_MODE="production"
                    ;;
                3)
                    CONFIG_MODE="example"
                    ;;
                *)
                    print_error "Invalid choice"
                    exit 1
                    ;;
            esac
        fi
        
        # Copy appropriate template
        case $CONFIG_MODE in
            development)
                cp .env.development .env
                print_success "Created .env from development template"
                ;;
            production)
                cp .env.production .env
                print_success "Created .env from production template"
                chmod 600 .env
                print_success "Secured .env file (chmod 600)"
                ;;
            example)
                cp .env.example .env
                print_success "Created .env from example template"
                ;;
        esac
        
        print_warning "You need to edit .env with your API keys!"
        echo ""
        echo "Minimum required:"
        echo "  - OPENAI_API_KEY=sk-your-key-here"
        if [ "$CONFIG_MODE" = "production" ]; then
            echo "  - ROBOT_IP=192.168.1.103 (your robot's IP)"
        fi
        echo ""
        read -p "Open .env for editing now? [Y/n]: " edit_choice
        
        if [[ "$edit_choice" != "n" && "$edit_choice" != "N" ]]; then
            ${EDITOR:-nano} .env
        else
            print_warning "Remember to edit .env before running!"
            exit 0
        fi
    else
        print_success ".env file exists"
    fi
    
    # Load .env
    if [ -f ".env" ]; then
        export $(grep -v '^#' .env | xargs)
        print_success "Loaded environment variables"
    fi
    
    # Validate critical variables
    if [ -z "$OPENAI_API_KEY" ] || [ "$OPENAI_API_KEY" = "sk-proj-your-api-key-here" ]; then
        print_error "OPENAI_API_KEY not set or still has placeholder value"
        print_info "Edit .env and add your OpenAI API key"
        print_info "Get key from: https://platform.openai.com/api-keys"
        exit 1
    fi
    print_success "OpenAI API key configured"
    
    # Check mock robot mode
    MOCK_ROBOT=${MOCK_ROBOT:-${MOCK_ROBOT_ENV:-false}}
    if [ "$MOCK_ROBOT" = "true" ]; then
        print_info "Using MOCK robot mode (no hardware needed)"
    else
        print_info "Using REAL robot mode"
        # Use ROBOT_IP (aligned with ROS2 SDK)
        if [ -z "$ROBOT_IP" ]; then
            export ROBOT_IP="192.168.10.167"  # Default IP
            print_warning "ROBOT_IP not set, using default: $ROBOT_IP"
        else
            print_success "Robot IP: $ROBOT_IP"
        fi
    fi
}

# ============================================================================
# Build Workspace
# ============================================================================

build_workspace() {
    print_section "Building Workspace"
    
    # Check if already built
    if [ -d "install" ] && [ -f "install/setup.bash" ] && [ "$FORCE_REBUILD" != "true" ]; then
        # Check if launch files are installed
        if [ ! -f "install/shadowhound_mission_agent/share/shadowhound_mission_agent/launch/mission_agent.launch.py" ]; then
            print_warning "Launch files not installed, rebuild required"
        else
            read -p "Workspace already built. Rebuild? [y/N]: " rebuild
            if [[ "$rebuild" != "y" && "$rebuild" != "Y" ]]; then
                print_info "Skipping build"
                return 0
            fi
        fi
    fi
    
    if [ "$FORCE_REBUILD" = "true" ]; then
        print_info "Code was updated - rebuilding workspace"
    fi
    
    print_info "Building ShadowHound packages..."
    
    # Build Go2 ROS2 SDK packages first (needed by DIMOS)
    print_info "Building Go2 ROS2 SDK packages..."
    if colcon build --packages-select go2_interfaces unitree_go go2_robot_sdk --symlink-install 2>&1 | tee -a /tmp/colcon_build.log | tail -10; then
        print_success "Go2 SDK packages built"
    else
        print_warning "Go2 SDK build had issues (may be OK if already built)"
    fi
    
    # Build our packages (skip DIMOS perception models with CUDA issues)
    print_info "Building ShadowHound mission agent..."
    if colcon build --packages-select shadowhound_mission_agent shadowhound_bringup --symlink-install 2>&1 | tee -a /tmp/colcon_build.log; then
        print_success "Build completed successfully"
    else
        print_error "Build failed"
        print_info "Check logs: /tmp/colcon_build.log"
        
        # Show last 20 lines of error
        echo ""
        echo "Last 20 lines of build output:"
        tail -20 /tmp/colcon_build.log
        
        exit 1
    fi
}

# ============================================================================
# Check Dependencies
# ============================================================================

check_dependencies() {
    print_section "Checking Python Dependencies"
    
    # Source ROS2 if not already sourced
    if [ -z "$ROS_DISTRO" ] && [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Source workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
    
    # Check critical Python packages
    local missing=()
    
    python3 -c "import fastapi" 2>/dev/null || missing+=("fastapi")
    python3 -c "import uvicorn" 2>/dev/null || missing+=("uvicorn")
    python3 -c "import openai" 2>/dev/null || missing+=("openai")
    python3 -c "import rclpy" 2>/dev/null || missing+=("ROS2 Python")
    python3 -c "import reactivex" 2>/dev/null || missing+=("reactivex (DIMOS)")
    python3 -c "import zmq" 2>/dev/null || missing+=("pyzmq (DIMOS)")
    python3 -c "import sounddevice" 2>/dev/null || missing+=("sounddevice (DIMOS)")
    python3 -c "import rxpy_backpressure" 2>/dev/null || missing+=("rxpy-backpressure (DIMOS)")
    python3 -c "import ultralytics" 2>/dev/null || missing+=("ultralytics (DIMOS perception)")
    python3 -c "import filterpy" 2>/dev/null || missing+=("filterpy (DIMOS perception)")
    python3 -c "import transformers" 2>/dev/null || missing+=("transformers (DIMOS vision)")
    python3 -c "import mmengine" 2>/dev/null || missing+=("mmengine (Metric3D depth)")
    python3 -c "import mmcv" 2>/dev/null || missing+=("mmcv (Metric3D depth)")
    
    if [ ${#missing[@]} -gt 0 ]; then
        print_warning "Missing Python packages: ${missing[*]}"
        read -p "Install missing packages? [Y/n]: " install_choice
        
        if [[ "$install_choice" != "n" && "$install_choice" != "N" ]]; then
            print_info "Installing Python packages (this may take a few minutes)..."
            
            local install_failed=false
            
            # Install all DIMOS base dependencies from requirements file
            if [ -f ".dimos-base-requirements.txt" ]; then
                print_info "Installing comprehensive DIMOS dependencies from requirements..."
                if ! pip3 install -q -r .dimos-base-requirements.txt; then
                    print_error "Failed to install some dependencies from requirements file"
                    install_failed=true
                fi
            else
                # Fallback to manual list if requirements file missing
                print_warning "Requirements file not found, using fallback install"
                if ! pip3 install -q \
                    fastapi uvicorn websockets pydantic \
                    openai anthropic tiktoken \
                    reactivex python-dotenv \
                    colorlog typeguard \
                    empy catkin_pkg lark \
                    Flask python-multipart \
                    pytest-asyncio asyncio \
                    sse-starlette \
                    langchain-chroma langchain-openai \
                    pyzmq numpy opencv-python \
                    ffmpeg-python sounddevice pyaudio \
                    requests wasmtime soundfile \
                    git+https://github.com/dimensionalOS/rxpy-backpressure.git; then
                    print_error "Failed to install some dependencies"
                    install_failed=true
                fi
            fi
            
            # Install mmcv separately (requires specific index for pre-built wheels)
            if [[ " ${missing[*]} " =~ " mmcv " ]]; then
                print_info "Installing mmcv-lite (pure Python, no CUDA compilation)..."
                # Try mmcv-lite first (no compilation needed)
                if ! pip3 install mmcv-lite; then
                    print_warning "mmcv-lite not available, trying OpenMMLab pre-built wheels..."
                    # Detect PyTorch version
                    local torch_version=$(python3 -c "import torch; print(torch.__version__.split('+')[0])" 2>/dev/null || echo "2.0.0")
                    local torch_major_minor=$(echo $torch_version | cut -d. -f1,2)
                    
                    print_info "Detected PyTorch ${torch_version}, using torch${torch_major_minor} index"
                    if ! pip3 install mmcv -f "https://download.openmmlab.com/mmcv/dist/cpu/torch${torch_major_minor}/index.html"; then
                        print_error "mmcv installation failed - Metric3D depth will not work"
                        print_info "You can try manually: pip install 'openmim' && mim install 'mmcv>=2.0.0'"
                        install_failed=true
                    fi
                fi
            fi
            
            if [ "$install_failed" = true ]; then
                print_error "Some packages failed to install"
                read -p "Continue anyway? (System may not work correctly) [y/N]: " continue_choice
                if [[ "$continue_choice" != "y" && "$continue_choice" != "Y" ]]; then
                    print_info "Exiting. Fix dependency issues and try again"
                    exit 1
                fi
                print_warning "Continuing with missing dependencies - expect errors"
            else
                print_success "All packages installed successfully"
            fi
        else
            print_warning "Some features may not work without these packages"
        fi
    else
        print_success "All required Python packages installed"
    fi
}

# ============================================================================
# Network Checks
# ============================================================================

check_network() {
    if [ "$MOCK_ROBOT" = "true" ]; then
        return 0  # Skip network checks for mock robot
    fi
    
    print_section "Network Check"
    
    local robot_ip=${ROBOT_IP:-192.168.10.167}
    
    print_info "Checking connection to robot at $robot_ip..."
    
    if ping -c 1 -W 2 "$robot_ip" &> /dev/null; then
        print_success "Robot is reachable at $robot_ip"
    else
        print_warning "Cannot reach robot at $robot_ip"
        print_info "This is OK if you're using mock mode"
        
        read -p "Continue anyway? [y/N]: " continue_choice
        if [[ "$continue_choice" != "y" && "$continue_choice" != "Y" ]]; then
            print_info "Exiting. Fix network connection or use --mock flag"
            exit 1
        fi
    fi
}

# ============================================================================
# Pre-flight Summary
# ============================================================================

show_summary() {
    print_section "Pre-flight Summary"
    
    # Create environment file for other terminals
    cat > "$SCRIPT_DIR/.shadowhound_env" << EOF
# ShadowHound Environment
# Source this file in other terminals to access the same ROS domain:
#   source .shadowhound_env

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROBOT_IP=${ROBOT_IP:-192.168.10.167}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Connection type for Unitree Go2 (cyclonedds for Ethernet, webrtc for WiFi)
# WebRTC required for DIMOS high-level API commands (sit, stand, wave, etc.)
export CONN_TYPE=${CONN_TYPE:-webrtc}

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

echo "✓ ShadowHound environment loaded"
echo "  ROS_DOMAIN_ID: \$ROS_DOMAIN_ID"
echo "  ROBOT_IP: \$ROBOT_IP"
echo "  CONN_TYPE: \$CONN_TYPE"
EOF
    
    echo ""
    echo "Configuration:"
    echo "  • Mode: ${CONFIG_MODE:-default}"
    echo "  • Mock Robot: ${MOCK_ROBOT:-false}"
    echo "  • Connection: ${CONN_TYPE:-webrtc}"
    echo "  • Web Interface: ${WEB_INTERFACE:-true}"
    echo "  • Web Port: ${WEB_PORT:-8080}"
    echo "  • ROS Domain: ${ROS_DOMAIN_ID:-0}"
    echo "  • OpenAI Model: ${OPENAI_MODEL:-gpt-4o}"
    echo ""
    if [ "${CONN_TYPE:-webrtc}" = "webrtc" ]; then
        echo -e "${CYAN}${INFO} WebRTC mode enabled - robot must be on WiFi network${NC}"
    else
        echo -e "${YELLOW}${WARN} CycloneDDS mode - high-level API commands (sit/stand/wave) unavailable${NC}"
    fi
    echo ""
    echo -e "${CYAN}${INFO} To access ROS topics in another terminal, run:${NC}"
    echo -e "${CYAN}    source .shadowhound_env${NC}"
    echo ""
    
    if [ "${WEB_INTERFACE:-true}" != "false" ]; then
        echo -e "${GREEN}${WEB} Web Dashboard: http://localhost:${WEB_PORT:-8080}${NC}"
        echo ""
    fi
    
    echo "ROS Topics:"
    echo "  • Commands: /mission_command"
    echo "  • Status: /mission_status"
    echo ""
}

# ============================================================================
# Robot Driver Launch
# ============================================================================

launch_robot_driver() {
    if [ "$MOCK_ROBOT" = "true" ]; then
        print_info "Mock robot mode - skipping robot driver launch"
        return 0
    fi
    
    print_section "Stage 1: Launching Robot Driver"
    
    local robot_ip=${ROBOT_IP:-192.168.10.167}
    export ROBOT_IP=$robot_ip
    
    # Ping robot one more time
    print_info "Verifying robot connectivity at $robot_ip..."
    if ! ping -c 1 -W 2 "$robot_ip" &> /dev/null; then
        print_error "Robot not reachable at $robot_ip"
        print_info "Make sure robot is powered on and connected"
        return 1
    fi
    print_success "Robot is reachable"
    
    # Check if robot driver is already running
    if ros2 topic list 2>/dev/null | grep -q "/go2_states"; then
        print_warning "Robot driver already running (topics detected)"
        read -p "Use existing driver? [Y/n]: " use_existing
        if [[ "$use_existing" != "n" && "$use_existing" != "N" ]]; then
            print_success "Using existing robot driver"
            return 0
        else
            print_info "Stopping existing driver..."
            pkill -f "go2_driver_node" 2>/dev/null || true
            pkill -f "robot.launch" 2>/dev/null || true
            sleep 2
        fi
    fi
    
    # Determine which launch file to use
    local robot_launch="launch/go2_sdk/robot.launch.py"
    if [ ! -f "$robot_launch" ]; then
        robot_launch="src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk/launch/robot.launch.py"
    fi
    
    if [ ! -f "$robot_launch" ]; then
        print_error "Robot launch file not found"
        print_info "Expected: launch/go2_sdk/robot.launch.py"
        return 1
    fi
    
    print_info "Launching robot driver in background..."
    print_info "Launch file: $robot_launch"
    echo ""
    
    # Launch robot driver in background with log file
    # Explicitly enable Nav2 (required for DIMOS /spin action) and RViz2
    local log_file="/tmp/shadowhound_robot_driver.log"
    ros2 launch "$robot_launch" nav2:=true rviz2:=true > "$log_file" 2>&1 &
    local driver_pid=$!
    
    print_success "Robot driver launched (PID: $driver_pid)"
    print_info "Logs: $log_file"
    echo ""
    
    # Wait for topics to appear
    print_info "Waiting for robot topics to appear..."
    local max_wait=30
    local waited=0
    
    while [ $waited -lt $max_wait ]; do
        if ros2 topic list 2>/dev/null | grep -q "/go2_states"; then
            print_success "Robot topics detected!"
            break
        fi
        
        # Check if driver process is still alive
        if ! kill -0 $driver_pid 2>/dev/null; then
            print_error "Robot driver process died"
            print_info "Check logs: $log_file"
            tail -20 "$log_file"
            return 1
        fi
        
        echo -n "."
        sleep 1
        waited=$((waited + 1))
    done
    echo ""
    
    if [ $waited -ge $max_wait ]; then
        print_error "Timeout waiting for robot topics"
        print_info "Driver may still be starting. Check logs: $log_file"
        return 1
    fi
    
    # Save PID for cleanup
    echo $driver_pid > /tmp/shadowhound_driver.pid
    
    return 0
}

# ============================================================================
# Verify Robot Topics
# ============================================================================

verify_robot_topics() {
    if [ "$MOCK_ROBOT" = "true" ]; then
        print_info "Mock robot mode - skipping topic verification"
        return 0
    fi
    
    print_section "Stage 2: Verifying Robot Topics"
    
    # Wait for Nav2 nodes to fully initialize (they take time after driver starts)
    print_info "Waiting for Nav2 nodes to initialize..."
    local nav2_wait=0
    local nav2_max_wait=15
    
    while [ $nav2_wait -lt $nav2_max_wait ]; do
        if ros2 node list 2>/dev/null | grep -q "behavior_server"; then
            print_success "Nav2 nodes detected"
            break
        fi
        echo -n "."
        sleep 1
        nav2_wait=$((nav2_wait + 1))
    done
    echo ""
    
    if [ $nav2_wait -ge $nav2_max_wait ]; then
        print_warning "Nav2 nodes not detected after ${nav2_max_wait}s"
        print_info "Expected nodes: behavior_server, controller_server, planner_server"
        print_warning "DIMOS may fail to initialize without /spin action"
    fi
    
    # Give Nav2 action servers a moment to register after nodes appear
    if [ $nav2_wait -lt $nav2_max_wait ]; then
        print_info "Waiting for Nav2 action servers to register..."
        sleep 3
        
        # DISABLED: Costmap trigger - can cause driver issues
        # Trigger costmap publication with a small movement
        # print_info "Triggering costmap publication (small robot movement)..."
        # if command -v python3 &> /dev/null && [ -f "scripts/trigger_costmap.py" ]; then
        #     python3 scripts/trigger_costmap.py
        # else
        #     print_warning "Costmap trigger script not found - costmaps may not publish until robot moves"
        # fi
    fi
    
    echo ""
    
    # Run our diagnostic script
    print_info "Running topic diagnostics..."
    echo ""
    
    local topics_ok=true
    if command -v python3 &> /dev/null && [ -f "scripts/check_topics.py" ]; then
        if ! python3 scripts/check_topics.py; then
            topics_ok=false
        fi
    else
        # Fallback: manual check
        print_info "Checking critical topics..."
        
        local critical_topics=(
            "/go2_states"
            "/camera/image_raw"
            "/imu"
            "/odom"
        )
        
        local all_ok=true
        for topic in "${critical_topics[@]}"; do
            if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
                print_success "$topic"
            else
                print_warning "$topic (missing)"
                all_ok=false
            fi
        done
        
        if [ "$all_ok" = false ]; then
            print_warning "Some topics are missing"
        fi
    fi
    
    echo ""
    
    # If critical topics are missing, abort
    if [ "$topics_ok" = false ]; then
        print_error "Critical topics are missing - cannot launch mission agent"
        print_info "Please check that:"
        print_info "  1. Robot is powered on and connected"
        print_info "  2. Robot driver launched successfully"
        print_info "  3. Check logs: /tmp/shadowhound_robot_driver.log"
        return 1
    fi
    
    # Topics look good, ask for final confirmation
    read -p "Topics look good? Continue to launch mission agent? [Y/n]: " continue_choice
    if [[ "$continue_choice" = "n" || "$continue_choice" = "N" ]]; then
        print_info "Launch aborted by user"
        return 1
    fi
    
    return 0
}

# ============================================================================
# Launch Mission Agent
# ============================================================================

launch_mission_agent() {
    print_section "Stage 3: Launching Mission Agent"
    
    # Build launch command
    local launch_cmd="ros2 launch shadowhound_mission_agent mission_agent.launch.py"
    
    # Add parameters
    if [ -n "$MOCK_ROBOT" ]; then
        launch_cmd="$launch_cmd mock_robot:=$MOCK_ROBOT"
    fi
    
    if [ -n "$WEB_INTERFACE" ]; then
        launch_cmd="$launch_cmd enable_web_interface:=$WEB_INTERFACE"
    fi
    
    if [ -n "$WEB_PORT" ]; then
        launch_cmd="$launch_cmd web_port:=$WEB_PORT"
    fi
    
    print_info "Launch command:"
    echo "  $launch_cmd"
    echo ""
    
    if [ "${WEB_INTERFACE:-true}" != "false" ]; then
        echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
        echo -e "${GREEN}${WEB} Web Dashboard will be available at: http://localhost:${WEB_PORT:-8080}${NC}"
        echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
        echo ""
    fi
    
    print_success "Starting Mission Agent..."
    echo ""
    
    # Launch!
    $launch_cmd
}

# ============================================================================
# Orchestrated Launch System
# ============================================================================

launch_system() {
    print_section "Orchestrated System Launch"
    
    # Source ROS2 first
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Source workspace
    source install/setup.bash
    
    # Set PYTHONPATH for DIMOS
    export PYTHONPATH="${SCRIPT_DIR}/src/dimos-unitree:${PYTHONPATH}"
    
    # Export CONN_TYPE for DIMOS (defaults to webrtc if not set)
    export CONN_TYPE=${CONN_TYPE:-webrtc}
    
    # Agent-only mode: skip driver and verification
    if [ "$AGENT_ONLY" = true ]; then
        print_info "Agent-only mode: Skipping driver launch and verification"
        echo ""
        launch_mission_agent
        return $?
    fi
    
    echo ""
    print_info "Launch sequence:"
    if [ "$SKIP_DRIVER" = true ] || [ "$MOCK_ROBOT" = "true" ]; then
        print_info "  1. [SKIPPED] Launch robot driver"
        print_info "  2. [SKIPPED] Verify robot topics"
        print_info "  3. Launch mission agent (DIMOS)"
    else
        print_info "  1. Launch robot driver (go2_ros2_sdk)"
        print_info "  2. Verify robot topics are publishing"
        print_info "  3. Launch mission agent (DIMOS)"
    fi
    echo ""
    
    # Stage 1: Launch robot driver (unless skipped)
    if [ "$SKIP_DRIVER" != true ]; then
        if ! launch_robot_driver; then
            print_error "Failed to launch robot driver"
            read -p "Continue anyway? [y/N]: " continue_choice
            if [[ "$continue_choice" != "y" && "$continue_choice" != "Y" ]]; then
                return 1
            fi
        fi
        sleep 2
    else
        print_info "Skipping robot driver launch (--skip-driver flag)"
    fi
    
    # Stage 2: Verify topics (unless skipped or mock mode)
    if [ "$SKIP_DRIVER" != true ] && [ "$MOCK_ROBOT" != "true" ]; then
        if ! verify_robot_topics; then
            print_error "Topic verification failed"
            read -p "Launch mission agent anyway? [y/N]: " continue_choice
            if [[ "$continue_choice" != "y" && "$continue_choice" != "Y" ]]; then
                return 1
            fi
        fi
    fi
    
    # Stage 3: Launch mission agent
    launch_mission_agent
}

# ============================================================================
# Pre-launch Cleanup
# ============================================================================

kill_all_ros_nodes() {
    print_info "Killing any existing ROS nodes for clean start..."
    
    # Kill mission agent processes
    pkill -f "shadowhound_mission_agent" 2>/dev/null || true
    pkill -f "mission_agent.launch" 2>/dev/null || true
    
    # Kill robot driver processes
    pkill -f "go2_driver_node" 2>/dev/null || true
    pkill -f "robot.launch" 2>/dev/null || true
    pkill -f "go2_rviz2" 2>/dev/null || true
    
    # Kill Nav2 nodes
    pkill -f "behavior_server" 2>/dev/null || true
    pkill -f "controller_server" 2>/dev/null || true
    pkill -f "planner_server" 2>/dev/null || true
    pkill -f "bt_navigator" 2>/dev/null || true
    pkill -f "waypoint_follower" 2>/dev/null || true
    pkill -f "velocity_smoother" 2>/dev/null || true
    
    # Kill SLAM and visualization
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "foxglove_bridge" 2>/dev/null || true
    
    # Kill DIMOS-specific nodes
    pkill -f "pointcloud_aggregator" 2>/dev/null || true
    pkill -f "tts_node" 2>/dev/null || true
    
    # Kill generic ROS launch processes
    pkill -f "ros2 launch" 2>/dev/null || true
    
    # Final sweep: kill all nodes in current ROS_DOMAIN_ID
    if command -v ros2 &> /dev/null && [ -n "$ROS_DOMAIN_ID" ]; then
        ros2 node list 2>/dev/null | while read node; do
            pkill -f "$node" 2>/dev/null || true
        done
    fi
    
    # Give processes time to die
    sleep 2
    
    print_success "Existing ROS nodes cleaned up"
}

# ============================================================================
# Cleanup Handler
# ============================================================================

CLEANUP_DONE=false

cleanup() {
    # Prevent re-entry
    if [ "$CLEANUP_DONE" = true ]; then
        return 0
    fi
    CLEANUP_DONE=true
    
    echo ""
    print_section "Shutting Down"
    print_info "Cleaning up..."
    
    # Kill mission agent and all its child processes
    pkill -f "shadowhound_mission_agent" 2>/dev/null || true
    pkill -f "mission_agent.launch" 2>/dev/null || true
    
    # Kill robot driver if we started it
    if [ -f "/tmp/shadowhound_driver.pid" ]; then
        local driver_pid=$(cat /tmp/shadowhound_driver.pid 2>/dev/null)
        if [ -n "$driver_pid" ]; then
            print_info "Stopping robot driver (PID: $driver_pid)..."
            kill $driver_pid 2>/dev/null || true
            sleep 1
            kill -9 $driver_pid 2>/dev/null || true
        fi
        rm -f /tmp/shadowhound_driver.pid
    fi
    
    # Kill any remaining go2/robot processes
    pkill -f "go2_driver_node" 2>/dev/null || true
    pkill -f "robot.launch" 2>/dev/null || true
    pkill -f "go2_rviz2" 2>/dev/null || true
    
    # Kill all Nav2 nodes
    pkill -f "behavior_server" 2>/dev/null || true
    pkill -f "controller_server" 2>/dev/null || true
    pkill -f "planner_server" 2>/dev/null || true
    pkill -f "bt_navigator" 2>/dev/null || true
    pkill -f "waypoint_follower" 2>/dev/null || true
    pkill -f "velocity_smoother" 2>/dev/null || true
    
    # Kill SLAM and other common nodes
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "foxglove_bridge" 2>/dev/null || true
    
    # Kill DIMOS-specific nodes
    pkill -f "pointcloud_aggregator" 2>/dev/null || true
    pkill -f "tts_node" 2>/dev/null || true
    
    # More aggressive: kill any ros2 launch processes
    pkill -f "ros2 launch" 2>/dev/null || true
    
    # Give processes time to die
    sleep 1
    
    # Final aggressive cleanup - kill any remaining ROS nodes from this domain
    if [ -n "$ROS_DOMAIN_ID" ]; then
        print_info "Killing remaining ROS2 nodes in domain $ROS_DOMAIN_ID..."
        # Get all running ROS nodes and kill their processes
        ros2 node list 2>/dev/null | while read node; do
            pkill -f "$node" 2>/dev/null || true
        done
    fi
    
    print_success "Shutdown complete"
    echo ""
    
    # Exit cleanly
    exit 0
}

trap cleanup EXIT INT TERM

# ============================================================================
# Main
# ============================================================================

main() {
    print_header
    
    # Parse command line arguments
    parse_args "$@"
    
    # Clean slate: kill any existing ROS nodes
    print_section "Pre-Launch Cleanup"
    kill_all_ros_nodes
    echo ""
    
    # Run checks and setup
    check_system
    check_git_updates  # NEW: Check for repo/submodule updates
    setup_config
    build_workspace
    check_dependencies
    check_network
    
    # Show summary
    show_summary
    
    # Final confirmation
    echo -e "${YELLOW}Ready to launch!${NC}"
    read -p "Press Enter to start (or Ctrl+C to cancel)..."
    echo ""
    
    # Launch!
    launch_system
}

# Run main function
main "$@"
