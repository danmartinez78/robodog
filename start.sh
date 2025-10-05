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
#   --help         Show this help message
#
# Examples:
#   ./start.sh                    # Interactive mode
#   ./start.sh --dev              # Development mode
#   ./start.sh --prod --no-web    # Production without web UI
#   ./start.sh --mock             # Mock robot mode
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

# Default options
MOCK_ROBOT=""
WEB_INTERFACE=""
WEB_PORT=""
CONFIG_MODE=""

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
            echo "  - GO2_IP=192.168.1.103 (your robot's IP)"
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
        if [ -z "$GO2_IP" ]; then
            print_warning "GO2_IP not set, using default: 192.168.1.103"
        else
            print_success "Robot IP: $GO2_IP"
        fi
    fi
}

# ============================================================================
# Build Workspace
# ============================================================================

build_workspace() {
    print_section "Building Workspace"
    
    # Check if already built
    if [ -d "install" ] && [ -f "install/setup.bash" ]; then
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
    
    print_info "Building ShadowHound packages..."
    
    # Build only our packages (skip DIMOS perception models with CUDA issues)
    if colcon build --packages-select shadowhound_mission_agent shadowhound_bringup --symlink-install 2>&1 | tee /tmp/colcon_build.log; then
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
    python3 -c "import anthropic" 2>/dev/null || missing+=("anthropic (DIMOS)")
    python3 -c "import zmq" 2>/dev/null || missing+=("pyzmq (DIMOS)")
    
    if [ ${#missing[@]} -gt 0 ]; then
        print_warning "Missing Python packages: ${missing[*]}"
        read -p "Install missing packages? [Y/n]: " install_choice
        
        if [[ "$install_choice" != "n" && "$install_choice" != "N" ]]; then
            print_info "Installing Python packages..."
            
            # Install ShadowHound dependencies
            pip3 install -q fastapi uvicorn[standard] websockets pydantic openai
            
            # Install DIMOS core dependencies (skip heavy perception/CUDA stuff)
            print_info "Installing DIMOS core dependencies..."
            pip3 install -q reactivex python-dotenv anthropic colorlog typeguard \
                empy catkin_pkg lark tiktoken Flask python-multipart pytest-asyncio \
                fastapi sse-starlette uvicorn langchain-chroma langchain-openai pydantic \
                pyzmq numpy
            
            print_success "Packages installed"
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
    
    local go2_ip=${GO2_IP:-192.168.1.103}
    
    print_info "Checking connection to robot at $go2_ip..."
    
    if ping -c 1 -W 2 "$go2_ip" &> /dev/null; then
        print_success "Robot is reachable at $go2_ip"
    else
        print_warning "Cannot reach robot at $go2_ip"
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
    
    echo ""
    echo "Configuration:"
    echo "  • Mode: ${CONFIG_MODE:-default}"
    echo "  • Mock Robot: ${MOCK_ROBOT:-false}"
    echo "  • Web Interface: ${WEB_INTERFACE:-true}"
    echo "  • Web Port: ${WEB_PORT:-8080}"
    echo "  • ROS Domain: ${ROS_DOMAIN_ID:-0}"
    echo "  • OpenAI Model: ${OPENAI_MODEL:-gpt-4o}"
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
# Launch System
# ============================================================================

launch_system() {
    print_section "Launching ShadowHound"
    
    # Source ROS2 first
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Source workspace
    source install/setup.bash
    
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
    
    print_success "Starting ShadowHound..."
    echo ""
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # Launch!
    $launch_cmd
}

# ============================================================================
# Cleanup Handler
# ============================================================================

cleanup() {
    echo ""
    print_section "Shutting Down"
    print_info "Cleaning up..."
    
    # Kill any remaining processes
    pkill -f "shadowhound_mission_agent" 2>/dev/null || true
    
    print_success "Shutdown complete"
    echo ""
}

trap cleanup EXIT INT TERM

# ============================================================================
# Main
# ============================================================================

main() {
    print_header
    
    # Parse command line arguments
    parse_args "$@"
    
    # Run checks and setup
    check_system
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
