#!/bin/bash
# ============================================================================
# ShadowHound Repository Updater
# ============================================================================
#
# Standalone script to update main repo and submodules from remote.
# Useful for pulling latest changes to go2_ros2_sdk and dimos-unitree.
#
# Usage:
#   ./scripts/update_repos.sh [OPTIONS]
#
# Options:
#   --auto         Automatically update without prompting
#   --fetch-only   Only fetch, don't pull
#   --submodules   Only update submodules
#   --help         Show this help
#
# ============================================================================

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

AUTO_UPDATE=false
FETCH_ONLY=false
SUBMODULES_ONLY=false

# Parse args
while [[ $# -gt 0 ]]; do
    case $1 in
        --auto) AUTO_UPDATE=true; shift ;;
        --fetch-only) FETCH_ONLY=true; shift ;;
        --submodules) SUBMODULES_ONLY=true; shift ;;
        --help|-h)
            grep '^#' "$0" | grep -v '#!/bin/bash' | sed 's/^# //' | sed 's/^#//'
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}  📦 ShadowHound Repository Updater${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

current_branch=$(git rev-parse --abbrev-ref HEAD)
echo -e "${BLUE}ℹ${NC} Current branch: $current_branch"
echo ""

# Fetch from remote
echo -e "${BLUE}ℹ${NC} Fetching updates from remote..."
git fetch origin

if [ "$SUBMODULES_ONLY" = false ]; then
    # Check main repo status
    local_commit=$(git rev-parse HEAD)
    remote_commit=$(git rev-parse origin/$current_branch 2>/dev/null || echo "$local_commit")
    
    if [ "$local_commit" != "$remote_commit" ]; then
        behind=$(git rev-list --count HEAD..origin/$current_branch 2>/dev/null || echo "0")
        ahead=$(git rev-list --count origin/$current_branch..HEAD 2>/dev/null || echo "0")
        
        echo ""
        echo -e "${YELLOW}⚠${NC} Main repo updates available:"
        echo -e "  • $behind commit(s) behind remote"
        echo -e "  • $ahead commit(s) ahead of remote"
        echo ""
        
        if [ "$behind" -gt 0 ]; then
            echo "Latest changes from remote:"
            git log --oneline --graph HEAD..origin/$current_branch | head -5 | sed 's/^/  /'
            echo ""
        fi
        
        if [ "$FETCH_ONLY" = false ]; then
            if [ "$AUTO_UPDATE" = true ] || read -p "Pull main repo updates? [Y/n]: " -n 1 -r && [[ $REPLY =~ ^[Yy]$ || -z $REPLY ]]; then
                echo ""
                git pull origin $current_branch
                echo -e "${GREEN}✓${NC} Main repo updated"
            else
                echo ""
                echo -e "${YELLOW}⚠${NC} Skipped main repo update"
            fi
        fi
    else
        echo -e "${GREEN}✓${NC} Main repo is up to date"
    fi
fi

# Update submodules
echo ""
echo -e "${BLUE}ℹ${NC} Checking submodules..."

if [ ! -f ".gitmodules" ]; then
    echo -e "${YELLOW}⚠${NC} No submodules found"
    exit 0
fi

# Update submodule references
git submodule update --init 2>/dev/null || true

# Check each submodule
git submodule foreach --quiet '
    submodule_name=$(basename "$sm_path")
    echo ""
    echo "─── $submodule_name ───"
    
    # Fetch
    git fetch origin 2>/dev/null || exit 0
    
    local_commit=$(git rev-parse HEAD 2>/dev/null)
    current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
    remote_commit=$(git rev-parse origin/$current_branch 2>/dev/null || echo "$local_commit")
    
    if [ "$local_commit" != "$remote_commit" ]; then
        behind=$(git rev-list --count HEAD..origin/$current_branch 2>/dev/null || echo "0")
        
        if [ "$behind" -gt 0 ]; then
            echo "⚠️  $behind commit(s) behind remote"
            git log --oneline HEAD..origin/$current_branch | head -3 | sed "s/^/    /"
        fi
    else
        echo "✓ Up to date"
    fi
'

if [ "$FETCH_ONLY" = false ]; then
    echo ""
    if [ "$AUTO_UPDATE" = true ] || read -p "Update all submodules? [Y/n]: " -n 1 -r && [[ $REPLY =~ ^[Yy]$ || -z $REPLY ]]; then
        echo ""
        echo ""
        echo -e "${BLUE}ℹ${NC} Updating submodules..."
        git submodule update --remote --merge
        echo -e "${GREEN}✓${NC} Submodules updated"
    else
        echo ""
        echo -e "${YELLOW}⚠${NC} Skipped submodule updates"
    fi
fi

echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}✓ Repository check complete${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if [ "$FETCH_ONLY" = false ]; then
    echo -e "${YELLOW}⚠${NC} Remember to rebuild after pulling changes:"
    echo "  colcon build --symlink-install"
    echo ""
fi
