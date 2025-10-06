#!/bin/bash
# Helper script to add a devlog entry

set -e

DEVLOG_FILE="DEVLOG.md"
TODO_FILE="TODO.md"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}ShadowHound DevLog Entry Helper${NC}\n"

# Get today's date
TODAY=$(date +"%Y-%m-%d")

# Check if files exist
if [ ! -f "$DEVLOG_FILE" ]; then
    echo -e "${YELLOW}Warning: $DEVLOG_FILE not found!${NC}"
    exit 1
fi

# Prompt for entry details
echo -e "${GREEN}Creating entry for $TODAY${NC}\n"

read -p "Brief title (e.g., 'Added Vision System'): " TITLE
echo ""

read -p "What was done? (brief description): " WHAT_DONE
echo ""

read -p "Technical details (press Enter to skip): " TECH_DETAILS
echo ""

read -p "Validation/testing (press Enter to skip): " VALIDATION
echo ""

read -p "Key learnings (press Enter to skip): " LEARNINGS
echo ""

read -p "Next steps (press Enter to skip): " NEXT_STEPS
echo ""

# Build the entry
ENTRY="## $TODAY - $TITLE\n\n"
ENTRY+="### What Was Done\n$WHAT_DONE\n\n"

if [ -n "$TECH_DETAILS" ]; then
    ENTRY+="### Technical Details\n$TECH_DETAILS\n\n"
fi

if [ -n "$VALIDATION" ]; then
    ENTRY+="### Validation\n$VALIDATION\n\n"
fi

if [ -n "$LEARNINGS" ]; then
    ENTRY+="### Key Learnings\n$LEARNINGS\n\n"
fi

if [ -n "$NEXT_STEPS" ]; then
    ENTRY+="### Next Steps\n$NEXT_STEPS\n\n"
fi

ENTRY+="---\n\n"

# Insert after the header (line 5)
# Create a temp file with the new entry
{
    head -n 5 "$DEVLOG_FILE"
    echo -e "$ENTRY"
    tail -n +6 "$DEVLOG_FILE"
} > "${DEVLOG_FILE}.tmp"

mv "${DEVLOG_FILE}.tmp" "$DEVLOG_FILE"

echo -e "\n${GREEN}✓ Entry added to $DEVLOG_FILE${NC}\n"

# Ask if they want to commit
read -p "Commit changes? (y/n): " COMMIT_CHOICE
if [ "$COMMIT_CHOICE" = "y" ] || [ "$COMMIT_CHOICE" = "Y" ]; then
    git add "$DEVLOG_FILE"
    
    # Check if TODO.md has changes
    if git diff --cached --quiet "$TODO_FILE" 2>/dev/null || ! git ls-files --error-unmatch "$TODO_FILE" >/dev/null 2>&1; then
        echo -e "${YELLOW}Note: $TODO_FILE has no staged changes${NC}"
    else
        git add "$TODO_FILE"
    fi
    
    COMMIT_MSG="docs: DevLog entry for $TODAY - $TITLE"
    git commit -m "$COMMIT_MSG"
    echo -e "${GREEN}✓ Changes committed${NC}"
else
    echo -e "${YELLOW}Changes not committed. Remember to commit manually!${NC}"
fi

echo -e "\n${GREEN}Done!${NC}"
