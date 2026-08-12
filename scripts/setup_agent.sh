#!/bin/bash

# Default instructions content
DEFAULT_CONTENT="You are an expert developer on this project (FRIDA - Friendly Robotics Interactive Domestic Assistant, ROS 2 service robot).

## Global Context — Always Read
Before writing code, read these files:
- docs/ai/architecture.md       — system areas, Docker execution model, run.sh usage
- docs/ai/tech_stack.md         — ROS 2, Python/C++, DDS, Docker, CUDA/L4T platforms
- docs/ai/coding_standards.md   — ROS 2 node patterns, frida_interfaces usage, Ruff, type hints

## Area-Specific Context — Read When Working on an Area
When working on a specific area, also read its doc:
- Vision:        docs/Run/Areas/vision.md
- Manipulation:  docs/Run/Areas/Manipulation/manipulation.md
  Pick & Place:  docs/Run/Areas/Manipulation/pick_and_place.md"

function show_usage() {
    echo "Usage: $0 [agent_type]"
    echo "Agent types: claude, gemini, copilot, custom"
}

function create_file() {
    local path=$1
    local dir=$(dirname "$path")
    
    if [ "$dir" != "." ]; then
        mkdir -p "$dir"
    fi

    echo "$DEFAULT_CONTENT" > "$path"
    echo "Successfully created agent instructions at: $path"
}

AGENT_TYPE=$1

if [ -z "$AGENT_TYPE" ]; then
    echo "Select the agent you want to set up instructions for:"
    echo "1) Claude (CLAUDE.md)"
    echo "2) Gemini (GEMINI.md)"
    echo "3) Copilot (.github/copilot-instructions.md)"
    echo "4) Custom file"
    echo "5) Exit"
    read -p "Enter choice [1-5]: " choice

    case $choice in
        1) AGENT_TYPE="claude" ;;
        2) AGENT_TYPE="gemini" ;;
        3) AGENT_TYPE="copilot" ;;
        4) AGENT_TYPE="custom" ;;
        5) exit 0 ;;
        *) echo "Invalid choice"; exit 1 ;;
    esac
fi

case $AGENT_TYPE in
    claude)
        create_file "CLAUDE.md"
        ;;
    gemini)
        create_file "GEMINI.md"
        ;;
    copilot)
        create_file ".github/copilot-instructions.md"
        ;;
    custom)
        read -p "Enter custom file path: " CUSTOM_PATH
        if [ -z "$CUSTOM_PATH" ]; then
            echo "Path cannot be empty."
            exit 1
        fi
        create_file "$CUSTOM_PATH"
        ;;
    *)
        show_usage
        exit 1
        ;;
esac
