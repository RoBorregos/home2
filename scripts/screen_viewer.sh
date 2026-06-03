#!/bin/bash

SESSION_NAME="frida"
# These must match the -t flags in lib.sh
WINDOWS=("vision" "manipulation" "navigation" "integration" "hri" "zed" "display")

# Check if session exists
if ! screen -ls | grep -q "\.${SESSION_NAME}"; then
    echo "Error: Screen session '$SESSION_NAME' not found."
    echo "Run a task first (e.g., ./run.sh --gpsr)"
    exit 1
fi

# Function to send split commands
apply_layout() {
    # Wait for the user to be attached
    sleep 1.2

    echo "Applying 3x2 split layout..."

    # Clear any existing splits first (only for this attachment)
    screen -S "$SESSION_NAME" -X only

    # Create 3 horizontal stripes
    screen -S "$SESSION_NAME" -X split
    screen -S "$SESSION_NAME" -X split

    # --- Top Row ---
    screen -S "$SESSION_NAME" -X focus top
    screen -S "$SESSION_NAME" -X select "${WINDOWS[0]}"
    screen -S "$SESSION_NAME" -X split -v
    screen -S "$SESSION_NAME" -X focus next
    screen -S "$SESSION_NAME" -X select "${WINDOWS[3]}"

    # --- Middle Row ---
    screen -S "$SESSION_NAME" -X focus top
    screen -S "$SESSION_NAME" -X focus down
    screen -S "$SESSION_NAME" -X select "${WINDOWS[1]}"
    screen -S "$SESSION_NAME" -X split -v
    screen -S "$SESSION_NAME" -X focus next
    screen -S "$SESSION_NAME" -X select "${WINDOWS[4]}"

    # --- Bottom Row ---
    screen -S "$SESSION_NAME" -X focus top
    screen -S "$SESSION_NAME" -X focus down
    screen -S "$SESSION_NAME" -X focus down
    screen -S "$SESSION_NAME" -X select "${WINDOWS[2]}"
    screen -S "$SESSION_NAME" -X split -v
    screen -S "$SESSION_NAME" -X focus next
    screen -S "$SESSION_NAME" -X select "${WINDOWS[5]}"

    # Back to Top Left
    screen -S "$SESSION_NAME" -X focus top

    # Add a status bar at the bottom
    screen -S "$SESSION_NAME" -X caption always "%{= kw}%-w%{= BY}%n %t%{-}%+w"

    echo "Layout applied! 3x2 grid active."
    echo "Windows: [0|3] [1|4] [2|5]"
}

# Run the layout applicator in the background
apply_layout &

echo "Attaching to session '$SESSION_NAME'..."
# Attach to the session. -x allows multiple attachments.
screen -x "$SESSION_NAME"