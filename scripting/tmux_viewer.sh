#!/bin/bash

# Configuration
REMOTE_SERVER="orin@192.168.31.10"  # Change this to your server details

# Terminal emulator detection
if command -v terminator &> /dev/null; then
    TERMINAL_CMD="terminator"
    TERMINAL_TYPE="terminator"
elif command -v gnome-terminal &> /dev/null; then
    TERMINAL_CMD="gnome-terminal"
    TERMINAL_TYPE="gnome-terminal"
else
    echo "Neither terminator nor gnome-terminal found. Please install one of them."
    exit 1
fi

# Define tmux sessions to view
SESSIONS=(
    "hri-gpsr"
    "integration-gpsr"
    "m-gpsr-planning"
    "m-gpsr-arm"
    "navigation-receptionist"
    "vision-gpsr"
)

# Create a layout configuration for terminator
create_terminator_config() {
    local config_file="/tmp/terminator_tmux_layout.config"
    
    echo "[global_config]" > "$config_file"
    echo "[layouts]" >> "$config_file"
    echo "  [[default]]" >> "$config_file"
    echo "    [[[window0]]]" >> "$config_file"
    echo "      type = Window" >> "$config_file"
    echo "      parent = \"\"" >> "$config_file"
    
    # Create a grid layout
    local columns=2
    local rows=$(( (${#SESSIONS[@]} + columns - 1) / columns ))
    local width=$((100 / columns))
    local height=$((100 / rows))
    
    # Add root terminal
    echo "    [[[terminal0]]]" >> "$config_file"
    echo "      type = Terminal" >> "$config_file"
    echo "      parent = window0" >> "$config_file"
    echo "      profile = default" >> "$config_file"
    echo "      command = ssh ${REMOTE_SERVER} -t \"tmux attach -t ${SESSIONS[0]} || echo 'Session ${SESSIONS[0]} not found'\"" >> "$config_file"
    
    local current=0
    for ((i=1; i<${#SESSIONS[@]}; i++)); do
        local session="${SESSIONS[i]}"
        
        # Split horizontally or vertically based on position
        if ((i % columns == 0)); then
            # New row - split the terminal at position current-columns+1 horizontally
            local split_term=$((current-columns+1))
            echo "    [[[terminal$i]]]" >> "$config_file"
            echo "      type = Terminal" >> "$config_file"
            echo "      parent = window0" >> "$config_file"
            echo "      position = $i" >> "$config_file"
            echo "      command = ssh ${REMOTE_SERVER} -t \"tmux attach -t $session || echo 'Session $session not found'\"" >> "$config_file"
        else
            # Same row - split vertically
            echo "    [[[terminal$i]]]" >> "$config_file"
            echo "      type = Terminal" >> "$config_file"
            echo "      parent = window0" >> "$config_file"
            echo "      position = $i" >> "$config_file"
            echo "      command = ssh ${REMOTE_SERVER} -t \"tmux attach -t $session || echo 'Session $session not found'\"" >> "$config_file"
        fi
        current=$i
    done
    
    echo "[plugins]" >> "$config_file"
    echo "[profiles]" >> "$config_file"
    
    echo "$config_file"
}

# Function to open sessions in separate terminals/tabs
open_sessions() {
    case "$TERMINAL_TYPE" in
        "terminator")
            # Create config and launch terminator with it
            config_file=$(create_terminator_config)
            terminator --config="$config_file" --layout=default &
            ;;
            
        "gnome-terminal")
            # Open first tab
            gnome-terminal -- bash -c "ssh ${REMOTE_SERVER} -t 'tmux attach -t ${SESSIONS[0]} || echo \"Session ${SESSIONS[0]} not found\"'; exec bash" &
            
            # Give time for the first window to open
            sleep 1
            
            # Get the ID of the most recently opened window
            window_id=$(xdotool search --class "gnome-terminal" | tail -1)
            
            # Open additional tabs in the same window
            for ((i=1; i<${#SESSIONS[@]}; i++)); do
                session="${SESSIONS[i]}"
                xdotool windowactivate "$window_id"
                xdotool key ctrl+shift+t
                xdotool type "ssh ${REMOTE_SERVER} -t 'tmux attach -t $session || echo \"Session $session not found\"'"
                xdotool key Return
            done
            ;;
    esac
}

# Main execution
echo "Opening terminals to view tmux sessions..."
open_sessions

# Wait for the terminals to open
sleep 2
echo "Done! You should now see your tmux sessions in separate terminal windows/tabs."