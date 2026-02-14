#!/bin/bash

# TODO: allow viewing local sessions
# Configuration
REMOTE_SERVER="orin@10.22.131.69"  # Change this to your server details

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

# TERMINAL_TYPE="gnome-terminal"


get_remote_sessions() {
    # Check SSH connection and get tmux sessions from remote server
    echo "Connecting to $REMOTE_SERVER to fetch tmux sessions..."
    
    # Try to connect and list sessions
    remote_sessions=$(ssh "$REMOTE_SERVER" "tmux list-sessions -F '#{session_name}' 2>/dev/null" 2>/dev/null)
    
    # Check if connection was successful
    if [ $? -ne 0 ]; then
        echo "Error: Could not connect to $REMOTE_SERVER"
        exit 1
    fi
    
    # Check if we got any sessions
    if [ -z "$remote_sessions" ]; then
        echo "No tmux sessions found on $REMOTE_SERVER"
        exit 1
    fi
    
    # Convert to array
    readarray -t SESSIONS <<< "$remote_sessions"
    
    echo "Found ${#SESSIONS[@]} tmux sessions on $REMOTE_SERVER:"
    for session in "${SESSIONS[@]}"; do
        echo "- $session"
    done
}


# # Define tmux sessions to view
# SESSIONS=(
#     "hri-gpsr"
#     "integration-gpsr"
#     "m-gpsr-planning"
#     "m-gpsr-arm"
#     "navigation-receptionist"
#     "vision-gpsr"
# )

create_terminator() {
    # launch terminator window 

    terminator -e "ssh ${REMOTE_SERVER} -t 'tmux attach -t ${SESSIONS[0]} || echo \"Session ${SESSIONS[0]} not found\"'" &

    sleep 1

    cols=4
    rows=$(((${#SESSIONS[@]} + cols - 1) / cols))
    window_id=$(xdotool search --class "terminator" | tail -1)
    for ((i=1; i<$rows; i++)); do 
        xdotool windowactivate "$window_id"
        xdotool key ctrl+shift+o
        sleep 2
        # alt + up key
        # xdotool key alt+"Up"
        # xdotool key alt+Up
        # xdotool key alt+up
        xdotool keydown alt 
        xdotool key Up
        xdotool keyup alt
        xdotool key Return
    done

    xdotool windowactivate "$window_id"
    xdotool keydown super
    xdotool key Up
    xdotool keyup super

    for ((i=1; i<${#SESSIONS[@]}; i++)); do
        session="${SESSIONS[i]}"
        xdotool windowactivate "$window_id"
        if ((i % cols != 0)); then 
            xdotool key ctrl+shift+e
        fi
        xdotool type "ssh ${REMOTE_SERVER} -t 'tmux attach -t $session || echo \"Session $session not found\"'"
        xdotool key Return
        if (((i == cols -1 ))); then
            xdotool windowactivate "$window_id"
            # alt + down key
            xdotool key alt+Down
            xdotool key Return
        fi
    done

}


# Function to open sessions in separate terminals/tabs
open_sessions() {
    case "$TERMINAL_TYPE" in
        "terminator")
            
            echo "Opening terminator with tmux sessions..."
            create_terminator 
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

get_remote_sessions

if [ ${#SESSIONS[@]} -eq 0 ]; then
    echo "No tmux sessions found on $REMOTE_SERVER."
    exit 1
fi


# Main execution
echo "Opening terminals to view tmux sessions..."
open_sessions

# Wait for the terminals to open
sleep 2
echo "Done! You should now see your tmux sessions in separate terminal windows/tabs."