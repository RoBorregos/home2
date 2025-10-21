# Create pulse audio socket to share with docker container
# If container doesn't start, delete /tmp/pulseaudio.socket and /tmp/pulseaudio.client.conf and run script again

# Ref: https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio

# Add pulseaudio socket on start up (user)
sudo mkdir -p /etc/pulse/default.pa.d
sudo mkdir -p /home/$USER/.config/pulse


sudo sh -c "echo '# RoBorregos@Home | Socket to use pulseaudio in docker container
load-module module-native-protocol-unix socket=/home/$USER/.config/pulse/pulseaudio.socket
' > /etc/pulse/default.pa.d/speech_sound.pa"

echo "Added pulseaudio socket config to /etc/pulse/default.pa.d/speech_sound.pa"

# Restart pulseaudio if socket is not present
if [ ! -S /home/$USER/.config/pulse/pulseaudio.socket ]; then
    echo "Pulseaudio socket not found. Restarting pulseaudio..."
    systemctl --user restart pulseaudio
fi

sudo usermod -aG audio $USER # Make sure current user has access to audio resources.

SCRIPT_PATH="$HOME/scripts/hri_devices.bash"
BASHRC="$HOME/.bashrc"
SCRIPT_CMD="source $SCRIPT_PATH"
SOURCE_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_SCRIPT="$SOURCE_SCRIPT_DIR/mic_script.bash"

# Check if the script already exists, otherwise copy it
if [ ! -f "$SCRIPT_PATH" ]; then
    echo "Copying $SOURCE_SCRIPT to $SCRIPT_PATH"
    mkdir -p "$(dirname "$SCRIPT_PATH")"
    cp "$SOURCE_SCRIPT" "$SCRIPT_PATH"
    chmod u+x "$SCRIPT_PATH"
fi

# Source the script in .bashrc if it isn't sourced already
grep -qxF "$SCRIPT_CMD" "$BASHRC" || echo "$SCRIPT_CMD" >> "$BASHRC"

bash -i $BASHRC

# If the usb rules file doesn't exist, create it and reload rules
if [ ! -f /etc/udev/rules.d/99-usb.rules ]; then
    # Access respeaker usb device
    echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2886\", ATTR{idProduct}==\"0018\", MODE=\"0666\"" | sudo tee -a /etc/udev/rules.d/99-usb.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
fi


echo "Finished hri setup configuration for docker."
