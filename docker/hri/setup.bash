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

# Restart pulseaudio
echo "Restarting pulseaudio for user."

systemctl --user restart pulseaudio

sudo usermod -aG audio $USER # Make sure current user has access to audio resources.

echo "Finished hri setup configuration for docker."
